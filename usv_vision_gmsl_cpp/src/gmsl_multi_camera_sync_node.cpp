/**
 * @file gmsl_multi_camera_sync_node.cpp
 * @brief 多相机硬件同步节点 - 一个进程控制多个GMSL相机，实现微秒级硬件触发同步
 *
 * 设计说明：
 * - 单进程初始化多个相机上下文，确保硬件同步触发生效
 * - 通过配置文件动态指定相机列表（支持2-8路）
 * - 每个相机发布到独立 topic，支持硬件时间戳
 * - 与 gmsl_camera_node_cpp 的区别：后者是单相机节点，可多实例但异步
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <std_msgs/msg/header.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

// Miivii SDK头文件
#include "MvGmslCamera.h"

// 修复SDK头文件宏冲突（必须放在include之后）
#ifdef filename
#undef filename
#endif

#include <memory>
#include <vector>
#include <string>
#include <thread>
#include <atomic>

#include "usv_vision_gmsl_cpp/camera_image_publishers.hpp"
#include "usv_vision_gmsl_cpp/camera_info_loader.hpp"

struct CameraInfo {
    std::string device_path;
    std::string sensor_name;
    std::string topic_name;         // image_raw 话题
    std::string info_topic_name;    // camera_info 话题（由 image 话题推导）
    uint8_t camera_no;
    std::shared_ptr<usv_vision_gmsl_cpp::CameraInfoLoader> info_loader;
};

class GmslMultiCameraSyncNode : public rclcpp::Node
{
public:
    GmslMultiCameraSyncNode() : Node("gmsl_multi_camera_sync_node")
    {
        RCLCPP_INFO(this->get_logger(), "=== GMSL Multi-Camera Hardware Sync Node ===");

        // 声明通用参数
        this->declare_parameter<int>("width", 1920);
        this->declare_parameter<int>("height", 1536);
        this->declare_parameter<double>("fps", 30.0);
        this->declare_parameter<std::string>("pixel_format", "UYVY");
        this->declare_parameter<bool>("enable_hardware_timestamp", true);
        this->declare_parameter<std::string>("image_qos_reliability", "best_effort");
        // 与单相机节点对齐：大帧场景下 depth10 只是白占缓冲，实测 depth2 送达帧数/延迟几乎无差
        this->declare_parameter<int>("image_qos_depth", 2);
        // 按需分发（与单相机节点一致）：image_raw 全分辨率 / image_algo 算法尺寸（识别 + 观察共用）
        this->declare_parameter<bool>("enable_algo_stream", true);
        this->declare_parameter<int>("algo_width", 640);
        this->declare_parameter<int>("algo_height", 640);

        width_ = this->get_parameter("width").as_int();
        height_ = this->get_parameter("height").as_int();
        fps_ = this->get_parameter("fps").as_double();
        pixel_format_ = this->get_parameter("pixel_format").as_string();
        enable_hw_timestamp_ = this->get_parameter("enable_hardware_timestamp").as_bool();
        image_qos_reliability_ = this->get_parameter("image_qos_reliability").as_string();
        image_qos_depth_ = this->get_parameter("image_qos_depth").as_int();
        enable_algo_stream_ = this->get_parameter("enable_algo_stream").as_bool();
        algo_width_ = this->get_parameter("algo_width").as_int();
        algo_height_ = this->get_parameter("algo_height").as_int();

        // 加载相机列表配置
        load_camera_configs();

        if (cameras_.empty()) {
            RCLCPP_ERROR(this->get_logger(), "No cameras configured! Check config file.");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Loaded %zu camera(s) from config", cameras_.size());

        // 初始化 GMSL SDK
        init_gmsl_sdk();

        // 为每个相机创建"按需分发"发布器（image_raw + camera_info + image_algo）
        for (const auto& cam : cameras_) {
            usv_vision_gmsl_cpp::CameraImagePublisherOptions opts;
            opts.frame_id = cam.sensor_name;
            opts.raw_image_topic = cam.topic_name;
            opts.raw_info_topic = cam.info_topic_name;
            opts.algo_image_topic = derive_sibling_topic(cam.topic_name, "image_algo");
            opts.algo_info_topic = derive_sibling_topic(cam.topic_name, "camera_info_algo");
            opts.raw_reliability = image_qos_reliability_;
            opts.raw_depth = image_qos_depth_;
            opts.enable_algo = enable_algo_stream_;
            opts.algo_width = algo_width_;
            opts.algo_height = algo_height_;

            publishers_.push_back(
                std::make_unique<usv_vision_gmsl_cpp::CameraImagePublishers>(
                    this, cam.info_loader, opts));
            RCLCPP_INFO(this->get_logger(),
                "Created publishers: %s (+%s, algo=%s %dx%d, %s/depth%d) (device: %s)",
                cam.topic_name.c_str(), cam.info_topic_name.c_str(),
                enable_algo_stream_ ? opts.algo_image_topic.c_str() : "disabled",
                algo_width_, algo_height_,
                image_qos_reliability_.c_str(), image_qos_depth_,
                cam.device_path.c_str());
        }

        // 启动采集线程
        running_ = true;
        capture_thread_ = std::thread(&GmslMultiCameraSyncNode::capture_loop, this);

        RCLCPP_INFO(this->get_logger(), "Multi-camera sync node started with %zu cameras",
            cameras_.size());
    }

    ~GmslMultiCameraSyncNode()
    {
        RCLCPP_INFO(this->get_logger(), "Shutting down multi-camera sync node...");
        running_ = false;
        if (capture_thread_.joinable()) {
            capture_thread_.join();
        }
        gmsl_camera_.reset();
        RCLCPP_INFO(this->get_logger(), "Multi-camera sync node shutdown complete");
    }

private:
    void load_camera_configs()
    {
        // 声明相机列表参数（YAML 数组）
        this->declare_parameter<std::vector<std::string>>("cameras.devices", std::vector<std::string>{});
        this->declare_parameter<std::vector<std::string>>("cameras.sensor_names", std::vector<std::string>{});
        this->declare_parameter<std::vector<std::string>>("cameras.topics", std::vector<std::string>{});
        // 相机标定（全部可选）：与 devices 一一对应；缺省 / 数组更短 / 留空 均按空标定处理
        this->declare_parameter<std::vector<std::string>>("cameras.camera_info_urls", std::vector<std::string>{});
        this->declare_parameter<std::vector<double>>("cameras.fov_x_degs", std::vector<double>{});
        this->declare_parameter<std::vector<double>>("cameras.fov_y_degs", std::vector<double>{});

        auto devices = this->get_parameter("cameras.devices").as_string_array();
        auto sensor_names = this->get_parameter("cameras.sensor_names").as_string_array();
        auto topics = this->get_parameter("cameras.topics").as_string_array();
        auto info_urls = this->get_parameter("cameras.camera_info_urls").as_string_array();
        auto fov_xs = this->get_parameter("cameras.fov_x_degs").as_double_array();
        auto fov_ys = this->get_parameter("cameras.fov_y_degs").as_double_array();

        if (devices.size() != sensor_names.size() || devices.size() != topics.size()) {
            RCLCPP_ERROR(this->get_logger(),
                "Camera config arrays size mismatch: devices=%zu, names=%zu, topics=%zu",
                devices.size(), sensor_names.size(), topics.size());
            return;
        }

        for (size_t i = 0; i < devices.size(); ++i) {
            CameraInfo info;
            info.device_path = devices[i];
            info.sensor_name = sensor_names[i];
            info.topic_name = topics[i];
            info.info_topic_name = derive_info_topic(topics[i]);

            // 从 device_path 解析 camera_no
            sscanf(info.device_path.c_str(), "/dev/video%hhu", &info.camera_no);

            // 标定项可选：数组未提供或更短时按空值处理（内参缺失不影响出图）
            std::string url = (i < info_urls.size()) ? info_urls[i] : "";
            double fov_x = (i < fov_xs.size()) ? fov_xs[i] : 0.0;
            double fov_y = (i < fov_ys.size()) ? fov_ys[i] : 0.0;
            info.info_loader = std::make_shared<usv_vision_gmsl_cpp::CameraInfoLoader>(
                this->get_logger(), info.sensor_name, url, fov_x, fov_y);

            cameras_.push_back(info);
        }
    }

    // 由 image 话题推导同族话题：末段 image_raw 替换为 suffix，
    // 保持在用户配置的同一命名空间下；不含 image_raw 时退回追加 /suffix
    static std::string derive_sibling_topic(const std::string& image_topic, const std::string& suffix)
    {
        const std::string tail = "image_raw";
        if (image_topic.size() >= tail.size() &&
            image_topic.compare(image_topic.size() - tail.size(), tail.size(), tail) == 0) {
            return image_topic.substr(0, image_topic.size() - tail.size()) + suffix;
        }
        std::string base = image_topic;
        if (!base.empty() && base.back() == '/') base.pop_back();
        return base + "/" + suffix;
    }

    static std::string derive_info_topic(const std::string& image_topic)
    {
        return derive_sibling_topic(image_topic, "camera_info");
    }

    void init_gmsl_sdk()
    {
        try {
            size_t num_cameras = cameras_.size();

            // 创建8个相机上下文（SDK 要求固定8个元素）
            camera_context_t contexts[8] = {};

            for (size_t i = 0; i < num_cameras; ++i) {
                contexts[cameras_[i].camera_no].dev_node = cameras_[i].device_path;
                contexts[cameras_[i].camera_no].camera_fmt_str = pixel_format_;
                contexts[cameras_[i].camera_no].output_fmt_str = "BGRA32";
                contexts[cameras_[i].camera_no].cam_w = width_;
                contexts[cameras_[i].camera_no].cam_h = height_;
                contexts[cameras_[i].camera_no].out_w = width_;
                contexts[cameras_[i].camera_no].out_h = height_;
            }

            // 配置硬件同步触发
            // sync_camera_num/bit_draw 固定为 8 路全触发（整板级同步，与单相机节点一致，
            // 详见 gmsl_camera_node_cpp.cpp 的说明）；bit_mask 才按实际接入相机计算。
            sync_out_a_cfg_client_t cfg = {};
            cfg.sync_camera_num = 8;  // SDK 要求固定8
            cfg.sync_freq = static_cast<uint8_t>(fps_);
            cfg.async_camera_num = 0;
            cfg.async_freq = 0;
            cfg.sync_camera_bit_draw = 0xff;

            // 计算 bit_mask（只启用实际使用的相机）
            uint8_t bit_mask = 0;
            for (const auto& cam : cameras_) {
                bit_mask |= (1 << cam.camera_no);
            }

            cfg.sync_camera_bit_mask = bit_mask;
            cfg.async_camera_bit_draw = 0;

            for (int i = 0; i < 8; i++) {
                cfg.async_camera_pos[i] = 0;
            }

            RCLCPP_INFO(this->get_logger(),
                "Initializing GMSL SDK: num_cameras=%zu, sync_freq=%d, bit_draw=0x%02x, bit_mask=0x%02x",
                num_cameras, cfg.sync_freq, cfg.sync_camera_bit_draw, cfg.sync_camera_bit_mask);

            // 初始化 SDK（传递固定8个contexts，但只启用 num_cameras 个）
            gmsl_camera_ = std::make_shared<miivii::MvGmslCamera>(
                contexts, num_cameras, cfg);

            RCLCPP_INFO(this->get_logger(),
                "GMSL SDK initialized successfully with %zu cameras in hardware sync mode",
                num_cameras);

        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(),
                "Failed to initialize GMSL SDK: %s", e.what());
            throw;
        }
    }

    void capture_loop()
    {
        RCLCPP_INFO(this->get_logger(), "Capture loop started for %zu cameras",
            cameras_.size());

        size_t num_cameras = cameras_.size();

        // 为所有相机预分配缓存（SDK 批量接口）
        std::vector<cv::Mat> out_mats(num_cameras);
        uint64_t timestamp = 0;

        while (running_ && rclcpp::ok()) {
            try {
                // 关键：一次调用获取所有同步相机的帧（SDK 批量接口）
                uint8_t first_camera_no = cameras_[0].camera_no;
                bool res = gmsl_camera_->GetImageCvMat(
                    out_mats.data(), timestamp, first_camera_no, "NONE");

                if (!res || timestamp == 0) {
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                        "Failed to get synchronized frames from SDK");
                    continue;
                }

                // 处理和发布图像（所有相机使用相同的时间戳）
                for (size_t i = 0; i < num_cameras; ++i) {
                    if (out_mats[i].empty()) {
                        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                            "Camera %zu frame is empty", i);
                        continue;
                    }

                    // 转换为 BGR
                    cv::Mat bgr_frame;
                    if (out_mats[i].channels() == 4) {
                        cv::cvtColor(out_mats[i], bgr_frame, cv::COLOR_BGRA2BGR);
                    } else if (out_mats[i].channels() == 2) {
                        cv::cvtColor(out_mats[i], bgr_frame, cv::COLOR_YUV2BGR_UYVY);
                    } else {
                        bgr_frame = out_mats[i];
                    }

                    // 转换硬件时间戳为 ROS 时间（所有相机使用相同的时间戳）
                    rclcpp::Time ros_stamp(timestamp);

                    // 一次成帧、按需分发：全分辨率→image_raw，算法尺寸→image_algo（识别 + 观察共用）
                    publishers_[i]->publish(bgr_frame, &out_mats[i], ros_stamp);
                }

            } catch (const std::exception& e) {
                RCLCPP_WARN(this->get_logger(), "Capture error: %s", e.what());
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        }

        RCLCPP_INFO(this->get_logger(), "Capture loop ended");
    }

    // 参数
    int width_;
    int height_;
    double fps_;
    std::string pixel_format_;
    bool enable_hw_timestamp_;
    std::string image_qos_reliability_;
    int image_qos_depth_;
    bool enable_algo_stream_;
    int algo_width_;
    int algo_height_;

    // 相机配置
    std::vector<CameraInfo> cameras_;

    // GMSL SDK
    std::shared_ptr<miivii::MvGmslCamera> gmsl_camera_;

    // 发布器（每个相机一套按需分发：image_raw / camera_info / image_algo）
    std::vector<std::unique_ptr<usv_vision_gmsl_cpp::CameraImagePublishers>> publishers_;

    // 采集线程
    std::thread capture_thread_;
    std::atomic<bool> running_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    try {
        auto node = std::make_shared<GmslMultiCameraSyncNode>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("gmsl_multi_sync"),
            "Fatal error: %s", e.what());
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}

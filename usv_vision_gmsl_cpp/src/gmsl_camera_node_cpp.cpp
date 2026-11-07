/**
 * GMSL相机ROS 2节点（C++实现）
 *
 * 功能：
 * 1. 通过OpenCV VideoCapture获取图像
 * 2. 通过Miivii SDK获取硬件时间戳
 * 3. 发布统一的ROS消息（与Python节点兼容）
 *
 * 编译依赖：
 * - ROS 2 Humble/Foxy
 * - OpenCV 4
 * - Miivii GMSL SDK (/opt/miivii)
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <std_msgs/msg/string.hpp>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>

// Miivii SDK头文件
#include "MvGmslCamera.h"

// 修复SDK头文件宏冲突（必须放在include之后）
#ifdef filename
#undef filename
#endif

#include <thread>
#include <atomic>
#include <chrono>
#include <sstream>
#include <memory>
#include <nlohmann/json.hpp>  // 需要安装: sudo apt install nlohmann-json3-dev

#include "usv_vision_gmsl_cpp/camera_image_publishers.hpp"
#include "usv_vision_gmsl_cpp/h264_streamer.hpp"
#include "usv_vision_gmsl_cpp/camera_info_loader.hpp"

using json = nlohmann::json;

class GmslCameraNode : public rclcpp::Node
{
public:
    GmslCameraNode() : Node("gmsl_camera_node")
    {
        // 声明参数（与Python版本保持一致）
        this->declare_parameter("sensor_name", "front");
        this->declare_parameter("camera_name", "front");
        this->declare_parameter("device_path", "/dev/video0");
        this->declare_parameter("frame_id", "front_camera_link");
        this->declare_parameter("camera_info_url", "");
        this->declare_parameter("fov_x_deg", 0.0);  // 规格书标称水平FOV(度)，标定缺失时兜底反算内参
        this->declare_parameter("fov_y_deg", 0.0);  // 规格书标称垂直FOV(度)
        this->declare_parameter("width", 1920);
        this->declare_parameter("height", 1536);
        this->declare_parameter("fps", 30.0);
        this->declare_parameter("pixel_format", "UYVY");
        this->declare_parameter("source_type", "v4l2_gmsl");
        this->declare_parameter("gstreamer_pipeline", "");
        this->declare_parameter("fallback_v4l2", true);
        this->declare_parameter("enable_hardware_timestamp", true);
        this->declare_parameter("auto_detect_resolution", false);  // 自动检测分辨率
        this->declare_parameter("put_timestamp", false);
        this->declare_parameter("sync_camera_bit_mask", 0);  // 多相机硬件同步掩码：0=自动(1<<camera_no), 非0=手动设置
        this->declare_parameter("image_qos_reliability", "best_effort");
        // depth 默认 2：8.4MB 大帧下 reliable/depth10 会白占 ~84MB/相机，实测 depth2 与 depth10
        // 在送达帧数/延迟上几乎无差（56 vs 53 帧、23.3 vs 24.1ms），故收敛到 2。
        this->declare_parameter("image_qos_depth", 2);
        // ── 算法链路：采集端一次成帧、按需分发（见 camera_image_publishers.hpp 说明）──
        this->declare_parameter("enable_algo_stream", true);   // 发布 image_algo（识别 + rviz 观察共用）
        this->declare_parameter("algo_width", 640);            // 算法图宽度；<=0 或与原始同尺寸则不缩放
        this->declare_parameter("algo_height", 640);           // 算法图高度

        // 获取参数
        sensor_name_ = this->get_parameter("sensor_name").as_string();
        device_path_ = this->get_parameter("device_path").as_string();
        frame_id_ = this->get_parameter("frame_id").as_string();
        camera_info_url_ = this->get_parameter("camera_info_url").as_string();
        width_ = this->get_parameter("width").as_int();
        height_ = this->get_parameter("height").as_int();
        fps_ = this->get_parameter("fps").as_double();
        pixel_format_ = this->get_parameter("pixel_format").as_string();
        gstreamer_pipeline_ = this->get_parameter("gstreamer_pipeline").as_string();
        enable_hw_timestamp_ = this->get_parameter("enable_hardware_timestamp").as_bool();
        auto_detect_resolution_ = this->get_parameter("auto_detect_resolution").as_bool();

        // 从device_path解析camera_no (例如 /dev/video0 -> 0)
        camera_no_ = parse_camera_no(device_path_);

        // 自动检测设备能力
        if (auto_detect_resolution_) {
            detect_device_capabilities();
        }

        // 创建发布者（统一话题命名）
        const std::string camera_base = "/sensors/camera/" + sensor_name_;
        std::string image_topic = camera_base + "/image_raw";
        std::string info_topic = camera_base + "/camera_info";
        std::string timing_topic = camera_base + "/frame_timing_json";
        std::string status_topic = camera_base + "/driver_status";

        pub_timing_ = this->create_publisher<std_msgs::msg::String>(timing_topic, 10);
        pub_status_ = this->create_publisher<std_msgs::msg::String>(status_topic, 10);

        // 初始化标定加载器：标定文件 > 规格书FOV反算 > 空标定，三级兜底，均不影响出图
        double fov_x_deg = this->get_parameter("fov_x_deg").as_double();
        double fov_y_deg = this->get_parameter("fov_y_deg").as_double();
        info_loader_ = std::make_shared<usv_vision_gmsl_cpp::CameraInfoLoader>(
            this->get_logger(), frame_id_, camera_info_url_, fov_x_deg, fov_y_deg);

        // M3：C++ 内置推流参数（默认关闭，走原 Python rtsp_stream_node）
        this->declare_parameter("enable_internal_stream", false);
        this->declare_parameter("stream_rtsp_url", std::string(""));
        this->declare_parameter("stream_bitrate", 4000000);
        this->declare_parameter("stream_fps", 30);
        this->declare_parameter("stream_iframe_interval", 15);
        this->declare_parameter("stream_idr_interval", 30);
        this->declare_parameter("stream_preset_level", 1);
        this->declare_parameter("stream_control_rate", 1);
        stream_cfg_enabled_ = this->get_parameter("enable_internal_stream").as_bool();
        stream_url_ = this->get_parameter("stream_rtsp_url").as_string();
        stream_bitrate_ = static_cast<int>(this->get_parameter("stream_bitrate").as_int());
        stream_fps_ = static_cast<int>(this->get_parameter("stream_fps").as_int());
        stream_iframe_ = static_cast<int>(this->get_parameter("stream_iframe_interval").as_int());
        stream_idr_ = static_cast<int>(this->get_parameter("stream_idr_interval").as_int());
        stream_preset_ = static_cast<int>(this->get_parameter("stream_preset_level").as_int());
        stream_control_ = static_cast<int>(this->get_parameter("stream_control_rate").as_int());

        // 按需分发：image_raw(全分辨率) / image_algo(算法尺寸，识别与观察共用)
        usv_vision_gmsl_cpp::CameraImagePublisherOptions pub_opts;
        pub_opts.frame_id = frame_id_;
        pub_opts.raw_image_topic = image_topic;
        pub_opts.raw_info_topic = info_topic;
        pub_opts.algo_image_topic = camera_base + "/image_algo";
        pub_opts.algo_info_topic = camera_base + "/camera_info_algo";
        pub_opts.raw_reliability = this->get_parameter("image_qos_reliability").as_string();
        pub_opts.raw_depth = static_cast<int>(this->get_parameter("image_qos_depth").as_int());

        // image_algo 独立 QoS：默认 best_effort/depth1（识别只取最新帧，
        // 避免 reliable 发布给慢读者带来重传开销）；可用参数回退到 reliable
        this->declare_parameter("algo_qos_reliability", std::string("best_effort"));
        this->declare_parameter("algo_qos_depth", 1);
        pub_opts.algo_reliability = this->get_parameter("algo_qos_reliability").as_string();
        pub_opts.algo_depth = static_cast<int>(this->get_parameter("algo_qos_depth").as_int());

        // 缩放后端与线程数（供整机资源分配切换：GPU vs CPU）
        this->declare_parameter("algo_resize_backend", std::string("cuda"));
        this->declare_parameter("algo_resize_threads", 1);
        pub_opts.algo_resize_backend = this->get_parameter("algo_resize_backend").as_string();
        pub_opts.algo_resize_threads = static_cast<int>(this->get_parameter("algo_resize_threads").as_int());
        pub_opts.enable_algo = this->get_parameter("enable_algo_stream").as_bool();
        pub_opts.algo_width = static_cast<int>(this->get_parameter("algo_width").as_int());
        pub_opts.algo_height = static_cast<int>(this->get_parameter("algo_height").as_int());

        images_ = std::make_unique<usv_vision_gmsl_cpp::CameraImagePublishers>(
            this, info_loader_, pub_opts);

        RCLCPP_INFO(this->get_logger(),
            "image distribution: raw=%s(%s/depth%d) algo=%s(%dx%d, 同 image_qos)",
            image_topic.c_str(),
            pub_opts.raw_reliability.c_str(), pub_opts.raw_depth,
            pub_opts.enable_algo ? pub_opts.algo_image_topic.c_str() : "disabled",
            pub_opts.enable_algo ? pub_opts.algo_width : 0,
            pub_opts.enable_algo ? pub_opts.algo_height : 0);

        // 初始化Miivii SDK
        if (enable_hw_timestamp_) {
            init_gmsl_sdk();
        }

        // 启动采集线程
        running_ = true;
        capture_thread_ = std::thread(&GmslCameraNode::capture_loop, this);

        RCLCPP_INFO(this->get_logger(),
            "GMSL Camera Node started: sensor=%s, device=%s, hw_timestamp=%s",
            sensor_name_.c_str(), device_path_.c_str(),
            enable_hw_timestamp_ ? "enabled" : "disabled");
    }

    ~GmslCameraNode()
    {
        running_ = false;
        if (capture_thread_.joinable()) {
            capture_thread_.join();
        }
        if (capture_ && capture_->isOpened()) {
            capture_->release();
        }
    }

private:
    // 参数
    std::string sensor_name_;
    std::string device_path_;
    std::string frame_id_;
    std::string camera_info_url_;
    int width_;
    int height_;
    double fps_;
    std::string pixel_format_;
    std::string gstreamer_pipeline_;
    bool enable_hw_timestamp_;
    bool auto_detect_resolution_;
    uint8_t camera_no_;

    // 发布者（image/camera_info 的按需分发统一由 CameraImagePublishers 承担）
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_timing_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_status_;

    // 采集
    std::unique_ptr<cv::VideoCapture> capture_;
    std::shared_ptr<miivii::MvGmslCamera> gmsl_camera_;
    std::thread capture_thread_;
    std::atomic<bool> running_{false};

    // camera_info 标定加载器（与Python CameraInfoLoader对齐，内参缺失时降级为空标定）
    // 用 shared_ptr：同一份加载器同时供 image_raw 与 image_algo 的 camera_info 使用
    // （它按分辨率缓存并自动缩放内参，算法图的内参因此天然一致）
    std::shared_ptr<usv_vision_gmsl_cpp::CameraInfoLoader> info_loader_;

    // 一次成帧、按需分发（image_raw / image_algo）
    std::unique_ptr<usv_vision_gmsl_cpp::CameraImagePublishers> images_;
    // M3：C++ 内置推流（绕过 DDS/Python，直接进硬件编码器）
    std::unique_ptr<usv_vision_gmsl_cpp::H264Streamer> streamer_;
    bool stream_cfg_enabled_ = false;
    std::string stream_url_;
    int stream_bitrate_ = 4000000;
    int stream_fps_ = 30;
    int stream_iframe_ = 15;
    int stream_idr_ = 30;
    int stream_preset_ = 1;
    int stream_control_ = 1;

    uint8_t parse_camera_no(const std::string& device_path)
    {
        // 从 "/dev/video0" 中提取 0
        size_t pos = device_path.find("video");
        if (pos != std::string::npos) {
            std::string num_str = device_path.substr(pos + 5);
            return static_cast<uint8_t>(std::stoi(num_str));
        }
        return 0;
    }

    void detect_device_capabilities()
    {
        /**
         * 自动检测设备能力
         * 尝试打开设备并读取实际支持的分辨率和帧率
         */
        RCLCPP_INFO(this->get_logger(), "Auto-detecting device capabilities for %s", device_path_.c_str());

        cv::VideoCapture test_cap(device_path_, cv::CAP_V4L2);
        if (!test_cap.isOpened()) {
            RCLCPP_WARN(this->get_logger(), "Cannot open device for auto-detection, using config values");
            return;
        }

        // 读取实际分辨率
        int actual_width = static_cast<int>(test_cap.get(cv::CAP_PROP_FRAME_WIDTH));
        int actual_height = static_cast<int>(test_cap.get(cv::CAP_PROP_FRAME_HEIGHT));
        double actual_fps = test_cap.get(cv::CAP_PROP_FPS);

        if (actual_width > 0 && actual_height > 0) {
            RCLCPP_INFO(this->get_logger(),
                "Detected resolution: %dx%d @ %.1f fps (config: %dx%d @ %.1f fps)",
                actual_width, actual_height, actual_fps,
                width_, height_, fps_);

            // 如果检测到的分辨率与配置不同，使用检测值
            if (actual_width != width_ || actual_height != height_) {
                RCLCPP_WARN(this->get_logger(),
                    "Using detected resolution instead of config");
                width_ = actual_width;
                height_ = actual_height;
            }

            if (actual_fps > 0 && std::abs(actual_fps - fps_) > 1.0) {
                fps_ = actual_fps;
            }
        }

        test_cap.release();
    }

    void init_gmsl_sdk()
    {
        try {
            // 从device_path解析camera_no
            int camera_no = 0;
            sscanf(device_path_.c_str(), "/dev/video%d", &camera_no);

            // ===== 使用三参数构造函数（与SDK示例一致） =====
            // 创建camera_context_t数组
            camera_context_t ctx[1];
            ctx[0].dev_node = device_path_;
            ctx[0].camera_fmt_str = pixel_format_;
            ctx[0].output_fmt_str = "BGRA32";  // 输出格式
            ctx[0].cam_w = width_;
            ctx[0].cam_h = height_;
            ctx[0].out_w = width_;   // 输出全分辨率（不缩放）
            ctx[0].out_h = height_;  // 输出全分辨率（不缩放）

            // 初始化GMSL SDK配置
            // ===== 8路配置稳定版：与官方demo匹配 =====
            // 注意：sync_camera_num/bit_draw 在此刻意写死为 8 路全触发，不随实际相机数变化。
            // 原因：Miivii GMSL SDK 的同步触发是"整板级"配置——deserializer 会按 sync_freq
            // 统一给所有链路打同步脉冲。实测只有按官方 demo 用满 8 路(num=8, bit_draw=0xff)时
            // 硬件时间戳才稳定；改成按实际相机数(如单路 num=1)会出现时间戳抖动/取帧失败。
            // 代价：即使只接 1 路，SDK 也按 8 路调度，单相机场景下有少量无用触发开销，但换来
            //       时间戳稳定，属于可接受取舍。若未来验证了按需路数也稳定，可将其参数化。
            // 注：这里只影响"触发多少路"；本节点实际取哪一路由 camera_no_ 决定(见 GetImageCvMat)。
            sync_out_a_cfg_client_t cfg = {};
            cfg.sync_camera_num = 8;                        // 与demo默认一致（写死，理由见上）
            cfg.sync_freq = static_cast<uint8_t>(fps_);
            cfg.async_camera_num = 0;
            cfg.async_freq = 0;
            cfg.sync_camera_bit_draw = 0xff;                // 触发全部8路（写死，理由见上）

            // 支持手动设置 bit_mask（用于多相机硬件同步）
            int manual_bit_mask = this->get_parameter("sync_camera_bit_mask").as_int();
            if (manual_bit_mask > 0) {
                cfg.sync_camera_bit_mask = static_cast<uint8_t>(manual_bit_mask);
            } else {
                cfg.sync_camera_bit_mask = (1 << camera_no);    // 默认：单相机掩码
            }

            cfg.async_camera_bit_draw = 0;

            // 初始化异步相机位置数组
            for (int i = 0; i < 8; i++) {
                cfg.async_camera_pos[i] = 0;
            }

            RCLCPP_INFO(this->get_logger(),
                "Initializing GMSL SDK (8-cam stable): camera_no=%d, sync_num=%d, sync_freq=%d, bit_draw=0x%02x, bit_mask=0x%02x",
                camera_no, cfg.sync_camera_num, cfg.sync_freq, cfg.sync_camera_bit_draw, cfg.sync_camera_bit_mask);

            // 使用三参数构造函数（与SDK示例一致）
            gmsl_camera_ = std::make_shared<miivii::MvGmslCamera>(ctx, 1, cfg);

            RCLCPP_INFO(this->get_logger(),
                "GMSL SDK initialized successfully with camera context");

        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(),
                "Failed to initialize GMSL SDK: %s. Falling back to ROS timestamp.", e.what());
            enable_hw_timestamp_ = false;
        }
    }

    std::string build_gstreamer_pipeline()
    {
        if (!gstreamer_pipeline_.empty()) {
            return gstreamer_pipeline_;
        }

        // 参考SDK示例的pipeline配置
        std::ostringstream ss;
        ss << "v4l2src device=" << device_path_ << " ! "
           << "video/x-raw,format=" << pixel_format_
           << ",width=" << width_ << ",height=" << height_ << " ! "
           << "queue ! "  // 添加queue缓冲
           << "videoconvert ! video/x-raw,format=BGR ! "
           << "appsink drop=false sync=true max-buffers=1";  // sync=true, drop=false
        return ss.str();
    }

    void capture_loop()
    {
        RCLCPP_INFO(this->get_logger(), "Capture loop started, using SDK GetImageCvMat");

        publish_status("playing", "sdk_direct");

        // SDK的GetImageCvMat需要数组和指针
        cv::Mat frames[1];  // 数组

        // 直接使用SDK的GetImageCvMat接口获取图像
        while (running_) {
            uint64_t hw_timestamp_ns = 0;

            // 使用SDK接口获取图像和时间戳（传指针）
            bool ok = gmsl_camera_->GetImageCvMat(frames, &hw_timestamp_ns, camera_no_);

            if (!ok || frames[0].empty()) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                    "Failed to get image from SDK");
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }

            // 构建时间戳
            builtin_interfaces::msg::Time stamp;
            if (enable_hw_timestamp_ && hw_timestamp_ns > 0) {
                stamp.sec = static_cast<int32_t>(hw_timestamp_ns / 1000000000ULL);
                stamp.nanosec = static_cast<uint32_t>(hw_timestamp_ns % 1000000000ULL);
            } else {
                // 降级到ROS时间
                stamp = this->now();
            }

            // 发布图像和相关消息
            publish_frame(frames[0], stamp);
        }

        RCLCPP_INFO(this->get_logger(), "Capture loop ended");
    }

    void publish_frame(const cv::Mat& frame, const builtin_interfaces::msg::Time& stamp)
    {
        // SDK输出的是BGRA32格式，需要转换为BGR
        cv::Mat bgr_frame;
        if (frame.channels() == 4) {
            // BGRA -> BGR
            cv::cvtColor(frame, bgr_frame, cv::COLOR_BGRA2BGR);
        } else if (frame.channels() == 2) {
            // UYVY -> BGR
            cv::cvtColor(frame, bgr_frame, cv::COLOR_YUV2BGR_UYVY);
        } else {
            // 已经是BGR或其他3通道格式
            bgr_frame = frame;
        }

        // 发布图像：一次成帧、按需分发（全分辨率给推流/录像，算法尺寸给识别，限频调试给 rviz）
        auto published = images_->publish(bgr_frame, &frame, stamp);

        // M3：内置推流（懒启动，用首帧尺寸）
        if (stream_cfg_enabled_ && !stream_url_.empty() && frame.channels() == 4) {
            if (!streamer_) {
                streamer_ = std::make_unique<usv_vision_gmsl_cpp::H264Streamer>();
                usv_vision_gmsl_cpp::H264StreamerOptions so;
                so.rtsp_url = stream_url_;
                so.width = frame.cols;
                so.height = frame.rows;
                so.fps = stream_fps_;
                so.bitrate = stream_bitrate_;
                so.iframe_interval = stream_iframe_;
                so.idr_interval = stream_idr_;
                so.preset_level = stream_preset_;
                so.control_rate = stream_control_;
                std::string serr;
                if (!streamer_->start(so, &serr)) {
                    RCLCPP_ERROR(this->get_logger(), "internal stream start failed: %s", serr.c_str());
                    stream_cfg_enabled_ = false;
                } else {
                    RCLCPP_INFO(this->get_logger(),
                        "internal stream started: %s (%dx%d@%d, %dkbps)",
                        so.rtsp_url.c_str(), so.width, so.height, so.fps, so.bitrate / 1000);
                }
            }
            if (streamer_) {
                streamer_->push(frame);   // 原始 BGRA，nvvidconv 按 BGRx 解释
            }
        }

        // 发布timing
        publish_timing(stamp, published);
    }

    void publish_timing(
        const builtin_interfaces::msg::Time& capture_stamp,
        const usv_vision_gmsl_cpp::FramePublishInfo& published)
    {
        auto publish_stamp_rclcpp = this->now();
        // 手动转换 rclcpp::Time 为 builtin_interfaces::msg::Time
        builtin_interfaces::msg::Time publish_stamp;
        publish_stamp.sec = static_cast<int32_t>(publish_stamp_rclcpp.nanoseconds() / 1000000000);
        publish_stamp.nanosec = static_cast<uint32_t>(publish_stamp_rclcpp.nanoseconds() % 1000000000);

        json timing;
        timing["frame_id"] = frame_id_;
        timing["sensor_name"] = sensor_name_;
        timing["source_type"] = enable_hw_timestamp_ ? "v4l2_gmsl_hw_timestamp" : "v4l2_gmsl";
        timing["backend"] = "miivii_sdk";   // 采集后端：本节点用 Miivii SDK 直接取帧（非 gstreamer/v4l2）
        timing["source"] = device_path_;    // 数据来源设备路径
        timing["image"]["width"] = published.raw_width;
        timing["image"]["height"] = published.raw_height;
        // 算法图/调试图的实际产出尺寸（便于下游核对分发是否生效）
        timing["image_algo"]["published"] = published.algo_published;
        timing["image_algo"]["width"] = published.algo_width;
        timing["image_algo"]["height"] = published.algo_height;
        timing["image_raw"]["published"] = published.raw_published;
        timing["timestamps"]["capture_stamp_ros"]["sec"] = capture_stamp.sec;
        timing["timestamps"]["capture_stamp_ros"]["nanosec"] = capture_stamp.nanosec;
        timing["timestamps"]["publish_stamp_ros"]["sec"] = publish_stamp.sec;
        timing["timestamps"]["publish_stamp_ros"]["nanosec"] = publish_stamp.nanosec;

        double latency_ms = (publish_stamp.sec - capture_stamp.sec) * 1000.0
                          + (static_cast<int64_t>(publish_stamp.nanosec) - static_cast<int64_t>(capture_stamp.nanosec)) / 1e6;
        timing["latency_ms"]["driver_latency_ms"] = latency_ms;

        auto msg = std_msgs::msg::String();
        msg.data = timing.dump();
        pub_timing_->publish(msg);
    }

    void publish_status(const std::string& state, const std::string& detail)
    {
        json status;
        status["state"] = state;
        status["detail"] = detail;
        status["source_type"] = enable_hw_timestamp_ ? "v4l2_gmsl_hw_timestamp" : "v4l2_gmsl";
        // stamp：状态发布时刻（与 Python 版对齐），格式 {sec, nanosec}
        auto now = this->now();
        status["stamp"]["sec"] = static_cast<int32_t>(now.nanoseconds() / 1000000000);
        status["stamp"]["nanosec"] = static_cast<uint32_t>(now.nanoseconds() % 1000000000);
        status["camera_name"] = sensor_name_;
        status["device_path"] = device_path_;

        auto msg = std_msgs::msg::String();
        msg.data = status.dump();
        pub_status_->publish(msg);
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GmslCameraNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

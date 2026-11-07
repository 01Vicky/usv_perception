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
#include <nlohmann/json.hpp>  // 需要安装: sudo apt install nlohmann-json3-dev

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
        this->declare_parameter("image_qos_reliability", "best_effort");
        this->declare_parameter("image_qos_depth", 1);

        // 获取参数
        sensor_name_ = this->get_parameter("sensor_name").as_string();
        device_path_ = this->get_parameter("device_path").as_string();
        frame_id_ = this->get_parameter("frame_id").as_string();
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
        auto qos = get_qos_profile();
        std::string image_topic = "/sensors/camera/" + sensor_name_ + "/image_raw";
        std::string info_topic = "/sensors/camera/" + sensor_name_ + "/camera_info";
        std::string timing_topic = "/sensors/camera/" + sensor_name_ + "/frame_timing_json";
        std::string status_topic = "/sensors/camera/" + sensor_name_ + "/driver_status";

        pub_image_ = this->create_publisher<sensor_msgs::msg::Image>(image_topic, qos);
        pub_info_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(info_topic, qos);
        pub_timing_ = this->create_publisher<std_msgs::msg::String>(timing_topic, 10);
        pub_status_ = this->create_publisher<std_msgs::msg::String>(status_topic, 10);

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
    int width_;
    int height_;
    double fps_;
    std::string pixel_format_;
    std::string gstreamer_pipeline_;
    bool enable_hw_timestamp_;
    bool auto_detect_resolution_;
    uint8_t camera_no_;

    // 发布者
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_image_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pub_info_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_timing_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_status_;

    // 采集
    std::unique_ptr<cv::VideoCapture> capture_;
    std::shared_ptr<miivii::MvGmslCamera> gmsl_camera_;
    std::thread capture_thread_;
    std::atomic<bool> running_{false};

    rclcpp::QoS get_qos_profile()
    {
        std::string reliability = this->get_parameter("image_qos_reliability").as_string();
        int depth = this->get_parameter("image_qos_depth").as_int();

        rclcpp::QoS qos(depth);
        if (reliability == "reliable") {
            qos.reliable();
        } else {
            qos.best_effort();
        }
        return qos;
    }

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

            // 初始化GMSL SDK配置
            sync_out_a_cfg_client_t cfg = {};

            // ===== 关键修复：ZED X立体相机需要配置为2路相机 =====
            // ZED X (video0+video1) 是立体相机对，必须同时触发
            // 参考SDK示例成功运行的配置：-m 2 参数表示2路相机
            if (camera_no == 0 || camera_no == 1) {
                // ZED X立体相机配置
                cfg.sync_camera_num = 2;                  // 2路同步相机
                cfg.sync_camera_bit_draw = 0x03;          // 0x03 = bit0 + bit1 (video0 + video1)
                RCLCPP_INFO(this->get_logger(),
                    "Configuring for ZED X stereo camera (video0+video1)");
            } else {
                // 其他GMSL相机：单路配置
                cfg.sync_camera_num = 1;
                cfg.sync_camera_bit_draw = (1 << camera_no);
            }

            cfg.sync_freq = static_cast<uint8_t>(fps_);
            cfg.async_camera_num = 0;
            cfg.async_freq = 0;
            cfg.async_camera_bit_draw = 0;

            // 初始化异步相机位置数组（必须清零）
            for (int i = 0; i < 8; i++) {
                cfg.async_camera_pos[i] = 0;
            }

            gmsl_camera_ = std::make_shared<miivii::MvGmslCamera>(cfg);

            RCLCPP_INFO(this->get_logger(),
                "Miivii GMSL SDK initialized: camera_no=%d, sync_num=%d, sync_freq=%d, bit_draw=0x%02x",
                camera_no, cfg.sync_camera_num, cfg.sync_freq, cfg.sync_camera_bit_draw);

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

        // ===== 关键修复：参考SDK示例的GStreamer配置 =====
        // SDK示例pipeline：v4l2src ! video/x-raw ! queue ! appsink drop=false sync=true
        std::ostringstream ss;
        ss << "v4l2src device=" << device_path_ << " ! "
           << "video/x-raw,format=" << pixel_format_
           << ",width=" << width_ << ",height=" << height_ << " ! "
           << "queue ! "  // 添加queue缓冲元素
           << "videoconvert ! video/x-raw,format=BGR ! "
           << "appsink drop=false sync=true max-buffers=1";  // drop=false, sync=true

        return ss.str();
    }

    void capture_loop()
    {
        while (running_) {
            // 打开相机
            std::string pipeline = build_gstreamer_pipeline();
            RCLCPP_INFO(this->get_logger(), "Opening GStreamer pipeline: %s", pipeline.c_str());
            capture_ = std::make_unique<cv::VideoCapture>(pipeline, cv::CAP_GSTREAMER);

            if (!capture_->isOpened()) {
                RCLCPP_ERROR(this->get_logger(), "Failed to open camera: %s", device_path_.c_str());
                publish_status("error", "open_failed");
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
                continue;
            }

            RCLCPP_INFO(this->get_logger(), "Camera opened successfully");
            publish_status("playing", "gstreamer");

            // 采集循环
            int frame_count = 0;
            while (running_) {
                cv::Mat frame;
                if (!capture_->read(frame) || frame.empty()) {
                    RCLCPP_WARN(this->get_logger(), "Failed to read frame after %d frames", frame_count);
                    break;
                }

                frame_count++;

                // 获取时间戳
                builtin_interfaces::msg::Time stamp;
                if (enable_hw_timestamp_ && gmsl_camera_) {
                    uint64_t hw_timestamp_ns = 0;
                    bool ok = gmsl_camera_->GetGmslTimeStamp(camera_no_, hw_timestamp_ns);
                    if (ok && hw_timestamp_ns > 0) {
                        stamp.sec = static_cast<int32_t>(hw_timestamp_ns / 1000000000ULL);
                        stamp.nanosec = static_cast<uint32_t>(hw_timestamp_ns % 1000000000ULL);
                    } else {
                        // 硬件时间戳获取失败，降级到ROS时间
                        if (frame_count % 30 == 0) {  // 每秒打印一次警告
                            RCLCPP_WARN(this->get_logger(),
                                "Hardware timestamp invalid (ok=%d, ts=%lu), using ROS time",
                                ok, hw_timestamp_ns);
                        }
                        stamp = this->now();
                    }
                } else {
                    // 使用ROS系统时间
                    stamp = this->now();
                }

                // 发布图像和相关消息
                publish_frame(frame, stamp);
            }

            capture_->release();
            RCLCPP_WARN(this->get_logger(), "Camera connection lost, restarting...");
            publish_status("restarting", "");
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
    }

    void publish_frame(const cv::Mat& frame, const builtin_interfaces::msg::Time& stamp)
    {
        // 发布图像
        auto img_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
        img_msg->header.stamp = stamp;
        img_msg->header.frame_id = frame_id_;
        pub_image_->publish(*img_msg);

        // 发布camera_info
        auto info_msg = sensor_msgs::msg::CameraInfo();
        info_msg.header.stamp = stamp;
        info_msg.header.frame_id = frame_id_;
        info_msg.width = frame.cols;
        info_msg.height = frame.rows;
        // TODO: 从camera_info_url加载标定参数
        pub_info_->publish(info_msg);

        // 发布timing
        publish_timing(stamp, frame.cols, frame.rows);
    }

    void publish_timing(const builtin_interfaces::msg::Time& capture_stamp, int width, int height)
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
        timing["image"]["width"] = width;
        timing["image"]["height"] = height;
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

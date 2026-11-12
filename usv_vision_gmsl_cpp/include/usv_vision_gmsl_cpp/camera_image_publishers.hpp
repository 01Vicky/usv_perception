/**
 * CameraImagePublishers —— 采集侧"按需分发"的头文件实现（header-only）
 *
 * 背景：原来采集节点只生产一种规格（全分辨率 BGR）并广播出去，识别、推流、rviz
 * 各个消费者各自从 DDS 里捞一遍、各自再转一遍，同一帧在机器内部被搬多份。
 * 本类把"分发"收回到采集进程内，一次成帧、按消费者需求分发：
 *
 *   image_raw   全分辨率 BGR —— 推流/录像等"要原图"的消费者（QoS 由参数决定）
 *   image_algo  算法尺寸 BGR —— 识别链（yolo_infer_node）与人工观察（rviz）共用，
 *               默认 640x640，QoS 与 image_raw 同一套（由 image_qos_* 决定，当前 reliable/depth2）。
 *
 * 为什么不再单开一条"调试图"：调试图与算法图是同一份像素，只是帧率不同。
 * 需要看检测结果叠加时，用 yolo_infer_node 的 annotated 话题。
 *
 * QoS 策略：发布端统一 reliable/depth2（由参数控制），订阅端一律 best_effort/depth1。
 * DDS 兼容方向是"提供强于等于请求"，所以 reliable 发布 + best_effort 订阅天然成立；
 * 好处是 rviz 用默认 QoS 就能看到画面，不必记得改 Reliability。
 * 已实测：8.4MB 帧 × depth10 × 2 个订阅者时 publish() 平均 5ms、最大 9~14ms、0 次超 33ms，
 * 1.2MB × depth2 的余量更大，可靠发布不会拖慢采集线程；万一出现，改 image_qos_* 即可回退。
 *
 * 设计约束：
 * - 只做"尺寸分发"，不做像素格式转换（全分辨率帧的格式链路保持原样，避免行为漂移）
 * - 算法图复用固定 Mat 缓冲，尺寸不变时不再重复分配
 * - 算法图无订阅者时跳过 resize 与发布（没人看/没开推理时零开销）
 * - camera_info 走同一个 CameraInfoLoader（按分辨率缓存并自动缩放内参），
 *   因此算法图的 camera_info 内参与算法图分辨率天然一致
 * - image_algo 与 image_raw 共用一套发布 QoS；订阅端建议 best_effort/depth1（只处理最新帧）
 *
 * 依赖：rclcpp / sensor_msgs / cv_bridge / OpenCV（不含 Miivii SDK，便于单独静态检查）
 */
#ifndef USV_VISION_GMSL_CPP_CAMERA_IMAGE_PUBLISHERS_HPP
#define USV_VISION_GMSL_CPP_CAMERA_IMAGE_PUBLISHERS_HPP

#include <algorithm>
#include <cstdint>
#include <memory>
#include <string>
#include <utility>

#include <builtin_interfaces/msg/time.hpp>
#include <cv_bridge/cv_bridge.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>

#include <opencv2/opencv.hpp>

#include "usv_vision_gmsl_cpp/algo_resize_cuda.hpp"

#include "usv_vision_gmsl_cpp/camera_info_loader.hpp"

namespace usv_vision_gmsl_cpp
{

/// 一次 publish() 的实际产出，供节点侧写 frame_timing_json 使用
struct FramePublishInfo
{
    int raw_width = 0;
    int raw_height = 0;
    int algo_width = 0;
    int algo_height = 0;
    bool raw_published = false;
    bool algo_published = false;
};

struct CameraImagePublisherOptions
{
    std::string frame_id;

    // 话题名（由节点按 /sensors/camera/{sensor_name}/... 拼好后传入）
    std::string raw_image_topic;
    std::string raw_info_topic;
    std::string algo_image_topic;
    std::string algo_info_topic;

    // image 话题的发布 QoS（image_raw 与 image_algo 共用，由节点参数决定）：
    // 默认 best_effort/depth2；当前现场配置为 reliable/depth2，以便 rviz 用默认 QoS 直接看图
    std::string raw_reliability = "best_effort";
    int raw_depth = 2;
    // image_algo 独立 QoS：识别只取最新帧，用 best_effort 可避免
    // "reliable 发布 + 慢读者" 的发布端重传开销（板上实测可省 1 核级）
    std::string algo_reliability = "best_effort";
    int algo_depth = 1;
    // 算法图缩放后端：cuda（GPU 融合转色+缩放）| cpu（OpenCV）
    // 供整机资源分配时切换：GPU 富余走 cuda，CPU 富余/无 GPU 走 cpu
    std::string algo_resize_backend = "cuda";
    int algo_resize_threads = 1;   // 仅 CPU 后端生效

    // 触发式发布：没人订阅就完全不做（不拷贝、不序列化、不发布）。
    // 推流/录包/rviz 订阅 image_raw 时才出图；没订阅者时连 cv_bridge 的 8.8MB 拷贝都省掉。
    bool publish_only_when_subscribed = true;

    // 算法图传输格式：raw（sensor_msgs/Image）| jpeg（sensor_msgs/CompressedImage）
    // jpeg 用 cv::imencode 编码后发布到 <algo_image_topic>/compressed，二选一、不并行
    std::string algo_transport = "raw";
    int algo_jpeg_quality = 95;

    // 算法图（识别 + 观察共用）：尺寸与开关
    bool enable_algo = true;
    int algo_width = 640;
    int algo_height = 640;
};

class CameraImagePublishers
{
public:
    CameraImagePublishers(
        rclcpp::Node * node,
        std::shared_ptr<CameraInfoLoader> info_loader,
        CameraImagePublisherOptions options)
    : node_(node),
      info_loader_(std::move(info_loader)),
      options_(std::move(options))
    {
        if (!options_.raw_image_topic.empty()) {
            raw_pub_ = node_->create_publisher<sensor_msgs::msg::Image>(
                options_.raw_image_topic, make_raw_qos());
        }
        if (!options_.raw_info_topic.empty()) {
            raw_info_pub_ = node_->create_publisher<sensor_msgs::msg::CameraInfo>(
                options_.raw_info_topic, make_raw_qos());
        }
        if (options_.enable_algo && options_.algo_transport == "jpeg" &&
            !options_.algo_image_topic.empty())
        {
            algo_compressed_pub_ =
                node_->create_publisher<sensor_msgs::msg::CompressedImage>(
                    options_.algo_image_topic + "/compressed", make_algo_qos());
        } else if (options_.enable_algo && !options_.algo_image_topic.empty()) {
            // 与 image_raw 同一套发布 QoS：rviz 默认（reliable）即可看到画面；
            // yolo/rtsp 侧用 best_effort 订阅，兼容方向成立
            algo_pub_ = node_->create_publisher<sensor_msgs::msg::Image>(
                options_.algo_image_topic, make_algo_qos());
            if (!options_.algo_info_topic.empty()) {
                algo_info_pub_ = node_->create_publisher<sensor_msgs::msg::CameraInfo>(
                    options_.algo_info_topic, make_algo_qos());
            }
        }
    }

    /// 发布一帧：image_raw(+camera_info) → image_algo(+camera_info_algo)
    FramePublishInfo publish(const cv::Mat & bgr, const cv::Mat * bgra_src,
                            const builtin_interfaces::msg::Time & stamp)
    {
        FramePublishInfo info;
        if (bgr.empty()) {
            return info;
        }

        info.raw_width = bgr.cols;
        info.raw_height = bgr.rows;

        std_msgs::msg::Header header;
        header.stamp = stamp;
        header.frame_id = options_.frame_id;

        publish_raw(bgr, header, info);
        publish_algo(bgr, bgra_src, header, info);
        return info;
    }

    // ---- 供节点日志/自检使用 ----
    const CameraImagePublisherOptions & options() const { return options_; }

    bool algo_enabled() const { return algo_pub_ != nullptr; }

    size_t raw_subscriber_count() const
    {
        return raw_pub_ == nullptr ? 0U : raw_pub_->get_subscription_count();
    }

    size_t algo_subscriber_count() const
    {
        return algo_pub_ == nullptr ? 0U : algo_pub_->get_subscription_count();
    }

private:
    void publish_raw(
        const cv::Mat & bgr,
        const std_msgs::msg::Header & header,
        FramePublishInfo & info)
    {
        const bool raw_wanted = raw_pub_ != nullptr &&
            (!options_.publish_only_when_subscribed || raw_pub_->get_subscription_count() > 0U);
        if (raw_wanted) {
            auto msg = cv_bridge::CvImage(header, "bgr8", bgr).toImageMsg();
            raw_pub_->publish(*msg);
            info.raw_published = true;
        }
        const bool raw_info_wanted = raw_info_pub_ != nullptr &&
            (!options_.publish_only_when_subscribed || raw_info_pub_->get_subscription_count() > 0U);
        if (raw_info_wanted) {
            auto info_msg = info_loader_->get(info.raw_width, info.raw_height);
            info_msg.header = header;
            raw_info_pub_->publish(info_msg);
        }
    }

    void publish_algo(
        const cv::Mat & bgr,
        const cv::Mat * bgra_src,
        const std_msgs::msg::Header & header,
        FramePublishInfo & info)
    {
        if (algo_pub_ == nullptr && algo_compressed_pub_ == nullptr) {
            return;
        }
        // 无订阅者时严格零开销：不 resize、不编码、不分配、不发布
        const bool algo_wanted =
            (algo_pub_ != nullptr && algo_pub_->get_subscription_count() > 0U) ||
            (algo_compressed_pub_ != nullptr && algo_compressed_pub_->get_subscription_count() > 0U);
        if (options_.publish_only_when_subscribed && !algo_wanted) {
            return;
        }

        const int target_w = options_.algo_width;
        const int target_h = options_.algo_height;

        const cv::Mat * source = nullptr;
        if (target_w <= 0 || target_h <= 0 || (target_w == bgr.cols && target_h == bgr.rows)) {
            // 未配置算法尺寸（或与原始尺寸相同）→ 直接复用原图，不做无谓拷贝
            source = &bgr;
        } else {
            bool resized = false;
            if (options_.algo_resize_backend == "cuda" && bgra_src != nullptr && !bgra_src->empty()) {
                resized = gpu_resize_bgra_to_bgr(*bgra_src, algo_mat_, info.algo_width, info.algo_height);
            }
            if (!resized) {
                cv::resize(bgr, algo_mat_, cv::Size(target_w, target_h), 0, 0, cv::INTER_LINEAR);
            }
            source = &algo_mat_;
        }

        if (algo_compressed_pub_ != nullptr) {
            // 压缩传输：JPEG 编码后发 CompressedImage（与 raw 二选一，不并行）
            std::vector<uchar> buf;
            const std::vector<int> params{cv::IMWRITE_JPEG_QUALITY, options_.algo_jpeg_quality};
            if (cv::imencode(".jpg", *source, buf, params) && !buf.empty()) {
                sensor_msgs::msg::CompressedImage msg;
                msg.header = header;
                msg.format = "jpeg";
                msg.data = std::move(buf);
                algo_compressed_pub_->publish(msg);
            }
        } else {
            auto msg = cv_bridge::CvImage(header, "bgr8", *source).toImageMsg();
            algo_pub_->publish(*msg);
        }
        info.algo_width = source->cols;
        info.algo_height = source->rows;
        info.algo_published = true;

        if (algo_info_pub_ != nullptr &&
            (!options_.publish_only_when_subscribed || algo_info_pub_->get_subscription_count() > 0U))
        {
            auto info_msg = info_loader_->get(info.algo_width, info.algo_height);
            info_msg.header = header;
            algo_info_pub_->publish(info_msg);
        }
    }

    rclcpp::QoS make_algo_qos() const
    {
        const int depth = std::max(1, options_.algo_depth);
        rclcpp::QoS qos(rclcpp::KeepLast(static_cast<size_t>(depth)));
        if (options_.algo_reliability == "reliable") {
            qos.reliable();
        } else {
            qos.best_effort();
        }
        return qos;
    }

    rclcpp::QoS make_raw_qos() const
    {
        const int depth = std::max(1, options_.raw_depth);
        rclcpp::QoS qos(static_cast<size_t>(depth));
        if (options_.raw_reliability == "reliable") {
            qos.reliable();
        } else {
            qos.best_effort();
        }
        return qos;
    }

    rclcpp::Node * node_;
    std::shared_ptr<CameraInfoLoader> info_loader_;
    CameraImagePublisherOptions options_;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr raw_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr raw_info_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr algo_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr algo_compressed_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr algo_info_pub_;

    cv::Mat algo_mat_;   // 算法图复用缓冲
};

}  // namespace usv_vision_gmsl_cpp

#endif  // USV_VISION_GMSL_CPP_CAMERA_IMAGE_PUBLISHERS_HPP

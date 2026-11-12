// yolo_infer_cpp_node.cpp —— 视觉识别的 C++ / TensorRT 原型
//
// 目的：验证把 yolo 推理从 Python（usv_vision 的 yolo_infer_node）挪到 C++ 后，
// 单帧耗时能否压到 25 ms 以内，从而跟得住 30 Hz 采集。**这是实验包，不动 usv_vision**。
//
// 与 Python 版的差异（刻意简化，先把性能跑出来）：
//   - 不做视图蒙板、时序平滑、候选框扩张、去重（这些是 Python 版后处理的一部分）
//   - 不做 min_box_* 尺寸过滤
//   - 类别按 class_names 映射，默认只保留 allowlist 里的类别
// 对齐的部分：输入 image_algo、输出 topics/消息字段、reference 坐标换算、
//             angle_model（pinhole / scaramuzza 鱼眼）角度算法。
//
// 自测用法见 README.md。

#include <chrono>
#include <cmath>
#include <cstring>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>
#include <algorithm>
#include <limits>
#include <iomanip>

#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>

#include <cuda_runtime_api.h>
#include <NvInfer.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <std_msgs/msg/string.hpp>
#include <cv_bridge/cv_bridge.h>

#include <usv_interfaces/msg/vision_detection.hpp>
#include <usv_interfaces/msg/vision_detection_array.hpp>
#include <usv_interfaces/msg/ai_vision_target.hpp>
#include <usv_interfaces/msg/ai_vision_targets.hpp>
#include <usv_interfaces/msg/pipeline_timestamp.hpp>

using namespace std::chrono_literals;

#define CUDA_CHECK(expr)                                                                    \
  do {                                                                                      \
    cudaError_t _e = (expr);                                                                \
    if (_e != cudaSuccess) {                                                                \
      RCLCPP_ERROR(rclcpp::get_logger("yolo_infer_cpp"), "CUDA error %s at %s:%d",          \
                   cudaGetErrorString(_e), __FILE__, __LINE__);                             \
      std::exit(EXIT_FAILURE);                                                              \
    }                                                                                       \
  } while (0)

namespace
{

class TrtLogger : public nvinfer1::ILogger
{
public:
  void log(Severity severity, const char * msg) noexcept override
  {
    if (severity <= Severity::kWARNING) {
      std::fprintf(stderr, "[TRT] %s\n", msg);
    }
  }
};

struct Detection
{
  float x1, y1, x2, y2, score;
  int cls;
};

float iou(const Detection & a, const Detection & b)
{
  const float ix1 = std::max(a.x1, b.x1);
  const float iy1 = std::max(a.y1, b.y1);
  const float ix2 = std::min(a.x2, b.x2);
  const float iy2 = std::min(a.y2, b.y2);
  const float iw = std::max(0.0f, ix2 - ix1);
  const float ih = std::max(0.0f, iy2 - iy1);
  const float inter = iw * ih;
  const float area_a = std::max(0.0f, a.x2 - a.x1) * std::max(0.0f, a.y2 - a.y1);
  const float area_b = std::max(0.0f, b.x2 - b.x1) * std::max(0.0f, b.y2 - b.y1);
  const float uni = area_a + area_b - inter;
  return uni > 0.0f ? inter / uni : 0.0f;
}

}  // namespace

class YoloInferCpp : public rclcpp::Node
{
public:
  YoloInferCpp()
  : rclcpp::Node("yolo_infer_cpp_node")
  {
    declareParameters();
    loadParams();
    loadEngine();
    createRosInterfaces();
    warmup();
    RCLCPP_INFO(get_logger(),
      "yolo_infer_cpp ready: engine=%s infer=%dx%d classes=%zu allowlist=%zu",
      engine_path_.c_str(), in_w_, in_h_, class_names_.size(), allowlist_.size());
  }

  ~YoloInferCpp() override
  {
    if (stream_) {cudaStreamDestroy(stream_);}
    if (dev_in_) {cudaFree(dev_in_);}
    if (dev_out_) {cudaFree(dev_out_);}
    if (ctx_) {delete ctx_;}
    if (engine_) {delete engine_;}
    if (runtime_) {delete runtime_;}
  }

private:
  // ───────────────────────── 参数 ─────────────────────────
  void declareParameters()
  {
    declare_parameter("sensor_name", "front");
    declare_parameter("camera_id", "");
    declare_parameter("image_topic", "");
    declare_parameter("camera_info_topic", "");
    declare_parameter("detections_topic", "");
    declare_parameter("objects_topic", "");
    declare_parameter("status_topic", "");
    declare_parameter("annotated_topic", "");
    declare_parameter("engine_path", "");
    declare_parameter("class_names", std::vector<std::string>{});
    declare_parameter("class_name_allowlist", std::vector<std::string>{});
    declare_parameter("conf_threshold", 0.40);
    declare_parameter("iou_threshold", 0.55);
    declare_parameter("max_det", 20);
    declare_parameter("reference_width", 1920);
    declare_parameter("reference_height", 1536);
    declare_parameter("angle_model", "pinhole");
    declare_parameter("publish_annotated", false);
    declare_parameter("publish_legacy_targets", true);
    declare_parameter("timing_log_hz", 1.0);
    declare_parameter("startup_report_hz", 0.5);
    declare_parameter("warmup_iters", 5);
  }

  void loadParams()
  {
    sensor_name_ = get_parameter("sensor_name").as_string();
    camera_id_ = get_parameter("camera_id").as_string();
    if (camera_id_.empty()) {camera_id_ = sensor_name_;}

    const std::string base = "/sensors/camera/" + sensor_name_;
    const std::string perc = "/perception/camera/" + sensor_name_;
    image_topic_ = get_parameter("image_topic").as_string();
    if (image_topic_.empty()) {image_topic_ = base + "/image_algo";}
    camera_info_topic_ = get_parameter("camera_info_topic").as_string();
    if (camera_info_topic_.empty()) {camera_info_topic_ = base + "/camera_info_algo";}
    detections_topic_ = get_parameter("detections_topic").as_string();
    if (detections_topic_.empty()) {detections_topic_ = perc + "/detections";}
    objects_topic_ = get_parameter("objects_topic").as_string();
    if (objects_topic_.empty()) {objects_topic_ = perc + "/objects";}
    status_topic_ = get_parameter("status_topic").as_string();
    if (status_topic_.empty()) {status_topic_ = perc + "/status";}
    annotated_topic_ = get_parameter("annotated_topic").as_string();
    if (annotated_topic_.empty()) {annotated_topic_ = perc + "/annotated";}

    engine_path_ = get_parameter("engine_path").as_string();
    class_names_ = get_parameter("class_names").as_string_array();
    allowlist_ = get_parameter("class_name_allowlist").as_string_array();
    conf_threshold_ = static_cast<float>(get_parameter("conf_threshold").as_double());
    iou_threshold_ = static_cast<float>(get_parameter("iou_threshold").as_double());
    max_det_ = static_cast<int>(get_parameter("max_det").as_int());
    ref_w_ = static_cast<int>(get_parameter("reference_width").as_int());
    ref_h_ = static_cast<int>(get_parameter("reference_height").as_int());
    angle_model_ = get_parameter("angle_model").as_string();
    publish_annotated_ = get_parameter("publish_annotated").as_bool();
    publish_legacy_ = get_parameter("publish_legacy_targets").as_bool();
    timing_log_hz_ = get_parameter("timing_log_hz").as_double();
    startup_report_hz_ = get_parameter("startup_report_hz").as_double();
    warmup_iters_ = static_cast<int>(get_parameter("warmup_iters").as_int());
  }

  // ───────────────────────── TensorRT ─────────────────────────
  void loadEngine()
  {
    std::ifstream file(engine_path_, std::ios::binary);
    if (!file.good()) {
      throw std::runtime_error("engine 打不开: " + engine_path_);
    }
    file.seekg(0, std::ios::end);
    const size_t size = static_cast<size_t>(file.tellg());
    file.seekg(0, std::ios::beg);
    std::vector<char> blob(size);
    file.read(blob.data(), static_cast<std::streamsize>(size));
    file.close();

    runtime_ = nvinfer1::createInferRuntime(logger_);
    engine_ = runtime_->deserializeCudaEngine(blob.data(), size);
    if (!engine_) {throw std::runtime_error("engine 反序列化失败");}
    ctx_ = engine_->createExecutionContext();
    if (!ctx_) {throw std::runtime_error("createExecutionContext 失败");}

    // 绑定名按 engine 实际 IO 取（这台板子上是 images / output0）
    for (int i = 0; i < engine_->getNbIOTensors(); ++i) {
      const char * name = engine_->getIOTensorName(i);
      const auto mode = engine_->getTensorIOMode(name);
      if (mode == nvinfer1::TensorIOMode::kINPUT) {
        in_name_ = name;
        auto d = engine_->getTensorShape(name);
        in_c_ = d.d[1]; in_h_ = d.d[2]; in_w_ = d.d[3];
      } else {
        out_name_ = name;
        auto d = engine_->getTensorShape(name);
        out_c_ = d.d[1]; out_n_ = d.d[2];
      }
    }
    if (in_name_.empty() || out_name_.empty()) {
      throw std::runtime_error("找不到输入/输出张量");
    }
    num_classes_ = out_c_ - 4;
    // 预处理画布预分配（复用，避免每帧分配）
    canvas_ = cv::Mat(in_h_, in_w_, CV_8UC3, cv::Scalar(114, 114, 114));
    // allowlist → 类别 id：解码时只扫这几路 channel，避免 8400×80 次跨行读
    allowed_ids_.clear();
    for (int c = 0; c < static_cast<int>(class_names_.size()); ++c) {
      if (allowlist_.empty() ||
        std::find(allowlist_.begin(), allowlist_.end(), class_names_[c]) != allowlist_.end())
      {
        allowed_ids_.push_back(c);
      }
    }
    if (allowed_ids_.empty()) {
      for (int c = 0; c < num_classes_; ++c) {allowed_ids_.push_back(c);}
    }

    host_in_.assign(static_cast<size_t>(in_c_ * in_h_ * in_w_), 0.0f);
    host_out_.assign(static_cast<size_t>(out_c_ * out_n_), 0.0f);
    CUDA_CHECK(cudaMalloc(&dev_in_, host_in_.size() * sizeof(float)));
    CUDA_CHECK(cudaMalloc(&dev_out_, host_out_.size() * sizeof(float)));
    CUDA_CHECK(cudaStreamCreate(&stream_));
    ctx_->setTensorAddress(in_name_.c_str(), dev_in_);
    ctx_->setTensorAddress(out_name_.c_str(), dev_out_);
  }

  void warmup()
  {
    cv::Mat blank(in_h_, in_w_, CV_8UC3, cv::Scalar(114, 114, 114));
    for (int i = 0; i < warmup_iters_; ++i) {
      std::vector<Detection> dets;
      runOnce(blank, dets);
    }
    RCLCPP_INFO(get_logger(), "warmup 完成（%d 次）", warmup_iters_);
  }

  // ───────────────────────── ROS 接口 ─────────────────────────
  void createRosInterfaces()
  {
    auto be = rclcpp::QoS(rclcpp::KeepLast(3)).best_effort();  // 深度 3：避免回调期间到达的帧被丢
    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      image_topic_, be,
      std::bind(&YoloInferCpp::onImage, this, std::placeholders::_1));

    auto info_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
    info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      camera_info_topic_, info_qos,
      std::bind(&YoloInferCpp::onCameraInfo, this, std::placeholders::_1));

    det_pub_ = create_publisher<usv_interfaces::msg::VisionDetectionArray>(detections_topic_, 10);
    if (publish_legacy_) {
      obj_pub_ = create_publisher<usv_interfaces::msg::AIVisionTargets>(objects_topic_, 10);
    }
    status_pub_ = create_publisher<std_msgs::msg::String>(status_topic_, 10);
    if (publish_annotated_) {
      ann_pub_ = create_publisher<sensor_msgs::msg::Image>(annotated_topic_, 10);
    }

    const auto period = std::chrono::duration<double>(1.0 / std::max(0.1, startup_report_hz_));
    status_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&YoloInferCpp::publishStatus, this));
  }

  void onCameraInfo(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
  {
    latest_info_ = msg;
  }

  void onImage(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    recv_count_++;
    const auto t0 = std::chrono::steady_clock::now();
    cv_bridge::CvImageConstPtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_ERROR(get_logger(), "cv_bridge: %s", e.what());
      return;
    }
    const auto t1 = std::chrono::steady_clock::now();

    std::vector<Detection> dets;
    double pre_ms = 0.0, inf_ms = 0.0, post_ms = 0.0;
    runOnce(cv_ptr->image, dets, &pre_ms, &inf_ms, &post_ms);
    const auto t2 = std::chrono::steady_clock::now();

    publishResults(msg, cv_ptr->image, dets);

    const auto t3 = std::chrono::steady_clock::now();
    const double decode_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    const double publish_ms = std::chrono::duration<double, std::milli>(t3 - t2).count();
    const double total_ms = decode_ms + pre_ms + inf_ms + post_ms + publish_ms;
    acc_total_ += total_ms; acc_infer_ += inf_ms; acc_pre_ += pre_ms; acc_post_ += post_ms; acc_count_++;
    frame_count_++;

    const auto now = std::chrono::steady_clock::now();
    const double dt = std::chrono::duration<double>(now - last_log_).count();
    if (dt >= 1.0 / std::max(0.05, timing_log_hz_)) {
      const double n = std::max(1.0, static_cast<double>(acc_count_));
      RCLCPP_INFO(get_logger(),
        "[#%lu] recv=%.1fHz decode=%.1fms pre=%.1fms infer=%.1fms post=%.1fms total=%.1fms (%.0ffps) dets=%zu",
        static_cast<unsigned long>(frame_count_),
        static_cast<double>(recv_count_) / std::max(0.2, dt),
        decode_ms, acc_pre_ / n, acc_infer_ / n, acc_post_ / n, acc_total_ / n,
        1000.0 / std::max(1e-3, acc_total_ / n), dets.size());
      acc_total_ = acc_infer_ = acc_pre_ = acc_post_ = 0.0; acc_count_ = 0;
      recv_count_ = 0; last_log_ = now;
    }
  }

  // ───────────── 推理主流程（preprocess → infer → postprocess） ─────────────
  void runOnce(const cv::Mat & bgr, std::vector<Detection> & dets,
    double * pre_ms = nullptr, double * inf_ms = nullptr, double * post_ms = nullptr)
  {
    const auto tp0 = std::chrono::steady_clock::now();
    // 1) letterbox 到 in_w × in_h，灰度 114 填充；BGR→RGB、/255、NCHW 由 blobFromImage 完成
    const float scale = std::min(
      static_cast<float>(in_w_) / bgr.cols, static_cast<float>(in_h_) / bgr.rows);
    const int nw = static_cast<int>(std::round(bgr.cols * scale));
    const int nh = static_cast<int>(std::round(bgr.rows * scale));
    const int dx = (in_w_ - nw) / 2;
    const int dy = (in_h_ - nh) / 2;
    // canvas / blob 都复用，避免每帧重新分配 + 二次 memcpy
    canvas_.setTo(cv::Scalar(114, 114, 114));
    if (nw == bgr.cols && nh == bgr.rows) {
      bgr.copyTo(canvas_(cv::Rect(dx, dy, nw, nh)));
    } else {
      cv::resize(bgr, resized_, cv::Size(nw, nh));
      resized_.copyTo(canvas_(cv::Rect(dx, dy, nw, nh)));
    }
    cv::dnn::blobFromImage(canvas_, blob_, 1.0 / 255.0, cv::Size(in_w_, in_h_),
      cv::Scalar(), true, false, CV_32F);
    const auto tp1 = std::chrono::steady_clock::now();

    // 2) H2D → enqueue → D2H
    CUDA_CHECK(cudaMemcpyAsync(dev_in_, blob_.ptr<float>(0), host_in_.size() * sizeof(float),
      cudaMemcpyHostToDevice, stream_));
    if (!ctx_->enqueueV3(stream_)) {
      RCLCPP_ERROR(get_logger(), "enqueueV3 失败");
      return;
    }
    CUDA_CHECK(cudaMemcpyAsync(host_out_.data(), dev_out_, host_out_.size() * sizeof(float),
      cudaMemcpyDeviceToHost, stream_));
    CUDA_CHECK(cudaStreamSynchronize(stream_));
    const auto tp2 = std::chrono::steady_clock::now();

    // 3) 解码 + 阈值 + NMS
    dets.clear();
    const float * out = host_out_.data();
    // output0: [1, 4+nc, N]，通道优先
    std::vector<Detection> cand;
    // 逐 channel 连续扫描（每路 8400 个连续 float），只保留 allowlist 里的类别
    best_score_.assign(out_n_, 0.0f);
    best_cls_.assign(out_n_, -1);
    for (int c : allowed_ids_) {
      const float * p = out + static_cast<size_t>(4 + c) * out_n_;
      for (int i = 0; i < out_n_; ++i) {
        if (p[i] > best_score_[i]) {best_score_[i] = p[i]; best_cls_[i] = c;}
      }
    }
    for (int i = 0; i < out_n_; ++i) {
      const float best_score = best_score_[i];
      const int best_cls = best_cls_[i];
      if (best_score < conf_threshold_ || best_cls < 0) {continue;}
      const float cx = out[0 * out_n_ + i];
      const float cy = out[1 * out_n_ + i];
      const float w = out[2 * out_n_ + i];
      const float h = out[3 * out_n_ + i];
      Detection d;
      d.x1 = (cx - w * 0.5f - dx) / scale;
      d.y1 = (cy - h * 0.5f - dy) / scale;
      d.x2 = (cx + w * 0.5f - dx) / scale;
      d.y2 = (cy + h * 0.5f - dy) / scale;
      d.x1 = std::max(0.0f, std::min(d.x1, static_cast<float>(bgr.cols - 1)));
      d.y1 = std::max(0.0f, std::min(d.y1, static_cast<float>(bgr.rows - 1)));
      d.x2 = std::max(0.0f, std::min(d.x2, static_cast<float>(bgr.cols - 1)));
      d.y2 = std::max(0.0f, std::min(d.y2, static_cast<float>(bgr.rows - 1)));
      if (d.x2 - d.x1 < 1.0f || d.y2 - d.y1 < 1.0f) {continue;}
      d.score = best_score;
      d.cls = best_cls;
      cand.push_back(d);
    }

    std::sort(cand.begin(), cand.end(),
      [](const Detection & a, const Detection & b) {return a.score > b.score;});
    std::vector<bool> removed(cand.size(), false);
    for (size_t i = 0; i < cand.size() && static_cast<int>(dets.size()) < max_det_; ++i) {
      if (removed[i]) {continue;}
      dets.push_back(cand[i]);
      for (size_t j = i + 1; j < cand.size(); ++j) {
        if (removed[j]) {continue;}
        if (iou(cand[i], cand[j]) > iou_threshold_) {removed[j] = true;}
      }
    }
    if (pre_ms) {*pre_ms = std::chrono::duration<double, std::milli>(tp1 - tp0).count();}
    if (inf_ms) {*inf_ms = std::chrono::duration<double, std::milli>(tp2 - tp1).count();}
    if (post_ms) {*post_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - tp2).count();}
  }

  std::string className(int cls) const
  {
    if (cls >= 0 && cls < static_cast<int>(class_names_.size())) {return class_names_[cls];}
    return std::to_string(cls);
  }

  // 像素 → 水平方位角（与 Python 版同款：pinhole 或 scaramuzza 鱼眼）
  double azimuthOf(double u, double v) const
  {
    if (!latest_info_ || latest_info_->k.size() < 6 || latest_info_->k[0] <= 0.0) {
      return std::numeric_limits<double>::quiet_NaN();
    }
    const double a0 = latest_info_->k[0];
    const double cx = latest_info_->k[2];
    const double cy = latest_info_->k[5];
    const double du = u - cx;
    const double dv = v - cy;
    if (angle_model_ == "scaramuzza" && latest_info_->d.size() >= 4) {
      const double a1 = latest_info_->d[0];
      const double a2 = latest_info_->d[1];
      const double a3 = latest_info_->d[2];
      const double a4 = latest_info_->d[3];
      const double rho = std::hypot(du, dv);
      const double f = a0 + a1 * rho + a2 * rho * rho + a3 * rho * rho * rho +
        a4 * rho * rho * rho * rho;
      if (f > 1e-6) {return std::atan2(du, f);}
    }
    return std::atan(du / a0);
  }

  void publishResults(const sensor_msgs::msg::Image::SharedPtr & msg,
    const cv::Mat & bgr, const std::vector<Detection> & dets)
  {
    const double sx = static_cast<double>(ref_w_) / std::max(1, bgr.cols);
    const double sy = static_cast<double>(ref_h_) / std::max(1, bgr.rows);

    usv_interfaces::msg::VisionDetectionArray det_msg;
    det_msg.header.stamp = msg->header.stamp;
    det_msg.header.frame_id = camera_id_;
    det_msg.output_stamp = now();

    usv_interfaces::msg::AIVisionTargets legacy;
    legacy.header = det_msg.header;

    for (const auto & d : dets) {
      const double u = (d.x1 + d.x2) * 0.5;
      const double v = (d.y1 + d.y2) * 0.5;
      const double az = azimuthOf(u, v);

      usv_interfaces::msg::VisionDetection det;
      det.camera_id = camera_id_;
      det.class_name = className(d.cls);
      det.class_id = d.cls;
      det.azimuth = std::isnan(az) ? 0.0 : std::round(az * 1e4) / 1e4;
      det.distance_predict = std::numeric_limits<double>::quiet_NaN();
      det.size_w = std::numeric_limits<double>::quiet_NaN();
      det.size_h = std::numeric_limits<double>::quiet_NaN();
      det.pixel_width = std::round((d.x2 - d.x1) * sx);
      det.confidence = std::round(d.score * 100.0) / 100.0;
      det_msg.detections.push_back(det);

      if (publish_legacy_) {
        usv_interfaces::msg::AIVisionTarget t;
        t.class_name = className(d.cls);
        t.confidence = static_cast<float>(std::round(d.score * 100.0) / 100.0);
        t.bbox_x = static_cast<int32_t>(std::round(d.x1 * sx));
        t.bbox_y = static_cast<int32_t>(std::round(d.y1 * sy));
        t.bbox_width = static_cast<int32_t>(std::round((d.x2 - d.x1) * sx));
        t.bbox_height = static_cast<int32_t>(std::round((d.y2 - d.y1) * sy));
        t.rel_ang_deg = static_cast<float>(std::round((std::isnan(az) ? 0.0 : az) * 180.0 / M_PI * 100.0) / 100.0);
        legacy.targets.push_back(t);
      }
    }
    det_pub_->publish(det_msg);
    if (publish_legacy_) {
      legacy.targets_count = static_cast<uint32_t>(legacy.targets.size());
      usv_interfaces::msg::PipelineTimestamp ts;
      ts.name = "inference_publish";
      ts.time_ms = static_cast<uint64_t>(now().nanoseconds() / 1000000);
      legacy.timestamps.push_back(ts);
      obj_pub_->publish(legacy);
    }

    if (ann_pub_ && publish_annotated_) {
      cv::Mat canvas = bgr.clone();
      for (const auto & d : dets) {
        cv::rectangle(canvas, cv::Point(static_cast<int>(d.x1), static_cast<int>(d.y1)),
          cv::Point(static_cast<int>(d.x2), static_cast<int>(d.y2)), cv::Scalar(0, 200, 0), 2);
        std::ostringstream os;
        os << className(d.cls) << " " << std::fixed << std::setprecision(2) << d.score;
        cv::putText(canvas, os.str(), cv::Point(static_cast<int>(d.x1), static_cast<int>(d.y1) - 4),
          cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 200, 0), 1);
      }
      std_msgs::msg::Header h = msg->header;
      auto out = cv_bridge::CvImage(h, "bgr8", canvas).toImageMsg();
      ann_pub_->publish(*out);
    }
  }

  void publishStatus()
  {
    std::ostringstream os;
    os << "{\"state\":\"" << (engine_ ? "running" : "error") << "\""
       << ",\"sensor_name\":\"" << sensor_name_ << "\""
       << ",\"camera_id\":\"" << camera_id_ << "\""
       << ",\"engine_path\":\"" << engine_path_ << "\""
       << ",\"infer_size\":\"" << in_w_ << "x" << in_h_ << "\""
       << ",\"num_classes\":" << num_classes_
       << ",\"angle_model\":\"" << angle_model_ << "\""
       << ",\"frames\":" << frame_count_
       << ",\"impl\":\"cpp_tensorrt\"}";
    std_msgs::msg::String msg;
    msg.data = os.str();
    status_pub_->publish(msg);
  }

  // ───────────── 成员 ─────────────
  TrtLogger logger_;
  nvinfer1::IRuntime * runtime_ = nullptr;
  nvinfer1::ICudaEngine * engine_ = nullptr;
  nvinfer1::IExecutionContext * ctx_ = nullptr;
  void * dev_in_ = nullptr;
  void * dev_out_ = nullptr;
  cudaStream_t stream_ = nullptr;
  std::vector<float> host_in_;
  std::vector<float> host_out_;
  std::vector<int> allowed_ids_;
  std::vector<float> best_score_;
  std::vector<int> best_cls_;
  cv::Mat canvas_, resized_, blob_;
  std::string in_name_, out_name_;
  int in_c_ = 3, in_w_ = 640, in_h_ = 640;
  int out_c_ = 84, out_n_ = 8400;
  int num_classes_ = 80;

  std::string sensor_name_, camera_id_;
  std::string image_topic_, camera_info_topic_;
  std::string detections_topic_, objects_topic_, status_topic_, annotated_topic_;
  std::string engine_path_;
  std::vector<std::string> class_names_, allowlist_;
  float conf_threshold_ = 0.4f, iou_threshold_ = 0.55f;
  int max_det_ = 20, ref_w_ = 1920, ref_h_ = 1536;
  std::string angle_model_ = "pinhole";
  bool publish_annotated_ = false, publish_legacy_ = true;
  double timing_log_hz_ = 1.0, startup_report_hz_ = 0.5;
  int warmup_iters_ = 5;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
  rclcpp::Publisher<usv_interfaces::msg::VisionDetectionArray>::SharedPtr det_pub_;
  rclcpp::Publisher<usv_interfaces::msg::AIVisionTargets>::SharedPtr obj_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr ann_pub_;
  rclcpp::TimerBase::SharedPtr status_timer_;
  sensor_msgs::msg::CameraInfo::SharedPtr latest_info_;

  uint64_t frame_count_ = 0;
  uint64_t recv_count_ = 0;
  double acc_total_ = 0.0, acc_infer_ = 0.0, acc_pre_ = 0.0, acc_post_ = 0.0;
  int acc_count_ = 0;
  std::chrono::steady_clock::time_point last_log_ = std::chrono::steady_clock::now();
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<YoloInferCpp>();
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    std::fprintf(stderr, "fatal: %s\n", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}

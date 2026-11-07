// 实现：appsrc(BGRx) → queue(leaky) → nvvidconv(NV12@NVMM) → nvv4l2h264enc → h264parse → rtspclientsink
#include "usv_vision_gmsl_cpp/h264_streamer.hpp"

#include <gst/app/gstappsrc.h>
#include <gst/gst.h>

#include <chrono>
#include <cstring>
#include <memory>
#include <sstream>

namespace usv_vision_gmsl_cpp
{
namespace
{
using Clock = std::chrono::steady_clock;
using FrameBuf = std::shared_ptr<std::vector<uint8_t>>;

void frame_unref(gpointer data) { delete static_cast<FrameBuf *>(data); }

std::string build_pipeline_desc(const H264StreamerOptions & o)
{
  std::ostringstream s;
  s << "appsrc name=src is-live=true block=false format=time do-timestamp=true "
    << "caps=video/x-raw,format=BGRx,width=" << o.width << ",height=" << o.height
    << ",framerate=" << o.fps << "/1 ! "
    << "queue name=q1 max-size-buffers=6 max-size-bytes=33554432 "
    << "max-size-time=200000000 leaky=downstream ! "
    << "nvvidconv ! "
    << "video/x-raw(memory:NVMM),format=NV12,width=" << o.width << ",height=" << o.height << " ! "
    << "queue name=q2 max-size-buffers=5 max-size-bytes=26214400 "
    << "max-size-time=200000000 leaky=downstream ! "
    << "nvv4l2h264enc bitrate=" << o.bitrate << " control-rate=" << o.control_rate
    << " iframeinterval=" << o.iframe_interval << " idrinterval=" << o.idr_interval
    << " insert-sps-pps=" << (o.insert_sps_pps ? 1 : 0)
    << " preset-level=" << o.preset_level << " ! "
    << "h264parse config-interval=-1 ! "
    << "rtspclientsink location=" << o.rtsp_url << " latency=500 protocols=tcp";
  return s.str();
}
}  // namespace

H264Streamer::~H264Streamer() { stop(); }

std::string H264Streamer::last_error() const
{
  std::lock_guard<std::mutex> lock(mtx_);
  return last_error_;
}

bool H264Streamer::start(const H264StreamerOptions & opts, std::string * err)
{
  if (running_.load()) {
    if (err != nullptr) {*err = "already running";}
    return false;
  }
  if (opts.width <= 0 || opts.height <= 0 || opts.rtsp_url.empty()) {
    if (err != nullptr) {*err = "invalid options (width/height/rtsp_url)";}
    return false;
  }
  opts_ = opts;
  if (opts_.queue_depth == 0) {opts_.queue_depth = 1;}
  stop_requested_ = false;
  {
    std::lock_guard<std::mutex> lock(mtx_);
    queue_.clear();
  }
  running_.store(true);
  worker_ = std::thread([this]() {this->worker_loop();});
  return true;
}

void H264Streamer::stop()
{
  if (!running_.exchange(false)) {
    if (worker_.joinable()) {worker_.join();}
    return;
  }
  {
    std::lock_guard<std::mutex> lock(mtx_);
    stop_requested_ = true;
    queue_.clear();
  }
  cv_.notify_all();
  if (worker_.joinable()) {worker_.join();}
  healthy_.store(false);
}

void H264Streamer::push(const cv::Mat & frame)
{
  if (!running_.load()) {dropped_.fetch_add(1); return;}
  if (frame.empty() || frame.cols != opts_.width || frame.rows != opts_.height) {
    dropped_.fetch_add(1);
    return;
  }
  // 深拷贝一次：SDK 的帧缓冲会被下一次采集覆盖，必须自己持有
  FrameBuf buf = std::make_shared<std::vector<uint8_t>>(
    static_cast<size_t>(frame.total() * frame.elemSize()));
  if (frame.isContinuous()) {
    std::memcpy(buf->data(), frame.data, buf->size());
  } else {
    for (int r = 0; r < frame.rows; ++r) {
      std::memcpy(buf->data() + static_cast<size_t>(r) * frame.cols * frame.elemSize(),
                  frame.ptr(r), static_cast<size_t>(frame.cols) * frame.elemSize());
    }
  }

  {
    std::lock_guard<std::mutex> lock(mtx_);
    while (queue_.size() >= opts_.queue_depth) {
      queue_.erase(queue_.begin());   // 丢最旧帧：只有最新画面重要
      dropped_.fetch_add(1);
    }
    queue_.push_back(std::move(buf));
  }
  cv_.notify_one();
}

void H264Streamer::worker_loop()
{
  gst_init(nullptr, nullptr);
  bool pipeline_up = false;
  Clock::time_point retry_at = Clock::now();

  while (running_.load()) {
    if (!pipeline_up) {
      if (Clock::now() < retry_at) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        continue;
      }
      std::string err;
      if (build_pipeline(&err)) {
        pipeline_up = true;
        healthy_.store(true);
      } else {
        {
          std::lock_guard<std::mutex> lock(mtx_);
          last_error_ = err;
        }
        healthy_.store(false);
        retry_at = Clock::now() + std::chrono::milliseconds(
          static_cast<int>(opts_.reconnect_delay_sec * 1000));
        continue;
      }
    }

    // 取一帧（最多等 100ms，便于及时响应 stop / bus 错误）
    FrameBuf frame;
    {
      std::unique_lock<std::mutex> lock(mtx_);
      cv_.wait_for(lock, std::chrono::milliseconds(100), [this]() {
        return stop_requested_ || !queue_.empty();
      });
      if (stop_requested_) {break;}
      if (queue_.empty()) {
        // 没帧也要看看总线
      } else {
        frame = std::move(queue_.front());
        queue_.erase(queue_.begin());
      }
    }

    if (poll_bus_error()) {
      teardown_pipeline();
      pipeline_up = false;
      healthy_.store(false);
      retry_at = Clock::now() + std::chrono::milliseconds(
        static_cast<int>(opts_.reconnect_delay_sec * 1000));
      continue;
    }

    if (frame) {
      FrameBuf * holder = new FrameBuf(frame);
      GstBuffer * gbuf = gst_buffer_new_wrapped_full(
        GST_MEMORY_FLAG_READONLY, holder->get()->data(), holder->get()->size(),
        0, holder->get()->size(), holder, frame_unref);
      GstFlowReturn ret = GST_FLOW_OK;
      {
        std::lock_guard<std::mutex> lock(mtx_);
        auto * appsrc = static_cast<GstElement *>(appsrc_);
        if (appsrc != nullptr) {
          ret = gst_app_src_push_buffer(GST_APP_SRC(appsrc), gbuf);
        } else {
          gst_buffer_unref(gbuf);
          ret = GST_FLOW_FLUSHING;
        }
      }
      if (ret != GST_FLOW_OK) {
        dropped_.fetch_add(1);
        teardown_pipeline();
        pipeline_up = false;
        healthy_.store(false);
        retry_at = Clock::now() + std::chrono::milliseconds(
          static_cast<int>(opts_.reconnect_delay_sec * 1000));
      } else {
        pushed_.fetch_add(1);
      }
    }
  }

  teardown_pipeline();
}

bool H264Streamer::build_pipeline(std::string * err)
{
  const std::string desc = build_pipeline_desc(opts_);
  GError * gerr = nullptr;
  GstElement * pipe = gst_parse_launch(desc.c_str(), &gerr);
  if (pipe == nullptr || gerr != nullptr) {
    if (err != nullptr) {
      *err = gerr != nullptr ? gerr->message : "gst_parse_launch failed";
    }
    if (gerr != nullptr) {g_error_free(gerr);}
    if (pipe != nullptr) {gst_object_unref(pipe);}
    return false;
  }
  GstElement * src = gst_bin_get_by_name(GST_BIN(pipe), "src");
  if (src == nullptr) {
    if (err != nullptr) {*err = "appsrc not found in pipeline";}
    gst_object_unref(pipe);
    return false;
  }
  gst_element_set_state(pipe, GST_STATE_PLAYING);
  pipeline_ = pipe;
  appsrc_ = src;
  return true;
}

bool H264Streamer::poll_bus_error()
{
  auto * pipe = static_cast<GstElement *>(pipeline_);
  if (pipe == nullptr) {return false;}
  GstBus * bus = gst_element_get_bus(pipe);
  if (bus == nullptr) {return false;}
  GstMessage * msg = gst_bus_pop_filtered(
    bus, static_cast<GstMessageType>(GST_MESSAGE_ERROR | GST_MESSAGE_EOS));
  gst_object_unref(bus);
  if (msg == nullptr) {return false;}

  bool has_error = GST_MESSAGE_TYPE(msg) == GST_MESSAGE_ERROR;
  if (has_error) {
    GError * gerr = nullptr;
    gchar * dbg = nullptr;
    gst_message_parse_error(msg, &gerr, &dbg);
    std::lock_guard<std::mutex> lock(mtx_);
    last_error_ = gerr != nullptr ? gerr->message : "gstreamer error";
    if (gerr != nullptr) {g_error_free(gerr);}
    if (dbg != nullptr) {g_free(dbg);}
  }
  gst_message_unref(msg);
  return has_error;
}

void H264Streamer::teardown_pipeline()
{
  auto * pipe = static_cast<GstElement *>(pipeline_);
  auto * src = static_cast<GstElement *>(appsrc_);
  pipeline_ = nullptr;
  appsrc_ = nullptr;
  if (src != nullptr) {gst_object_unref(src);}
  if (pipe != nullptr) {
    gst_element_set_state(pipe, GST_STATE_NULL);
    gst_object_unref(pipe);
  }
}

}  // namespace usv_vision_gmsl_cpp

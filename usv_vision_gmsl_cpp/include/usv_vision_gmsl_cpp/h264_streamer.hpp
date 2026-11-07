// H.264 推流：在 C++ 采集节点内直接把帧送进硬件编码器，绕过 DDS 与 Python。
// 设计要点：
//  - 采集线程只做一次浅层拷贝到内部队列，绝不阻塞在编码上；
//  - 队列满时丢最旧帧（leaky），保证"总能推出最新画面"；
//  - GStreamer 只在推流线程里被调用；
//  - bus 监听 ERROR/EOS，按 reconnect_delay_sec 重建管线。
#pragma once

#include <opencv2/core.hpp>

#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <mutex>
#include <string>
#include <thread>
#include <cstdint>
#include <memory>
#include <vector>

namespace usv_vision_gmsl_cpp
{

struct H264StreamerOptions
{
  std::string rtsp_url;
  int width = 0;
  int height = 0;
  int fps = 30;
  int bitrate = 4000000;
  int iframe_interval = 15;
  int idr_interval = 30;
  int control_rate = 1;      // 1 = CBR
  int preset_level = 1;
  bool insert_sps_pps = true;
  double reconnect_delay_sec = 3.0;
  size_t queue_depth = 3;    // 队列里最多留几帧（超过丢旧帧）
};

class H264Streamer
{
public:
  H264Streamer() = default;
  ~H264Streamer();

  H264Streamer(const H264Streamer &) = delete;
  H264Streamer & operator=(const H264Streamer &) = delete;

  /// 启动推流线程。失败返回 false 并写入 err。
  bool start(const H264StreamerOptions & opts, std::string * err = nullptr);

  /// 停止并回收（幂等）。
  void stop();

  /// 非阻塞推入一帧（BGRx/BGRA 均可，第 4 字节会被 nvvidconv 忽略）。
  /// 帧必须与 opts.width/height 一致，否则丢弃。
  void push(const cv::Mat & frame);

  bool healthy() const { return healthy_.load(); }
  /// 累计丢弃帧数（队满 + 尺寸不符 + 管线未就绪）
  uint64_t dropped() const { return dropped_.load(); }
  uint64_t pushed() const { return pushed_.load(); }
  std::string last_error() const;

private:
  void worker_loop();
  bool build_pipeline(std::string * err);
  void teardown_pipeline();
  bool poll_bus_error();

  H264StreamerOptions opts_;
  std::thread worker_;
  std::atomic<bool> running_{false};
  std::atomic<bool> healthy_{false};
  std::atomic<uint64_t> dropped_{0};
  std::atomic<uint64_t> pushed_{0};

  mutable std::mutex mtx_;
  std::condition_variable cv_;
  using FramePtr = std::shared_ptr<std::vector<uint8_t>>;
  std::vector<FramePtr> queue_;   // 最多 opts_.queue_depth 帧，超出丢旧帧
  bool stop_requested_ = false;
  std::string last_error_;

  // GStreamer 句柄（仅在推流线程内使用）
  void * pipeline_ = nullptr;
  void * appsrc_ = nullptr;
};

}  // namespace usv_vision_gmsl_cpp

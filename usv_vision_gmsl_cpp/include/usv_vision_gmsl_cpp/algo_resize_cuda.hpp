// GPU 端融合缩放：BGRA(全分辨率) → BGR(算法尺寸)，一次核函数完成转色 + 双线性降采样。
// 设计：设备缓冲懒分配并复用；上传/下载走独立 cudaStream；失败时返回 false 由调用方回退 CPU。
#pragma once

#include <opencv2/core.hpp>

namespace usv_vision_gmsl_cpp
{

/// 把 BGRA 源图融合转换为 BGR 目标图（双线性）。返回 false 表示 CUDA 不可用或失败。
bool gpu_resize_bgra_to_bgr(
  const cv::Mat & src_bgra, cv::Mat & dst_bgr, int out_width, int out_height);

/// 释放设备缓冲（析构时调用；非必须）。
void gpu_resize_release();

/// CUDA 是否可用（编译期支持 + 运行期设备存在）。
bool gpu_resize_available();

}  // namespace usv_vision_gmsl_cpp

#include "usv_vision_gmsl_cpp/algo_resize_cuda.hpp"

#include <cuda_runtime.h>

#include <mutex>

namespace usv_vision_gmsl_cpp
{
namespace
{
std::mutex g_mtx;
uchar4 * g_src_dev = nullptr;   // 输入：BGRA，按像素存为 uchar4
size_t g_src_cap = 0;           // 容量（像素个数）
uchar * g_dst_dev = nullptr;    // 输出：BGR 紧凑缓冲
size_t g_dst_cap = 0;
cudaStream_t g_stream = nullptr;

__global__ void bgra_to_bgr_bilinear(
  const uchar4 * __restrict__ src, int sw, int sh,
  uchar * __restrict__ dst, int dw, int dh)
{
  const int x = blockIdx.x * blockDim.x + threadIdx.x;
  const int y = blockIdx.y * blockDim.y + threadIdx.y;
  if (x >= dw || y >= dh) {return;}

  // 以像素中心对齐的双线性采样
  const float fx = (x + 0.5f) * (static_cast<float>(sw) / dw) - 0.5f;
  const float fy = (y + 0.5f) * (static_cast<float>(sh) / dh) - 0.5f;
  const int x0 = max(0, min(sw - 1, static_cast<int>(floorf(fx))));
  const int y0 = max(0, min(sh - 1, static_cast<int>(floorf(fy))));
  const int x1 = min(sw - 1, x0 + 1);
  const int y1 = min(sh - 1, y0 + 1);
  const float ax = fminf(fmaxf(fx - x0, 0.f), 1.f);
  const float ay = fminf(fmaxf(fy - y0, 0.f), 1.f);

  const uchar4 p00 = src[y0 * sw + x0];
  const uchar4 p10 = src[y0 * sw + x1];
  const uchar4 p01 = src[y1 * sw + x0];
  const uchar4 p11 = src[y1 * sw + x1];

  const float w00 = (1.f - ax) * (1.f - ay);
  const float w10 = ax * (1.f - ay);
  const float w01 = (1.f - ax) * ay;
  const float w11 = ax * ay;

  uchar3 out;
  out.x = static_cast<uchar>(p00.x * w00 + p10.x * w10 + p01.x * w01 + p11.x * w11 + 0.5f);
  out.y = static_cast<uchar>(p00.y * w00 + p10.y * w10 + p01.y * w01 + p11.y * w11 + 0.5f);
  out.z = static_cast<uchar>(p00.z * w00 + p10.z * w10 + p01.z * w01 + p11.z * w11 + 0.5f);

  const size_t o = (static_cast<size_t>(y) * dw + x) * 3;
  dst[o + 0] = out.x;
  dst[o + 1] = out.y;
  dst[o + 2] = out.z;
}

bool reserve(size_t src_px, size_t dst_bytes)
{
  if (src_px > g_src_cap) {
    if (g_src_dev != nullptr) {cudaFree(g_src_dev); g_src_dev = nullptr;}
    if (cudaMalloc(&g_src_dev, src_px * sizeof(uchar4)) != cudaSuccess) {
      g_src_cap = 0;
      return false;
    }
    g_src_cap = src_px;
  }
  if (dst_bytes > g_dst_cap) {
    if (g_dst_dev != nullptr) {cudaFree(g_dst_dev); g_dst_dev = nullptr;}
    if (cudaMalloc(&g_dst_dev, dst_bytes) != cudaSuccess) {
      g_dst_cap = 0;
      return false;
    }
    g_dst_cap = dst_bytes;
  }
  if (g_stream == nullptr && cudaStreamCreate(&g_stream) != cudaSuccess) {return false;}
  return true;
}
}  // namespace

bool gpu_resize_available()
{
  static int cached = -1;
  if (cached < 0) {
    int count = 0;
    cached = (cudaGetDeviceCount(&count) == cudaSuccess && count > 0) ? 1 : 0;
  }
  return cached == 1;
}

bool gpu_resize_bgra_to_bgr(
  const cv::Mat & src_bgra, cv::Mat & dst_bgr, int out_width, int out_height)
{
  if (!gpu_resize_available()) {return false;}
  if (src_bgra.empty() || src_bgra.channels() != 4 || src_bgra.type() != CV_8UC4) {return false;}
  if (out_width <= 0 || out_height <= 0) {return false;}

  const int sw = src_bgra.cols;
  const int sh = src_bgra.rows;
  dst_bgr.create(out_height, out_width, CV_8UC3);

  std::lock_guard<std::mutex> lock(g_mtx);
  if (!reserve(static_cast<size_t>(sw) * sh,
               static_cast<size_t>(out_width) * out_height * 3)) {
    return false;
  }

  const size_t src_bytes = static_cast<size_t>(sw) * sh * 4;
  if (cudaMemcpyAsync(g_src_dev, src_bgra.data, src_bytes,
                      cudaMemcpyHostToDevice, g_stream) != cudaSuccess) {
    return false;
  }

  const dim3 block(16, 16);
  const dim3 grid((out_width + block.x - 1) / block.x, (out_height + block.y - 1) / block.y);
  bgra_to_bgr_bilinear<<<grid, block, 0, g_stream>>>(
    g_src_dev, sw, sh, g_dst_dev, out_width, out_height);

  const size_t dst_bytes = static_cast<size_t>(out_width) * out_height * 3;
  if (cudaMemcpyAsync(dst_bgr.data, g_dst_dev, dst_bytes,
                      cudaMemcpyDeviceToHost, g_stream) != cudaSuccess) {
    return false;
  }
  if (cudaStreamSynchronize(g_stream) != cudaSuccess) {return false;}
  return true;
}

void gpu_resize_release()
{
  std::lock_guard<std::mutex> lock(g_mtx);
  if (g_src_dev != nullptr) {cudaFree(g_src_dev); g_src_dev = nullptr; g_src_cap = 0;}
  if (g_dst_dev != nullptr) {cudaFree(g_dst_dev); g_dst_dev = nullptr; g_dst_cap = 0;}
  if (g_stream != nullptr) {cudaStreamDestroy(g_stream); g_stream = nullptr;}
}

}  // namespace usv_vision_gmsl_cpp

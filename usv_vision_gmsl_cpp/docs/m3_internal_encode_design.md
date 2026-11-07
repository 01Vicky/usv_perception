# M3 设计方案：把 H.264 编码搬进 C++ 采集节点

> 目标：消除推流链路里 **DDS 传 8.8MB 全分辨率图 → Python 收包 → `bytes()` 拷贝 → videoconvert** 这条最贵的路径。
> 现状单路推流 0.69 核（其中 Python/DDS 侧约 0.42、GStreamer 管线 0.27）；三路合计 1.64 核。

## 1. 方案总览

```
现状：
  采集节点 ──image_raw 8.8MB──▶ DDS ──▶ Python rtsp_stream_node ──▶ appsrc → videoconvert → nvvidconv → enc → RTSP
                                        (DDS 读 + bytes 拷贝 + Python 编码)   ↑ 0.42 核/路        ↑ 0.27 核/路

改造后：
  采集节点 ──┬─image_raw 8.8MB─▶ DDS（保留，给其它订阅者）
             └─▶ 内部直接编码（同进程，零 DDS 跳转）──▶ appsrc(BGRx 零转换) → nvvidconv → nvv4l2h264enc → rtspclientsink
```

- 编码器与采集在**同一进程**，帧数据不经过 DDS，也不进 Python
- 用 `BGRx` 喂 `nvvidconv`（SDK 原生 BGRA 的第 4 字节被忽略），**彻底去掉 `videoconvert`**
- Python 的 `rtsp_stream_node` 保留为**可选回退**（默认关闭推流路径时行为不变）

## 2. 改动清单（只动两个包）

| # | 文件 | 改动 |
|---|---|---|
| 1 | `usv_vision_gmsl_cpp/src/h264_streamer.{hpp,cpp}`（**新建**） | 封装 GStreamer 管线：`appsrc → queue(leaky) → nvvidconv(BGRx→NV12) → nvv4l2h264enc → h264parse → rtspclientsink`；提供 `push(const cv::Mat& bgra) / is_healthy() / stats()` |
| 2 | `usv_vision_gmsl_cpp/src/gmsl_camera_node_cpp.cpp` | 参数：`enable_internal_stream`（默认 false）、`stream_rtsp_url`、`stream_bitrate`、`stream_iframe_interval`；在采集循环里把 BGRA 帧同时推给 streamer |
| 3 | `usv_vision_gmsl_cpp/src/gmsl_multi_camera_sync_node.cpp` | 同上（多相机各自一路） |
| 4 | `usv_vision_gmsl_cpp/CMakeLists.txt` | 链接 `gstreamer-1.0`、`gstreamer-app-1.0`、`gstreamer-rtsp-1.0` |
| 5 | `usv_vision/launch/jetson_gmsl_full.launch.py` | `streaming:=true` 时改为启动 C++ 内置推流（不再起 Python `rtsp_stream_node`）；保留 `legacy_streaming:=true` 走老路径 |
| 6 | `config/gmsl_camera_cpp_*.yaml` | 新增上述 streaming 参数（默认关，便于灰度） |
| 7 | `usv_vision_gmsl_cpp/README.md` | 说明两条推流路径与切换方式 |

## 3. 关键设计点

**a) 帧格式与零转换**
- SDK 出 BGRA → 直接 `Gst.Buffer` 包 `BGRx` caps（第 4 字节忽略，像素完全一致）
- `nvvidconv` 负责 BGRx→NV12 + 上传 NVMM，全部硬件

**b) 背压与实时性（重点）**
- `appsrc` 的 `block=false` + `queue leaky=downstream`（丢旧帧保最新），与现有 Python 版策略一致
- 编码不阻塞采集线程：**独立推流线程 + 最新帧覆盖**（采集线程只做一次浅拷贝指针/引用计数，绝不等待编码）
- 风险控制：如果推流线程超过 N 帧未消费，直接丢帧，不影响采集与识别

**c) 断线自愈**
- 监听 GStreamer bus 的 `ERROR`/`EOS`，指数退避重建管线（沿用 Python 版的 3s 重连逻辑）
- 状态心跳：`/sensors/camera/{sensor}/stream_status`（复用现有 `VideoStreamStatus`，若 `usv_interfaces` 不可用则跳过）

**d) 与现有 QoS/分发逻辑的关系**
- `image_raw` 仍然照常发布（其它消费者不受影响），只是不再有人为推流去订阅它
- 若 `enable_internal_stream=true` 且没有其它 `image_raw` 订阅者，可按需跳过 raw 发布（参数 `raw_when_unused`，默认保留）

## 4. 验证计划

| 项 | 方法 | 通过标准 |
|---|---|---|
| 画质/分辨率 | `ffprobe` 拉流确认 1920×1536/h264；截图与 Python 版逐帧比对（PSNR>40dB） | 视觉一致 |
| 推流延迟 | 拍摄毫秒计时器，端到端抓帧比对 | 不差于现状（现状 RTSP latency=500） |
| CPU | 三路全开，`/proc` 增量 10s×3 次 | 推流 ≤0.25 核/路（现状 0.69） |
| 采集/识别不受影响 | `frame_timing_json` 出帧率、`detections` 频率 | 仍 30Hz / ≥27Hz |
| 断线自愈 | `pkill mediamtx` 后重启 | 3~10s 内自动恢复推流 |
| 回退 | `enable_internal_stream=false` + launch `legacy_streaming:=true` | 回到现状 |

## 5. 预期收益与代价

| 指标 | 现状 | 改造后 |
|---|---|---|
| 推流 CPU（3 路） | 1.64 核 | **约 0.4 核** |
| DDS 带宽 | 0.90 GB/s | **0.11 GB/s**（不再有 8.8MB×3 的推流订阅） |
| 视觉合计 | 4.09 核（整机 51%） | **约 2.85 核（整机 36%）** |
| 再叠加采集端 GPU resize | — | 约 2.5 核（整机 31%） |

代价：C++ 里多一条 GStreamer 生命周期管理（重连、线程、内存），约 **5~7 人日**；已用参数与 legacy 开关隔离风险。

## 6. 风险与对策

| 风险 | 对策 |
|---|---|
| C++ 里 GStreamer 崩溃拖垮采集 | 推流跑在独立线程 + bus 监听；异常只影响推流，采集/识别继续 |
| NVMM 内存泄漏（长时间跑） | queue 限长 + 定期统计 `nvvidconv` 输出缓冲；压测 2 小时看 RSS |
| RTSP 客户端兼容性变化 | 保留 Python 路径做 A/B 与回退 |
| 本机无法编译验证 | 全部编译/测试在板上进行；本机只做静态检查（ast/yaml/字段一致性） |

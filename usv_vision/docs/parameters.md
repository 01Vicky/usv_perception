# 参数参考手册（视觉链路 / 资源分配）

> 版本：2026-11-07（对应锁频基线：三路全开 视觉 2.90 核 / 整机 41.8%）
> 用途：所有"用 GPU 还是 CPU、开还是不开"的取舍都通过这些参数控制，便于整机资源分配。
> **默认值 = 实测最优**；除非有明确理由，不要改默认。

---

## 1. `usv_vision_gmsl_cpp/config/gmsl_camera_cpp_<name>.yaml`（每路相机一份）

| 参数 | 默认 | 说明 | 影响的资源 |
|---|---|---|---|
| `sensor_name` | front/right/left | 相机名，决定话题 `/sensors/camera/<name>/...` | — |
| `device_path` | 见文件 | V4L2 设备节点（front/right/left 对应板上的 videoN，**换机时必须核对**） | — |
| `enable_hardware_timestamp` | true | 用 SDK 硬件时间戳；失效时回退 ROS 时间 | — |
| `image_qos_reliability` | `reliable` | `image_raw` 的发布可靠性。reliable 让 rviz/录像零配置可用 | CPU（慢读者会重传） |
| `image_qos_depth` | 2 | `image_raw` 队列深度。8.8MB 大帧下 depth 越大只是白占内存 | 内存 |
| **`publish_only_when_subscribed`** | true | **触发式发布**：没有订阅者时 `image_raw` / `image_algo` / `camera_info` 完全不发（不拷贝、不序列化、不写共享内存）。三路实测采集节点合计 **0.71 → 0.45 核**；推流/录包/rviz 订阅后下一轮立即恢复发布。`false`=旧行为（总是发），仅排障用 | CPU | 
| `put_timestamp` | false | 在 `image_raw` / `image_algo` 左上角叠加该帧采集时间戳（`ms` 格式，对时/延迟排查用）。开启后每帧多一次 `putText`（约亚毫秒级），默认关 | CPU（开启时） |
| `enable_algo_stream` | true | 是否发布 `image_algo`（识别用小图）。关闭后识别不可用 | CPU / DDS 带宽 |
| `algo_width` / `algo_height` | 640 / 640 | 算法图尺寸。原始 1920×1536 是 5:4，保比例应配 **640×512** | CPU（resize） |
| **`algo_resize_backend`** | `cpu` | `cuda`=GPU 融合转色+缩放；`cpu`=OpenCV。**实测两者等同**（cuda 的显存流量吃掉收益），默认 cpu 少一层依赖；整机 CPU 紧张时可切 cuda 换 CPU 余量 | CPU ↔ GPU |
| **`algo_resize_threads`** | 1 | 仅 cpu 后端生效。OpenCV 多线程的 CPU 时间≈墙钟×线程数，1 最小 | CPU |
| `algo_qos_reliability` | `best_effort` | `image_algo` 发布可靠性。识别只要最新帧，best_effort 避免给慢读者重传 | CPU |
| `algo_qos_depth` | 1 | `image_algo` 队列深度 | 内存 |
| `algo_transport` / `algo_jpeg_quality` | `raw` / 95 | 算法图传输格式（`raw`=未压缩，`jpeg`=压缩）。**目前只暴露接口**：`jpeg` 为已实现但**暂不采用**（三路下解码成本 3.5ms/帧，越过 33.3ms 节拍）。由 launch 的 `algo_transport` 统一下发 | 带宽（预期） |
| **`enable_internal_stream`** | false | 是否启用 **C++ 内置推流**（采集进程内直接编码）。由 launch 的 `stream_mode` 统一下发，一般不要手改 | CPU（省一整条链路） |
| **`stream_rtsp_url`** | `""` | 推流地址。**空 = 由 launch 按相机顺序注入** `rtsp://127.0.0.1:8554/camN`；仅特殊场景手填 | — |
| `stream_bitrate` | 4000000 | 编码码率（bps） | 带宽 / GPU 编码器 |
| `stream_fps` | 30 | 编码帧率 | CPU / GPU |
| `stream_iframe_interval` / `stream_idr_interval` | 15 / 30 | 关键帧间隔（影响首帧与重连速度） | 带宽 |
| `stream_preset_level` | 1 | 编码器预设（越大越省码率、延迟越高） | GPU |
| `stream_control_rate` | 1 | 1=CBR 恒定码率，0=VBR | 带宽 |

---

## 2. `usv_vision/config/yolo_infer.yaml`（识别链）

| 参数 | 默认 | 说明 | 影响的资源 |
|---|---|---|---|
| `engine_path` | `model/jetson/yolo11n_fp16_640.engine` | TensorRT engine（**相对启动 cwd**，一般在 `~/usv_ws_test` 下启动） | — |
| `model_backend` | `tensorrt` | 推理后端 | GPU |
| `infer_width` / `infer_height` | 640 / 640 | engine 输入尺寸。**必须与 engine 一致**，否则 ultralytics 会再拉伸一次 | GPU |
| **`use_algo_image`** | true | true=订阅 `image_algo`（小图，推荐）；false=订阅 `image_raw`（旧形态，CPU 高） | CPU / DDS |
| **`reference_width` / `reference_height`** | 1920 / 1536 | 坐标基准。对外 bbox 换算回原始像素，下游无感 | — |
| `enable_mask` | **false** | 蒙板总开关。实测开启后识别 CPU +15.6pp、detections 26→22Hz，**默认必须关** | CPU |
| `mask_path` | `config/masks/front_mask.png` | 蒙板图（仅 enable_mask=true 时用） | — |
| `conf_threshold` / `iou_threshold` | 0.40 / 0.55 | 检测阈值 | — |
| `max_det` | 20 | 每帧最大检测数 | — |

---

## 3. `usv_vision/config/rtsp_stream.yaml`（**仅** `stream_mode:=python` 时生效）

| 参数 | 默认 | 说明 |
|---|---|---|
| `input_image_topic` | `""` | 空=按 `sensor_name` 推导（`image_raw`）；可指向 `image_algo` 推小图 |
| `width` / `height` | 1920 / 1536 | **必须与所订阅话题一致**，否则丢帧并告警 |
| `fps` / `bitrate` / `*_interval` | 30 / 4Mbps | 编码参数（与 C++ 内置推流含义相同） |
| `rtsp_url` | `rtsp://127.0.0.1:8554/cam1` | 推流地址（launch 会覆盖） |
| `reconnect_delay_sec` | 3.0 | 断线重连间隔 |

---

## 4. `usv_vision/launch/jetson_gmsl_full.launch.py`（启动参数）

| 参数 | 默认 | 说明 |
|---|---|---|
| `cameras` | `['front','rear']` | 相机列表，决定启几路 |
| `inference` | true | 是否每路起 yolo 节点 |
| **`stream_mode`** | `internal` | `internal`=C++ 内置推流（省 0.79 核/三路）；`python`=旧的独立 Python 推流节点；`off`=不推流 |
| `streaming` | `""` | **兼容旧用法**：非空时覆盖 `stream_mode`（`true`→python，`false`→off） |
| `algo_stream` | true | 透传给 `enable_algo_stream`，同时决定 yolo 的 `use_algo_image` |

---

## 5. 资源分配速查（锁频基线，三路全开）

| 配置 | 视觉 CPU | 整机 | 备注 |
|---|---|---|---|
| 仅采集 | 0.65 核 | 11.4% | 底线 |
| 采集+推理（无推流） | 2.19 核 | 31.1% | `stream_mode:=off` |
| **全开 internal** | **2.90 核** | **41.8%** | **默认** |
| 全开 python | 3.69 核 | 51.8% | 旧形态，仅回退用 |

**已验证"调了也没用"的开关**（别再折腾）：`algo_resize_backend`（cuda vs cpu 差 0.01 核）、`algo_qos_reliability`（0）、`algo_width/height`（0）、`infer_width/height`（0）、推理降频（每路 −0.08 核）。

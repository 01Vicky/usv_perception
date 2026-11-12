> ⚠️ **本文件中的吞吐数字已作废**（+44%/+57% 来自启动预热期取样）。权威数据见 `docs/image_link_ab_results.md` 第九节（统一口径实测）。

# 板上 A/B/C 对照实测记录（2026-11-07）

> 目的：在真实 Jetson 板上量出「采集端按需分发 + 识别端改订小图」这条改造**到底有没有收益、收益是什么**。
> 结论先行：**总 CPU 基本持平，真正的收益是识别吞吐 +57%；蒙板单独算账且成本不低，默认关闭。**

## 一、实验环境

| 项 | 值 |
|---|---|
| 板卡 | Jetson AGX Orin Developer Kit（L4T R36.4.7 / JetPack 6.2.1） |
| CUDA / cuDNN / TensorRT | 12.6.68 / 9.3.0 / **10.3.0.30** |
| Python | 3.10.12（依赖装在 `~/.local`，系统级，**未用 venv**） |
| torch / torchvision | **2.11.0**（jetson `linux_aarch64` CUDA 版，cu126）/ 0.26.0 |
| ultralytics | 8.4.147 |
| numpy | 1.26.4 |
| cuDSS | nvidia-cudss-cu12 0.4.0.2.post1（软链到 `torch/lib/`，torch 2.11 需要） |
| 工作区 | `/home/nvidia/usv_ws_test`（`src/usv_perception/{usv_vision,usv_vision_gmsl_cpp}`） |
| 相机 | `/dev/video0`，1920×1536 @ 30fps，单路 |

### engine

- 模型：`yolo11n.pt`（**COCO 预训练，不含 boat 类**，仅用于打通链路与测性能，不代表检测效果）
- 构建：`trtexec --onnx=yolo11n.onnx --saveEngine=yolo11n_fp16_640.engine --fp16 --builderOptimizationLevel=0`
  - ⚠️ **不要加 `--memPoolSize`**：加上会把 workspace 池挤爆，报 `Could not find any implementation for node .../attn/Split.../Add} due to insufficient workspace`（看起来像模型不兼容，实际是参数用错）
- 校验：`--loadEngine` 实测 **GPU compute 7.1 ms/帧、140 qps、H2D 0.25ms**
- 路径：`~/usv_ws_test/model/jetson/yolo11n_fp16_640.engine`（`engine_path` 相对 cwd 解析）

## 二、控制变量与测量方法

### 变量隔离（本次最容易踩的坑）

三组之间**只允许一个变量变化**，其余全部相同（同一份代码、同一 engine、同一相机、同一负载）：

| 组 | 变量 | 说明 |
|---|---|---|
| A | `use_algo_image:=false` | 识别端订阅全分辨率 `image_raw`，自己 resize（= 改造前形态） |
| B | `use_algo_image:=true` | 识别端订阅 `image_algo`（640×640，驱动侧分发） |
| C | `enable_mask:=true` | 在 B 之上只开蒙板 |

> 用户明确要求：对比驱动优化时**不能把蒙板等新特性混进来**，否则结论不可信。

### 测量手段

| 指标 | 方法 | 为什么不用更"直观"的办法 |
|---|---|---|
| 采集出帧率 | 订阅 `frame_timing_json`（一帧一条小消息）计数 | `ros2 topic hz image_raw` 在 8.8MB×30fps 下 Python 跟不上，会把 30Hz 误报成 16Hz |
| 进程 CPU | `/proc/<pid>/stat` 的 `utime+stime` 增量 ÷ CLK_TCK ÷ 12s | `ps` 瞬时值噪声极大（曾出现"无订阅 51%""有订阅 41%"的错值） |
| 识别吞吐 | `ros2 topic hz /perception/camera/front/detections` | — |
| 单帧耗时 | 节点自带日志 `decode/infer/post` | — |

- CPU 均为**单核基准**（8 核 SoC，100% = 占满 1 个核）。
- 测量前确认：无残留节点（`pkill -f gmsl_camera_node_cpp`）、无浏览器等 GUI 负载（会让帧率从 30 掉到 18~22）。
- engine 冷启动帧（首帧 `infer≈1.8s`）不计入。

## 三、结果

### 3.1 主对照：识别端吃大图 vs 吃小图（无蒙板）

| 指标 | A（全分辨率） | B（`image_algo` 640） | 变化 |
|---|---|---|---|
| 采集进程 CPU | 29.1% | 42.7% ~ 45.1% | **+13.6 ~ +16.0 pp** |
| 识别进程 CPU | 66.8% | 48.3% ~ 48.5% | **−18.3 ~ −18.5 pp** |
| **两者合计** | **95.9%** | **91.2% ~ 93.4%** | **−2.5 ~ −4.7 pp** |
| `image_raw` 出帧率 | 30.0 Hz | 30.0 Hz | 不变 |
| **`detections` 输出频率** | **17.6 Hz** | **25.4 ~ 27.6 Hz** | **+44% ~ +57%** |
| 稳态 `infer` | — | 24 ~ 29 ms | — |

### 3.2 阶段 A（只换驱动，识别端仍是旧版）— 补充分发正确性

| 指标 | 结果 |
|---|---|
| `image_raw` 出帧率 | 30.0 Hz（与基线一致，不掉） |
| `image_algo` | **30.0 Hz / 36.5 MB/s**（640×640，理论 36MB/s ✓） |
| timing JSON | `published=true, width=640`；**无订阅者时 `published=false`**（不 resize 不发，零开销逻辑生效） |
| 采集进程 CPU | 无订阅 **17.0%**、+1 个 algo 消费者 **50.3%** |
| 反压（rosbag 作可靠读者 + `SIGSTOP` 冻结） | 出帧率仍 **29.0 Hz，不掉速** |

### 3.3 阶段 C：蒙板开销

| 指标 | 无蒙板 | 开蒙板 | 变化 |
|---|---|---|---|
| 识别进程 CPU | 39.6% | **55.2%** | +15.6 pp |
| `detections` | 26.1 Hz | **22.0 Hz** | −4.1 Hz |
| 稳态 `infer` | 24.6 ~ 28.0 ms | **34.6 ~ 39.9 ms** | +10 ms |
| 日志 | — | `view mask active (640x640): 111281 ignore px` | 生效确认 |

## 四、结论

1. **改造不是"省 CPU"，而是"换吞吐"**：总 CPU 基本持平（甚至略降），但识别吞吐从 17.6Hz 提到 25~27Hz（**+57%**）。
2. **成本位置很清楚**：采集端 +13.6~16pp 来自"多发一路 1.2MB"的发布路径（resize 已用 `cv::setNumThreads(1)` 压到 4.9ms CPU/帧，剩余主要是序列化 + 共享内存拷贝）；收益位置在识别端（不再搬 8.8MB、不再做 2.9MP→640 的 Python 侧缩放）。
3. **反压安全**：reliable 发布 + 僵死可靠读者不拖慢采集（板上实测 29Hz 不掉）。
4. **蒙板必须单独算账**：开蒙板会吃掉大部分收益（合计 CPU 回到 ~98%，吞吐掉到 22Hz）。**默认关闭是正确选择**；确需开启时建议每 N 帧更新一次 mask，或把 mask 位运算下沉到 C++。
5. **DDS 零拷贝依然不可行**（`sensor_msgs/Image` 不支持 loaned message），本改造的收益全部来自"减少搬运"。

## 五、限制与风险

- 模型是 COCO 版 `yolo11n`，**没有 boat 类**，`detections` 内容为空；本记录比较的是**链路吞吐与 CPU**，不代表检测精度。
- 只测了单路相机、30fps、无 RTSP 推流；多路/带推流时绝对数值会变，但方向应一致。
- 未测长期稳定性（连续跑数小时的热降频影响）。
- 采集端 +13.6pp 的发布成本在小图分辨率提高（如 960×768）时会同步上升。

## 六、复现步骤

```bash
# 0) 环境（一次性）
python3 -m pip install --user --ignore-installed --no-deps --find-links ~/env_wheels \
    torch==2.11.0 torchvision==0.26.0 ultralytics==8.4.147 numpy==1.26.4 \
    nvidia-cudss-cu12==0.4.0.2.post1 cuda-python==12.6.2 onnx onnxslim ...
ln -sf ~/.local/lib/python3.10/site-packages/nvidia/cu12/lib/libcudss.so.0 \
       ~/.local/lib/python3.10/site-packages/torch/lib/libcudss.so.0

# 1) engine（一次性）
/usr/src/tensorrt/bin/trtexec --onnx=~/yolo11n.onnx \
  --saveEngine=~/usv_ws_test/model/jetson/yolo11n_fp16_640.engine \
  --fp16 --builderOptimizationLevel=0

# 2) 起驱动（推流/推理都由外部起，便于控制变量）
cd ~/usv_ws_test && source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 launch usv_vision jetson_gmsl_full.launch.py "cameras:=['front']" \
    "inference:=false" "streaming:=false"

# 3) 起识别节点，只切一个参数
python3 -u ~/usv_ws_test/install/usv_vision/lib/usv_vision/yolo_infer_node --ros-args \
  --params-file src/usv_perception/usv_vision/config/yolo_infer.yaml \
  -p use_algo_image:=true          # A 组用 false；C 组再加 -p enable_mask:=true

# 4) 测量
timeout 12 ros2 topic hz /sensors/camera/front/frame_timing_json     # 采集出帧率
timeout 12 ros2 topic hz /perception/camera/front/detections          # 识别吞吐
# CPU：读 /proc/<pid>/stat 的 utime+stime 增量（不要用 ps 瞬时值）
```

## 七、遗留

- 若要把总 CPU 也压下去：可尝试降低 `image_algo` 发布频率（30→20fps）或复用消息缓冲，预计再省 5~8pp。
- M7（`algo_height` 640→512 保持 5:4）、M3（RTSP 编码进 C++ 采集进程）仍未实施。

> ⚠️ **实验已否决（2026-11-10 实测）**：这个包是把推理从 Python 搬到 C++ 的原型，用来验证
> "压单帧能不能让三路跑满 30 Hz"。结论是否决的——**单帧不是瓶颈**（单路 9–12ms/帧、能力 80+fps，
> 但输出仍只有 28.2Hz），CPU 反而比 Python 版更高（59% vs 44.5%）。真正的原因是"无人订阅仍发 image_raw"
> 占满了发布路径，已由触发式发布解决（三路现达 30Hz）。
> **不要基于这个包继续开发**，留档仅供追溯当时的验证过程。

---

# usv_vision_cpp（实验包）

把视觉识别从 Python（`usv_vision/yolo_infer_node`）挪到 C++ / TensorRT 的原型，用来回答一个问题：
**单帧耗时能不能压到 25 ms 以内，从而跟得住 30 Hz 采集。**

- **不影响 `usv_vision`**：独立包、独立节点、独立话题（默认话题与 Python 版同名，用于替换；也可参数改成 `cpp_*` 并排跑）。
- 消息格式与 Python 版一致（`VisionDetectionArray` / `AIVisionTargets` / `status` / `annotated`）。

## 起法

```bash
# 1) 只起采集（不要起 Python 的 yolo_infer_node，否则同一话题双发布）
ros2 launch usv_vision jetson_gmsl_full.launch.py cameras:="['front']" inference:=false stream_mode:=off

# 2) 起 C++ 推理
ros2 launch usv_vision_cpp yolo_infer_cpp.launch.py sensor:=front

# 3) 看输出与耗时
ros2 topic hz /perception/camera/front/detections
# 节点日志每秒一行：[#N] decode=..ms infer=..ms post+publish=..ms total=..ms (..fps) dets=..
```

想和 Python 版并排对比，就把输出话题错开，两边各跑一路相机：

```bash
ros2 launch usv_vision_cpp yolo_infer_cpp.launch.py sensor:=front \
    detections_topic:=/perception/camera/front/cpp_detections \
    objects_topic:=/perception/camera/front/cpp_objects
```

## 与 Python 版的差异（刻意简化）

| 项 | Python 版 | 本原型 |
|---|---|---|
| 视图蒙板 | 支持 | 未实现 |
| 时序平滑 | 支持（α、track_min_hits） | 未实现 |
| 候选框扩张 / 去重 | 支持 | 未实现 |
| min_box_* 尺寸过滤 | 支持 | 未实现 |
| 类别过滤 | `class_name_allowlist` | 同左 |
| 坐标换算 | `reference_width/height` | 同左 |
| 角度 | pinhole / scaramuzza | 同左 |
| 单帧耗时日志 | 硬编码 0.2 Hz | 参数 `timing_log_hz`，默认 1 Hz |

也就是说：**本原型只做「预处理 → TensorRT 推理 → 阈值 + NMS → 发布」**，用来证明性能上限；
后处理那些特性如果要上，需要再补回 C++。

## 自测口径

1. 起采集 + 本节点，日志里的 `total` 就是单帧耗时（decode + infer + post/publish）。
2. 与 Python 版对比时，建议用同一相机、同一 engine、同一场景，各跑 60 s 取中位数。
3. 30 Hz 的节拍是 33.3 ms；`total` 中位数 ≤25 ms 才有余量跟住 30 Hz。

## 自测结果（2026-11-10，板上 AGX Orin，三路，60 s）

| 配置 | 单帧耗时（pre / infer / post） | 输出频率 | CPU | GPU |
|---|---|---|---|---|
| Python 版（对照，S09） | 0.4 / 16–36 / 2.0 ms，总 24–31 ms | 26.5 / 27.6 / 27.4 Hz | 44.5% | 81.7% |
| C++ + 640 引擎（3 路） | 3.3 / 13–22 / 0.2 ms，总 17–27 ms | 26.3 / 26.4 / 27.1 Hz | 59.1% | 72.4% |
| C++ + 512 引擎（3 路） | 3.1 / 6.5 / 0.1 ms，总 ~14 ms | 25.1 / 25.8 Hz | 69.5% | 63.0% |
| C++ + 512 引擎（**单路**） | 3.3 / 6.5 / 0.1 ms，总 9.7–10.5 ms | **28.2–28.6 Hz** | — | — |

**结论（重要）**：

1. **单帧速度不是瓶颈**。单路时节点处理一帧只要 10 ms（能力约 100 fps），但输出只有 28.2 Hz——说明掉帧发生在相机发布/传输侧，不在推理。
2. **3 路时 GPU 聚合吞吐是墙**：每帧 13–22 ms、GPU 70–98%，每路被压到 26–27 Hz。换 512 引擎把单帧从 20 ms 压到 14 ms，输出反而略降（25–26 Hz），因为 512 需要每帧多一次 resize，CPU 从 59% 升到 69.5%。
3. **"识别改 C++ 省 0.5 核"这个预期没有在本原型上复现**：CPU 反而比 Python 高（59–69% vs 44.5%），主要吃在每帧 `blobFromImage` 的整幅浮点转换与 4.9 MB/2.8 MB 的 H2D/D2H 拷贝上。要真降 CPU 需要 zero-copy（`cudaHostAllocMapped`）+ 不做整幅转换，属于后续优化项。

因此：**只把推理搬到 C++ 并不能让三路跑满 30 Hz**；瓶颈在「相机发布/传输侧 ~28 Hz 上限」与「三路 GPU 聚合吞吐」。要真正达 30 Hz，得先查清传输侧那 2 Hz 掉在哪。

# usv_perception（视觉链路 / ARM 侧）

无人艇（USV）视觉链路：**Miivii GMSL 相机 → C++ 采集/按需分发 → YOLO TensorRT 识别 → RTSP 回传**。
本仓库只包含视觉相关的两个 ROS 2 包：

| 包 | 职责 |
|---|---|
| `usv_vision_gmsl_cpp` | C++ 采集节点：SDK 取帧 → 按需分发 `image_raw`(全分辨率) / `image_algo`(640) → **内置 H.264 硬件编码推流** |
| `usv_vision` | Python 侧：`yolo_infer_node`（TensorRT 推理）、`rtsp_stream_node`（旧推流路径，保留回退）、launch 与配置 |

## 性能（Jetson AGX Orin 32GB，三路相机全开，锁频实测）

| 指标 | 改造前（Python 推流） | 现在（C++ 内置推流） |
|---|---|---|
| 视觉 CPU | 3.69 核 | **2.90 核（−21%）** |
| 整机 CPU | 51.8% | **41.8%** |
| 推流成本 | 1.50 核 | **0.70 核（−53%）** |
| 内存 | 10.17 GB | 9.57 GB |
| 帧率 / 画质 | 30Hz / 1920×1536 | 30Hz / 1920×1536（PSNR 43.8dB） |

## 启动

```bash
# 默认：C++ 内置推流
ros2 launch usv_vision jetson_gmsl_full.launch.py "cameras:=['front','right','left']" "inference:=true"

# 回退到旧 Python 推流 / 关闭推流
ros2 launch usv_vision jetson_gmsl_full.launch.py "cameras:=['front','right','left']" "stream_mode:=python"
ros2 launch usv_vision jetson_gmsl_full.launch.py "cameras:=['front','right','left']" "stream_mode:=off"
```

## 文档

- 参数手册与资源分配开关：`usv_vision/docs/parameters.md`
- 性能实测与结论：`usv_vision/docs/image_link_ab_results.md`
- M3（内置编码推流）设计：`usv_vision_gmsl_cpp/docs/m3_internal_encode_design.md`

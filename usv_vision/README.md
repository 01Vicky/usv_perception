# usv_vision

## 当前部署入口

当前安装入口为 `yolo_infer_node`、`rtsp_stream_node` 和
`jetson_gmsl_full.launch.py`。后者使用独立的 `usv_vision_gmsl_cpp` 包采集图像，
推理节点执行 YOLO 检测，推流节点保留 Jetson GStreamer 硬件编码方案。
下文的早期 bring-up 记录保留供排障参考，不代表当前推理节点仍是占位实现。

推理现已支持视图蒙板，默认关闭。使用 `capture_mask_frame`、`make_mask`、
`draw_mask` 生成蒙板，设置 `enable_mask` / `mask_path` 后通过
`/perception/camera/front/reload_mask` 服务重新加载。样板和文档随包安装，
详见 [docs/mask_usage.md](docs/mask_usage.md)。

`usv_vision` 是 `usv_perception` 下的视觉接入包，当前主要负责两件事：

1. 把 Jetson 上的 GMSL / V4L2 相机稳定接入 ROS 2
2. 给后续视觉算法、视频回传和现场问题排查提供统一入口

这个包当前**不是完整视觉算法包**。它的现状更准确地说是：

- 采集链已具备最小可用实现
- 低分辨率满帧率接入已经有实测结论
- RTSP / RTMP 推流当前是脚本能力，不是正式 ROS 节点
- YOLO / TensorRT 推理节点还只是占位骨架


## 1. 包定位

当前仓库里，`usv_vision` 的职责边界是：

- 负责相机接入
- 负责图像消息输出
- 负责驱动状态和帧时序输出
- 负责推流测试脚本
- 负责 Jetson 现场 bring-up 文档

不负责：

- MQTT 协议桥接
- MAVLink / 飞控链路
- 雷达、AIS 等非视觉感知
- 正式的检测、跟踪、融合算法实现


## 2. 当前目录结构

核心文件如下：

- `usv_vision/gmsl_camera_node.py`
  采集节点。当前通过 OpenCV `VideoCapture` 打开 GStreamer 管线，把图像发布到 ROS 2。
- `usv_vision/camera_output.py`
  统一负责发布：
  `image_raw`
  `camera_info`
  `frame_timing_json`
  `driver_status`
- `usv_vision/yolo_infer_node.py`
  推理节点占位实现。当前只发布占位状态，不执行真正推理。
- `launch/usv_vision.launch.py`
  单路启动入口。
- `launch/dual_gmsl.launch.py`
  双路启动入口。
- `config/gmsl_camera.yaml`
  前路高分辨率接入配置。
- `config/gmsl_camera_rear.yaml`
  后路高分辨率接入配置。
- `config/gmsl_camera_algo_640.yaml`
  前路低分辨率算法链配置，当前优先走 `nvvidconv`。
- `config/gmsl_camera_algo_640_cpu_backup.yaml`
  前路低分辨率 CPU 旧配置，保留作对照和回退。
- `scripts/gmsl_to_rtsp.sh`
  GMSL 相机到 RTSP 的脚本。
- `scripts/gmsl_to_rtmp.sh`
  GMSL 相机或 RTSP 输入到 RTMP 的脚本。
- `scripts/gmsl_health_check.sh`
  GMSL 健康检查脚本。
- `scripts/jetson_collect_benchmarks.sh`
  Jetson benchmark 采样脚本。


## 3. 当前架构

### 3.1 采集层

采集层现在只有一个正式节点：`gmsl_camera_node`。

它的工作方式是：

1. 读取参数文件里的 `gstreamer_pipeline`
2. 使用 OpenCV `cv2.VideoCapture(..., cv2.CAP_GSTREAMER)` 打开
3. 从 GStreamer 管线取帧
4. 通过 `cv_bridge` 转成 ROS `sensor_msgs/Image`
5. 同步发布 `camera_info`
6. 额外发布：
   `frame_timing_json`
   `driver_status`

这里有一个关键现实约束：

- 节点本身是 Python 实现
- 当前图像消息最终仍然以 `bgr8` 进入 ROS
- 所以即使前面的 GStreamer 管线做了硬件加速，最后一步仍然有 Python / OpenCV / `cv_bridge` 开销

### 3.2 算法层

`yolo_infer_node.py` 当前只是参数骨架，不做真正推理。

它现在能提供的是：

- 未来推理参数的稳定入口
- 未来 topic 约定
- 占位状态上报

它当前**不应该**被当成“已完成的识别节点”。

### 3.3 转发层

转发层当前不做成 ROS 节点，而是保留为脚本能力：

- `gmsl_to_rtsp.sh`
- `gmsl_to_rtmp.sh`

这样做的原因是：

- 推流链和 ROS 图像链的性能目标不同
- 推流更适合尽量走 Jetson 编码器
- 如果把推流也强行塞进当前 Python ROS 节点，问题会更难定位

### 3.4 测试与诊断层

测试与诊断层由命令文档和脚本组成：

- 满帧率接入检查
- namespace 隔离 benchmark
- GMSL 健康检查
- RTSP / RTMP 推流手册

这部分对现场联调很重要，因为当前你们遇到的问题大多不是“功能有没有”，而是：

- 帧率是否满
- 是否串测
- 设备是否被占用
- 编码链是否稳定


## 4. 当前推荐拓扑

当前推荐把视觉接入拆成两条链，而且这里的“两条链”指**两台不同物理相机**：

1. 前路相机 `/dev/video4`
   走低分辨率算法链
2. 后路相机 `/dev/video5`
   走高分辨率显示 / 回传链

推荐原因：

- 高分辨率 `1920x1536 bgr8` 直接进 ROS 主链开销很大
- 算法链实际只需要较低分辨率
- 回传链更适合走 H.264 / RTSP / RTMP

所以当前推荐拓扑是：

- `/dev/video4 -> nvvidconv -> 640x640 -> ROS image_raw`
- `/dev/video5 -> H.264 -> RTSP / RTMP`

不推荐的拓扑是：

- 把 `1920x1536` raw BGR 长期作为 ROS 主链


## 5. 当前已实现能力

### 5.1 单路 ROS 图像接入

已实现，并且可以启动。

命令：

```bash
ros2 launch usv_vision usv_vision.launch.py
```

### 5.2 双路 ROS 图像接入

已实现，并且有独立前后路配置。

命令：

```bash
ros2 launch usv_vision dual_gmsl.launch.py
```

### 5.3 低分辨率算法链接入

已实现，并且经过实测。

命令：

```bash
ros2 launch usv_vision usv_vision.launch.py \
  namespace:=front_gmsl_algo \
  gmsl_config:=<ws>/src/usv_perception/usv_vision/config/gmsl_camera_algo_640.yaml
```

### 5.4 RTSP 推流脚本

已实现，脚本在：

`scripts/gmsl_to_rtsp.sh`

### 5.5 RTMP 推流脚本

已实现，脚本在：

`scripts/gmsl_to_rtmp.sh`

### 5.6 现场 benchmark 脚本

已实现，脚本在：

`scripts/jetson_collect_benchmarks.sh`

它当前支持：

- `gst`
- `ros`
- `rtsp`

并且在 `ros` 模式下已支持 namespace 隔离，避免不同测试 case 串测。


## 6. 当前未完成或仅占位能力

以下内容当前不能写成“已完成”：

### 6.1 YOLO / TensorRT 推理

`yolo_infer_node.py` 现在只是占位节点。

### 6.2 正式的 RTSP / RTMP ROS 节点

当前只有脚本，没有集成为 ROS 节点。

### 6.3 多路同步

当前没有正式实现双路时间同步、双光谱对齐或 `message_filters` 同步节点。

### 6.4 完整的生产级状态机

当前采集节点能输出 `driver_status`，但还没有更细的恢复策略和异常分类。


## 7. 已验证的定量结论

以下结论是基于现场 Jetson 测试得到的，当前 README 直接记录，避免后续重复误判。

### 7.1 相机底层直出正常

`/dev/video4`、`/dev/video5` 在纯 `v4l2src` 下可接近 `30fps`。

### 7.2 CPU 低分辨率链很慢

CPU 路径：

`v4l2src -> videoscale -> videoconvert -> BGR`

实测大约只有：

- `7 ~ 9 fps`

这说明问题不在相机底层，而在 CPU 缩放 / 颜色转换链。

### 7.3 nvvidconv 低分辨率链可接近满帧

硬件路径：

`v4l2src -> nvvidconv -> BGRx -> videoconvert -> BGR`

在纯 gst 和 ROS 低分辨率算法链下，已经验证可接近：

- `28 ~ 30 fps`

### 7.4 高分辨率 ROS 原始图像链不适合长期主用

高分辨率 `1920x1536 bgr8` ROS 链路之前实测大约：

- `6 fps` 左右

这不是底层相机问题，而是 Python + OpenCV + `cv_bridge` + 大图 raw 消息开销导致。

### 7.5 namespace 串测问题已经定位过

之前出现过 benchmark 结果异常接近、旧版像是“突然也变快”的情况。

后续已通过：

- 自动 namespace 隔离
- 首帧等待
- 子进程组回收

确认那是测试污染，不是 CPU 管线真实性能变好。


## 8. 配置说明

### `gmsl_camera.yaml`

用途：

- 前路高分辨率原始接入

默认：

- `device_path=/dev/video4`
- `1920x1536`
- `30fps`
- CPU `videoconvert -> BGR`

### `gmsl_camera_rear.yaml`

用途：

- 后路高分辨率原始接入

默认：

- `device_path=/dev/video5`
- `1920x1536`
- `30fps`

### `gmsl_camera_algo_640.yaml`

用途：

- 前路低分辨率算法链

默认：

- `device_path=/dev/video4`
- `640x640`
- `30fps`
- `source_type=v4l2_gmsl_algo_nvvidconv`
- `nvvidconv -> BGRx -> videoconvert -> BGR`

这是当前推荐算法链接入配置。

### `gmsl_camera_algo_640_cpu_backup.yaml`

用途：

- CPU 旧链回退和对照测试

默认：

- `device_path=/dev/video4`
- `640x640`
- `fps=15.0`
- `videoscale -> videoconvert -> BGR`

这不是当前推荐默认方案，只保留作对照。


## 9. 发布的 topic

以 namespace `front_gmsl_algo` 为例，节点会发布：

- `/front_gmsl_algo/image_raw`
- `/front_gmsl_algo/camera_info`
- `/front_gmsl_algo/frame_timing_json`
- `/front_gmsl_algo/driver_status`

说明：

- `image_raw`：`sensor_msgs/msg/Image`
- `camera_info`：`sensor_msgs/msg/CameraInfo`
- `frame_timing_json`：`std_msgs/msg/String`
- `driver_status`：`std_msgs/msg/String`

其中：

- `frame_timing_json` 用于记录分辨率、采集戳、发布时间戳、驱动延迟
- `driver_status` 用于记录 `playing / error / restarting` 等状态


## 10. 启动方式

### 单路默认启动

```bash
ros2 launch usv_vision usv_vision.launch.py
```

### 双路启动

```bash
ros2 launch usv_vision dual_gmsl.launch.py
```

### 低分辨率算法链

```bash
ros2 launch usv_vision usv_vision.launch.py \
  namespace:=front_gmsl_algo \
  gmsl_config:=<ws>/src/usv_perception/usv_vision/config/gmsl_camera_algo_640.yaml
```

### CPU 备份对照链

```bash
ros2 launch usv_vision usv_vision.launch.py \
  namespace:=front_gmsl_algo_cpu \
  gmsl_config:=<ws>/src/usv_perception/usv_vision/config/gmsl_camera_algo_640_cpu_backup.yaml
```

### 启用推理占位节点

```bash
ros2 launch usv_vision usv_vision.launch.py enable_yolo:=true
```

注意：这不会启用真正推理，只会启动占位节点。


## 11. 推流能力

### RTSP

脚本：

```bash
bash src/usv_perception/usv_vision/scripts/gmsl_to_rtsp.sh /dev/video5 rtsp://127.0.0.1:8554/cam1
```

### RTMP

脚本：

```bash
bash src/usv_perception/usv_vision/scripts/gmsl_to_rtmp.sh rtmp://101.43.14.147:1935/live/test
```

说明：

- 当前推流能力是脚本，不是 ROS 节点
- 如果现场花屏，应优先怀疑编码链、设备占用和码率设置
- 先把 RTSP 本地推拉跑稳，再考虑公网 RTMP


## 12. 现场检查与 benchmark

### 满帧率接入命令

看：

- `docs/jetson_full_fps_ingest_commands.txt`

### 推流手册

看：

- `docs/jetson_streaming_manual.txt`

### namespace 隔离复核

看：

- `docs/jetson_namespace_isolation_retry.txt`

### 健康检查

```bash
bash src/usv_perception/usv_vision/scripts/gmsl_health_check.sh
```

带 ROS 检查：

```bash
CHECK_ROS=1 bash src/usv_perception/usv_vision/scripts/gmsl_health_check.sh
```

### benchmark

```bash
bash src/usv_perception/usv_vision/scripts/jetson_collect_benchmarks.sh --help
```


## 13. 当前已知问题

### 13.1 高分辨率 ROS 原始链性能差

这是当前实现形态决定的，不是简单参数问题。

### 13.2 `yolo_infer_node` 还未接实推理

当前不能用于正式算法联调。

### 13.3 benchmark CSV 会累积旧 case

如果复用相同 `case_name`，结果 CSV 会追加多行，分析时要以最后一轮有效结果为准。

### 13.4 `observed_resolution` 汇总字段还不稳定

虽然 `first_message.log` 和 `frame_timing_json` 能确认分辨率，但 summary 里该字段当前可能仍为 `NA`。

### 13.5 推流链仍需现场继续调参

尤其是：

- RTSP 花屏
- RTMP 卡顿

当前还不能写成“已完全稳定”。


## 14. 推荐下一步

当前更合理的推进顺序是：

1. 把 `nvvidconv` 低分辨率算法链作为默认接入方案
2. 保留 CPU 备份配置用于对照和回退
3. 单独继续收敛 RTSP / RTMP 推流稳定性
4. 在板端环境具备后，再把真实推理节点接进来
5. 最后再考虑双路同步和更完整的视觉主链


## 15. 结论

`usv_vision` 当前已经不是单纯的“空骨架”了。

它已经具备：

- 单路 / 双路 GMSL 相机接入
- 低分辨率满帧率接入方案
- RTSP / RTMP 推流脚本
- 现场 benchmark 和排查脚本

但它也还没有到“完整视觉系统”的程度。

当前最准确的定位是：

**这是一个已能承担 Jetson 视觉接入和现场调试工作的工程包，但推理层和生产级推流稳定性仍需继续完善。**

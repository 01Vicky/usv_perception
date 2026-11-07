# usv_vision Jetson Deploy Notes

## 当前定位

这份文档先记录部署边界，不假设当前 Jetson 已具备完整视觉环境。

## 当前已知板端状态

根据现场 `jtop` 信息，当前板端环境可认为是：

- Ubuntu 22.04 / kernel `5.15.148-tegra`
- Python `3.10.12`
- OpenCV `4.5.4`
- `CUDA: MISSING`
- `TensorRT: MISSING`
- `cuDNN: 9.3.0.75`
- `VPI: 3.2.4`

这意味着当前适合先做：

- GMSL 采集
- ROS 图像发布
- RTSP 转发
- 资源监控与日志

当前不适合先做：

- TensorRT 推理
- YOLO 正式联调

## 未来部署需要补齐的环境

至少包括：

- ROS2 Python 运行环境
- `sensor_msgs`
- `cv_bridge`
- `opencv-python` 或系统 OpenCV
- GStreamer 运行时与 Jetson 相关插件
- TensorRT 运行环境
- 识别模型和 engine 文件

## 当前阶段建议

在 Jetson 环境尚未齐全前，先完成：

1. 包结构落库
2. 参数和 launch 入口固定
3. GMSL 采集节点先落地
4. 旧视觉框架代码分层裁剪方案明确

待板端环境具备后，再分三步验证：

1. `/dev/video4`、`/dev/video5` GMSL 采集
2. `image_raw` 到推理节点的单路识别
3. RTSP 转发和双路同步

## 当前可直接使用的配置

- 前路相机配置：
  - `config/gmsl_camera.yaml`
  - 默认设备：`/dev/video4`
- 后路相机配置：
  - `config/gmsl_camera_rear.yaml`
  - 默认设备：`/dev/video5`

双路启动入口：

```bash
ros2 launch usv_vision dual_gmsl.launch.py
```

这里需要明确：

- `/dev/video4` 和 `/dev/video5` 是两台不同的物理 GMSL 相机
- 当前文档里的“算法链”和“回传链”是分别绑定到这两台相机上的
- 不是把同一台相机同时拆成两条主链来跑

## 当前推荐运行方式

不要把 `1920x1536` raw BGR 直接作为 ROS 主链长期运行。

推荐拆成：

1. 低分辨率算法链

```bash
ros2 launch usv_vision usv_vision.launch.py \
  namespace:=front_gmsl_algo \
  gmsl_config:=/absolute/path/to/gmsl_camera_algo_640.yaml
```

默认对应前路相机 `/dev/video4`

2. 高分辨率 RTSP 回传链

```bash
bash src/usv_perception/usv_vision/scripts/gmsl_to_rtsp.sh /dev/video5 rtsp://127.0.0.1:8554/cam1
```

3. 一键健康检查

```bash
bash src/usv_perception/usv_vision/scripts/gmsl_health_check.sh
```

如果需要把 ROS 话题检查也一起带上：

```bash
CHECK_ROS=1 bash src/usv_perception/usv_vision/scripts/gmsl_health_check.sh
```

默认对应后路相机 `/dev/video5`

## 已验证的现场命令

当前现场已知可用的基础命令应作为后续 ROS 节点接入参考：

```bash
gst-launch-1.0 -ev v4l2src device=/dev/video4 ! "video/x-raw, format=(string)UYVY, width=(int)1920, height=(int)1536" ! fpsdisplaysink text-overlay=0 video-sink=xvimagesink sync=0
```

```bash
./mediamtx
gst-launch-1.0 -v v4l2src device=/dev/video5 ! video/x-raw,format=UYVY,width=1920,height=1536,framerate=30/1 ! nvvidconv ! nvv4l2h264enc bitrate=4000000 ! h264parse ! rtspclientsink location=rtsp://127.0.0.1:8554/cam1
```

```bash
ffplay rtsp://10.42.0.81:8554/cam1
```

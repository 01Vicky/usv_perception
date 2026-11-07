# usv_vision Architecture

## 目标

`usv_vision` 用来在当前仓库里规范承接视觉主链路，避免继续把 GMSL、转发、识别代码散落在不同工作区。

## 规划模块

### 1. 采集层

当前已由 `gmsl_camera_node.py` 承接最小实现，负责：

- `/dev/video4`、`/dev/video5` 等 V4L2 GMSL 设备接入
- GStreamer 管线配置
- 发布 `image_raw`
- 发布 `camera_info`
- 发布 `frame_timing_json`
- 发布 `driver_status`

当前已提供：

- 单路启动：`launch/usv_vision.launch.py`
- 双路启动：`launch/dual_gmsl.launch.py`

### 2. 识别层

后续由 `yolo_infer_node.py` 承接，负责：

- 订阅 `image_raw`
- 读取 `camera_info`
- 执行 TensorRT / YOLO 推理
- 发布检测结果
- 可选发布 `image_yolo`

### 3. 转发层

当前不放进 ROS 节点主链，先保留为脚本能力，后续可扩展为：

- RTSP 转发
- UDP 推流
- compressed 图像输出

这样可以在没有 CUDA / TensorRT 的 Jetson 上先独立验证采集与转发，不阻塞后续识别环境准备。

## 当前推荐拓扑

推荐把采集分成两条独立链路：

1. 高分辨率回传链
   后路相机：
   `/dev/video5 -> H264 -> mediamtx -> RTSP`

2. 低分辨率算法链
   前路相机：
   `/dev/video4 -> GStreamer 降分辨率 -> ROS image_raw`

原因：

- 高分辨率 raw BGR 经 Python/OpenCV/cv_bridge 进入 ROS，吞吐开销过大
- RTSP/H264 更适合作为显示、回传、录像主链
- 算法链只保留推理真正需要的分辨率和帧率

注意：

- `/dev/video4` 和 `/dev/video5` 是两台不同的 GMSL 相机
- 当前推荐方案不是“同一路相机同时做高分辨率回传和低分辨率算法”
- 而是“前路相机负责算法链，后路相机负责回传链”

### 4. 联调与日志

建议复用仓库根目录 `monitor/` 下工具，后续补充视觉专项脚本：

- 资源占用
- 话题频率与延迟
- GMSL 健康检查
- 转发质量统计

## 设计边界

`usv_vision` 只负责视觉主链路，不负责：

- MQTT 协议桥接
- MAVLink / 飞控控制
- 航迹融合
- 雷达 / AIS 感知处理

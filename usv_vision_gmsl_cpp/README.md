# usv_vision_gmsl_cpp

GMSL相机C++节点，支持Miivii SDK硬件时间戳。

## 功能

- 通过Miivii GMSL SDK获取硬件时间戳（纳秒级精度）
- 支持多相机硬件同步
- 兼容usv_vision的话题接口
- 支持任意/dev/videoX设备
- 可选的分辨率自动检测

## 依赖

### 硬件要求
- NVIDIA Jetson平台（Xavier/Orin等）
- Miivii GMSL相机模组

### 软件依赖
```bash
# ROS 2 Humble
sudo apt install ros-humble-cv-bridge

# OpenCV 4
sudo apt install libopencv-dev

# JSON库
sudo apt install nlohmann-json3-dev

# Miivii GMSL SDK
# 需要安装到: /opt/miivii/
```

## 编译

```bash
cd ~/your_ros2_ws
colcon build --packages-select usv_vision_gmsl_cpp
```

如果SDK不存在，编译会跳过C++节点但不会失败。

## 使用

### 单相机
```bash
ros2 launch usv_vision_gmsl_cpp gmsl_camera_cpp.launch.py
```

### 双相机
```bash
ros2 launch usv_vision_gmsl_cpp dual_gmsl_cpp.launch.py
```

### 自定义设备
换设备/分辨率等请编辑 config yaml 里的 `device_path` 等字段，再用 `config:` 指定：
```bash
ros2 launch usv_vision_gmsl_cpp gmsl_camera_cpp.launch.py \
  config:=$(ros2 pkg prefix usv_vision_gmsl_cpp)/share/usv_vision_gmsl_cpp/config/gmsl_camera_video2.yaml
```
（本 launch 不提供 `device_path:=` 命令行覆盖，避免看似生效实则被忽略。）

## 配置文件

- `config/gmsl_camera_cpp_front.yaml` - 前置相机
- `config/gmsl_camera_cpp_rear.yaml` - 后置相机
- `config/gmsl_camera_template.yaml` - 模板（完整注释）

## 话题输出

```
/sensors/camera/{sensor_name}/image_raw          # sensor_msgs/Image
/sensors/camera/{sensor_name}/camera_info        # sensor_msgs/CameraInfo
/sensors/camera/{sensor_name}/frame_timing_json  # std_msgs/String
/sensors/camera/{sensor_name}/driver_status      # std_msgs/String
```

### 按需分发（image_raw / image_algo）

采集端一次成帧，在**同一个进程里**按消费者需求分发两种规格，避免同一帧被多个下游
各自从 DDS 捞一遍、各自再转换一遍：

| 话题 | 尺寸 | QoS | 消费者 |
|---|---|---|---|
| `image_raw` + `camera_info` | 全分辨率（如 1920x1536） | 由 `image_qos_*` 决定（默认 best_effort/depth2） | 推流/录像等"要原图"的消费方 |
| `image_algo` + `camera_info_algo` | `algo_width`×`algo_height`（默认 640） | 与 `image_raw` 同一套（当前 reliable/depth2） | `yolo_infer_node` 识别链 + rviz 人工观察 |

要点：

- 两者共用同一份已转换好的 BGR 帧，差别只在一次 `cv::resize`；`camera_info` 走同一个
  `CameraInfoLoader`，按分辨率缓存并自动缩放内参，所以算法图的内参天然与算法图分辨率一致。
- 算法图在 `get_subscription_count() == 0` 时**不发、不 resize、不分配**，没人看就零开销。
- QoS：发布端由 `image_qos_*` 统一控制（当前 `reliable/depth2`），订阅端一律 `best_effort/depth1`。
  DDS 兼容方向是"提供强于等于请求"，所以 reliable 发布 + best_effort 订阅成立；
  已实测 8.4MB×depth10×2 订阅者时 `publish()` 平均 5ms、最大 9~14ms、0 次超 33ms，
  1.2MB×depth2 余量更大，可靠发布不会拖慢采集线程。
- `algo_width/algo_height` 设为 ≤0 或与原始尺寸相同时不缩放，直接复用原图。
- 参数：`enable_algo_stream`、`algo_width`、`algo_height`，见 `config/gmsl_camera_template.yaml` 注释。

**人工观察怎么开**：rviz 直接订阅 `image_algo` 即可（默认 QoS 就能看到画面，不必改 Reliability）；
要看检测框叠加则订阅 `/perception/camera/{sensor_name}/annotated`。之前那条独立的
`image_debug` 话题已合并进 `image_algo`——两者是同一份像素，单开一条没有收益。

> 为什么不做"真零拷贝"：ROS 2 Humble 的 loaned message 只支持定长/POD 类型，
> `sensor_msgs/Image` 实测 `can_loan_messages() == false`，换 iceoryx/CycloneDDS 也绕不开；
> `rclpy` 也没有 loan 接口。因此收益来自"别让同一帧被搬多份"（按需分发），而不是"让 DDS 不拷贝"。

## 文档

**入门 / 原理**
- [快速开始](docs/GMSL_QUICKSTART.md)
- [硬件时间戳详细文档](docs/GMSL_HARDWARE_TIMESTAMP.md)
- [实现总结](docs/IMPLEMENTATION_SUMMARY.md)

**内参标定**
- [相机内参说明](docs/CAMERA_INFO.md) - camera_info 字段/单位、三级兜底、FOV 反算、分辨率缩放
- 内参样例：[config/camera_info_sample.json](config/camera_info_sample.json)

**部署 / 对接**
- [与主功能包 usv_vision 的对接](docs/INTEGRATION_WITH_USV_VISION.md) - 话题解耦、分别启动、对接自检
- [Jetson 编译验证 Checklist](docs/JETSON_VERIFY_CHECKLIST.md) - 环境检查、补救命令、逐项验证

## 与usv_vision的关系

- `usv_vision` - Python节点，适用于仿真和测试
- `usv_vision_gmsl_cpp` - C++节点，适用于实船硬件

两个包发布相同格式的ROS话题，可互相替换。详见
[与主功能包 usv_vision 的对接](docs/INTEGRATION_WITH_USV_VISION.md)。

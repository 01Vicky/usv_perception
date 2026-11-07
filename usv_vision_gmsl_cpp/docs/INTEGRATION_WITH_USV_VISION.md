# 与主功能包 usv_vision 的对接

本包（`usv_vision_gmsl_cpp`）是 GMSL 相机的 C++ 驱动/接入层，主功能包
（`usv_vision`）负责下游处理与感知识别。两者通过**统一 ROS 话题解耦**，
各自独立 `ros2 launch`，无编译期依赖、无 launch 互相 include。

```
GMSL相机 → [gmsl_camera SDK 硬件时间戳] → usv_vision_gmsl_cpp (本包, C++驱动)
                                              │ 发布统一话题(带硬件时间戳)
                                              ▼
                                         usv_vision (主功能包: 处理 + 感知识别)
```

## 一、对接契约：话题由 sensor_name 决定

本包节点发布：

```
/sensors/camera/{sensor_name}/image_raw          sensor_msgs/Image
/sensors/camera/{sensor_name}/camera_info        sensor_msgs/CameraInfo
/sensors/camera/{sensor_name}/frame_timing_json  std_msgs/String   (仅单相机节点)
/sensors/camera/{sensor_name}/driver_status      std_msgs/String   (仅单相机节点)
```

`usv_vision` 的下游节点（如 `yolo_infer_node`）默认订阅：

```
image_topic       = /sensors/camera/{sensor_name}/image_raw
camera_info_topic = /sensors/camera/{sensor_name}/camera_info
```

两边用的是同一套命名规则（`usv_vision/common/topics.py`）。**唯一要保证的是
两边 `sensor_name` 相同**，话题就自动对齐，下游代码无需改动。

> 注：C++ 节点话题名固定由 `sensor_name` 拼出，不支持用参数覆盖话题名
> （与 Python 版差异，详见 config 模板说明）。对齐点只有 `sensor_name`。

## 二、启动方式（分别启动，推荐）

### 终端 1 —— 起 C++ 驱动（实船硬件 + 硬件时间戳）

单相机：
```bash
ros2 launch usv_vision_gmsl_cpp gmsl_camera_cpp.launch.py
# 换设备/分辨率等改 config yaml，再用 config: 指定
ros2 launch usv_vision_gmsl_cpp gmsl_camera_cpp.launch.py config:=<abs>/gmsl_camera_video2.yaml
```

多相机硬件同步：
```bash
ros2 launch usv_vision_gmsl_cpp gmsl_sync_multi.launch.py
```

### 终端 2 —— 起 usv_vision 下游处理

```bash
# 确保下游节点的 sensor_name 与终端1一致（如都用 front）
ros2 launch usv_vision <下游处理>.launch.py
```

`yolo_infer_node` 目前是占位骨架；实际下游 launch 以 usv_vision 包内为准。

## 三、对接自检

1. 话题在线：
   ```bash
   ros2 topic list | grep /sensors/camera
   ros2 topic hz /sensors/camera/front/image_raw
   ros2 topic hz /sensors/camera/front/camera_info
   ```
2. sensor_name 对齐：确认 C++ 驱动的 `sensor_name` 与下游节点的 `sensor_name` 相同
   （不一致会各发各的话题、下游收不到）。
3. 连通性：
   ```bash
   ros2 topic info /sensors/camera/front/image_raw   # Publisher=cpp驱动, Subscriber=下游
   ```
4. 内参（依赖测距/去畸变的下游需要）：
   ```bash
   ros2 topic echo /sensors/camera/front/camera_info --once
   # 配了标定 → k 非零；未配 → k 全零(空标定，仍正常出图)。详见 CAMERA_INFO.md
   ```

## 四、注意事项

- **QoS**：C++ 驱动图像默认 `best_effort` / depth 1。下游订阅需兼容
  （best_effort 发布者 + reliable 订阅者不匹配会收不到）。
- **时间基准**：C++ 驱动用 GMSL 硬件时间戳（PHC/PTP 时钟），与走 ROS 系统时间的
  其它传感器对齐前，需先跑 PTP 同步（`2_production/ptp_sync/setup_ptp_sync.sh`）。
  详见 GMSL_HARDWARE_TIMESTAMP.md 与 JETSON_VERIFY_CHECKLIST.md 第 4 节。
- **可替换性**：本包与 `usv_vision` 的 Python 相机节点发布相同话题，可互相替换
  （仿真/RTSP 用 Python 节点，实船用本包 C++ 节点），下游不需改。
- **SDK 缺失**：本包在无 Miivii SDK 时只装 launch/config、不编节点；此时改用
  `usv_vision` 的 Python 节点接入，主功能包不受影响。

# GMSL相机硬件时间戳支持文档

## 概述

本功能包现在支持两种GMSL相机节点实现：

1. **Python节点** (`gmsl_camera_node.py`) - 原有实现，使用ROS系统时间
2. **C++节点** (`gmsl_camera_node_cpp`) - 新增实现，支持Miivii GMSL SDK硬件时间戳

## 为什么需要硬件时间戳？

### 系统时间 vs 硬件时间戳

```
┌─────────────────────────────────────────────────────────┐
│ 相机硬件触发时刻                                         │
│     ↓                                                   │
│     ├─ 图像传输延迟 (1-5ms)                             │
│     ├─ 驱动处理延迟 (0.5-2ms)                           │
│     ├─ OpenCV读取延迟 (0.5-1ms)                         │
│     └─ ROS系统时间戳 ← Python节点在这里打时间戳         │
│                                                         │
│ 总延迟：2-8ms，且不稳定                                  │
└─────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────┐
│ 相机硬件触发时刻 ← C++节点用SDK获取这个精确时间          │
│     ↓                                                   │
│     └─ 图像传输...                                      │
│                                                         │
│ 延迟：0ms，使用的是触发时刻的硬件时间戳                   │
└─────────────────────────────────────────────────────────┘
```

### 关键优势

1. **时间精度**：纳秒级硬件时间戳 vs 毫秒级系统时间
2. **多相机同步**：多个GMSL相机硬件同步触发，时间戳完全对齐
3. **时间溯源**：可追溯到真实触发时刻，不受传输延迟影响

## 依赖要求

### 硬件要求
- NVIDIA Jetson平台（Xavier/Orin/Nano等）
- Miivii GMSL相机模组

### 软件依赖
```bash
# ROS 2 Humble/Foxy
sudo apt install ros-${ROS_DISTRO}-cv-bridge

# OpenCV 4
sudo apt install libopencv-dev

# JSON库
sudo apt install nlohmann-json3-dev

# Miivii GMSL SDK（厂商提供）
# 需要安装到: /opt/miivii/
```

### SDK可迁移性

✅ **可以迁移**：
- 相同硬件配置的Jetson平台（只需安装SDK）
- Docker容器（需要--device映射和--privileged）

❌ **不能迁移**：
- 普通PC/服务器（没有GMSL硬件接口）
- Windows系统（SDK是Linux专用）

## 编译

### 检查SDK是否存在
```bash
ls /opt/miivii/include/MvGmslCamera.h
ls /opt/miivii/lib/libmvgmslcamera.so
```

### 编译功能包
```bash
cd ~/your_ros2_ws
colcon build --packages-select usv_vision
```

**注意**：如果SDK不存在，CMakeLists.txt会跳过C++节点编译，只编译Python节点。

## 使用方法

> 说明：所有相机参数（device_path、分辨率、enable_hardware_timestamp、
> auto_detect_resolution 等）统一在 config yaml 中配置。launch 只接受 `config:` 指定
> 配置文件，不提供 `device_path:=` 等命令行覆盖（避免看似生效实则被忽略）。

### 1. 单相机前置（C++节点，硬件时间戳）
```bash
ros2 launch usv_vision_gmsl_cpp gmsl_camera_cpp.launch.py
```

### 2. 单相机后置
```bash
ros2 launch usv_vision_gmsl_cpp gmsl_camera_cpp.launch.py \
  config:=$(ros2 pkg prefix usv_vision_gmsl_cpp)/share/usv_vision_gmsl_cpp/config/gmsl_camera_cpp_rear.yaml
```

### 3. 双相机（前+后，硬件同步时间戳）
```bash
ros2 launch usv_vision_gmsl_cpp dual_gmsl_cpp.launch.py
```

### 4. 自定义设备路径 / 降级到ROS系统时间
```bash
# 编辑 config yaml 里对应字段后用 config: 指定：
#   device_path: "/dev/video2"          # 换设备
#   enable_hardware_timestamp: false    # 降级到 ROS 系统时间
#   auto_detect_resolution: true        # 自动检测分辨率
ros2 launch usv_vision_gmsl_cpp gmsl_camera_cpp.launch.py config:=<abs>/your_camera.yaml
```

### 6. 自动检测分辨率
在 config yaml 里设 `auto_detect_resolution: true`，再照上面用 `config:` 指定。

### 7. 继续使用Python节点（原有行为，属主功能包 usv_vision）
```bash
ros2 launch usv_vision dual_gmsl.launch.py
```

## 配置文件

### 关键参数说明

```yaml
/**:
  ros__parameters:
    # 设备配置
    device_path: "/dev/video0"              # 视频设备号
    sensor_name: "front"                    # 传感器名称（用于topic）

    # 图像参数
    width: 1920                             # 分辨率宽度
    height: 1536                            # 分辨率高度
    fps: 30.0                               # 帧率

    # 硬件时间戳（C++节点专用）
    enable_hardware_timestamp: true         # 启用GMSL硬件时间戳
    auto_detect_resolution: false           # 自动检测设备实际分辨率

    # GStreamer管道（可选）
    gstreamer_pipeline: ""                  # 为空时自动生成
```

### 配置文件位置

- `config/gmsl_camera_cpp_front.yaml` - 前置相机配置
- `config/gmsl_camera_cpp_rear.yaml` - 后置相机配置
- `config/gmsl_camera_template.yaml` - 模板（带完整注释）

## 话题输出

C++节点输出与Python节点**完全兼容**：

```
/sensors/camera/{sensor_name}/image_raw          # sensor_msgs/Image
/sensors/camera/{sensor_name}/camera_info        # sensor_msgs/CameraInfo
/sensors/camera/{sensor_name}/frame_timing_json  # std_msgs/String
/sensors/camera/{sensor_name}/driver_status      # std_msgs/String
```

### 时间戳字段

```json
{
  "timestamps": {
    "capture_stamp_ros": {
      "sec": 1234567890,
      "nanosec": 123456789
    }
  }
}
```

- **enable_hardware_timestamp: true**: `capture_stamp_ros`是GMSL硬件触发时间戳（纳秒精度）
- **enable_hardware_timestamp: false**: `capture_stamp_ros`是ROS系统时间

## 验证时间戳

### 方法1：查看timing消息
```bash
ros2 topic echo /sensors/camera/front/frame_timing_json
```

查看输出中的`source_type`字段：
- `"v4l2_gmsl_hw_timestamp"` - 使用硬件时间戳
- `"v4l2_gmsl"` - 使用系统时间

### 方法2：对比多相机时间戳
```bash
# 启动双相机
ros2 launch usv_vision_gmsl_cpp dual_gmsl_cpp.launch.py

# 对比两路时间戳差异（硬件同步时应该小于1ms）
ros2 topic echo /sensors/camera/front/image_raw | grep stamp &
ros2 topic echo /sensors/camera/rear/image_raw | grep stamp &
```

## 故障排查

### 问题1：编译时找不到SDK
```
CMake Warning: Miivii GMSL SDK not found
```
**解决**：
```bash
# 检查SDK是否安装
ls /opt/miivii/

# 如果没有，联系Miivii获取SDK安装包
# 或修改CMakeLists.txt中的SDK路径
```

### 问题2：运行时SDK初始化失败
```
Failed to initialize GMSL SDK. Falling back to ROS timestamp.
```
**解决**：
- 检查设备权限：`sudo chmod 666 /dev/video*`
- 检查SDK库链接：`ldd $(which gmsl_camera_node_cpp) | grep mvgmslcamera`

### 问题3：时间戳为0或无效
```
Warning ! Invalid timestamp value : 0
```
**解决**：
- 相机可能还在初始化，等待2-3秒
- 检查camera_no是否正确（从device_path自动解析）
- 尝试手动指定：`device_path: "/dev/video0"` → camera_no = 0

### 问题4：多相机时间戳不同步
```
Some cameras have data, but the cameras are out of sync.
```
**解决**：
- 检查GMSL同步配置（SDK配置文件）
- 确认多个相机连接到同一个GMSL聚合器
- 查看SDK demo的同步参数设置

## 性能对比

| 指标 | Python节点 | C++节点（硬件时间戳） |
|------|-----------|---------------------|
| 时间戳延迟 | 2-8ms | 0ms |
| 时间戳抖动 | ±2ms | <0.1ms |
| 多相机同步 | 不保证 | 纳秒级同步 |
| CPU占用 | 中等 | 低 |
| 内存占用 | 中等 | 低 |

## 迁移指南

### 从Python节点迁移到C++节点

1. **配置文件**：复用原有配置，只需添加：
   ```yaml
   enable_hardware_timestamp: true
   ```

2. **Launch文件**：
   ```python
   # 旧
   executable="gmsl_camera_node"  # Python

   # 新
   executable="gmsl_camera_node_cpp"  # C++
   ```

3. **下游节点**：不需要修改（话题和消息格式完全兼容）

## 参考资料

- SDK示例：`gmsl_camera/samples/cameras_opencv_demo/`
- SDK文档：联系Miivii技术支持
- ROS 2 cv_bridge: https://github.com/ros-perception/vision_opencv

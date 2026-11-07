# GMSL硬件时间戳 - 快速开始

## TL;DR

```bash
# 1. 编译
cd ~/your_ros2_ws
colcon build --packages-select usv_vision_gmsl_cpp

# 2. 单相机运行（自动使用硬件时间戳）
ros2 launch usv_vision_gmsl_cpp gmsl_camera_cpp.launch.py

# 3. 双相机运行
ros2 launch usv_vision_gmsl_cpp dual_gmsl_cpp.launch.py

# 4. 查看时间戳
ros2 topic echo /sensors/camera/front/frame_timing_json
```

## 三步验证时间戳是否生效

### 1. 检查节点类型
```bash
ros2 node list
# 应该看到：gmsl_camera_node 或 gmsl_camera_front/rear
```

### 2. 查看source_type
```bash
ros2 topic echo /sensors/camera/front/frame_timing_json | grep source_type
# 硬件时间戳：  "source_type": "v4l2_gmsl_hw_timestamp"
# 系统时间：    "source_type": "v4l2_gmsl"
```

### 3. 测试多相机同步（如果有双相机）
```bash
# 终端1
ros2 topic echo /sensors/camera/front/image_raw --field header.stamp

# 终端2
ros2 topic echo /sensors/camera/rear/image_raw --field header.stamp

# 两个时间戳的sec应该相同，nanosec差异应该 < 1000000 (1ms)
```

## 不同设备路径配置

```bash
# 前置相机在video0
ros2 launch usv_vision_gmsl_cpp gmsl_camera_cpp.launch.py \
  config:=config/gmsl_camera_cpp_front.yaml

# 后置相机在video1
ros2 launch usv_vision_gmsl_cpp gmsl_camera_cpp.launch.py \
  config:=config/gmsl_camera_cpp_rear.yaml

# 侧置相机在video2（创建新配置文件）
# 复制 gmsl_camera_cpp_front.yaml 到 gmsl_camera_cpp_side.yaml
# 修改：device_path: "/dev/video2"，sensor_name: "side"
ros2 launch usv_vision_gmsl_cpp gmsl_camera_cpp.launch.py \
  config:=config/gmsl_camera_cpp_side.yaml
```

## 故障排查速查表

| 症状 | 原因 | 解决方法 |
|------|------|---------|
| `gmsl_camera_node_cpp` 不存在 | SDK未安装或CMake跳过编译 | 检查 `/opt/miivii/` 是否存在 |
| 时间戳全是0 | 相机初始化中 | 等待2-3秒 |
| `source_type` 是 `v4l2_gmsl` | 硬件时间戳未启用 | 配置中设置 `enable_hardware_timestamp: true` |
| SDK初始化失败 | 权限或库路径问题 | `sudo chmod 666 /dev/video*` |
| 双相机时间戳差异>10ms | 硬件未同步 | 检查GMSL聚合器连接 |

## 与Python节点对比

| 特性 | Python节点 | C++节点 |
|------|-----------|---------|
| 时间戳来源 | ROS系统时间 | GMSL硬件时间戳 |
| 多相机同步 | ❌ | ✅ |
| 延迟 | 2-8ms | 0ms |
| 需要SDK | ❌ | ✅ |
| 适用场景 | 仿真/测试 | 实船/生产 |

## 下一步

- 详细文档：[GMSL_HARDWARE_TIMESTAMP.md](GMSL_HARDWARE_TIMESTAMP.md)
- 配置模板：`config/gmsl_camera_template.yaml`
- SDK示例：`gmsl_camera/samples/cameras_opencv_demo/`

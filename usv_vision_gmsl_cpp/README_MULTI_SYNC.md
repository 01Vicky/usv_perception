# GMSL Multi-Camera Sync Node - 使用说明

## 📋 新增文件清单

### 1. 核心节点
- `src/gmsl_multi_camera_sync_node.cpp` - 多相机硬件同步节点（单进程控制多相机）

### 2. 配置文件
- `config/gmsl_multi_sync.yaml` - 相机列表配置（支持 2-8 路动态扩展）

### 3. 启动文件
- `launch/gmsl_sync_multi.launch.py` - 多相机同步启动脚本

### 4. CMakeLists.txt
- 已更新，添加 `gmsl_multi_camera_sync_node` 可执行文件编译

---

## 🔧 两个节点的区别

| 特性 | `gmsl_camera_node_cpp` | `gmsl_multi_camera_sync_node` |
|---|---|---|
| **用途** | 单相机采集 | 多相机硬件同步 |
| **进程数** | 每相机一个进程 | 所有相机共用一个进程 |
| **同步方式** | 各自独立（异步） | 硬件触发同步（微秒级） |
| **适用场景** | 单相机、测试 | 多相机融合、对时间敏感的应用 |
| **配置方式** | YAML 参数 | YAML 相机列表数组 |

---

## 🚀 使用方法

### 1. 编译
```bash
cd /media/nvidia/nvme0n1p1/USV_ROS
colcon build --packages-select usv_vision_gmsl_cpp --symlink-install
source install/setup.bash
```

### 2. 启动多相机同步节点
```bash
ros2 launch usv_vision_gmsl_cpp gmsl_sync_multi.launch.py
```

### 3. 验证同步效果
```bash
# 另一个终端
python3 /media/nvidia/nvme0n1p1/USV_ROS/scripts/check_multi_sync.py
```

**预期结果**：`Max diff < 1ms` ✅（微秒级硬件同步）

---

## ⚙️ 配置扩展

### 当前配置（2 相机）
```yaml
cameras:
  devices: ["/dev/video0", "/dev/video1"]
  sensor_names: ["front", "rear"]
  topics: ["/sensors/camera/front/image_raw", "/sensors/camera/rear/image_raw"]
```

### 扩展到 3 相机
编辑 `config/gmsl_multi_sync.yaml`：
```yaml
cameras:
  devices: ["/dev/video0", "/dev/video1", "/dev/video2"]
  sensor_names: ["front", "rear", "left"]
  topics: ["/sensors/camera/front/image_raw", "/sensors/camera/rear/image_raw", "/sensors/camera/left/image_raw"]
```

重新启动即可，代码无需修改。

---

## 📊 topic 输出

启动后发布以下 topic（根据配置）：
- `/sensors/camera/front/image_raw` (video0)
- `/sensors/camera/rear/image_raw` (video1)

每个 topic 携带硬件时间戳，确保多相机时间对齐。

---

## 🔍 故障排查

### 问题：只有一个相机有图像
**原因**：设备路径配置错误或相机未连接

**解决**：
```bash
ls -l /dev/video*  # 检查设备是否存在
v4l2-ctl -d /dev/video0 --all  # 验证设备可用
```

### 问题：时间戳差异 > 10ms
**原因**：硬件同步未生效（配置错误或 SDK 版本问题）

**解决**：
1. 检查日志中的 `bit_draw` 和 `bit_mask` 是否正确
2. 确认所有相机收到 `INIT_DONE` 消息
3. 联系 Miivii 技术支持

---

## 💡 下一步

拷贝整个 `usv_vision_gmsl_cpp` 功能包到 Jetson 后：
1. 编译
2. 启动 `gmsl_sync_multi.launch.py`
3. 运行同步验证脚本
4. 确认 `Max diff < 1ms`

成功后即可用于多传感器融合。

# Jetson 编译验证 Checklist

本轮改动（camera_info 三级兜底 + config 修正 + cpp 残留清理）均在**无 Miivii SDK、无 ROS**
的开发机上完成，C++ 节点未经过 `colcon build`。加载器解析逻辑已用脱-ROS stub 单测验证，
但完整编译与运行必须在装了 Miivii SDK 的 Jetson 上核对。按下面逐条过。

改动涉及文件：
- `include/usv_vision_gmsl_cpp/camera_info_loader.hpp`（新增）
- `src/gmsl_camera_node_cpp.cpp`
- `src/gmsl_multi_camera_sync_node.cpp`
- `CMakeLists.txt`
- `config/gmsl_camera_template.yaml`、`gmsl_camera_video0.yaml`、`gmsl_multi_sync.yaml`
- `config/camera_info_sample.json`（新增）、`docs/CAMERA_INFO.md`（新增）

---

## 0. 前置环境

逐项检查，**缺了就按右侧补救命令装**。所有 `apt install` 前先 `sudo apt update`。

### 0a. Miivii GMSL SDK（C++ 节点硬依赖）
- [ ] 检查：`ls /opt/miivii/include/MvGmslCamera.h /opt/miivii/lib/libmvgmslcamera.so`
- 缺失后果：CMake 打印 `Skipping C++ node build`，只装 launch/config，**编不出节点**
- 补救：SDK 不在 apt 源里，随 Miivii 板卡/BSP 提供，不能 apt 装。按厂商包安装到 `/opt/miivii/`：
  ```bash
  # 典型：厂商给的是 .deb 或 tar 包，安装/解压后确认路径
  sudo dpkg -i miivii-gmsl-sdk_*.deb          # 若是 deb
  # 或解压厂商 tar 包到 /opt/miivii/
  ls /opt/miivii/include /opt/miivii/lib      # 确认头文件与 .so 到位
  ```
- 若 SDK 装在非 `/opt/miivii` 路径：改 `CMakeLists.txt` 顶部的
  `MIIVII_INCLUDE_DIR` / `MIIVII_LIB_DIR` 两个变量指向实际位置

### 0b. nlohmann-json（标定加载器解析 JSON 用）
- [ ] 检查：`ls /usr/include/nlohmann/json.hpp` 或 `dpkg -l | grep nlohmann`
- 补救：
  ```bash
  sudo apt install -y nlohmann-json3-dev
  ```
- 注：CMake 里是 `find_package(nlohmann_json QUIET)`，缺失时会告警并尝试硬编译。
  装上最稳妥，避免 `#include <nlohmann/json.hpp>` 找不到头文件。

### 0c. cv_bridge / OpenCV（图像转 ROS 消息用）
- [ ] 检查：`ros2 pkg prefix cv_bridge` 有输出；`dpkg -l | grep libopencv-dev`
- 补救：
  ```bash
  sudo apt install -y ros-humble-cv-bridge libopencv-dev
  ```

### 0d. ROS 2 Humble 环境
- [ ] 检查：`printenv ROS_DISTRO` 输出 `humble`；`which colcon` 有输出
- 补救：
  ```bash
  source /opt/ros/humble/setup.bash           # 每个新终端都要 source
  sudo apt install -y python3-colcon-common-extensions   # 若无 colcon
  ```

## 1. 编译

在工作空间根目录（含 `src/` 的那层）执行：
```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select usv_vision_gmsl_cpp
source install/setup.bash
```

- [ ] `colcon build` 成功
- [ ] 编译输出里出现 `Miivii GMSL SDK found ... building C++ node`
      （若是 `Skipping C++ node build` 说明 SDK 没找到，回到第 0a 步）
- [ ] 两个可执行文件都生成：
      `ls install/usv_vision_gmsl_cpp/lib/usv_vision_gmsl_cpp/{gmsl_camera_node_cpp,gmsl_multi_camera_sync_node}`
- [ ] 头文件被安装：`ls install/usv_vision_gmsl_cpp/include/usv_vision_gmsl_cpp/camera_info_loader.hpp`
- [ ] 无新增编译告警（重点关注 `-Wall -Wextra -Wpedantic` 下的未使用变量、类型转换）

编译失败的常见补救：
```bash
# 改了 CMakeLists / 加了新文件后，缓存没刷新导致的怪错误：清干净重编
rm -rf build/usv_vision_gmsl_cpp install/usv_vision_gmsl_cpp
colcon build --packages-select usv_vision_gmsl_cpp

# 看完整报错（默认输出被吞时）
colcon build --packages-select usv_vision_gmsl_cpp --event-handlers console_direct+
```

## 2. 单相机节点 —— camera_info 三级兜底

`source install/setup.bash` 后逐个验证：

### 2a. 空标定（默认，无 url 无 fov）
- [ ] `ros2 launch usv_vision_gmsl_cpp gmsl_camera_cpp.launch.py` 正常出图
- [ ] `ros2 topic echo /sensors/camera/front/camera_info --once`：
      `k` 全 0、`distortion_model` 为空、`width/height` 等于实际出图尺寸
- [ ] 日志**无** `loaded camera info` 或 `FOV fallback` 字样

### 2b. 标定文件（最高优先级）
- [ ] 在 config 里设 `camera_info_url: "<绝对路径>/camera_info_sample.json"` 重启
- [ ] 日志出现 `loaded camera info: <path>`
- [ ] `camera_info` 的 `k[0]/k[4]/k[2]/k[5]` = 文件里的 fx/fy/cx/cy（出图分辨率与标定一致时）
- [ ] `d` 有 5 个值、`distortion_model` = `plumb_bob`

### 2c. 分辨率缩放
- [ ] 标定文件 `resolution` 与实际出图分辨率不同（如标定 1920x1080、出图 1920x1536）
- [ ] 日志出现 `camera info resized from 1920x1080 to 1920x1536`
- [ ] `k[4]`(fy)、`k[5]`(cy) 按高度比缩放；`k[0]`(fx)、`k[2]`(cx) 因宽度相同不变

### 2d. FOV 兜底（无标定文件时）
- [ ] `camera_info_url: ""` 且设 `fov_x_deg: 135.68`、`fov_y_deg: 108.97` 重启
- [ ] 日志出现 `no calibration file, using spec-sheet FOV fallback: ...`
- [ ] `k[0]/k[4]` ≈ 反算值，`k[2]/k[5]` = 图像中心(width/2, height/2)，`distortion_model` = `plumb_bob`

### 2e. 坏路径不崩
- [ ] `camera_info_url` 指向不存在的文件 → 日志 ERROR/降级，节点**继续出图**不退出

## 3. 多相机同步节点 —— camera_info

- [ ] `ros2 launch usv_vision_gmsl_cpp gmsl_sync_multi.launch.py`（或对应 multi 的 launch）启动
- [ ] 每路相机都出现 `Created publisher: <image_topic> (+<info_topic>)`
- [ ] 每路的 `camera_info` 话题名 = image 话题把末段 `image_raw` 换成 `camera_info`
      （如 `/sensors/camera/front/camera_info`）
- [ ] `ros2 topic hz` 确认 image 与 camera_info 帧率一致
- [ ] `camera_info.header.stamp` 与同帧 image 的 `stamp` 相同（同源硬件时间戳）
- [ ] 不配任何标定数组时：所有 camera_info 为空标定，节点正常
- [ ] 配 `cameras.camera_info_urls`（部分留 `""`）：有 url 的路加载成功、留空的路空标定，互不影响
- [ ] `camera_info_urls` 数组比 `devices` 短时不崩（缺的按空标定）

## 4. 时间基准（GMSL 接入核心，硬件时间戳专有）

- [ ] 用硬件时间戳前，PTP 已同步：确认 `2_production/ptp_sync` 的 ptp4l/phc2sys 在跑
- [ ] `camera_info.header.stamp` / image `stamp` 与其它走 ROS 时间的传感器在**同一时间基准**
      （系统时钟未与 PHC 对齐时，硬件时间戳会与 ROS 时间源整体错位）

## 5. 下游对接（与 usv_vision 主功能包）

- [ ] `usv_vision` 下游节点（订阅 `/sensors/camera/{name}/image_raw` + `camera_info`）
      能同时收到两个 C++ 节点发的话题，无需改下游代码
- [ ] 依赖内参的下游（去畸变 / 单目测距 / azimuth）在 2b 标定生效后拿到非零 K

## 6. 配置文档一致性（无需编译，抽查）

- [ ] `gmsl_camera_template.yaml` 列出的参数节点都认（无失效项误导）
- [ ] `gmsl_camera_video0.yaml` 用的是 `enable_hardware_timestamp`（非旧的 `enable_hw_timestamp`）
- [ ] `usv_vision/README.md` 的 GMSL C++ 启动命令是 `ros2 launch usv_vision_gmsl_cpp ...`

---

## 附：出问题时的快速定位与补救

| 现象 | 可能原因 | 补救 |
|---|---|---|
| CMake 报 `Skipping C++ node build` | `/opt/miivii` 缺失或路径不对 | 装 SDK（见 0a）；或改 CMakeLists 的 `MIIVII_INCLUDE_DIR`/`MIIVII_LIB_DIR` |
| 编译报找不到 `nlohmann/json.hpp` | 未装 nlohmann-json | `sudo apt install -y nlohmann-json3-dev` |
| 编译报找不到 `camera_info_loader.hpp` | include 路径没生效 / 旧缓存 | 确认 `CMakeLists` 含 `${CMAKE_CURRENT_SOURCE_DIR}/include`，然后清缓存重编（见第 1 节） |
| 运行时报 `libmvgmslcamera.so: cannot open shared object` | 动态库不在链接路径 | `echo /opt/miivii/lib | sudo tee /etc/ld.so.conf.d/miivii.conf && sudo ldconfig` |
| `ros2 launch` 报找不到包 | 没 source overlay | `source install/setup.bash`（每个新终端都要） |
| 打不开 `/dev/videoX` / 权限拒绝 | 设备权限 | `sudo chmod 666 /dev/video*`（临时）；或把用户加入 `video` 组 |
| camera_info 一直空标定 | url 写成 `file://` 格式（应为普通绝对路径）；或文件不存在 | 看节点 ERROR 日志确认路径；改成绝对文件路径 |
| FOV 兜底不生效 | `camera_info_url` 非空（标定优先级更高会跳过 FOV）；或 fov 值为 0 | 清空 url 或确认标定文件确实缺失；填非 0 的 fov 度数 |
| 时间戳与其它传感器错位 | PTP/phc2sys 未同步 | 起同步：`cd 2_production/ptp_sync && ./setup_ptp_sync.sh`（见第 4 节） |

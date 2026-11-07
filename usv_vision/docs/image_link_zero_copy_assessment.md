> ⚠️ **本文件中的吞吐数字已作废**（+44%/+57% 来自启动预热期取样）。权威数据见 `docs/image_link_ab_results.md` 第九节（统一口径实测）。

# 图像链路评估：零拷贝与按需分发

> 本文档记录"图像消息从驱动到识别能否零拷贝"这一问题的评估过程、结论与改造方案，
> 供后续维护与复盘使用。文中"待板上实测"的数字来自 x86 开发机的同构对照实验，仅作结构性参考。

## 一、问题与背景

图像是当前系统里数据量最大的模态。最初的提问集中在三件事上：

1. 图像消息从驱动到识别，是不是零拷贝传输的？
2. 是否必须经过 DDS 传输？
3. 如果 rviz 没有观察需求，是否可以改成零拷贝？

评估后得到的结论与原始设想不同：**"让 DDS 不拷贝"这条路在 ROS 2 上走不通，真正的收益
来自"别让同一帧被搬三遍"——即采集端按消费者需求分发，而不是生产单一"万能规格"再广播。**

## 二、链路现状与拷贝账

单帧 1920×1536 bgr8（8.4 MiB）在链路内的搬运量，按消费者分段统计：

| 段 | 环节 | 流量 (MB) |
|---|---|---|
| 采集发布端 | SDK UYVY→BGRA32 | 17.7 |
| | `cvtColor` BGRA→BGR | 20.6 |
| | `cv_bridge::toImageMsg` 分配+memcpy | 17.6 |
| | DDS 写给 2 个 reader（各分配+写入） | 35.2 |
| | **小计** | **≈91** |
| 识别端 | DDS 读出 + `imgmsg_to_cv2` + 蒙板 copy + letterbox | **≈63** |
| 推流端 | DDS 读出 | 17.6 |
| | `bytes(msg.data)` | 17.6 |
| | `Gst.Buffer.new_wrapped` | 17.6 |
| | `videoconvert` BGR→BGRx | 20.6 |
| | `nvvidconv` 上传 NVMM | 17.7 |
| | **小计** | **≈91** |
| **合计** | | **≈245 MB/帧/相机** |

两路 30fps ≈ **14.7 GB/s**，另有每帧 4 次 8.4 MB 大块分配。

结构性观察：

- 一份全分辨率帧被当成"通用货币"，识别、推流各从 DDS 捞一遍再各自转换；
- 识别只需要 640（0.41 MP），却承担 2.9 MP 的搬运（约 7 倍）；
- 推流与采集在同一台机器、同一进程内可完成，却要绕 DDS 回环一次；
- 全分辨率帧经历 3 次 CPU 色彩转换（SDK BGRA、`cvtColor` BGR、`videoconvert` BGRx）。

## 三、为什么不做 DDS 级零拷贝

| 路线 | 可行性 | 说明 |
|---|---|---|
| RMW loaned message（官方零拷贝） | ❌ | 仅支持定长/POD 类型。实测 Humble + `rmw_fastrtps_cpp` 下 `sensor_msgs/Image` 的 `can_loan_messages()` 返回 **false**（`std_msgs/String` 同样为 false）。换 iceoryx / CycloneDDS 不改变该限制。 |
| intra-process + `unique_ptr` 发布 | ❌（当前架构） | 要求发布者与订阅者同进程同语言；采集是 C++、推理是 Python(ultralytics)。 |
| 自建 SHM 帧池 | ⭕ 可行但成本高 | 需自建传输层 + seqlock 防撕裂 + 两侧改造；覆盖语义与 reliable 冲突；`image_raw` 仍须为推流保留。 |
| NVMM/DMABUF 直通 TensorRT | ⭕ 收益最大 | 需 C++/CUDA 推理实现，属于技术栈替换，另立项目。 |

另核实：rviz2（Humble）的 Image display 自带 Reliability 选项，可设 `Best effort`，
因此"主链必须 reliable 才能被 rviz 观察"这个前提是不成立的。**不过最终仍选择发布端
reliable**：这样 rviz 用默认 QoS 就能看图，省掉现场"忘记改 Reliability 导致没画面"的坑；
订阅端（识别/推流）用 best_effort，兼容方向成立。

## 四、QoS 取舍（带实测）

同构对照实验：1920×1536 bgr8、发布端 30fps、订阅端模拟每帧处理耗时；数据单位 ms。

**订阅端处理得过来（25ms/帧）**

| 配置 | 处理帧数 | 平均 | 最大 |
|---|---|---|---|
| pub rel/d10 + sub best_effort/d1 | 85/90 | 8.3 | 17.8 |
| pub rel/d10 + sub reliable/d10 | 90/90 | 10.8 | 35.4 |
| pub best_effort/d1 + sub best_effort/d1 | 48/90 | 8.4 | 18.3 |

**订阅端处理不过来（45ms/帧）**

| 配置 | 处理帧数 | 平均 | 最大 |
|---|---|---|---|
| pub rel/d10 + sub best_effort/d1 | 56/90 | 23.3 | 49 |
| **pub rel/d10 + sub reliable/d10** | 70/90 | **292.3** | **439** |
| pub rel/d2 + sub best_effort/d1 | 53/90 | 24.1 | 47 |
| pub rel/d10 + sub reliable/d2 | 63/90 | 54.3 | 79.7 |
| pub best_effort/d1 + sub best_effort/d1 | 43/90 | 24.5 | 45 |

**结论**

1. 订阅端用 reliable 会"多拿到帧"，但拿到的是**过期帧**：过载时端到端延迟从 23ms 涨到 292ms。
   图像是"最新帧优先"（latest-wins）数据，延迟比帧率更重要，故订阅端统一保持 `best_effort/depth 1`。
2. 发布端 `depth 10` 相对 `depth 2` 几乎无收益（56 帧/23.3ms vs 53 帧/24.1ms），
   但每相机多占约 67MB，故收敛为 `depth 2`。
3. 实测 `publish()` 并未出现阻塞（平均 5ms、最大 9~14ms、0 次 >33ms），
   大缓冲的代价体现为订阅端延迟累积，而非采集线程被卡。

## 五、改造方案：按需分发

采集端一次成帧，在**同一进程内**按消费者需求分发两种规格：

| 话题 | 尺寸 | QoS | 消费者 |
|---|---|---|---|
| `image_raw` + `camera_info` | 全分辨率 | 由 `image_qos_*` 决定（当前 reliable / depth2） | 推流/录像等"要原图"的消费方 |
| `image_algo` + `camera_info_algo` | `algo_width`×`algo_height`（默认 640） | **与 `image_raw` 同一套**（当前 reliable / depth2） | `yolo_infer_node` 识别 + rviz 人工观察 |

设计要点：

- 两者共用同一份已转换好的 BGR 帧，差别只在一次 `cv::resize`；
- `camera_info` 由同一个 `CameraInfoLoader` 提供，按分辨率缓存并自动缩放内参，
  算法图内参天然与算法图分辨率一致，无需重做标定；
- 算法图在 `get_subscription_count() == 0` 时不 resize、不分配、不发布，无人订阅时零开销；
- **QoS 策略**：发布端由 `image_qos_*` 统一控制（当前 reliable/depth2），订阅端一律
  `best_effort/depth1`。DDS 兼容方向是"提供强于等于请求"，因此 reliable 发布 + best_effort
  订阅成立；好处是 rviz 用默认 QoS 即可观察，不必记得改 Reliability。
  反压风险已用实测排除：8.4MB 帧 × depth10 × 2 个订阅者时 `publish()` 平均 5ms、
  最大 9~14ms、0 次超 33ms；1.2MB × depth2 的余量更大。板上仍建议做一次"开/关 rviz"对照，
  指标看 `image_raw` 的 hz 与 `frame_timing_json` 的 `driver_latency_ms`；若出现掉速，
  把 `image_qos_reliability` 改回 `best_effort` 即回退（代价是 rviz 需手动改 Reliability）。
- **不单开"调试图"话题**：调试图与算法图是同一份像素，只是帧率不同；合并后少一条话题、
  少一套 QoS 语义。需要看检测框叠加时用 `yolo_infer_node` 的 `annotated` 话题即可。
  代价：观察端不能单独限频（需要时把 rviz 放到操作端，或临时下调 `algo_width/height`）。
- 算法图尺寸设为 ≤0 或与原始尺寸相同时不缩放，直接复用原图；
- 识别侧新增 `reference_width/reference_height`（基准 = 原生分辨率）：对外 bbox 坐标与
  `pixel_width` 乘 `ref/input`，`min_box_width_px`/`min_box_height_px` 除之，面积比类参数天然不变，
  因此**下游消费者的坐标基准不变**；
- 蒙板图像按帧尺寸自动 resize（INTER_NEAREST），无需重画。

## 六、已实施内容

驱动 `usv_vision_gmsl_cpp`：

- 新增 `include/usv_vision_gmsl_cpp/camera_image_publishers.hpp`（header-only，单/多相机节点共用）
- `src/gmsl_camera_node_cpp.cpp`：接入分发器；`frame_timing_json` 增加 `image_algo`/`image_raw`
  实际产出字段；`image_qos_depth` 默认 10→2
- `src/gmsl_multi_camera_sync_node.cpp`：同样接入；depth 参数化默认 2；`image_algo`/`camera_info_algo`
  话题由 `image_raw` 自动推导
- `config/`：front / template / multi_sync 三份配置补参数与注释
- 新增参数：`enable_algo_stream`、`algo_width`、`algo_height`

识别 `usv_vision`：

- `common/topics.py`：新增 `algo_image_topic` / `algo_camera_info_topic`
- `yolo_infer_node.py`：新增 `use_algo_image`（默认 true）、`reference_width/height`（默认 0＝旧行为）
- `config/yolo_infer.yaml`：`use_algo_image: true`、`reference_width/height: 1920/1536`
- `launch/jetson_gmsl_full.launch.py`：新增 `algo_stream` 统一开关
- `launch/usv_vision.launch.py`：该链路使用 Python 采集节点（不做分发），显式 `use_algo_image: false`

**回滚开关**：launch 传 `algo_stream:=false`（识别退回 `image_raw`）／配置 `enable_algo_stream: false`／
`image_qos_depth: 10`（恢复旧 QoS）。

## 七、收益：板上实测 + 改造前预估

> **实测结论（2026-11-07，Jetson AGX Orin，见 [onboard_ab_test_20261107.md](onboard_ab_test_20261107.md)）**：
> - 总 CPU 基本持平：**95.9% → 91.2%**（单核基准），不是"省 CPU"的改造；
> - 真正收益是吞吐：`detections` **17.6 Hz → 25.4~27.6 Hz（+44%~+57%）**；
> - 成本位置：采集端 **+13.6pp**（多发一路 1.2MB 的发布路径），识别端 **−18.4pp**（不再搬 8.8MB、不再做 2.9MP 缩放）；
> - 反压安全：rosbag 作可靠读者并 `SIGSTOP` 冻结时，采集出帧率仍 29.0 Hz；
> - 蒙板另算：开启后识别 **+15.6pp**、吞吐降到 22 Hz、单帧 infer +10ms → **默认关闭**。

以下为改造前按拷贝账做的**预估**（供对照，非实测）：

| 指标 | 改造前 | 改造后（预估） |
|---|---|---|
| 单帧搬运量 | ≈245 MB | ≈80 MB（-67%） |
| 两路 30fps 聚合 | ≈14.7 GB/s | ≈4.8 GB/s |
| 识别端单帧 | ≈63 MB | ≈5 MB |
| 每帧 8.4MB 大块分配 | 4 次 | 0 次 |
| 发布端缓冲/相机 | 84 MB | 17 MB |

剩余 ≈80 MB 中已无"可被零拷贝技术省掉"的部分：SDK 一次格式转换、一次 GPU 上传、
一次 640 缩放、一次 1.2 MB 小图过 DDS，均为物理必需。进一步压缩需靠下文第 9 节的 M3/M4。

## 八、板上验证方法

**一键验收（推荐）**：`usv_vision/scripts/jetson_image_chain_test.sh`——自动起 pipeline、采
hz/bw/frame_timing/detections/tegrastats、跑 A/B/C 反压验证，并把结果写成
`summary.md` + `qos_backpressure.md`，目录可直接回传。

```bash
bash usv_vision/scripts/jetson_image_chain_test.sh --sensor front --duration 30
bash usv_vision/scripts/jetson_image_chain_test.sh --no-launch --skip-qos   # 只采指标
bash usv_vision/scripts/jetson_image_chain_test.sh --dry-run                # 先看计划
```

注意：A 组基线要求 **rviz 未启动**；脚本会在 B 组提示你打开 rviz 并等待，C 组自己
`SIGSTOP`/`SIGCONT` 模拟 GUI 卡死。退出码 0 通过 / 2 环境或数据问题 / 3 反压验证失败。

手动分步验证：
```bash
colcon build --packages-select usv_vision_gmsl_cpp usv_vision && source install/setup.bash
ros2 launch usv_vision jetson_gmsl_full.launch.py

ros2 topic hz  /sensors/camera/front/image_algo        # 期望 ≈30Hz
ros2 topic bw  /sensors/camera/front/image_algo        # 期望 ≈35 MB/s（640 BGR）
ros2 topic hz  /sensors/camera/front/image_raw
ros2 topic bw  /sensors/camera/front/image_raw         # 期望 ≈265 MB/s（1920x1536）
ros2 topic echo /sensors/camera/front/frame_timing_json --once   # 核对 image_algo.published/width
ros2 topic hz  /perception/camera/front/detections
jtop                                                    # CPU 占用、内存
```

识别节点日志中的 `decode=/infer=/post=/total=` 与 `frame_timing_json` 的 `driver_latency_ms`
是改造前后对比的关键指标。

**「可靠发布 + GUI 订阅」是否反压采集线程**——用现成脚本做 A/B/C 三组对照：

```bash
ros2 run usv_vision check_image_qos --sensor front            # 交互式，按提示依次 A→B→C
ros2 run usv_vision check_image_qos --sensor front --auto      # 按打印的提示自行操作 rviz
ros2 run usv_vision check_image_qos --phase A --record qos.md  # 只测基线并留档
```

- A 基线（不开 rviz）／B（rviz 正常订阅 `image_algo`）／C（把 rviz 冻住：`Ctrl+Z` 或
  `kill -STOP`，采样后 `fg` 恢复）——**C 才是关键**：正常 rviz 会及时 ack，测不出理论反压。
- 指标取发布端自带的 `frame_timing_json`：相邻 `capture_stamp_ros` 之差＝真实采集周期，
  `driver_latency_ms`＝捕获→发布完成，正好包住两次 `publish()`，因此不受本脚本收包节奏影响。
- 判据：采集周期下降 <1.5 帧、`driver_latency_ms` p95 增量 <2ms 视为无影响；
  >3 帧或 p95 +>5ms 或多见 >50ms 尖峰视为受影响（脚本会直接给结论与退出码）。
- 不要用 CPU 占用判断：阻塞是等待，CPU 反而会降。

## 九、后续

- **M3：推流编码链移入 C++ 采集进程**（同进程 appsrc，独立推流线程 + `block=false`）。
  预计再省 70~90 MB/帧，为单项收益最大者；需在板上重调编码参数，待本版基线出来后再上。
- **M4：全分辨率走 UYVY 直通**。若 Miivii SDK 的 `output_fmt_str` 支持 UYVY/NV12，
  可由 GPU 完成色彩转换并省掉 `videoconvert`，全分辨率帧从 8.8MB 降到 5.9MB。
  需先用小探针验证 SDK 支持范围。

## 十、附注：本机静态检查

本仓库开发机为 x86，无 Miivii SDK 与 ARM 环境，不做整包构建。改动通过以下方式静态验证：

- Python：`py_compile` 与 YAML 解析；
- C++：以桩 `MvGmslCamera.h` 提供 SDK 类型，将真实 `.cpp` 编为 STATIC 库（不链接即无需真 SDK），
  `-Wall -Wextra -Wpedantic` 下 0 告警；新增 header 另建 TU 触发全部模板实例化。

**这不替代板上构建验证**——`MvGmslCamera.h` 的真实接口若与桩不同，仍可能在板上暴露问题。

## 十一、附录：对外沟通口径

本节用于对外/对上说明本项目，与正文（内部留档、含全部实测与风险）分开。

**主打表述**

> 图像链路由"单一全分辨率话题供所有下游使用"改为"采集端一次成帧、按消费者需求分发"，
> 减少冗余搬运：全分辨率给推流、算法尺寸给识别与人工观察共用。单帧链路搬运量降到
> 原来约三分之一，识别端降到约十分之一（估算，以板上实测为准）。

**可公开的技术结论**

1. ROS 2 的零拷贝（loaned message）仅支持固定尺寸消息类型；`sensor_msgs/Image` 的
   `data` 为无界变长数组，不满足条件，更换 DDS 实现同样不满足。
2. `rclpy` 未提供 loan API，因此 Python 节点无法参与该机制。
3. 因此本系统收益来自"减少冗余搬运"（按需分发 / 同进程直通 / 共享内存帧池），
   而非"DDS 零拷贝"。
4. QoS 按角色分：发布端 reliable（保证 rviz/录像等要完整画面的用途开箱可用），
   订阅端 best_effort + 小缓冲（只取最新帧，不积压旧帧）。

**避免的说法与替代**

| 避免 | 替代 |
|---|---|
| 现在被搬三遍、浪费 | 原来下游各自取全分辨率数据，供需不完全匹配 |
| 不是零拷贝、做不到 | 官方零拷贝机制不适用于图像类型，改用按需分发 |
| 延迟从 23ms 涨到 292ms | 订阅端改用 best_effort 后只取最新帧，不再堆积旧帧 |
| 配置有问题 / 之前做错了 | 早期以跑通为主，这次把分发与 QoS 一起收敛 |

**待实测后补充的对外数字**

- `image_algo` 的 `ros2 topic hz` / `bw`
- 识别节点 `total=` 与 `frame_timing_json` 的 `driver_latency_ms` 改造前后对比
- `jtop` 的 CPU 占用与内存变化


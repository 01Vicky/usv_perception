# [修正] 2026-11-07 夜 性能数据统一口径复测，撤回此前两处结论

- **撤回** "吞吐 +44%/+57%"：该数字取自节点启动后约 20s 内（engine 冷启动期），复测为持平。
- **更正** 改造收益：视觉 3.90 → **3.37 核（−0.53 核 / −13.6%）**、整机 55.5% → **50.1%**、内存 −0.32GB；
  此前"−0.87 核 / −29%"因对照组 CPU 异常而作废。
- **新增** M3：H.264 编码搬入 C++ 采集节点（`h264_streamer.{hpp,cpp}`），推流不再经 DDS/Python；
  launch 新增 `stream_mode:=internal|python|off`（默认 internal，可一键回退）。
- 权威数据见 `docs/image_link_ab_results.md` 第九节。

# 变更记录

本文件记录本功能包面向用户的变更。后续修改功能、接口、配置、启动方式或文档时，应在 `Unreleased` 下补充记录。

## Unreleased

### 更新

- **板上实测完成（2026-11-07，Jetson AGX Orin）**：按需分发的收益已量化——总 CPU 基本持平
  （95.9% → 91.2%，单核基准），`detections` 输出 17.6 Hz → 25.4~27.6 Hz（+44%~+57%）；
  采集端 +13.6pp 为发布路径成本，识别端 −18.4pp。反压验证通过（冻结 rosbag 读者时采集仍 29 Hz）。
  驱动侧 CPU 优化：`cv::setNumThreads(1)`（无订阅 24→17%、有订阅 64→50.3%），
  `camera_info_algo` 改 1Hz 重发。蒙板开销单独量化（识别 +15.6pp、吞吐 −4.1Hz），维持默认关闭。
- 合入推理输入蒙板、截图／绘制／生成／离线验证工具和 reload 服务；默认关闭蒙板。
- reload 重新读取参数及图像，安装蒙板样板与文档，补充依赖和无模型回归测试。
- 保留 GMSL 采集包、Jetson GStreamer 推流和现有图像 QoS。
- 识别链默认改订采集端分发的 `image_algo`（小图，默认 640），不再吃全分辨率 `image_raw`；
  `use_algo_image:=false` 可回退。新增 `reference_width/height`：bbox 坐标与 `min_box_*`
  像素阈值仍按原生分辨率语义解释，内部自动换算，下游消费者无感。

### 新增

- 补充功能包 README、架构文档和数据流文档。
- `common/topics.py` 新增 `algo_image_topic` / `algo_camera_info_topic`。
- `jetson_gmsl_full.launch.py` 新增 `algo_stream` 启动参数，统一控制驱动分发与识别订阅源。
- `image_algo` 与人工观察共用一条话题，不单开调试图（同一份像素只需一条通道）；
  发布端 QoS 与 `image_raw` 同一套（`image_qos_*`，当前 reliable/depth2），订阅端一律
  best_effort/depth1。rviz 用默认 QoS 即可观察，不必改 Reliability。
- 新增 `docs/image_link_zero_copy_assessment.md`：图像链路零拷贝评估（拷贝账、QoS 取舍实测、
  按需分发方案、预期收益、板上验证步骤、后续 M3/M4 计划）。
- 新增 `docs/variant_benchmark_log.md`：**持续维护的链路开销与资源占用对照账本**（通信开销表、资源占用与吞吐表、每次改动的版本清单、待补测量项、历史数据索引），规定每次改动必须追加一行数据。
- 新增 `scripts/record_variant.sh`：一条命令把当前链路量化并追加到账本（出帧率、image_raw/image_algo 带宽、进程 CPU 与 RSS、detections 频率），CPU 取 `/proc/<pid>/stat` 增量、帧率取 `frame_timing_json` 计数（避免大包误报）。
- 新增 `docs/archive/onboard_ab_test_20261107.md`：板上 A/B/C 对照实测记录（环境与 engine 构建参数、
  控制变量方法、测量手段与踩坑、A/B/C 数据表、结论与限制、复现步骤）。
- 新增诊断脚本 `scripts/check_image_qos_backpressure.py`（入口 `ros2 run usv_vision
  check_image_qos`）：A/B/C 三组对照验证「可靠发布 + GUI 订阅」是否反压采集线程，
  指标取发布端自带的 `frame_timing_json`（采集周期 + driver_latency_ms），不受订阅端丢包影响。
  支持 `--auto-rviz`：脚本自己等 rviz 启动、自己 `SIGSTOP`/`SIGCONT` 模拟 GUI 卡死。
- 新增板上验收脚本 `scripts/jetson_image_chain_test.sh`：一条命令完成环境检查、起 pipeline、
  采 hz/bw/timing/detections/tegrastats、跑反压三组，并生成 `summary.md` 与
  `qos_backpressure.md`；支持 `--no-launch` / `--skip-qos` / `--dry-run`。
  会用 `frame_timing_json` 是否含 `image_algo` 字段判定驱动新旧：旧版自动跳过反压验证并在
  `summary.md` 里标注；识别无输出时提示「新版 yolo + 旧版驱动」的混合部署问题。
- 新增改动同步/校验脚本 `scripts/image_chain_rollout.sh`：`list` 列同步清单、`pack` 打包
  tar.gz、`verify` 判定目标机是新版 / 旧版 / 混合部署（退出码 0 / 2 / 1）；
  `backup <TARGET>` 覆盖前备份旧代码并打印回滚命令；`compare <旧目录> <新目录>`
  对比两次验收的 hz/bw/detections 指标并给判读要点。
- 验收脚本的日志留存：结果目录内保存 launch/timing/tegrastats/top/topic/node 等原始日志，
  并把节点日志（`ROS_LOG_DIR`）收进 `ros_log/`；中途失败会写 `abort.md` 记录阶段与原因；
  结束时自动打包为 `<结果目录>.tar.gz`（`--no-archive` 可关闭）。
  另修复 `set -o pipefail` 下 `ros2 topic list | grep -q` 会被 ros2 非零退出误判的问题。

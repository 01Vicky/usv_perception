# usv_vision 文档索引

## 看哪个

| 想做的事 | 文档 |
|---|---|
| 部署到板子 | [jetson_deploy.md](jetson_deploy.md) |
| 查参数含义、资源影响 | [parameters.md](parameters.md) |
| 查性能数据（唯一权威口径） | [image_link_ab_results.md](image_link_ab_results.md) |
| 跑二阶段视觉测试（4.1.x） | [视觉测试.md](视觉测试.md) |
| 为什么不做 DDS 零拷贝 | [image_link_zero_copy_assessment.md](image_link_zero_copy_assessment.md) |
| 蒙板怎么生成和使用 | [mask_usage.md](mask_usage.md) |
| 架构与图像链路 | [architecture.md](architecture.md) |
| 面向管理层的性能报告 | [视觉链路性能优化报告261107.md](视觉链路性能优化报告261107.md)（基线）、[视觉链路性能优化报告261111.md](视觉链路性能优化报告261111.md)（本轮增量：触发式发布 / 28Hz 定位 / 压缩评估） |
| 变体对照账本（实验记录） | [variant_benchmark_log.md](variant_benchmark_log.md) |
| 早期命令稿、一次性排查记录 | [archive/](archive/)（不要当现行方法用） |

## 脚本清单

### 测试与验收

| 脚本 | 运行位置 | 作用 |
|---|---|---|
| [vision_test_record.sh](../../vision_test/vision_test_record.sh) | 板上 | 录测试消息包 + `param dump` + `node/topic info` + `tegrastats` + `scene.json`，产出给 `vision_eval` 用的目录 |
| [vision_eval.py](../../vision_test/vision_eval.py) | 板上或本机（需 source ROS） | 读 bag，出 4.1.x 判分报告（Markdown，按计划表填空顺序）；`--selftest` 可脱离 ROS 自检判定逻辑 |
| [check_image_qos_backpressure.py](../scripts/check_image_qos_backpressure.py) | 板上 | 验证 reliable 发布 + GUI 订阅是否会反压采集线程（A/B/C 三组） |
| [jetson_image_chain_test.sh](../scripts/jetson_image_chain_test.sh) | 板上 | 图像链路一键验收：前置检查 → 采 hz/带宽/时序 → 反压 → 生成 summary.md |
| [image_chain_rollout.sh](../scripts/image_chain_rollout.sh) | 板上 | 按阶段切换采集/识别配置并验证 |
| [record_variant.sh](../scripts/record_variant.sh) | 板上 | 起 pipeline 跑 10 min 资源采样，写入变体账本 |
| [jetson_collect_benchmarks.sh](../scripts/jetson_collect_benchmarks.sh) | 板上 | 统一性能测量：清理 → 预热 → 3×20s 中位数 → ffprobe 校验推流 |
| [gmsl_health_check.sh](../scripts/gmsl_health_check.sh) | 板上 | GMSL 相机链路健康检查（设备、驱动、出图） |
| [verify_mask_recognition.py](../scripts/verify_mask_recognition.py) | 本机（需 ultralytics） | 同一张图开/关蒙板对比，证明蒙板确实改变推理结果 |

### 蒙板工具

| 脚本 | 作用 |
|---|---|
| [capture_mask_frame.py](../scripts/capture_mask_frame.py) | 抓一帧用于画蒙板 |
| [make_mask.py](../scripts/make_mask.py) | 生成忽略区域蒙板 |
| [draw_mask.py](../scripts/draw_mask.py) | 在图上叠加显示蒙板，检查覆盖 |

### 推流脚本（仅 `stream_mode:=python` 回退路径用）

| 脚本 | 作用 |
|---|---|
| [gmsl_to_rtsp.sh](../scripts/gmsl_to_rtsp.sh) | v4l2 → H.264 → RTSP，参数可调 |
| [gmsl_to_rtmp.sh](../scripts/gmsl_to_rtmp.sh) | 同上，推 RTMP |

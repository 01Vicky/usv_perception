# 归档：早期 bring-up 与一次性命令记录

这些是 2026-10 到 11-07 期间在板上调试时留下的命令稿和终端输出，保留只为查历史，
**不是现行方法**。要看现行用法请回上一级：

| 需要什么 | 看哪个 |
|---|---|
| 现在的部署步骤 | `../jetson_deploy.md` |
| 参数含义与资源影响 | `../parameters.md` |
| 性能数据（唯一权威） | `../image_link_ab_results.md` |
| 为什么不做零拷贝 | `../image_link_zero_copy_assessment.md` |
| 二阶段测试怎么跑 | `../视觉测试.md` |

归档内容大致分三类：
- `COMMANDS.txt`、`jetson_commands_only.txt`、`jetson_0_to_1_checklist.txt`：早期命令速查/清单，已被 `jetson_deploy.md` 取代。
- `jetson_fps_*`、`jetson_next_round_*`、`jetson_nvvidconv_try_*`、`jetson_namespace_isolation_*`、`jetson_full_fps_ingest_commands.txt`、`jetson_streaming_manual.txt`、`result.txt`、`problem_isolation_check.txt`：帧率/隔离/推流问题排查过程的命令与输出。
- `new.txt`：一次低分辨率算法链改动的说明（实验性，已被正式实现取代）。
- `onboard_ab_test_20261107.md`：11-07 的板上 A/B 记录，结论已并入 `../image_link_ab_results.md` 第九节。

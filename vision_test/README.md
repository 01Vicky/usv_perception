# vision_test —— 二阶段视觉测试工具（独立于功能包）

这两个脚本**不属于任何 ROS 功能包**，放在工作区根目录，和工作区一起同步到板子。
理由：测试工具会随测试口径频繁改，混进 `usv_vision` 会让功能包的版本、依赖和编译产物跟着变。

| 文件 | 运行位置 | 作用 |
|---|---|---|
| `vision_test_record.sh` | 板上 | 录测试消息包 + `param dump` + `node/topic info` + `tegrastats` + `scene.json` |
| `vision_eval.py` | 板上或本机（需 source ROS） | 读 bag，出 4.1.x 判分报告；`--selftest` 可脱离 ROS 自检 |

板上的位置：`~/usv_ws_test/vision_test/`（带 `COLCON_IGNORE`，colcon 不会把它当包扫）。
采集结果也放这个目录下（`S01/`、`S02/`……）。

## 用法

```bash
# 采集（先按交付方式起好采集与识别，本脚本只录不启动）
./vision_test_record.sh -s S03 -c front,left,right -d 600 -n "目标船左前方约300m"

# 判分
source /opt/ros/humble/setup.bash && source <ws>/install/setup.bash
python3 vision_eval.py S03 -o S03/report.md --tegrastats S03/tegrastats.log

# 判定逻辑自检（不需要 ROS、不需要相机）
python3 vision_eval.py --selftest
```

口径、24 卡与测试计划的对应对照、已知冲突，见 `../usv_vision/docs/视觉测试.md`。

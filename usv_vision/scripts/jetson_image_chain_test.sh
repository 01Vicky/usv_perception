#!/usr/bin/env bash
# jetson_image_chain_test.sh —— AGX 板上图像链路验收脚本
#
# 干什么：一键跑完"按需分发"改造的验收项，把结果落成一个目录，便于回传：
#   1) 前置检查（ROS 环境、包、话题、设备）
#   2) 采指标：image_raw / image_algo 的 hz 与 bw、frame_timing_json 首帧、
#      detections 频率、tegrastats 与 CPU 快照
#   3) 反压验证：调用 check_image_qos 跑 A/B/C 三组（脚本自己等 rviz 启动、
#      自己 SIGSTOP/SIGCONT 模拟 GUI 卡死）
#   4) 生成 summary.md（含期望带宽比，便于一眼判断分发是否生效）
#
# 用法：
#   ./jetson_image_chain_test.sh                          # 单路 front，自动起采集+推理
#   ./jetson_image_chain_test.sh --sensor front --duration 30
#   ./jetson_image_chain_test.sh --no-launch              # 已手动起了 pipeline，只采指标
#   ./jetson_image_chain_test.sh --skip-qos               # 跳过 rviz 反压验证
#   ./jetson_image_chain_test.sh --dry-run                # 只打印计划
#
# 退出码：0 通过 / 2 环境或数据问题 / 3 反压验证失败

set -uo pipefail

SENSOR="front"
DURATION=30
OUT=""
DO_LAUNCH=1
INFERENCE="true"
STREAMING="false"
ALGO="true"
SKIP_QOS=0
DRY_RUN=0
ARCHIVE=1
WAIT_RVIZ=60
ABORT_MSG=""

while [ $# -gt 0 ]; do
  case "$1" in
    --sensor)     SENSOR="$2"; shift 2 ;;
    --duration)   DURATION="$2"; shift 2 ;;
    --out)        OUT="$2"; shift 2 ;;
    --no-launch)  DO_LAUNCH=0; shift ;;
    --inference)  INFERENCE="$2"; shift 2 ;;
    --streaming)  STREAMING="$2"; shift 2 ;;
    --algo)       ALGO="$2"; shift 2 ;;
    --skip-qos)   SKIP_QOS=1; shift ;;
    --wait-rviz)  WAIT_RVIZ="$2"; shift 2 ;;
    --no-archive) ARCHIVE=0; shift ;;
    --dry-run)    DRY_RUN=1; shift ;;
    -h|--help)    sed -n '2,30p' "$0"; exit 0 ;;
    *) echo "未知参数：$1（-h 查看用法）" >&2; exit 2 ;;
  esac
done

STAMP="$(date +%Y%m%d_%H%M%S)"
[ -n "$OUT" ] || OUT="./image_chain_test_${STAMP}"
BASE="/sensors/camera/${SENSOR}"

log()  { printf '[%s] %s\n' "$(date +%H:%M:%S)" "$*"; }
warn() { printf '[%s] [警告] %s\n' "$(date +%H:%M:%S)" "$*" >&2; }
die()  {
  ABORT_MSG="$*"
  printf '[%s] [中止] %s\n' "$(date +%H:%M:%S)" "$*" >&2
  exit 2
}


# ---------------------------------------------------------------------------
# 0. 前置检查
# ---------------------------------------------------------------------------
log "前置检查 ..."
if [ "$DRY_RUN" -eq 1 ]; then
  warn "dry-run：跳过 ros2/包/设备检查"
else
  command -v ros2 >/dev/null 2>&1 || die "找不到 ros2，请先 source /opt/ros/humble/setup.bash 与工作区 install/setup.bash"
  ros2 pkg prefix usv_vision >/dev/null 2>&1 || die "找不到 usv_vision 包，请先 colcon build 并 source install/setup.bash"
  ros2 pkg prefix usv_vision_gmsl_cpp >/dev/null 2>&1 || die "找不到 usv_vision_gmsl_cpp 包"
fi

if [ "$DO_LAUNCH" -eq 1 ] && [ ! -e "/dev/video0" ] && [ ! -e "/dev/video3" ]; then
  warn "没看到 /dev/video*，采集节点可能起不来（若走 GMSL SDK 可忽略）"
fi

if pgrep -f "rviz2" >/dev/null 2>&1; then
  warn "检测到 rviz2 正在运行：反压验证的 A 组基线要求 rviz 未启动，请先关闭"
  [ "$SKIP_QOS" -eq 0 ] && warn "否则 check_image_qos 会在 A 组开始时直接中止"
fi

log "sensor=${SENSOR} duration=${DURATION}s out=${OUT}"
log "launch=${DO_LAUNCH} inference=${INFERENCE} streaming=${STREAMING} algo_stream=${ALGO} skip_qos=${SKIP_QOS}"

if [ "$DRY_RUN" -eq 1 ]; then
  cat <<EOF

将执行：
  1) 采集：hz/bw on ${BASE}/image_raw、${BASE}/image_algo
  2) 首帧：ros2 topic echo ${BASE}/frame_timing_json --once
  3) 识别：hz on /perception/camera/${SENSOR}/detections（若 inference=true）
  4) 资源：tegrastats 快照 + ps CPU 前列
  5) 反压：check_image_qos --sensor ${SENSOR} --duration ${DURATION} --auto-rviz
  6) 输出：${OUT}/summary.md
EOF
  exit 0
fi

mkdir -p "$OUT" || die "无法创建输出目录 $OUT"
OUT="$(cd "$OUT" && pwd)"
mkdir -p "${OUT}/ros_log" || die "无法创建 ${OUT}/ros_log"
# 让本次 launch 的节点日志（~/.ros/log 那套）也落进结果目录，便于回传排查
export ROS_LOG_DIR="${OUT}/ros_log"
export ROS_HOME="${ROS_HOME:-$HOME/.ros}"
log "输出目录：${OUT}（节点日志在 ros_log/）"

# ---------------------------------------------------------------------------
# 1. 可选：起 pipeline
# ---------------------------------------------------------------------------
LAUNCH_PID=""
cleanup() {
  local rc=$?
  if [ -n "$LAUNCH_PID" ] && kill -0 "$LAUNCH_PID" 2>/dev/null; then
    log "关闭本次启动的 pipeline (pid=$LAUNCH_PID)"
    kill -INT "$LAUNCH_PID" 2>/dev/null
    sleep 3
    kill -TERM "$LAUNCH_PID" 2>/dev/null
  fi
  if [ -d "$OUT" ]; then
    if [ ! -f "${OUT}/summary.md" ]; then
      {
        echo "# 验收中止（未跑完）"
        echo
        echo "- 时间：$(date '+%Y-%m-%d %H:%M:%S')"
        echo "- 原因：${ABORT_MSG:-脚本被中断（Ctrl-C 或异常退出）}"
        echo "- 退出码：${rc}"
        echo
        echo "## 已保存的日志"
        ls -lh "${OUT}" 2>/dev/null | sed 's/^/    /'
        if [ -d "${OUT}/ros_log" ]; then
          echo
          echo "## 节点日志（ros_log/）"
          ls -lh "${OUT}/ros_log" 2>/dev/null | sed 's/^/    /'
        fi
      } > "${OUT}/abort.md" 2>/dev/null
      log "已写中止报告：${OUT}/abort.md"
    fi
    if [ "$ARCHIVE" -eq 1 ] && [ ! -f "${OUT}.tar.gz" ]; then
      tar czf "${OUT}.tar.gz" -C "$(dirname "$OUT")" "$(basename "$OUT")" 2>/dev/null \
        && log "已归档：${OUT}.tar.gz"
    fi
  fi
}
trap cleanup EXIT INT TERM

if [ "$DO_LAUNCH" -eq 1 ]; then
  log "启动 pipeline（日志 ${OUT}/launch.log）..."
  # 旧版 launch 可能还没有 algo_stream 参数：传了未声明的参数会直接报错，先探测
  LAUNCH_ARGS=("cameras:=['${SENSOR}']" "inference:=${INFERENCE}" "streaming:=${STREAMING}")
  LAUNCH_HELP="$(ros2 launch usv_vision jetson_gmsl_full.launch.py --show-args 2>/dev/null || true)"
  if printf '%s\n' "$LAUNCH_HELP" | grep -q "algo_stream"; then
    LAUNCH_ARGS+=("algo_stream:=${ALGO}")
  else
    warn "当前 launch 文件不支持 algo_stream 参数（多半是旧版代码），已跳过该参数"
    warn "  旧版本来就不分发 image_algo，属预期；基线测试可直接继续"
  fi
  ros2 launch usv_vision jetson_gmsl_full.launch.py "${LAUNCH_ARGS[@]}" \
      > "${OUT}/launch.log" 2>&1 &
  LAUNCH_PID=$!

  log "等待 ${BASE}/image_raw 上线（最多 40s）..."
  online=0
  for _ in $(seq 1 40); do
    topics="$(ros2 topic list 2>/dev/null || true)"
    if printf '%s\n' "$topics" | grep -qx "${BASE}/image_raw"; then online=1; break; fi
    sleep 1
  done
  [ "$online" -eq 1 ] || die "image_raw 未上线，请看 ${OUT}/launch.log 里的报错"
  sleep 3
else
  log "使用已在运行的 pipeline（--no-launch）"
  topics="$(ros2 topic list 2>/dev/null || true)"
  printf '%s\n' "$topics" | grep -qx "${BASE}/image_raw" || die "image_raw 不在线"
fi

# 旧版驱动不会发 image_algo：先探测，避免后面测出一堆 0 还以为是改造没生效
ALGO_TOPIC_ONLINE=0
topics="$(ros2 topic list 2>/dev/null || true)"
if printf '%s\n' "$topics" | grep -qx "${BASE}/image_algo"; then
  ALGO_TOPIC_ONLINE=1
fi
if [ "$ALGO" = "true" ] && [ "$ALGO_TOPIC_ONLINE" -eq 0 ]; then
  warn "未检测到 ${BASE}/image_algo —— 可能是①驱动是旧版（未做按需分发）；"
  warn "  ②launch 传了 algo_stream:=false。本次将跳过 image_algo 相关指标。"
fi

# ---------------------------------------------------------------------------
# 2. 采指标
# ---------------------------------------------------------------------------
hz_of() {  # hz_of <topic> <seconds> <outfile> -> 打印平均值
  local topic="$1" secs="$2" out="$3" val
  timeout "$secs" ros2 topic hz "$topic" > "$out" 2>&1
  val="$(grep -o 'average rate: [0-9.]*' "$out" | tail -1 | awk '{print $3}')"
  printf '%s' "${val:-0}"
}

bw_of() {  # bw_of <topic> <seconds> <outfile> -> 打印平均带宽
  local topic="$1" secs="$2" out="$3" val
  timeout "$secs" ros2 topic bw "$topic" > "$out" 2>&1
  # 实际输出形如 "267.71 MB/s from 28 messages"；取最后一条完整行（timeout 可能截断末行）
  val="$(grep -oE '^[0-9.]+ [KMG]B/s' "$out" | tail -1)"
  printf '%s' "${val:-无数据}"
}

GUI_BUSY="$(pgrep -c -f 'firefox|chrome|chromium' 2>/dev/null || echo 0)"
if [ "${GUI_BUSY:-0}" -gt 0 ]; then
  warn "检测到 ${GUI_BUSY} 个浏览器进程在跑：会占用 CPU 并让 hz 偏低，建议测量期间关闭"
fi

log "采 ${DURATION}s：image_raw hz"
RAW_HZ="$(hz_of "${BASE}/image_raw" "$DURATION" "${OUT}/hz_image_raw.log")"
log "  image_raw ≈ ${RAW_HZ} Hz"

if [ "$ALGO_TOPIC_ONLINE" -eq 1 ]; then
  log "采 ${DURATION}s：image_algo hz"
  ALGO_HZ="$(hz_of "${BASE}/image_algo" "$DURATION" "${OUT}/hz_image_algo.log")"
  log "  image_algo ≈ ${ALGO_HZ} Hz"
else
  ALGO_HZ="—（无此话题）"
  log "跳过 image_algo hz：话题不在线"
fi

log "采 10s：image_raw bw"
RAW_BW="$(bw_of "${BASE}/image_raw" 10 "${OUT}/bw_image_raw.log")"
if [ "$ALGO_TOPIC_ONLINE" -eq 1 ]; then
  ALGO_BW="$(bw_of "${BASE}/image_algo" 10 "${OUT}/bw_image_algo.log")"
else
  ALGO_BW="—（无此话题）"
fi
log "  image_raw ${RAW_BW} ; image_algo ${ALGO_BW}"

log "抓一帧 frame_timing_json"
# --full-length(-f) 必须有：长 JSON 默认会被截断成 "..."
for _try in 1 2 3; do
  timeout 10 ros2 topic echo --full-length "${BASE}/frame_timing_json" --once \
      > "${OUT}/timing_once.log" 2>&1
  grep -q "driver_latency_ms" "${OUT}/timing_once.log" && break
  sleep 1
done

# 延迟合理性检查：硬件时间戳失效时（板上 SDK 可能恒返回 1）driver_latency_ms 会是天文数字
LAT_NUM="$(grep -o '"driver_latency_ms":[-0-9.]*' "${OUT}/timing_once.log" 2>/dev/null | head -1 | cut -d: -f2)"
if [ -n "$LAT_NUM" ]; then
  if awk -v v="$LAT_NUM" 'BEGIN{exit !(v > 10000 || v < -10000)}'; then
    warn "driver_latency_ms=${LAT_NUM} 明显异常：硬件时间戳疑似失效，延迟指标本次不可用"
    warn "  以 ros2 topic hz/bw 与 CPU 占用为准；check_image_qos 会自动改用 publish 时间戳算周期"
  fi
fi

# 驱动新旧判定：新版 timing JSON 里有 "image_algo" 字段（旧版没有）
DRIVER_NEW=0
if grep -q '"image_algo"' "${OUT}/timing_once.log" 2>/dev/null; then
  DRIVER_NEW=1
fi
if [ "$DRIVER_NEW" -eq 1 ]; then
  log "代码版本判定：驱动为新版（timing 里有 image_algo 字段）"
else
  warn "代码版本判定：驱动为旧版（timing 里没有 image_algo 字段）"
  warn "  新版验收指标（image_algo 分发、反压三组）需要先同步代码并重新 colcon build："
  warn "    ./image_chain_rollout.sh list   # 看同步清单"
  if [ "$SKIP_QOS" -eq 0 ]; then
    warn "  反压 B/C 组需要 rviz 订阅 image_algo，旧版下无意义 → 本次自动跳过反压验证"
    SKIP_QOS=1
  fi
fi

DET_HZ="—"
if [ "$INFERENCE" = "true" ]; then
  log "采 ${DURATION}s：detections hz"
  DET_HZ="$(hz_of "/perception/camera/${SENSOR}/detections" "$DURATION" "${OUT}/hz_detections.log")"
  log "  detections ≈ ${DET_HZ} Hz"
  if [ "${DET_HZ}" = "0" ] || [ -z "${DET_HZ}" ]; then
    warn "detections 没有输出：若代码是混合部署（新版 usv_vision + 旧版驱动），"
    warn "  新版 yolo 会订阅 image_algo 但收不到数据。请检查 image_chain_rollout.sh verify"
  fi
fi

log "采 5s tegrastats / CPU 快照"
if command -v tegrastats >/dev/null 2>&1; then
  timeout 5 tegrastats > "${OUT}/tegrastats.log" 2>&1 || true
else
  echo "tegrastats 不可用" > "${OUT}/tegrastats.log"
fi
ps -eo pid,pcpu,pmem,comm --sort=-pcpu | head -15 > "${OUT}/top.log" 2>&1 || true

ros2 topic list  > "${OUT}/topic_list.log" 2>&1 || true
ros2 node list   > "${OUT}/node_list.log" 2>&1 || true
if command -v ros2 >/dev/null 2>&1; then
  ros2 node info "/gmsl_camera_${SENSOR}" > "${OUT}/node_info_driver.log" 2>&1 || true
fi
{ echo "uname: $(uname -a)"; echo "jetpack: $(cat /etc/nv_tegra_release 2>/dev/null | head -1)"; 
  echo "ros: $(printenv ROS_DISTRO)"; echo "python: $(python3 --version 2>&1)"; } > "${OUT}/env.log"

# 从首帧 JSON 里解析实际分辨率，用来自动算"期望带宽"（避免写死误导）
grab_dim() {  # grab_dim <key> <field>
  grep -o "\"$1\":{[^}]*}" "${OUT}/timing_once.log" 2>/dev/null \
    | grep -o "\"$2\":[0-9]*" | head -1 | cut -d: -f2
}
RAW_W="$(grab_dim image width)";     RAW_H="$(grab_dim image height)"
ALGO_W="$(grab_dim image_algo width)"; ALGO_H="$(grab_dim image_algo height)"
RAW_W="${RAW_W:-1920}"; RAW_H="${RAW_H:-1536}"
ALGO_W="${ALGO_W:-640}"; ALGO_H="${ALGO_H:-512}"
EXP_RAW_BW="$(awk -v w="$RAW_W" -v h="$RAW_H" 'BEGIN{printf "%.0f", w*h*3*30/1e6}')"
EXP_ALGO_BW="$(awk -v w="$ALGO_W" -v h="$ALGO_H" 'BEGIN{printf "%.0f", w*h*3*30/1e6}')"
EXPECTED_RATIO="$(awk -v aw="$ALGO_W" -v ah="$ALGO_H" -v rw="$RAW_W" -v rh="$RAW_H" \
  'BEGIN{printf "%.2f", (aw*ah)/(rw*rh)}')"
log "实测分辨率：raw=${RAW_W}x${RAW_H}，algo=${ALGO_W}x${ALGO_H}（期望带宽比 ${EXPECTED_RATIO}）"

# ---------------------------------------------------------------------------
# 3. 反压验证（A/B/C）
# ---------------------------------------------------------------------------
QOS_RC=0
if [ "$SKIP_QOS" -eq 1 ]; then
  log "跳过反压验证（--skip-qos）"
else
  log "开始反压验证：A（仅采集）→ 提示你打开 rviz → B → 冻住 rviz → C"
  log "注意：A 组要求 rviz 未启动；B 组会等你打开 rviz（最多 ${WAIT_RVIZ}s）"
  ros2 run usv_vision check_image_qos \
      --sensor "$SENSOR" --duration "$DURATION" \
      --auto-rviz --wait-rviz "$WAIT_RVIZ" \
      --record "${OUT}/qos_backpressure.md" 2>&1 | tee "${OUT}/qos_backpressure.log"
  QOS_RC="${PIPESTATUS[0]}"
  log "check_image_qos 退出码=${QOS_RC}（0=通过 1=可疑 2=受影响/数据不足）"
fi

# ---------------------------------------------------------------------------
# 4. 汇总
# ---------------------------------------------------------------------------
{
  echo "# 图像链路验收 ${STAMP}"
  echo
  echo "## 环境"
  echo '```'
  cat "${OUT}/env.log"
  echo '```'
  echo
  echo "## 代码版本"
  echo
  if [ "$DRIVER_NEW" -eq 1 ]; then
    echo "- 驱动：**新版**（timing JSON 含 image_algo 字段）"
  else
    echo "- 驱动：**旧版**（timing JSON 无 image_algo 字段）→ 分发相关指标不可信，需先同步代码"
  fi
  if [ "$ALGO_TOPIC_ONLINE" -eq 1 ]; then
    echo "- image_algo 话题：在线"
  else
    echo "- image_algo 话题：**不在线**（旧版驱动，或 algo_stream:=false）"
  fi
  echo
  echo "## 指标"
  echo
  echo "| 项目 | 值 | 期望 |"
  echo "|---|---|---|"
  echo "| image_raw hz | ${RAW_HZ} | ≈ 相机帧率（30） |"
  echo "| image_algo hz | ${ALGO_HZ} | ≈ image_raw hz（分发不应掉帧） |"
  echo "| image_raw bw | ${RAW_BW} | ${RAW_W}x${RAW_H}x3x30fps ≈ ${EXP_RAW_BW} MB/s |"
  echo "| image_algo bw | ${ALGO_BW} | ${ALGO_W}x${ALGO_H}x3x30fps ≈ ${EXP_ALGO_BW} MB/s |"
  echo "| detections hz | ${DET_HZ} | 帧率允许丢帧（best_effort/depth1） |"
  echo
  echo "> 带宽比期望 ≈ ${EXPECTED_RATIO}（${ALGO_W}x${ALGO_H} / ${RAW_W}x${RAW_H}）。"
  echo "> 若 image_algo 与 image_raw 的 hz 差距大，说明分发或订阅端跟不上。"
  echo
  echo "## frame_timing_json 首帧"
  echo '```'
  head -20 "${OUT}/timing_once.log"
  echo '```'
  echo
  echo "## 反压验证（reliable 发布 + rviz 订阅）"
  if [ -f "${OUT}/qos_backpressure.md" ]; then
    echo "详见 \`qos_backpressure.md\`，摘要："
    echo
    sed -n '/## 采样/,$p' "${OUT}/qos_backpressure.md" | tail -20
  else
    echo "未执行（--skip-qos 或提前中止），见 \`qos_backpressure.log\`。"
  fi
  echo
  echo "## 资源"
  echo '```'
  head -5 "${OUT}/tegrastats.log"
  echo '---'
  head -8 "${OUT}/top.log"
  echo '```'
  echo
  echo "## 日志与文件清单"
  echo
  echo '```'
  ls -lh "${OUT}" | sed 's/^/  /'
  if [ -d "${OUT}/ros_log" ]; then
    echo
    echo "  [ros_log/]"
    ls -lh "${OUT}/ros_log" | sed 's/^/  /'
  fi
  echo '```'
  echo
  echo "> 全部日志已保存在本目录，退出时自动打包为 \`$(basename "${OUT}").tar.gz\`（--no-archive 可关闭）。"
} > "${OUT}/summary.md" 2>/dev/null

log "完成。结果目录：${OUT}"
log "  summary.md           ← 汇总（可直接贴）"
log "  qos_backpressure.md  ← 反压验证表格"
log "  *.log                ← 原始日志"
[ "$QOS_RC" -eq 2 ] && { warn "反压验证报告受影响，请按结论里的回退步骤处理"; exit 3; }
exit 0

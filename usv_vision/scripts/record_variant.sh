#!/usr/bin/env bash
# record_variant.sh —— 把当前运行的链路状态「量化并追加」到对照账本
#
# 用法（pipeline 已在跑的情况下）：
#   ./scripts/record_variant.sh -l V5 -n "改了 xxx" [-s 12] [-d docs/variant_benchmark_log.md]
#
# 它会：
#   1. 采采集出帧率（frame_timing_json 计数）、image_raw/image_algo 带宽、detections 频率
#   2. 采采集进程 / 识别进程的 CPU（/proc 增量，非 ps 瞬时值）与 RSS
#   3. 把一行 Markdown 追加到账本的「自动记录」段
#
# 为什么不用 ros2 topic hz 测 image_raw：8.8MB×30fps 下 Python 跟不上，会把 30Hz 报成 16Hz。

set -uo pipefail

LABEL=""
NOTE=""
SECS=12
LEDGER="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)/docs/variant_benchmark_log.md"
SENSOR="front"

while [ $# -gt 0 ]; do
  case "$1" in
    -l|--label)  LABEL="$2"; shift 2 ;;
    -n|--note)   NOTE="$2"; shift 2 ;;
    -s|--secs)   SECS="$2"; shift 2 ;;
    -d|--ledger) LEDGER="$2"; shift 2 ;;
    -c|--sensor) SENSOR="$2"; shift 2 ;;
    -h|--help)   sed -n '2,14p' "$0"; exit 0 ;;
    *) echo "未知参数: $1" >&2; exit 2 ;;
  esac
done

[ -n "$LABEL" ] || { echo "必须用 -l 指定版本标签，如 -l V5" >&2; exit 2; }
[ -n "${ROS_DISTRO:-}" ] || echo "提示：最好先 source /opt/ros/humble/setup.bash 与工作区 install/setup.bash" >&2

BASE="/sensors/camera/${SENSOR}"
CAM_PID="$(pgrep -f gmsl_camera_node_cpp | head -1 || true)"
YOLO_PID="$(pgrep -f 'lib/usv_vision/yolo_infer_node' | head -1 || true)"

hz_of() {   # hz_of <topic> <secs>
  local val
  timeout "$2" ros2 topic hz "$1" >/tmp/_rv_hz.log 2>&1 || true
  val="$(grep -o 'average rate: [0-9.]*' /tmp/_rv_hz.log | tail -1 | awk '{print $3}')"
  printf '%s' "${val:-n/a}"
}
bw_of() {   # bw_of <topic> <secs>
  local val
  timeout "$2" ros2 topic bw "$1" >/tmp/_rv_bw.log 2>&1 || true
  val="$(grep -oE '^[0-9.]+ [KMG]B/s' /tmp/_rv_bw.log | tail -1)"
  printf '%s' "${val:-n/a}"
}
cpu_rss() { # cpu_rss <pid> <secs>  -> "cpu%|rssMB"
  local pid="$1" secs="$2"
  [ -n "$pid" ] && [ -r "/proc/$pid/stat" ] || { printf '%s' "n/a|n/a"; return; }
  local a b t1 t2 el rss
  a=$(awk '{print $14+$15}' "/proc/$pid/stat"); t1=$(date +%s.%N)
  sleep "$secs"
  b=$(awk '{print $14+$15}' "/proc/$pid/stat"); t2=$(date +%s.%N)
  el=$(awk -v x="$t1" -v y="$t2" 'BEGIN{print y-x}')
  rss=$(awk '/VmRSS/{printf "%.0f", $2/1024}' "/proc/$pid/status" 2>/dev/null)
  awk -v d=$((b-a)) -v e="$el" -v h="$(getconf CLK_TCK)" -v m="${rss:-0}" \
      'BEGIN{printf "%.1f%%|%d", d/h/e*100, m}'
}

CAM="$(cpu_rss "$CAM_PID" "$SECS")"
YOLO="$(cpu_rss "$YOLO_PID" "$SECS")"
CAM_CPU="${CAM%%|*}";  CAM_RSS="${CAM##*|}"
YOLO_CPU="${YOLO%%|*}"; YOLO_RSS="${YOLO##*|}"
TOTAL_CPU=$(awk -v a="$CAM_CPU" -v b="$YOLO_CPU" 'BEGIN{
  if (a=="n/a") a=0; if (b=="n/a") b=0; printf "%.1f%%", a+b }')

FRAME_HZ="$(hz_of "${BASE}/frame_timing_json" 10)"
RAW_BW="$(bw_of "${BASE}/image_raw" 8)"
ALGO_BW="$(bw_of "${BASE}/image_algo" 8)"
DET_HZ="$(hz_of "/perception/camera/${SENSOR}/detections" 8)"

ROW="| ${LABEL} | $(date +%F' '%H:%M) | ${FRAME_HZ} Hz | ${RAW_BW} | ${ALGO_BW} | ${CAM_CPU} | ${YOLO_CPU} | ${TOTAL_CPU} | ${DET_HZ} Hz | ${CAM_RSS} / ${YOLO_RSS} | ${NOTE} |"

mkdir -p "$(dirname "$LEDGER")"
if ! grep -q '^## 六、自动记录' "$LEDGER" 2>/dev/null; then
  {
    echo
    echo '## 六、自动记录（`scripts/record_variant.sh` 追加）'
    echo
    echo '| 版本 | 时间 | 出帧率 | image_raw | image_algo | 采集CPU | 识别CPU | 合计CPU | detections | RSS(MB) 采集/识别 | 说明 |'
    echo '|---|---|---|---|---|---|---|---|---|---|---|'
  } >> "$LEDGER"
fi
printf '%s\n' "$ROW" >> "$LEDGER"

echo "$ROW"
echo
echo "已追加到：$LEDGER"
echo "提示：结论/取舍请手工补到「三、改动清单」，那部分需要人判断。"

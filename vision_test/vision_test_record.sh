#!/usr/bin/env bash
# vision_test_record.sh —— 录一次可用于 4.1.x 判分的包 + 现场快照
#
# 产出目录结构（vision_eval 直接读这个目录）：
#   <out>/
#     ├── bag/            测试消息包（不压缩，三路 10min 约 30MB）
#     ├── snapshot/       ros2 param dump（各节点生效参数）
#     ├── graph/          node list / topic list / topic info
#     ├── scene.json      场景与版本信息（vision_eval 读）
#     ├── tegrastats.log  资源日志（4.1.21）
#     ├── bag_info.txt
#     └── manifest.sha256 原始数据指纹（判分报告可追溯）
#
# 用法：
#   ./vision_test_record.sh -s S03 -c front,left,right -d 600 -n "目标船左前方约300m"
#   ./vision_test_record.sh -s S04 -c front -d 300 --with-algo     # 顺便录算法图，供异常注入素材
#   ./vision_test_record.sh -s S06 -c front -d 120 --with-annotated # 4.1.10 用（需 publish_debug_image:=true）
#   ./vision_test_record.sh -s S05 -c front,left,right -d 600 --no-tegra
#
# 注意：
#   1. 先按交付方式启动采集/识别，本脚本只录不启动（避免重复启动驱动）
#   2. 默认不录图像：1920x1536 单帧 8.85MB，三路 30Hz 约 790MB/s，录不下也没必要
#   3. 录包会在 /perception/*/detections 上产生订阅者，不影响数据本身

set -uo pipefail

SCENE=""
CAMS="front,left,right"
DURATION=600
OUT=""
NOTES=""
WITH_ALGO=0
WITH_ANNOTATED=0
TEGRA=1
TARGET_HZ=20
REPO=""

while [ $# -gt 0 ]; do
  case "$1" in
    -s|--scene)     SCENE="$2"; shift 2 ;;
    -c|--cameras)   CAMS="$2"; shift 2 ;;
    -d|--duration)  DURATION="$2"; shift 2 ;;
    -o|--out)       OUT="$2"; shift 2 ;;
    -n|--notes)     NOTES="$2"; shift 2 ;;
    --with-algo)    WITH_ALGO=1; shift ;;
    --with-annotated) WITH_ANNOTATED=1; shift ;;
    --no-tegra)     TEGRA=0; shift ;;
    --target-hz)    TARGET_HZ="$2"; shift 2 ;;
    --repo)         REPO="$2"; shift 2 ;;
    -h|--help)      sed -n '2,25p' "$0"; exit 0 ;;
    *) echo "未知参数：$1（-h 查看用法）" >&2; exit 2 ;;
  esac
done

[ -n "$SCENE" ] || { echo "必须给 -s <场景号>" >&2; exit 2; }
STAMP="$(date +%Y%m%d_%H%M%S)"
[ -n "$OUT" ] || OUT="./snap_${SCENE}_${STAMP}"

log() { printf '[%s] %s\n' "$(date +%H:%M:%S)" "$*"; }
warn() { printf '[%s] [警告] %s\n' "$(date +%H:%M:%S)" "$*" >&2; }

if ! command -v ros2 >/dev/null 2>&1; then
  echo "找不到 ros2，先 source /opt/ros/humble/setup.bash 和工作区 install/setup.bash" >&2
  exit 2
fi

# 注意：不要预先建 bag/，ros2 bag record 会拒绝已存在的输出目录
mkdir -p "$OUT"/{snapshot,graph} || exit 2
log "输出目录：$OUT"

# ── 1. 话题清单 ──────────────────────────────────────────────
IFS=',' read -r -a CAM_ARR <<< "$CAMS"
TOPICS=()
for cam in "${CAM_ARR[@]}"; do
  cam="$(echo "$cam" | xargs)"
  [ -n "$cam" ] || continue
  TOPICS+=(
    "/sensors/camera/${cam}/frame_timing_json"
    "/sensors/camera/${cam}/camera_info"
    "/sensors/camera/${cam}/camera_info_algo"
    "/perception/camera/${cam}/detections"
    "/perception/camera/${cam}/objects"
    "/perception/camera/${cam}/status"
  )
  if [ "$WITH_ALGO" = "1" ]; then
    TOPICS+=("/sensors/camera/${cam}/image_algo")
  fi
  if [ "$WITH_ANNOTATED" = "1" ]; then
    TOPICS+=("/perception/camera/${cam}/annotated")
  fi
done

log "录制话题：${#TOPICS[@]} 个"
# ros2 topic list 每次调用都要连一次 daemon，逐个查会慢且偶发拿到不完整的列表
timeout 10 ros2 topic list > "$OUT/graph/topic_list_now.txt" 2>/dev/null
for t in "${TOPICS[@]}"; do
  if ! grep -qx "$t" "$OUT/graph/topic_list_now.txt"; then
    warn "话题不存在：$t（该路可能未启动）"
  fi
done

# ── 2. 现场快照（4.1.1 / 4.1.7 用） ──────────────────────────
log "采集节点与话题快照"
timeout 10 ros2 node list  > "$OUT/graph/node_list.txt" 2>&1
timeout 10 ros2 topic list -t > "$OUT/graph/topic_list.txt" 2>&1
for t in "${TOPICS[@]}"; do
  {
    echo "### $t"
    timeout 8 ros2 topic info "$t" -v 2>&1
    echo
  } >> "$OUT/graph/topic_info.txt"
done
# 节点可能不响应参数服务，必须加超时，否则整个采集卡死在这里
for node in $(timeout 10 ros2 node list 2>/dev/null); do
  name="$(echo "$node" | tr '/' '_')"
  timeout 10 ros2 param dump "$node" > "$OUT/snapshot/param${name}.yaml" 2>/dev/null \
    || rm -f "$OUT/snapshot/param${name}.yaml"
done

# ── 3. 版本信息 ──────────────────────────────────────────────
# 仓库根：显式 --repo > 当前目录向上找 > $HOME/usv_ws_test（板上布局）
if [ -z "$REPO" ]; then
  for cand in "$(pwd)" "$HOME/usv_ws_test" "$HOME"; do
    root="$(git -C "$cand" rev-parse --show-toplevel 2>/dev/null || true)"
    if [ -n "$root" ]; then REPO="$root"; break; fi
  done
fi
COMMIT=""
DIRTY="false"
if [ -n "$REPO" ] && command -v git >/dev/null 2>&1; then
  COMMIT="$(git -C "$REPO" rev-parse --short HEAD 2>/dev/null || true)"
  if [ -n "$(git -C "$REPO" status --porcelain 2>/dev/null | head -1)" ]; then DIRTY="true"; fi
fi
ENGINE_PATH="$(grep -h "engine_path" "$OUT"/snapshot/*.yaml 2>/dev/null | head -1 | sed 's/.*engine_path: *//' | tr -d '"')"
ENGINE_SHA=""
# yaml 里是相对路径（相对节点启动目录），挨个可能的位置找一下
if [ -n "$ENGINE_PATH" ] && [ ! -f "$ENGINE_PATH" ]; then
  for base in "$HOME/usv_ws_test" "$HOME" "$(pwd)"; do
    if [ -f "$base/$ENGINE_PATH" ]; then
      ENGINE_PATH="$base/$ENGINE_PATH"
      break
    fi
  done
fi
if [ -n "$ENGINE_PATH" ] && [ -f "$ENGINE_PATH" ]; then
  ENGINE_SHA="$(sha256sum "$ENGINE_PATH" | cut -d' ' -f1)"
elif [ -n "$ENGINE_PATH" ]; then
  warn "engine 文件不在本机路径：$ENGINE_PATH（请在板上确认 hash）"
fi

cat > "$OUT/scene.json" <<JSON
{
  "scene_id": "${SCENE}",
  "mode": "single_module_${#CAM_ARR[@]}cam",
  "cameras": [$(printf '"%s",' "${CAM_ARR[@]}" | sed 's/,$//')],
  "repo": "${REPO}",
  "commit": "${COMMIT}",
  "dirty": ${DIRTY},
  "engine": "${ENGINE_PATH}",
  "engine_sha256": "${ENGINE_SHA}",
  "recorded_at": "$(date '+%Y-%m-%d %H:%M:%S')",
  "duration_s": ${DURATION},
  "target_hz": ${TARGET_HZ},
  "operator": "${USER:-unknown}",
  "host": "$(hostname)",
  "bag_path": "${OUT}/bag",
  "notes": "${NOTES}"
}
JSON
log "scene.json 已写入"

# ── 4. tegrastats（4.1.21） ─────────────────────────────────
TEGRA_PID=""
if [ "$TEGRA" = "1" ] && command -v tegrastats >/dev/null 2>&1; then
  # stdbuf 保证被 kill 时不丢缓冲里的最后几行
  stdbuf -oL tegrastats --interval 2000 > "$OUT/tegrastats.log" 2>&1 &
  TEGRA_PID=$!
  log "tegrastats 已启动（pid $TEGRA_PID）"
elif [ "$TEGRA" = "1" ]; then
  warn "板上没有 tegrastats，4.1.21 将记未执行"
fi

cleanup() {
  if [ -n "$TEGRA_PID" ]; then
    kill "$TEGRA_PID" 2>/dev/null || true
    wait "$TEGRA_PID" 2>/dev/null || true
    log "tegrastats 已停止"
  fi
}
trap cleanup EXIT INT TERM

# ── 5. 录包 ─────────────────────────────────────────────────
log "开始录制 ${DURATION}s（Ctrl+C 可提前结束）"
# 用 timeout 发 SIGINT 停止：rosbag2 收到 SIGINT 才会把 metadata 写完整
# 不压缩：这些消息很小（三路 10 min 约 30 MB），压缩后 rosbag2_py 读不了还得先解压
timeout -s INT "${DURATION}s" ros2 bag record -o "$OUT/bag" \
  "${TOPICS[@]}"
BAG_RC=$?
if [ "$BAG_RC" = "124" ]; then BAG_RC=0; fi
log "录制结束（退出码 $BAG_RC）"

cleanup
trap - EXIT INT TERM

if [ -d "$OUT/bag" ]; then
  ros2 bag info "$OUT/bag" > "$OUT/bag_info.txt" 2>&1
  log "包信息：$OUT/bag_info.txt"
else
  warn "没有生成 bag 目录，检查录制是否被中断"
fi

# ── 原始数据指纹：报告里引用的每个文件都能事后校验 ──────────────
if command -v sha256sum >/dev/null 2>&1; then
  ( cd "$OUT" && find . -type f ! -name "manifest.sha256" -print0 \
      | sort -z | xargs -0 sha256sum > manifest.sha256 2>/dev/null ) \
    && log "原始数据指纹：$OUT/manifest.sha256（$(wc -l < "$OUT/manifest.sha256") 个文件）" \
    || warn "manifest.sha256 生成失败（不影响判分）"
fi

cat <<EOF

────────────────────────────────────────
采集完成：$OUT
下一步（本机也能判分，需先 source /opt/ros/humble/setup.bash）：
  python3 vision_test/vision_eval.py "$OUT" -o "$OUT/report.md"   # 在仓库/工作区根执行；已在本目录则用 ./vision_eval.py
校验原始数据未被改动：cd "$OUT" && sha256sum -c manifest.sha256
需要 4.1.21 资源数据时补 --tegrastats "$OUT/tegrastats.log"
────────────────────────────────────────
EOF

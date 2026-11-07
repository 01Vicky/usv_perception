#!/usr/bin/env bash
# image_chain_rollout.sh —— 图像链路改动的「清单 / 打包 / 版本校验」工具
#
# 用途：测试主机上往往还是旧版代码。本脚本解决三件事：
#   1) list   ：列出本次改动涉及的全部文件（同步清单）
#   2) pack   ：把这些文件打成 tar.gz，便于拷到测试主机
#   3) verify ：在任意一侧校验代码是不是新版（含"混合部署"检测）
#
# 用法：
#   ./image_chain_rollout.sh                      # = list
#   ./image_chain_rollout.sh pack /tmp/out         # 打包到 /tmp/out
#   ./image_chain_rollout.sh verify                # 校验本仓库
#   ./image_chain_rollout.sh verify --root /path   # 校验指定仓库根
#   ./image_chain_rollout.sh backup ~/USV_ROS/src/usv_perception   # 覆盖前先备份旧代码
#   ./image_chain_rollout.sh compare <旧结果目录> <新结果目录>       # 改造前后指标对比
#
# verify 退出码：0=全部新版  1=部分缺失（混合部署）  2=基本是旧版
#
# 注意：仓库布局假定为 <root>/usv_vision/... 与 <root>/usv_vision_gmsl_cpp/...

set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

# 本次改动涉及的文件（相对仓库根）
FILES=(
  "usv_vision_gmsl_cpp/include/usv_vision_gmsl_cpp/camera_image_publishers.hpp"
  "usv_vision_gmsl_cpp/src/gmsl_camera_node_cpp.cpp"
  "usv_vision_gmsl_cpp/src/gmsl_multi_camera_sync_node.cpp"
  "usv_vision_gmsl_cpp/config/gmsl_camera_cpp_front.yaml"
  "usv_vision_gmsl_cpp/config/gmsl_camera_template.yaml"
  "usv_vision_gmsl_cpp/config/gmsl_multi_sync.yaml"
  "usv_vision_gmsl_cpp/README.md"
  "usv_vision/usv_vision/common/topics.py"
  "usv_vision/usv_vision/yolo_infer_node.py"
  "usv_vision/config/yolo_infer.yaml"
  "usv_vision/launch/jetson_gmsl_full.launch.py"
  "usv_vision/launch/usv_vision.launch.py"
  "usv_vision/setup.py"
  "usv_vision/scripts/check_image_qos_backpressure.py"
  "usv_vision/scripts/jetson_image_chain_test.sh"
  "usv_vision/scripts/image_chain_rollout.sh"
  "usv_vision/docs/image_link_zero_copy_assessment.md"
  "usv_vision/CHANGELOG.md"
)

# 关键标记：文件 -> 只在新版里出现的字符串（用于 verify）
MARKERS=(
  "usv_vision_gmsl_cpp/include/usv_vision_gmsl_cpp/camera_image_publishers.hpp|CameraImagePublishers"
  "usv_vision_gmsl_cpp/src/gmsl_camera_node_cpp.cpp|image_algo"
  "usv_vision_gmsl_cpp/src/gmsl_multi_camera_sync_node.cpp|derive_sibling_topic"
  "usv_vision_gmsl_cpp/config/gmsl_camera_cpp_front.yaml|enable_algo_stream"
  "usv_vision_gmsl_cpp/config/gmsl_camera_template.yaml|enable_algo_stream"
  "usv_vision_gmsl_cpp/config/gmsl_multi_sync.yaml|enable_algo_stream"
  "usv_vision/usv_vision/common/topics.py|image_algo"
  "usv_vision/usv_vision/yolo_infer_node.py|use_algo_image"
  "usv_vision/config/yolo_infer.yaml|reference_width"
  "usv_vision/launch/jetson_gmsl_full.launch.py|algo_stream"
  "usv_vision/launch/usv_vision.launch.py|use_algo_image"
  "usv_vision/setup.py|check_image_qos"
  "usv_vision/scripts/check_image_qos_backpressure.py|check_image_qos"
  "usv_vision/scripts/jetson_image_chain_test.sh|jetson_image_chain_test"
)

usage() { sed -n '2,20p' "${BASH_SOURCE[0]}"; }

cmd_list() {
  echo "仓库根：${ROOT}"
  echo "本次改动文件（${#FILES[@]} 个）："
  local f
  for f in "${FILES[@]}"; do
    if [ -e "${ROOT}/${f}" ]; then echo "  [有] ${f}"; else echo "  [缺] ${f}"; fi
  done
  cat <<'EOF'

同步到测试主机的建议做法（在开发机执行）：
  ./image_chain_rollout.sh pack /tmp/out
  scp /tmp/out/image_chain_changes_*.tar.gz <user>@<host>:~/

  # 测试主机上（TARGET = 含 usv_vision/ 与 usv_vision_gmsl_cpp/ 的那个目录，
  #            通常是 ~/USV_ROS/src/usv_perception，不要解到工作区根）：
  TARGET=~/USV_ROS/src/usv_perception
  ls "$TARGET/usv_vision" "$TARGET/usv_vision_gmsl_cpp" >/dev/null || echo "TARGET 不对，先确认布局"
  tar xzf ~/image_chain_changes_*.tar.gz -C "$TARGET"
  cd ~/USV_ROS && source /opt/ros/humble/setup.bash
  colcon build --packages-select usv_vision_gmsl_cpp usv_vision && source install/setup.bash
  "$TARGET/usv_vision/scripts/image_chain_rollout.sh" verify   # 应全绿

只同步脚本、不同步代码时（旧版驱动下也能跑指标，但没有 image_algo）：
  scp usv_vision/scripts/check_image_qos_backpressure.py \
      usv_vision/scripts/jetson_image_chain_test.sh \
      usv_vision/scripts/image_chain_rollout.sh <user>@<host>:~/USV_ROS/scripts/
EOF
}

cmd_pack() {
  local out="${1:-/tmp/out}"
  mkdir -p "$out" || { echo "无法创建 $out" >&2; exit 2; }
  local stamp tarball
  stamp="$(date +%Y%m%d_%H%M%S)"
  tarball="${out}/image_chain_changes_${stamp}.tar.gz"
  local existing=()
  local f
  for f in "${FILES[@]}"; do
    [ -e "${ROOT}/${f}" ] && existing+=("${f}")
  done
  if [ "${#existing[@]}" -eq 0 ]; then
    echo "没找到任何改动文件，检查 ROOT=${ROOT}" >&2
    exit 2
  fi
  tar czf "$tarball" -C "$ROOT" "${existing[@]}" || { echo "打包失败" >&2; exit 2; }
  echo "已打包 ${#existing[@]} 个文件 -> $tarball"
  echo "大小：$(du -h "$tarball" | cut -f1)"
  echo "内容："
  tar tzf "$tarball" | sed 's/^/  /'
}

cmd_backup() {
  local target="${1:-}"
  [ -n "$target" ] || { echo "用法：image_chain_rollout.sh backup <TARGET>" >&2; exit 2; }
  [ -d "$target" ] || { echo "目录不存在：$target" >&2; exit 2; }
  local stamp tarball found=()
  stamp="$(date +%Y%m%d_%H%M%S)"
  tarball="image_chain_old_backup_${stamp}.tar.gz"
  local f
  for f in "${FILES[@]}"; do
    [ -e "${target}/${f}" ] && found+=("${f}")
  done
  if [ "${#found[@]}" -eq 0 ]; then
    echo "在 $target 下没找到这些文件，确认 TARGET 是否是含两个包的那一层" >&2
    exit 2
  fi
  tar czf "${tarball}" -C "$target" "${found[@]}" || { echo "备份失败" >&2; exit 2; }
  echo "已备份旧代码 ${#found[@]} 个文件 -> $(cd "$(dirname "${tarball}")" && pwd)/${tarball}"
  echo
  echo "回滚方式（需要时）："
  echo "  tar xzf ${tarball} -C ${target}"
  echo "  cd <工作区根> && colcon build --packages-select usv_vision_gmsl_cpp usv_vision"
}

# 从 summary.md 的指标表里取一个字段值（$1=文件 $2=行名 $3=列号，2=本值 3=期望）
_sum_cell() {
  awk -F'|' -v k="$2" -v c="$3" '
    $2 ~ k {
      v = $(c+1)
      gsub(/^[ \t]+|[ \t]+$/, "", v)
      print v; exit
    }' "$1" 2>/dev/null
}

# 归一化指标值：去掉中文/前缀/空格，保证表格按 ASCII 对齐
_norm_cell() {
  local v="$1"
  v="${v//（无此话题）/}"
  v="${v//average: /}"
  v="${v// /}"
  v="${v/—/-}"
  printf '%s' "${v:-n/a}"
}

# 取字符串开头的数字（用于算变化量），没有则返回空
_num_of() {
  printf '%s' "$1" | grep -oE '^[0-9]+(\.[0-9]+)?' || true
}

cmd_compare() {
  local old_dir="${1:-}" new_dir="${2:-}"
  if [ -z "$old_dir" ] || [ -z "$new_dir" ]; then
    echo "用法：image_chain_rollout.sh compare <改造前结果目录> <改造后结果目录>" >&2
    exit 2
  fi
  local old_sum="${old_dir%/}/summary.md" new_sum="${new_dir%/}/summary.md"
  [ -f "$old_sum" ] || { echo "找不到 $old_sum" >&2; exit 2; }
  [ -f "$new_sum" ] || { echo "找不到 $new_sum" >&2; exit 2; }

  echo "改造前：$old_dir"
  echo "改造后：$new_dir"
  echo
  local keys=("image_raw hz" "image_algo hz" "image_raw bw" "image_algo bw" "detections hz")
  printf '%-16s %-16s %-16s %s\n' "metric" "before" "after" "delta"
  printf '%s\n' "$(printf '%.0s-' $(seq 1 62))"
  local k before after delta nb na
  for k in "${keys[@]}"; do
    before="$(_norm_cell "$(_sum_cell "$old_sum" "$k" 2)")"
    after="$(_norm_cell "$(_sum_cell "$new_sum" "$k" 2)")"
    delta="-"
    nb="$(_num_of "$before")"; na="$(_num_of "$after")"
    if [ -n "$nb" ] && [ -n "$na" ]; then
      delta="$(awk -v a="$nb" -v b="$na" 'BEGIN{printf "%+.1f", b-a}')"
    elif [ -z "$nb" ] && [ -n "$na" ]; then
      delta="new"
    fi
    printf '%-16s %-16s %-16s %s\n' "$k" "$before" "$after" "$delta"
  done
  echo
  echo "判读要点："
  echo "  1) image_algo hz 应 ≈ image_raw hz（分发不该掉帧）；差得多说明订阅端跟不上或分发异常。"
  echo "  2) image_algo bw / image_raw bw 应 ≈ 算法图与全分辨率的像素比（640x640≈0.14，640x512≈0.11）。"
  echo "  3) 改造前若是旧版驱动，image_algo 两列会是 - 或 n/a，属正常（旧版没有该话题）。"
  echo "  4) detections hz 允许低于图像帧率（订阅端 best_effort/depth1 主动丢旧帧）。"
}

cmd_verify() {
  local root="$ROOT"
  while [ $# -gt 0 ]; do
    case "$1" in
      --root) root="$2"; shift 2 ;;
      *) echo "未知参数：$1" >&2; exit 2 ;;
    esac
  done
  echo "校验仓库：${root}"
  echo
  local width=40 entry marker file tmp
  for entry in "${MARKERS[@]}"; do
    tmp="${entry%%|*}"
    [ "${#tmp}" -gt "$width" ] && width="${#tmp}"
  done
  width=$((width + 2))
  printf "%-${width}s %-6s %s\n" "文件" "状态" "依据"
  printf '%s\n' "$(printf '%.0s-' $(seq 1 $((width + 40))))"
  local ok=0 missing=0 old=0
  for entry in "${MARKERS[@]}"; do
    file="${entry%%|*}"
    marker="${entry##*|}"
    if [ ! -e "${root}/${file}" ]; then
      printf "%-${width}s %-6s %s\n" "$file" "缺失" "文件不存在"
      missing=$((missing + 1))
      continue
    fi
    if grep -q -- "$marker" "${root}/${file}" 2>/dev/null; then
      printf "%-${width}s %-6s %s\n" "$file" "新版" "含 ${marker}"
      ok=$((ok + 1))
    else
      printf "%-${width}s %-6s %s\n" "$file" "旧版" "不含 ${marker}"
      old=$((old + 1))
    fi
  done
  echo
  echo "统计：新版 ${ok} / 旧版 ${old} / 缺失 ${missing}（共 ${#MARKERS[@]} 项）"
  echo
  if [ "$old" -eq 0 ] && [ "$missing" -eq 0 ]; then
    echo "结论：全部为新版，可以直接跑验收脚本。"
    exit 0
  fi
  if [ "$ok" -eq 0 ]; then
    echo "结论：基本是旧版代码。请按 list 的同步步骤把改动拷过来并重新 colcon build。"
    exit 2
  fi
  echo "结论：混合部署（部分新部分旧）。最危险的组合是「新版 yolo + 旧版驱动」——"
  echo "      识别会订阅 image_algo 但收不到任何数据，detections 频率≈0。"
  local detail=""
  [ "$old" -gt 0 ] && detail="${old} 个旧版项"
  [ "$missing" -gt 0 ] && detail="${detail}${detail:+ + }${missing} 个缺失项"
  echo "      需要补齐：${detail}，然后重新 colcon build。"
  exit 1
}

ACTION="${1:-list}"
[ $# -gt 0 ] && shift
case "$ACTION" in
  list|ls)   cmd_list ;;
  pack)      cmd_pack "${1:-/tmp/out}" ;;
  verify)    cmd_verify "$@" ;;
  backup)    cmd_backup "$@" ;;
  compare)   cmd_compare "$@" ;;
  -h|--help|help) usage ;;
  *) echo "未知子命令：$ACTION（可用：list / pack / verify / backup / compare）" >&2; exit 2 ;;
esac

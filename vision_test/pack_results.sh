#!/usr/bin/env bash
# pack_results.sh —— 把板上测试产物打成一个包，方便优盘/无网络场景带走
#
# 用法：
#   ./pack_results.sh -o /media/usb                      # 打全部 S* 结果 + 脚本 + 文档 + 源码
#   ./pack_results.sh -o /media/usb -s S13,S14           # 只打指定结果
#   ./pack_results.sh -o ~/out --no-repo                 # 不带源码
#
# 产出：<dest>/vision_board_<时间戳>.tar.gz（内含 README.txt 与 sha256.txt）
#
# 为什么需要它：现场可能没有网络，优盘是唯一的搬运方式。这个脚本保证
# 带回去的东西是"自解释"的——光看包里的 README 就知道是哪台板、哪个版本、
# 每个结果目录对应什么场景。

set -uo pipefail

DEST=""
SCENES=""
WITH_REPO=1
WS="${WS:-$HOME/usv_ws_test}"
REPO="$WS/src/usv_perception"

while [ $# -gt 0 ]; do
  case "$1" in
    -o|--out)   DEST="$2"; shift 2 ;;
    -s|--scenes) SCENES="$2"; shift 2 ;;
    --no-repo)  WITH_REPO=0; shift ;;
    -h|--help)  sed -n '2,16p' "$0"; exit 0 ;;
    *) echo "未知参数：$1" >&2; exit 2 ;;
  esac
done

[ -n "$DEST" ] || { echo "必须给 -o <目标目录，比如 /media/usb>" >&2; exit 2; }
[ -d "$DEST" ] || { echo "目标目录不存在：$DEST" >&2; exit 2; }
[ -d "$WS/vision_test" ] || { echo "找不到工作区测试目录：$WS/vision_test" >&2; exit 2; }

STAMP="$(date +%Y%m%d_%H%M%S)"
NAME="vision_board_${STAMP}"
TARBALL="$DEST/${NAME}.tar.gz"
STAGE="$(mktemp -d)"
trap 'rm -rf "$STAGE"' EXIT

mkdir -p "$STAGE/$NAME"
C="$STAGE/$NAME"

# ① 脚本（判分/采集工具本身要带回去，将来复跑）
mkdir -p "$C/vision_test"
cp -f "$WS"/vision_test/*.py "$WS"/vision_test/*.sh "$WS"/vision_test/README.md "$C/vision_test/" 2>/dev/null
cp -f "$WS"/vision_test/COLCON_IGNORE "$C/vision_test/" 2>/dev/null

# ② 结果目录
mkdir -p "$C/results"
if [ -n "$SCENES" ]; then
  IFS=',' read -ra LIST <<< "$SCENES"
  for s in "${LIST[@]}"; do
    [ -d "$WS/vision_test/$s" ] && cp -a "$WS/vision_test/$s" "$C/results/" \
      || echo "[警告] 没有结果目录 $s" >&2
  done
else
  for d in "$WS"/vision_test/S*; do
    [ -d "$d" ] && cp -a "$d" "$C/results/"
  done
fi

# ③ 文档（命令手册/测试口径/性能报告）
if [ -d "$REPO/usv_vision/docs" ]; then
  mkdir -p "$C/docs"
  cp -f "$REPO"/usv_vision/docs/*.md "$C/docs/" 2>/dev/null
fi

# ④ 源码快照（可选）
if [ "$WITH_REPO" = "1" ] && [ -d "$REPO" ]; then
  mkdir -p "$C/repo"
  tar czf "$C/repo/usv_perception_src.tar.gz" -C "$REPO" \
    --exclude='__pycache__' --exclude='*.pyc' --exclude='build' --exclude='install' --exclude='log' \
    usv_vision usv_vision_gmsl_cpp vision_test 2>/dev/null \
    && echo "源码快照：repo/usv_perception_src.tar.gz" >> "$C/README.txt"
fi

# ⑤ README：让包自解释
COMMIT="$(git -C "$REPO" rev-parse --short HEAD 2>/dev/null || echo "未知")"
{
  echo "视觉链路测试产物包"
  echo "================================"
  echo "打包时间 : $(date '+%F %T')"
  echo "来源主机 : $(hostname)  ($(uname -m))"
  echo "工作区   : $WS"
  echo "代码版本 : ${COMMIT}"
  echo ""
  echo "内容说明"
  echo "  vision_test/  采集与判分脚本（vision_test_record.sh / vision_eval.py）"
  echo "  results/      测试结果目录（每个 Sxx 含 bag/、snapshot/、graph/、scene.json、tegrastats.log、manifest.sha256、report.md）"
  echo "  docs/         命令手册、测试口径、性能报告"
  echo "  repo/         源码快照（本次$([ "$WITH_REPO" = "1" ] && echo 已包含 || echo 未包含)）"
  echo ""
  echo "拿到本机后怎么用"
  echo "  1) 解开：tar xzf ${NAME}.tar.gz"
  echo "  2) 看报告：results/S*/report.md"
  echo "  3) 校原始数据：cd results/Sxx && sha256sum -c manifest.sha256"
  echo "  4) 想复算：source /opt/ros/humble/setup.bash 后"
  echo "     python3 vision_test/vision_eval.py results/Sxx -o /tmp/r.md --tegrastats results/Sxx/tegrastats.log"
  echo ""
  echo "结果目录清单"
  ( cd "$C/results" 2>/dev/null && for d in */; do
      note="$(python3 -c "import json,sys;print(json.load(open('${d}scene.json')).get('notes',''))" 2>/dev/null)"
      echo "  ${d%/}  ${note}"
    done )
} > "$C/README.txt"

# ⑥ 指纹
( cd "$STAGE" && find "$NAME" -type f ! -name sha256.txt -print0 \
    | sort -z | xargs -0 sha256sum > "$NAME/sha256.txt" 2>/dev/null )

tar czf "$TARBALL" -C "$STAGE" "$NAME" || { echo "打包失败" >&2; exit 1; }
SIZE="$(du -h "$TARBALL" | awk '{print $1}')"

echo "已生成：$TARBALL（$SIZE）"
echo "内容："
echo "  results : $(ls "$C/results" 2>/dev/null | wc -l) 个结果目录"
echo "  docs    : $(ls "$C/docs" 2>/dev/null | wc -l) 份文档"
echo "  scripts : $(ls "$C/vision_test" 2>/dev/null | wc -l) 个文件"
echo "带回去后：tar xzf $(basename "$TARBALL")，先看 README.txt"

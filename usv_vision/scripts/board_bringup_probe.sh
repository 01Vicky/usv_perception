#!/usr/bin/env bash
# board_bringup_probe.sh —— 换板迁移体检（只读，不改系统）
#
# 用途：拿到一块**没跑过本视觉链路**的 Jetson，先跑这个，一眼看出缺什么、
#       哪些配置必须现场改（device_path / engine / 标定）。
#
# 用法：
#   bash board_bringup_probe.sh                 # 打印 + 写 probe_<host>_<时间>.log
#   bash board_bringup_probe.sh --quiet         # 只记日志
#
# 判读：出现 FAIL 的项按提示补；WARN 项属于"能用但要注意"。
set -uo pipefail

QUIET=0
[[ "${1:-}" == "--quiet" ]] && QUIET=1
HOST="$(hostname 2>/dev/null || echo unknown)"
STAMP="$(date +%Y%m%d_%H%M%S)"
LOG="probe_${HOST}_${STAMP}.log"
FAIL=0
WARN=0

emit() { printf '%s\n' "$*" >>"${LOG}"; [[ "${QUIET}" -eq 1 ]] || printf '%s\n' "$*"; }
ok()   { emit "  [OK]   $*"; }
fail() { emit "  [FAIL] $*"; FAIL=$((FAIL+1)); }
warn() { emit "  [WARN] $*"; WARN=$((WARN+1)); }
sec()  { emit ""; emit "── $* ──────────────────────────────────────"; }

have() { command -v "$1" >/dev/null 2>&1; }

emit "board_bringup_probe  host=${HOST}  time=${STAMP}"
emit "只读体检，不改任何系统配置；结果同时写入 ${LOG}"

# ── 1. 板卡与系统 ───────────────────────────────────────────────
sec "1. 板卡 / JetPack"
if [[ -r /etc/nv_tegra_release ]]; then
  emit "  $(head -1 /etc/nv_tegra_release)"
else
  warn "没找到 /etc/nv_tegra_release（不是 Jetson 或镜像非 L4T？）"
fi
if [[ -r /etc/nv_boot_control.conf ]]; then
  emit "  $(grep -hE 'TNSPEC|COMPATIBLE_SPEC' /etc/nv_boot_control.conf | head -2 | tr '\n' ' ')"
fi
emit "  $(. /etc/os-release 2>/dev/null && echo "${PRETTY_NAME:-unknown}") / kernel $(uname -r) / $(uname -m)"
emit "  CPU 核数 $(nproc)  内存 $(free -g 2>/dev/null | awk '/^Mem:/{print $2" GB"}')"
if have tegrastats; then ok "tegrastats 可用（可做功耗/占用测量）"; else warn "tegrastats 缺失：无法做功耗测量"; fi
if have jetson_clocks; then
  if jetson_clocks --show >/dev/null 2>&1; then
    ok "jetson_clocks 可查询（锁频前先 --show 存档）"
  else
    warn "jetson_clocks 存在但需 sudo 才能查询"
  fi
else
  warn "jetson_clocks 缺失"
fi

# ── 2. CUDA / TensorRT ─────────────────────────────────────────
sec "2. CUDA / TensorRT / cuDNN"
if have nvcc; then
  ok "nvcc: $(nvcc --version | awk '/release/{print $5, $6}')"
elif [[ -x /usr/local/cuda/bin/nvcc ]]; then
  ok "nvcc(未入 PATH): $(/usr/local/cuda/bin/nvcc --version | awk '/release/{print $5, $6}')"
else
  fail "找不到 nvcc：C++ 采集包（CUDA 缩放开）编不了"
fi
emit "  /usr/local/cuda*: $(ls -d /usr/local/cuda* 2>/dev/null | tr '\n' ' ')"
DPKG_LIST="$(dpkg -l 2>/dev/null || true)"
if grep -qE '^ii +tensorrt' <<<"${DPKG_LIST}"; then
  ok "TensorRT 包: $(grep -E '^ii +tensorrt' <<<"${DPKG_LIST}" | awk '{print $2, $3}' | head -1)"
else
  warn "dpkg 里没查到 tensorrt 包（可能来自其它途径，继续看 python 侧）"
fi
python3 - <<'PY' 2>/dev/null | while IFS= read -r line; do emit "  ${line}"; done
mods = ["tensorrt", "torch", "torchvision", "numpy", "cv2", "ultralytics", "pycuda"]
for m in mods:
    try:
        mod = __import__(m)
        v = getattr(mod, "__version__", "?")
        extra = ""
        if m == "torch":
            extra = f" cuda_available={mod.cuda.is_available()} cuda={mod.version.cuda}"
        print(f"{m}={v}{extra}")
    except Exception as e:
        print(f"{m}=MISSING ({type(e).__name__})")
PY

# ── 3. Miivii GMSL SDK ────────────────────────────────────────
sec "3. Miivii GMSL SDK（C++ 采集必需）"
if [[ -f /opt/miivii/include/MvGmslCamera.h && -f /opt/miivii/lib/libmvgmslcamera.so ]]; then
  ok "SDK 在位：/opt/miivii/include/MvGmslCamera.h + lib/libmvgmslcamera.so"
else
  fail "SDK 缺失：CMake 会跳过 gmsl_camera_node_cpp（采集节点不会编译出来）"
  emit "         期望: /opt/miivii/include/MvGmslCamera.h 与 /opt/miivii/lib/libmvgmslcamera.so"
fi
[[ -d /opt/miivii ]] && emit "  /opt/miivii 内容: $(ls /opt/miivii 2>/dev/null | tr '\n' ' ')"
grep -iE 'miivii|gmsl' <<<"${DPKG_LIST:-$(dpkg -l 2>/dev/null || true)}" | head -3 | sed 's/^/  /' >>"${LOG}" || true

# ── 4. 相机与 video 节点 ──────────────────────────────────────
sec "4. 相机 / video 设备（device_path 必须按这块板重定）"
if have v4l2-ctl; then
  v4l2-ctl --list-devices 2>/dev/null | sed 's/^/  /' | tee -a "${LOG}" >/dev/null
  emit "  （完整列表见日志；下面只列 /dev/video* 与 sysfs 名称）"
  [[ "${QUIET}" -eq 0 ]] && v4l2-ctl --list-devices 2>/dev/null | sed 's/^/  /'
else
  warn "v4l2-ctl 缺失（可选：sudo apt install v4l-utils）；下面用 /dev/video* 与 sysfs 名称兜底"
fi
emit "  /dev/video*: $(ls /dev/video* 2>/dev/null | tr '\n' ' ')"
for d in /sys/class/video4linux/video*; do
  [[ -r "${d}/name" ]] && emit "    $(basename "${d}") → $(cat "${d}/name")"
done
emit "  注意：GMSL 的 SDK 索引(00/01/02) 与 /dev/videoN 的对应关系跟板上设备树有关，"
emit "        换板后必须重新核对，然后改 config/gmsl_camera_cpp_{front,right,left}.yaml 的 device_path。"

# ── 5. GStreamer ─────────────────────────────────────────────
sec "5. GStreamer（推流/取帧依赖）"
for p in v4l2src appsink videoconvert nvvidconv nvv4l2h264enc h264parse rtph264pay; do
  if have gst-inspect-1.0 && gst-inspect-1.0 "${p}" >/dev/null 2>&1; then
    ok "plugin ${p}"
  else
    fail "plugin 缺失: ${p}（推流或取帧会失败）"
  fi
done

# ── 6. ROS 2 与工作区 ────────────────────────────────────────
sec "6. ROS 2"
# 注意：ROS 的 setup.bash 会引用未定义变量，set -u 下直接 abort；这里临时关掉
ros_env() { ( set +u; source /opt/ros/humble/setup.bash >/dev/null 2>&1; eval "$*" ); }

if [[ -d /opt/ros/humble ]]; then
  ok "ROS 2 Humble 在位（/opt/ros/humble）"
  if ros_env 'have ros2'; then
    ok "ros2 可执行"
    ros_env 'ros2 pkg list 2>/dev/null' | grep -E '^usv_vision' | sed 's/^/  已编译包: /' >>"${LOG}"
    [[ "${QUIET}" -eq 0 ]] && ros_env 'ros2 pkg list 2>/dev/null' | grep -E '^usv_vision' | sed 's/^/  已编译包: /' || true
  else
    fail "ros2 命令不可用"
  fi
else
  fail "没有 /opt/ros/humble：先装 ROS 2 Humble（或改 setup 脚本里的发行版）"
fi
for pkg in cv-bridge nlohmann-json3-dev libgstreamer1.0-dev; do
  if grep -qxE "(ros-humble-)?${pkg}(:[a-z0-9]+)?" <<<"$(awk '{print $2}' <<<"${DPKG_LIST}" | sed 's/:.*//')"; then
    ok "apt 包 ${pkg}"
  else
    warn "apt 包 ${pkg} 未见（编译时若报缺再装）"
  fi
done

# ── 7. 模型与 engine ─────────────────────────────────────────
sec "7. 模型 / engine（engine 与 GPU+TRT 版本绑定，换板必须重导）"
shopt -s nullglob
engines=( ./model/jetson/*.engine ./../model/jetson/*.engine )
if [[ ${#engines[@]} -gt 0 ]]; then
  for e in "${engines[@]}"; do ok "engine 存在: ${e}"; done
else
  warn "当前目录下没有 model/jetson/*.engine：起识别前要先用 yolo11n.pt 导出（见 00_先读我 第三节）"
fi
shopt -u nullglob

# ── 8. 结论 ─────────────────────────────────────────────────
sec "结论"
emit "  FAIL ${FAIL} 项 / WARN ${WARN} 项   日志: ${LOG}"
if [[ ${FAIL} -eq 0 ]]; then
  emit "  硬件与依赖侧没有硬缺项：可继续编译 + 重导 engine + 改 device_path。"
else
  emit "  有 ${FAIL} 项硬缺：先按上面 [FAIL] 补齐，再谈编译和迁移。"
fi
emit ""
emit "  迁移必做三件事（脚本不会替你做）："
emit "   1) 改 config/gmsl_camera_cpp_{front,right,left}.yaml 的 device_path（按第 4 节核对）"
emit "   2) 重导 engine：python3 -c \"from ultralytics import YOLO; YOLO('yolo11n.pt').export(format='engine', half=True, imgsz=640, device=0)\""
emit "   3) 标定文件跟相机序列号绑定：换相机必须重标定，否则角度/内参是错的"
exit 0

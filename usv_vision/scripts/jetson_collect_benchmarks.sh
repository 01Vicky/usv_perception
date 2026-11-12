#!/usr/bin/env bash
set -euo pipefail

CASE_NAME=""
MODE=""
LAUNCH_CMD=""
TOPIC=""
GST_CMD=""
DURATION="20"
RESULTS_DIR="${RESULTS_DIR:-$(pwd)/test_logs/benchmarks}"  # 默认落在当前目录；换机器不用改
TARGET_RESOLUTION=""
DEVICE_OR_TOPIC=""
PIPELINE_TYPE=""
NOTES=""
STARTUP_WAIT="5"
ROS_NAMESPACE=""

print_help() {
  cat <<'EOF'
Usage:
  jetson_collect_benchmarks.sh --case-name NAME --mode MODE [options]

Modes:
  gst   Run a gst command and parse fps from fpsdisplaysink output
  ros   Run a ROS launch command, auto-isolate namespace, then sample ros2 topic hz/bw
  rtsp  Run a gst/rtsp receive command and parse fps from fpsdisplaysink output

Options:
  --case-name NAME
  --mode gst|ros|rtsp
  --launch-cmd "ros2 launch ..."
  --topic /topic/name
  --gst-cmd "gst-launch-1.0 ..."
  --duration SECONDS
  --results-dir PATH
  --target-resolution WxH
  --device-or-topic VALUE
  --pipeline-type VALUE
  --notes TEXT
  --startup-wait SECONDS
  --namespace VALUE
  --help
EOF
}

while (($#)); do
  case "$1" in
    --case-name)
      CASE_NAME="$2"
      shift 2
      ;;
    --mode)
      MODE="$2"
      shift 2
      ;;
    --launch-cmd)
      LAUNCH_CMD="$2"
      shift 2
      ;;
    --topic)
      TOPIC="$2"
      shift 2
      ;;
    --gst-cmd)
      GST_CMD="$2"
      shift 2
      ;;
    --duration)
      DURATION="$2"
      shift 2
      ;;
    --results-dir)
      RESULTS_DIR="$2"
      shift 2
      ;;
    --target-resolution)
      TARGET_RESOLUTION="$2"
      shift 2
      ;;
    --device-or-topic)
      DEVICE_OR_TOPIC="$2"
      shift 2
      ;;
    --pipeline-type)
      PIPELINE_TYPE="$2"
      shift 2
      ;;
    --notes)
      NOTES="$2"
      shift 2
      ;;
    --startup-wait)
      STARTUP_WAIT="$2"
      shift 2
      ;;
    --namespace)
      ROS_NAMESPACE="$2"
      shift 2
      ;;
    --help)
      print_help
      exit 0
      ;;
    *)
      echo "unknown argument: $1" >&2
      exit 2
      ;;
  esac
done

if [[ -z "$CASE_NAME" || -z "$MODE" ]]; then
  print_help >&2
  exit 2
fi

if [[ "$MODE" == "ros" ]]; then
  [[ -n "$LAUNCH_CMD" && -n "$TOPIC" ]] || { echo "ros mode requires --launch-cmd and --topic" >&2; exit 2; }
fi

if [[ "$MODE" == "gst" || "$MODE" == "rtsp" ]]; then
  [[ -n "$GST_CMD" ]] || { echo "$MODE mode requires --gst-cmd" >&2; exit 2; }
fi

mkdir -p "$RESULTS_DIR"

STDOUT_LOG="${RESULTS_DIR}/${CASE_NAME}_stdout.log"
TEGRA_LOG="${RESULTS_DIR}/${CASE_NAME}_tegrastats.log"
SUMMARY_JSON="${RESULTS_DIR}/${CASE_NAME}_summary.json"
RESULTS_CSV="${RESULTS_DIR}/benchmark_results.csv"
HZ_LOG="${RESULTS_DIR}/${CASE_NAME}_topic_hz.log"
BW_LOG="${RESULTS_DIR}/${CASE_NAME}_topic_bw.log"
FRAME_INFO_LOG="${RESULTS_DIR}/${CASE_NAME}_frame_info.log"
TOPIC_INFO_LOG="${RESULTS_DIR}/${CASE_NAME}_topic_info.log"
FIRST_MESSAGE_LOG="${RESULTS_DIR}/${CASE_NAME}_first_message.log"

PIDS=()
PGIDS=()

cleanup() {
  for pid in "${PIDS[@]:-}"; do
    if kill -0 "$pid" >/dev/null 2>&1; then
      kill "$pid" >/dev/null 2>&1 || true
      wait "$pid" >/dev/null 2>&1 || true
    fi
  done
  for pgid in "${PGIDS[@]:-}"; do
    if kill -0 "-$pgid" >/dev/null 2>&1; then
      kill "-$pgid" >/dev/null 2>&1 || true
      sleep 1
      kill -9 "-$pgid" >/dev/null 2>&1 || true
    fi
  done
}
trap cleanup EXIT

run_bg() {
  "$@" &
  PIDS+=("$!")
}

command_exists() {
  command -v "$1" >/dev/null 2>&1
}

json_escape() {
  printf '%s' "$1" | sed 's/\\/\\\\/g; s/"/\\"/g'
}

csv_escape() {
  local value="$1"
  value=${value//\"/\"\"}
  printf '"%s"' "$value"
}

sanitize_namespace() {
  local raw="$1"
  raw=$(printf '%s' "$raw" | tr '[:upper:]' '[:lower:]')
  raw=$(printf '%s' "$raw" | sed 's/[^a-z0-9_]/_/g; s/__*/_/g; s/^_//; s/_$//')
  if [[ -z "$raw" ]]; then
    raw="bench"
  fi
  printf '%s' "$raw"
}

resolve_ros_namespace() {
  if [[ -n "$ROS_NAMESPACE" ]]; then
    sanitize_namespace "$ROS_NAMESPACE"
  else
    sanitize_namespace "bench_${CASE_NAME}"
  fi
}

rewrite_topic_namespace() {
  local topic="$1"
  local namespace="$2"
  local suffix="$topic"
  suffix="${suffix#/}"
  if [[ "$suffix" == */* ]]; then
    suffix="${suffix#*/}"
  fi
  printf '/%s/%s' "$namespace" "$suffix"
}

rewrite_launch_namespace() {
  local launch_cmd="$1"
  local namespace="$2"
  if [[ "$launch_cmd" == *"namespace:="* ]]; then
    printf '%s' "$launch_cmd" | sed -E "s/namespace:=[^ ]+/namespace:=${namespace}/g"
  else
    printf '%s namespace:=%s' "$launch_cmd" "$namespace"
  fi
}

wait_for_ros_topic() {
  local topic="$1"
  local timeout_seconds="$2"
  local waited=0
  while (( waited < timeout_seconds )); do
    if ros2 topic info "$topic" >"$TOPIC_INFO_LOG" 2>&1; then
      if ! grep -q "Unknown topic" "$TOPIC_INFO_LOG"; then
        return 0
      fi
    fi
    sleep 1
    waited=$((waited + 1))
  done
  return 1
}

wait_for_ros_message() {
  local topic="$1"
  local timeout_seconds="$2"
  timeout "${timeout_seconds}s" ros2 topic echo "$topic" --once >"$FIRST_MESSAGE_LOG" 2>&1
}

launch_ros_command() {
  local cmd="$1"
  setsid bash -lc "$cmd" >"$STDOUT_LOG" 2>&1 &
  local launch_pid=$!
  local launch_pgid
  launch_pgid=$(ps -o pgid= "$launch_pid" | tr -d '[:space:]')
  PIDS+=("$launch_pid")
  if [[ -n "$launch_pgid" ]]; then
    PGIDS+=("$launch_pgid")
  fi
}

extract_fps_from_fpsdisplay() {
  local log_file="$1"
  awk '
    BEGIN { sum=0; count=0; min=""; max="" }
    /average:/ {
      for (i = 1; i <= NF; i++) {
        if ($i == "average:") {
          val = $(i+1) + 0.0
          sum += val
          count += 1
          if (min == "" || val < min) min = val
          if (max == "" || val > max) max = val
        }
      }
    }
    /current:/ {
      for (i = 1; i <= NF; i++) {
        gsub(",", "", $i)
        if ($i == "current:") {
          val = $(i+1) + 0.0
          sum += val
          count += 1
          if (min == "" || val < min) min = val
          if (max == "" || val > max) max = val
        }
      }
    }
    END {
      if (count == 0) {
        print "NA,NA,NA"
      } else {
        printf "%.3f,%.3f,%.3f\n", sum / count, min, max
      }
    }
  ' "$log_file"
}

extract_hz_from_ros2_topic_hz() {
  local log_file="$1"
  awk '
    BEGIN { sum=0; count=0; min=""; max="" }
    /average rate:/ {
      val = $3 + 0.0
      sum += val
      count += 1
      if (min == "" || val < min) min = val
      if (max == "" || val > max) max = val
    }
    END {
      if (count == 0) {
        print "NA,NA,NA"
      } else {
        printf "%.3f,%.3f,%.3f\n", sum / count, min, max
      }
    }
  ' "$log_file"
}

extract_bw_from_ros2_topic_bw() {
  local log_file="$1"
  awk '
    BEGIN { bw_sum=0; bw_count=0; msg_sum=0; msg_count=0 }
    /MB\/s from/ {
      val = $1 + 0.0
      bw_sum += val
      bw_count += 1
    }
    /Message size mean:/ {
      val = $4 + 0.0
      msg_sum += val
      msg_count += 1
    }
    END {
      bw = (bw_count == 0) ? "NA" : sprintf("%.3f", bw_sum / bw_count)
      msg = (msg_count == 0) ? "NA" : sprintf("%.3f", msg_sum / msg_count)
      printf "%s,%s\n", msg, bw
    }
  ' "$log_file"
}

extract_resolution_from_frame_info() {
  local log_file="$1"
  python3 - "$log_file" <<'PY'
import json
import re
import sys

path = sys.argv[1]
text = open(path, "r", encoding="utf-8", errors="ignore").read()
match = re.search(r'data:\s*\'([^\']+)\'', text)
if not match:
    print("NA")
    raise SystemExit
payload = match.group(1)
try:
    obj = json.loads(payload)
    image = obj.get("image", {})
    width = image.get("width")
    height = image.get("height")
    if width and height:
        print(f"{width}x{height}")
    else:
        print("NA")
except Exception:
    print("NA")
PY
}

extract_tegrastats_summary() {
  local log_file="$1"
  python3 - "$log_file" <<'PY'
import re
import sys

path = sys.argv[1]
cpu_vals = []
gpu_vals = []
temp_vals = []

cpu_re = re.compile(r'CPU \[([^\]]+)\]')
gpu_re = re.compile(r'GR3D_FREQ (\d+)%')
temp_re = re.compile(r'([A-Za-z0-9_]+)@(-?\d+(?:\.\d+)?)C')

for line in open(path, "r", encoding="utf-8", errors="ignore"):
    m = cpu_re.search(line)
    if m:
        parts = [p.strip() for p in m.group(1).split(',')]
        vals = []
        for p in parts:
            if p.endswith('%'):
                try:
                    vals.append(float(p[:-1]))
                except ValueError:
                    pass
        if vals:
            cpu_vals.append(sum(vals) / len(vals))
    m = gpu_re.search(line)
    if m:
        gpu_vals.append(float(m.group(1)))
    for label, temp in temp_re.findall(line):
        if label in ("GPU", "CPU", "AO", "Tdiode", "SOC"):
            try:
                temp_vals.append(float(temp))
                break
            except ValueError:
                pass

def avg(vals):
    if not vals:
        return "NA"
    return f"{sum(vals)/len(vals):.3f}"

print(",".join([avg(cpu_vals), avg(gpu_vals), avg(temp_vals)]))
PY
}

start_tegrastats() {
  if command_exists tegrastats; then
    run_bg tegrastats --interval 1000 --logfile "$TEGRA_LOG"
  else
    : > "$TEGRA_LOG"
  fi
}

run_gst_mode() {
  start_tegrastats
  timeout "${DURATION}s" bash -lc "$GST_CMD" >"$STDOUT_LOG" 2>&1 || true
}

run_ros_mode() {
  local ros_namespace
  local resolved_launch_cmd
  local resolved_topic
  local resolved_frame_timing_topic

  ros_namespace="$(resolve_ros_namespace)"
  resolved_launch_cmd="$(rewrite_launch_namespace "$LAUNCH_CMD" "$ros_namespace")"
  resolved_topic="$(rewrite_topic_namespace "$TOPIC" "$ros_namespace")"
  resolved_frame_timing_topic="${resolved_topic%/image_raw}/frame_timing_json"

  start_tegrastats
  launch_ros_command "$resolved_launch_cmd"
  sleep "$STARTUP_WAIT"

  if ! wait_for_ros_topic "$resolved_topic" 10; then
    printf 'timed out waiting for topic %s\n' "$resolved_topic" >>"$STDOUT_LOG"
  fi

  TOPIC="$resolved_topic"
  DEVICE_OR_TOPIC="$resolved_topic"
  if [[ -n "$NOTES" ]]; then
    NOTES="${NOTES}; namespace=${ros_namespace}"
  else
    NOTES="namespace=${ros_namespace}"
  fi

  ros2 topic info -v "$resolved_topic" >"$TOPIC_INFO_LOG" 2>&1 || true
  if ! wait_for_ros_message "$resolved_topic" 10; then
    printf 'timed out waiting for first message on %s\n' "$resolved_topic" >>"$STDOUT_LOG"
  fi
  timeout "${DURATION}s" ros2 topic hz "$resolved_topic" >"$HZ_LOG" 2>&1 || true
  timeout "${DURATION}s" ros2 topic bw "$resolved_topic" >"$BW_LOG" 2>&1 || true
  timeout 10s ros2 topic echo "$resolved_frame_timing_topic" --once >"$FRAME_INFO_LOG" 2>&1 || true
}

append_csv_header_if_needed() {
  if [[ ! -f "$RESULTS_CSV" ]]; then
    printf '%s\n' \
      "case_name,device_or_topic,pipeline_type,target_resolution,observed_resolution,fps_mean,fps_min,fps_max,msg_size_mb,bandwidth_mb_s,cpu_percent_avg,gpu_percent_avg,temp_c_avg,notes" \
      >"$RESULTS_CSV"
  fi
}

write_summary() {
  local fps_mean="NA"
  local fps_min="NA"
  local fps_max="NA"
  local msg_size_mb="NA"
  local bandwidth_mb_s="NA"
  local observed_resolution="NA"
  local cpu_percent_avg="NA"
  local gpu_percent_avg="NA"
  local temp_c_avg="NA"

  if [[ "$MODE" == "ros" ]]; then
    IFS=, read -r fps_mean fps_min fps_max <<<"$(extract_hz_from_ros2_topic_hz "$HZ_LOG")"
    IFS=, read -r msg_size_mb bandwidth_mb_s <<<"$(extract_bw_from_ros2_topic_bw "$BW_LOG")"
    observed_resolution="$(extract_resolution_from_frame_info "$FRAME_INFO_LOG")"
  else
    IFS=, read -r fps_mean fps_min fps_max <<<"$(extract_fps_from_fpsdisplay "$STDOUT_LOG")"
  fi

  IFS=, read -r cpu_percent_avg gpu_percent_avg temp_c_avg <<<"$(extract_tegrastats_summary "$TEGRA_LOG")"

  if [[ -z "$DEVICE_OR_TOPIC" ]]; then
    if [[ "$MODE" == "ros" ]]; then
      DEVICE_OR_TOPIC="$TOPIC"
    else
      DEVICE_OR_TOPIC="NA"
    fi
  fi

  if [[ -z "$PIPELINE_TYPE" ]]; then
    PIPELINE_TYPE="$MODE"
  fi

  cat >"$SUMMARY_JSON" <<EOF
{
  "case_name": "$(json_escape "$CASE_NAME")",
  "mode": "$(json_escape "$MODE")",
  "device_or_topic": "$(json_escape "$DEVICE_OR_TOPIC")",
  "pipeline_type": "$(json_escape "$PIPELINE_TYPE")",
  "target_resolution": "$(json_escape "$TARGET_RESOLUTION")",
  "observed_resolution": "$(json_escape "$observed_resolution")",
  "fps_mean": "$fps_mean",
  "fps_min": "$fps_min",
  "fps_max": "$fps_max",
  "msg_size_mb": "$msg_size_mb",
  "bandwidth_mb_s": "$bandwidth_mb_s",
  "cpu_percent_avg": "$cpu_percent_avg",
  "gpu_percent_avg": "$gpu_percent_avg",
  "temp_c_avg": "$temp_c_avg",
  "notes": "$(json_escape "$NOTES")"
}
EOF

  append_csv_header_if_needed
  printf '%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s\n' \
    "$(csv_escape "$CASE_NAME")" \
    "$(csv_escape "$DEVICE_OR_TOPIC")" \
    "$(csv_escape "$PIPELINE_TYPE")" \
    "$(csv_escape "$TARGET_RESOLUTION")" \
    "$(csv_escape "$observed_resolution")" \
    "$(csv_escape "$fps_mean")" \
    "$(csv_escape "$fps_min")" \
    "$(csv_escape "$fps_max")" \
    "$(csv_escape "$msg_size_mb")" \
    "$(csv_escape "$bandwidth_mb_s")" \
    "$(csv_escape "$cpu_percent_avg")" \
    "$(csv_escape "$gpu_percent_avg")" \
    "$(csv_escape "$temp_c_avg")" \
    "$(csv_escape "$NOTES")" \
    >>"$RESULTS_CSV"

  cat "$SUMMARY_JSON"
}

case "$MODE" in
  gst|rtsp)
    run_gst_mode
    ;;
  ros)
    run_ros_mode
    ;;
  *)
    echo "unsupported mode: $MODE" >&2
    exit 2
    ;;
esac

cleanup
trap - EXIT
write_summary

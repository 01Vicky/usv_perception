#!/usr/bin/env bash
set -euo pipefail

FRONT_DEVICE="${FRONT_DEVICE:-/dev/video4}"
REAR_DEVICE="${REAR_DEVICE:-/dev/video5}"
WIDTH="${WIDTH:-1920}"
HEIGHT="${HEIGHT:-1536}"
FPS="${FPS:-30}"
PIXEL_FORMAT="${PIXEL_FORMAT:-UYVY}"
RTSP_URL="${RTSP_URL:-rtsp://127.0.0.1:8554/cam1}"
LOG_DIR="${LOG_DIR:-$(pwd)/gmsl_check_logs}"
CHECK_ROS="${CHECK_ROS:-0}"
ROS_NAMESPACE_FRONT="${ROS_NAMESPACE_FRONT:-/front_gmsl}"
ROS_NAMESPACE_REAR="${ROS_NAMESPACE_REAR:-/rear_gmsl}"

mkdir -p "${LOG_DIR}"
LOG_FILE="${LOG_DIR}/gmsl_check_$(date +%Y%m%d_%H%M%S).log"

log() {
  printf '%s %s\n' "[$(date '+%F %T')]" "$*" | tee -a "${LOG_FILE}"
}

run_and_log() {
  log "+ $*"
  "$@" 2>&1 | tee -a "${LOG_FILE}"
}

check_cmd() {
  if command -v "$1" >/dev/null 2>&1; then
    log "OK command: $1"
    return 0
  fi
  log "FAIL command missing: $1"
  return 1
}

check_path() {
  if [[ -e "$1" ]]; then
    log "OK path exists: $1"
    return 0
  fi
  log "FAIL path missing: $1"
  return 1
}

check_gst_plugin() {
  local plugin="$1"
  if gst-inspect-1.0 "$plugin" >/dev/null 2>&1; then
    log "OK gst plugin: $plugin"
    return 0
  fi
  log "FAIL gst plugin missing: $plugin"
  return 1
}

check_single_frame_capture() {
  local device="$1"
  local role="$2"
  log "Checking single-frame capture for ${role}: ${device}"
  gst-launch-1.0 -e \
    v4l2src device="${device}" num-buffers=1 ! \
    "video/x-raw,format=${PIXEL_FORMAT},width=${WIDTH},height=${HEIGHT},framerate=${FPS}/1" ! \
    videoconvert ! \
    fakesink sync=false >/dev/null 2>&1
  log "OK single-frame capture: ${role}"
}

check_rtsp_encode_path() {
  local device="$1"
  local role="$2"
  log "Checking RTSP encode path prerequisites for ${role}: ${device}"
  gst-launch-1.0 -e \
    v4l2src device="${device}" num-buffers=1 ! \
    "video/x-raw,format=${PIXEL_FORMAT},width=${WIDTH},height=${HEIGHT},framerate=${FPS}/1" ! \
    nvvidconv ! \
    nvv4l2h264enc bitrate=4000000 ! \
    h264parse ! \
    fakesink sync=false >/dev/null 2>&1
  log "OK encode path: ${role}"
}

check_ros_topics() {
  if ! command -v ros2 >/dev/null 2>&1; then
    log "SKIP ros2 topic checks: ros2 command missing"
    return 0
  fi
  log "Checking ROS topics"
  run_and_log ros2 topic list
  run_and_log ros2 node list
  run_and_log ros2 topic echo "${ROS_NAMESPACE_FRONT}/driver_status" --once
  run_and_log ros2 topic echo "${ROS_NAMESPACE_FRONT}/frame_timing_json" --once
  run_and_log ros2 topic hz "${ROS_NAMESPACE_FRONT}/image_raw"
}

main() {
  log "GMSL health check started"
  log "Log file: ${LOG_FILE}"
  log "Front device: ${FRONT_DEVICE}"
  log "Rear device: ${REAR_DEVICE}"
  log "Resolution: ${WIDTH}x${HEIGHT} @ ${FPS}fps format=${PIXEL_FORMAT}"
  log "RTSP URL: ${RTSP_URL}"

  check_cmd gst-launch-1.0
  check_cmd gst-inspect-1.0

  check_path "${FRONT_DEVICE}"
  check_path "${REAR_DEVICE}"

  run_and_log ls -l /dev/video*

  check_gst_plugin v4l2src
  check_gst_plugin videoconvert
  check_gst_plugin appsink
  check_gst_plugin nvvidconv
  check_gst_plugin nvv4l2h264enc
  check_gst_plugin h264parse

  check_single_frame_capture "${FRONT_DEVICE}" "front"
  check_single_frame_capture "${REAR_DEVICE}" "rear"

  check_rtsp_encode_path "${REAR_DEVICE}" "rear"

  log "Reference RTSP push command:"
  log "bash src/usv_perception/usv_vision/scripts/gmsl_to_rtsp.sh ${REAR_DEVICE} ${RTSP_URL}"

  if [[ "${CHECK_ROS}" == "1" ]]; then
    check_ros_topics
  else
    log "ROS topic checks skipped. Set CHECK_ROS=1 to enable."
  fi

  log "GMSL health check completed"
}

main "$@"

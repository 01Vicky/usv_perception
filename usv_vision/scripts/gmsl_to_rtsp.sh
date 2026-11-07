#!/usr/bin/env bash
set -euo pipefail

DEVICE="${1:-/dev/video5}"
RTSP_URL="${2:-rtsp://127.0.0.1:8554/cam1}"
WIDTH="${WIDTH:-1920}"
HEIGHT="${HEIGHT:-1536}"
FPS="${FPS:-30}"
BITRATE="${BITRATE:-8000000}"
PIXEL_FORMAT="${PIXEL_FORMAT:-UYVY}"
ENCODER_FORMAT="${ENCODER_FORMAT:-NV12}"
IFRAME_INTERVAL="${IFRAME_INTERVAL:-30}"
IDR_INTERVAL="${IDR_INTERVAL:-30}"
CONTROL_RATE="${CONTROL_RATE:-1}"
INSERT_SPS_PPS="${INSERT_SPS_PPS:-1}"
INSERT_VUI="${INSERT_VUI:-1}"
MAXPERF_ENABLE="${MAXPERF_ENABLE:-1}"
PRESET_LEVEL="${PRESET_LEVEL:-1}"

exec gst-launch-1.0 -v \
  v4l2src device="${DEVICE}" ! \
  video/x-raw,format="${PIXEL_FORMAT}",width="${WIDTH}",height="${HEIGHT}",framerate="${FPS}/1" ! \
  nvvidconv ! \
  video/x-raw\(memory:NVMM\),format="${ENCODER_FORMAT}",width="${WIDTH}",height="${HEIGHT}",framerate="${FPS}/1" ! \
  nvv4l2h264enc \
    bitrate="${BITRATE}" \
    control-rate="${CONTROL_RATE}" \
    iframeinterval="${IFRAME_INTERVAL}" \
    idrinterval="${IDR_INTERVAL}" \
    insert-sps-pps="${INSERT_SPS_PPS}" \
    insert-vui="${INSERT_VUI}" \
    maxperf-enable="${MAXPERF_ENABLE}" \
    preset-level="${PRESET_LEVEL}" ! \
  h264parse config-interval=-1 disable-passthrough=true ! \
  rtspclientsink location="${RTSP_URL}"

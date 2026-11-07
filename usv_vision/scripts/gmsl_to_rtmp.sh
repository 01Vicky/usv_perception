#!/usr/bin/env bash
set -euo pipefail

print_help() {
  cat <<'EOF'
Usage:
  gmsl_to_rtmp.sh [RTMP_URL]

Environment variables:
  INPUT_MODE=v4l2|rtsp        Default: v4l2
  DEVICE=/dev/video5          Used when INPUT_MODE=v4l2
  INPUT_URL=rtsp://...        Used when INPUT_MODE=rtsp
  WIDTH=1920
  HEIGHT=1536
  FPS=30
  PIXEL_FORMAT=uyvy422        ffmpeg v4l2 input pixel format
  RTSP_TRANSPORT=tcp          Used when INPUT_MODE=rtsp
  VIDEO_CODEC=libx264
  PRESET=veryfast
  TUNE=zerolatency
  GOP=30
  VIDEO_BITRATE=8M
  MAXRATE=8M
  BUFSIZE=16M
  EXTRA_FFMPEG_ARGS=""        Extra ffmpeg arguments appended before output URL

Examples:
  bash gmsl_to_rtmp.sh rtmp://101.43.14.147:1935/live/test

  INPUT_MODE=rtsp \
  INPUT_URL=rtsp://127.0.0.1:8554/cam1 \
  bash gmsl_to_rtmp.sh rtmp://101.43.14.147:1935/live/test
EOF
}

if [[ "${1:-}" == "--help" || "${1:-}" == "-h" ]]; then
  print_help
  exit 0
fi

INPUT_MODE="${INPUT_MODE:-v4l2}"
DEVICE="${DEVICE:-/dev/video5}"
INPUT_URL="${INPUT_URL:-}"
RTMP_URL="${1:-${RTMP_URL:-rtmp://127.0.0.1:1935/live/gmsl5}}"
WIDTH="${WIDTH:-1920}"
HEIGHT="${HEIGHT:-1536}"
FPS="${FPS:-30}"
PIXEL_FORMAT="${PIXEL_FORMAT:-uyvy422}"
RTSP_TRANSPORT="${RTSP_TRANSPORT:-tcp}"
VIDEO_CODEC="${VIDEO_CODEC:-libx264}"
PRESET="${PRESET:-veryfast}"
TUNE="${TUNE:-zerolatency}"
GOP="${GOP:-30}"
VIDEO_BITRATE="${VIDEO_BITRATE:-8M}"
MAXRATE="${MAXRATE:-8M}"
BUFSIZE="${BUFSIZE:-16M}"
EXTRA_FFMPEG_ARGS="${EXTRA_FFMPEG_ARGS:-}"

if ! command -v ffmpeg >/dev/null 2>&1; then
  echo "ffmpeg not found in PATH" >&2
  exit 127
fi

if [[ "$INPUT_MODE" != "v4l2" && "$INPUT_MODE" != "rtsp" ]]; then
  echo "unsupported INPUT_MODE: $INPUT_MODE" >&2
  exit 2
fi

if [[ "$INPUT_MODE" == "rtsp" && -z "$INPUT_URL" ]]; then
  echo "INPUT_MODE=rtsp requires INPUT_URL" >&2
  exit 2
fi

echo "starting RTMP push"
echo "  input_mode: $INPUT_MODE"
echo "  output: $RTMP_URL"

if [[ "$INPUT_MODE" == "v4l2" ]]; then
  echo "  device: $DEVICE"
  echo "  size: ${WIDTH}x${HEIGHT}@${FPS}"
  exec ffmpeg \
    -hide_banner \
    -loglevel info \
    -fflags nobuffer \
    -flags low_delay \
    -thread_queue_size 512 \
    -f v4l2 \
    -input_format "$PIXEL_FORMAT" \
    -video_size "${WIDTH}x${HEIGHT}" \
    -framerate "$FPS" \
    -i "$DEVICE" \
    -an \
    -c:v "$VIDEO_CODEC" \
    -pix_fmt yuv420p \
    -preset "$PRESET" \
    -tune "$TUNE" \
    -profile:v high \
    -bf 0 \
    -g "$GOP" \
    -b:v "$VIDEO_BITRATE" \
    -maxrate "$MAXRATE" \
    -bufsize "$BUFSIZE" \
    ${EXTRA_FFMPEG_ARGS} \
    -f flv \
    "$RTMP_URL"
fi

echo "  input_url: $INPUT_URL"
exec ffmpeg \
  -hide_banner \
  -loglevel info \
  -fflags nobuffer \
  -flags low_delay \
  -rtsp_transport "$RTSP_TRANSPORT" \
  -i "$INPUT_URL" \
  -an \
  -c:v "$VIDEO_CODEC" \
  -pix_fmt yuv420p \
  -preset "$PRESET" \
  -tune "$TUNE" \
  -profile:v high \
  -bf 0 \
  -g "$GOP" \
  -b:v "$VIDEO_BITRATE" \
  -maxrate "$MAXRATE" \
  -bufsize "$BUFSIZE" \
  ${EXTRA_FFMPEG_ARGS} \
  -f flv \
  "$RTMP_URL"

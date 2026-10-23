#!/usr/bin/env bash
set -eo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WORKSPACE="${PROJECT_ROOT}/ros2_ws"
UNDERLAY="${ROS2_UNDERLAY:-/home/ac/ros2_humble/install/setup.bash}"
PORT="${LANE_VIEWER_PORT:-8080}"
MAX_FPS="${LANE_VIEWER_MAX_FPS:-5.0}"
JPEG_QUALITY="${LANE_VIEWER_JPEG_QUALITY:-60}"
SHOW_SEGMENTATION="${LANE_VIEWER_SHOW_SEGMENTATION:-false}"

ROI_TOP_RATIO="${LANE_SCAN_TOP_RATIO:-0.40}"
ROI_BOTTOM_RATIO="${LANE_SCAN_BOTTOM_RATIO:-0.63}"

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  echo "Usage: observe_lane_web.sh"
  echo "Chạy web viewer lane localhost; đổi port bằng LANE_VIEWER_PORT."
  echo "Đổi ROI bằng LANE_SCAN_TOP_RATIO và LANE_SCAN_BOTTOM_RATIO."
  echo "Mặc định chỉ mở Tracking / Output ở 5 FPS, JPEG quality 60."
  echo "Bật lại Segmentation bằng LANE_VIEWER_SHOW_SEGMENTATION=true."
  exit 0
elif (($#)); then
  echo "Tham số không hợp lệ: $1" >&2
  exit 2
fi

if [[ ! -f "${UNDERLAY}" || ! -f "${WORKSPACE}/install/setup.bash" ]]; then
  echo "Thiếu ROS 2 underlay hoặc V2 install. Hãy build project trước." >&2
  exit 2
fi

set +u
source "${UNDERLAY}"
source "${WORKSPACE}/install/setup.bash"
set -u

echo "Web viewer tracking nhẹ đang chạy tại remote http://127.0.0.1:${PORT}"
echo "Hãy forward port ${PORT} trong VS Code rồi mở địa chỉ Forwarded Address."
echo "Mặc định đã tắt cửa sổ Segmentation; bật lại bằng LANE_VIEWER_SHOW_SEGMENTATION=true."

exec ros2 run hardware_camera web_viewer --ros-args \
  -p "port:=${PORT}" \
  -p "max_fps:=${MAX_FPS}" \
  -p "jpeg_quality:=${JPEG_QUALITY}" \
  -p "show_segmentation:=${SHOW_SEGMENTATION}" \
  -p "roi_top_ratio:=${ROI_TOP_RATIO}" \
  -p "roi_bottom_ratio:=${ROI_BOTTOM_RATIO}"

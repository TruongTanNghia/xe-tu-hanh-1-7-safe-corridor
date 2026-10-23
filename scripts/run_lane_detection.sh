#!/usr/bin/env bash
set -eo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WORKSPACE="${PROJECT_ROOT}/ros2_ws"
UNDERLAY="${ROS2_UNDERLAY:-/home/ac/ros2_humble/install/setup.bash}"

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  echo "Usage: run_lane_detection.sh [launch-args...]"
  echo "Launch hardware_camera + image_processing; không chạy control/actuation."
  echo "Ví dụ: run_lane_detection.sh manual_controls:=true exposure_time_us:=8000"
  exit 0
fi

if [[ ! -f "${UNDERLAY}" ]]; then
  echo "Không tìm thấy ROS 2 underlay: ${UNDERLAY}" >&2
  exit 2
fi
if [[ ! -f "${WORKSPACE}/install/setup.bash" ]]; then
  echo "Chưa có workspace install. Hãy chạy build_lane_detection.sh." >&2
  exit 3
fi

set +u
source "${UNDERLAY}"
source "${WORKSPACE}/install/setup.bash"
set -u

exec ros2 launch hardware_camera lane_detection.launch.py "$@"

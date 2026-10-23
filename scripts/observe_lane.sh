#!/usr/bin/env bash
set -eo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WORKSPACE="${PROJECT_ROOT}/ros2_ws"
UNDERLAY="${ROS2_UNDERLAY:-/home/ac/ros2_humble/install/setup.bash}"

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  echo "Usage: observe_lane.sh"
  echo "Mở camera_publisher trong terminal có DISPLAY/desktop GUI."
  exit 0
elif (($#)); then
  echo "Tham số không hợp lệ: $1" >&2
  exit 2
fi

if [[ ! -f "${UNDERLAY}" || ! -f "${WORKSPACE}/install/setup.bash" ]]; then
  echo "Thiếu ROS 2 underlay hoặc V2 install. Hãy build project trước." >&2
  exit 2
fi
if [[ -z "${DISPLAY:-}" ]]; then
  echo "DISPLAY chưa được đặt. Node quan sát cần terminal có desktop GUI." >&2
  exit 3
fi

set +u
source "${UNDERLAY}"
source "${WORKSPACE}/install/setup.bash"
set -u

exec ros2 run image_processing camera_publisher

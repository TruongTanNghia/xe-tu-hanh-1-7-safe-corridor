#!/usr/bin/env bash
set -eo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WORKSPACE="${PROJECT_ROOT}/ros2_ws"
UNDERLAY="${ROS2_UNDERLAY:-/home/ac/ros2_humble/install/setup.bash}"

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  echo "Usage: build_image_processing.sh"
  echo "Compatibility wrapper: build packages-up-to image_processing."
  exit 0
elif (($#)); then
  echo "Tham số không hợp lệ: $1" >&2
  exit 2
fi

if [[ ! -f "${UNDERLAY}" ]]; then
  echo "Không tìm thấy ROS 2 underlay: ${UNDERLAY}" >&2
  exit 2
fi

set +u
source "${UNDERLAY}"
set -u

cd "${WORKSPACE}"
colcon build \
  --packages-up-to image_processing \
  --event-handlers console_direct+

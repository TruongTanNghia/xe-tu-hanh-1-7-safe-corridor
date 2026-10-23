#!/usr/bin/env bash
set -Ee -o pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
UNDERLAY="${RC_ROS_UNDERLAY:-/home/ac/ros2_humble/install/setup.bash}"
OVERLAY="${PROJECT_ROOT}/ros2_ws/install/setup.bash"

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  echo "Usage: run_fusion.sh [ROS_LAUNCH_ARGUMENT ...]"
  echo "Launch A1M8/RF2O/BNO055/EKF; không chạy NMPC hoặc actuation."
  echo "Ví dụ: run_fusion.sh start_lidar:=false"
  exit 0
fi

if [[ ! -f "${UNDERLAY}" || ! -f "${OVERLAY}" ]]; then
  echo "ERROR: Build rc_localization trước khi chạy." >&2
  exit 1
fi

# shellcheck disable=SC1090
source "${UNDERLAY}"
# shellcheck disable=SC1090
source "${OVERLAY}"
set -u

exec ros2 launch rc_localization fusion_hardware.launch.py "$@"

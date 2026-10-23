#!/usr/bin/env bash
set -eo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WORKSPACE="${PROJECT_ROOT}/ros2_ws"
UNDERLAY="${ROS2_UNDERLAY:-/home/ac/ros2_humble/install/setup.bash}"

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  echo "Usage: capture_lane_pose_evidence.sh [--labels straight,enter_curve,exit_curve] [--output-dir PATH]"
  echo "Nhấn Space để lưu raw/debug/fitting/4-panel và snapshot các topic /line vào evidence."
  exit 0
fi

if [[ ! -f "${UNDERLAY}" || ! -f "${WORKSPACE}/install/setup.bash" ]]; then
  echo "Thiếu ROS 2 underlay hoặc workspace install. Hãy build project trước." >&2
  exit 2
fi

set +u
source "${UNDERLAY}"
source "${WORKSPACE}/install/setup.bash"
set -u

exec python3 "${PROJECT_ROOT}/scripts/capture_lane_pose_evidence.py" "$@"

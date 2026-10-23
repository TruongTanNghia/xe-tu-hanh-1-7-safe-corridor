#!/usr/bin/env bash
set -Eeuo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
UNDERLAY="${RC_ROS_UNDERLAY:-/home/ac/ros2_humble/install/setup.bash}"
OVERLAY="${PROJECT_ROOT}/ros2_ws/install/setup.bash"
TOPIC="${RC_SCAN_TOPIC:-/scan}"

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  echo "Usage: save_lidar_snapshot.sh [OUTPUT.svg]"
  echo "Lưu một /scan thành SVG và JSON; đổi topic bằng RC_SCAN_TOPIC."
  exit 0
elif (($# > 1)); then
  echo "Chỉ nhận tối đa một OUTPUT.svg." >&2
  exit 2
fi

STAMP="$(date +%Y%m%d-%H%M%S)"
OUTPUT="${1:-${PROJECT_ROOT}/evidence/lidar/${STAMP}/scan.svg}"

if [[ ! -f "${UNDERLAY}" || ! -f "${OVERLAY}" ]]; then
  echo "ERROR: Chưa có ROS underlay/overlay. Build rc_validation trước." >&2
  exit 1
fi

# shellcheck disable=SC1090
set +u
source "${UNDERLAY}"
# shellcheck disable=SC1090
source "${OVERLAY}"
set -u

ros2 run rc_validation lidar_scan_snapshot --ros-args \
  -p "topic:=${TOPIC}" \
  -p "output_path:=${OUTPUT}" \
  -p "timeout_sec:=10.0"

echo "Mở bằng VS Code:"
echo "  ${OUTPUT}"
echo "Dữ liệu số:"
echo "  ${OUTPUT%.svg}.json"

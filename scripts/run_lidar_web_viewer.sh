#!/usr/bin/env bash
set -Eeuo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
UNDERLAY="${RC_ROS_UNDERLAY:-/home/ac/ros2_humble/install/setup.bash}"
OVERLAY="${PROJECT_ROOT}/ros2_ws/install/setup.bash"
PORT="${RC_LIDAR_WEB_PORT:-8090}"
TOPIC="${RC_SCAN_TOPIC:-/scan}"

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  echo "Usage: run_lidar_web_viewer.sh"
  echo "Chạy viewer /scan localhost; đổi port/topic bằng RC_LIDAR_WEB_PORT/RC_SCAN_TOPIC."
  exit 0
elif (($#)); then
  echo "Tham số không hợp lệ: $1" >&2
  exit 2
fi

if [[ ! -f "${UNDERLAY}" || ! -f "${OVERLAY}" ]]; then
  echo "ERROR: Hãy build rc_validation trước:" >&2
  echo "  cd ${PROJECT_ROOT}/ros2_ws" >&2
  echo "  colcon build --symlink-install --packages-select rc_validation" >&2
  exit 1
fi

# shellcheck disable=SC1090
set +u
source "${UNDERLAY}"
# shellcheck disable=SC1090
source "${OVERLAY}"
set -u

echo "Mở panel PORTS của VS Code, forward port ${PORT}, rồi mở:"
echo "  http://127.0.0.1:${PORT}"
echo "Viewer subscribe ${TOPIC} bằng QoS sensor_data."

exec ros2 run rc_validation lidar_web_viewer --ros-args \
  -p "topic:=${TOPIC}" \
  -p "port:=${PORT}" \
  -p "bind_address:=127.0.0.1"

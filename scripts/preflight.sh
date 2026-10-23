#!/usr/bin/env bash
set -Eeuo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

usage() {
  cat <<'EOF'
Usage: preflight.sh [--hardware]

Mặc định kiểm tra build/package/config tĩnh. --hardware yêu cầu camera, LiDAR,
BNO055 và STM32 hiện diện; không khởi động hay dừng process nào.
EOF
}

HARDWARE=false
while (($#)); do
  case "$1" in
    --hardware) HARDWARE=true; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Tham số không hợp lệ: $1" >&2; usage; exit 2 ;;
  esac
done

set +u
source /home/ac/ros2_humble/install/setup.bash
source "${ROOT_DIR}/ros2_ws/install/setup.bash"
set -u

FAILURES=0
check() {
  local label="$1"
  shift
  if "$@" >/dev/null 2>&1; then
    printf 'PASS  %s\n' "${label}"
  else
    printf 'FAIL  %s\n' "${label}"
    FAILURES=$((FAILURES + 1))
  fi
}

check "ROS 2 CLI" command -v ros2
check "ros2 bag CLI" ros2 bag --help
check "acados native + Python" "${ROOT_DIR}/scripts/bootstrap_acados.sh" --verify-only
for package in \
  hardware_camera image_processing rc_interfaces rc_localization \
  race_cars rc_actuation rc_bringup rc_validation \
  sllidar_ros2 rf2o_laser_odometry bno055 robot_localization \
  micro_ros_agent; do
  check "package ${package}" ros2 pkg prefix "${package}"
done
check "vehicle config" test -f "${ROOT_DIR}/config/vehicle_1_7.yaml"
check "firmware source" "${ROOT_DIR}/scripts/verify_firmware_source.sh"

if [[ "${HARDWARE}" == true ]]; then
  check "IMX708 via libcamera" bash -c \
    'command -v rpicam-hello >/dev/null || command -v libcamera-hello >/dev/null'
  check "LiDAR stable serial" bash -c \
    'compgen -G "/dev/serial/by-id/*Silicon_Labs*CP2102*" >/dev/null'
  check "BNO055 UART" test -e /dev/ttyAMA0
  check "STM32 stable serial" bash -c \
    'compgen -G "/dev/serial/by-id/*STMicroelectronics*" >/dev/null'
fi

if ((FAILURES)); then
  echo "Preflight FAIL: ${FAILURES} mục."
  exit 2
fi
echo "Preflight PASS."

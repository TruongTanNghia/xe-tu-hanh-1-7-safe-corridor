#!/usr/bin/env bash
set -eo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WORKSPACE="${PROJECT_ROOT}/ros2_ws"
UNDERLAY="${ROS2_UNDERLAY:-/home/ac/ros2_humble/install/setup.bash}"

usage() {
  cat <<'EOF'
Usage:
  run_safe_corridor.sh stack [launch-args...]
      Chỉ chạy obstacle_detector + safe_corridor_planner + safety_monitor.
      Dùng khi sensors (camera/LiDAR/EKF) và NMPC đã chạy ở terminal khác.
      Ví dụ: run_safe_corridor.sh stack enable_disarm:=false

  run_safe_corridor.sh full [launch-args...]
      Sensors + lane + EKF + safe corridor + NMPC (shadow mặc định).
      Ví dụ: run_safe_corridor.sh full nmpc_shadow:=false corridor_mode:=required

Không script nào ở đây khởi động micro-ROS/actuation; dùng
run_micro_ros_actuation.sh ở terminal riêng và arm theo runbook.
EOF
}

MODE="${1:-}"
case "${MODE}" in
  -h|--help|"") usage; exit 0 ;;
  stack|full) shift ;;
  *) usage; exit 2 ;;
esac

if [[ ! -f "${UNDERLAY}" ]]; then
  echo "Không tìm thấy ROS 2 underlay: ${UNDERLAY}" >&2
  exit 2
fi
if [[ ! -f "${WORKSPACE}/install/setup.bash" ]]; then
  echo "Chưa có workspace install. Hãy chạy scripts/build.sh rc_safe_corridor race_cars rc_bringup." >&2
  exit 3
fi

# NMPC loads the generated acados libraries.
source "${PROJECT_ROOT}/scripts/acados_env.sh"
set +u
source "${UNDERLAY}"
source "${WORKSPACE}/install/setup.bash"
set -u

if [[ "${MODE}" == "stack" ]]; then
  exec ros2 launch rc_safe_corridor safe_corridor.launch.py "$@"
else
  exec ros2 launch rc_bringup safe_corridor_full.launch.py "$@"
fi

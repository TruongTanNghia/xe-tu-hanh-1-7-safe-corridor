#!/usr/bin/env bash
set -Eeuo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

usage() {
  cat <<'EOF'
Usage: run_esc_step_calibration.sh --wheels-up-confirmed

Publish typed step commands 1580,1585,1590,1595,1600 µs. Node starts at stop
and waits for `/calibration/start`. It does not start agent or arm actuation.
Never run concurrently with NMPC.
EOF
}

[[ "${1:-}" == "--wheels-up-confirmed" && $# -eq 1 ]] || {
  usage
  exit 2
}

set +u
source /home/ac/ros2_humble/install/setup.bash
source "${ROOT_DIR}/ros2_ws/install/setup.bash"
set -u

exec ros2 run rc_actuation esc_step_source --ros-args \
  -p safety_ack:=WHEELS_UP_MOTOR_POWER_CONTROLLED

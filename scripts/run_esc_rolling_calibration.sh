#!/usr/bin/env bash
set -Eeuo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if [[ "${1:-}" != "--rolling-slow-confirmed" || $# -ne 1 ]]; then
  cat <<'EOF'
Usage: run_esc_rolling_calibration.sh --rolling-slow-confirmed

Publish typed low-speed rolling calibration steps. Requires live
/odometry/filtered, enforces a 0.35 m/s speed limit, uses a 0.6 s odometry
freshness timeout, and starts READY_STOP.
It does not start the agent or arm actuation. Never run concurrently with NMPC.
EOF
  exit 2
fi

set +u
source /home/ac/ros2_humble/install/setup.bash
source "${ROOT_DIR}/ros2_ws/install/setup.bash"
set -u

exec ros2 run rc_actuation rolling_step_source --ros-args \
  -p safety_ack:=ROLLING_SLOW_MOTOR_POWER_CONTROLLED

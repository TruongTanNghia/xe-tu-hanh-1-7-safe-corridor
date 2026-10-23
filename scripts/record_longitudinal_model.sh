#!/usr/bin/env bash
set -Eeuo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

usage() {
  cat <<'EOF'
Usage: record_longitudinal_model.sh SESSION_DIR [DURATION_SEC]

Chỉ record `/mcu/actuator_state` + `/odometry/filtered` thành CSV. Script không
publish command và không arm xe.
EOF
}

[[ "${1:-}" != "-h" && "${1:-}" != "--help" ]] || { usage; exit 0; }
[[ $# -ge 1 && $# -le 2 ]] || { usage; exit 2; }
SESSION_DIR="$(realpath "$1")"
DURATION="${2:-60}"
[[ -d "${SESSION_DIR}/csv" ]] || {
  echo "Session không hợp lệ: ${SESSION_DIR}" >&2
  exit 2
}

set +u
source /home/ac/ros2_humble/install/setup.bash
source "${ROOT_DIR}/ros2_ws/install/setup.bash"
set -u

ros2 run rc_validation longitudinal_recorder --ros-args \
  -p "duration_sec:=${DURATION}" \
  -p "output_path:=${SESSION_DIR}/csv/longitudinal_samples.csv"

#!/usr/bin/env bash
set -Ee -o pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_UNDERLAY="${RC_ROS_UNDERLAY:-/home/ac/ros2_humble/install/setup.bash}"
MICROROS_OVERLAY="${RC_MICROROS_OVERLAY:-/home/ac/microros_ws/install/setup.bash}"
PROJECT_OVERLAY="${PROJECT_ROOT}/ros2_ws/install/setup.bash"

usage() {
  cat <<'EOF'
Usage: run_micro_ros_actuation.sh [ROS_LAUNCH_ARGUMENT ...]

Chạy micro-ROS agent + actuation manager ở trạng thái DISARMED. Agent ưu tiên
package đã build trong V2; overlay /home/ac/microros_ws chỉ là compatibility
fallback. Không tự arm và không flash firmware.
EOF
}

[[ "${1:-}" != "-h" && "${1:-}" != "--help" ]] || { usage; exit 0; }

for setup_file in "${ROS_UNDERLAY}" "${PROJECT_OVERLAY}"; do
  if [[ ! -f "${setup_file}" ]]; then
    echo "ERROR: Không tìm thấy ${setup_file}" >&2
    exit 1
  fi
done

set +u
# shellcheck disable=SC1090
source "${ROS_UNDERLAY}"
if [[ -f "${MICROROS_OVERLAY}" ]]; then
  # shellcheck disable=SC1090
  source "${MICROROS_OVERLAY}"
fi
# shellcheck disable=SC1090
source "${PROJECT_OVERLAY}"
set -u

ros2 pkg prefix micro_ros_agent >/dev/null 2>&1 || {
  echo "ERROR: micro_ros_agent chưa được build." >&2
  echo "Chạy: bash scripts/bootstrap.sh && bash scripts/build.sh rc_actuation" >&2
  exit 3
}

exec ros2 launch rc_actuation actuation_host.launch.py "$@"

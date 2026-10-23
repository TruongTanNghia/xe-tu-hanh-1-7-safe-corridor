#!/usr/bin/env bash
set -Ee -o pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WORKSPACE="${PROJECT_ROOT}/ros2_ws"
UNDERLAY="${RC_ROS_UNDERLAY:-/home/ac/ros2_humble/install/setup.bash}"
PROJECT_PACKAGES=(
  hardware_camera
  image_processing
  rc_interfaces
  rc_localization
  race_cars
  rc_actuation
  rc_bringup
  rc_validation
  rc_safe_corridor
)

usage() {
  cat <<'EOF'
Usage:
  test.sh                         # test package do V2 sở hữu
  test.sh PACKAGE [PACKAGE ...]  # test package chỉ định
  test.sh --include-vendor       # kể cả test/lint upstream

Vendor được build ở mọi lần build nhưng không nằm trong acceptance test mặc
định. `--include-vendor` có thể lộ lỗi lint của upstream không thuộc code V2.
EOF
}

if [[ ! -f "${UNDERLAY}" ]]; then
  echo "ERROR: Không tìm thấy ROS 2 underlay: ${UNDERLAY}" >&2
  exit 1
fi

# NMPC tests load generated shared libraries.
source "${PROJECT_ROOT}/scripts/acados_env.sh"

# shellcheck disable=SC1090
source "${UNDERLAY}"
if [[ -f "${WORKSPACE}/install/setup.bash" ]]; then
  # shellcheck disable=SC1090
  source "${WORKSPACE}/install/setup.bash"
fi
set -u

cd "${WORKSPACE}"
RESULT_BASE="${WORKSPACE}/test_results/current"
colcon test-result \
  --test-result-base "${RESULT_BASE}" \
  --delete-yes >/dev/null 2>&1 || true
COMMON_ARGS=(
  --test-result-base "${RESULT_BASE}"
  --event-handlers console_direct+
)

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  usage
  exit 0
elif [[ "${1:-}" == "--include-vendor" ]]; then
  [[ $# -eq 1 ]] || { usage; exit 2; }
  colcon test "${COMMON_ARGS[@]}"
elif (( $# > 0 )); then
  colcon test --packages-select "$@" "${COMMON_ARGS[@]}"
else
  colcon test --packages-select "${PROJECT_PACKAGES[@]}" \
    "${COMMON_ARGS[@]}"
fi
colcon test-result --test-result-base "${RESULT_BASE}" --verbose

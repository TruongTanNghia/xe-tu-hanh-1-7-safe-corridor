#!/usr/bin/env bash
set -Ee -o pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WORKSPACE="${PROJECT_ROOT}/ros2_ws"
UNDERLAY="${RC_ROS_UNDERLAY:-/home/ac/ros2_humble/install/setup.bash}"

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  echo "Usage: build.sh [PACKAGE ...]"
  echo "Không truyền package: build toàn workspace; có package: build packages-up-to."
  exit 0
fi

if [[ ! -f "${UNDERLAY}" ]]; then
  echo "ERROR: Không tìm thấy ROS 2 underlay: ${UNDERLAY}" >&2
  exit 1
fi

# ROS setup files are not guaranteed to be nounset-safe.
# shellcheck disable=SC1090
source "${UNDERLAY}"
set -u

cd "${WORKSPACE}"
if (( $# > 0 )); then
  colcon build --symlink-install --packages-up-to "$@" \
    --event-handlers console_direct+
else
  colcon build --symlink-install --event-handlers console_direct+
fi

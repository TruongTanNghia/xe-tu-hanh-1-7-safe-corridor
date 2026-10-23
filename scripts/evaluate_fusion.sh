#!/usr/bin/env bash
set -Ee -o pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  echo "Usage: evaluate_fusion.sh [SCENARIO] [DURATION_SEC] [EXPECTED_SPEED_MPS]"
  echo "Compatibility wrapper cho evaluator live; workflow mới dùng scripts/evidence/evaluate.sh."
  exit 0
elif (($# > 3)); then
  echo "Quá nhiều tham số." >&2
  exit 2
fi

source /home/ac/ros2_humble/install/setup.bash
source "${PROJECT_ROOT}/ros2_ws/install/setup.bash"
set -u

SCENARIO="${1:-stationary}"
DURATION="${2:-60}"
EXPECTED_SPEED="${3:-0.0}"
STAMP="$(date +%Y%m%d-%H%M%S)"
OUTPUT="${PROJECT_ROOT}/evidence/fusion/${STAMP}_${SCENARIO}"

mkdir -p "${OUTPUT}"
{
  git -C "${PROJECT_ROOT}" rev-parse HEAD
  git -C "${PROJECT_ROOT}" status --short
} > "${OUTPUT}/git.txt"
env | grep -E '^(ROS|RMW|CYCLONEDDS|RC_)' | sort > "${OUTPUT}/environment.txt"
ls -l /dev/serial/by-id /dev/ttyUSB* /dev/ttyAMA* \
  > "${OUTPUT}/devices.txt" 2>&1 || true

ros2 run rc_validation velocity_evaluator --ros-args \
  -p "duration_sec:=${DURATION}" \
  -p "expected_speed_mps:=${EXPECTED_SPEED}" \
  -p "output_dir:=${OUTPUT}"

(
  cd "${OUTPUT}"
  sha256sum velocity_samples.csv metrics.json report.md git.txt \
    environment.txt devices.txt > SHA256SUMS
)

echo "Evidence: ${OUTPUT}"

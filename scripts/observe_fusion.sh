#!/usr/bin/env bash
set -Ee -o pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  echo "Usage: observe_fusion.sh"
  echo "Đọc-only: kiểm topic fusion và lấy một mẫu vx filtered."
  exit 0
elif (($#)); then
  echo "Tham số không hợp lệ: $1" >&2
  exit 2
fi

source /home/ac/ros2_humble/install/setup.bash
source "${PROJECT_ROOT}/ros2_ws/install/setup.bash"
set -u

echo "=== TOPICS ==="
for topic in /scan /odometry/rf2o_raw /odometry/rf2o \
  /odometry/filtered /bno055/imu /diagnostics; do
  printf '%-28s ' "${topic}"
  ros2 topic type "${topic}" --no-daemon 2>/dev/null || echo "MISSING"
done

echo
echo "=== RATE COMMANDS (mỗi lệnh chạy ở terminal riêng nếu cần) ==="
echo "ros2 topic hz /scan"
echo "ros2 topic hz /odometry/rf2o_raw"
echo "ros2 topic hz /odometry/filtered"
echo "ros2 topic echo /odometry/filtered --field twist.twist.linear.x"
echo "ros2 topic echo /diagnostics"

echo
echo "=== ONE FILTERED SAMPLE ==="
timeout 6 ros2 topic echo /odometry/filtered --once --no-daemon \
  --field twist.twist.linear.x || {
    echo "Không nhận được /odometry/filtered trong 6 giây." >&2
    exit 2
  }

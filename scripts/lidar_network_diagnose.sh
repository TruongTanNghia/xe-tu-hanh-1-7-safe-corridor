#!/usr/bin/env bash
set -Eeuo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
UNDERLAY="${RC_ROS_UNDERLAY:-/home/ac/ros2_humble/install/setup.bash}"
OVERLAY="${PROJECT_ROOT}/ros2_ws/install/setup.bash"
SCAN_TOPIC="${RC_SCAN_TOPIC:-/scan}"

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  echo "Usage: lidar_network_diagnose.sh"
  echo "Đọc-only: in network/ROS env, graph, QoS và thử nhận một LaserScan."
  exit 0
elif (($#)); then
  echo "Tham số không hợp lệ: $1" >&2
  exit 2
fi

if [[ ! -f "${UNDERLAY}" ]]; then
  echo "ERROR: Không tìm thấy ROS 2 underlay: ${UNDERLAY}" >&2
  exit 1
fi

# shellcheck disable=SC1090
set +u
source "${UNDERLAY}"
if [[ -f "${OVERLAY}" ]]; then
  # shellcheck disable=SC1090
  source "${OVERLAY}"
fi
set -u

echo "=== NETWORK ==="
ip -brief address
ip route

echo
echo "=== ROS ENVIRONMENT ==="
printf 'ROS_DOMAIN_ID=%s\n' "${ROS_DOMAIN_ID:-0}"
printf 'ROS_LOCALHOST_ONLY=%s\n' "${ROS_LOCALHOST_ONLY:-<unset>}"
printf 'RMW_IMPLEMENTATION=%s\n' "${RMW_IMPLEMENTATION:-<default>}"
printf 'ROS_IP=%s\n' "${ROS_IP:-<unset>}"
printf 'CYCLONEDDS_URI=%s\n' "${CYCLONEDDS_URI:-<unset>}"
if [[ -n "${ROS_IP:-}" ]]; then
  echo "WARN: ROS_IP là biến ROS 1; không dùng địa chỉ tĩnh này cho ROS 2 DDS."
fi

echo
echo "=== LOCAL ROS GRAPH (NO DAEMON) ==="
ros2 node list --no-daemon || true
ros2 topic list --no-daemon -t || true

echo
echo "=== ${SCAN_TOPIC} ==="
if ros2 topic type "${SCAN_TOPIC}" --no-daemon 2>/dev/null | grep -qx \
  "sensor_msgs/msg/LaserScan"; then
  ros2 topic info "${SCAN_TOPIC}" --verbose --no-daemon
  echo
  echo "Đợi tối đa 6 giây để nhận một scan với QoS sensor_data..."
  if timeout 6 ros2 topic echo "${SCAN_TOPIC}" --once --no-daemon \
    --qos-profile sensor_data \
    --field header; then
    echo "PASS: Pi nhận được ${SCAN_TOPIC}."
  else
    echo "FAIL: Topic tồn tại nhưng không nhận được sample trong 6 giây." >&2
    exit 3
  fi
else
  echo "FAIL: Pi chưa thấy ${SCAN_TOPIC} kiểu sensor_msgs/msg/LaserScan." >&2
  echo "Kiểm tra nguồn LiDAR, serial device và sllidar launch trước DDS laptop." >&2
  exit 2
fi

echo
echo "=== MULTICAST TEST COMMANDS ==="
echo "Laptop: ros2 multicast receive"
echo "Pi    : ros2 multicast send"
echo "Sau đó đảo chiều. Nếu ping được nhưng multicast fail, dùng CycloneDDS peer"
echo "hoặc web viewer thay vì tiếp tục đổi ROS_DOMAIN_ID."

#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/common.sh"

usage() {
  cat <<'EOF'
Usage: observe.sh [--session PATH] [--duration SEC]

Lưu graph ROS, QoS, một message mẫu và tốc độ topic vào logs/.
Không dừng hoặc thay đổi node đang chạy.
EOF
}

SESSION=""
DURATION=5
while (($#)); do
  case "$1" in
    --session) SESSION="${2:?thiếu PATH}"; shift 2 ;;
    --duration) DURATION="${2:?thiếu SEC}"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Tham số không hợp lệ: $1" >&2; usage; exit 2 ;;
  esac
done

resolve_session "${SESSION}"
source_ros_environment
LOG_DIR="${SESSION_DIR}/logs"

ros2 node list > "${LOG_DIR}/nodes.txt" 2>&1 || true
ros2 topic list -t > "${LOG_DIR}/topics.txt" 2>&1 || true
ros2 service list -t > "${LOG_DIR}/services.txt" 2>&1 || true
ros2 doctor --report > "${LOG_DIR}/ros2_doctor.txt" 2>&1 || true

TOPICS=(
  /scan
  /bno055/imu
  /odometry/rf2o_raw
  /odometry/rf2o
  /odometry/filtered
  /lane/reference
  /control/nmpc
  /mcu/status
  /mcu/actuator_state
  /actuation/status
)

for topic in "${TOPICS[@]}"; do
  safe_name="${topic//\//_}"
  safe_name="${safe_name#_}"
  if topic_exists "${topic}"; then
    ros2 topic info -v "${topic}" \
      > "${LOG_DIR}/${safe_name}_info.txt" 2>&1 || true
    timeout --signal=INT "${DURATION}" \
      ros2 topic hz "${topic}" \
      > "${LOG_DIR}/${safe_name}_hz.txt" 2>&1 || true
    if [[ "${topic}" == "/scan" || "${topic}" == "/bno055/imu" ]]; then
      timeout --signal=INT 5 ros2 topic echo "${topic}" --once \
        --qos-reliability best_effort \
        > "${LOG_DIR}/${safe_name}_sample.yaml" 2>&1 || true
    else
      timeout --signal=INT 5 ros2 topic echo "${topic}" --once \
        > "${LOG_DIR}/${safe_name}_sample.yaml" 2>&1 || true
    fi
  else
    echo "MISSING at $(date --iso-8601=seconds)" \
      > "${LOG_DIR}/${safe_name}_MISSING.txt"
  fi
done

echo "Đã lưu quan sát vào ${LOG_DIR}"

#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/common.sh"

usage() {
  cat <<'EOF'
Usage: record.sh [--session PATH] [--duration SEC] [-- TOPIC ...]

Record rosbag trong thời gian hữu hạn. Nếu không truyền topic, dùng bộ topic
canonical. Script chỉ gửi SIGINT đến process group do chính nó tạo.
EOF
}

SESSION=""
DURATION=30
TOPICS=()
while (($#)); do
  case "$1" in
    --session) SESSION="${2:?thiếu PATH}"; shift 2 ;;
    --duration) DURATION="${2:?thiếu SEC}"; shift 2 ;;
    --) shift; TOPICS=("$@"); break ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Tham số không hợp lệ: $1" >&2; usage; exit 2 ;;
  esac
done

if ((${#TOPICS[@]} == 0)); then
  TOPICS=(
    /scan /bno055/imu
    /odometry/rf2o_raw /odometry/rf2o /odometry/filtered
    /lane/reference /control/nmpc /nmpc_cmd_shadow
    /mcu/status /mcu/actuator_state /actuation/status
  )
fi

resolve_session "${SESSION}"
source_ros_environment
if ! ros2 bag --help >/dev/null 2>&1; then
  echo "Thiếu extension 'ros2 bag' trong ROS 2 underlay." >&2
  echo "Chạy scripts/bootstrap_rosbag2.sh rồi thử lại." >&2
  exit 3
fi
if ! [[ "${DURATION}" =~ ^[1-9][0-9]*$ ]]; then
  echo "--duration phải là số giây nguyên dương." >&2
  exit 2
fi

STAMP="$(date +%H%M%S)"
BAG_DIR="${SESSION_DIR}/rosbag/${STAMP}"
LOG_FILE="${SESSION_DIR}/logs/rosbag_${STAMP}.log"
PID_FILE="${SESSION_DIR}/logs/rosbag_${STAMP}.pid"
RECORDER_PID=""

stop_recorder() {
  if [[ -n "${RECORDER_PID}" ]] && kill -0 "${RECORDER_PID}" 2>/dev/null; then
    kill -INT -- "-${RECORDER_PID}" 2>/dev/null || true
    wait "${RECORDER_PID}" 2>/dev/null || true
  fi
  rm -f "${PID_FILE}"
}
trap stop_recorder EXIT INT TERM

setsid ros2 bag record -o "${BAG_DIR}" "${TOPICS[@]}" \
  > "${LOG_FILE}" 2>&1 &
RECORDER_PID=$!
echo "${RECORDER_PID}" > "${PID_FILE}"
echo "Recording PID ${RECORDER_PID} trong ${DURATION}s..."

DEADLINE=$((SECONDS + DURATION))
while ((SECONDS < DEADLINE)); do
  if ! kill -0 "${RECORDER_PID}" 2>/dev/null; then
    set +e
    wait "${RECORDER_PID}"
    STATUS=$?
    set -e
    rm -f "${PID_FILE}"
    echo "ros2 bag record dừng sớm (exit ${STATUS}). Log cuối:" >&2
    tail -n 20 "${LOG_FILE}" >&2 || true
    exit 4
  fi
  sleep 1
done

stop_recorder
trap - EXIT INT TERM

ros2 bag info "${BAG_DIR}" > "${SESSION_DIR}/logs/rosbag_${STAMP}_info.txt"
ln -sfn "${STAMP}" "${SESSION_DIR}/rosbag/latest"
echo "Đã lưu rosbag: ${BAG_DIR}"

#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/common.sh"

usage() {
  cat <<'EOF'
Usage:
  evaluate.sh --mode lidar [--session PATH]
  evaluate.sh --mode fusion [--session PATH] [--duration SEC] [--expected-speed MPS]
  evaluate.sh --mode fusion --bag PATH [--session PATH] [--expected-speed MPS]
  evaluate.sh --mode imu-axis [--bag PATH] [--session PATH]
              [--expected-direction ccw|cw]

lidar  : lưu SVG + JSON top-down từ /scan.
fusion : lưu CSV, metrics JSON và report vận tốc raw/conditioned/EKF. Có
         --bag thì đọc đúng dữ liệu đã ghi; không có thì subscribe live.
imu-axis: kiểm tra dấu gyro.z từ bag. Nếu không có --bag, dùng
          <session>/rosbag/latest.
EOF
}

MODE=""
SESSION=""
DURATION=60
EXPECTED_SPEED=0.0
EXPECTED_DIRECTION="ccw"
BAG_PATH=""
while (($#)); do
  case "$1" in
    --mode) MODE="${2:?thiếu MODE}"; shift 2 ;;
    --session) SESSION="${2:?thiếu PATH}"; shift 2 ;;
    --duration) DURATION="${2:?thiếu SEC}"; shift 2 ;;
    --expected-speed) EXPECTED_SPEED="${2:?thiếu MPS}"; shift 2 ;;
    --expected-direction)
      EXPECTED_DIRECTION="${2:?thiếu ccw hoặc cw}"
      shift 2
      ;;
    --bag) BAG_PATH="${2:?thiếu PATH}"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Tham số không hợp lệ: $1" >&2; usage; exit 2 ;;
  esac
done
[[ "${MODE}" =~ ^(lidar|fusion|imu-axis)$ ]] || {
  usage
  exit 2
}
[[ "${EXPECTED_DIRECTION}" =~ ^(ccw|cw)$ ]] || {
  echo "--expected-direction chỉ nhận ccw hoặc cw." >&2
  exit 2
}

resolve_session "${SESSION}"
source_ros_environment

if [[ "${MODE}" == "lidar" ]]; then
  ros2 run rc_validation lidar_scan_snapshot --ros-args \
    -p "output_path:=${SESSION_DIR}/images/lidar_scan.svg" \
    -p "timeout_sec:=10.0"
  echo "PASS: ${SESSION_DIR}/images/lidar_scan.svg"
elif [[ "${MODE}" == "fusion" ]]; then
  OUTPUT_DIR="${SESSION_DIR}/csv/fusion"
  set +e
  if [[ -n "${BAG_PATH}" ]]; then
    BAG_PATH="$(realpath "${BAG_PATH}")"
    ros2 run rc_validation evaluate_velocity_bag \
      "${BAG_PATH}" \
      --output-dir "${OUTPUT_DIR}" \
      --expected-speed "${EXPECTED_SPEED}"
  else
    ros2 run rc_validation velocity_evaluator --ros-args \
      -p "duration_sec:=${DURATION}" \
      -p "expected_speed_mps:=${EXPECTED_SPEED}" \
      -p "output_dir:=${OUTPUT_DIR}"
  fi
  RESULT=$?
  set -e
  {
    echo
    echo "## Fusion evaluator"
    echo
    echo "- Expected: \`${EXPECTED_SPEED} m/s\`"
    echo "- Source: \`${BAG_PATH:-live}\`"
    echo "- Requested live duration: \`${DURATION} s\`"
    echo "- Exit code: \`${RESULT}\`"
    echo "- Report: \`csv/fusion/report.md\`"
  } >> "${SESSION_DIR}/report.md"
  exit "${RESULT}"
else
  if [[ -z "${BAG_PATH}" ]]; then
    BAG_PATH="${SESSION_DIR}/rosbag/latest"
  fi
  if [[ ! -e "${BAG_PATH}" ]]; then
    echo "Không tìm thấy rosbag: ${BAG_PATH}" >&2
    exit 3
  fi
  BAG_PATH="$(realpath "${BAG_PATH}")"
  OUTPUT_DIR="${SESSION_DIR}/csv/imu_axis"
  set +e
  ros2 run rc_validation check_imu_axis_bag \
    "${BAG_PATH}" \
    --expected "${EXPECTED_DIRECTION}" \
    --output-dir "${OUTPUT_DIR}"
  RESULT=$?
  set -e
  {
    echo
    echo "## IMU axis evaluator"
    echo
    echo "- Expected direction: \`${EXPECTED_DIRECTION}\`"
    echo "- Source: \`${BAG_PATH}\`"
    echo "- Exit code: \`${RESULT}\`"
    echo "- Report: \`csv/imu_axis/gyro_axis_report.md\`"
  } >> "${SESSION_DIR}/report.md"
  exit "${RESULT}"
fi

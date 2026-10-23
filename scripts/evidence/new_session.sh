#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/common.sh"

usage() {
  cat <<'EOF'
Usage: new_session.sh --scenario NAME

Tạo cây evidence mới và lưu metadata môi trường, Git, thiết bị, package.
EOF
}

SCENARIO=""
while (($#)); do
  case "$1" in
    --scenario) SCENARIO="${2:?thiếu NAME}"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Tham số không hợp lệ: $1" >&2; usage; exit 2 ;;
  esac
done
[[ -n "${SCENARIO}" ]] || { usage; exit 2; }
[[ "${SCENARIO}" =~ ^[a-zA-Z0-9_-]+$ ]] || {
  echo "Scenario chỉ dùng chữ, số, _ hoặc -" >&2
  exit 2
}

source_ros_environment
STAMP="$(date +%Y%m%d-%H%M%S)"
SESSION_DIR="${EVIDENCE_ROOT}/evidence/${STAMP}_${SCENARIO}"
mkdir -p \
  "${SESSION_DIR}"/{metadata,logs,rosbag,csv,images,plots}

{
  echo "timestamp=$(date --iso-8601=seconds)"
  echo "scenario=${SCENARIO}"
  echo "hostname=$(hostname)"
  echo "user=$(id -un)"
  echo "kernel=$(uname -srmo)"
  echo "ros_distro=${ROS_DISTRO:-unset}"
  echo "ros_domain_id=${ROS_DOMAIN_ID:-0}"
  echo "ros_localhost_only=${ROS_LOCALHOST_ONLY:-unset}"
  echo "rmw_implementation=${RMW_IMPLEMENTATION:-unset}"
  echo "cyclonedds_uri=${CYCLONEDDS_URI:-unset}"
} > "${SESSION_DIR}/metadata/environment.txt"

{
  git -C "${EVIDENCE_ROOT}" rev-parse HEAD
  git -C "${EVIDENCE_ROOT}" status --short
  git -C "${EVIDENCE_ROOT}" diff --stat
} > "${SESSION_DIR}/metadata/git.txt"

{
  ls -l /dev/serial/by-id 2>&1 || true
  ls -l /dev/ttyUSB* /dev/ttyACM* /dev/ttyAMA* 2>&1 || true
} > "${SESSION_DIR}/metadata/devices.txt"

ps -eo pid,ppid,lstart,cmd --sort=pid \
  > "${SESSION_DIR}/metadata/processes.txt"
ros2 pkg list | sort > "${SESSION_DIR}/metadata/ros_packages.txt"
cp "${EVIDENCE_ROOT}/config/vehicle_1_7.yaml" \
  "${SESSION_DIR}/metadata/vehicle_1_7.yaml"

cat > "${SESSION_DIR}/report.md" <<EOF
# Báo cáo ${SCENARIO}

- Bắt đầu: $(date --iso-8601=seconds)
- Session: \`${SESSION_DIR}\`
- Gate dự kiến: **CHƯA ĐÁNH GIÁ**

## Ghi chú người vận hành

Điền điều kiện mặt đường, trạng thái bánh/motor và hiện tượng quan sát tại đây.

## Kết quả

Chạy \`observe.sh\`, \`record.sh\`, \`evaluate.sh\`, sau đó
\`finish_session.sh\`.
EOF

ln -sfn "$(basename "${SESSION_DIR}")" "${EVIDENCE_ROOT}/evidence/latest"
echo "${SESSION_DIR}"

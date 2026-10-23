#!/usr/bin/env bash
set -Eeuo pipefail

ROS_ROOT="${ROS_ROOT:-/home/ac/ros2_humble}"

usage() {
  cat <<'EOF'
Usage: bootstrap_rosbag2.sh

Build đúng nhóm package rosbag2 còn thiếu trong ROS 2 Humble source underlay.
Script không dùng sudo, không sửa source và không build lại toàn bộ ROS 2.
Có thể đổi vị trí underlay bằng biến ROS_ROOT.
EOF
}

if (($#)); then
  case "$1" in
    -h|--help) usage; exit 0 ;;
    *) echo "Tham số không hợp lệ: $1" >&2; usage; exit 2 ;;
  esac
fi

test -f "${ROS_ROOT}/install/setup.bash" || {
  echo "Không tìm thấy underlay: ${ROS_ROOT}/install/setup.bash" >&2
  exit 3
}
test -d "${ROS_ROOT}/src/ros2/rosbag2" || {
  echo "Không tìm thấy source rosbag2 tại ${ROS_ROOT}/src/ros2/rosbag2" >&2
  exit 3
}

set +u
source "${ROS_ROOT}/install/setup.bash"
set -u
cd "${ROS_ROOT}"

colcon build --symlink-install --packages-select \
  rosbag2_storage \
  rosbag2_storage_default_plugins \
  rosbag2_storage_mcap \
  rosbag2_compression \
  rosbag2_compression_zstd \
  rosbag2_cpp \
  rosbag2_transport \
  rosbag2_py \
  ros2bag \
  --event-handlers console_cohesion+

set +u
source "${ROS_ROOT}/install/setup.bash"
set -u
ros2 bag --help >/dev/null
echo "ros2 bag đã sẵn sàng."

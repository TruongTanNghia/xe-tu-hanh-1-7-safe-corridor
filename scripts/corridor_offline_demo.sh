#!/usr/bin/env bash
# Chạy demo Safe Corridor offline (không cần ROS, không cần xe): sinh scan
# LiDAR tổng hợp cho các kịch bản S1-S5/S7, chạy clustering -> tracking ->
# Frenet -> corridor -> FSM và xuất SVG + summary.json.
#
#   bash scripts/corridor_offline_demo.sh                 # tất cả kịch bản
#   bash scripts/corridor_offline_demo.sh s4 /tmp/demo    # một kịch bản
set -eo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PACKAGE_DIR="${PROJECT_ROOT}/ros2_ws/src/rc_safe_corridor"
SCENARIO="${1:-all}"
OUTPUT="${2:-${PROJECT_ROOT}/evidence/corridor_offline_$(date +%Y%m%d-%H%M%S)}"

cd "${PACKAGE_DIR}"
python3 -m rc_safe_corridor.offline_demo --scenario "${SCENARIO}" --output "${OUTPUT}"
echo "Mở các file SVG trong: ${OUTPUT}"

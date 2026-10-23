#!/usr/bin/env bash
set -eo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WORKSPACE="${PROJECT_ROOT}/ros2_ws"
UNDERLAY="${ROS2_UNDERLAY:-/home/ac/ros2_humble/install/setup.bash}"

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  echo "Usage: capture_lane_evidence.sh [LABEL]"
  echo "Lưu một snapshot raw/debug/fitting và các topic lane vào evidence/lane_<time>_<label>."
  exit 0
elif (($# > 1)); then
  echo "Chỉ nhận tối đa một LABEL." >&2
  exit 2
fi

LABEL="${1:-snapshot}"
SAFE_LABEL="$(printf '%s' "${LABEL}" | tr -cs 'A-Za-z0-9_-' '_')"
TIMESTAMP="$(date +%Y%m%d_%H%M%S)"
OUTPUT_DIR="${PROJECT_ROOT}/evidence/lane_${TIMESTAMP}_${SAFE_LABEL}"

if [[ ! -f "${UNDERLAY}" || ! -f "${WORKSPACE}/install/setup.bash" ]]; then
  echo "Thiếu ROS 2 underlay hoặc V2 install. Hãy build project trước." >&2
  exit 2
fi

set +u
source "${UNDERLAY}"
source "${WORKSPACE}/install/setup.bash"
set -u

mkdir -p "${OUTPUT_DIR}"

capture_image() {
  local topic="$1"
  local filename="$2"
  timeout 15 ros2 run hardware_camera save_image --ros-args \
    -p "topic:=${topic}" \
    -p "output_path:=${OUTPUT_DIR}/${filename}"
}

capture_value() {
  local topic="$1"
  local filename="$2"
  timeout 10 ros2 topic echo "${topic}" --once > "${OUTPUT_DIR}/${filename}"
}

capture_image /camera/front_camera/image_raw raw.jpg
capture_image /line/debug_image debug.jpg
capture_image /line/fitting_image fitting.jpg

capture_value /camera/front_camera/camera_info camera_info.yaml
capture_value /line/status status.yaml
capture_value /line/confidence confidence.yaml
capture_value /line/cte cte.yaml
capture_value /line/heading_angle heading_angle.yaml
capture_value /line/kappa kappa.yaml
capture_value /line/s_metric s_metric.yaml
capture_value /line/kappa_s kappa_s.yaml

{
  echo "timestamp=${TIMESTAMP}"
  echo "label=${LABEL}"
  echo "project_branch=$(git -C "${PROJECT_ROOT}" branch --show-current)"
  echo "project_head=$(git -C "${PROJECT_ROOT}" rev-parse HEAD)"
  echo "image_processing_baseline=0b0723bb606d24336ed46dacc4a1d716c65886f5"
  echo "image_encoding=bgr8"
  echo "processor_roi_y=0.23..0.46"
  echo "processor_roi_x_fallback=0.25..0.75"
  echo "camera=imx708_wide"
  echo "sensor_mode=2304x1296"
  echo "output_mode=640x360@30"
  echo "vehicle_motion=STATIC"
  echo "actuation_started_by_v2=NO"
} > "${OUTPUT_DIR}/metadata.txt"

git -C "${PROJECT_ROOT}" status --short > "${OUTPUT_DIR}/git_status.txt"

echo
echo "Đã lưu evidence tại:"
echo "${OUTPUT_DIR}"
echo
echo "Các file chính:"
echo "  raw.jpg"
echo "  debug.jpg"
echo "  fitting.jpg"
echo "  status.yaml / confidence.yaml / cte.yaml / heading_angle.yaml"

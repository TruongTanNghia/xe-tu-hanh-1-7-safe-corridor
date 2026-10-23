#!/usr/bin/env bash
set -Eeuo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

usage() {
  cat <<'EOF'
Usage: fit_longitudinal_model.sh SESSION_DIR [MASS_KG]

Fit model từ csv/longitudinal_samples.csv, ghi candidate + held-out report vào
plots/longitudinal_model/. Không tự sửa YAML NMPC.
EOF
}

[[ "${1:-}" != "-h" && "${1:-}" != "--help" ]] || { usage; exit 0; }
[[ $# -ge 1 && $# -le 2 ]] || { usage; exit 2; }
SESSION_DIR="$(realpath "$1")"
MASS_KG="${2:-6.5}"
CSV_PATH="${SESSION_DIR}/csv/longitudinal_samples.csv"
[[ -f "${CSV_PATH}" ]] || {
  echo "Không tìm thấy ${CSV_PATH}" >&2
  exit 2
}

set +u
source /home/ac/ros2_humble/install/setup.bash
source "${ROOT_DIR}/ros2_ws/install/setup.bash"
set -u

ros2 run rc_validation fit_longitudinal_model \
  "${CSV_PATH}" \
  --output-dir "${SESSION_DIR}/plots/longitudinal_model" \
  --mass-kg "${MASS_KG}"

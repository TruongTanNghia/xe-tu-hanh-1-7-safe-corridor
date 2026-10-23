#!/usr/bin/env bash

EVIDENCE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

source_ros_environment() {
  # setup.bash chưa an toàn với nounset trên một số overlay.
  set +u
  source /home/ac/ros2_humble/install/setup.bash
  source "${EVIDENCE_ROOT}/ros2_ws/install/setup.bash"
  set -u
}

resolve_session() {
  local requested="${1:-}"
  if [[ -n "${requested}" ]]; then
    SESSION_DIR="$(realpath "${requested}")"
  elif [[ -L "${EVIDENCE_ROOT}/evidence/latest" ]]; then
    SESSION_DIR="$(realpath "${EVIDENCE_ROOT}/evidence/latest")"
  else
    echo "Không có session. Chạy new_session.sh trước." >&2
    return 2
  fi
  [[ -d "${SESSION_DIR}/metadata" ]] || {
    echo "Session không hợp lệ: ${SESSION_DIR}" >&2
    return 2
  }
  export SESSION_DIR
}

topic_exists() {
  ros2 topic list 2>/dev/null | grep -Fxq "$1"
}

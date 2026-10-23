#!/usr/bin/env bash
set -Eeuo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
VENDOR_ROOT="${PROJECT_ROOT}/ros2_ws/src/vendor"
MANIFEST="${PROJECT_ROOT}/dependencies.repos"

if (($#)); then
  case "$1" in
    -h|--help)
      echo "Usage: bootstrap.sh"
      echo "Import dependency đúng commit từ dependencies.repos vào ros2_ws/src/vendor."
      exit 0
      ;;
    *) echo "Tham số không hợp lệ: $1" >&2; exit 2 ;;
  esac
fi

mkdir -p "${VENDOR_ROOT}"
vcs import "${VENDOR_ROOT}" < "${MANIFEST}"
vcs status "${VENDOR_ROOT}"

#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/common.sh"

usage() {
  cat <<'EOF'
Usage: finish_session.sh [--session PATH] [--result PASS|FAIL|INCOMPLETE]

Đóng report, liệt kê artifact và tạo SHA256SUMS tái kiểm tra được.
EOF
}

SESSION=""
RESULT="INCOMPLETE"
while (($#)); do
  case "$1" in
    --session) SESSION="${2:?thiếu PATH}"; shift 2 ;;
    --result) RESULT="${2:?thiếu RESULT}"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Tham số không hợp lệ: $1" >&2; usage; exit 2 ;;
  esac
done
[[ "${RESULT}" =~ ^(PASS|FAIL|INCOMPLETE)$ ]] || {
  echo "Result phải là PASS, FAIL hoặc INCOMPLETE" >&2
  exit 2
}

resolve_session "${SESSION}"
{
  echo
  echo "## Đóng session"
  echo
  echo "- Kết quả do người vận hành xác nhận: **${RESULT}**"
  echo "- Kết thúc: $(date --iso-8601=seconds)"
  echo "- Số file: $(find "${SESSION_DIR}" -type f | wc -l)"
} >> "${SESSION_DIR}/report.md"

(
  cd "${SESSION_DIR}"
  find . -type f ! -name SHA256SUMS -print0 \
    | sort -z \
    | xargs -0 sha256sum > SHA256SUMS
  sha256sum --check SHA256SUMS
)

echo "Đã đóng session: ${SESSION_DIR}"

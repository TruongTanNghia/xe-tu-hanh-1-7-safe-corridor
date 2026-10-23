#!/usr/bin/env bash
set -Eeuo pipefail

ACADOS_COMMIT="8e1a6f856e063c423de583e03c691e3b2b7fc0a0"
SOURCE_DIR="${RC_ACADOS_SOURCE:-/home/ac/main/1_projects/acados}"
INSTALL_DIR="${RC_ACADOS_INSTALL:-/home/ac/acados}"

usage() {
  cat <<'EOF'
Usage: bootstrap_acados.sh [--verify-only]

Clone/build acados đúng commit đã kiểm chứng trên Raspberry Pi 5, cài thư viện
vào /home/ac/acados và cài editable acados_template cho user hiện tại.

Biến tùy chọn:
  RC_ACADOS_SOURCE   source checkout (mặc định /home/ac/main/1_projects/acados)
  RC_ACADOS_INSTALL  install prefix (mặc định /home/ac/acados)

Script từ chối tự đổi một checkout đang dirty hoặc sai commit.
EOF
}

VERIFY_ONLY=false
while (($#)); do
  case "$1" in
    --verify-only) VERIFY_ONLY=true; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Tham số không hợp lệ: $1" >&2; usage; exit 2 ;;
  esac
done

verify() {
  test -d "${SOURCE_DIR}/.git"
  test "$(git -C "${SOURCE_DIR}" rev-parse HEAD)" = "${ACADOS_COMMIT}"
  test -z "$(git -C "${SOURCE_DIR}" status --porcelain)"
  test -f "${INSTALL_DIR}/lib/libacados.so"
  test -f "${INSTALL_DIR}/lib/git_commit_hash"
  ACADOS_SOURCE_DIR="${INSTALL_DIR}" \
    LD_LIBRARY_PATH="${INSTALL_DIR}/lib:${LD_LIBRARY_PATH:-}" \
    python3 -c \
      'import acados_template, casadi; print("acados_template OK; casadi=" + casadi.__version__)'
}

if [[ "${VERIFY_ONLY}" == true ]]; then
  verify
  echo "acados ${ACADOS_COMMIT} đã sẵn sàng."
  exit 0
fi

for command in git cmake make python3; do
  command -v "${command}" >/dev/null || {
    echo "Thiếu command: ${command}" >&2
    exit 3
  }
done

if [[ -e "${SOURCE_DIR}" && ! -d "${SOURCE_DIR}/.git" ]]; then
  echo "${SOURCE_DIR} tồn tại nhưng không phải Git checkout; từ chối ghi đè." >&2
  exit 4
fi

if [[ ! -d "${SOURCE_DIR}/.git" ]]; then
  git clone https://github.com/acados/acados.git "${SOURCE_DIR}"
  git -C "${SOURCE_DIR}" checkout "${ACADOS_COMMIT}"
  git -C "${SOURCE_DIR}" submodule update --init --recursive
else
  CURRENT="$(git -C "${SOURCE_DIR}" rev-parse HEAD)"
  DIRTY="$(git -C "${SOURCE_DIR}" status --porcelain)"
  if [[ -n "${DIRTY}" || "${CURRENT}" != "${ACADOS_COMMIT}" ]]; then
    echo "Checkout acados không sạch hoặc sai commit; từ chối tự thay đổi:" >&2
    echo "  path=${SOURCE_DIR}" >&2
    echo "  expected=${ACADOS_COMMIT}" >&2
    echo "  current=${CURRENT}" >&2
    exit 4
  fi
  git -C "${SOURCE_DIR}" submodule update --init --recursive
fi

cmake \
  -S "${SOURCE_DIR}" \
  -B "${SOURCE_DIR}/build" \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="${INSTALL_DIR}" \
  -DACADOS_WITH_QPOASES=ON
cmake --build "${SOURCE_DIR}/build" --parallel "$(nproc)"
cmake --install "${SOURCE_DIR}/build"
cmake -E copy_if_different \
  "${SOURCE_DIR}/lib/git_commit_hash" \
  "${INSTALL_DIR}/lib/git_commit_hash"
python3 -m pip install --user --no-deps --editable \
  "${SOURCE_DIR}/interfaces/acados_template"

verify
echo "acados đã build và cài tại ${INSTALL_DIR}."

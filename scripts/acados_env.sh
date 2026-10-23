#!/usr/bin/env bash

# Source file này trước khi chạy NMPC:
#   source scripts/acados_env.sh

if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
  echo "ERROR: Hãy source script để biến môi trường có hiệu lực:" >&2
  echo "  source scripts/acados_env.sh" >&2
  exit 2
fi

export ACADOS_SOURCE_DIR="${RC_ACADOS_INSTALL:-/home/ac/acados}"
if [[ ! -f "${ACADOS_SOURCE_DIR}/lib/libacados.so" ]]; then
  echo "ERROR: Chưa có libacados tại ${ACADOS_SOURCE_DIR}/lib." >&2
  echo "Chạy bash scripts/bootstrap_acados.sh trước." >&2
  return 1
fi

case ":${LD_LIBRARY_PATH:-}:" in
  *":${ACADOS_SOURCE_DIR}/lib:"*) ;;
  *) export LD_LIBRARY_PATH="${ACADOS_SOURCE_DIR}/lib:${LD_LIBRARY_PATH:-}" ;;
esac


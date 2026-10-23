#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
FW_DIR="${ROOT_DIR}/firmware/stm32/nucleo_f411re_microros"
MAIN_C="${FW_DIR}/Src/main.c"

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  echo "Usage: verify_firmware_source.sh"
  echo "Kiểm invariant + C syntax; không thay thế ARM target link/flash."
  exit 0
elif (($#)); then
  echo "Tham số không hợp lệ: $1" >&2
  exit 2
fi

fail() {
  echo "FAIL: $*" >&2
  exit 1
}

[[ -f "${MAIN_C}" ]] || fail "không tìm thấy ${MAIN_C}"

if grep -R -nE '/home/[^/]+|ESC_NEUTRAL_US|ESC_BENCH_MAX_US' \
  "${FW_DIR}/.cproject" "${MAIN_C}"; then
  fail "còn đường dẫn tuyệt đối hoặc hằng số firmware cũ"
fi

for expected in \
  '#define SERVO_MIN_SAFE_US    1150' \
  '#define SERVO_MAX_SAFE_US    1850' \
  '#define ESC_STOP_US          1550' \
  '#define ESC_FORWARD_START_US 1580' \
  '#define ESC_MAX_US           1600' \
  '#define ACTUATOR_CMD_TIMEOUT_MS 250U' \
  '"/mcu/actuator_cmd"' \
  '"/mcu/arm"' \
  '"/mcu/status"' \
  '"/mcu/actuator_state"'; do
  grep -Fq "${expected}" "${MAIN_C}" \
    || fail "thiếu invariant: ${expected}"
done

COMPILER="${CC:-}"
if [[ -z "${COMPILER}" ]]; then
  if command -v arm-none-eabi-gcc >/dev/null 2>&1; then
    COMPILER="arm-none-eabi-gcc"
  else
    COMPILER="gcc"
  fi
fi

"${COMPILER}" -fsyntax-only -std=gnu11 \
  -Wno-int-to-pointer-cast \
  -Werror=return-type \
  -Werror=incompatible-pointer-types \
  -Werror=implicit-function-declaration \
  -DSTM32F411xE -DUSE_HAL_DRIVER \
  -I"${FW_DIR}/Inc" \
  -I"${FW_DIR}/Drivers/STM32F4xx_HAL_Driver/Inc" \
  -I"${FW_DIR}/Drivers/CMSIS/Device/ST/STM32F4xx/Include" \
  -I"${FW_DIR}/Drivers/CMSIS/Include" \
  -I"${FW_DIR}/micro_ros_stm32cubemx_utils/microros_static_library_ide/libmicroros/include" \
  "${MAIN_C}"

echo "PASS: firmware source invariants + C syntax (${COMPILER})"
echo "Lưu ý: đây không thay thế build/link bằng STM32CubeIDE."

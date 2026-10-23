#include <uxr/client/transport.h>

#include "main.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define UART_IT_BUFFER_SIZE 2048U

static uint8_t rx_buffer[UART_IT_BUFFER_SIZE];
static uint8_t rx_byte;

static volatile size_t rx_head = 0U;
static volatile size_t rx_tail = 0U;

static UART_HandleTypeDef * transport_uart = NULL;

static size_t ring_available(void)
{
  if (rx_tail >= rx_head) {
    return rx_tail - rx_head;
  }

  return UART_IT_BUFFER_SIZE - rx_head + rx_tail;
}

bool cubemx_transport_open(struct uxrCustomTransport * transport)
{
  if (transport == NULL || transport->args == NULL) {
    return false;
  }

  transport_uart = (UART_HandleTypeDef *) transport->args;
  rx_head = 0U;
  rx_tail = 0U;

  return HAL_UART_Receive_IT(transport_uart, &rx_byte, 1U) == HAL_OK;
}

bool cubemx_transport_close(struct uxrCustomTransport * transport)
{
  (void) transport;

  if (transport_uart != NULL) {
    HAL_UART_Abort_IT(transport_uart);
    transport_uart = NULL;
  }

  return true;
}

size_t cubemx_transport_write(
  struct uxrCustomTransport * transport,
  const uint8_t * buf,
  size_t len,
  uint8_t * err)
{
  (void) err;

  if (transport == NULL || transport->args == NULL || buf == NULL || len == 0U) {
    return 0U;
  }

  UART_HandleTypeDef * uart = (UART_HandleTypeDef *) transport->args;

  HAL_StatusTypeDef ret = HAL_UART_Transmit(uart, (uint8_t *) buf, (uint16_t) len, 100U);

  return (ret == HAL_OK) ? len : 0U;
}

size_t cubemx_transport_read(
  struct uxrCustomTransport * transport,
  uint8_t * buf,
  size_t len,
  int timeout,
  uint8_t * err)
{
  (void) transport;
  (void) err;

  if (buf == NULL || len == 0U) {
    return 0U;
  }

  size_t read_count = 0U;
  uint32_t start_ms = HAL_GetTick();
  uint32_t timeout_ms = (timeout > 0) ? (uint32_t) timeout : 0U;

  while (read_count < len) {
    if (ring_available() > 0U) {
      buf[read_count] = rx_buffer[rx_head];
      rx_head = (rx_head + 1U) % UART_IT_BUFFER_SIZE;
      read_count++;
      continue;
    }

    if (timeout_ms == 0U) {
      break;
    }

    if ((HAL_GetTick() - start_ms) >= timeout_ms) {
      break;
    }
  }

  return read_count;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef * huart)
{
  if (transport_uart != NULL && huart == transport_uart) {
    size_t next_tail = (rx_tail + 1U) % UART_IT_BUFFER_SIZE;

    if (next_tail != rx_head) {
      rx_buffer[rx_tail] = rx_byte;
      rx_tail = next_tail;
    }

    HAL_UART_Receive_IT(transport_uart, &rx_byte, 1U);
  }
}

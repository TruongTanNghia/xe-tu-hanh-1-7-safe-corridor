/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <rcutils/allocator.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/u_int32.h>
#include <std_msgs/msg/int16_multi_array.h>
#include <rosidl_runtime_c/message_type_support_struct.h>
#include <uxr/client/transport.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* Host and firmware must use the same physical safety envelope. */
#define SERVO_CENTER_US      1500
#define SERVO_MIN_SAFE_US    1150
#define SERVO_MAX_SAFE_US    1850

#define ESC_STOP_US          1550
#define ESC_FORWARD_START_US 1580
#define ESC_MAX_US           1600

#define ACTUATOR_CMD_TIMEOUT_MS 250U
#define ARM_TIMEOUT_MS          250U
#define MCU_STATUS_PERIOD_MS    100U
#define AGENT_PING_PERIOD_MS    500U
#define AGENT_PING_TIMEOUT_MS   100U
#define MAIN_LOOP_PERIOD_MS     10U
#define WAITING_AGENT_LED_PERIOD_MS 250U

#define STATE_FLAG_AGENT_CONNECTED 0x01
#define STATE_FLAG_ARMED           0x02
#define STATE_FLAG_COMMAND_FRESH   0x04
#define STATE_FLAG_ARM_FRESH       0x08
#define STATE_FLAG_WATCHDOG        0x10
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
static uint16_t current_servo_us = SERVO_CENTER_US;
static uint16_t current_esc_us = ESC_STOP_US;

static rcl_allocator_t microros_allocator;
static rclc_support_t microros_support;
static rcl_node_t microros_node;
static rcl_publisher_t status_publisher;
static rcl_publisher_t actuator_state_publisher;
static rcl_subscription_t actuator_subscriber;
static rcl_subscription_t arm_subscriber;
static rclc_executor_t microros_executor;
static std_msgs__msg__UInt32 status_msg;
static std_msgs__msg__Int16MultiArray actuator_cmd_msg;
static std_msgs__msg__Bool arm_msg;
static std_msgs__msg__Int16MultiArray actuator_state_msg;
static int16_t actuator_cmd_data[2] = {SERVO_CENTER_US, ESC_STOP_US};
static int16_t actuator_state_data[3] = {SERVO_CENTER_US, ESC_STOP_US, 0};
static uint32_t heartbeat_counter = 0U;
static bool microros_ready = false;
static bool support_initialized = false;
static bool node_initialized = false;
static bool status_publisher_initialized = false;
static bool actuator_state_publisher_initialized = false;
static bool actuator_subscriber_initialized = false;
static bool arm_subscriber_initialized = false;
static bool executor_initialized = false;

static uint32_t last_actuator_cmd_ms = 0U;
static uint32_t last_arm_msg_ms = 0U;
static uint32_t last_status_publish_ms = 0U;
static uint32_t last_agent_ping_ms = 0U;
static bool actuator_cmd_seen = false;
static bool arm_msg_seen = false;
static bool arm_requested = false;
static bool actuator_armed = false;

static uint16_t desired_servo_us = SERVO_CENTER_US;
static uint16_t desired_esc_us = ESC_STOP_US;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
extern bool cubemx_transport_open(struct uxrCustomTransport * transport);
extern bool cubemx_transport_close(struct uxrCustomTransport * transport);
extern size_t cubemx_transport_write(struct uxrCustomTransport * transport, const uint8_t * buf, size_t len, uint8_t * err);
extern size_t cubemx_transport_read(struct uxrCustomTransport * transport, uint8_t * buf, size_t len, int timeout, uint8_t * err);

extern void * microros_allocate(size_t size, void * state);
extern void microros_deallocate(void * pointer, void * state);
extern void * microros_reallocate(void * pointer, size_t size, void * state);
extern void * microros_zero_allocate(size_t number_of_elements, size_t size_of_element, void * state);
static bool microros_transport_init(void);
static bool microros_create_entities(void);
static void microros_destroy_entities(void);
static bool microros_publish_status_once(void);
static void microros_prepare_messages(void);
static void actuator_cmd_callback(const void * msgin);
static void arm_callback(const void * msgin);
static void apply_actuator_safety_update(void);
static void watchdog_init(void);
static void watchdog_refresh(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static uint16_t clamp_u16(uint16_t value, uint16_t min_value, uint16_t max_value)
{
  if (value < min_value)
  {
    return min_value;
  }

  if (value > max_value)
  {
    return max_value;
  }

  return value;
}

static void servo_write_us(uint16_t pulse_us)
{
  pulse_us = clamp_u16(pulse_us, SERVO_MIN_SAFE_US, SERVO_MAX_SAFE_US);
  current_servo_us = pulse_us;
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, pulse_us);
}

static void esc_write_us(uint16_t pulse_us)
{
  if (pulse_us <= ESC_STOP_US) {
    pulse_us = ESC_STOP_US;
  } else {
    pulse_us = clamp_u16(pulse_us, ESC_FORWARD_START_US, ESC_MAX_US);
  }
  current_esc_us = pulse_us;
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, pulse_us);
}

static void force_stop_outputs(void)
{
  servo_write_us(SERVO_CENTER_US);
  esc_write_us(ESC_STOP_US);
  actuator_armed = false;
}

static void watchdog_init(void)
{
  /*
   * Bring-up mode: keep watchdog disabled.
   *
   * The previous direct IWDG register setup can hide boot failures during
   * urgent micro-ROS/ST-LINK validation because the firmware may reset before
   * LD2 or the agent handshake becomes observable. Re-enable only after the
   * heartbeat/actuation link is proven stable on hardware.
   */
}

static void watchdog_refresh(void)
{
  /* Bring-up mode: watchdog disabled. */
}

static bool microros_transport_init(void)
{
  microros_allocator = rcutils_get_zero_initialized_allocator();
  microros_allocator.allocate = microros_allocate;
  microros_allocator.deallocate = microros_deallocate;
  microros_allocator.reallocate = microros_reallocate;
  microros_allocator.zero_allocate = microros_zero_allocate;

  if (!rcutils_set_default_allocator(&microros_allocator)) {
    return false;
  }

  rmw_uros_set_custom_transport(
    true,
    (void *) &huart2,
    cubemx_transport_open,
    cubemx_transport_close,
    cubemx_transport_write,
    cubemx_transport_read);
  return true;
}

static bool microros_create_entities(void)
{
  microros_support = (rclc_support_t) {0};
  microros_node = rcl_get_zero_initialized_node();
  status_publisher = rcl_get_zero_initialized_publisher();
  actuator_state_publisher = rcl_get_zero_initialized_publisher();
  actuator_subscriber = rcl_get_zero_initialized_subscription();
  arm_subscriber = rcl_get_zero_initialized_subscription();
  microros_executor = rclc_executor_get_zero_initialized_executor();

  if (rclc_support_init(&microros_support, 0, NULL, &microros_allocator) != RCL_RET_OK) {
    return false;
  }
  support_initialized = true;

  if (rclc_node_init_default(&microros_node, "nucleo_f411re", "", &microros_support) != RCL_RET_OK) {
    goto fail;
  }
  node_initialized = true;

  if (rclc_publisher_init_default(
      &status_publisher,
      &microros_node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt32),
      "/mcu/status") != RCL_RET_OK) {
    goto fail;
  }
  status_publisher_initialized = true;

  if (rclc_publisher_init_default(
      &actuator_state_publisher,
      &microros_node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16MultiArray),
      "/mcu/actuator_state") != RCL_RET_OK) {
    goto fail;
  }
  actuator_state_publisher_initialized = true;

  if (rclc_subscription_init_default(
      &actuator_subscriber,
      &microros_node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16MultiArray),
      "/mcu/actuator_cmd") != RCL_RET_OK) {
    goto fail;
  }
  actuator_subscriber_initialized = true;

  if (rclc_subscription_init_default(
      &arm_subscriber,
      &microros_node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
      "/mcu/arm") != RCL_RET_OK) {
    goto fail;
  }
  arm_subscriber_initialized = true;

  microros_prepare_messages();

  if (rclc_executor_init(&microros_executor, &microros_support.context, 2U, &microros_allocator) != RCL_RET_OK) {
    goto fail;
  }
  executor_initialized = true;

  if (rclc_executor_add_subscription(
      &microros_executor,
      &actuator_subscriber,
      &actuator_cmd_msg,
      &actuator_cmd_callback,
      ON_NEW_DATA) != RCL_RET_OK) {
    goto fail;
  }

  if (rclc_executor_add_subscription(
      &microros_executor,
      &arm_subscriber,
      &arm_msg,
      &arm_callback,
      ON_NEW_DATA) != RCL_RET_OK) {
    goto fail;
  }

  status_msg.data = 0U;
  heartbeat_counter = 0U;
  actuator_cmd_seen = false;
  arm_msg_seen = false;
  arm_requested = false;
  force_stop_outputs();
  microros_ready = true;
  return true;

fail:
  microros_destroy_entities();
  return false;
}

static void microros_destroy_entities(void)
{
  microros_ready = false;
  arm_requested = false;
  actuator_cmd_seen = false;
  arm_msg_seen = false;
  force_stop_outputs();

  if (support_initialized) {
    rmw_context_t * rmw_context =
      rcl_context_get_rmw_context(&microros_support.context);
    (void) rmw_uros_set_context_entity_destroy_session_timeout(
      rmw_context, 0);
  }
  if (executor_initialized) {
    (void) rclc_executor_fini(&microros_executor);
  }
  if (arm_subscriber_initialized) {
    (void) rcl_subscription_fini(&arm_subscriber, &microros_node);
  }
  if (actuator_subscriber_initialized) {
    (void) rcl_subscription_fini(&actuator_subscriber, &microros_node);
  }
  if (actuator_state_publisher_initialized) {
    (void) rcl_publisher_fini(&actuator_state_publisher, &microros_node);
  }
  if (status_publisher_initialized) {
    (void) rcl_publisher_fini(&status_publisher, &microros_node);
  }
  if (node_initialized) {
    (void) rcl_node_fini(&microros_node);
  }
  if (support_initialized) {
    (void) rclc_support_fini(&microros_support);
  }

  executor_initialized = false;
  arm_subscriber_initialized = false;
  actuator_subscriber_initialized = false;
  actuator_state_publisher_initialized = false;
  status_publisher_initialized = false;
  node_initialized = false;
  support_initialized = false;
}

static bool microros_publish_status_once(void)
{
  if (!microros_ready) {
    return false;
  }

  uint32_t now_ms = HAL_GetTick();
  bool command_fresh = actuator_cmd_seen
    && ((now_ms - last_actuator_cmd_ms) <= ACTUATOR_CMD_TIMEOUT_MS);
  bool arm_fresh = arm_msg_seen
    && ((now_ms - last_arm_msg_ms) <= ARM_TIMEOUT_MS);
  int16_t flags = STATE_FLAG_AGENT_CONNECTED | STATE_FLAG_WATCHDOG;
  if (actuator_armed) {
    flags |= STATE_FLAG_ARMED;
  }
  if (command_fresh) {
    flags |= STATE_FLAG_COMMAND_FRESH;
  }
  if (arm_fresh) {
    flags |= STATE_FLAG_ARM_FRESH;
  }

  status_msg.data = heartbeat_counter;
  heartbeat_counter++;
  actuator_state_data[0] = (int16_t) current_servo_us;
  actuator_state_data[1] = (int16_t) current_esc_us;
  actuator_state_data[2] = flags;

  if (rcl_publish(&status_publisher, &status_msg, NULL) != RCL_RET_OK) {
    return false;
  }
  return rcl_publish(
    &actuator_state_publisher, &actuator_state_msg, NULL) == RCL_RET_OK;
}

static void microros_prepare_messages(void)
{
  actuator_cmd_data[0] = SERVO_CENTER_US;
  actuator_cmd_data[1] = ESC_STOP_US;

  actuator_cmd_msg.layout.dim.data = NULL;
  actuator_cmd_msg.layout.dim.size = 0U;
  actuator_cmd_msg.layout.dim.capacity = 0U;
  actuator_cmd_msg.layout.data_offset = 0U;

  actuator_cmd_msg.data.data = actuator_cmd_data;
  actuator_cmd_msg.data.size = 0U;
  actuator_cmd_msg.data.capacity = 2U;

  actuator_state_msg.layout.dim.data = NULL;
  actuator_state_msg.layout.dim.size = 0U;
  actuator_state_msg.layout.dim.capacity = 0U;
  actuator_state_msg.layout.data_offset = 0U;
  actuator_state_msg.data.data = actuator_state_data;
  actuator_state_msg.data.size = 3U;
  actuator_state_msg.data.capacity = 3U;

  arm_msg.data = false;
}

static void actuator_cmd_callback(const void * msgin)
{
  const std_msgs__msg__Int16MultiArray * msg =
    (const std_msgs__msg__Int16MultiArray *) msgin;

  if (msg == NULL || msg->data.data == NULL || msg->data.size < 1U) {
    desired_servo_us = SERVO_CENTER_US;
    desired_esc_us = ESC_STOP_US;
    actuator_cmd_seen = false;
    return;
  }

  int32_t servo_cmd = (int32_t) msg->data.data[0];

  if (servo_cmd < SERVO_MIN_SAFE_US) {
    servo_cmd = SERVO_MIN_SAFE_US;
  }

  if (servo_cmd > SERVO_MAX_SAFE_US) {
    servo_cmd = SERVO_MAX_SAFE_US;
  }

  int32_t esc_cmd = ESC_STOP_US;

  if (msg->data.size >= 2U) {
    esc_cmd = (int32_t) msg->data.data[1];
  }

  if (esc_cmd <= ESC_STOP_US) {
    esc_cmd = ESC_STOP_US;
  } else {
    if (esc_cmd < ESC_FORWARD_START_US) {
      esc_cmd = ESC_FORWARD_START_US;
    }
    if (esc_cmd > ESC_MAX_US) {
      esc_cmd = ESC_MAX_US;
    }
  }

  desired_servo_us = (uint16_t) servo_cmd;
  desired_esc_us = (uint16_t) esc_cmd;
  last_actuator_cmd_ms = HAL_GetTick();
  actuator_cmd_seen = true;
}

static void arm_callback(const void * msgin)
{
  const std_msgs__msg__Bool * msg =
    (const std_msgs__msg__Bool *) msgin;
  if (msg == NULL) {
    arm_requested = false;
    arm_msg_seen = false;
    force_stop_outputs();
    return;
  }

  arm_requested = msg->data;
  arm_msg_seen = true;
  last_arm_msg_ms = HAL_GetTick();
  if (!arm_requested) {
    force_stop_outputs();
  }
}

static void apply_actuator_safety_update(void)
{
  uint32_t now_ms = HAL_GetTick();
  bool command_fresh = actuator_cmd_seen
    && ((now_ms - last_actuator_cmd_ms) <= ACTUATOR_CMD_TIMEOUT_MS);
  bool arm_fresh = arm_msg_seen
    && ((now_ms - last_arm_msg_ms) <= ARM_TIMEOUT_MS);

  if (!microros_ready || !arm_requested || !command_fresh || !arm_fresh) {
    force_stop_outputs();
    return;
  }

  actuator_armed = true;
  servo_write_us(desired_servo_us);
  esc_write_us(desired_esc_us);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM4_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

/* Boot is always stopped and disarmed. */
force_stop_outputs();
if (!microros_transport_init()) {
  Error_Handler();
}
watchdog_init();
HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
uint32_t last_waiting_agent_led_toggle_ms = HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    uint32_t now_ms = HAL_GetTick();

    if (!microros_ready) {
      force_stop_outputs();
      if ((now_ms - last_waiting_agent_led_toggle_ms) >= WAITING_AGENT_LED_PERIOD_MS) {
        last_waiting_agent_led_toggle_ms = now_ms;
        HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
      }
      if ((now_ms - last_agent_ping_ms) >= AGENT_PING_PERIOD_MS) {
        last_agent_ping_ms = now_ms;
        if (rmw_uros_ping_agent(AGENT_PING_TIMEOUT_MS, 1U) == RMW_RET_OK) {
          if (microros_create_entities()) {
            last_status_publish_ms = now_ms;
            HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
          }
        }
      }
    } else {
      rcl_ret_t spin_result = rclc_executor_spin_some(
        &microros_executor, RCL_MS_TO_NS(5));
      if (spin_result != RCL_RET_OK && spin_result != RCL_RET_TIMEOUT) {
        microros_destroy_entities();
        HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
      }

      apply_actuator_safety_update();
      now_ms = HAL_GetTick();

      if (microros_ready
          && (now_ms - last_status_publish_ms) >= MCU_STATUS_PERIOD_MS) {
        if (!microros_publish_status_once()) {
          microros_destroy_entities();
          HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
        }
        last_status_publish_ms = now_ms;
      }

      if (microros_ready
          && (now_ms - last_agent_ping_ms) >= AGENT_PING_PERIOD_MS) {
        last_agent_ping_ms = now_ms;
        if (rmw_uros_ping_agent(AGENT_PING_TIMEOUT_MS, 1U) != RMW_RET_OK) {
          microros_destroy_entities();
          HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
        }
      }
    }

    apply_actuator_safety_update();
    watchdog_refresh();
    HAL_Delay(MAIN_LOOP_PERIOD_MS);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 99;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 19999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 99;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 19999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

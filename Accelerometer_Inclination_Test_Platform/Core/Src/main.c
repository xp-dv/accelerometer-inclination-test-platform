/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

//* Private Includes
#include "main.h"

//* Standard Includes
#include "stm32f4xx_hal.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <ctype.h>
#include <string.h>
#include <math.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef uint16_t input_t; // Typedef set to fit numbers of size INPUT_T_MAX
typedef uint16_t ccr_t; // Typedef set to fit CCR register value >= 0 and <= 20000

typedef enum {
  STARTUP_STATE,
  HOME_STATE,
  IDLE_STATE,
  RUN_SETPOINT_STATE,
  RUN_PROFILE_STATE,
  RUN_TEST_STATE,
} system_state_t;

typedef enum {
  // Instruction Code 0 = RESERVED
  // Motion Commands
  MOVE_INSTRUCTION = 1,
  STOP_INSTRUCTION,
  CANCEL_INSTRUCTION,
  RUN_SETPOINT_INSTRUCTION,
  RUN_PROFILE_INSTRUCTION,
  // Setpoint Commands
  GET_SETPOINT_INSTRUCTION,
  ADD_SETPOINT_INSTRUCTION,
  REMOVE_SETPOINT_INSTRUCTION,
  // Profile Commands
  GET_PROFILE_INSTRUCTION,
  CLEAR_PROFILE_INSTRUCTION,
  // Test Commands
  TEST_SERVOS_INSTRUCTION = 995U,
  TEST_ADXL_INSTRUCTION,
  TEST_FLASH_INSTRUCTION,
  TEST_LED_INSTRUCTION,
  TEST_ECHO_INSTRUCTION,
} instruction_code_t;

typedef enum {
  //* General
  STATUS_OK,
  //* Thrown by UART Callback
  STATUS_ERR_UART_OF, // UART Buffer Overflow
  //* Thrown by instruction handler
  STATUS_ERR_NO_INDICATOR,
  STATUS_ERR_NO_TERMINATOR,
  STATUS_ERR_INVALID_INSTRUCTION,
  STATUS_ERR_INSTRUCTION_OUT_OF_RANGE,
  STATUS_ERR_TOO_MANY_INSTRUCTIONS,
  STATUS_ERR_INVALID_ARG,
  STATUS_ERR_INVALID_ARG_COUNT,
  STATUS_ERR_TOO_MANY_ARGS,
  STATUS_ERR_ARG_OUT_OF_RANGE,
  //* Thrown by user data functions
  STATUS_ERR_INVALID_SETPOINT, // The given setpoint or profile does not contain data
  STATUS_ERR_PROFILE_FULL,
  STATUS_ERR_EMPTY_PROFILE,
  STATUS_ERR_FLASH_WRITE_FAILED,
} status_code_t;

typedef struct {
  status_code_t status;
  input_t code;
  input_t args[INSTRUCTION_MAX_ARGS];
  input_t arg_count;
} instruction_t;

typedef struct setpoint {
  input_t x; // X position in degrees
  input_t y; // Y position in degrees
  input_t speed; // Multiplier for the PID controller’s Proportional constant
} setpoint_t;

// Struct for active setpoint for pid control
typedef struct active_setpoint {
  ccr_t x;
  ccr_t y;
  input_t speed;
  int error;
  input_t profile;
  input_t index;
} active_setpoint_t;

typedef struct previous_setpoint {
  ccr_t x;
  ccr_t y;
} previous_setpoint_t;

// Struct for PID controller
typedef struct {
  // Controller gain constants
  float Kp;
  float Ki;
  float Kd;

  // Derivative low-pass filter time constant
  float tau;

  // Output limits
  float limMin;
  float limMax;

  // Integrator limits
  float limMinInt;
  float limMaxInt;

  // Sample time (in seconds)
  float T;

  // Controller "memory"
  float integrator;
  float prevError;      // Required for integrator
  float differentiator;
  float prevMeasurement;    // Required for differentiator

  // Controller output
  float out;

} pidcontroller_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

//* Accelerometer Read
uint8_t adxl_rx_buf[ADXL_DATA_SIZE]; // The 6 bytes of ADXL data are stored here
float x_g, y_g, z_g;
float adxl_x_ang, adxl_y_ang, z_ang;

//* Servo Motor Control
float pos_x;
float pos_y;
float pos_x_last;
float pos_y_last;

//* UART
char uart_rx_char[2]; // Stores Rx char and string terminator
char uart_circ_buf[UART_RX_BUF_LEN]; // Circular Rx Buffer
uint8_t instruction_flag = 0;
status_code_t uart_it_status = STATUS_OK;

//pid control
int current_setpoint_x;
int current_setpoint_y;
float current_position_x;
float current_position_y;
float pid_x;
float pid_y;
float pulse_width_x;
float pulse_width_y;
pidcontroller_t pid = {
  .integrator = 0.0,
  .prevError  = 0.0,
  .differentiator  = 0.0,
  .prevMeasurement = 0.0,
  .out = 0.0
};

active_setpoint_t active_setpoint = {
  .index = 0,
};

previous_setpoint_t previous_setpoint = {
  .x = PULSE_WIDTH_0 + PULSE_WIDTH_OFFSET_X ,
  .y = PULSE_WIDTH_0 + PULSE_WIDTH_OFFSET_Y,
};

//* Startup State
system_state_t next_state_e = STARTUP_STATE;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

//* Internal Function Prototypes
int strtoint(char* str);
instruction_t parse_instruction(char* parse_buf);
void uart_echo(char* tx_buf, const char* rx_buf, const int status);
status_code_t clear_flash(void);
void adxl_tx(uint8_t address, uint8_t value);
void adxl_rx(uint8_t address);
void adxl_init(void);
void adxl_read(void);

//* Instruction Function Prototypes
status_code_t move(input_t x, input_t y, input_t speed);
status_code_t stop(void);
status_code_t cancel(void);
status_code_t run_setpoint(input_t index, input_t profile);
status_code_t run_profile(input_t profile);
status_code_t get_setpoint(input_t index, input_t profile);
status_code_t add_setpoint(input_t x_ang, input_t y_ang, input_t speed, input_t profile);
status_code_t remove_setpoint(input_t index, input_t profile);
status_code_t get_profile(input_t profile);
status_code_t clear_profile(input_t profile);
status_code_t test_servos(void);
status_code_t test_adxl(void);
status_code_t test_flash(void);

//* Event Handler Prototypes
void idle_state_handler(void);
system_state_t startup_state_handler(void);
system_state_t home_state_handler(void);
system_state_t run_setpoint_state_handler(void);
system_state_t run_profile_state_handler(void);
system_state_t run_test_state_handler(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//* Interrupt Callbacks
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  /**
   * TODO: True circular buffer
   * Should allow quick successive messages to fill buffer while waiting for CPU
   * Segments commands by their terminator '}'
   * Perhaps, uses two separate index pointers
   */
  static int i = 0;
  if (instruction_flag == 0) {
    uart_circ_buf[i] = uart_rx_char[0];
    if (uart_circ_buf[i] == '}') {
      instruction_flag = 1;
      i = 0;
    } else if (++i >= (sizeof(uart_circ_buf) - 1)) { // - 1 for NULL terminator
      i = 0;
      uart_it_status = STATUS_ERR_UART_OF; // Buffer Overflow
    }
  }

  idle_state_handler();

  // UART Interrupt Rx Reactivate
  HAL_UART_Receive_IT(huart, (uint8_t *)uart_rx_char, 1U); // Receive single char
}

//* Internal Functions
ccr_t degtoccr(input_t angle) {
  // angle -= 90;
  return (ccr_t)(PULSE_WIDTH_NEG_90 + (((int)angle - ANGLE_MIN) * (PULSE_WIDTH_POS_90 - PULSE_WIDTH_NEG_90) / ANGLE_RANGE));
}

int strtoint(char* str) {
  int num = 0;
  if (str != NULL) {
    while (*str != '\0') { // Terminates at NULL character
      if (*str >= '0' && *str <= '9') {
        num = num * 10U + (*str - '0'); // String (base 10) to int
      } else {
        return -1; // NaN
      }
      str++;
    }
  } else {
    return -1; // NULL
  }
  return num;
}

instruction_t parse_instruction(char* parse_buf) {
  instruction_t instruction = {
    .status = STATUS_OK,
    .code = RESERVED,
    .arg_count = 0U,
  };
  char* indicator_p;
  char* terminator_p;
  char* digit_token_p;
  int num;

  // Find start and end characters
  indicator_p = strchr(parse_buf, '{');
  if (!indicator_p) {
    instruction.status = STATUS_ERR_NO_INDICATOR;
    return instruction;
  }
  terminator_p = strchr(parse_buf, '}');
  if (!terminator_p) {
    instruction.status = STATUS_ERR_NO_TERMINATOR;
    return instruction;
  }
  ++indicator_p; // Increment past the start indicator '{'

  // Parse instruction code
  digit_token_p = strtok(indicator_p, "|}"); // Isolate digits by tokenizing the string between '{' and '}'
  num = strtoint(digit_token_p);
  if (num < 0) {
    instruction.status = STATUS_ERR_INVALID_INSTRUCTION;
    return instruction;
  }
  if (num > INPUT_T_MAX) {
    instruction.status = STATUS_ERR_INSTRUCTION_OUT_OF_RANGE;
    return instruction;
  }
  instruction.code = num;

  // Parse arguments
  while ((digit_token_p = strtok(NULL, " }")) != NULL) {
    if (*digit_token_p == '{') {
      instruction.status = STATUS_ERR_TOO_MANY_INSTRUCTIONS;
      return instruction;
    }
    num = strtoint(digit_token_p);
    if (num < 0) {
      instruction.status = STATUS_ERR_INVALID_ARG;
      return instruction;
    }
    if (num > INPUT_T_MAX) {
      instruction.status = STATUS_ERR_ARG_OUT_OF_RANGE;
      return instruction;
    }
    if (instruction.arg_count >= INSTRUCTION_MAX_ARGS) {
      instruction.status = STATUS_ERR_TOO_MANY_ARGS;
      return instruction;
    }
    instruction.args[instruction.arg_count++] = num;
  }

  return instruction;
}

void uart_echo(char* tx_buf, const char* rx_buf, const int status) {
  sprintf(tx_buf, "[%02u]%s\r\n", (status % 100), rx_buf); // Modulo truncates to 2 digits
  HAL_UART_Transmit(HUART_PTR, (uint8_t*)tx_buf, strlen(tx_buf), UART_TX_TIMEOUT);
}

status_code_t clear_flash(void) {
  // Unlock flash memory to enable writing
  HAL_FLASH_Unlock();

  // Erase flash memory by sector
  FLASH_EraseInitTypeDef flash_erase_setup = {
    .TypeErase = FLASH_TYPEERASE_SECTORS,
    .Sector = FLASH_SECTOR_NUMBER,
    .NbSectors = 1U,
    .VoltageRange = FLASH_VOLTAGE_RANGE_3,
  };
  uint32_t sector_error = 0;
  if (HAL_FLASHEx_Erase(&flash_erase_setup, &sector_error) != HAL_OK) {
    return STATUS_ERR_FLASH_WRITE_FAILED;
  }

  // Lock flash memory after writing
  HAL_FLASH_Lock();

  return STATUS_OK;
}

void adxl_tx(uint8_t address, uint8_t value) {
  uint8_t data[2];
  data[0] = address | BIT_6_MASK;  // Multibyte write enabled
  data[1] = value;
  HAL_GPIO_WritePin(ADXL_CS_GPIO_Port, ADXL_CS_Pin, GPIO_PIN_RESET); // Pull the cs pin low to enable the slave
  HAL_SPI_Transmit(&hspi3, data, sizeof(data), ADXL_TIMEOUT); // Transmit the address and data
  HAL_GPIO_WritePin(ADXL_CS_GPIO_Port, ADXL_CS_Pin, GPIO_PIN_SET); // Pull the cs pin high to disable the slave
}

void adxl_rx(uint8_t address) {
  address |= BIT_7_MASK;  // Read operation
  address |= BIT_6_MASK;  // Multibyte read
  HAL_GPIO_WritePin(ADXL_CS_GPIO_Port, ADXL_CS_Pin, GPIO_PIN_RESET);  // Pull the cs pin low to enable the slave
  HAL_SPI_Transmit(&hspi3, &address, ADXL_ADDRESS_SIZE, ADXL_TIMEOUT);  // Send single-byte address from where you want to read data
  HAL_SPI_Receive(&hspi3, adxl_rx_buf, ADXL_DATA_SIZE, ADXL_TIMEOUT);  // Read the 6 bytes of data
  HAL_GPIO_WritePin(ADXL_CS_GPIO_Port, ADXL_CS_Pin, GPIO_PIN_SET);  // Pull the cs pin high to disable the slave
}

void adxl_init(void) {
  HAL_Delay(STARTUP_DELAY);
  adxl_tx(ADXL_DATA_FORMAT, BIT_0_MASK);  // Data format range = +/- 4g
  adxl_tx(ADXL_POWER_CTL, BIT_RESET_MASK);  // Reset all bits
  adxl_tx(ADXL_POWER_CTL, BIT_3_MASK);  // Set power control mode to measure
  adxl_rx(ADXL_DEVID); // Stores the SPI Device ID in adxl_rx_buf
}

void adxl_read(void) {
  adxl_rx(ADXL_DATAX0); // Request data starting from the first data register
  // Convert the accelerometer values to 16-bit signed integers
  int16_t x_raw = (int16_t)((adxl_rx_buf[1] << 8) | adxl_rx_buf[0]);
  int16_t y_raw = (int16_t)((adxl_rx_buf[3] << 8) | adxl_rx_buf[2]);
  int16_t z_raw = (int16_t)((adxl_rx_buf[5] << 8) | adxl_rx_buf[4]);

  // Convert raw values to g's
  x_g = x_raw * ADXL_SCALE_FACTOR;
  y_g = y_raw * ADXL_SCALE_FACTOR;
  z_g = z_raw * ADXL_SCALE_FACTOR;

  adxl_x_ang = atan2f(-y_g, sqrtf(x_g * x_g + z_g * z_g));
  adxl_y_ang = atan2f(-x_g, sqrtf(y_g * y_g + z_g * z_g));
  // z_ang = atan2f(sqrtf(x_g * x_g + y_g * y_g), z_g);

  adxl_x_ang = adxl_x_ang * (180.0f / M_PI) + 90U;
  adxl_y_ang = adxl_y_ang * (180.0f / M_PI) + 90U;
  // z_ang = z_ang * (180.0f / M_PI);
}

//* Instruction Functions
/** 
 * @defgroup ImmediateFunctions Immediate Functions
 * Functions for immediate platform movement
 * @{
 * 
 */

/**
 * @brief Sets the system's active setpoint and transitions to the movement state.
 * 
 * @ingroup ImmediateFunctions
 * This function updates the system's active setpoint with the target X and Y coordinates
 * and the desired movement speed, after validating that the inputs are within the allowed
 * ranges. It also transitions the system to the state required for executing the movement.
 * 
 * @param x The target position for the X-axis in degrees. Must be within [ANGLE_INPUT_MIN, ANGLE_INPUT_MAX].
 * @param y The target position for the Y-axis in degrees. Must be within [ANGLE_INPUT_MIN, ANGLE_INPUT_MAX].
 * @param speed The speed of the movement. Must be within [SPEED_MIN, SPEED_MAX].
 * 
 * @return status_code_t 
 *         - STATUS_OK: If the setpoint is updated successfully and the system transitions to the movement state.
 *         - STATUS_ERR_ARG_OUT_OF_RANGE: If any of the input arguments are out of the allowed range.
 * 
 * @details
 * The X and Y input values are converted from degrees to corresponding pulse-width
 * modulation values using the `degtoccr()` function and are adjusted by specific offsets
 * (`PULSE_WIDTH_OFFSET_X` and `PULSE_WIDTH_OFFSET_Y`) to account for system calibration.
 * If any input parameter is out of range, the function returns an error and does not update the setpoint.
 * On success, the function transitions the system state to `RUN_SETPOINT_STATE`.
 */
status_code_t move(input_t x, input_t y, input_t speed) {
  // Check argument range
  if ((x < ANGLE_INPUT_MIN || x > ANGLE_INPUT_MAX) ||
      (y < ANGLE_INPUT_MIN || y > ANGLE_INPUT_MAX) ||
      (speed < SPEED_MIN || speed > SPEED_MAX)) {
      return STATUS_ERR_ARG_OUT_OF_RANGE; // Return error if any argument is out of range
  }

 
  active_setpoint.x = degtoccr(x) + PULSE_WIDTH_OFFSET_X;
  active_setpoint.y = degtoccr(y) + PULSE_WIDTH_OFFSET_Y;
  active_setpoint.speed = speed;
  
  // TODO: Take current position and set that as start point
  // CCR_X = previous_setpoint.x;
  // CCR_Y = previous_setpoint.y;
  next_state_e = RUN_SETPOINT_STATE;
  return STATUS_OK;
}
/**
 * @brief Stops all active platform operations and transitions to the idle state.
 * 
 * @ingroup ImmediateFunctions
 * This function halts PWM signals for the specified channels to immediately stop
 * power delivery to platforms or servos. It transitions the system to `IDLE_STATE`
 * to indicate that operations are no longer active. 
 * 
 * @return status_code_t 
 *         - STATUS_OK: If the stop operation completes successfully.
 * 
 * @note Designed to be used in case of emergency. Running this command will require for the device to be power-cycled for 
 *       continued operation. 
 */
status_code_t stop(void) {
  /**
   * TODO:
   * Implement stop interrupt that can immediately stop the pwm signal and cut power.
   * Maybe use a simple transistor and GPIO pin?
   */

  // Escape RUN_STATE
  next_state_e = IDLE_STATE;

  // Stop pwm signals to stop platforms/servos
  HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);

  return STATUS_OK;
}

/**
 * @brief Cancels the current operation and transitions the device to the home state.
 * 
 * @ingroup ImmediateFunctions
 * This function resets the active setpoint to its initial state and transitions the 
 * system to `HOME_STATE`, which just passes the idle state 
 * 
 * @return status_code_t 
 *         - STATUS_OK: If the cancel operation completes successfully.
 */
status_code_t cancel(void) {
  active_setpoint.index = 0;
  // Escape RUN_STATE
  next_state_e = HOME_STATE;
  return STATUS_OK;
}
/** @} */ // end of Immediate Functions group



/** 
 * @defgroup RunCommands Run Commands
 * Functions for immediate platform movement
 * @{
 * 
 */

/**
 * @brief Configures the system to run a specific setpoint with a given profile.
 * 
 * @ingroup RunCommands
 * This function retrieves the setpoint parameters (X, Y, and speed) from the specified 
 * index and profile, converts the X and Y values to PWM signals, 
 * and updates the active setpoint. It then transitions the system to the `RUN_SETPOINT_STATE`.
 * 
 * @param index The index of the desired setpoint in the setpoint array.
 * @param profile The profile to use for the specified setpoint.
 * 
 * @return status_code_t 
 *         - STATUS_OK: If the setpoint is successfully applied and the system transitions to `RUN_SETPOINT_STATE`.
 */
status_code_t run_setpoint(input_t index, input_t profile) {
 // Retrieve the setpoint from the specified index and profile.
  setpoint_t* setpoint = SETPOINT_ADDRESS(index, profile);

  active_setpoint.x = degtoccr(setpoint->x) + PULSE_WIDTH_OFFSET_X;
  active_setpoint.y = degtoccr(setpoint->y) + PULSE_WIDTH_OFFSET_Y;
  active_setpoint.speed = setpoint->speed;
  
  next_state_e = RUN_SETPOINT_STATE;
  return STATUS_OK;
}
/**
 * @brief Configures the system to execute a specific profile.
 * 
 * @ingroup RunCommands
 * This function sets the active profile and transitions the system
 * to the `RUN_PROFILE_STATE`, preparing it for executing the associated setpoints.
 * 
 * @param profile The profile to be executed.
 * 
 * @return status_code_t 
 *         - STATUS_OK: If the profile is successfully set, and the system transitions to `RUN_PROFILE_STATE`.
 */
status_code_t run_profile(input_t profile) {
  active_setpoint.profile = profile;
  next_state_e = RUN_PROFILE_STATE;
  return STATUS_OK;
}
/** @} */ // end of Run Commands group


/**
 * @brief Retrieves and transmits the setpoint data for a given index and profile.
 * 
 * This function checks if the provided index and profile are within valid ranges, retrieves
 * the corresponding setpoint data, and transmits the setpoint's X, Y, and speed values over UART.
 * If the requested setpoint is invalid or empty, the function returns an error status.
 * 
 * @param index The index of the desired setpoint.
 * @param profile The profile associated with the setpoint.
 * 
 * @return status_code_t 
 *         - STATUS_OK: If the setpoint is found, valid, and successfully transmitted.
 *         - STATUS_ERR_ARG_OUT_OF_RANGE: If the index or profile is out of range.
 *         - STATUS_ERR_INVALID_SETPOINT: If the requested setpoint contains invalid data (all fields empty).
 */
status_code_t get_setpoint(input_t index, input_t profile) {
  // Check argument range 
  if ((profile < PROFILE_ARG_MIN || profile > PROFILE_ARG_MAX) ||
    (index < INDEX_ARG_MIN || index > INDEX_ARG_MAX)) {
    return STATUS_ERR_ARG_OUT_OF_RANGE;
  }
  // Get setpoint pointer from index and profile arguments
  setpoint_t* setpoint = SETPOINT_ADDRESS(index, profile);

  // Check if requested setpoint contains data
  if (setpoint->x == FLASH_EMPTY && setpoint->y == FLASH_EMPTY && setpoint->speed == FLASH_EMPTY) {
    return STATUS_ERR_INVALID_SETPOINT;
  }

  // Transmit setpoint data
  char uart_tx_buf[UART_TX_BUF_LEN];
  sprintf(uart_tx_buf, "{%03u %03u %03u}\r\n", (setpoint->x % 1000), (setpoint->y % 1000), (setpoint->speed % 1000));
  HAL_UART_Transmit(HUART_PTR, (uint8_t*)uart_tx_buf, strlen(uart_tx_buf), UART_TX_TIMEOUT);

  return STATUS_OK;
}
/**
 * @brief Adds a new setpoint to the specified profile.
 * 
 * This function checks if the provided arguments are within valid ranges, searches for an empty 
 * slot in the profile to store the new setpoint, and writes the setpoint data (X, Y, and speed) 
 * to flash memory. If the profile is full or if there is an issue with writing to flash, the function 
 * returns an error.
 * 
 * @param x The X coordinate of the setpoint (in degrees).
 * @param y The Y coordinate of the setpoint (in degrees).
 * @param speed The speed for the setpoint (in valid range).
 * @param profile The profile to which the setpoint will be added.
 * 
 * @return status_code_t 
 *         - STATUS_OK: If the setpoint is successfully added to the profile.
 *         - STATUS_ERR_ARG_OUT_OF_RANGE: If any argument is out of the valid range.
 *         - STATUS_ERR_PROFILE_FULL: If the profile does not have space for more setpoints.
 *         - STATUS_ERR_FLASH_WRITE_FAILED: If there is an error while writing to flash memory.
 */
status_code_t add_setpoint(input_t x, input_t y, input_t speed, input_t profile) {
  // Check argument range
  if ((profile < PROFILE_ARG_MIN || profile > PROFILE_ARG_MAX) ||
      (x < ANGLE_INPUT_MIN || x > ANGLE_INPUT_MAX) ||
      (y < ANGLE_INPUT_MIN || y > ANGLE_INPUT_MAX) ||
      (speed < SPEED_MIN || speed > SPEED_MAX)) {
      return STATUS_ERR_ARG_OUT_OF_RANGE; // Return error if any argument is out of range
  }
  // Get address and index of last setpoint
  setpoint_t* setpoint = PROFILE_ADDRESS(profile);
  input_t index = 0;
  while(index <= INPUT_T_MAX) {
    if (setpoint->x == FLASH_EMPTY && setpoint->y == FLASH_EMPTY && setpoint->speed == FLASH_EMPTY) {
      break;
    }
    if (index == INPUT_T_MAX) {
      return STATUS_ERR_PROFILE_FULL;
    }
    ++setpoint;
    ++index;
  }

  // Unlock flash memory to enable writing
  HAL_FLASH_Unlock();

  // Write the array to flash memory
  if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, (uint32_t)(&(setpoint->x)), x) != HAL_OK ||
      HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, (uint32_t)(&(setpoint->y)), y) != HAL_OK ||
      HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, (uint32_t)(&(setpoint->speed)), speed) != HAL_OK) {
    HAL_FLASH_Lock();
    return STATUS_ERR_FLASH_WRITE_FAILED;
  }

  // Lock flash memory after writing
  HAL_FLASH_Lock();

  return STATUS_OK;
}
/**
 * @brief Removes a setpoint from the specified profile 
 * 
 * This function checks if the provided arguments are within valid ranges, ensures the specified 
 * setpoint exists, and then removes the setpoint by copying the remaining setpoints into RAM, 
 * clearing the old setpoints from flash, and writing the modified setpoints back into flash memory.
 * 
 * @param index The index of the setpoint to be removed.
 * @param profile The motion profile from which the setpoint will be removed.
 * 
 * @return status_code_t 
 *         - STATUS_OK: If the setpoint is successfully removed and flash memory is updated.
 *         - STATUS_ERR_ARG_OUT_OF_RANGE: If the provided index or profile is out of range.
 *         - STATUS_ERR_INVALID_SETPOINT: If the specified setpoint is empty or invalid.
 *         - STATUS_ERR_FLASH_WRITE_FAILED: If there is an error while writing to flash memory.
 */
status_code_t remove_setpoint(input_t index, input_t profile) {
  // Check argument range
  if ((profile < PROFILE_ARG_MIN || profile > PROFILE_ARG_MAX) ||
    (index < INDEX_ARG_MIN || index > INDEX_ARG_MAX)) {
    return STATUS_ERR_ARG_OUT_OF_RANGE;
    }
  // Check if requested setpoint contains data
  setpoint_t* given_setpoint = SETPOINT_ADDRESS(index, profile);
  if (given_setpoint->x == FLASH_EMPTY && given_setpoint->y == FLASH_EMPTY && given_setpoint->speed == FLASH_EMPTY){
    return STATUS_ERR_INVALID_SETPOINT;
  }

  // Get setpoint pointer from index and profile arguments
  setpoint_t* flash_setpoints = PROFILE_ADDRESS(profile);

  // Copy the kept setpoints to RAM
  setpoint_t kept_setpoints[PROFILE_LEN];
  input_t new_index = 0;

  // Copy all setpoints excluding the specified index
  for (input_t i = 0; i < PROFILE_LEN; i++) {
    if (i != index) {
      kept_setpoints[new_index++] = flash_setpoints[i];
    }
  }

  // Clear old setpoints from flash
  clear_profile(profile);

  // Unlock flash to begin writing
  HAL_FLASH_Unlock();

  // Write back all setpoints into flash except for the specified index
  for (input_t i = 0; i < new_index; i++) {
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, (uint32_t)(&(flash_setpoints[i].x)), kept_setpoints[i].x) != HAL_OK ||
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, (uint32_t)(&(flash_setpoints[i].y)), kept_setpoints[i].y) != HAL_OK ||
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, (uint32_t)(&(flash_setpoints[i].speed)), kept_setpoints[i].speed) != HAL_OK) {
      // Lock flash memory after writing
      HAL_FLASH_Lock();
      return STATUS_ERR_FLASH_WRITE_FAILED;
    }
  }

  // Lock flash memory after writing
  HAL_FLASH_Lock();

  return STATUS_OK;
}
/**
 * @brief Retrieves the setpoints from a specified profile and transmits them via UART.
 * 
 * This function checks if the provided profile is within the valid range, then retrieves and transmits 
 * the setpoints associated with the profile via UART. If the profile is empty or the setpoints 
 * cannot be found, an error code is returned.
 * 
 * @param profile The profile number to retrieve the setpoints from.
 * 
 * @return status_code_t 
 *         - STATUS_OK: If the profile and its setpoints are successfully retrieved and transmitted.
 *         - STATUS_ERR_ARG_OUT_OF_RANGE: If the provided profile is out of range.
 *         - STATUS_ERR_EMPTY_PROFILE: If the specified profile does not contain any data.
 */
status_code_t get_profile(input_t profile) {
  // Check argument range
  if (profile < PROFILE_ARG_MIN || profile > PROFILE_ARG_MAX) {
    return STATUS_ERR_ARG_OUT_OF_RANGE; // Return an error code for invalid profile
  }

  // Get setpoint pointer from index and profile arguments
  setpoint_t* setpoint = PROFILE_ADDRESS(profile);

  // Check if requested profile contains data
  if (PROFILE_ADDRESS(profile)->x == FLASH_EMPTY && PROFILE_ADDRESS(profile)->y == FLASH_EMPTY && PROFILE_ADDRESS(profile)->speed == FLASH_EMPTY){
    return STATUS_ERR_EMPTY_PROFILE;
  }
  
  input_t index = 0;
  char uart_tx_buf[UART_TX_BUF_LEN];
  // Transmit start indicator and first setpoint index
  sprintf(uart_tx_buf, "{000");
  HAL_UART_Transmit(HUART_PTR, (uint8_t*)uart_tx_buf, strlen(uart_tx_buf), UART_TX_TIMEOUT);
  while(index <= INPUT_T_MAX) {
    // Stop when next setpoint is empty or when limit is reached
    if (setpoint->x == FLASH_EMPTY && setpoint->y == FLASH_EMPTY && setpoint->speed == FLASH_EMPTY) {
      break;
    }
    if (index == INPUT_T_MAX) {
      break;
    }

    // Create new packet every 10 setpoints
    if (index > 0 && (index % 10U) == 0) {
      sprintf(uart_tx_buf, "}\r\n{%03u", (index % 1000U));
      HAL_UART_Transmit(HUART_PTR, (uint8_t*)uart_tx_buf, strlen(uart_tx_buf), UART_TX_TIMEOUT);
    }

    // Transmit setpoint data
    sprintf(uart_tx_buf, "|%03u %03u %03u", (setpoint->x % 1000U), (setpoint->y % 1000U), (setpoint->speed % 1000U));
    HAL_UART_Transmit(HUART_PTR, (uint8_t*)uart_tx_buf, strlen(uart_tx_buf), UART_TX_TIMEOUT);

    // Increment setpoint and index
    ++setpoint;
    ++index;
  }
  // Transmit terminator
  sprintf(uart_tx_buf, "}\r\n");
  HAL_UART_Transmit(HUART_PTR, (uint8_t*)uart_tx_buf, strlen(uart_tx_buf), UART_TX_TIMEOUT);

  return STATUS_OK;
}


/**
 * @brief Clears the setpoints from a specific profile while preserving other profiles in flash memory.
 * 
 * This function checks if the provided profile is valid and contains data. It then clears all setpoints 
 * from the specified profile while retaining the other profiles. The function performs a sector-by-sector 
 * flash erase and writes the remaining profiles back into flash memory. If the flash write fails at any point, 
 * an error is returned.
 * 
 * @param profile The profile number to clear.
 * 
 * @return status_code_t 
 *         - STATUS_OK: If the profile is successfully cleared and remaining profiles are preserved.
 *         - STATUS_ERR_ARG_OUT_OF_RANGE: If the provided profile is out of range.
 *         - STATUS_ERR_EMPTY_PROFILE: If the specified profile is empty and cannot be cleared.
 *         - STATUS_ERR_FLASH_WRITE_FAILED: If there was a failure during the flash write operation.
 */

status_code_t clear_profile(input_t profile) {
  // Check argument range
  if (profile < PROFILE_ARG_MIN || profile > PROFILE_ARG_MAX) {
    return  STATUS_ERR_ARG_OUT_OF_RANGE; // Return an error code for invalid profile
  }
  // Get setpoint pointer from profile argument
  setpoint_t* flash_setpoints = PROFILE_ADDRESS(0U);

  // Check if requested profile contains data
  if (PROFILE_ADDRESS(profile)->x == FLASH_EMPTY && PROFILE_ADDRESS(profile)->y == FLASH_EMPTY && PROFILE_ADDRESS(profile)->speed == FLASH_EMPTY){
    return STATUS_ERR_EMPTY_PROFILE;
  }

  // Copy the setpoints from all the kept profiles to RAM
  setpoint_t kept_setpoints[TOTAL_SETPOINTS];
  memset(kept_setpoints, FLASH_EMPTY, sizeof(kept_setpoints));
  for (input_t i = 0; i < TOTAL_SETPOINTS; i += PROFILE_LEN) {
    // Copy all setpoints excluding the specified index
    if (i >= (PROFILE_LEN * profile) && i < (PROFILE_LEN * (profile + 1))) {
      continue;
    }
    memcpy(&kept_setpoints[i], &flash_setpoints[i], (PROFILE_LEN * sizeof(setpoint_t)));
  }

  // Clear all profiles from flash. Note:
  // STM32 flash can only be erased sector-by-sector which is why the specified profile can't be individually cleared.
  if (clear_flash() != STATUS_OK) {
    return STATUS_ERR_FLASH_WRITE_FAILED;
  }

  // Unlock flash to begin writing
  HAL_FLASH_Unlock();

  // Write back all profiles into flash except for the specified profile
  for (uint16_t i = 0; i < TOTAL_SETPOINTS; ++i) {
    if (kept_setpoints[i].x != FLASH_EMPTY && kept_setpoints[i].y != FLASH_EMPTY && kept_setpoints[i].speed != FLASH_EMPTY) {
      if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, (uint32_t)(&(flash_setpoints[i].x)), kept_setpoints[i].x) != HAL_OK ||
          HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, (uint32_t)(&(flash_setpoints[i].y)), kept_setpoints[i].y) != HAL_OK ||
          HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, (uint32_t)(&(flash_setpoints[i].speed)), kept_setpoints[i].speed) != HAL_OK) {
        // Lock flash memory after writing
        HAL_FLASH_Lock();
        return STATUS_ERR_FLASH_WRITE_FAILED;
      }
    }
  }

  // Lock flash memory after writing
  HAL_FLASH_Lock();

  return STATUS_OK;
}

/**
 * @defgroup TestFunctions Test Functions
 * Functions for testing the system components such as servos, sensors, etc.
 * @{
 */

/**
 * @brief Test the servos by setting a test profile.
 * 
 * This function initializes a test profile for the servos and sets the state 
 * to `RUN_TEST_STATE`. It prepares the system for running a test on the servos.
 * 
 * @return STATUS_OK on successful initiation of the test.
 * @ingroup TestFunctions
 */
status_code_t test_servos(void) {
  // TODO: Change test printout to {X Y} format
  active_setpoint.profile = TEST_PROFILE;
  active_setpoint.index = 0;
  next_state_e = RUN_TEST_STATE;
  return STATUS_OK;
}



/**
 * @brief Tests the ADXL sensor by reading its angle data and transmitting it via UART.
 * 
 * This function calls the `adxl_read` function to retrieve angle data from the ADXL sensor, formats the 
 * data into a string, and transmits it via UART. The transmitted string includes the X and Y angle values 
 * retrieved from the sensor.
 * 
 * @return status_code_t 
 *         - STATUS_OK: If the test is successful and the data is transmitted.
 */
status_code_t test_adxl(void) {
  adxl_read();
  uint8_t test_msg[UART_TX_BUF_LEN];
  sprintf((char*)test_msg, "{%03f %03f}\n", adxl_x_ang, adxl_y_ang);
  HAL_UART_Transmit(HUART_PTR, test_msg, strlen((char*)test_msg), UART_TX_TIMEOUT);

  return STATUS_OK;
}

/**
 * @brief Tests the flash memory functionality by clearing, adding setpoints, and retrieving profiles.
 * 
 * @defgroup TestFunctions Test Functions
 * This function iterates through all available profiles, performing the following steps for each profile:
 * 1. Clears the profile using `clear_profile()`.
 * 2. Adds 300 setpoints using `add_setpoint()` with a fixed argument based on the profile index.
 * 3. Retrieves the profile using `get_profile()`.
 * If any operation fails, the function returns the error status code.
 * 
 * @return status_code_t 
 *         - STATUS_OK: If all operations for all profiles succeed.
 *         - An error code: If any operation (clear, add, or get) fails.
 */
status_code_t test_flash(void) {
  status_code_t status;
  for (input_t profile = 0; profile <= PROFILE_ARG_MAX; profile++) {
    status = clear_profile(profile);
    if (status != STATUS_OK) {
      return status;
    }

    uint16_t arg1 = profile ; // Set the first argument as the profile index + 1
    uint16_t arg2 = profile ; // Set the second argument as the profile index + 1
   
    // Populate setpoints for each profile
    for (uint16_t data = 0; data < 300U; data += 1) {
      status = add_setpoint(arg1, arg2, 1, profile);  // Add the specific setpoint for this profile
      if (status != STATUS_OK) {
        return status;  // Stop if an error occurs while adding setpoints
      }
    }

    status = get_profile(profile);
    if (status != STATUS_OK) {
      return status;
    }
  }

  return STATUS_OK;
}
/** @} */ // end of TestFunctions group


//* Event Handlers
/**
 * 
 * /**
 * @defgroup StateHandler State Handlers
 * Functions for testing the system components such as servos, sensors, etc.
 * @{
 */

 /**
 * @brief Handles the startup state of the system, initializing hardware peripherals and configuring settings.
 * 
 * @ingroup StateHanlder
 * This function performs the necessary steps to initialize the system's peripherals and set the initial state. The steps include:
 * - Configuring PWM for motors (with specified frequency and pulse width).
 * - Starting the PWM signals for motor control.
 * - Setting initial positions for the motors by calling the `move` function.
 * - Starting the timer interrupt for servo control.
 * - Initializing the accelerometer for motion sensing.
 * - Configuring the UART to receive data via interrupts.
 * - Transmitting a startup message code over UART to indicate the device is ready.
 * 
 * @return system_state_t
 *         - IDLE_STATE: The system transitions to the idle state after the startup operations are completed.
 */

system_state_t startup_state_handler(void) {
  //* PWM
  // Internal Clock (HCLK) = 100 MHz. If Prescaler = (100 - 1) & Max Timer Count = (20000 - 1),
  // then f = 100 MHz / 100 = 1 MHz, T = 1 us, and PWM f = 1/(20000 * T) = 50 Hz
  CCR_X = PULSE_WIDTH_0 + PULSE_WIDTH_OFFSET_X;
  CCR_Y = PULSE_WIDTH_0 + PULSE_WIDTH_OFFSET_Y;

  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  move(90, 90, 1);


  //* Servo Interrupt Start
  HAL_TIM_Base_Start_IT(&htim4);

  //* Accelerometer
  adxl_init();

  //* UART
  // Rx Interrupt Setup
  HAL_UART_Receive_IT(HUART_PTR, (uint8_t *)uart_rx_char, 1U); // Receive single char
  
  // Startup Message
  uint8_t startup_msg[] = "[00]{0}\n"; // STATUS_OK / Device is ready
  HAL_UART_Transmit(HUART_PTR, startup_msg, strlen((char*)startup_msg), UART_TX_TIMEOUT);

  return IDLE_STATE;
}

system_state_t home_state_handler(void) {
  return IDLE_STATE;
}


/**
 * @brief Handles the idle state of the system.
 * 
 * @ingroup StateHanlder
 * In the idle state, the system waits for a UART instruction to be received. Once the instruction is received
 * and parsed, it is executed based on the command. The handler supports a wide range of instructions,
 * including movement, stop, cancel, and testing commands. It also handles argument validation and echoes
 * the status of the executed instruction back via UART.
 *
 * The instructions handled by this handler include:
 * - MOVE_INSTRUCTION: Moves the system with specified arguments.
 * - STOP_INSTRUCTION: Stops the system's current operation (system reboot needed to resume operations)
 * - CANCEL_INSTRUCTION: Cancels the current operation.
 * - RUN_SETPOINT_INSTRUCTION: Runs a specified setpoint.
 * - RUN_PROFILE_INSTRUCTION: Runs a profile of setpoints.
 * - GET_SETPOINT_INSTRUCTION: Retrieves a setpoint.
 * - ADD_SETPOINT_INSTRUCTION: Adds a new setpoint.
 * - REMOVE_SETPOINT_INSTRUCTION: Removes an existing setpoint.
 * - GET_PROFILE_INSTRUCTION: Retrieves a profile.
 * - CLEAR_PROFILE_INSTRUCTION: Clears a profile from memory.
 * - TEST_SERVOS_INSTRUCTION: Tests the servo motors.
 * - TEST_ADXL_INSTRUCTION: Tests the accelerometer (ADXL).
 * - TEST_FLASH_INSTRUCTION: Tests the flash memory.
 * - TEST_LED_INSTRUCTION: Toggles an LED for testing.
 * - TEST_ECHO_INSTRUCTION: Echoes the received instruction back.
 *
 * Each instruction is validated for the correct number of arguments, and the corresponding action is performed.
 * If an invalid instruction or incorrect argument count is encountered, an error status is echoed back.
 *
 * @return None
 */
void idle_state_handler(void) {
  static char uart_tx_buf[UART_TX_BUF_LEN];
  static char uart_rx_buf[UART_RX_BUF_LEN];

  //* Wait for UART instruction_flag to trigger via interrupt
  if (instruction_flag) {
    strcpy(uart_rx_buf, uart_circ_buf); // TODO: True circular buffer

    if (uart_it_status != STATUS_OK) {

      // Echo received instruction with error status
      uart_echo(uart_tx_buf, uart_rx_buf, uart_it_status);
      uart_it_status = STATUS_OK;

    } else if (uart_it_status == STATUS_OK) {

      //* Parse received instruction
      // Copy UART buffer to prevent parse_instruction() from manipulating the original (due to strtok())
      char* parse_buf = calloc((strlen(uart_rx_buf) + 1U), sizeof(char)); // Allocate memory based on size of text in buffer
      strcpy(parse_buf, uart_rx_buf);

      instruction_t instruction = parse_instruction(parse_buf);
      free(parse_buf); // Free memory allocated by calloc()

      //* Echo received instruction with status if error occurred
      if (instruction.status != STATUS_OK) {
        uart_echo(uart_tx_buf, uart_rx_buf, instruction.status);
      } else {
        //* Instruction switch
        switch (instruction.code) {
        case MOVE_INSTRUCTION:
          if (instruction.arg_count != 3U) {
            instruction.status = STATUS_ERR_INVALID_ARG_COUNT;
            uart_echo(uart_tx_buf, uart_rx_buf, instruction.status);
            break;
          }
          instruction.status = move(instruction.args[0], instruction.args[1], instruction.args[2]);
          uart_echo(uart_tx_buf, uart_rx_buf, instruction.status);
          break;
        case STOP_INSTRUCTION:
          if (instruction.arg_count != 0U) {
            instruction.status = STATUS_ERR_INVALID_ARG_COUNT;
            uart_echo(uart_tx_buf, uart_rx_buf, instruction.status);
            break;
          }
          instruction.status = stop();
          uart_echo(uart_tx_buf, uart_rx_buf, instruction.status);
          break;
        case CANCEL_INSTRUCTION:
          if (instruction.arg_count != 0U) {
            instruction.status = STATUS_ERR_INVALID_ARG_COUNT;
            uart_echo(uart_tx_buf, uart_rx_buf, instruction.status);
            break;
          }
          instruction.status = cancel();
          uart_echo(uart_tx_buf, uart_rx_buf, instruction.status);
          break;
        case RUN_SETPOINT_INSTRUCTION:
          if (instruction.arg_count != 2U) {
            instruction.status = STATUS_ERR_INVALID_ARG_COUNT;
            uart_echo(uart_tx_buf, uart_rx_buf, instruction.status);
            break;
          }
          instruction.status = run_setpoint(instruction.args[0], instruction.args[1]);
          uart_echo(uart_tx_buf, uart_rx_buf, instruction.status);
          break;
        case RUN_PROFILE_INSTRUCTION:
          if (instruction.arg_count != 1U) {
            instruction.status = STATUS_ERR_INVALID_ARG_COUNT;
            uart_echo(uart_tx_buf, uart_rx_buf, instruction.status);
            break;
          }
          instruction.status = run_profile(instruction.args[0]);
          uart_echo(uart_tx_buf, uart_rx_buf, instruction.status);
          break;
        case GET_SETPOINT_INSTRUCTION:
          if (instruction.arg_count != 2U) {
            instruction.status = STATUS_ERR_INVALID_ARG_COUNT;
            uart_echo(uart_tx_buf, uart_rx_buf, instruction.status);
            break;
          }
          instruction.status = get_setpoint(instruction.args[0], instruction.args[1]);
          uart_echo(uart_tx_buf, uart_rx_buf, instruction.status);
          break;
        case ADD_SETPOINT_INSTRUCTION:
          if (instruction.arg_count != 4U) {
            instruction.status = STATUS_ERR_INVALID_ARG_COUNT;
            uart_echo(uart_tx_buf, uart_rx_buf, instruction.status);
            break;
          }
          instruction.status = add_setpoint(instruction.args[0], instruction.args[1], instruction.args[2], instruction.args[3]);
          // TODO: Consider echoing index after adding. Otherwise, user can use get_profile().
          uart_echo(uart_tx_buf, uart_rx_buf, instruction.status);
          break;
        case REMOVE_SETPOINT_INSTRUCTION:
          if (instruction.arg_count != 2U) {
            instruction.status = STATUS_ERR_INVALID_ARG_COUNT;
            uart_echo(uart_tx_buf, uart_rx_buf, instruction.status);
            break;
          }
          instruction.status = remove_setpoint(instruction.args[0], instruction.args[1]);
          uart_echo(uart_tx_buf, uart_rx_buf, instruction.status);
          break;
        case GET_PROFILE_INSTRUCTION:
          if (instruction.arg_count != 1U) {
            instruction.status = STATUS_ERR_INVALID_ARG_COUNT;
            uart_echo(uart_tx_buf, uart_rx_buf, instruction.status);
            break;
          }
          instruction.status = get_profile(instruction.args[0]);
          uart_echo(uart_tx_buf, uart_rx_buf, instruction.status);
          break;
        case CLEAR_PROFILE_INSTRUCTION:
          if (instruction.arg_count != 1U) {
            instruction.status = STATUS_ERR_INVALID_ARG_COUNT;
            uart_echo(uart_tx_buf, uart_rx_buf, instruction.status);
            break;
          }
          instruction.status = clear_profile(instruction.args[0]);
          uart_echo(uart_tx_buf, uart_rx_buf, instruction.status);
          break;
        case TEST_SERVOS_INSTRUCTION:
          if (instruction.arg_count != 0U) {
            instruction.status = STATUS_ERR_INVALID_ARG_COUNT;
            uart_echo(uart_tx_buf, uart_rx_buf, instruction.status);
            break;
          }
          instruction.status = test_servos();
          uart_echo(uart_tx_buf, uart_rx_buf, instruction.status);
          break;
        case TEST_ADXL_INSTRUCTION:
          if (instruction.arg_count != 0U) {
            instruction.status = STATUS_ERR_INVALID_ARG_COUNT;
            uart_echo(uart_tx_buf, uart_rx_buf, instruction.status);
            break;
          }
          instruction.status = test_adxl();
          uart_echo(uart_tx_buf, uart_rx_buf, instruction.status);
          break;
        case TEST_FLASH_INSTRUCTION:
          if (instruction.arg_count != 0U) {
            instruction.status = STATUS_ERR_INVALID_ARG_COUNT;
            uart_echo(uart_tx_buf, uart_rx_buf, instruction.status);
            break;
          }
          instruction.status = test_flash();
          uart_echo(uart_tx_buf, uart_rx_buf, instruction.status);
          break;
        case TEST_LED_INSTRUCTION:
          if (instruction.arg_count != 0U) {
            instruction.status = STATUS_ERR_INVALID_ARG_COUNT;
            uart_echo(uart_tx_buf, uart_rx_buf, instruction.status);
            break;
          }
          HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
          uart_echo(uart_tx_buf, uart_rx_buf, instruction.status);
          break;
        case TEST_ECHO_INSTRUCTION:
          if (instruction.arg_count != 0U) {
            instruction.status = STATUS_ERR_INVALID_ARG_COUNT;
            uart_echo(uart_tx_buf, uart_rx_buf, instruction.status);
            break;
          }
          uart_echo(uart_tx_buf, uart_rx_buf, instruction.status);
          break;

        // Instruction not in list of instruction codes
        default:
          instruction.status = STATUS_ERR_INVALID_INSTRUCTION;
          uart_echo(uart_tx_buf, uart_rx_buf, instruction.status);
          break;
        }
      }
    }
    // Reset buffers and flags
    memset(uart_circ_buf, 0, sizeof(uart_circ_buf)); // TODO: True circular buffer
    memset(uart_tx_buf, 0, sizeof(uart_tx_buf));
    instruction_flag = 0;
  }
}

/**
 * @brief Handles the running of a setpoint.
 * 
 * @ingroup StateHanlder
 * This function manages the movement of the system towards a setpoint defined by `active_setpoint.x` 
 * and `active_setpoint.y`. It increments or decrements the pulse width values (`CCR_X` and `CCR_Y`) 
 * in small steps, determined by the speed specified in `active_setpoint.speed`, until both the `CCR_X` 
 * and `CCR_Y` values match the target setpoint values.
 * 
 * The speed of the movement is inversely related to the `active_setpoint.speed` value, meaning a higher 
 * speed value results in a faster movement, while a lower speed results in slower movement. The system 
 * prevents overshooting the setpoint by adjusting the pulse width values to exactly match the target when 
 * it gets close enough.
 * 
 * Once both `CCR_X` and `CCR_Y` match the target setpoint values, the state transitions to `IDLE_STATE`.
 * Otherwise, the function continues to increment or decrement the pulse width values in the next cycle.
 * 
 * @return The current state, which will be `RUN_SETPOINT_STATE` while the setpoint is being processed 
 *         and `IDLE_STATE` once the setpoint is reached.
 */
system_state_t run_setpoint_state_handler(void) {
  if (CCR_X == active_setpoint.x && CCR_Y == active_setpoint.y) {
    return IDLE_STATE;
  }

  uint16_t step =  STEP_DELAY * (PULSE_WIDTH_POS_90 - PULSE_WIDTH_NEG_90) / (SPEED_MAX + 1U - active_setpoint.speed) / 1000;

  // Increment or decrement CCR_X towards active_setpoint.x
  if (CCR_X < active_setpoint.x) {
    CCR_X += step;
    if (CCR_X > active_setpoint.x) {
      CCR_X = active_setpoint.x; // Prevent overshooting
    }
  } else if (CCR_X > active_setpoint.x) {
    CCR_X -= step;
    if (CCR_X < active_setpoint.x) {
      CCR_X = active_setpoint.x; // Prevent overshooting
    }
  }

  // Increment or decrement CCR_Y towards active_setpoint.y
  if (CCR_Y < active_setpoint.y) {
    CCR_Y += step;
    if (CCR_Y > active_setpoint.y) {
      CCR_Y = active_setpoint.y; // Prevent overshooting
    }
  } else if (CCR_Y > active_setpoint.y) {
    CCR_Y -= step;
    if (CCR_Y < active_setpoint.y) {
      CCR_Y = active_setpoint.y; // Prevent overshooting
    }
  }

  HAL_Delay(STEP_DELAY);
  return RUN_SETPOINT_STATE;
}

/**
 * @brief Handles the running of a profile by iterating through setpoints.
 * 
 * @ingroup StateHanlder
 * This handler processes the current setpoint from a specified profile and moves the system towards 
 * the target setpoint values. The setpoint data is retrieved from flash memory, and the system adjusts 
 * the pulse width values (`CCR_X` and `CCR_Y`) to match the setpoint's `x` and `y` values. The movement 
 * speed is determined by the `setpoint.speed` value. Once a setpoint is reached, the function waits 
 * (via PROFILE_WAIT) and proceeds to the next setpoint in the profile.
 * 
 * If the requested setpoint contains invalid data (indicated by `FLASH_EMPTY` values), the function 
 * returns to the idle state.
 * 
 * The function continues running through the profile until all setpoints are processed, at which point 
 * the state transitions to `IDLE_STATE`.
 * 
 * @return The current state, which can be `RUN_PROFILE_STATE` if there are more setpoints to process, 
 *         or `IDLE_STATE` when the profile has been fully processed or if no valid setpoints are found.
 */
system_state_t run_profile_state_handler(void) {
  HAL_Delay(STEP_DELAY);
  if (next_state_e == IDLE_STATE) {
    return IDLE_STATE;
  }

  // Get setpoint pointer from index and profile arguments
  setpoint_t* setpoint = SETPOINT_ADDRESS(active_setpoint.index, active_setpoint.profile);

  // Check if requested setpoint contains data
  if (setpoint->x == FLASH_EMPTY && setpoint->y == FLASH_EMPTY && setpoint->speed == FLASH_EMPTY) {
    active_setpoint.index = 0;
    return IDLE_STATE;
  }
  
  active_setpoint.x = degtoccr(setpoint->x) + PULSE_WIDTH_OFFSET_X;
  active_setpoint.y = degtoccr(setpoint->y) + PULSE_WIDTH_OFFSET_Y;
  active_setpoint.speed = setpoint->speed;

  uint16_t step =  STEP_DELAY * (PULSE_WIDTH_POS_90 - PULSE_WIDTH_NEG_90) / (SPEED_MAX + 1U - active_setpoint.speed) / 1000;
  
  // Increment or decrement CCR_X towards active_setpoint.x
  if (CCR_X < active_setpoint.x) {
    CCR_X += step;
    if (CCR_X > active_setpoint.x) {
      CCR_X = active_setpoint.x; // Prevent overshooting
    }
  } else if (CCR_X > active_setpoint.x) {
    CCR_X -= step;
    if (CCR_X < active_setpoint.x) {
      CCR_X = active_setpoint.x; // Prevent overshooting
    }
  }

  // Increment or decrement CCR_Y towards active_setpoint.y
  if (CCR_Y < active_setpoint.y) {
    CCR_Y += step;
    if (CCR_Y > active_setpoint.y) {
      CCR_Y = active_setpoint.y; // Prevent overshooting
    }
  } else if (CCR_Y > active_setpoint.y) {
    CCR_Y -= step;
    if (CCR_Y < active_setpoint.y) {
      CCR_Y = active_setpoint.y; // Prevent overshooting
    }
  }
  
  if (CCR_X == active_setpoint.x && CCR_Y == active_setpoint.y) {
    active_setpoint.index++;
    HAL_Delay(PROFILE_WAIT);
  }
  if (next_state_e == RUN_PROFILE_STATE) {
    return RUN_PROFILE_STATE;
  } else {
    return IDLE_STATE;
  }
}
/**
 * @brief Handles the running for running a test on the servo
 * 
 * @ingroup StateHanlder
 * 
 * This handler manages the test state by controlling the movement of the servos 
 * towards their target setpoints and updating the system accordingly. It checks 
 * whether the setpoint has been reached and moves to the next setpoint if necessary.
 * After completing the movement, it performs additional operations such as testing 
 * the accelerometer and retrieving the next setpoint for the profile.
 * 
 * @return The current state, which can be `IDLE_STATE` if there are more setpoints to process, 
 *         or `RUN_TEST_STATE` when the profile has been fully processed or if no valid setpoints are found.
 */
system_state_t run_test_state_handler(void) {
  HAL_Delay(STEP_DELAY);
  //char test_buf[UART_TX_BUF_LEN];
  if (next_state_e == IDLE_STATE) {
    return IDLE_STATE;
  }

  // Get setpoint pointer from index and profile arguments
  setpoint_t* setpoint = SETPOINT_ADDRESS(active_setpoint.index, active_setpoint.profile);

  // Check if requested setpoint contains data
  if (setpoint->x == FLASH_EMPTY && setpoint->y == FLASH_EMPTY && setpoint->speed == FLASH_EMPTY) {
    active_setpoint.index = 0;
    return IDLE_STATE;
  }
  
  active_setpoint.x = degtoccr(setpoint->x) + PULSE_WIDTH_OFFSET_X;
  active_setpoint.y = degtoccr(setpoint->y) + PULSE_WIDTH_OFFSET_Y;
  active_setpoint.speed = setpoint->speed;

  uint16_t step =  STEP_DELAY * (PULSE_WIDTH_POS_90 - PULSE_WIDTH_NEG_90) / (SPEED_MAX + 1U - active_setpoint.speed) / 1000;
  
  // Increment or decrement CCR_X towards active_setpoint.x
  if (CCR_X < active_setpoint.x) {
    CCR_X += step;
    if (CCR_X > active_setpoint.x) {
      CCR_X = active_setpoint.x; // Prevent overshooting
    }
  } else if (CCR_X > active_setpoint.x) {
    CCR_X -= step;
    if (CCR_X < active_setpoint.x) {
      CCR_X = active_setpoint.x; // Prevent overshooting
    }
  }

  // Increment or decrement CCR_Y towards active_setpoint.y
  if (CCR_Y < active_setpoint.y) {
    CCR_Y += step;
    if (CCR_Y > active_setpoint.y) {
      CCR_Y = active_setpoint.y; // Prevent overshooting
    }
  } else if (CCR_Y > active_setpoint.y) {
    CCR_Y -= step;
    if (CCR_Y < active_setpoint.y) {
      CCR_Y = active_setpoint.y; // Prevent overshooting
    }
  }
  
  if (CCR_X == active_setpoint.x && CCR_Y == active_setpoint.y) {
    active_setpoint.index++;
    HAL_Delay(PROFILE_WAIT);
    test_adxl();
    get_setpoint(active_setpoint.index, active_setpoint.profile);
    //sprintf(test_buf, "{X Y}\r\n");
    //HAL_UART_Transmit(HUART_PTR, (uint8_t*)test_buf, strlen(test_buf), HAL_MAX_DELAY);
  }

  if (next_state_e == RUN_TEST_STATE) {
    return RUN_TEST_STATE;
  } else {
    return IDLE_STATE;
  }
}

/** @} */ // end of State Handler group
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
  MX_USART2_UART_Init();
  MX_SPI3_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

  //begin timer interrupt
  if (HAL_TIM_Base_Start_IT(&htim4) != HAL_OK)
  {
    /* Starting Error */
    Error_Handler();
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    //* State Handlers
    // Perform functions for given state, then return the value of the next state
    switch (next_state_e) {
      case STARTUP_STATE:
        next_state_e = startup_state_handler();
        break;
      case HOME_STATE:
        next_state_e = home_state_handler();
        break;
      case IDLE_STATE:
        break;
      case RUN_SETPOINT_STATE:
        next_state_e = run_setpoint_state_handler();
        break;
      case RUN_PROFILE_STATE:
        next_state_e = run_profile_state_handler();
        break;
      case RUN_TEST_STATE:
        next_state_e = run_test_state_handler();
        break;
      default:
        next_state_e = startup_state_handler();
        break;
    }
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
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
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi3.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 100 - 1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 20000 - 1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 100-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 20000;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = BAUD_RATE;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  huart2.Init.BaudRate = BAUD_RATE;
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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ADXL_CS_GPIO_Port, ADXL_CS_Pin, GPIO_PIN_RESET);

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

  /*Configure GPIO pin : ADXL_CS_Pin */
  GPIO_InitStruct.Pin = ADXL_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ADXL_CS_GPIO_Port, &GPIO_InitStruct);

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

#ifdef  USE_FULL_ASSERT
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
     ex: printf("Wrong parameters value: file %s on line %u\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

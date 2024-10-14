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
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <ctype.h>
#include <string.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef uint16_t instruction_size_t; // Typedef set to fit numbers of size INSTRUCTION_CODE_T_MAX

typedef enum {
  STARTUP_STATE,
  INSTRUCTION_WAIT_STATE,
  EDIT_SEQUENCE_STATE,
  RUN_STATE,
} system_state_t;

typedef enum {
  // Instruction code 0 = RESERVED
  // Motion Commands
  MOVE_INSTRUCTION = 1,
  STOP_INSTRUCTION,
  // Setpoint Commands
  ADD_SETPOINT_INSTRUCTION,
  REMOVE_SETPOINT_INSTRUCTION,
  CLEAR_SETPOINTS_INSTRUCTION,
  FINISH_EDIT_SETPOINT_INSTRUCTION,
  SET_SPEED_SETPOINT_INSTRUCTION,
  GET_SETPOINT_INSTRUCTION,
  GET_ALL_SETPOINTS_INSTRUCTION,
  // Sequence Commands
  GET_SEQUENCE_INSTRUCTION,
  RUN_SEQUENCE_INSTRUCTION,
  ADD_SETPOINT_TO_SEQUENCE_INSTRUCTION,
  CLEAR_SEQUENCE_INSTRUCTION,
  // Test Commands
  TEST_SERVOS_INSTRUCTION = 998U,
  TEST_ECHO_INSTRUCTION,
} instruction_code_t;

typedef enum {
  //* General
  STATUS_OK,
  STATUS_PROCESSING,
  //* Thrown by instruction handler
  STATUS_ERR_NO_INDICATOR,
  STATUS_ERR_NO_TERMINATOR,
  STATUS_ERR_INVALID_INSTRUCTION,
  STATUS_ERR_INSTRUCTION_OUT_OF_RANGE,
  STATUS_ERR_TOO_MANY_INSTRUCTIONS,
  STATUS_ERR_INVALID_ARG,
  STATUS_ERR_ARG_OUT_OF_RANGE,
  STATUS_ERR_TOO_MANY_ARGS,
} status_code_t;

typedef struct {
  status_code_t status;
  instruction_size_t code;
  instruction_size_t arguments[INSTRUCTION_MAX_ARGS];
  instruction_size_t arg_count;
} instruction_t;

typedef struct {
  float position; // In degrees
  float speed;    // Multiplier for the PID controller’s Proportional constant
} setpoint_t;

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

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

setpoint_t setpoints[INSTRUCTION_CODE_T_MAX + 1] = {{
  .position = 0,
  .speed = 0,
}};

struct setpoint_sequence {
  setpoint_t setpoints[INSTRUCTION_CODE_T_MAX + 1]; // Array of setpoint objects
};

//* Accelerometer Read
uint8_t adxl_rx_buf[ADXL_DATA_SIZE]; // The 6 bytes of ADXL data are stored here
float x_g, y_g, z_g;
float x_ang, y_ang, z_ang;

//* Servo Motor Control
float pos_x;
float pos_y;
float pos_x_last;
float pos_y_last;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//* Private Functions
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
    .status = STATUS_PROCESSING,
    .code = RESERVED,
    .arg_count = 0
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
  if (num > INSTRUCTION_CODE_T_MAX) {
    instruction.status = STATUS_ERR_INSTRUCTION_OUT_OF_RANGE;
    return instruction;
  }
  instruction.code = num;

  // Parse arguments
  while ((digit_token_p = strtok(NULL, "|}")) != NULL) {
    if (*digit_token_p == '{') {
      instruction.status = STATUS_ERR_TOO_MANY_INSTRUCTIONS;
      return instruction;
    }
    num = strtoint(digit_token_p);
    if (num < 0) {
      instruction.status = STATUS_ERR_INVALID_ARG;
      return instruction;
    }
    if (num > INSTRUCTION_CODE_T_MAX) {
      instruction.status = STATUS_ERR_ARG_OUT_OF_RANGE;
      return instruction;
    }
    if (instruction.arg_count >= INSTRUCTION_MAX_ARGS) {
      instruction.status = STATUS_ERR_TOO_MANY_ARGS;
      return instruction;
    }
    instruction.arguments[instruction.arg_count++] = num;
  }

  return instruction;
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
}

void adxl_id(void) {
  uint8_t device_id = 0;
  adxl_rx(ADXL_DEVID); // Assuming this will populate 'adxl_rx_buf' with the Device ID
  device_id = adxl_rx_buf[0]; // Assuming the ID is the first byte read

  #ifdef DEBUG
    char debug_buf[30];
    sprintf(debug_buf, "Device ID: 0x%X\r\n", device_id);
    HAL_UART_Transmit(&huart1, (uint8_t*)debug_buf, strlen(debug_buf), UART_TIMEOUT);
    HAL_UART_Transmit(&huart2, (uint8_t*)debug_buf, strlen(debug_buf), UART_TIMEOUT);
  #endif
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

  x_ang = atan2f(-y_g, sqrtf(x_g * x_g + z_g * z_g));
  y_ang = atan2f(-x_g, sqrtf(y_g * y_g + z_g * z_g));
  // z_ang = atan2f(sqrtf(x_g * x_g + y_g * y_g), z_g);

  x_ang = x_ang * (180.0f / M_PI);
  y_ang = y_ang * (180.0f / M_PI);
  // z_ang = z_ang * (180.0f / M_PI);

  #ifdef DEBUG
    char debug_buf[64];
    int len = snprintf(debug_buf, sizeof(debug_buf), "X(°)=%0.2f, Y(°)=%0.2f, Z(g)=%0.2f\r\n", x_ang, y_ang, z_g);
    if (len > 0 && len < sizeof(debug_buf)) {
      HAL_Delay(200); //*TODO Remove this delay when sample timer has been implemented
      HAL_UART_Transmit(&huart1, (uint8_t*)debug_buf, len, HAL_MAX_DELAY);
      HAL_UART_Transmit(&huart2, (uint8_t*)debug_buf, len, HAL_MAX_DELAY);
    }
  #endif
}

//* Instruction Functions
int move(float x_ang, float y_ang, int speed) {
  /**
   * TASK:
   * ! current_position = adxl_read();
   * PID_Update();
   * Function to translate x_ang, y_ang, & speed from degrees to CCR value
   * Set CCR values to final calculated position
   */
  /**
   * TODO:
   * Servo Control and Accelerometer Sampling Timer (Basically the system clock)
   * HAL_TIM_Base_Start(&htim1); // Clock (APB2) = 84 MHz. If Prescaler = (84 - 1) & Max Timer Count = (2^16 - 1), then f = 84 MHz / 84 = 1 MHz, T = 1 us, and Max Delay = (2^16 - 1) * T = 65.535 ms
   */
  return STATUS_OK;
}

// ! Do not create stop() function here. Trigger an interrupt from the instruction_handler() for the stop instruction, instead.
// Attempts to stop platform. If platform does not stop, reset stm board.

int run_setpoint(instruction_size_t setpoint_i) {
  // TODO: Same as move, but the setpoint information must be parsed from setpoints[]
  return STATUS_OK;
}

int test_servos(void) {
  char test_pos[100];
  int len;
  if (HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == GPIO_PIN_RESET) { //! Needs to be polled
    // Center Platform
    CCR_X = PULSE_WIDTH_0 + PULSE_WIDTH_OFFSET_X; // 0°
    CCR_Y = PULSE_WIDTH_0 + PULSE_WIDTH_OFFSET_Y; // 0°
    HAL_Delay(SERVO_TEST_DELAY);
    
    #ifdef DEBUG
      len = snprintf(test_pos, sizeof(test_pos), "Testing 0° for X and Y axes:\r\n");
      HAL_UART_Transmit(&huart2, (uint8_t*)test_pos, len, HAL_MAX_DELAY);
      adxl_read();
    #endif

    // Move X-Axis
    CCR_X = PULSE_WIDTH_NEG_90 + PULSE_WIDTH_OFFSET_X; // -90° or 45° (From PULSE_WIDTH_MIN)
    CCR_Y = PULSE_WIDTH_0 + PULSE_WIDTH_OFFSET_Y; // 0° or 135°
    HAL_Delay(SERVO_TEST_DELAY);
    
    #ifdef DEBUG
      len = snprintf(test_pos, sizeof(test_pos), "Testing -90° for X, 0° for Y axis:\r\n");
      HAL_UART_Transmit(&huart2, (uint8_t*)test_pos, len, HAL_MAX_DELAY);
      adxl_read();
    #endif
    CCR_X = PULSE_WIDTH_NEG_45 + PULSE_WIDTH_OFFSET_X; // -45° or 90° (From PULSE_WIDTH_MIN)
    CCR_Y = PULSE_WIDTH_0 + PULSE_WIDTH_OFFSET_Y; // 0° or 135°
    HAL_Delay(SERVO_TEST_DELAY);
    
    #ifdef DEBUG
      len = snprintf(test_pos, sizeof(test_pos), "Testing -45° for X, 0° for Y axis:\r\n");
      HAL_UART_Transmit(&huart2, (uint8_t*)test_pos, len, HAL_MAX_DELAY);
      adxl_read();
    #endif
    CCR_X = PULSE_WIDTH_0 + PULSE_WIDTH_OFFSET_X; // 0° or 135°
    CCR_Y = PULSE_WIDTH_0 + PULSE_WIDTH_OFFSET_Y; // 0° or 135°
    HAL_Delay(SERVO_TEST_DELAY);
    
    #ifdef DEBUG
      len = snprintf(test_pos, sizeof(test_pos), "Testing 0° for X and Y axes:\r\n");
      HAL_UART_Transmit(&huart2, (uint8_t*)test_pos, len, HAL_MAX_DELAY);
      adxl_read();
    #endif
    CCR_X = PULSE_WIDTH_POS_45 + PULSE_WIDTH_OFFSET_X; // +45° or 180° (From PULSE_WIDTH_MIN)
    CCR_Y = PULSE_WIDTH_0 + PULSE_WIDTH_OFFSET_Y; // 0° or 135°
    HAL_Delay(SERVO_TEST_DELAY);
    
    #ifdef DEBUG
      len = snprintf(test_pos, sizeof(test_pos), "Testing +45° for X, 0° for Y axis:\r\n");
      HAL_UART_Transmit(&huart2, (uint8_t*)test_pos, len, HAL_MAX_DELAY);
      adxl_read();
    #endif
    CCR_X = PULSE_WIDTH_POS_90 + PULSE_WIDTH_OFFSET_X; // +90° or 225° (From PULSE_WIDTH_MIN)
    CCR_Y = PULSE_WIDTH_0 + PULSE_WIDTH_OFFSET_Y; // 0° or 135°
    HAL_Delay(SERVO_TEST_DELAY);
    
    #ifdef DEBUG
      len = snprintf(test_pos, sizeof(test_pos), "Testing +90° for X, 0° for Y axis:\r\n");
      HAL_UART_Transmit(&huart2, (uint8_t*)test_pos, len, HAL_MAX_DELAY);
      adxl_read();
    #endif

    // Move Y-Axis
    CCR_X = PULSE_WIDTH_0 + PULSE_WIDTH_OFFSET_X; // 0° or 135°
    CCR_Y = PULSE_WIDTH_NEG_90 + PULSE_WIDTH_OFFSET_Y; // -90° or 45° (From PULSE_WIDTH_MIN)
    HAL_Delay(SERVO_TEST_DELAY);
    
    #ifdef DEBUG
      len = snprintf(test_pos, sizeof(test_pos), "Testing 0° for X, -90° for Y axis:\r\n");
      HAL_UART_Transmit(&huart2, (uint8_t*)test_pos, len, HAL_MAX_DELAY);
      adxl_read();
    #endif
    CCR_X = PULSE_WIDTH_0 + PULSE_WIDTH_OFFSET_X; // 0° or 135°
    CCR_Y = PULSE_WIDTH_NEG_45 + PULSE_WIDTH_OFFSET_Y; // -45° or 90° (From PULSE_WIDTH_MIN)
    HAL_Delay(SERVO_TEST_DELAY);
    
    #ifdef DEBUG
      len = snprintf(test_pos, sizeof(test_pos), "Testing 0° for X, -45° for Y axis:\r\n");
      HAL_UART_Transmit(&huart2, (uint8_t*)test_pos, len, HAL_MAX_DELAY);
      adxl_read();
    #endif
    CCR_X = PULSE_WIDTH_0 + PULSE_WIDTH_OFFSET_X; // 0° or 135°
    CCR_Y = PULSE_WIDTH_0 + PULSE_WIDTH_OFFSET_Y; // 0° or 135°
    HAL_Delay(SERVO_TEST_DELAY);
    
    #ifdef DEBUG
      len = snprintf(test_pos, sizeof(test_pos), "Testing 0° for X and Y axes:\r\n");
      HAL_UART_Transmit(&huart2, (uint8_t*)test_pos, len, HAL_MAX_DELAY);
      adxl_read();
    #endif
    CCR_X = PULSE_WIDTH_0 + PULSE_WIDTH_OFFSET_X; // 0° or 135°
    CCR_Y = PULSE_WIDTH_POS_45 + PULSE_WIDTH_OFFSET_Y; // +45° or 180° (From PULSE_WIDTH_MIN)
    HAL_Delay(SERVO_TEST_DELAY);
    
    #ifdef DEBUG
      len = snprintf(test_pos, sizeof(test_pos), "Testing 0° for X, +45° for Y axis:\r\n");
      HAL_UART_Transmit(&huart2, (uint8_t*)test_pos, len, HAL_MAX_DELAY);
      adxl_read();
    #endif
    CCR_X = PULSE_WIDTH_0 + PULSE_WIDTH_OFFSET_X; // 0° or 135°
    CCR_Y = PULSE_WIDTH_POS_90 + PULSE_WIDTH_OFFSET_Y; // +90° or 225° (From PULSE_WIDTH_MIN)
    HAL_Delay(SERVO_TEST_DELAY);
    
    #ifdef DEBUG
      len = snprintf(test_pos, sizeof(test_pos), "Testing 0° for X, +90° for Y axis:\r\n");
      HAL_UART_Transmit(&huart2, (uint8_t*)test_pos, len, HAL_MAX_DELAY);
      adxl_read();
    #endif

    // Move Both Axes
    CCR_X = PULSE_WIDTH_NEG_45 + PULSE_WIDTH_OFFSET_X; // -45°
    CCR_Y = PULSE_WIDTH_NEG_45 + PULSE_WIDTH_OFFSET_Y; // -45°
    HAL_Delay(SERVO_TEST_DELAY);
    
    #ifdef DEBUG
      len = snprintf(test_pos, sizeof(test_pos), "Testing -45° for X and Y axes:\r\n");
      HAL_UART_Transmit(&huart2, (uint8_t*)test_pos, len, HAL_MAX_DELAY);
      adxl_read();
    #endif
    CCR_X = PULSE_WIDTH_POS_45 + PULSE_WIDTH_OFFSET_X; // +45°
    CCR_Y = PULSE_WIDTH_POS_45 + PULSE_WIDTH_OFFSET_Y; // +45°
    HAL_Delay(SERVO_TEST_DELAY);
    
    #ifdef DEBUG
      len = snprintf(test_pos, sizeof(test_pos), "Testing +45° for X and Y axes:\r\n");
      HAL_UART_Transmit(&huart2, (uint8_t*)test_pos, len, HAL_MAX_DELAY);
      adxl_read();
    #endif
    CCR_X = PULSE_WIDTH_POS_45 + PULSE_WIDTH_OFFSET_X; // +45°
    CCR_Y = PULSE_WIDTH_NEG_45 + PULSE_WIDTH_OFFSET_Y; // -45°
    HAL_Delay(SERVO_TEST_DELAY);
    
    #ifdef DEBUG
      len = snprintf(test_pos, sizeof(test_pos), "Testing +45° for X, -45° for Y axis:\r\n");
      HAL_UART_Transmit(&huart2, (uint8_t*)test_pos, len, HAL_MAX_DELAY);
      adxl_read();
    #endif
    CCR_X = PULSE_WIDTH_NEG_45 + PULSE_WIDTH_OFFSET_X; // -45°
    CCR_Y = PULSE_WIDTH_POS_45 + PULSE_WIDTH_OFFSET_Y; // +45°
    HAL_Delay(SERVO_TEST_DELAY);
    
    #ifdef DEBUG
      len = snprintf(test_pos, sizeof(test_pos), "Testing -45° for X, +45° for Y axis:\r\n");
      HAL_UART_Transmit(&huart2, (uint8_t*)test_pos, len, HAL_MAX_DELAY);
      adxl_read();
    #endif
    CCR_X = PULSE_WIDTH_0 + PULSE_WIDTH_OFFSET_X; // 0°
    CCR_Y = PULSE_WIDTH_0 + PULSE_WIDTH_OFFSET_Y; // 0°
    HAL_Delay(SERVO_TEST_DELAY);
    
    #ifdef DEBUG
      len = snprintf(test_pos, sizeof(test_pos), "Testing 0° for X and Y axes:\r\n");
      HAL_UART_Transmit(&huart2, (uint8_t*)test_pos, len, HAL_MAX_DELAY);
      adxl_read();
    #endif
  }
  return STATUS_OK;
}

//* Event Handlers
system_state_t startup_state_handler(void) {
  // PWM
  // Internal Clock (HCLK) = 100 MHz. If Prescaler = (100 - 1) & Max Timer Count = (20000 - 1),
  // then f = 100 MHz / 100 = 1 MHz, T = 1 us, and PWM f = 1/(20000 * T) = 50 Hz
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);

  // Accelerometer
  adxl_init();
  adxl_id();

  return INSTRUCTION_WAIT_STATE;
}

// Waits for UART instruction interrupt to trigger, then processes instruction
system_state_t instruction_wait_state_handler(void) {
  //* Receive instruction via interrupt
  char uart_rx_buf[RX_BUF_LEN] = "{999|111|22|3|00004}"; // ! Here for testing
  // TODO: uart_rx_buf Interrupt

  //* Parse received instruction
  char uart_tx_buf[TX_BUF_LEN] = { 0 };

  // Copy UART buffer to prevent parse_instruction() from manipulating the original (due to strtok())
  char* parse_buf = calloc((strlen(uart_rx_buf) + 1U), sizeof(char)); // Allocate memory based on size of text in buffer
  strcpy(parse_buf, uart_rx_buf);

  instruction_t instruction = parse_instruction(parse_buf);
  free(parse_buf); // Free memory allocated by calloc()

  //* Echo received instruction with status after parsing
  sprintf(uart_tx_buf, "%s[%02d]", uart_rx_buf, (instruction.status % 100)); // Modulo truncates to 2 digits
  HAL_UART_Transmit(&huart1, (uint8_t*)uart_tx_buf, sizeof(uart_tx_buf), UART_TIMEOUT);

  //* Instruction switch
  // TODO: switch () {}

  // TODO: Change debug_print() to print with UART
  debug_print("Echo: %s\n", uart_tx_buf);

  debug_print("Instruction code: %d\n", instruction.code);
  for (int i = 0; i < instruction.arg_count; ++i) {
    debug_print("Argument %d: %d\n", i + 1, instruction.arguments[i]);
  }
  debug_print("Arg Count: %d\n", instruction.arg_count);

  return INSTRUCTION_WAIT_STATE;
}

system_state_t edit_sequence_state_handler(void) {
  // Allow user to define and store a sequence
  return INSTRUCTION_WAIT_STATE;
}

system_state_t run_state_handler(void) {
  // Listen for stop commands
  // Run test setpoint
  // Calls move servo function
  return INSTRUCTION_WAIT_STATE;
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
  MX_USART2_UART_Init();
  MX_SPI3_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  //* Startup State
  system_state_t next_state_e = STARTUP_STATE;

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
      case INSTRUCTION_WAIT_STATE:
        next_state_e = instruction_wait_state_handler();
        break;
      case EDIT_SEQUENCE_STATE:
        next_state_e = edit_sequence_state_handler();
        break;
      case RUN_STATE:
        next_state_e = run_state_handler();
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

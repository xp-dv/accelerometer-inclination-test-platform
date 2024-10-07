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
#include <string.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
  STARTUP_STATE,
  WAIT_FOR_INSTRUCTION,
  SEQUENCE_EDIT,
  RUN_SEQUENCE,
  MOVE_PLATFORM
} system_state_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#ifdef TEST // The following code will only be compiled if TEST is defined in the header file
  #define PULSE_WIDTH_RANGE (PULSE_WIDTH_MAX - PULSE_WIDTH_MIN)
  #define PULSE_WIDTH_MIN 500 // us
  #define PULSE_WIDTH_NEG_90 (PULSE_WIDTH_0 - (PULSE_WIDTH_RANGE / 3))
  #define PULSE_WIDTH_NEG_45 (PULSE_WIDTH_0 - (PULSE_WIDTH_RANGE / 6))
  #define PULSE_WIDTH_0 ((PULSE_WIDTH_MAX + PULSE_WIDTH_MIN) / 2)
  #define PULSE_WIDTH_POS_45 (PULSE_WIDTH_0 + (PULSE_WIDTH_RANGE / 6))
  #define PULSE_WIDTH_POS_90 (PULSE_WIDTH_0 + (PULSE_WIDTH_RANGE / 3))
  #define PULSE_WIDTH_MAX 2500 // us
  #define PULSE_WIDTH_OFFSET_X -15
  #define PULSE_WIDTH_OFFSET_Y -25
  #define SERVO_TEST_DELAY 3000 // ms

  //! Alternate Servo Control Method
  // #define SERVO_NEUTRAL 75.5
#endif
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
// Accelerometer Read
uint8_t data_rec[6]; // The 6 bytes of ADXL data are stored here
float x_g, y_g, z_g;
float x_ang, y_ang, z_ang;

// Servo Motor Control
float pos_x;
float pos_y;
float pos_x_last;
float pos_y_last;

#ifdef TEST // The following code will only be compiled if TEST is defined in the header file
  //! Alternate Servo Control Method
  // float offset = 0; // Angle of center position relative to servo neutral
  // float XPos = 90; // Desired X Axis angle, range: -135 to +135
  // float YPos = 90; // Desired Y Axis angle, range: -135 to +135
  // // CCR values for the desired X or Y Axis angle
  // float XValP, XValN, YValP, YValN;
  // // CCR values for the desired X or Y Axis angle / 2
  // float XValP2, XValN2, YValP2, YValN2;
#endif
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
void instruction_handler(void) {
  int status = 1; // Assume failure

  // TODO: replace with real instructions defined in a typedef
  // instruction_t instruction = uart_rx();
  // switch (instruction) {
  //   case INSTRUCTION:
  //     instruction_handler();
  //     break;
  // }
}

int move_platform(int x_ang, int y_ang, int speed) {
  // TODO: current_position = adxl_read();
  // TODO: PID_Update();
  // TODO: Function to translate x_ang, y_ang, & speed from degrees to CCR value
  // TODO: Set CCR values to final calculated position
}

int run_setpoint(int* setpoint_i) {
 // TODO: Same as move_platform, but the setpoint information must be parsed from setpoints[]
}

void adxl_tx(uint8_t address, uint8_t value) {
  uint8_t data[2];
  data[0] = address | 0x40;  // multibyte write enabled
  data[1] = value;
  HAL_GPIO_WritePin(ADXL_CS_GPIO_Port, ADXL_CS_Pin, GPIO_PIN_RESET); // pull the cs pin low to enable the slave
  HAL_SPI_Transmit(&hspi3, data, 2, 100); // transmit the address and data
  HAL_GPIO_WritePin(ADXL_CS_GPIO_Port, ADXL_CS_Pin, GPIO_PIN_SET); // pull the cs pin high to disable the slave
}

void adxl_rx(uint8_t address) {
  address |= 0x80;  // read operation
  address |= 0x40;  // multibyte read
  HAL_GPIO_WritePin(ADXL_CS_GPIO_Port, ADXL_CS_Pin, GPIO_PIN_RESET);  // pull the cs pin low to enable the slave
  HAL_SPI_Transmit(&hspi3, &address, 1, 100);  // send the address from where you want to read data
  HAL_SPI_Receive(&hspi3, data_rec, 6, 100);  // read 6 BYTES of data
  HAL_GPIO_WritePin(ADXL_CS_GPIO_Port, ADXL_CS_Pin, GPIO_PIN_SET);  // pull the cs pin high to disable the slave
}

void adxl_init(void) {
  HAL_Delay(2000);
  adxl_tx(0x31, 0x01);  // data_format range= +- 4g
  adxl_tx(0x2d, 0x00);  // reset all bits
  adxl_tx(0x2d, 0x08);  // power_cntl measure and wake up 8hz
}

void adxl_id(void) {
  uint8_t device_id_addr = 0x00; // Address of the device ID register
  uint8_t device_id = 0;
  adxl_rx(device_id_addr); // Assuming this will populate 'data_rec' with the ID

  device_id = data_rec[0]; // Assuming the ID is the first byte read
  char debug_message[30];
  sprintf(debug_message, "Device ID: 0x%X\r\n", device_id);
  HAL_UART_Transmit(&huart1, (uint8_t*)debug_message, strlen(debug_message), 100);
  HAL_UART_Transmit(&huart2, (uint8_t*)debug_message, strlen(debug_message), 100);
}

void adxl_read(void) {
  adxl_rx(0x32); // Request data starting from the DATAX0 register
  // Convert the accelerometer values to 16-bit signed integers
  int16_t x_raw = (int16_t)((data_rec[1] << 8) | data_rec[0]);
  int16_t y_raw = (int16_t)((data_rec[3] << 8) | data_rec[2]);
  int16_t z_raw = (int16_t)((data_rec[5] << 8) | data_rec[4]);

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

  char uart_buf[64];
  int len = snprintf(uart_buf, sizeof(uart_buf), "X(°)=%0.2f, Y(°)=%0.2f, Z(g)=%0.2f\r\n", x_ang, y_ang, z_g);
  if (len > 0 && len < sizeof(uart_buf)) {
  HAL_Delay(200); //*TODO Remove this delay when sample timer has been implemented
  HAL_UART_Transmit(&huart1, (uint8_t*)uart_buf, len, HAL_MAX_DELAY);
  HAL_UART_Transmit(&huart2, (uint8_t*)uart_buf, len, HAL_MAX_DELAY); // Send data
  }
}

//*TODO Incremental position movement. Call the read accelerometer function every time the position is updated
//*TODO Convert x and y-axis angular position to pulse width position
//! For now pos_x, pos_y, v_x, and v_y are in units of 1 ms pulse width
// int move_servo(float pos_x, float pos_y, int v_x, int v_y) {
//   int pos_x_pw = pos_x;
//   int pos_y_pw = pos_y;
//   int sample_rate = 1; // 1 ms
//   v_x = pos_x - pos_x_last;
//   v_y = pos_y - pos_y_last;
//   for (pos_x_pw = pos_x_last, pos_y_pw = pos_y_last; pos_x_pw != pos_x_last && pos_y_pw != pos_y_last; pos_x_pw += v_x, pos_y_pw += v_y) { //*TODO Conditional statements not functional in case velocity isn't an exact multiple
//     adxl_read();
//     CCR_X = pos_x_pw;
//     CCR_X = pos_y_pw;
//     delay_us(sample_rate);
//   }
//   //! Alternate Servo Control Method
//   // CCR = 75 + (angle)(1/2.7)
//   // 1 degree = 2.7 CCR
//   // CCR1 = 75 + pos_x*(1/2.7); // Move x-axis servo

//   // CCR2 = 75 + pos_y*(1/2.7); // Move y-axis servo
//   // HAL_Delay(1000); // Wait before moving again
// }

#ifdef TEST // The following code will only be compiled if TEST is defined in the header file
  void servo_test(void) {
    char test_pos[100];
    int len;
    if (HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == GPIO_PIN_RESET) {
      // Center Platform
      CCR_X = PULSE_WIDTH_0 + PULSE_WIDTH_OFFSET_X; // 0°
      CCR_Y = PULSE_WIDTH_0 + PULSE_WIDTH_OFFSET_Y; // 0°
      HAL_Delay(SERVO_TEST_DELAY);
      len = snprintf(test_pos, sizeof(test_pos), "Testing 0° for X and Y axes:\r\n");
      HAL_UART_Transmit(&huart2, (uint8_t*)test_pos, len, HAL_MAX_DELAY);
      adxl_read();

      // Move X-Axis
      CCR_X = PULSE_WIDTH_NEG_90 + PULSE_WIDTH_OFFSET_X; // -90° or 45° (From PULSE_WIDTH_MIN)
      CCR_Y = PULSE_WIDTH_0 + PULSE_WIDTH_OFFSET_Y; // 0° or 135°
      HAL_Delay(SERVO_TEST_DELAY);
      len = snprintf(test_pos, sizeof(test_pos), "Testing -90° for X, 0° for Y axis:\r\n");
      HAL_UART_Transmit(&huart2, (uint8_t*)test_pos, len, HAL_MAX_DELAY);
      adxl_read();
      CCR_X = PULSE_WIDTH_NEG_45 + PULSE_WIDTH_OFFSET_X; // -45° or 90° (From PULSE_WIDTH_MIN)
      CCR_Y = PULSE_WIDTH_0 + PULSE_WIDTH_OFFSET_Y; // 0° or 135°
      HAL_Delay(SERVO_TEST_DELAY);
      len = snprintf(test_pos, sizeof(test_pos), "Testing -45° for X, 0° for Y axis:\r\n");
      HAL_UART_Transmit(&huart2, (uint8_t*)test_pos, len, HAL_MAX_DELAY);
      adxl_read();
      CCR_X = PULSE_WIDTH_0 + PULSE_WIDTH_OFFSET_X; // 0° or 135°
      CCR_Y = PULSE_WIDTH_0 + PULSE_WIDTH_OFFSET_Y; // 0° or 135°
      HAL_Delay(SERVO_TEST_DELAY);
      len = snprintf(test_pos, sizeof(test_pos), "Testing 0° for X and Y axes:\r\n");
      HAL_UART_Transmit(&huart2, (uint8_t*)test_pos, len, HAL_MAX_DELAY);
      adxl_read();
      CCR_X = PULSE_WIDTH_POS_45 + PULSE_WIDTH_OFFSET_X; // +45° or 180° (From PULSE_WIDTH_MIN)
      CCR_Y = PULSE_WIDTH_0 + PULSE_WIDTH_OFFSET_Y; // 0° or 135°
      HAL_Delay(SERVO_TEST_DELAY);
      len = snprintf(test_pos, sizeof(test_pos), "Testing +45° for X, 0° for Y axis:\r\n");
      HAL_UART_Transmit(&huart2, (uint8_t*)test_pos, len, HAL_MAX_DELAY);
      adxl_read();
      CCR_X = PULSE_WIDTH_POS_90 + PULSE_WIDTH_OFFSET_X; // +90° or 225° (From PULSE_WIDTH_MIN)
      CCR_Y = PULSE_WIDTH_0 + PULSE_WIDTH_OFFSET_Y; // 0° or 135°
      HAL_Delay(SERVO_TEST_DELAY);
      len = snprintf(test_pos, sizeof(test_pos), "Testing +90° for X, 0° for Y axis:\r\n");
      HAL_UART_Transmit(&huart2, (uint8_t*)test_pos, len, HAL_MAX_DELAY);
      adxl_read();

      // Move Y-Axis
      CCR_X = PULSE_WIDTH_0 + PULSE_WIDTH_OFFSET_X; // 0° or 135°
      CCR_Y = PULSE_WIDTH_NEG_90 + PULSE_WIDTH_OFFSET_Y; // -90° or 45° (From PULSE_WIDTH_MIN)
      HAL_Delay(SERVO_TEST_DELAY);
      len = snprintf(test_pos, sizeof(test_pos), "Testing 0° for X, -90° for Y axis:\r\n");
      HAL_UART_Transmit(&huart2, (uint8_t*)test_pos, len, HAL_MAX_DELAY);
      adxl_read();
      CCR_X = PULSE_WIDTH_0 + PULSE_WIDTH_OFFSET_X; // 0° or 135°
      CCR_Y = PULSE_WIDTH_NEG_45 + PULSE_WIDTH_OFFSET_Y; // -45° or 90° (From PULSE_WIDTH_MIN)
      HAL_Delay(SERVO_TEST_DELAY);
      len = snprintf(test_pos, sizeof(test_pos), "Testing 0° for X, -45° for Y axis:\r\n");
      HAL_UART_Transmit(&huart2, (uint8_t*)test_pos, len, HAL_MAX_DELAY);
      adxl_read();
      CCR_X = PULSE_WIDTH_0 + PULSE_WIDTH_OFFSET_X; // 0° or 135°
      CCR_Y = PULSE_WIDTH_0 + PULSE_WIDTH_OFFSET_Y; // 0° or 135°
      HAL_Delay(SERVO_TEST_DELAY);
      len = snprintf(test_pos, sizeof(test_pos), "Testing 0° for X and Y axes:\r\n");
      HAL_UART_Transmit(&huart2, (uint8_t*)test_pos, len, HAL_MAX_DELAY);
      adxl_read();
      CCR_X = PULSE_WIDTH_0 + PULSE_WIDTH_OFFSET_X; // 0° or 135°
      CCR_Y = PULSE_WIDTH_POS_45 + PULSE_WIDTH_OFFSET_Y; // +45° or 180° (From PULSE_WIDTH_MIN)
      HAL_Delay(SERVO_TEST_DELAY);
      len = snprintf(test_pos, sizeof(test_pos), "Testing 0° for X, +45° for Y axis:\r\n");
      HAL_UART_Transmit(&huart2, (uint8_t*)test_pos, len, HAL_MAX_DELAY);
      adxl_read();
      CCR_X = PULSE_WIDTH_0 + PULSE_WIDTH_OFFSET_X; // 0° or 135°
      CCR_Y = PULSE_WIDTH_POS_90 + PULSE_WIDTH_OFFSET_Y; // +90° or 225° (From PULSE_WIDTH_MIN)
      HAL_Delay(SERVO_TEST_DELAY);
      len = snprintf(test_pos, sizeof(test_pos), "Testing 0° for X, +90° for Y axis:\r\n");
      HAL_UART_Transmit(&huart2, (uint8_t*)test_pos, len, HAL_MAX_DELAY);
      adxl_read();

      // Move Both Axes
      CCR_X = PULSE_WIDTH_NEG_45 + PULSE_WIDTH_OFFSET_X; // -45°
      CCR_Y = PULSE_WIDTH_NEG_45 + PULSE_WIDTH_OFFSET_Y; // -45°
      HAL_Delay(SERVO_TEST_DELAY);
      len = snprintf(test_pos, sizeof(test_pos), "Testing -45° for X and Y axes:\r\n");
      HAL_UART_Transmit(&huart2, (uint8_t*)test_pos, len, HAL_MAX_DELAY);
      adxl_read();
      CCR_X = PULSE_WIDTH_POS_45 + PULSE_WIDTH_OFFSET_X; // +45°
      CCR_Y = PULSE_WIDTH_POS_45 + PULSE_WIDTH_OFFSET_Y; // +45°
      HAL_Delay(SERVO_TEST_DELAY);
      len = snprintf(test_pos, sizeof(test_pos), "Testing +45° for X and Y axes:\r\n");
      HAL_UART_Transmit(&huart2, (uint8_t*)test_pos, len, HAL_MAX_DELAY);
      adxl_read();
      CCR_X = PULSE_WIDTH_POS_45 + PULSE_WIDTH_OFFSET_X; // +45°
      CCR_Y = PULSE_WIDTH_NEG_45 + PULSE_WIDTH_OFFSET_Y; // -45°
      HAL_Delay(SERVO_TEST_DELAY);
      len = snprintf(test_pos, sizeof(test_pos), "Testing +45° for X, -45° for Y axis:\r\n");
      HAL_UART_Transmit(&huart2, (uint8_t*)test_pos, len, HAL_MAX_DELAY);
      adxl_read();
      CCR_X = PULSE_WIDTH_NEG_45 + PULSE_WIDTH_OFFSET_X; // -45°
      CCR_Y = PULSE_WIDTH_POS_45 + PULSE_WIDTH_OFFSET_Y; // +45°
      HAL_Delay(SERVO_TEST_DELAY);
      len = snprintf(test_pos, sizeof(test_pos), "Testing -45° for X, +45° for Y axis:\r\n");
      HAL_UART_Transmit(&huart2, (uint8_t*)test_pos, len, HAL_MAX_DELAY);
      adxl_read();
      CCR_X = PULSE_WIDTH_0 + PULSE_WIDTH_OFFSET_X; // 0°
      CCR_Y = PULSE_WIDTH_0 + PULSE_WIDTH_OFFSET_Y; // 0°
      HAL_Delay(SERVO_TEST_DELAY);
      len = snprintf(test_pos, sizeof(test_pos), "Testing 0° for X and Y axes:\r\n");
      HAL_UART_Transmit(&huart2, (uint8_t*)test_pos, len, HAL_MAX_DELAY);
      adxl_read();

      //! Alternate Servo Control Method
      // Calculate CCR Values
      // XValP = SERVO_NEUTRAL + (XPos + offset)*(1/2.7); // Calculate CCR value for desired +XPos
      // XValN = SERVO_NEUTRAL + (-XPos + offset)*(1/2.7); // Calculate CCR value for desired -XPos
      // YValP = SERVO_NEUTRAL + (YPos + offset)*(1/2.7); // Calculate CCR value for desired +YPos
      // YValN = SERVO_NEUTRAL + (-YPos + offset)*(1/2.7); // Calculate CCR value for desired -YPos
      // XValP2 = SERVO_NEUTRAL + (XPos/2 + offset)*(1/2.7); // Calculate CCR value for desired +XPos
      // XValN2 = SERVO_NEUTRAL + (-XPos/2 + offset)*(1/2.7); // Calculate CCR value for desired -XPos
      // YValP2 = SERVO_NEUTRAL + (YPos/2 + offset)*(1/2.7); // Calculate CCR value for desired +YPos
      // YValN2 = SERVO_NEUTRAL + (-YPos/2 + offset)*(1/2.7); // Calculate CCR value for desired -YPos

      // /* Move X Axis */
      // CCR_X = XValN; // Move X-Axis to -XPos
      // HAL_Delay(SERVO_TEST_DELAY);
      // CCR_X = XValP; // Move X-Axis to +XPos
      // HAL_Delay(SERVO_TEST_DELAY);
      // CCR_X = SERVO_NEUTRAL; // Return X-Axis to Neutral
      // HAL_Delay(SERVO_TEST_DELAY);

      // /* Move Y Axis */
      // CCR_Y = YValN; // Move Y-Axis to -YPos
      // HAL_Delay(SERVO_TEST_DELAY);
      // CCR_Y = YValP; // Move Y-Axis to +YPos
      // HAL_Delay(SERVO_TEST_DELAY);
      // CCR_Y = SERVO_NEUTRAL; // Return Y-Axis to Neutral
      // HAL_Delay(SERVO_TEST_DELAY);

      // /* Move Both Axes */
      // CCR_X = XValN2; // Move X-Axis to -XPos
      // CCR_Y = YValN2; // Move Y-Axis to -YPos
      // HAL_Delay(SERVO_TEST_DELAY);
      // CCR_X = XValP2; // Move X-Axis to -XPos
      // CCR_Y = YValN2; // Move Y-Axis to -YPos
      // HAL_Delay(SERVO_TEST_DELAY);
      // CCR_X = XValP2; // Move X-Axis to -XPos
      // CCR_Y = YValP2; // Move Y-Axis to -YPos
      // HAL_Delay(SERVO_TEST_DELAY);
      // CCR_X = XValN2; // Move X-Axis to +XPos
      // CCR_Y = YValP2; // Move Y-Axis to +YPos
      // HAL_Delay(SERVO_TEST_DELAY);
      // CCR_X = SERVO_NEUTRAL; // Return X-Axis to Neutral
      // CCR_Y = SERVO_NEUTRAL; // Return Y-Axis to Neutral
      // HAL_Delay(SERVO_TEST_DELAY);
    }
  }
#endif
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* Prototype Event Handlers */
system_state_t startup_handler(void) {
  // System startup processes
  // Calibrate/center servos
  return WAIT_FOR_INSTRUCTION;
}

system_state_t waiting_handler(void) {
  //*TODO Detect and decode all incoming serial instructions to determine the next state

  // Else
  return WAIT_FOR_INSTRUCTION;
}

system_state_t edit_sequence_handler(void) {
  // Allow user to define and store a test setpoint
  return WAIT_FOR_INSTRUCTION;
}

system_state_t run_setpoint_handler(void) {
  // Run test setpoint
  // Calls move servo function
  return WAIT_FOR_INSTRUCTION;
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

  /* PWM Timers */
  // Internal Clock (HCLK) = 100 MHz. If Prescaler = (100 - 1) & Max Timer Count = (20000 - 1), then f = 100 MHz / 100 = 1 MHz, T = 1 us, and PWM f = 1/(20000 * T) = 50 Hz
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  //*TODO Servo Control and Accelerometer Sample Timer (Basically the system clock)
  // HAL_TIM_Base_Start(&htim1); // Internal Clock (APB2) = 84 MHz. If Prescaler = (84 - 1) & Max Timer Count = (2^16 - 1), then f = 84 MHz / 84 = 1 MHz, T = 1 us, and Max Delay = (2^16 - 1) * T = 65.535 ms

  system_state_t next_state_e = STARTUP_STATE;
  adxl_init();
  adxl_id();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    switch (next_state_e) {
      case STARTUP_STATE:
        next_state_e = startup_handler();
        break;
      case WAIT_FOR_INSTRUCTION:
        #ifdef TEST // The following code will only be compiled if TEST is defined in the header file
          servo_test();
        #endif
          
        next_state_e = waiting_handler();
        break;
      case SEQUENCE_EDIT:
        next_state_e = edit_sequence_handler();
        break;
      case RUN_SEQUENCE:
        next_state_e = run_setpoint_handler();
        break;
      
      default:
        next_state_e = startup_handler();
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
  huart1.Init.BaudRate = 115200;
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

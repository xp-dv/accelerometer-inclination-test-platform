/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

 /** 
 * @page Command_Table Command Codes and Descriptions
 *
 * @section overview Overview
 * This table lists the available commands, their corresponding codes, arguments, and descriptions.
 * These commands control the system's motion, profiles, and tests.
 * 
 * @section Command Syntax
Each instruction will follow the following format:

`{code|arg1 arg2 ... argN}`

`<code>`: Required instruction code within the range (0-999). <br>
`[arg1], [arg2], […]`: Optional arguments. The number of arguments and range of accepted values for each argument will depend on the instruction code given. <br>
> Ex: Some instructions have no arguments, others require multiple.

`‘{’`: All instructions must begin with the start indicator. <br>
`‘}’`: All instructions must end with the end terminator. <br>
`‘|’`: The first argument (if applicable) must be separated from the instruction code with the instruction separator. <br>
`‘ ’`: All subsequent arguments must be separated by the argument separator. <br>
All numbers must be represented in decimal.

After an instruction is carried out, the received instruction will be echoed by the device in the following format:

`[2-digit status code]{<code>|[arg_1] [arg_2] ... [arg_n]}`


 * @section command_table Command Table
 * | Name            | Code | Arguments                  | Description                                              |
 * |-----------------|------|---------------------------|----------------------------------------------------------|
 * | MOVE            | 001  | x-ang, y-ang, speed       | Moves the platform to the input angle at the input speed |
 * | STOP            | 002  | None                      | Stops the platform from moving, will require reboot      |
 * | CANCEL          | 003  | None                      | Cancels platform movement and returns to home position   |
 * | RUN_SETPOINT    | 004  | Setpoint Index, Profile Index | Moves the platform to a setpoint in the profile        |
 * | RUN_PROFILE     | 005  | Profile Index             | Moves the platform sequentially through profile setpoints|
 * | GET_SETPOINT    | 006  | Setpoint Index, Profile Index | Shows values of a setpoint in the profile              |
 * | ADD_SETPOINT    | 007  | x-ang, y-ang, speed, Profile Index | Adds a setpoint to the profile                    |
 * | REMOVE_SETPOINT | 008  | Setpoint Index, Profile Index | Removes a setpoint from the profile                   |
 * | GET_PROFILE     | 009  | Profile Index             | Shows values for all setpoints in the profile           |
 * | CLEAR_PROFILE   | 010  | Profile Index             | Removes all setpoints from the profile                  |
 * | TEST_SERVOS     | 995  | None                      | Moves the platform to preset setpoints                  |
 * | TEST_ADXL       | 996  | None                      | Reads and prints accelerometer output                   |
 * | TEST_FLASH      | 997  | None                      | Saves preset values into flash                          |
 * | TEST_LED        | 998  | None                      | Toggles the onboard LED                                 |
 * | TEST_ECHO       | 999  | None                      | Displays command entered back to the user               |
 */



/**
 * @page Status_Codes Status Codes
 *
 * @section overview Overview
 * The following table describes the various status codes used in the system to indicate success or specific types of errors.
 *
 * @section status_table Status Codes Table
 * | Name                          | Code  | Description                                         |
 * |-------------------------------|-------|-----------------------------------------------------|
 * | STATUS_OK                     | 000   | Operation completed successfully.                  |
 * | STATUS_ERR_UART_OF            | 001   | UART Buffer Overflow error.                        |
 * | STATUS_ERR_NO_INDICATOR       | 002   | Instruction missing starting indicator.            |
 * | STATUS_ERR_NO_TERMINATOR      | 003   | Instruction missing terminating character.          |
 * | STATUS_ERR_INVALID_INSTRUCTION| 004   | Invalid instruction received.                      |
 * | STATUS_ERR_INSTRUCTION_OUT_OF_RANGE | 005 | Instruction code is out of the allowable range. |
 * | STATUS_ERR_TOO_MANY_INSTRUCTIONS | 006 | Too many instructions in a single command string. |
 * | STATUS_ERR_INVALID_ARG        | 007   | One or more arguments are invalid.                 |
 * | STATUS_ERR_INVALID_ARG_COUNT  | 008   | Argument count does not match the required number.  |
 * | STATUS_ERR_TOO_MANY_ARGS      | 009   | Too many arguments provided in the command.         |
 * | STATUS_ERR_ARG_OUT_OF_RANGE   | 010   | One or more arguments are outside allowable range.  |
 * | STATUS_ERR_INVALID_SETPOINT   | 011   | The given setpoint or profile does not contain data.|
 * | STATUS_ERR_PROFILE_FULL       | 012   | The profile is full and cannot accommodate more setpoints. |
 * | STATUS_ERR_EMPTY_PROFILE      | 013   | The profile contains no setpoints.                 |
 * | STATUS_ERR_FLASH_WRITE_FAILED | 014   | Flash write operation failed.                      |
 */



#pragma once

/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BAUD_RATE 38400
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define PWM_Y_Pin GPIO_PIN_0
#define PWM_Y_GPIO_Port GPIOA
#define PWM_X_Pin GPIO_PIN_1
#define PWM_X_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define ADXL_CS_Pin GPIO_PIN_10
#define ADXL_CS_GPIO_Port GPIOB
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define ADXL_SCL_Pin GPIO_PIN_3
#define ADXL_SCL_GPIO_Port GPIOB
#define ADXL_SDO_Pin GPIO_PIN_4
#define ADXL_SDO_GPIO_Port GPIOB
#define ADXL_SDA_Pin GPIO_PIN_5
#define ADXL_SDA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

// Binary Manipulation
#define RESERVED 0U
#define BIT_RESET_MASK 0x00U
#define BIT_0_MASK 0x01U
#define BIT_1_MASK 0x02U
#define BIT_2_MASK 0x04U
#define BIT_3_MASK 0x08U
#define BIT_4_MASK 0x10U
#define BIT_5_MASK 0x20U
#define BIT_6_MASK 0x40U
#define BIT_7_MASK 0x80U
#define FLASH_EMPTY 0xFFFFU

// User Input
#define INPUT_T_MAX 999U // Max possible instruction code value
#define PROFILE_LEN (INPUT_T_MAX + 1U) // 1000 setpoints per profile
#define INSTRUCTION_MAX_ARGS 4U // Maximum user-enterable parameters per instruction
#define TOTAL_PROFILES 11U
#define TOTAL_SETPOINTS (TOTAL_PROFILES * PROFILE_LEN)

// User-Facing Function Argument Limitations
#define PROFILE_ARG_MIN 0U
#define PROFILE_ARG_MAX 10U
#define INDEX_ARG_MIN 0U
#define INDEX_ARG_MAX 999U

// Flash Memory
#define FLASH_SECTOR_NUMBER FLASH_SECTOR_6
#define FLASH_SECTOR_6_BASE 0x08040000UL
#define PROFILE_ADDRESS(profile) ((setpoint_t*)(FLASH_SECTOR_6_BASE + (sizeof(setpoint_t) * PROFILE_LEN * profile)))
#define SETPOINT_ADDRESS(index, profile) ((setpoint_t*)(FLASH_SECTOR_6_BASE + (sizeof(setpoint_t) * index) + (sizeof(setpoint_t) * PROFILE_LEN * profile)))

// ADXL Register Map
#define ADXL_DEVID 0x00U // Device ID
#define ADXL_POWER_CTL 0x2DU // Power Mode Control
#define ADXL_DATA_FORMAT 0x31U // Data Format
#define ADXL_DATAX0 0x32U // Data Format

// ADXL
#define ADXL_ADDRESS_SIZE 1 // Size of the ADXL address in bytes
#define ADXL_DATA_SIZE 6 // Size of the ADXL data register in bytes
#define ADXL_TIMEOUT 100 // SPI Timeout in ms
#define ADXL_SCALE_FACTOR 0.00414

// UART
#define HUART_PTR &huart2
#define UART_TX_TIMEOUT 50 // in ms
#define UART_TX_BUF_LEN 128U
#define UART_RX_BUF_LEN (UART_TX_BUF_LEN - 6U) // 6 bytes is size of status code extension

// Servos
#define CCR_X htim2.Instance->CCR1
#define CCR_Y htim2.Instance->CCR2
#define ANGLE_MIN 0
#define ANGLE_MAX 180
#define ANGLE_INPUT_MIN 0
#define ANGLE_INPUT_MAX 180
#define ANGLE_RANGE (ANGLE_MAX - ANGLE_MIN)
#define SPEED_MIN 1U
#define SPEED_MAX 10U
#define STEP_DELAY 100U // ms
#define PROFILE_WAIT 500U //ms
#define TEST_PROFILE 10U 

#define PULSE_WIDTH_RANGE (PULSE_WIDTH_MAX - PULSE_WIDTH_MIN)
#define PULSE_WIDTH_MIN 500 // us
#define PULSE_WIDTH_NEG_90 (PULSE_WIDTH_0 - (PULSE_WIDTH_RANGE / 3))
#define PULSE_WIDTH_NEG_45 (PULSE_WIDTH_0 - (PULSE_WIDTH_RANGE / 6))
#define PULSE_WIDTH_0 ((PULSE_WIDTH_MAX + PULSE_WIDTH_MIN) / 2)
#define PULSE_WIDTH_POS_45 (PULSE_WIDTH_0 + (PULSE_WIDTH_RANGE / 6))
#define PULSE_WIDTH_POS_90 (PULSE_WIDTH_0 + (PULSE_WIDTH_RANGE / 3))
#define PULSE_WIDTH_MAX 2500 // us
#define PULSE_WIDTH_OFFSET_X -15 // us
#define PULSE_WIDTH_OFFSET_Y -25 // us
#define SERVO_TEST_DELAY 3000 // ms

// Misc
#define STARTUP_DELAY 2000

// #define TEST // Code within #ifdef TEST will only compile if TEST is defined

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

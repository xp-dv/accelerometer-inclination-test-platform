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
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define PWM_X_Pin GPIO_PIN_0
#define PWM_X_GPIO_Port GPIOA
#define PWM_Y_Pin GPIO_PIN_1
#define PWM_Y_GPIO_Port GPIOA
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
#define FLASH_USER_START_ADDR ((uint32_t)0x08040000)
#define SECTOR_NUMBER FLASH_SECTOR_6
/* USER CODE BEGIN Private defines */

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
#define FLASH_EMPTY 0xFFFF

#define CCR_X htim2.Instance->CCR1
#define CCR_Y htim2.Instance->CCR2

#define STARTUP_DELAY 2000

// ADXL Register Map
#define ADXL_DEVID 0x00U // Device ID
#define ADXL_POWER_CTL 0x2DU // Power Mode Control
#define ADXL_DATA_FORMAT 0x31U // Data Format
#define ADXL_DATAX0 0x32U // Data Format
// ADXL Macros
#define ADXL_ADDRESS_SIZE 1 // Size of the ADXL address in bytes
#define ADXL_DATA_SIZE 6 // Size of the ADXL data register in bytes
#define ADXL_TIMEOUT 100 // SPI Timeout in ms
#define ADXL_SCALE_FACTOR 0.00414

#define BAUD_RATE 115200
#define UART_TX_TIMEOUT 50 // in ms
#define UART_TX_BUF_LEN 128U
#define UART_RX_BUF_LEN (UART_TX_BUF_LEN - 6U) // 6 bytes is size of status code extension
#define INPUT_T_MAX 999U // Max possible instruction code value
#define MAX_LEN (INPUT_T_MAX + 1U) // Max possible instruction code value
#define INSTRUCTION_MAX_ARGS 4U // Maximum user-enterable parameters per instruction

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

// #define TEST // Code in #ifdef TEST will only compile if TEST is defined

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

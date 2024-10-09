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
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
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

#define BAUD_RATE 115200
#define CCR_X htim2.Instance->CCR1
#define CCR_Y htim2.Instance->CCR2
#define ADXL_SCALE_FACTOR 0.00414
#define SIZE 100
#define INSTRUCTION_TIMEOUT 5000 /* 5 sec */
#define RX_BUF_LEN 128
#define TX_BUF_LEN (RX_BUF_LEN + 128)

#define TEST

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
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

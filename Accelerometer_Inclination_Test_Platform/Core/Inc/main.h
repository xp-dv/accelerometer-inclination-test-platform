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
 * | STATUS_ERR_EMPTY_SETPOINT     | 011   | The given setpoint or profile does not contain data.|
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

//* Instruction Function Prototypes

/** 
 * @defgroup MovementCommands Movement Commands
 * Functions for immediate platform movement
 * @{
 */

/**
 * @brief Sets the system's active setpoint and transitions to the movement state.
 * 
 * @ingroup MovementCommands
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
status_code_t move(input_t x, input_t y, input_t speed);
/**
 * @brief Stops all active platform operations and transitions to the idle state.
 * 
 * @ingroup MovementCommands
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
status_code_t stop(void);
/**
 * @brief Cancels the current operation and transitions the device to the home state.
 * 
 * @ingroup MovementCommands
 * This function resets the active setpoint to its initial state and transitions the 
 * system to `HOME_STATE`, which just passes the idle state 
 * 
 * @return status_code_t 
 *         - STATUS_OK: If the cancel operation completes successfully.
 */
status_code_t cancel(void);
/**
 * @brief Configures the system to run a specific setpoint with a given profile.
 * 
 * @ingroup MovementCommands
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
status_code_t run_setpoint(input_t index, input_t profile);
/**
 * @brief Configures the system to execute a specific profile.
 * 
 * @ingroup MovementCommands
 * This function sets the active profile and transitions the system
 * to the `RUN_PROFILE_STATE`, preparing it for executing the associated setpoints.
 * 
 * @param profile The profile to be executed.
 * 
 * @return status_code_t 
 *         - STATUS_OK: If the profile is successfully set, and the system transitions to `RUN_PROFILE_STATE`.
 */
status_code_t run_profile(input_t profile);

/** @} */ // end of Movement Commands group

/** 
 * @defgroup EditProfileCommands Edit Profile Commands
 * Functions for the editing of profiles
 * @{
 */ 

/**
 * @brief Retrieves and transmits the setpoint data for a given index and profile.
 * 
 * @ingroup EditProfileCommands 
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
 *         - STATUS_ERR_EMPTY_SETPOINT: If the requested setpoint contains invalid data (all fields empty).
 */
status_code_t get_setpoint(input_t index, input_t profile);
/**
 * @brief Adds a new setpoint to the specified profile.
 * 
 * @ingroup EditProfileCommands 
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
status_code_t add_setpoint(input_t x_ang, input_t y_ang, input_t speed, input_t profile);
/**
 * @brief Removes a setpoint from the specified profile 
 * 
 * @ingroup EditProfileCommands 
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
 *         - STATUS_ERR_EMPTY_SETPOINT: If the specified setpoint is empty or invalid.
 *         - STATUS_ERR_FLASH_WRITE_FAILED: If there is an error while writing to flash memory.
 */
status_code_t remove_setpoint(input_t index, input_t profile);
/**
 * @brief Transmits all setpoints from a specified profile in packets of 10 setpoints or less until
 * all setpoints in that profile have been printed. The first number to print in a packet, indicated
 * by the instruction separator `'|'` is the decimal index of the first setpoint.
 * 
 * @ingroup EditProfileCommands 
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
status_code_t get_profile(input_t profile);

/** @} */ // end of Edit Profile Commands group

/**
 * @brief Clears all setpoints in the specified profile while preserving all other profiles in flash memory.
 * 
 * @ingroup EditProfileCommands 
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
status_code_t clear_profile(input_t profile);

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
status_code_t test_servos(void);
/**
 * @brief Tests the ADXL sensor by reading its angle data and transmitting it via UART. The first
 * number is the x-axis angle and the second number is the y-axis angle.
 * 
 * This function calls the `adxl_read` function to retrieve angle data from the ADXL sensor, formats the 
 * data into a string, and transmits it via UART. The transmitted string includes the X and Y angle values 
 * retrieved from the sensor.
 * 
 * @return status_code_t 
 *         - STATUS_OK: If the test is successful and the data is transmitted.
 */
status_code_t test_adxl(void);
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
status_code_t test_flash(void);
/**
 * @brief Toggles the onboard green LED. This LED is not visible unless the top of the case is removed.
 * @ingroup TestFunctions Test Functions
 * 
 * toggles the LD2 GPIO pin and echoes the UART respones
 * 
 * @return status_code_t 
 *         - STATUS_OK: If all operations for all profiles succeed.
 *         
 */
status_code_t test_led(void);

/** @} */ // end of TestFunctions group

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BAUD_RATE 38400
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
#define TOTAL_PROFILES 11U // User profiles 0-9 + test servo profile 10
#define TOTAL_SETPOINTS (TOTAL_PROFILES * PROFILE_LEN)

// Public Function Argument Limits
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
#define ADXL_REG_DEVID 0x00U // Device ID
#define ADXL_REG_POWER_CTL 0x2DU // Power Mode Control
#define ADXL_REG_DATA_FORMAT 0x31U // Data Format
#define ADXL_REG_DATAX0 0x32U // Data Format

// ADXL
#define ADXL_ADDRESS_SIZE 1 // Size of the ADXL address in bytes
#define ADXL_DATA_SIZE 6 // Size of the ADXL data register in bytes
#define ADXL_TIMEOUT 100 // SPI Timeout in ms
#define ADXL_SCALE_FACTOR 0.00414

// UART
#define HUART_PTR &huart2
#define UART_TX_TIMEOUT 50 // ms
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
#define PROFILE_WAIT 500U // ms
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

// Misc
#define STARTUP_DELAY 2000

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define P_14_8V_SENS_Pin GPIO_PIN_0
#define P_14_8V_SENS_GPIO_Port GPIOA
#define P_M1_DIR_Pin GPIO_PIN_1
#define P_M1_DIR_GPIO_Port GPIOA
#define P_M1_PWM_Pin GPIO_PIN_2
#define P_M1_PWM_GPIO_Port GPIOA
#define P_M1_FAULT_Pin GPIO_PIN_3
#define P_M1_FAULT_GPIO_Port GPIOA
#define P_M1_12V_SENS_Pin GPIO_PIN_4
#define P_M1_12V_SENS_GPIO_Port GPIOA
#define P_M2_12V_SENS_Pin GPIO_PIN_5
#define P_M2_12V_SENS_GPIO_Port GPIOA
#define P_M2_PWM_Pin GPIO_PIN_6
#define P_M2_PWM_GPIO_Port GPIOA
#define P_M2_DIR_Pin GPIO_PIN_7
#define P_M2_DIR_GPIO_Port GPIOA
#define P_M2_EN_Pin GPIO_PIN_0
#define P_M2_EN_GPIO_Port GPIOB
#define P_M2_FAULT_Pin GPIO_PIN_1
#define P_M2_FAULT_GPIO_Port GPIOB
#define P_WG_SCL_Pin GPIO_PIN_10
#define P_WG_SCL_GPIO_Port GPIOB
#define P_WG_SDA_Pin GPIO_PIN_11
#define P_WG_SDA_GPIO_Port GPIOB
#define P_EEPROM_SCK_Pin GPIO_PIN_13
#define P_EEPROM_SCK_GPIO_Port GPIOB
#define P_EEPROM_MISO_Pin GPIO_PIN_14
#define P_EEPROM_MISO_GPIO_Port GPIOB
#define P_EEPROM_MOSI_Pin GPIO_PIN_15
#define P_EEPROM_MOSI_GPIO_Port GPIOB
#define P_EEPROM_WP_Pin GPIO_PIN_8
#define P_EEPROM_WP_GPIO_Port GPIOA
#define P_UART_TX_Pin GPIO_PIN_9
#define P_UART_TX_GPIO_Port GPIOA
#define P_UART_RX_Pin GPIO_PIN_10
#define P_UART_RX_GPIO_Port GPIOA
#define P_DEBUG_SWDIO_Pin GPIO_PIN_13
#define P_DEBUG_SWDIO_GPIO_Port GPIOA
#define P_DEBUG_SWCLK_Pin GPIO_PIN_14
#define P_DEBUG_SWCLK_GPIO_Port GPIOA
#define P_SPARE_GPIO_1_Pin GPIO_PIN_15
#define P_SPARE_GPIO_1_GPIO_Port GPIOA
#define P_SPARE_GPIO_2_Pin GPIO_PIN_3
#define P_SPARE_GPIO_2_GPIO_Port GPIOB
#define P_SPARE_GPIO_3_Pin GPIO_PIN_4
#define P_SPARE_GPIO_3_GPIO_Port GPIOB
#define P_SPARE_GPIO_4_Pin GPIO_PIN_5
#define P_SPARE_GPIO_4_GPIO_Port GPIOB
#define P_MM_SCL_Pin GPIO_PIN_6
#define P_MM_SCL_GPIO_Port GPIOB
#define P_MM_SDA_Pin GPIO_PIN_7
#define P_MM_SDA_GPIO_Port GPIOB
#define P_SPARE_GPIO_5_Pin GPIO_PIN_8
#define P_SPARE_GPIO_5_GPIO_Port GPIOB
#define P_M1_EN_Pin GPIO_PIN_9
#define P_M1_EN_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

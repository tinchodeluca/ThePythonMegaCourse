/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
/**************************************
 * RTOS LIB
 **************************************/
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
/**************************************
 * EXTRA LIB
 **************************************/
#include "string.h"
/**************************************
 * PROTOTYPES AND DEFINITIOSN
 **************************************/
#include "df_states.h"
#include "df_queues.h"
/**************************************
 * TASKS
 **************************************/
#include "df_tasks.h"
/**************************************
 * FUNCTIONS
 **************************************/
#include "df_functions.h"
/**************************************
 * DRIVERS
 **************************************/
#include "ADS1292.h"
#include "MAX30102.h"
#include "max3010x_sparkfun.h"
#include "bluetooth.h"
#include "led_rgb.h"
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
#define LED_PILL_Pin GPIO_PIN_13
#define LED_PILL_GPIO_Port GPIOC
#define BTN_Pin GPIO_PIN_0
#define BTN_GPIO_Port GPIOA
#define LED_2_R_Pin GPIO_PIN_1
#define LED_2_R_GPIO_Port GPIOA
#define LED_2_G_Pin GPIO_PIN_2
#define LED_2_G_GPIO_Port GPIOA
#define LED_2_B_Pin GPIO_PIN_3
#define LED_2_B_GPIO_Port GPIOA
#define BAT_LVL_Pin GPIO_PIN_4
#define BAT_LVL_GPIO_Port GPIOA
#define BTH_STATE_Pin GPIO_PIN_5
#define BTH_STATE_GPIO_Port GPIOA
#define LED_1_R_Pin GPIO_PIN_6
#define LED_1_R_GPIO_Port GPIOA
#define LED_1_B_Pin GPIO_PIN_7
#define LED_1_B_GPIO_Port GPIOA
#define ADS_DRDY_Pin GPIO_PIN_0
#define ADS_DRDY_GPIO_Port GPIOB
#define ADS_DRDY_EXTI_IRQn EXTI0_IRQn
#define LED_1_G_Pin GPIO_PIN_1
#define LED_1_G_GPIO_Port GPIOB
#define SPI_CS_Pin GPIO_PIN_12
#define SPI_CS_GPIO_Port GPIOB
#define BAT_SENS_Pin GPIO_PIN_8
#define BAT_SENS_GPIO_Port GPIOA
#define BTH_KEY_Pin GPIO_PIN_9
#define BTH_KEY_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

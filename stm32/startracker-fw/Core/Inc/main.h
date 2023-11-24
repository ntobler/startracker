/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOC
#define TMC3_STEP_Pin GPIO_PIN_14
#define TMC3_STEP_GPIO_Port GPIOC
#define TMC3_DIR_Pin GPIO_PIN_15
#define TMC3_DIR_GPIO_Port GPIOC
#define POWER_ENABLE_Pin GPIO_PIN_0
#define POWER_ENABLE_GPIO_Port GPIOA
#define MOTOR_BOOST_ENABLE_Pin GPIO_PIN_3
#define MOTOR_BOOST_ENABLE_GPIO_Port GPIOA
#define CHARGER_DONE_Pin GPIO_PIN_4
#define CHARGER_DONE_GPIO_Port GPIOA
#define BUTTON_UP_Pin GPIO_PIN_7
#define BUTTON_UP_GPIO_Port GPIOA
#define BUTTON_RIGHT_Pin GPIO_PIN_0
#define BUTTON_RIGHT_GPIO_Port GPIOB
#define ADC_9_VSYS_Pin GPIO_PIN_1
#define ADC_9_VSYS_GPIO_Port GPIOB
#define LED_CLK_Pin GPIO_PIN_2
#define LED_CLK_GPIO_Port GPIOB
#define LED_DATA_Pin GPIO_PIN_10
#define LED_DATA_GPIO_Port GPIOB
#define IMU_INTERRUPT_Pin GPIO_PIN_12
#define IMU_INTERRUPT_GPIO_Port GPIOB
#define RPI_ENABLE_Pin GPIO_PIN_13
#define RPI_ENABLE_GPIO_Port GPIOB
#define CHARGER_CHARGING_Pin GPIO_PIN_14
#define CHARGER_CHARGING_GPIO_Port GPIOB
#define BUTTON_DOWN_Pin GPIO_PIN_9
#define BUTTON_DOWN_GPIO_Port GPIOA
#define BUTTON_LEFT_Pin GPIO_PIN_10
#define BUTTON_LEFT_GPIO_Port GPIOA
#define TMC_ENABLE_Pin GPIO_PIN_15
#define TMC_ENABLE_GPIO_Port GPIOA
#define TMC1_STEP_Pin GPIO_PIN_3
#define TMC1_STEP_GPIO_Port GPIOB
#define TMC1_DIR_Pin GPIO_PIN_5
#define TMC1_DIR_GPIO_Port GPIOB
#define TMC2_STEP_Pin GPIO_PIN_8
#define TMC2_STEP_GPIO_Port GPIOB
#define TMC2_DIR_Pin GPIO_PIN_9
#define TMC2_DIR_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

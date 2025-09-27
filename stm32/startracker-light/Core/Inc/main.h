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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define TP4_Pin GPIO_PIN_1
#define TP4_GPIO_Port GPIOA
#define TP3_Pin GPIO_PIN_2
#define TP3_GPIO_Port GPIOA
#define LED_VSYNC_Pin GPIO_PIN_3
#define LED_VSYNC_GPIO_Port GPIOA
#define SPI1_LED_CS_Pin GPIO_PIN_4
#define SPI1_LED_CS_GPIO_Port GPIOA
#define SPI1_LED_SCK_Pin GPIO_PIN_5
#define SPI1_LED_SCK_GPIO_Port GPIOA
#define SPI1_LED_MISO_Pin GPIO_PIN_6
#define SPI1_LED_MISO_GPIO_Port GPIOA
#define SPI1_LED_MOSI_Pin GPIO_PIN_7
#define SPI1_LED_MOSI_GPIO_Port GPIOA
#define RPI_ENABLE_Pin GPIO_PIN_12
#define RPI_ENABLE_GPIO_Port GPIOB
#define BUTTON_POWER_Pin GPIO_PIN_13
#define BUTTON_POWER_GPIO_Port GPIOB
#define BUTTON_POWER_EXTI_IRQn EXTI15_10_IRQn
#define TP2_Pin GPIO_PIN_14
#define TP2_GPIO_Port GPIOB
#define TP1_Pin GPIO_PIN_15
#define TP1_GPIO_Port GPIOB
#define SPI3_GYRO_SCK_Pin GPIO_PIN_3
#define SPI3_GYRO_SCK_GPIO_Port GPIOB
#define SPI3_GYRO_MISO_Pin GPIO_PIN_4
#define SPI3_GYRO_MISO_GPIO_Port GPIOB
#define SPI3_GYRO_MOSI_Pin GPIO_PIN_5
#define SPI3_GYRO_MOSI_GPIO_Port GPIOB
#define SPI3_GYRO_CS_Pin GPIO_PIN_6
#define SPI3_GYRO_CS_GPIO_Port GPIOB
#define GYRO_FSYNC_Pin GPIO_PIN_7
#define GYRO_FSYNC_GPIO_Port GPIOB
#define GYRO_INT2_Pin GPIO_PIN_8
#define GYRO_INT2_GPIO_Port GPIOB
#define GYRO_INT2_EXTI_IRQn EXTI9_5_IRQn
#define GYRO_INT1_Pin GPIO_PIN_9
#define GYRO_INT1_GPIO_Port GPIOB
#define GYRO_INT1_EXTI_IRQn EXTI9_5_IRQn

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

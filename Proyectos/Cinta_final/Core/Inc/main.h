/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#define Led_ON_Pin GPIO_PIN_5
#define Led_ON_GPIO_Port GPIOA
#define Led_OFF_Pin GPIO_PIN_6
#define Led_OFF_GPIO_Port GPIOA
#define RS485_DE_Pin GPIO_PIN_1
#define RS485_DE_GPIO_Port GPIOB
#define DC_IN1_Pin GPIO_PIN_12
#define DC_IN1_GPIO_Port GPIOB
#define DC_IN2_Pin GPIO_PIN_13
#define DC_IN2_GPIO_Port GPIOB
#define Sensor_1_Pin GPIO_PIN_3
#define Sensor_1_GPIO_Port GPIOB
#define Sensor_1_EXTI_IRQn EXTI3_IRQn
#define Sensor_2_Pin GPIO_PIN_4
#define Sensor_2_GPIO_Port GPIOB
#define Sensor_2_EXTI_IRQn EXTI4_IRQn
#define Sensor_3_Pin GPIO_PIN_5
#define Sensor_3_GPIO_Port GPIOB
#define Sensor_3_EXTI_IRQn EXTI9_5_IRQn
#define HX711_DT_Pin GPIO_PIN_8
#define HX711_DT_GPIO_Port GPIOB
#define HX711_SCK_Pin GPIO_PIN_9
#define HX711_SCK_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

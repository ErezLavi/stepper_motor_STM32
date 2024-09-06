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
#include "stm32g4xx_hal.h"

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
#define SERVO_1_Pin GPIO_PIN_0
#define SERVO_1_GPIO_Port GPIOA
#define SERVO_2_Pin GPIO_PIN_1
#define SERVO_2_GPIO_Port GPIOA
#define SERVO_3_Pin GPIO_PIN_2
#define SERVO_3_GPIO_Port GPIOA
#define SERVO_4_Pin GPIO_PIN_3
#define SERVO_4_GPIO_Port GPIOA
#define ECHO_2_Pin GPIO_PIN_4
#define ECHO_2_GPIO_Port GPIOA
#define ECHO_1_Pin GPIO_PIN_6
#define ECHO_1_GPIO_Port GPIOA
#define TRIG_Pin GPIO_PIN_7
#define TRIG_GPIO_Port GPIOA
#define ECHO_3_Pin GPIO_PIN_0
#define ECHO_3_GPIO_Port GPIOB
#define ECHO_4_Pin GPIO_PIN_1
#define ECHO_4_GPIO_Port GPIOB
#define FLT_1_Pin GPIO_PIN_2
#define FLT_1_GPIO_Port GPIOB
#define SERVO_5_Pin GPIO_PIN_8
#define SERVO_5_GPIO_Port GPIOA
#define SERVO_6_Pin GPIO_PIN_9
#define SERVO_6_GPIO_Port GPIOA
#define SERVO_7_Pin GPIO_PIN_10
#define SERVO_7_GPIO_Port GPIOA
#define FLT_2_Pin GPIO_PIN_3
#define FLT_2_GPIO_Port GPIOB
#define FLT_3_Pin GPIO_PIN_4
#define FLT_3_GPIO_Port GPIOB
#define FLT_4_Pin GPIO_PIN_5
#define FLT_4_GPIO_Port GPIOB
#define LED_Pin GPIO_PIN_7
#define LED_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

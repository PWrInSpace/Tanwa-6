/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BI_LED_Pin GPIO_PIN_13
#define BI_LED_GPIO_Port GPIOC
#define Sol1State_Pin GPIO_PIN_0
#define Sol1State_GPIO_Port GPIOA
#define Pressire1Measure_Pin GPIO_PIN_1
#define Pressire1Measure_GPIO_Port GPIOA
#define Sol2State_Pin GPIO_PIN_2
#define Sol2State_GPIO_Port GPIOA
#define M2OpenLimitSwitchEXT_Pin GPIO_PIN_3
#define M2OpenLimitSwitchEXT_GPIO_Port GPIOA
#define M2CloseLimitSwitchEXT_Pin GPIO_PIN_4
#define M2CloseLimitSwitchEXT_GPIO_Port GPIOA
#define M2Fault_Pin GPIO_PIN_5
#define M2Fault_GPIO_Port GPIOA
#define M2Dir_Pin GPIO_PIN_6
#define M2Dir_GPIO_Port GPIOA
#define M2PWM_Pin GPIO_PIN_7
#define M2PWM_GPIO_Port GPIOA
#define M1OpenLimitSwitchEXT_Pin GPIO_PIN_0
#define M1OpenLimitSwitchEXT_GPIO_Port GPIOB
#define M1CloseLimitSwitchEXT_Pin GPIO_PIN_1
#define M1CloseLimitSwitchEXT_GPIO_Port GPIOB
#define M1Fault_Pin GPIO_PIN_14
#define M1Fault_GPIO_Port GPIOB
#define M1Dir_Pin GPIO_PIN_15
#define M1Dir_GPIO_Port GPIOB
#define M1PWM_Pin GPIO_PIN_8
#define M1PWM_GPIO_Port GPIOA
#define Servo2PWM_Pin GPIO_PIN_6
#define Servo2PWM_GPIO_Port GPIOB
#define Sol2Dir_Pin GPIO_PIN_7
#define Sol2Dir_GPIO_Port GPIOB
#define Servo1PWM_Pin GPIO_PIN_8
#define Servo1PWM_GPIO_Port GPIOB
#define Sol1Dir_Pin GPIO_PIN_9
#define Sol1Dir_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

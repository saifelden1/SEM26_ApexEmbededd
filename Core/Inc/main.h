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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Throttle_Pin GPIO_PIN_0
#define Throttle_GPIO_Port GPIOA
#define Voltage_sensor_Pin GPIO_PIN_2
#define Voltage_sensor_GPIO_Port GPIOA
#define ACS812_Pin GPIO_PIN_4
#define ACS812_GPIO_Port GPIOA
#define WL_Pin GPIO_PIN_7
#define WL_GPIO_Port GPIOA
#define WH_Pin GPIO_PIN_0
#define WH_GPIO_Port GPIOB
#define UL_Pin GPIO_PIN_9
#define UL_GPIO_Port GPIOA
#define UH_Pin GPIO_PIN_10
#define UH_GPIO_Port GPIOA
#define hallA_Pin GPIO_PIN_3
#define hallA_GPIO_Port GPIOB
#define hallB_Pin GPIO_PIN_4
#define hallB_GPIO_Port GPIOB
#define hallC_Pin GPIO_PIN_5
#define hallC_GPIO_Port GPIOB
#define VL_Pin GPIO_PIN_6
#define VL_GPIO_Port GPIOB
#define VH_Pin GPIO_PIN_7
#define VH_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

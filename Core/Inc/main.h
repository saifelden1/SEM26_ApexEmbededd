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
void delay_us(uint32_t us);
long map(long x, long in_min, long in_max, long out_min, long out_max);
uint32_t Read_ADC(uint32_t channel);
void calculate_power();
void calculate_speed_and_distance();
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
#define CL_Pin GPIO_PIN_7
#define CL_GPIO_Port GPIOA
#define CH_Pin GPIO_PIN_0
#define CH_GPIO_Port GPIOB
#define AL_Pin GPIO_PIN_9
#define AL_GPIO_Port GPIOA
#define AH_Pin GPIO_PIN_10
#define AH_GPIO_Port GPIOA
#define HA_Pin GPIO_PIN_3
#define HA_GPIO_Port GPIOB
#define HB_Pin GPIO_PIN_4
#define HB_GPIO_Port GPIOB
#define HC_Pin GPIO_PIN_5
#define HC_GPIO_Port GPIOB
#define BL_Pin GPIO_PIN_6
#define BL_GPIO_Port GPIOB
#define BH_Pin GPIO_PIN_7
#define BH_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/*
 * Delay.h
 *
 *  Created on: Oct 18, 2025
 *      Author: 01226
 */

#ifndef SERVICE_INC_DELAY_H_
#define SERVICE_INC_DELAY_H_


#include "stm32f1xx_hal.h"  // STM32 HAL header (adjust this based on your STM32 series)

// Function prototypes
void DelayUs_init(TIM_HandleTypeDef *htim);     // Initialize delay module with timer
void DelayUs(uint32_t delay);                 // Generate delay in microseconds




#endif /* SERVICE_INC_DELAY_H_ */

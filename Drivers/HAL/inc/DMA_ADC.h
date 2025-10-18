/*
 * DMAADC.h
 *
 *  Created on: Oct 18, 2025
 *      Author: 01226
 */

#ifndef HAL_INC_DMA_ADC_H_
#define HAL_INC_DMA_ADC_H_



#include "stm32f1xx_hal.h"

// Define ADC buffer size (3 channels for current, voltage, and throttle)
#define DMA_ADC_Buffer_SIZE 3

// Declare the DMA buffer for storing ADC values
extern uint16_t DMA_ADC_Buffer[DMA_ADC_Buffer_SIZE];

// Function declarations
void DMA_ADC_Init(ADC_HandleTypeDef *hadc);  // Initialize ADC and DMA with passed ADC handle
void ADC_Stop(ADC_HandleTypeDef *hadc);      // Stop ADC with DMA using the passed ADC handle



#endif /* HAL_INC_DMA_ADC_H_ */

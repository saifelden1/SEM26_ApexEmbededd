/*
 * DMAADC.c
 *
 *  Created on: Oct 18, 2025
 *      Author: 01226
 */

#include "DMA_ADC.h"

// DMA buffer to hold 3 ADC values (one for each sensor)
uint16_t DMA_ADC_Buffer[DMA_ADC_Buffer_SIZE]={0,0,0};  // DMA buffer for current, voltage, and throttle sensors

// Initialize ADC and DMA (configured by CubeMX, we'll just start the ADC here)
void DMA_ADC_Init(ADC_HandleTypeDef *hadc) {
    // CubeMX will configure the ADC for 3 channels and DMA, we just need to start the ADC
    HAL_ADC_Start_DMA(hadc, (uint32_t*)DMA_ADC_Buffer, DMA_ADC_Buffer_SIZE);  // Start ADC with DMA (3 channels)
}


// Stop ADC with DMA (if needed to stop or restart)
void ADC_Stop(ADC_HandleTypeDef *hadc) {
    HAL_ADC_Stop_DMA(hadc);  // Stop ADC and DMA
}


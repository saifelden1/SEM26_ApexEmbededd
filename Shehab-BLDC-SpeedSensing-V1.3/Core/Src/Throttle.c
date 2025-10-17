/*
 * Throttle.c
 *
 *  Created on: Oct 17, 2025
 *      Author: sheha
 */

#include "Throttle.h"

uint16_t ADC_RawValue = 0;
uint16_t ADC_FilteredValue = 0;
uint16_t DutyCycle = 0;


static ADC_HandleTypeDef *ThrottleADC;  // Store ADC pointer


extern ADC_HandleTypeDef hadc1;   // استخدم الـADC المعرّف في main.c

// Initialize ADC with DMA
void Throttle_Init(ADC_HandleTypeDef *hadc)
{
    ThrottleADC = hadc;
    HAL_ADC_Start_DMA(ThrottleADC, (uint32_t *)&ADC_RawValue, 1);

}

//  Moving Average Filter
uint16_t Throttle_GetFilteredADC(uint16_t newValue)
{
    static uint16_t buffer[ADC_AVG_SAMPLES];
    static uint8_t index = 0;
    uint32_t sum = 0;

    buffer[index++] = newValue;

    if (index >= ADC_AVG_SAMPLES) index = 0;

    for (uint8_t i = 0; i < ADC_AVG_SAMPLES; i++)
        sum += buffer[i];

    return (uint16_t)(sum / ADC_AVG_SAMPLES);
}


void Throttle_Update(void)
{
    ADC_FilteredValue = Throttle_GetFilteredADC(ADC_RawValue);

    DutyCycle = (ADC_FilteredValue * MAX_PWM) / ADC_MAX_VALUE;

    if (DutyCycle > MAX_PWM)
        DutyCycle = MAX_PWM;
    else if (DutyCycle < MIN_PWM)
        DutyCycle = 0;
}


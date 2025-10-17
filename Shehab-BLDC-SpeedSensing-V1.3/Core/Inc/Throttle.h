/*
 * Throttle.h
 *
 *  Created on: Oct 17, 2025
 *      Author: sheha
 */

#ifndef INC_THROTTLE_H_
#define INC_THROTTLE_H_

#include "main.h"   // عشان نضمن تعريف HAL_ADC وغيره

#define FILTER_SIZE 10
#define MAX_PWM     98
#define MIN_PWM     5
#define ADC_AVG_SAMPLES 10  // Number of samples for smoothing
#define ADC_MAX_VALUE 4095  // For 12-bit ADC

extern uint16_t ADC_RawValue;
extern uint16_t ADC_FilteredValue;
extern uint16_t DutyCycle;


void Throttle_Init(ADC_HandleTypeDef *hadc);
void Throttle_Update(void);
uint16_t Throttle_GetFilteredADC(uint16_t newValue);

#endif /* INC_THROTTLE_H_ */

/*
 * Delay.c
 *
 *  Created on: Oct 18, 2025
 *      Author: 01226
 */


#include "Delay.h"

static TIM_HandleTypeDef *htim_delay;  // Pointer to timer handle for delay

// Function to initialize the delay module with a timer handle
void DelayUs_init(TIM_HandleTypeDef *htim) {
    htim_delay = htim;  // Store the timer handle
    HAL_TIM_Base_Start(htim_delay);  // Start the timer
}

// Function to generate a delay in microseconds
void DelayUs(uint32_t delay) {
    // Reset the timer counter
    __HAL_TIM_SET_COUNTER(htim_delay, 0);


    // Wait until the timer reaches the delay value
    while (__HAL_TIM_GET_COUNTER(htim_delay) < delay);
}

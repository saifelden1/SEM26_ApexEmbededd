/*
 * BLDC.h
 *
 *  Created on: Oct 18, 2025
 *      Author: 01226
 */

#ifndef APPLICATION_INC_BLDC_H_
#define APPLICATION_INC_BLDC_H_



#include "stm32f1xx_hal.h"
#include "hall_sensor.h"
#include "throttle.h"
#include "Delay.h"

// Declare timer handles as extern (defined elsewhere)
extern TIM_HandleTypeDef htim1;  // Timer for phase U
extern TIM_HandleTypeDef htim4;  // Timer for phase V
extern TIM_HandleTypeDef htim3;  // Timer for phase W

//the phase defines
// Phase U
#define UH_TIMER        &htim1
#define UH_CHANNEL      TIM_CHANNEL_3
#define UL_TIMER        &htim1
#define UL_CHANNEL      TIM_CHANNEL_2

// Phase V
#define VH_TIMER        &htim4
#define VH_CHANNEL      TIM_CHANNEL_2
#define VL_TIMER        &htim4
#define VL_CHANNEL      TIM_CHANNEL_1

// Phase W
#define WH_TIMER        &htim3
#define WH_CHANNEL      TIM_CHANNEL_3
#define WL_TIMER        &htim3
#define WL_CHANNEL      TIM_CHANNEL_2

// Define the PWM range for throttle (0 to 100%)
#define PWM_MIN           0
#define PWM_MAX           100

// External variables (defined in main or other parts of the program)
extern uint8_t HallSensor_Combined;  // Combined Hall sensor state (3 bits)
extern uint16_t duty;                // PWM duty cycle (for throttle control)
//extern uint16_t step;                // PWM duty cycle (for throttle control)
//extern uint16_t laststep;                // PWM duty cycle (for throttle control)

// Declare functions for BLDC control
void BLDC_Init(void);                // Initialize BLDC motor control
void BLDC_DecideStep(void);          // Function to decide the step based on Hall sensor state
void BLDC_Comutate(void);            // Function to perform commutation based on step and throttle



#endif /* APPLICATION_INC_BLDC_H_ */

/*
 * BLDC..c
 *
 *  Created on: Oct 18, 2025
 *      Author: 01226
 */


#include "BLDC.h"

// Global variable for commutation step (mapped from Hall sensor states)
uint8_t step = 0;  // Step for BLDC commutation
//uint8_t laststep = 0;  // Step for BLDC commutation

//the init function
void BLDC_Init(void) {
    // Start PWM for all phases
    HAL_TIM_PWM_Start(UH_TIMER, UH_CHANNEL);   // Start upper MOSFET for phase U
    HAL_TIM_PWM_Start(UL_TIMER, UL_CHANNEL);   // Start lower MOSFET for phase U

    HAL_TIM_PWM_Start(VH_TIMER, VH_CHANNEL);   // Start upper MOSFET for phase V
    HAL_TIM_PWM_Start(VL_TIMER, VL_CHANNEL);   // Start lower MOSFET for phase V

    HAL_TIM_PWM_Start(WH_TIMER, WH_CHANNEL);   // Start upper MOSFET for phase W
    HAL_TIM_PWM_Start(WL_TIMER, WL_CHANNEL);   // Start lower MOSFET for phase W
}

// Decide the commutation step based on Hall sensor combined state
void BLDC_DecideStep(void) {
	HallSensor_GetCombinedHallState();
    switch (HallSensor_Combined) {
    		case 0b110:  // Hall = 110
                step = 1;  // Step 1
                break;
            case 0b010:  // Hall = 011
                step = 2;  // Step 2
                break;
            case 0b011:  // Hall = 010
                step = 3;  // Step 3
                break;
            case 0b001:  // Hall = 110
                step = 4;  // Step 4
                break;
            case 0b101:  // Hall = 100
                step = 5;  // Step 5
                break;
            case 0b100:  // Hall = 101
                step = 6;  // Step 6
                break;
            default:
                step = 7;  // Default: no valid state
                break;
    }
//    laststep = step ;
}

// Perform commutation based on the step and throttle value
void BLDC_Comutate(void) {
    // Use the throttle value (mapped PWM duty cycle) for speed control
//cba
    switch (step) {
        case 1: // Hall = 110 /r
            __HAL_TIM_SET_COMPARE(VL_TIMER, VL_CHANNEL, 0);
            __HAL_TIM_SET_COMPARE(VH_TIMER, VH_CHANNEL, 0);
            __HAL_TIM_SET_COMPARE(UH_TIMER, UH_CHANNEL, 0);
            __HAL_TIM_SET_COMPARE(WL_TIMER, WL_CHANNEL, 0);
            DelayUs(2);  // Small delay for stabilization
            __HAL_TIM_SET_COMPARE(UL_TIMER, UL_CHANNEL, throttle.mappedValue);
            __HAL_TIM_SET_COMPARE(WH_TIMER, WH_CHANNEL, throttle.mappedValue);
            break;

        case 2: // Hall = 010 /r
            __HAL_TIM_SET_COMPARE(VL_TIMER, VL_CHANNEL, 0);
            __HAL_TIM_SET_COMPARE(UH_TIMER, UH_CHANNEL, 0);
            __HAL_TIM_SET_COMPARE(WL_TIMER, WL_CHANNEL, 0);
            __HAL_TIM_SET_COMPARE(WH_TIMER, WH_CHANNEL, 0);
            DelayUs(2);  // Small delay for stabilization
            __HAL_TIM_SET_COMPARE(VH_TIMER, VH_CHANNEL, throttle.mappedValue);
            __HAL_TIM_SET_COMPARE(UL_TIMER, UL_CHANNEL, throttle.mappedValue);
            break;

        case 3: // Hall = 011 /r
        	 __HAL_TIM_SET_COMPARE(VL_TIMER, VL_CHANNEL, 0);
        	 __HAL_TIM_SET_COMPARE(WH_TIMER, WH_CHANNEL, 0);
        	 __HAL_TIM_SET_COMPARE(UL_TIMER, UL_CHANNEL, 0);
        	 __HAL_TIM_SET_COMPARE(UH_TIMER, UH_CHANNEL, 0);
        	 DelayUs(2);  // Small delay for stabilization
        	 __HAL_TIM_SET_COMPARE(VH_TIMER, VH_CHANNEL, throttle.mappedValue);
        	 __HAL_TIM_SET_COMPARE(WL_TIMER, WL_CHANNEL, throttle.mappedValue);
        	   break;

        case 4: // Hall = 001	/r
            __HAL_TIM_SET_COMPARE(WH_TIMER, WH_CHANNEL, 0);
            __HAL_TIM_SET_COMPARE(VH_TIMER, VH_CHANNEL, 0);
            __HAL_TIM_SET_COMPARE(VL_TIMER, VL_CHANNEL, 0);
            __HAL_TIM_SET_COMPARE(UL_TIMER, UL_CHANNEL, 0);
            DelayUs(2);  // Small delay for stabilization
            __HAL_TIM_SET_COMPARE(UH_TIMER, UH_CHANNEL, throttle.mappedValue);
            __HAL_TIM_SET_COMPARE(WL_TIMER, WL_CHANNEL, throttle.mappedValue);
            break;

        case 5: // Hall = 101 /r
            __HAL_TIM_SET_COMPARE(UL_TIMER, UL_CHANNEL, 0);
            __HAL_TIM_SET_COMPARE(VH_TIMER, VH_CHANNEL, 0);
            __HAL_TIM_SET_COMPARE(WL_TIMER, WL_CHANNEL, 0);
            __HAL_TIM_SET_COMPARE(WH_TIMER,	WH_CHANNEL, 0);
            DelayUs(5);  // Small delay for stabilization
            __HAL_TIM_SET_COMPARE(UH_TIMER, UH_CHANNEL, throttle.mappedValue);
            __HAL_TIM_SET_COMPARE(VL_TIMER, VL_CHANNEL, throttle.mappedValue);
            break;

        case 6: // Hall = 100 /r
            __HAL_TIM_SET_COMPARE(UL_TIMER, UL_CHANNEL, 0);
            __HAL_TIM_SET_COMPARE(UH_TIMER,	UH_CHANNEL, 0);
            __HAL_TIM_SET_COMPARE(VH_TIMER, VH_CHANNEL, 0);
            __HAL_TIM_SET_COMPARE(WL_TIMER, WL_CHANNEL, 0);
            DelayUs(2);  // Small delay for stabilization
            __HAL_TIM_SET_COMPARE(WH_TIMER, WH_CHANNEL, throttle.mappedValue);
            __HAL_TIM_SET_COMPARE(VL_TIMER, VL_CHANNEL, throttle.mappedValue);
            break;

        case 7: // Hall = 110 or 111 (Idle or Full-On State)
            __HAL_TIM_SET_COMPARE(UH_TIMER, UH_CHANNEL, 0);
            __HAL_TIM_SET_COMPARE(UL_TIMER, UL_CHANNEL, 0);
            __HAL_TIM_SET_COMPARE(WH_TIMER,	WH_CHANNEL, 0);
            __HAL_TIM_SET_COMPARE(WL_TIMER,	WL_CHANNEL , 0);
            __HAL_TIM_SET_COMPARE(VH_TIMER, VH_CHANNEL, 0);
            __HAL_TIM_SET_COMPARE(VL_TIMER, VL_CHANNEL, 0);
            break;

        default:
            break; // Default case if no valid state is detected
    }
}

/*
 * Throttle.h
 *
 *  Created on: Oct 17, 2025
 *      Author: 01226
 */

#ifndef HAL_INC_THROTTLE_H_
#define HAL_INC_THROTTLE_H_



#include "stm32f1xx_hal.h"
#include "DMA_ADC.h"


// Define the PWM range for throttle (0 to 100%)
#define MAP_MIN                0
#define MAP_MAX                100
#define Throttle_Rank                0

// Declare the Throttle struct
typedef struct {
    uint16_t rawValue;         // The raw ADC value for the throttle
    uint8_t mappedValue;       // The mapped PWM value (0 to 100%)
    uint8_t channelRank;       // The ADC rank for the throttle in the DMA buffer (1, 2, or 3)
} Throttle_t;

// Declare a Throttle instance to hold throttle data
extern Throttle_t throttle;

// Function declarations
void Throttle_Init(void);                  // Initialize ADC and DMA for throttle
uint16_t Throttle_GetRaw(void)   ;              // Start ADC with DMA for throttle
uint16_t Throttle_Map(void);          // Get the mapped PWM throttle value



#endif /* HAL_INC_THROTTLE_H_ */

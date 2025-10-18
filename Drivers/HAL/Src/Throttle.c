/*
 * Throttle.c
 *
 *  Created on: Oct 17, 2025
 *      Author: 01226
 */



#include "throttle.h"


// Declare the throttle instance to store throttle data
Throttle_t throttle = {0, 0, Throttle_Rank};  // Initialize throttle struct with a default rank (e.g., 2 for ADC_CHANNEL_2)
void Throttle_Init(void){

}

// Get the raw ADC throttle value (value from ADC channel)
uint16_t Throttle_GetRaw(void) {
    // Access the DMA buffer based on the configured throttle rank
    // The throttle channel rank is stored in throttle.channelRank (1-based)
	throttle.rawValue = DMA_ADC_Buffer[throttle.channelRank ];
    return throttle.rawValue;  // Adjust the index
}

// Map the raw ADC value to a PWM value (0 to 100%)
uint16_t Throttle_Map(void) {
    // Assuming the raw ADC value is in the range [0, 4095] (12-bit ADC)
     Throttle_GetRaw();

    // Map the raw value (0-4095) to a PWM range (0-100%)
    throttle.mappedValue = ((throttle.rawValue) * (MAP_MAX - MAP_MIN)) / 4095;  // Linear mapping to PWM
    return throttle.mappedValue;
}




#include "throttle.h"

// Declare the throttle instance to store throttle data
Throttle_t throttle = {0, 0, Throttle_Rank};  // Initialize throttle struct with a default rank (e.g., 2 for ADC_CHANNEL_2)

void Throttle_Init(void){
    // Initialization code (if needed)
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
    // Assuming the raw ADC value is in the range [380, 2530] for the throttle
    Throttle_GetRaw();

    // Map the raw value (380-2530) to the PWM range (0-100%)
    // Linear mapping formula:
    throttle.mappedValue = ((throttle.rawValue - 340) * (MAP_MAX - MAP_MIN)) / (2500 - 300);

    // Ensure the mapped value is within the range of [0, 100]
    if (throttle.mappedValue < MAP_MIN) {
        throttle.mappedValue = MAP_MIN;
    } else if (throttle.mappedValue > MAP_MAX) {
        throttle.mappedValue = MAP_MAX;
    }

    return throttle.mappedValue;
}

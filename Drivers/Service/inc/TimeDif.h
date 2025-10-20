/*
 * TimeDif.h
 *
 *  Created on: Oct 20, 2025
 *      Author: 01226
 */

#ifndef SERVICE_INC_TIMEDIF_H_
#define SERVICE_INC_TIMEDIF_H_


#include "stm32f1xx_hal.h"

// TimDif structure to encapsulate internal variables
typedef struct {
    uint32_t last_time;  // Last time of Hall transition
    uint32_t last_diff;  // Time difference between two Hall transitions
} TimDif_t;

// Public functions for interacting with the TimDif module
void TimDif_Init(void);        // Initialize the time difference module
void TimDif_Capture(void);     // Capture the time difference at Hall transition
uint32_t TimDif_Get(void);     // Get the last captured time difference (in cycles)
TimDif_t* TimDif_GetState(void); // Optional: Get the full state (useful for debugging)


#endif /* SERVICE_INC_TIMEDIF_H_ */

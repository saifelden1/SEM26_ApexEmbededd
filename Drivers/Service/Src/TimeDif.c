/*
 * TimeDif.c
 *
 *  Created on: Oct 20, 2025
 *      Author: 01226
 */

#include "TimeDif.h"

// Static variable to store the time difference state
static TimDif_t timdif_state = {0, 0};  // Initialize to zero

// Initialize the DWT cycle counter
void TimDif_Init(void)
{
    // Enable the trace system for DWT
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;  // Enable trace
    DWT->CYCCNT = 0;  // Reset counter
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;  // Enable the DWT cycle counter

    // Initialize the time tracking variables
    timdif_state.last_time = 0;
    timdif_state.last_diff = 0;
}

// Capture the time difference between Hall transitions (use inside Hall interrupt)
void TimDif_Capture(void)
{
    uint32_t now = DWT->CYCCNT;
    timdif_state.last_diff = now - timdif_state.last_time;  // Handles overflow automatically
    timdif_state.last_time = now;
}

// Get the last captured time difference (in CPU cycles)
uint32_t TimDif_Get(void)
{
    return timdif_state.last_diff;
}

// Optional: Get the full state (useful for debugging)
TimDif_t* TimDif_GetState(void)
{
    return &timdif_state;
}

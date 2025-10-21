/*
 * SoftThrottle.c
 *
 *  Created on: Oct 21, 2025
 *      Author: 01226
 */


/*
 * SoftThrottle.c
 *
 *  Created on: Oct 21, 2025
 *      Author: 01226
 *
 *  Description:
 *  Implements smooth throttle ramping using DWT timer.
 */

#include "SoftThrottle.h"

// ===== Internal state =====

static uint32_t last_update_cyc = 0;  // last DWT timestamp

// ===== Initialization =====
void SoftThrottle_Init(void)
{
    DWT_Timer_Init();
    throttle.SoftValue = 0;
    last_update_cyc = DWT_Timer_GetCycles();
}

// ===== Reset instantly =====
void SoftThrottle_Reset(void)
{
	throttle.SoftValue = 0.0f;
}

// ===== Main update =====
// Should be called every few milliseconds (e.g., 5–10 ms)
void SoftThrottle_Update(void)
{
    // Get target from Throttle module (mapped value 0–100%)
	Throttle_Map();

    // Calculate elapsed time in ms since last update
    float dt_ms = DWT_Timer_Elapsed_ms(last_update_cyc);
    if (dt_ms < 10.0f) return;   // skip if too soon

   // last_update_cyc = DWT_Timer_GetCycles();

    // Compute allowed rate of change (% per ms)
    //float rate = MAP_MAX / (float)SOFTTHROTTLE_RAMP_TIME_MS;  // % per ms
    //float delta = rate * dt_ms;

    // Ensure a minimum ramp step
    //if (delta < SOFTTHROTTLE_MIN_STEP) delta = SOFTTHROTTLE_MIN_STEP;

    // Smoothly move current_output toward target_output
    if (throttle.mappedValue > throttle.SoftValue)
    {
    	throttle.SoftValue += SOFTTHROTTLE_STEP_UP;
        if (throttle.SoftValue > throttle.mappedValue)
        	throttle.SoftValue = throttle.mappedValue;
    }
    else if (throttle.mappedValue < throttle.SoftValue)
    {
    	throttle.SoftValue = throttle.mappedValue;
//    	throttle.SoftValue -= delta;
//        if (throttle.SoftValue < throttle.mappedValue)
//        	throttle.SoftValue = throttle.mappedValue;
    }

    // Clamp to valid range [0,100]
    if (throttle.SoftValue > 100.0f) throttle.SoftValue = 100.0f;
    if (throttle.SoftValue < 0.0f)   throttle.SoftValue = 0.0f;
}

// ===== Accessor =====
float SoftThrottle_GetOutput(void)
{
    return throttle.SoftValue;
}

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
    // Get the target from the Throttle module (mapped value 0–100%)
    Throttle_Map();

    // Calculate elapsed time in ms since last update
    uint32_t dt_ms = DWT_Timer_Elapsed_ms(last_update_cyc);

    // If the time difference is less than 10 ms, skip this update
    if (dt_ms < SOFTTHROTTLE_RAMP_TIME_MS) return;

    // Update last_update_cyc to the current cycle count for the next update comparison
    last_update_cyc = DWT_Timer_GetCycles();

    // Smoothly adjust SoftValue based on the mapped value
    if (throttle.mappedValue > throttle.SoftValue)
    {
        throttle.SoftValue += SOFTTHROTTLE_STEP_UP;  // Increase SoftValue
        if (throttle.SoftValue > throttle.mappedValue)
            throttle.SoftValue = throttle.mappedValue;
    }
    else if (throttle.mappedValue < throttle.SoftValue)
    {
//        throttle.SoftValue -= SOFTTHROTTLE_STEP_UP;  // Decrease SoftValue
//        if (throttle.SoftValue < throttle.mappedValue)
//            throttle.SoftValue = throttle.mappedValue;
    		throttle.SoftValue = throttle.mappedValue;//no ramp down its instant

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

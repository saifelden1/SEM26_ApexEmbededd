/*
 * SoftThrottle.h
 *
 *  Created on: Oct 21, 2025
 *      Author: 01226
 */

#ifndef SERVICE_INC_SOFTTHROTTLE_H_
#define SERVICE_INC_SOFTTHROTTLE_H_

/*
 * SoftThrottle.h
 *
 *  Created on: Oct 21, 2025
 *      Author: 01226
 *
 *  Description:
 *  Smoothly ramps throttle input (from Throttle module) using DWT timing.
 */

#pragma once
#include "stm32f1xx_hal.h"
#include "DWTTimer.h"
#include "throttle.h"


// ===== Configuration =====
#define SOFTTHROTTLE_RAMP_TIME_MS   50U   // time to go 0 → 100% throttle
#define SOFTTHROTTLE_MIN_STEP       1f   // minimum PWM change per update (%)
#define SOFTTHROTTLE_STEP_UP     2   // throttle increases by 2 every 5 ms
#define SOFTTHROTTLE_STEP_DOWN   3   // throttle decreases by 3 every 5 ms (faster release)

// ===== API =====
void     SoftThrottle_Init(void);                 // Initialize module
void     SoftThrottle_Update(void);               // Call periodically (e.g. every 5–10 ms)
float    SoftThrottle_GetOutput(void);            // Get current throttle output (%)
void     SoftThrottle_Reset(void);                // Reset ramp to zero instantly

#ifdef __cplusplus
}
#endif


#endif /* SERVICE_INC_SOFTTHROTTLE_H_ */

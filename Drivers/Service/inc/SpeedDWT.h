/*
 * SpeedDWT.h
 *
 *  Created on: Oct 20, 2025
 *      Author: 01226
 */

#ifndef SERVICE_INC_SPEEDDWT_H_
#define SERVICE_INC_SPEEDDWT_H_



#include "stm32f1xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

// ===== User Configuration =====
#define POLE_PAIRS          26
#define HALL_TRANSITIONS     (POLE_PAIRS * 6)
#define ALPHA               0.2f    // Filter smoothing factor (0–1)
#define SPEED_TIMEOUT_MS    200     // Stop timeout (ms)

// ===== Public API =====
void Speed_Init(void);              // Enable DWT counter
void Speed_CaptureDiff(void);       // Fast: capture time diff on step change
void Speed_Process(void);           // Slow: compute RPM + filter
float Speed_GetRPM(void);           // Get latest filtered RPM
bool  Speed_IsStopped(void);        // Check if motor stopped




#endif /* SERVICE_INC_SPEEDDWT_H_ */

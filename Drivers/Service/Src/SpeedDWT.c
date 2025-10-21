/*
 * SpeeddCal.s
 *
 *  Created on: Oct 20, 2025
 *      Author: 01226
 */


/*
 * Speed.c
 *
 *  Created on: Oct 21, 2025
 *  Author: 01226
 *
 *  BLDC Speed Measurement using DWT timer module.
 */

#include "SpeedDWT.h"

// ===== Internal state =====
static uint32_t last_time = 0;         // Previous DWT timestamp
static volatile uint32_t last_diff = 0; // Cycle difference between two Hall edges
static float rpm_filtered = 0.0f;
static uint32_t last_update_tick = 0;
static bool stopped = true;

// ===== Initialization =====
void Speed_Init(void)
{
    // Initialize DWT timing system
    DWT_Timer_Init();

    last_time = 0;
    last_diff = 0;
    rpm_filtered = 0.0f;
    stopped = true;
    last_update_tick = HAL_GetTick();
}

// ===== FAST PATH =====
// Called on every Hall transition (interrupt or commutation step)
void Speed_CaptureDiff(void)
{
    uint32_t now = DWT_Timer_GetCycles();

    if (last_time != 0)
        last_diff = now - last_time; // handle overflow automatically

    last_time = now;
    last_update_tick = HAL_GetTick();
    stopped = false;
}

// ===== SLOW PATH =====
// Called periodically (e.g. every 10–20 ms)
void Speed_Process(void)
{
    if (last_diff == 0) return; // skip until first valid sample

    // Compute instantaneous RPM
    float rpm_inst = (60.0f * (float)SystemCoreClock) /
                     ((float)HALL_TRANSITIONS * (float)last_diff);

    // Low-pass filter for smoothness
    rpm_filtered = (1.0f - ALPHA) * rpm_filtered + ALPHA * rpm_inst;

    // Stop detection: if no update for too long, set RPM = 0
    if ((HAL_GetTick() - last_update_tick) > SPEED_TIMEOUT_MS)
    {
        rpm_filtered = 0.0f;
        stopped = true;
    }
}

// ===== Accessors =====
float Speed_GetRPM(void)
{
    return rpm_filtered;
}

float Speed_GetRPS(void)
{
    return rpm_filtered / 60.0f;
}

bool Speed_IsStopped(void)
{
    return stopped;
}

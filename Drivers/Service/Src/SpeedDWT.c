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
static uint32_t transition_count = 0;   // Counter for Hall transitions
static uint32_t time = 0;          // Timestamp of the first Hall transition
static volatile uint32_t last_diff = 0;  // Cycle difference between two Hall edges
static float rpm_filtered = 0.0f;
static uint32_t last_update_tick = 0;
static bool stopped = true;
// ===== Initialization =====
void Speed_Init(void)
{
    // Initialize DWT timing system
    DWT_Timer_Init();

    time = DWT_Timer_GetCycles();
    last_diff = 0;
    rpm_filtered = 0.0f;
    stopped = true;
    last_update_tick = HAL_GetTick();
}

// ===== FAST PATH =====
// Called on every Hall transition (interrupt or commutation step)
// Called on every Hall transition (interrupt or commutation step)
void Speed_CaptureDiff(void)
{

    if (transition_count == 0) {
        // Record the timestamp for the first Hall transition
    	time = DWT_Timer_GetCycles();
    }

    transition_count++;  // Increment the transition counter

    if (transition_count == 10) {
        // Calculate the time difference between the first and the 10th transition
        uint32_t time_diff = DWT_Timer_Elapsed_ms(time);  // Time in cycles

        // Now you can calculate the RPM or handle time as needed
        float rpm_inst = (60.0f * (float)SystemCoreClock) / ((float)HALL_TRANSITIONS * (float)time_diff);

        // You could filter or handle the RPM here if needed

        // Reset the counter for the next batch of transitions
        transition_count = 0;
    }


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

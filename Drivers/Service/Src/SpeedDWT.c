/*
 * SpeeddCal.s
 *
 *  Created on: Oct 20, 2025
 *      Author: 01226
 */


#include "SpeedDWT.h"


// ===== Static variables =====
static uint32_t last_time = 0;
static volatile uint32_t last_diff = 0;   // raw time difference (cycles)
static float rpm_filtered = 0.0f;
static uint32_t last_update_tick = 0;
static bool stopped = true;

// ===== Helper macros =====
//#define DWT_CTRL_CYCCNTENA_Pos   0
//#define DWT_CTRL_CYCCNTENA_Msk   (1UL << DWT_CTRL_CYCCNTENA_Pos)


// ===== Initialization =====
void Speed_Init(void)
{
    // 1. Enable trace system
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

    // 2. Reset cycle counter
    DWT->CYCCNT = 0;

    // 3. Enable the DWT cycle counter (critical line!)
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    // Reset variables
    last_time = 0;
    last_diff = 0;
    rpm_filtered = 0.0f;
    stopped = true;
    last_update_tick = HAL_GetTick();
}

// ===== FAST PATH =====
// Called at each Hall step change (commutation event)
void Speed_CaptureDiff(void)
{
    uint32_t now = DWT->CYCCNT; //gets the current tick from the DWT
    last_diff = now - last_time;  // handles overflow automatically due to being unsigned
    last_time = now; //recrding now into last

    // update timestamp for timeout detection
    last_update_tick = HAL_GetTick();
    stopped = false;
}

// ===== SLOW PATH =====
// Called periodically in main loop (e.g. every few ms)
void Speed_Process(void)
{
    // If no new diff captured yet, skip for the first loop for example
    if (last_diff == 0) return;

    // Compute instantaneous RPM
    float rpm_inst = (60.0f * (float)SystemCoreClock) /
                     ((float)HALL_TRANSITIONS * (float)last_diff);

    // Simple low-pass filter for stability a filter to get 80% new and 20% for example for stability
    rpm_filtered = (1.0f - ALPHA) * rpm_filtered + ALPHA * rpm_inst;

    //missing speed calculations and adding a struct for encapulating all this data


    // Check stop condition if the last update tick isnot updated for certain time (no stpe change for certain time )
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

//bool Speed_IsStopped(void)
//{
//    return stopped;
//}

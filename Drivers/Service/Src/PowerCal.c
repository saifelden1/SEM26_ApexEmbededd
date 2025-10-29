///*
// * PowerCal.c
// *
// *  Created on: Oct 29, 2025
// *      Author: 01226
// */
//
//
//#include "PowerCal.h"
//#include "main.h"
//#include "DWTTimer.h"  // Include the DWT timer header
//
//extern float Power;
//extern float Energy_Wh;
//extern float battery_percentage;
//
//#define BATTERY_VOLTAGE_MAX 54.0f
//#define BATTERY_VOLTAGE_NOMINAL 48.0f
//#define BATTERY_VOLTAGE_MIN 42.0f
//#define UPDATE_PERIOD_MS 5  // Set the update period to 5 milliseconds
//
//static uint32_t last_update_cyc = 0;  // Store the cycle count of the last update
//
//void PowerMonitor_Init(void) {
//    // Initialize power and energy
//    Power = VoltageSensor_bV * CurrentSensor_bC;
//    Energy_Wh = 0.0f;
//    battery_percentage = 0.0f;
//
//    // Initialize the DWT cycle counter
//    last_update_cyc = DWT_Timer_GetCycles();  // Capture the starting cycle count
//}
//
//void PowerMonitor_Update(void) {
//    // Calculate elapsed time in ms since last update using DWT_Timer_Elapsed_ms
//    uint32_t dt_ms = DWT_Timer_Elapsed_ms(last_update_cyc);
//
//    // If the elapsed time is less than the desired update period (5 ms), skip the update
//    if (dt_ms < UPDATE_PERIOD_MS) return;
//
//    // Power calculation (Voltage * Current)
//    Power = VoltageSensor_bV * CurrentSensor_bC;
//
//    // Calculate the elapsed time in milliseconds for energy accumulation
//    float delta_ms = dt_ms;  // Elapsed time in ms
//
//    // Convert the time to hours for energy calculation
//    float delta_h = delta_ms / 1000.0f / 3600.0f;  // Convert ms to hours
//
//    // Accumulate energy (in Wh)
//    Energy_Wh += Power * delta_h;
//
//    // Battery percentage calculation based on voltage
//    if (VoltageSensor_bV >= BATTERY_VOLTAGE_MAX) {
//        battery_percentage = 100.0f;
//    } else if (VoltageSensor_bV <= BATTERY_VOLTAGE_MIN) {
//        battery_percentage = 0.0f;
//    } else {
//        battery_percentage = ((VoltageSensor_bV - BATTERY_VOLTAGE_MIN) /
//                              (BATTERY_VOLTAGE_MAX - BATTERY_VOLTAGE_MIN)) * 100.0f;
//    }
//
//    // Update last_update_cyc to the current cycle count for the next update
//    last_update_cyc = DWT_Timer_GetCycles();
//}
//
//float PowerMonitor_GetEnergy(void) {
//    return Energy_Wh;
//}
//
//float PowerMonitor_GetBatteryPercentage(void) {
//    return battery_percentage;
//}

/*
 * SpeedCal.h
 *
 *  Created on: Oct 18, 2025
 *      Author: 01226
 */

#ifndef SERVICE_INC_SPEEDCAL_H_
#define SERVICE_INC_SPEEDCAL_H_



#include "main.h"

// Define the Speed struct to store speed-related parameters
typedef struct {
    unsigned long rpm;          // RPM (Revolutions Per Minute)
    unsigned long distance;     // Distance covered (in meters, for future use)
    unsigned long revolutionTime; // Time for one full revolution (for speed calculation)
    unsigned long stepCount;    // Counts steps to complete one revolution
    unsigned long lastUpdateTime; // Last update time for speed calculation
} Speed;

// Function prototypes
void SpeedSensing_Init(void);           // Initialize the speed sensing module
void SpeedSensing_HandleTransition(void); // Handle step transition and calculate speed
unsigned long SpeedSensing_GetRPM(void); // Get the current RPM from the speed struct
unsigned long SpeedSensing_GetDistance(void); // Get the current distance (for future use)

// External Speed struct instance
extern Speed motorSpeed;



#endif /* SERVICE_INC_SPEEDCAL_H_ */

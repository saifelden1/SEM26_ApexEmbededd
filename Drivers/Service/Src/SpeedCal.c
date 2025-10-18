/*
 * SpeedCal.c
 *
 *  Created on: Oct 18, 2025
 *      Author: 01226
 */

#include "SpeedCal.h"
#include "hall_sensor.h"  // For step detection

// Define the Speed struct instance
Speed motorSpeed = {0};  // Initialize the Speed struct with zero values
unsigned long previousTime = 0; // Time of the last transition
unsigned long currentTime  =  0; // Current time in milliseconds
unsigned long timeDifference = 0; // Calculate the time difference

// Initialize the speed sensing module
void SpeedSensing_Init(void) {
    motorSpeed.rpm = 0;          // Initialize RPM to 0
    motorSpeed.distance = 0;     // Initialize distance to 0
    motorSpeed.revolutionTime = 0; // Initialize revolution time to 0
    motorSpeed.stepCount = 0;    // Initialize step count to 0
    motorSpeed.lastUpdateTime = HAL_GetTick(); // Initialize the last update time
}

// This function is called to handle step transitions and calculate speed
void SpeedSensing_HandleTransition(void) {
     previousTime = 0; // Time of the last transition
     currentTime = HAL_GetTick(); // Current time in milliseconds
     timeDifference = currentTime - previousTime; // Calculate the time difference

    // Update the previous time for the next transition
    previousTime = currentTime;

    // Accumulate revolution time and step count
    motorSpeed.revolutionTime += timeDifference;
    motorSpeed.stepCount++;

    // If one full revolution (26 steps) is completed, calculate RPM
    if (motorSpeed.stepCount >= 26) {  // One full revolution (26 steps)
        motorSpeed.rpm = 60000 / motorSpeed.revolutionTime;  // Convert time to RPM
        motorSpeed.revolutionTime = 0;  // Reset revolution time for the next cycle
        motorSpeed.stepCount = 0;  // Reset step count for the next cycle

        // For future use: calculate distance (e.g., if you have the wheel circumference)
        // motorSpeed.distance += (motorSpeed.rpm * wheelCircumference) / 60;
    }

    // Store the time of the last update
    motorSpeed.lastUpdateTime = currentTime;
}

// Get the current RPM
unsigned long SpeedSensing_GetRPM(void) {
    return motorSpeed.rpm;
}

// Get the current distance (for future use)
unsigned long SpeedSensing_GetDistance(void) {
    return motorSpeed.distance;
}


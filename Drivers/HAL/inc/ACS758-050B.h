/*
 * ACS712.h
 *
 *  Created on: Oct 18, 2025
 *      Author: 01226
 */

#ifndef HAL_INC_ACS712_H_
#define HAL_INC_ACS712_H_

#include "stm32f1xx_hal.h"

// Define constants for the current sensor (ACS758-050B)
#define SENSOR_OFFSET_VOLTAGE 2.5f    // Offset voltage (2.5V at 0A)
#define SENSOR_SENSITIVITY 0.04f     // Sensitivity (40mV/A or 0.04V/A)
#define ADC_MAX_VOLTAGE 3.3f         // Max voltage that the ADC reads (3.3V after voltage divider)
#define ADC_RESOLUTION 4095          // 12-bit ADC resolution
#define VOLTAGE_DIVIDER_SCALING 0.66f // Voltage divider scaling factor (3.3V/5V)
#define ACS758_sensor_Rank 0 // Voltage divider scaling factor (3.3V/5V)

// Declare the global variable to hold the current measurement
extern float CurrentSensor_bC;  // The actual current reading (in Amperes)

// Function declarations
void CurrentSensor_Init(void);   // Initialize the current sensor (ADC setup)
float CurrentSensor_Read(void);   // Read the current sensor ADC value and convert it to current



#endif /* HAL_INC_ACS712_H_ */

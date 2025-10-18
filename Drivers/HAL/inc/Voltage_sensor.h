/*
 * Voltage_sensor.h
 *
 *  Created on: Oct 18, 2025
 *      Author: 01226
 */

#ifndef HAL_INC_VOLTAGE_SENSOR_H_
#define HAL_INC_VOLTAGE_SENSOR_H_

#include "stm32f1xx_hal.h"



// Define constants for the voltage divider
#define R1 2000000.0f  // Resistor R1 = 2MΩ
#define R2 110000.0f   // Resistor R2 = 110kΩ
#define ADC_MAX_VOLTAGE_V 2.815f // Maximum ADC voltage (2.815V)

// Define the rank of the voltage sensor in the DMA buffer
#define Voltage_sensor_Rank 1  // Change this to the correct rank (e.g., 0, 1, or 2)

// Declare the global variable to hold the battery voltage
extern float VoltageSensor_bV;  // The actual battery voltage

// Function declaration
void VoltageSensor_Init(void);
float VoltageSensor_Read(void);        // Read the voltage sensor ADC value and convert to battery voltage




#endif /* HAL_INC_VOLTAGE_SENSOR_H_ */

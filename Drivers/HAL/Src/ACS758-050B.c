/*
 * ACS712.h
 *
 *  Created on: Oct 18, 2025
 *      Author: 01226
 */
#include "ACS758-050B.h"
#include "DMA_ADC.h"


// Global variable to store the current reading
float CurrentSensor_bC = 0.0f;  // Initialize current reading to 0

// Initialize the current sensor (ADC setup)
void CurrentSensor_Init(void) {
    // Initialization is handled by CubeMX or other configuration code.
    // This function is left empty for now, and you can add any additional initialization here if needed.
}

// Read the current sensor ADC value from the DMA buffer and convert it to current
float CurrentSensor_Read(void) {
    // Get the ADC value for the current sensor from the DMA buffer
    uint16_t adcValue = DMA_ADC_Buffer[ACS758_sensor_Rank];  // Assuming the current sensor is in the first channel (index 0)

    // Convert ADC value to output voltage (range 0 to 3.3V)
    float Vout = ((float)adcValue / ADC_RESOLUTION) * ADC_MAX_VOLTAGE;

    // Scale back to the original sensor voltage (before the voltage divider)
    float Vsensor = Vout / VOLTAGE_DIVIDER_SCALING;

    // Calculate the current using the formula:
    // I_current = (V_sensor - 2.5V) / 0.04V
    CurrentSensor_bC = (Vsensor - SENSOR_OFFSET_VOLTAGE) / SENSOR_SENSITIVITY;
    return CurrentSensor_bC;
}



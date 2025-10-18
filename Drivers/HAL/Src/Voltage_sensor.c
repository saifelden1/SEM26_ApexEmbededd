/*
 * voltage_sensor.c
 *
 *  Created on: Oct 18, 2025
 *      Author: 01226
 */
#include "Voltage_sensor.h"
#include "DMA_ADC.h"



// Global variable to hold the battery voltage
float VoltageSensor_bV = 0.0f;  // Initialize battery voltage to 0

void VoltageSensor_Init(void){

}

// Read the voltage sensor ADC value from the DMA buffer and convert it to battery voltage
float VoltageSensor_Read(void) {

    // Convert ADC value to voltage (range 0 to 2.815V)
    float Vout = (((float)DMA_ADC_Buffer[Voltage_sensor_Rank]) / 4095) * ADC_MAX_VOLTAGE_V;

    // Calculate the real battery voltage using the inverse of the voltage divider formula
    // V_in = V_out * (R1 + R2) / R2
    VoltageSensor_bV = Vout * ((R1 + R2) / R2);
    return VoltageSensor_bV;
}

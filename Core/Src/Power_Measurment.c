/*
 * Power_Measurment.c
 *
 *  Created on: Aug 26, 2024
 *      Author: FM
 */
#include "main.h"
#include "Power_Measurment.h"


float Current(){
	  uint32_t adc_value1 = Read_ADC(ADC_CHANNEL_0); //  Current Measurement
	  float A_voltage = (adc_value1 / 4095.0) * 3.3;
	  current = A_voltage-2.5 / 0.04;



}
float Voltage(){
	uint32_t adc_value2 = Read_ADC(ADC_CHANNEL_1); // (voltage measurement)
	voltage = (adc_value2 / 4095.0) * (R1+R2/R2)*3.3;

}
void calculate_power(){
	Voltage();
	Current();
	power = voltage * current;
}

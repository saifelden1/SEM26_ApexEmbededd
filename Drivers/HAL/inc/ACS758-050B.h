/**
  ******************************************************************************
  * @file           : <ACS758>.h
  * @brief          : <*the current sensor ACS758 driver * >
  ******************************************************************************
  * @attention
  *
  * Copyright (c) <2025>, <apex racing team>.
  * All rights reserved.
  *
  * This software is provided by the efforts of the ApexRacingTeamEV @HTi university
  *
  * You may contact us at https://www.facebook.com/ApexRTe
  *
  *
  *
  ******************************************************************************
  * @version        : V1.0.0
  * @date           : <18/10/2025>
  * @author         : <saifelden mahmoud >
  *
  * @version        : V1.0.0
  * @date           : <17/10/2025>
  * @author         : <saifelden mahmoud >
  ******************************************************************************
  */
/*---------------------------------How to use this driver ----------------------*/
/*
 * this driver is intended to be used with the ACS current sensor throw the adc in dma mode throw the buffer
 * its needed to use the DMA_ADC.h driver with this driver since the buffer and the dma is configuerd ther
 *
 * NOTE:need to implement if defs to support mode selection wehter intrupt adc or dma mode in future
 *
 *  ------------------------------what to confuigure before use-----------------
 * you will need to enable the adc chaneel first and settingits rank and make the dma in the circular modeif there is multichannels
 * (all spicified in the dma adc driver )
 * sensor working theroy
 * -50 to 0 ->0 to 2.5 volts
 * 0 to 50 ->2.5 to 5 volts
 * operates on 5 volts and must config the sensitivty and the offset and the adc max voltage if needed
 * also if used a voltage devider add its ration
 *
 *
 * */

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

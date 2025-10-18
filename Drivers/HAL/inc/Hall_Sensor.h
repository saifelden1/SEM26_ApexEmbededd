/**
  ******************************************************************************
  * @file           : <Hall_Sensor>.ch
  * @brief          : <*Hall sensor driver for a bldc motor * >
  ******************************************************************************
  * @attention
  *
  * Copyright (c) <year>, <Your Name or Organization>.
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
  * @date           : <17/10/2025>
  * @author         : <saifelden mahmoud >
  *
  * @version        : V1.0.0
  * @date           : <17/10/2025>
  * @author         : <saifelden mahmoud >
  ******************************************************************************
  */
/*---------------------------------How to use this driver ----------------------*/
/*
 * this driver in intended to be used with the hall sensors attached to a bldc motor spicificaly for the three hallsensors setup
 * you can edit the structs in the .h file and increase their number but also you will need to modify the return combined function
 *  because its still till V1.0 does not have the ability to dynamicaly incerease the number of the hall sensors
 *
 *  ------------------------------what to confuigure before use-----------------
 *  you will need to configure three gpio input pins in normal mode (no intrupt mode yet)
 *  then go to the .h and spiciy the port and the pins of the three hall sensors and their order
 *
 *	you have three instances of sturcts of Hallsensor data type arr init in the .c file with names
 *	hallA , hallB , hallC
 *
 *
 * */
#ifndef HAL_INC_HALL_SENSOR_H_
#define HAL_INC_HALL_SENSOR_H_

#include "stm32f1xx_hal.h"

// Define the number of iterations for the filter
#define FILTER_ITERATIONS 5

// Define struct for Hall sensor
typedef struct {
    GPIO_PinState state;        // Current state of the Hall sensor (raw state)
    GPIO_PinState filteredState; // Filtered state after applying the filter
    uint16_t pin;               // GPIO pin for the Hall sensor (GPIO_PIN_3, GPIO_PIN_4, etc.)
    GPIO_TypeDef* port;        // GPIO port for the Hall sensor (GPIOB, GPIOC, etc.)
} HallSensor_t;

// Global variable to store the combined filtered value
extern uint8_t HallSensor_Combined;
//Hall sensor instanc
extern HallSensor_t hallA;   // Hall A
extern HallSensor_t hallB;   // Hall B
extern HallSensor_t hallC;   // Hall C
// Function declarations
void HallSensor_Init(void);                      // Initialize Hall sensor GPIO pins
uint8_t HallSensor_Read(HallSensor_t *sensor); // Read Hall sensor by pin
uint8_t HallSensor_ReadWithFilter(HallSensor_t *sensor, uint8_t iterations);  // Read sensor with filtering
uint8_t HallSensor_GetCombinedHallState(void);

#endif /* HAL_INC_HALL_SENSOR_H_ */

/*
 * Measurment.h
 *
 *  Created on: Aug 26, 2024
 *      Author: FM
 */

#ifndef INC_SPEED_DISTANCE_MEASURMENT_H_
#define INC_SPEED_DISTANCE_MEASURMENT_H_

#include "main.h"

float Speed = 0;
float Distance = 0;
float rpm = 0;
extern uint32_t time_interval;

extern float time_interval_minutes;
#define PULSES_PER_REVOLUTION 32
#define WHEEL_RADIUS 0.3
#define PI  3.14

#endif /* INC_SPEED_DISTANCE_MEASURMENT_H_ */

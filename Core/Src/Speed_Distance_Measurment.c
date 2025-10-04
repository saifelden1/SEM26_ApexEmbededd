/*
 * Measurment.c
 *
 *  Created on: Aug 26, 2024
 *      Author: FM
 */

#include <Speed_Distance_Measurment.h>
void calculate_speed_and_distance(){
  if (time_interval > 0) {

      time_interval_minutes = time_interval / 60000.0; // Convert milliseconds to minutes

	  // Calculate speed (rotations per minute)
	  rpm =  PULSES_PER_REVOLUTION / time_interval_minutes;
	  Speed = (rpm * 2 * PI * WHEEL_RADIUS) / 60; //speed in m/s

	  // Calculate distance (accumulated distance)
	  Distance += Speed * time_interval_minutes *60;
  }
}

/*
 * SpeedModule.h
 *
 *  Created on: Oct 6, 2025
 *      Author: sheha
 */

#ifndef INC_SPEEDMODULE_H_
#define INC_SPEEDMODULE_H_


#define PPR 			156//Pulses per revolution=ElectricChange*NPoles(Rising and Falling)
#define NPoles 			26//Number Of poles
#define ElectricChange	6
#define WheelRadius 	0.34
#define PI  			3.14159f
#define TimeInterval	0.02f //200ms to for each update of the Speed Readings


void SpeedCalculation(float * SpeedRpm,float * SpeedKmh);/*Pass The Variable address to get current
speed in RPM and Kmh
*/
#endif /* INC_SPEEDMODULE_H_ */

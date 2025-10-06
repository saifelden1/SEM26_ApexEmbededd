/*
 * SpeedModule.c
 *
 *  Created on: Oct 6, 2025
 *      Author: sheha
 */
#include <stdint.h>
#include "SpeedModule.h"

uint32_t PulsesPerInterval=0;//Number of pulses for the time interval(updated for each step change)

float 	FrequencyOfPulses=0; //The Frequency of the pulsese in Hz

void SpeedCalculation(float * SpeedRpm,float * SpeedKmh)/*Pass The Variable address
to get current speed in RPM and Kmh
*/
{
	FrequencyOfPulses=((float)PulsesPerInterval/(float)IntervalTime);//Calculate the pulses/second (Hz)

	(*SpeedRpm)=((60.0f *FrequencyOfPulses)/(float)PPR);//Calculate the speed in RPM
	(*SpeedKmh)=(((float)(2*PI*WheelRadius*(*SpeedRpm))/60.0f)*3.6);/*3.6 is for
	( / 60*60 but its in denominator so it subtract to numerator) to convert from hour to hour
	and divide by 1000 from m to km
*/
}

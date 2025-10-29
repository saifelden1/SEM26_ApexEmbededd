/*
 * PowerCal.h
 *
 *  Created on: Oct 18, 2025
 *      Author: 01226
 */

#ifndef SERVICE_INC_POWERCAL_H_
#define SERVICE_INC_POWERCAL_H_

#include "Voltage_sensor.h"
#include "ACS758-050B.h"
#include "stm32f1xx_hal.h"


typedef struct {
    float voltage;
    float current;
    float power;
    float energy_Wh;
    float battery_percentage;
} PowerData_t;

extern float Power;
extern float Energy_wh;
extern float battery_percentage;
// Function prototypes
void PowerMonitor_Init(void);
void PowerMonitor_Update(void);
float PowerMonitor_GetEnergy(void);
float PowerMonitor_GetBatteryPercentage(void);


#endif /* SERVICE_INC_POWERCAL_H_ */

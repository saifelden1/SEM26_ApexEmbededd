/*
 * DWTTimer.h
 *
 *  Created on: Oct 21, 2025
 *      Author: 01226
 */
/*
 * Speed.h
 *
 * Measures BLDC motor speed (RPM) using the DWT cycle counter.
 * Depends on DWTTimer.c / DWTTimer.h for timing.
 */

/*
 * DWTTimer.h
 *
 *  Created on: Oct 21, 2025
 *      Author: 01226
 *
 *  Description:
 *  Provides access to the Cortex-M DWT (Data Watchpoint and Trace) cycle counter
 *  for precise time measurements in microseconds or milliseconds.
 *
 *  Works with STM32F1 series (e.g., Blue Pill) and other Cortex-M3/M4/M7 devices.
 */
#ifndef HAL_INC_DWTTIMER_H_
#define HAL_INC_DWTTIMER_H_
#include "stm32f1xx_hal.h"


// ===== API =====

/**
 * @brief Initialize and enable the DWT cycle counter.
 *        Must be called once before using any other DWT functions.
 */
void DWT_Timer_Init(void);

/**
 * @brief Read the current DWT cycle counter value.
 * @return Current CPU cycle count (increments every clock cycle)
 */
uint32_t DWT_Timer_GetCycles(void);

/**
 * @brief Compute elapsed time since a previous timestamp in microseconds.
 * @param start_cyc  The DWT cycle counter value captured earlier.
 * @return Elapsed time in microseconds as a float.
 */
float DWT_Timer_Elapsed_us(uint32_t start_cyc);

/**
 * @brief Compute elapsed time since a previous timestamp in milliseconds.
 * @param start_cyc  The DWT cycle counter value captured earlier.
 * @return Elapsed time in milliseconds as a float.
 */
float DWT_Timer_Elapsed_ms(uint32_t start_cyc);







#endif /* HAL_INC_DWTTIMER_H_ */

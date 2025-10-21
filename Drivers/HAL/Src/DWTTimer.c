/*
 * DWTTimer.c
 *
 *  Created on: Oct 21, 2025
 *      Author: 01226
 */


#include "DWTTimer.h"

void DWT_Timer_Init(void)
{
    // Enable TRC
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

    // Reset cycle counter
    DWT->CYCCNT = 0;

    // Enable the counter
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

uint32_t DWT_Timer_GetCycles(void)
{
    return DWT->CYCCNT;
}

// Return elapsed time since start_cyc in microseconds
float DWT_Timer_Elapsed_us(uint32_t start_cyc)
{
    return (float)(DWT->CYCCNT - start_cyc) / (SystemCoreClock / 1e6f);
}

// Return elapsed time since start_cyc in milliseconds
float DWT_Timer_Elapsed_ms(uint32_t start_cyc)
{
    return (float)(DWT->CYCCNT - start_cyc) / (SystemCoreClock / 1e3f);
}

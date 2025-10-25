/*
 * CAN.h
 *
 *  Created on: Oct 23, 2025
 *      Author: sheha
 */

#ifndef INC_CAN_H_
#define INC_CAN_H_

#include "stm32f1xx_hal.h"
#include <string.h>
#define CanDataArraySize		4
#define CanAddressOffset		0x20
#define CanBaseAddress			0x100


#define SpeedRpmMessageAddress				(CanBaseAddress+CanAddressOffset)
#define SpeedKmhMessageAddress				(SpeedRpmMessageAddress+CanAddressOffset)

#define AdcThrottleMessageAddress			(SpeedKmhMessageAddress+CanAddressOffset)

#define AdcCurrentSensorMessageAddress		(AdcThrottleMessageAddress+CanAddressOffset)
#define AdcVoltageSensorMessageAddress		(AdcCurrentSensorMessageAddress+CanAddressOffset)
#define EfficiencyMessageAddress			(AdcVoltageSensorMessageAddress+CanAddressOffset)

extern CAN_HandleTypeDef hcan;
extern CAN_TxHeaderTypeDef TxHeader;




#define 				CanNormal			CAN_MODE_NORMAL                     /*!< Normal mode   */
#define 				CanLoopBack			CAN_MODE_LOOPBACK					/*!< Loop back mode */
#define 				CanSilent			CAN_MODE_SILENT						/*!< Silent mode   */
#define 				CanCombined 		CAN_MODE_SILENT_LOOPBACK 			/*!< Loop back combined with silent mode   */


typedef enum
{
	AutoBusOffDisable=DISABLE,
	AutoBusOffEnable=!DISABLE

}CanBusState_t;

typedef enum
{
	AutoWakeupDisable=DISABLE,
	AutoWakeupEnable=!DISABLE

}CanWakeupState_t;

typedef enum
{
	AutoReTransmissionDisable=DISABLE,
	AutoReTransmissionEnable=!DISABLE

}CanReTransmissionState_t;

typedef enum
{
	ReceiveFifoLockedDisable=DISABLE,
	ReceiveFifoLockedEnable=!DISABLE

}CanReceiveFifoLockedState_t;

typedef enum
{
	TransmitFifoPriorityID=DISABLE,
	TransmitFifoPriorityOrder=!DISABLE

}CanTransmitFifoPriorityState_t;


typedef enum
{
    /* ====== Transmit Interrupt ====== */
    CAN_INT_TX_MAILBOX_EMPTY        = CAN_IT_TX_MAILBOX_EMPTY,    // Transmit mailbox empty

    /* ====== Receive FIFO0 Interrupts ====== */
    CAN_INT_RX_FIFO0_MSG_PENDING    = CAN_IT_RX_FIFO0_MSG_PENDING, // FIFO0 message pending
    CAN_INT_RX_FIFO0_FULL           = CAN_IT_RX_FIFO0_FULL,        // FIFO0 full
    CAN_INT_RX_FIFO0_OVERRUN        = CAN_IT_RX_FIFO0_OVERRUN,     // FIFO0 overrun

    /* ====== Receive FIFO1 Interrupts ====== */
    CAN_INT_RX_FIFO1_MSG_PENDING    = CAN_IT_RX_FIFO1_MSG_PENDING, // FIFO1 message pending
    CAN_INT_RX_FIFO1_FULL           = CAN_IT_RX_FIFO1_FULL,        // FIFO1 full
    CAN_INT_RX_FIFO1_OVERRUN        = CAN_IT_RX_FIFO1_OVERRUN,     // FIFO1 overrun

    /* ====== Error & Status Interrupts ====== */
    CAN_INT_ERROR_WARNING           = CAN_IT_ERROR_WARNING,        // Error warning
    CAN_INT_ERROR_PASSIVE           = CAN_IT_ERROR_PASSIVE,        // Error passive
    CAN_INT_BUS_OFF                 = CAN_IT_BUSOFF,               // Bus off
    CAN_INT_LAST_ERROR_CODE         = CAN_IT_LAST_ERROR_CODE,      // Last error code
    CAN_INT_ERROR                   = CAN_IT_ERROR,                // General error

    /* ====== Power Management ====== */
    CAN_INT_WAKEUP                  = CAN_IT_WAKEUP,               // Wake-up interrupt
    CAN_INT_SLEEP_ACK               = CAN_IT_SLEEP_ACK             // Sleep acknowledge
} CAN_InterruptSources_t;





typedef struct {
    uint32_t Prescaler;
    uint32_t Mode;
    uint32_t SJW;
    uint32_t BS1;
    uint32_t BS2;

    CanBusState_t   AutoBusOffMode;
    CanWakeupState_t AutoWakeupMode;
    CanReTransmissionState_t AutomaticReTransmissionMode;

    CanReceiveFifoLockedState_t ReceiveFifoLockedMode;
    CanTransmitFifoPriorityState_t TransmitFifoPriorityMode;
} CAN_CustomConfig_t;



/**
  * @brief  CAN message configuration structure
  */
typedef struct
{
    uint16_t NodeID;         // Standard 11-bit ID
    uint32_t NodeExtdID;     // Extended 29-bit ID
    uint8_t *DataArray;      // Pointer to data buffer (max 8 bytes)
    uint8_t  DataLength;     // Number of bytes in DataArray (0–8)
    uint8_t  IDE;            // CAN_ID_STD or CAN_ID_EXT
} CAN_ConfigMessageID_t;


extern CAN_ConfigMessageID_t CAN_SpeedRpmMsg ;

extern CAN_ConfigMessageID_t CAN_SpeedKmhMsg ;

extern CAN_ConfigMessageID_t CAN_ThrottleMsg ;

extern CAN_ConfigMessageID_t CAN_CurrentSensorMsg ;

extern CAN_ConfigMessageID_t CAN_VoltageSensorMsg ;

extern CAN_ConfigMessageID_t CAN_EfficiencyMsg ;
void CAN_DefaultInit(void);
void CAN_CustomInit(CAN_CustomConfig_t *config);
void CAN_ConfigDefaultFilter(uint8_t CAN_RxFifo);
HAL_StatusTypeDef CAN_SendMsg(CAN_ConfigMessageID_t *MessageConfig);
void CAN_SendRawData(CAN_ConfigMessageID_t *msg, void *data);

void CAN_InterruptControl(CAN_HandleTypeDef*hcan,CAN_InterruptSources_t InterruptSource,FunctionalState InterruptState);
void CAN_App_TxCompleteCallback(CAN_HandleTypeDef *hcan);
void CAN_App_RxCompleteCallback(CAN_HandleTypeDef *hcan,uint32_t CAN_FIFO);

#endif /* INC_CAN_H_ */

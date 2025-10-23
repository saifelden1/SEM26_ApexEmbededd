/*
 * CAN.h
 *
 *  Created on: Oct 23, 2025
 *      Author: sheha
 */

#ifndef INC_CAN_H_
#define INC_CAN_H_

#include "stm32f1xx_hal.h"
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



void CAN_CustomInit(CAN_CustomConfig_t *config);
void CAN_DefaultInit(void);
void CAN_ConfigDefaultFilter(void);
HAL_StatusTypeDef CAN_SendMsg(CAN_ConfigMessageID_t *MessageConfig);


#endif /* INC_CAN_H_ */

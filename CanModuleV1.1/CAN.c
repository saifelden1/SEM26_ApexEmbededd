/*
 * CAN.c
 *
 *  Created on: Oct 23, 2025
 *      Author: sheha
 */
#include "CAN.h"


uint32_t TxMailbox;//Return MailboxNumber
CAN_TxHeaderTypeDef TxHeader;

CAN_ConfigMessageID_t CAN_SpeedRpmMsg = {
    .NodeID = SpeedRpmMessageAddress,
    .IDE = CAN_ID_STD,
    .DataLength = CanDataArraySize
};

CAN_ConfigMessageID_t CAN_SpeedKmhMsg = {
    .NodeID = SpeedKmhMessageAddress,
    .IDE = CAN_ID_STD,
    .DataLength = CanDataArraySize
};

CAN_ConfigMessageID_t CAN_ThrottleMsg = {
    .NodeID = AdcThrottleMessageAddress,
    .IDE = CAN_ID_STD,
    .DataLength = CanDataArraySize
};

CAN_ConfigMessageID_t CAN_CurrentSensorMsg = {
    .NodeID = AdcCurrentSensorMessageAddress,
    .IDE = CAN_ID_STD,
    .DataLength = CanDataArraySize
};

CAN_ConfigMessageID_t CAN_VoltageSensorMsg = {
    .NodeID = AdcVoltageSensorMessageAddress,
    .IDE = CAN_ID_STD,
    .DataLength = CanDataArraySize
};

CAN_ConfigMessageID_t CAN_EfficiencyMsg = {
    .NodeID = EfficiencyMessageAddress,
    .IDE = CAN_ID_STD,
    .DataLength = CanDataArraySize
};


void CAN_DefaultInit(void)
{
	  hcan.Instance = CAN1;
	  hcan.Init.Prescaler = 4;
	  hcan.Init.Mode = CAN_MODE_NORMAL;
	  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
	  hcan.Init.TimeSeg1 = CAN_BS1_15TQ;
	  hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
	  hcan.Init.TimeTriggeredMode = DISABLE;
	  hcan.Init.AutoBusOff = ENABLE;
	  hcan.Init.AutoWakeUp = ENABLE;
	  hcan.Init.AutoRetransmission = DISABLE;
	  hcan.Init.ReceiveFifoLocked = DISABLE;
	  hcan.Init.TransmitFifoPriority = DISABLE;
	  if (HAL_CAN_Init(&hcan) != HAL_OK)
	  {
	    Error_Handler();
	  }

    CAN_ConfigDefaultFilter();    // Set filter
      HAL_CAN_Start(&hcan);        // Start CAN peripheral
}


void CAN_CustomInit(CAN_CustomConfig_t *CanCustomConfig)
{
	hcan.Instance = CAN1;
	hcan.Init.Prescaler = CanCustomConfig->Prescaler;
	hcan.Init.Mode = CanCustomConfig->Mode;
	hcan.Init.SyncJumpWidth = CanCustomConfig->SJW;
	hcan.Init.TimeSeg1 = CanCustomConfig->BS1;
	hcan.Init.TimeSeg2 = CanCustomConfig->BS2;

	hcan.Init.TimeTriggeredMode = DISABLE;
	hcan.Init.AutoBusOff = CanCustomConfig->AutoBusOffMode;
	hcan.Init.AutoWakeUp = CanCustomConfig->AutoWakeupMode;
	hcan.Init.AutoRetransmission = CanCustomConfig->AutomaticReTransmissionMode;
	hcan.Init.ReceiveFifoLocked = CanCustomConfig->ReceiveFifoLockedMode;
	hcan.Init.TransmitFifoPriority = CanCustomConfig->TransmitFifoPriorityMode;

    if (HAL_CAN_Init(&hcan) != HAL_OK)
    {
        Error_Handler();
    }
    CAN_ConfigDefaultFilter();    // Set filter
    HAL_CAN_Start(&hcan);        // Start CAN peripheral
}


/**
  * @brief  Default CAN filter configuration (accept all)
  * @retval None
  */
void CAN_ConfigDefaultFilter(void)
{
    CAN_FilterTypeDef canFilter;

    canFilter.FilterBank = 0;                    // Use filter bank 0
    canFilter.FilterMode = CAN_FILTERMODE_IDMASK; // Mask mode (accept range)
    canFilter.FilterScale = CAN_FILTERSCALE_32BIT;

    // Accept all IDs (both standard and extended)
    canFilter.FilterIdHigh = 0x0000;
    canFilter.FilterIdLow  = 0x0000;
    canFilter.FilterMaskIdHigh = 0x0000;
    canFilter.FilterMaskIdLow  = 0x0000;

    canFilter.FilterFIFOAssignment = CAN_RX_FIFO0; // Put received msgs in FIFO0
    canFilter.FilterActivation = ENABLE;           // Enable filter

    if (HAL_CAN_ConfigFilter(&hcan, &canFilter) != HAL_OK)
    {
        Error_Handler();
    }
}




/**
  * @brief  Send CAN message (custom function using HAL)
  * @param  CAN_SendMessage_t: Message Configuration Structure
  * @retval HAL status
  */

HAL_StatusTypeDef CAN_SendMsg(CAN_ConfigMessageID_t * MessageConfig)
{
	 // Configure header depending on ID type
	if (MessageConfig->IDE == CAN_ID_STD)
	    {
	        TxHeader.StdId = MessageConfig->NodeID;
	        TxHeader.IDE   = CAN_ID_STD;
	    }
	else if (MessageConfig->IDE == CAN_ID_EXT)
	   {
		   TxHeader.ExtId = MessageConfig->NodeExtdID;
		   TxHeader.IDE   = CAN_ID_EXT;
	   }
	else
	   {

	   }

    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.DLC = (MessageConfig->DataLength);
    TxHeader.TransmitGlobalTime = DISABLE;

    return HAL_CAN_AddTxMessage(&hcan, &TxHeader, (MessageConfig->DataArray), &TxMailbox);
}




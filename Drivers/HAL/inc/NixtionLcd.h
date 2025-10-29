/*
 * NixtionLcd.h
 *
 *  Created on: Oct 25, 2025
 *      Author: 01226
 */

#ifndef HAL_INC_NIXTIONLCD_H_
#define HAL_INC_NIXTIONLCD_H_



#include "stm32f1xx_hal.h"
#include "stdio.h"
#include "string.h"

/**
 * @brief  Struct to hold Nextion LCD configuration
 */
typedef struct {
    UART_HandleTypeDef *huart;  /**< UART handle used for communication */
} Nextion_HandleTypeDef;

/**
 * @brief  Initialize the Nextion LCD driver
 * @param  hnextion: Pointer to Nextion handle structure
 * @param  huart: Pointer to UART handle used for Nextion communication
 */
void Nextion_Init(Nextion_HandleTypeDef *hnextion, UART_HandleTypeDef *huart);

/**
 * @brief  Send a string command to Nextion LCD
 * @param  hnextion: Pointer to Nextion handle
 * @param  command: Command string (without 0xFF terminators)
 */
void Nextion_SendCommand(Nextion_HandleTypeDef *hnextion, const char *command);

/**
 * @brief  Send a text field update to Nextion LCD (e.g. t0.txt="Hello")
 * @param  hnextion: Pointer to Nextion handle
 * @param  id: Text object ID (e.g. "t0.txt")
 * @param  text: The string to display
 */
void Nextion_SendText(Nextion_HandleTypeDef *hnextion, const char *id, const char *text);

/**
 * @brief  Send a numeric update to Nextion LCD (e.g. n0.val=123)
 * @param  hnextion: Pointer to Nextion handle
 * @param  id: Numeric object ID (e.g. "n0.val")
 * @param  value: The integer value to send
 */
void Nextion_SendNumber(Nextion_HandleTypeDef *hnextion, const char *id, int value);

/**
 * @brief  Show an image (component visibility on)
 * @param  hnextion: Pointer to Nextion handle
 * @param  imageID: ID of the image component (e.g. "p0")
 */
void Nextion_ShowImage(Nextion_HandleTypeDef *hnextion, const char *imageID);

/**
 * @brief  Hide an image (component visibility off)
 * @param  hnextion: Pointer to Nextion handle
 * @param  imageID: ID of the image component (e.g. "p0")
 */
void Nextion_HideImage(Nextion_HandleTypeDef *hnextion, const char *imageID);





#endif /* HAL_INC_NIXTIONLCD_H_ */

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
<<<<<<< HEAD
extern UART_HandleTypeDef huart3;

#define Cuart  huart3
/**
 * @brief  Struct to hold Nextion LCD configuration
 */
// #define Cuart huart1
=======

/**
 * @brief  Struct to hold Nextion LCD configuration
 */
typedef struct {
    UART_HandleTypeDef *huart;  /**< UART handle used for communication */
} Nextion_HandleTypeDef;

>>>>>>> 12d4570193bed6ed4acd4090cb02d43ffdec1142
/**
 * @brief  Initialize the Nextion LCD driver
 * @param  hnextion: Pointer to Nextion handle structure
 * @param  huart: Pointer to UART handle used for Nextion communication
 */
<<<<<<< HEAD
void Nextion_Init( );
=======
void Nextion_Init(Nextion_HandleTypeDef *hnextion, UART_HandleTypeDef *huart);
>>>>>>> 12d4570193bed6ed4acd4090cb02d43ffdec1142

/**
 * @brief  Send a string command to Nextion LCD
 * @param  hnextion: Pointer to Nextion handle
 * @param  command: Command string (without 0xFF terminators)
 */
<<<<<<< HEAD
void Nextion_SendCommand(const char *command);
=======
void Nextion_SendCommand(Nextion_HandleTypeDef *hnextion, const char *command);
>>>>>>> 12d4570193bed6ed4acd4090cb02d43ffdec1142

/**
 * @brief  Send a text field update to Nextion LCD (e.g. t0.txt="Hello")
 * @param  hnextion: Pointer to Nextion handle
 * @param  id: Text object ID (e.g. "t0.txt")
 * @param  text: The string to display
 */
<<<<<<< HEAD
void Nextion_SendText( const char *id, const char *text);
=======
void Nextion_SendText(Nextion_HandleTypeDef *hnextion, const char *id, const char *text);
>>>>>>> 12d4570193bed6ed4acd4090cb02d43ffdec1142

/**
 * @brief  Send a numeric update to Nextion LCD (e.g. n0.val=123)
 * @param  hnextion: Pointer to Nextion handle
 * @param  id: Numeric object ID (e.g. "n0.val")
 * @param  value: The integer value to send
 */
<<<<<<< HEAD
void Nextion_SendNumber(const char *id, int value);
=======
void Nextion_SendNumber(Nextion_HandleTypeDef *hnextion, const char *id, int value);
>>>>>>> 12d4570193bed6ed4acd4090cb02d43ffdec1142

/**
 * @brief  Show an image (component visibility on)
 * @param  hnextion: Pointer to Nextion handle
 * @param  imageID: ID of the image component (e.g. "p0")
 */
<<<<<<< HEAD
void Nextion_ShowImage( const char *imageID);
=======
void Nextion_ShowImage(Nextion_HandleTypeDef *hnextion, const char *imageID);
>>>>>>> 12d4570193bed6ed4acd4090cb02d43ffdec1142

/**
 * @brief  Hide an image (component visibility off)
 * @param  hnextion: Pointer to Nextion handle
 * @param  imageID: ID of the image component (e.g. "p0")
 */
<<<<<<< HEAD
void Nextion_HideImage( const char *imageID);
=======
void Nextion_HideImage(Nextion_HandleTypeDef *hnextion, const char *imageID);
>>>>>>> 12d4570193bed6ed4acd4090cb02d43ffdec1142





#endif /* HAL_INC_NIXTIONLCD_H_ */

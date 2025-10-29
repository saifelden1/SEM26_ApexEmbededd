/*
 * NixtonLcd.c
 *
 *  Created on: Oct 18, 2025
 *      Author: 01226
 */

#include "NixtionLcd.h"

static uint8_t endCmd[3] = {0xFF, 0xFF, 0xFF};

/**
 * @brief Initialize Nextion handle with given UART
 */
<<<<<<< HEAD
void Nextion_Init()
{
=======
void Nextion_Init(Nextion_HandleTypeDef *hnextion, UART_HandleTypeDef *huart)
{
    hnextion->huart = huart;
>>>>>>> 12d4570193bed6ed4acd4090cb02d43ffdec1142
}

/**
 * @brief Send a generic command to Nextion
 */
<<<<<<< HEAD
void Nextion_SendCommand( const char *command)
{
    HAL_UART_Transmit(&Cuart, (uint8_t *)command, strlen(command), 1000);
    HAL_UART_Transmit(&Cuart, endCmd, 3, 100);
    HAL_UART_Transmit(&Cuart, endCmd, 3, 100);
    HAL_UART_Transmit(&Cuart, endCmd, 3, 100);
=======
void Nextion_SendCommand(Nextion_HandleTypeDef *hnextion, const char *command)
{
    HAL_UART_Transmit(hnextion->huart, (uint8_t *)command, strlen(command), 1000);
    HAL_UART_Transmit(hnextion->huart, endCmd, 3, 100);
    HAL_UART_Transmit(hnextion->huart, endCmd, 3, 100);
    HAL_UART_Transmit(hnextion->huart, endCmd, 3, 100);
>>>>>>> 12d4570193bed6ed4acd4090cb02d43ffdec1142
}//aproved

/**
 * @brief Send a text update (e.g. t0.txt="Voltage OK")
 */
<<<<<<< HEAD
void Nextion_SendText( const char *id, const char *text)
{
    char buffer[64];
    int len = snprintf(buffer, sizeof(buffer), "%s=\"%s\"", id, text);
    HAL_UART_Transmit(&Cuart, (uint8_t *)buffer, len, 1000);
    HAL_UART_Transmit(&Cuart, endCmd, 3, 100);
=======
void Nextion_SendText(Nextion_HandleTypeDef *hnextion, const char *id, const char *text)
{
    char buffer[64];
    int len = snprintf(buffer, sizeof(buffer), "%s=\"%s\"", id, text);
    HAL_UART_Transmit(hnextion->huart, (uint8_t *)buffer, len, 1000);
    HAL_UART_Transmit(hnextion->huart, endCmd, 3, 100);
>>>>>>> 12d4570193bed6ed4acd4090cb02d43ffdec1142
}//imp

/**
 * @brief Send a numeric update (e.g. n0.val=55)
 */
<<<<<<< HEAD
void Nextion_SendNumber( const char *id, int value)
{
    char buffer[32];
    int len = snprintf(buffer, sizeof(buffer), "%s=%d", id, value);
    HAL_UART_Transmit(&Cuart, (uint8_t *)buffer, len, 1000);
    HAL_UART_Transmit(&Cuart, endCmd, 3, 100);
}
//imp
=======
void Nextion_SendNumber(Nextion_HandleTypeDef *hnextion, const char *id, int value)
{
    char buffer[32];
    int len = snprintf(buffer, sizeof(buffer), "%s=%d", id, value);
    HAL_UART_Transmit(hnextion->huart, (uint8_t *)buffer, len, 1000);
    HAL_UART_Transmit(hnextion->huart, endCmd, 3, 100);
}//imp
>>>>>>> 12d4570193bed6ed4acd4090cb02d43ffdec1142

/**
 * @brief Show an image (make visible)
 */
<<<<<<< HEAD
void Nextion_ShowImage( const char *imageID)
{
    char cmd[32];
    snprintf(cmd, sizeof(cmd), "vis %s,1", imageID);
    Nextion_SendCommand( cmd);
=======
void Nextion_ShowImage(Nextion_HandleTypeDef *hnextion, const char *imageID)
{
    char cmd[32];
    snprintf(cmd, sizeof(cmd), "vis %s,1", imageID);
    Nextion_SendCommand(hnextion, cmd);
>>>>>>> 12d4570193bed6ed4acd4090cb02d43ffdec1142
}//not working

/**
 * @brief Hide an image (make invisible)
 */
<<<<<<< HEAD
void Nextion_HideImage( const char *imageID)
{
    char cmd[32];
    snprintf(cmd, sizeof(cmd), "vis %s,0", imageID);
    Nextion_SendCommand( cmd);
=======
void Nextion_HideImage(Nextion_HandleTypeDef *hnextion, const char *imageID)
{
    char cmd[32];
    snprintf(cmd, sizeof(cmd), "vis %s,0", imageID);
    Nextion_SendCommand(hnextion, cmd);
>>>>>>> 12d4570193bed6ed4acd4090cb02d43ffdec1142
}//not working


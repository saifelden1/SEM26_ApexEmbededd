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
void Nextion_Init(Nextion_HandleTypeDef *hnextion, UART_HandleTypeDef *huart)
{
    hnextion->huart = huart;
}

/**
 * @brief Send a generic command to Nextion
 */
void Nextion_SendCommand(Nextion_HandleTypeDef *hnextion, const char *command)
{
    HAL_UART_Transmit(hnextion->huart, (uint8_t *)command, strlen(command), 1000);
    HAL_UART_Transmit(hnextion->huart, endCmd, 3, 100);
    HAL_UART_Transmit(hnextion->huart, endCmd, 3, 100);
    HAL_UART_Transmit(hnextion->huart, endCmd, 3, 100);
}//aproved

/**
 * @brief Send a text update (e.g. t0.txt="Voltage OK")
 */
void Nextion_SendText(Nextion_HandleTypeDef *hnextion, const char *id, const char *text)
{
    char buffer[64];
    int len = snprintf(buffer, sizeof(buffer), "%s=\"%s\"", id, text);
    HAL_UART_Transmit(hnextion->huart, (uint8_t *)buffer, len, 1000);
    HAL_UART_Transmit(hnextion->huart, endCmd, 3, 100);
}//imp

/**
 * @brief Send a numeric update (e.g. n0.val=55)
 */
void Nextion_SendNumber(Nextion_HandleTypeDef *hnextion, const char *id, int value)
{
    char buffer[32];
    int len = snprintf(buffer, sizeof(buffer), "%s=%d", id, value);
    HAL_UART_Transmit(hnextion->huart, (uint8_t *)buffer, len, 1000);
    HAL_UART_Transmit(hnextion->huart, endCmd, 3, 100);
}//imp

/**
 * @brief Show an image (make visible)
 */
void Nextion_ShowImage(Nextion_HandleTypeDef *hnextion, const char *imageID)
{
    char cmd[32];
    snprintf(cmd, sizeof(cmd), "vis %s,1", imageID);
    Nextion_SendCommand(hnextion, cmd);
}//not working

/**
 * @brief Hide an image (make invisible)
 */
void Nextion_HideImage(Nextion_HandleTypeDef *hnextion, const char *imageID)
{
    char cmd[32];
    snprintf(cmd, sizeof(cmd), "vis %s,0", imageID);
    Nextion_SendCommand(hnextion, cmd);
}//not working


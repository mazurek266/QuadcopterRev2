#ifndef INC_ESP8266_H_
#define INC_ESP8266_H_

#include <stdint.h>
#include <stdio.h>
#include <stm32f4xx_hal.h>
#include "DataStructures.h"

extern UART_HandleTypeDef huart1;

void Esp8266_StartTransmissionCommand(UART_HandleTypeDef *huart, uint8_t nCharactersToBeSend);
void Esp8266_Init(UART_HandleTypeDef *huart);
void Esp8266_SendData(UART_HandleTypeDef *huart, const char* message);
void Send_Telemetry(UART_HandleTypeDef *huart, struct Data mpu6050data, Vector3AxisF angleVector);


#endif

#include "ESP8266.h"


/****************************************************************************************************************************************************************/
void Esp8266_StartTransmissionCommand(UART_HandleTypeDef *huart, uint8_t nCharactersToBeSend)
{
	uint16_t msgLength;
	uint8_t commandBuffer[34];

	msgLength = sprintf((char*) commandBuffer, "AT+CIPSEND=0,%d\r\n",nCharactersToBeSend);	//sending %d characters on port 0
	while(HAL_UART_Transmit(huart, (uint8_t *) commandBuffer, msgLength,10) != HAL_OK);
}
/****************************************************************************************************************************************************************/
void Esp8266_Init(UART_HandleTypeDef *huart)
{
	uint16_t msgLength;
	uint8_t commandBuffer[34];

	msgLength = sprintf( (char*)commandBuffer,"AT+RST\r\n");							//Restarting module
	HAL_UART_Transmit(huart,(uint8_t *)commandBuffer,msgLength,100) ;

	msgLength = sprintf( (char*)commandBuffer,"AT+CWMODE=2\r\n");						//Configuring module as Access Point
	HAL_UART_Transmit(huart,(uint8_t *)commandBuffer,msgLength,100) ;

	msgLength = sprintf( (char*)commandBuffer,"AT+CIPMUX=1\r\n");						//Allowing multiple access devices
	HAL_UART_Transmit(huart,(uint8_t *)commandBuffer,msgLength,100) ;

	msgLength = sprintf( (char*)commandBuffer,"AT+CIPSERVER=1,333\r\n");				// Enabling server and setting port number 333
	HAL_UART_Transmit(huart,(uint8_t *)commandBuffer,msgLength,100);

}
/****************************************************************************************************************************************************************/
void Esp8266_SendData(UART_HandleTypeDef *huart, const char* message)
{
	uint16_t msgLength;
	uint8_t sendBuffer[50];

	Esp8266_StartTransmissionCommand(huart, 11);

	msgLength = sprintf( (char*)sendBuffer, *message + "\r\n");
	while(HAL_UART_Transmit(huart,(uint8_t *)sendBuffer,msgLength,10) != HAL_OK);

}
/****************************************************************************************************************************************************************/
void Send_Telemetry(UART_HandleTypeDef *huart, struct Data mpu6050data, Vector3AxisF angleVector)
{
	uint16_t msgLength;
	uint8_t sendBuffer[77];

	//HAL_GPIO_WritePin(GPIOD,GPIO_PIN_12,GPIO_PIN_SET);

			msgLength = sprintf((char*) sendBuffer, "% 4.2f;% 4.2f;% 4.2f;% 5.2f;% 5.2f;% 5.2f;% 4.2f;% 4.2f;% 4.2f;% 4.2f;",
			(mpu6050data.accelX),(mpu6050data.accelY),(mpu6050data.accelZ),(mpu6050data.rotX),(mpu6050data.rotY),(mpu6050data.rotZ),(mpu6050data.temp),
			(angleVector.x),(angleVector.y),(angleVector.z));

			Esp8266_StartTransmissionCommand(huart, msgLength);
			while(HAL_UART_Transmit(huart, (uint8_t *) sendBuffer, msgLength,10) != HAL_OK);

	//HAL_GPIO_WritePin(GPIOD,GPIO_PIN_12,GPIO_PIN_RESET);

}

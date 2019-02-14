/*
 * wifi.c
 *
 *  Created on: Aug 4, 2018
 *      Author: daniel
 */

#include "wifi.h"

UART_HandleTypeDef usartHandle;

UART_HandleTypeDef wifiInit() {

	usartHandle.Instance = USART6;
	usartHandle.Init.BaudRate = 115200;
	usartHandle.Init.WordLength = UART_WORDLENGTH_8B;
	usartHandle.Init.StopBits = UART_STOPBITS_1;
	usartHandle.Init.Parity = UART_PARITY_NONE;
	usartHandle.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	usartHandle.Init.Mode = UART_MODE_TX_RX;


	HAL_StatusTypeDef status;
	if ((status = HAL_UART_Init(&usartHandle)) != HAL_OK) {
		changeLedState(1);
	}

	HAL_Delay(1000);

	return usartHandle;
}

void wifiSendMessage(char * command){
	int length;
	for (length = 0; command[length] != '\0'; length++);
	HAL_StatusTypeDef status;
	if ((status = HAL_UART_Transmit(&usartHandle, (uint8_t*) command, length, 1000))
			!= HAL_OK) {
		///Problem with uart
	}
}

void wifiSendBytes(char* buffer, uint8_t bytesNum) {
	HAL_StatusTypeDef status;
		if ((status = HAL_UART_Transmit(&usartHandle, (uint8_t*) buffer, bytesNum, 1000))
				!= HAL_OK) {
			///Problem with uart
		}
}


void wifiReceiveMessage(char* buffer, uint8_t bytesNum) {
	for (int i = 0; i < bytesNum; i++) {
		buffer[i] = 0;
	}
	if (HAL_UART_Receive(&usartHandle, (uint8_t *) buffer, bytesNum, 1)
			== HAL_OK) {
		//buffer contains the received bytes, nothing to do here
	}
	buffer[bytesNum] = '\0';
}

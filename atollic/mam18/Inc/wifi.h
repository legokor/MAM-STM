/*
 * wifi.h
 *
 *  Created on: Aug 4, 2018
 *      Author: daniel
 */

#ifndef WIFI_H_
#define WIFI_H_

#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_conf.h"
#include "led.h"

#define USART_RX GPIO_PIN_7
#define USART_TX GPIO_PIN_6

UART_HandleTypeDef wifiInit();

void wifiReceiveMessage(char* buffer, uint8_t bytesNum);
void wifiSendBytes(char* buffer, uint8_t bytesNum);
void wifiSendMessage(char * command);

#endif /* WIFI_H_ */

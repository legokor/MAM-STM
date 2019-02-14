/*
 * led.h
 *
 *  Created on: Aug 4, 2018
 *      Author: daniel
 */

#ifndef LED_H_
#define LED_H_

#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_conf.h"

#define DEBUG_LED GPIO_PIN_13

void changeLedState(int state);

#endif /* LED_H_ */

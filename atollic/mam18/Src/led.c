/*
 * led.c
 *
 *  Created on: Aug 4, 2018
 *      Author: daniel
 */
#include "led.h"

void changeLedState(int state) {
	HAL_GPIO_WritePin(GPIOC, DEBUG_LED, state);
}


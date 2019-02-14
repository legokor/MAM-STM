/*
 * adc.c
 *
 *  Created on: Aug 4, 2018
 *      Author: daniel
 */
#include "adc.h"

ADC_HandleTypeDef hadc1;

ADC_HandleTypeDef adcInit(){
		ADC_ChannelConfTypeDef sConfig;

		GPIO_InitTypeDef gpioInit;

		__GPIOC_CLK_ENABLE();
		__ADC1_CLK_ENABLE();

		gpioInit.Pin = GPIO_PIN_0;
		gpioInit.Mode = GPIO_MODE_ANALOG;
		gpioInit.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(GPIOC, &gpioInit);

		HAL_NVIC_SetPriority(ADC_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(ADC_IRQn);

		/**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
		 */
		hadc1.Instance = ADC1;
		hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
		hadc1.Init.Resolution = ADC_RESOLUTION_12B;
		hadc1.Init.ScanConvMode = DISABLE;
		hadc1.Init.ContinuousConvMode = ENABLE;
		hadc1.Init.DiscontinuousConvMode = DISABLE;
		hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
		hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T1_CC1;
		hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
		hadc1.Init.NbrOfConversion = 1;
		hadc1.Init.DMAContinuousRequests = ENABLE;
		hadc1.Init.EOCSelection = DISABLE;

		if (HAL_ADC_Init(&hadc1) != HAL_OK) {
			_Error_Handler(__FILE__, __LINE__);
		}

		/**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
		 */
		sConfig.Channel = ADC_CHANNEL_10;
		sConfig.Rank = 1;
		sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
		sConfig.Offset = 0;

		if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
			_Error_Handler(__FILE__, __LINE__);
		}
		return hadc1;
}
void adcStart(){
	HAL_ADC_Start(&hadc1);
}
void adcMeasure(uint32_t * ADCValue){
	if (HAL_ADC_PollForConversion(&hadc1, 1000000) == HAL_OK)
	{
	  ADCValue = HAL_ADC_GetValue(&hadc1);
	}
}


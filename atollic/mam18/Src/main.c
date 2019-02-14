/**
 ******************************************************************************
 * File Name          : main.c
 * Description        : Main program body
 ******************************************************************************
 * COPYRIGHT(c) 2018 STMicroelectronics
 */

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include <math.h>
#include <stdbool.h>
#include "main.h"
#include "stm32f4xx_hal.h"

#include "wifi.h"
#include "led.h"
#include "adc.h"

#define SERVO_RIGHT_BACK 5
#define SERVO_RIGHT_FRONT 6
#define SERVO_LEFT_FRONT 7
#define SERVO_LEFT_BACK 0
#define SERVO_ARM_YAW 8

#define MID_POS 200
#define MIN_POS 140
#define MAX_POS 270

/* Private variables ---------------------------------------------------------*/


TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim9;

//UART_HandleTypeDef huart6;

/* --------------------------------------------------------My Global Variables --------------------------------------------------------*/

char receiveBuffer[100];

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM9_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* --------------------------------------------------------My Function prototipes --------------------------------------------------------*/

void PWM_start_all(void);
void PWM_set_pulse(int ch, int pulse);
void PWM_set_all_pulse(int pulse);

void DC_motor_set(int motor_num, int dir, int speed);
void carMode(int angle);
void softStartAll(int speed, int dir);
void changeLedState(int state);

bool newMessage = false;

char rxBuf;
ADC_HandleTypeDef hadc1;

int main(void) {
	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* Configure the system clock */
	SystemClock_Config();

	/* Initialize all configured peripherals */
	MX_GPIO_Init();

	MX_TIM2_Init();
	MX_TIM3_Init();
	MX_TIM4_Init();
	MX_TIM5_Init();
	MX_TIM9_Init();

	/* -------------------------------------------------------- My init calls -------------------------------------------------------- */

	PWM_start_all();

	//HAL_UART_Receive_IT(&huart6, &rxBuf, 1);

	//0,4,5,6,7,8

	// ------------------------------------------------------------ while(1) ------------------------------------------------------------
	//int k = 21;//servo positions : 14-27; -- 21 közép
	int steeringAngle = 0;
	int pinAngle = MID_POS;
	int motorSpeed = 500;

	// PWM_set_pulse(SERVO_RIGHT_BACK,MAX_POS);

	/* int k = 2000;
	 PWM_set_pulse(1,k);
	 PWM_set_pulse(2,k);
	 PWM_set_pulse(3,k);

	 PWM_set_pulse(9,k);
	 PWM_set_pulse(10,k);
	 PWM_set_pulse(11,k);*/

	PWM_set_pulse(4, MID_POS);

	PWM_set_pulse(SERVO_RIGHT_FRONT, MID_POS);
	PWM_set_pulse(SERVO_RIGHT_BACK, MID_POS);
	PWM_set_pulse(SERVO_LEFT_FRONT, MID_POS);
	PWM_set_pulse(SERVO_LEFT_BACK, MID_POS);

	PWM_set_pulse(SERVO_ARM_YAW, MID_POS);

	wifiInit();
	hadc1 = adcInit();
	HAL_ADC_Start(&hadc1);
	//changeLedState(1);
	while (1) {

		//if (newMessage) {
			//  do the thing
			// HAL_Delay(1000);
			// HAL_UART_Transmit_IT(&huart6, transmit_data, 4);
			//HAL_UART_Transmit(&huart6, (uint8_t *) transmit_data, strlen(transmit_data), 120);
			//HAL_UART_Transmit

			// HAL_UART_Transmit(&huart6, (uint8_t *) receiveBuffer, strlen(receiveBuffer), 120);
			//newMessage = false;
			wifiReceiveMessage(receiveBuffer,10);

//			if(strcmp(receiveBuffer,"LDON") == 0){
//				wifiSendMessage("led on");
//				changeLedState(1);
//			}
//			else if(strcmp(receiveBuffer,"LDOFF") == 0){
//				changeLedState(0);
//			}
//			else if(receiveBuffer[0] == 'X'){
//							  int szaz = (receiveBuffer[1] - '0') * 100;
//							  int tiz = (receiveBuffer[2] - '0') * 10;
//							  int egy = (receiveBuffer[3] - '0') * 1;
//							  int pwm = (szaz + tiz + egy)*70/100;
//
//							  int wheelAngle = pwm+170;
//							  int dist = MID_POS-wheelAngle;
//							  //int backWheelAngle = MAX_POS-wheelAngle;
//
//							  PWM_set_pulse(SERVO_LEFT_FRONT, MID_POS+dist);
//							  PWM_set_pulse(SERVO_RIGHT_FRONT, MID_POS+dist);
//							  PWM_set_pulse(SERVO_RIGHT_BACK, MID_POS-dist);
//							  PWM_set_pulse(SERVO_LEFT_BACK, MID_POS-dist);
//			}

			if (strcmp(receiveBuffer, "FOWD") == 0) {
				softStartAll(2000, 1);
			} else if (strcmp(receiveBuffer, "BAWD") == 0) {
				softStartAll(2000, 0);
			} else if (strcmp(receiveBuffer, "RGHT") == 0) {
				if (MID_POS + steeringAngle < MAX_POS - 30)
					steeringAngle += 5;
				carMode(steeringAngle);
			} else if (strcmp(receiveBuffer, "LEFT") == 0) {
				if (MID_POS + steeringAngle > MIN_POS + 30)
					steeringAngle -= 5;
				carMode(steeringAngle);
			} else if (strcmp(receiveBuffer, "LDON") == 0) {
				changeLedState(1);
			} else if (strcmp(receiveBuffer, "LDOF") == 0) {
				changeLedState(0);
			} else if (strcmp(receiveBuffer, "PNDN") == 0) {
				PWM_set_pulse(SERVO_ARM_YAW, MAX_POS);

			} else if (strcmp(receiveBuffer, "PNUP") == 0) {
				PWM_set_pulse(SERVO_ARM_YAW, MID_POS - 20);
			}
			else if(receiveBuffer[0] == 'X'){
				  int szaz = (receiveBuffer[1] - '0') * 100;
				  int tiz = (receiveBuffer[2] - '0') * 10;
				  int egy = (receiveBuffer[3] - '0') * 1;
				  int pwm = (szaz + tiz + egy)*70/100;

				  int wheelAngle = pwm+170;
				  int dist = MID_POS-wheelAngle;
				  //int backWheelAngle = MAX_POS-wheelAngle;

				  PWM_set_pulse(SERVO_LEFT_FRONT, MID_POS+dist);
				  PWM_set_pulse(SERVO_RIGHT_FRONT, MID_POS+dist);
				  PWM_set_pulse(SERVO_RIGHT_BACK, MID_POS-dist);
				  PWM_set_pulse(SERVO_LEFT_BACK, MID_POS-dist);
			}
			else if(receiveBuffer[0] == 'Q'){
				  int szaz = (receiveBuffer[1] - '0') * 100;
				  int tiz = (receiveBuffer[2] - '0') * 10;
				  int egy = (receiveBuffer[3] - '0') * 1;
				  int pwm = (szaz + tiz + egy)*130/100;

					PWM_set_pulse(4,MIN_POS+pwm);
			}
			else if(strcmp(receiveBuffer, "STOP") == 0){
				for (int i = 0; i < 6; i++) {
					DC_motor_set(i, 1, 0);
				}
				wifiSendMessage("stop");
			}
			else if(strcmp(receiveBuffer,"ADC") == 0){

				if (HAL_ADC_PollForConversion(&hadc1, 1000000) == HAL_OK)
				{
				  uint32_t ADCValue = HAL_ADC_GetValue(&hadc1);
				  char buffer[32];
				  sprintf(buffer, "%lu", ADCValue);
				  buffer[31] = '\0';
				  wifiSendMessage(buffer);
				}
			}

			//HAL_UART_Transmit(&huart6, (uint8_t *) "ok", strlen("ok"), 120);
			//strcpy(receiveBuffer, "");

		//}
		//HAL_Delay(10);

	}
}


void carMode(int angle) {

	///if(angle > 5) angle = 5;
	///else if(angle < -5) angle = -5;
	int newAngle = MID_POS + angle;
	int newBackAngle = MID_POS - angle;
	if (newAngle > (MAX_POS - 30))
		newAngle = MAX_POS - 30;
	if (newAngle < (MIN_POS + 30))
		newAngle = MIN_POS + 30;

	PWM_set_pulse(SERVO_LEFT_FRONT, newAngle);

	PWM_set_pulse(SERVO_RIGHT_FRONT, newAngle);

	PWM_set_pulse(SERVO_RIGHT_BACK, newBackAngle);

	PWM_set_pulse(SERVO_LEFT_BACK, newBackAngle);

}

void rotateMode() {
	PWM_set_pulse(SERVO_LEFT_FRONT, MID_POS + 4);
	PWM_set_pulse(SERVO_RIGHT_BACK, MID_POS + 4);
	PWM_set_pulse(SERVO_RIGHT_FRONT, MID_POS - 4);
	PWM_set_pulse(SERVO_LEFT_BACK, MID_POS - 4);
}

void allMotorStart(int speed, int dir) {
	for (int i = 0; i < 6; i++) {
		DC_motor_set(i, dir, speed);
	}
}

void softStartAll(int speed, int dir) {
	for (int k = 0; k < speed; k += 10) {
		for (int i = 0; i < 6; i++) {
			DC_motor_set(i, dir, speed);
		}
		HAL_Delay(10);
	}
}

// starts al pwm channels
void PWM_start_all(void) {
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);

	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);

	//HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_4);

	HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_2);
}

/*
 * Sets a pulse with of a pwm channel
 * param 1: int Channel number
 * param 2: int Pulse with in percent: 0 to 100
 */
void PWM_set_pulse(int ch, int pulse) {

	int period = 199;	//24000;
	pulse = period * (pulse / 255.0);

	switch (ch) {

	case 0: {
		TIM2->CCR3 = pulse;
		break;
	}

	case 1: {
		TIM2->CCR4 = pulse;
		break;
	}

	case 2: {
		TIM3->CCR1 = pulse;
		break;
	}

	case 3: {
		TIM3->CCR2 = pulse;
		break;
	}

	case 4: {
		TIM3->CCR3 = pulse;
		break;
	}

	case 5: {
		TIM3->CCR4 = pulse;
		break;
	}

	case 6: {
		TIM4->CCR3 = pulse;
		break;
	}

	case 7: {
		TIM4->CCR4 = pulse;
		break;
	}

	case 8: {
		//TIM5->CCR3 = pulse;
		TIM2->CCR1 = pulse;
		break;
	}

	case 9: {
		TIM5->CCR4 = pulse;
		break;
	}

	case 10: {
		TIM9->CCR1 = pulse;
		break;
	}

	case 11: {
		TIM9->CCR2 = pulse;
		break;
	}

	default: {
		break;
	}
		// end case
	}
}

/*
 * Sets all pwm to the same pulse width
 */
void PWM_set_all_pulse(int pulse) {
	for (int i = 0; i < 12; i++) {
		PWM_set_pulse(i, pulse);
	}
}

/*
 * Sets a motor to a certain direction with a certain speed
 * motor num: 0 - 5
 * dir        : 0 fwd, anithing else bck
 * speed    ? 0 - 100 (%)    0 speed sets 0 PWM
 */
void DC_motor_set(int motor_num, int dir, int speed) {
	switch (motor_num) {
	case 0: {
		PWM_set_pulse(9, speed);

		if (speed == 0) {
			break;
		}

		if (dir == 0) {
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
		} else {
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
		}
		break;
	}

	case 5: {
		PWM_set_pulse(1, speed);

		if (speed == 0) {
			break;
		}

		if (dir == 0) {
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_RESET);
		} else {
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_SET);
		}
		break;
	}

	case 1: {
		PWM_set_pulse(11, speed);

		if (speed == 0) {
			break;
		}

		if (dir == 0) {
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, GPIO_PIN_SET);

		} else {
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, GPIO_PIN_RESET);
		}
		break;
	}
	case 2: {
		PWM_set_pulse(10, speed);

		if (speed == 0) {
			break;
		}

		if (dir == 0) {
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_RESET);
		} else {
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_SET);

		}
		break;
	}

	case 4: {
		PWM_set_pulse(2, speed);

		if (speed == 0) {
			break;
		}

		if (dir == 0) {
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);

		} else {
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
		}
		break;
	}

	case 3: {
		PWM_set_pulse(3, speed);

		if (speed == 0) {
			break;
		}

		if (dir == 0) {
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
		} else {
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
		}
		break;
	}

	default: {
		// ???
		break;
	}
		//end switch
	}
}

// ------------------------------------------------------------ generated functions ------------------------------------------------------------

/** System Clock Configuration
 */
void SystemClock_Config(void) {

	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_ClkInitTypeDef RCC_ClkInitStruct;

	/**Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE()
	;

	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = 16;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 8;
	RCC_OscInitStruct.PLL.PLLN = 96;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 4;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Configure the Systick interrupt time
	 */
	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

	/**Configure the Systick
	 */
	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

	/* SysTick_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}



/* TIM2 init function */
static void MX_TIM2_Init(void) {

	TIM_MasterConfigTypeDef sMasterConfig;
	TIM_OC_InitTypeDef sConfigOC;

	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 479;	//4;//2;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 1999;	//24000;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	if (HAL_TIM_PWM_Init(&htim2) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig)
			!= HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 12000;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

	if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3)
			!= HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4)
			!= HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	HAL_TIM_MspPostInit(&htim2);

}

/* TIM3 init function */
static void MX_TIM3_Init(void) {

	TIM_MasterConfigTypeDef sMasterConfig;
	TIM_OC_InitTypeDef sConfigOC;

	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 479;	//2;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 1999;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig)
			!= HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 12000;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2)
			!= HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3)
			!= HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4)
			!= HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	HAL_TIM_MspPostInit(&htim3);

}

/* TIM4 init function */
static void MX_TIM4_Init(void) {

	TIM_MasterConfigTypeDef sMasterConfig;
	TIM_OC_InitTypeDef sConfigOC;

	htim4.Instance = TIM4;
	htim4.Init.Prescaler = 479;	//2;
	htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim4.Init.Period = 1999;
	htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	if (HAL_TIM_PWM_Init(&htim4) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig)
			!= HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 12000;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3)
			!= HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4)
			!= HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	HAL_TIM_MspPostInit(&htim4);

}

/* TIM5 init function */
static void MX_TIM5_Init(void) {

	TIM_MasterConfigTypeDef sMasterConfig;
	TIM_OC_InitTypeDef sConfigOC;

	htim5.Instance = TIM5;
	htim5.Init.Prescaler = 479;	//2;
	htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim5.Init.Period = 1999;
	htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	if (HAL_TIM_PWM_Init(&htim5) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig)
			!= HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 12000;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	/* if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
	 {
	 _Error_Handler(__FILE__, __LINE__);
	 }*/

	if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_4)
			!= HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	HAL_TIM_MspPostInit(&htim5);

}

/* TIM9 init function */
static void MX_TIM9_Init(void) {

	TIM_OC_InitTypeDef sConfigOC;

	htim9.Instance = TIM9;
	htim9.Init.Prescaler = 479;	//2
	htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim9.Init.Period = 1999;
	htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	if (HAL_TIM_PWM_Init(&htim9) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 12000;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	if (HAL_TIM_PWM_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_2)
			!= HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	HAL_TIM_MspPostInit(&htim9);

}


/** Configure pins as
 * Analog
 * Input
 * Output
 * EVENT_OUT
 * EXTI
 */
static void MX_GPIO_Init(void) {

	GPIO_InitTypeDef GPIO_InitStruct;

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOE_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOE,
			GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_13 | GPIO_PIN_15
					| GPIO_PIN_1, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13 | GPIO_PIN_15 | GPIO_PIN_5,
			GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB,
			GPIO_PIN_1 | GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9,
			GPIO_PIN_RESET);

	/*Configure GPIO pins : PE2 PE3 PE4 PE13
	 PE15 PE1 */
	GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_13
			| GPIO_PIN_15 | GPIO_PIN_1;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

	/*Configure GPIO pins : PC13 PC15 PC5 */
	GPIO_InitStruct.Pin = GPIO_PIN_13 | GPIO_PIN_15 | GPIO_PIN_5;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : PB1 PB6 PB7 PB8
	 PB9 */
	GPIO_InitStruct.Pin = GPIO_PIN_1 | GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8
			| GPIO_PIN_9;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : const_0_in_Pin */
	GPIO_InitStruct.Pin = const_0_in_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(const_0_in_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : const_0_inD11_Pin const_0_inD12_Pin const_0_inD13_Pin const_0_inD0_Pin */
	GPIO_InitStruct.Pin = const_0_inD11_Pin | const_0_inD12_Pin
			| const_0_inD13_Pin | const_0_inD0_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	/*Configure GPIO pin : const_0_inC12_Pin */
	GPIO_InitStruct.Pin = const_0_inC12_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(const_0_inC12_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @param  None
 * @retval None
 */
void _Error_Handler(char * file, int line) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	while (1) {
	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT

/**
 * @brief Reports the name of the source file and the source line number
 * where the assert_param error has occurred.
 * @param file: pointer to the source file name
 * @param line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t* file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
	 ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */

}

#endif

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

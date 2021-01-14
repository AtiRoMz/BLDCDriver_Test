/*
 * bldc_calib.c
 *
 *  Created on: Dec 16, 2020
 *      Author: Ryohei
 */

#include "bldc_calib.h"
#include "as5147.h"
#include <stdio.h>
#include <math.h>

void BLDCCalibZeroPos(void) {
	int8_t n_loop = 12;		//pole number
	uint16_t angle_data;
	float elec_angle_deg;

	while (n_loop--) {
		for(bldc_conduction_phase i = BLDC_UtoV; i <= BLDC_WtoV; i++) {
			BLDC120DegConduction(i, 0.10f);
			HAL_Delay(300);
			angle_data = (AS5147Read(AS5147_ANGLECOM) & 0x3FFF);		//mask lower 14bit
			elec_angle_deg = fmodf(((float)angle_data + ((float)0x3FFF / 12) - 361), ((float)0x3FFF / 12)) * ((float)(360 * 12) / 0x3FFF);

			printf("%d %d %f\n", i, angle_data, elec_angle_deg);
		}
	}
}

void BLDC120DegConduction(bldc_conduction_phase phase, float pwm_duty) {
	switch(phase) {
	case 0 :
		__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, __HAL_TIM_GET_AUTORELOAD(&htim8) * pwm_duty);
		__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 0);
		__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, 0);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
		break;
	case 1 :
		__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, __HAL_TIM_GET_AUTORELOAD(&htim8) * pwm_duty);
		__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 0);
		__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, 0);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
		break;
	case 2 :
		__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 0);
		__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, __HAL_TIM_GET_AUTORELOAD(&htim8) * pwm_duty);
		__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, 0);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
		break;
	case 3 :
		__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 0);
		__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, __HAL_TIM_GET_AUTORELOAD(&htim8) * pwm_duty);
		__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, 0);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
		break;
	case 4 :
		__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 0);
		__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 0);
		__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, __HAL_TIM_GET_AUTORELOAD(&htim8) * pwm_duty);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
		break;
	case 5 :
		__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 0);
		__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 0);
		__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, __HAL_TIM_GET_AUTORELOAD(&htim8) * pwm_duty);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
		break;
	}
}

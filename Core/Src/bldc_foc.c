/*
 * bldc_foc.c
 *
 *  Created on: Dec 23, 2020
 *      Author: Ryohei
 */

#include "bldc_foc.h"
#include <stdio.h>

//private define
#define ADC_CURRENT_SENSE_BUFFER_SIZE	((uint32_t)3)

//static variables
static uint16_t current_sense_data[ADC_CURRENT_SENSE_BUFFER_SIZE] = {};

/*
 * Start current sensing using DMA transfer
 * @param
 * @return
 * @note	Triggered by tim8(at the center of center aligned PWM.
 * 			Data will be automatically transfered to the variable by DMA
 */
void BLDCStartCurrentSense(void) {
	HAL_ADC_Start_DMA(&hadc1, (uint32_t *)current_sense_data, ADC_CURRENT_SENSE_BUFFER_SIZE);
}

/*
 * ADC Conversion Complete Callback(unique function of HAL)
 * @param
 * @return
 * @note	for debugging
 */
/*
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
	printf("%d %d %d\n", current_sense_data[0], current_sense_data[1], current_sense_data[2]);
}
*/

/*
 * Enable BLDC Motor(Enable Gate Driver & Start TIM8 PWM Generation)
 * @param
 * @return
 */
void BLDCEnable(void) {
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);		//EN_GATE = HIGH
	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 0);
	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 0);
	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, 0);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
}

/*
 * Disable BLDC Motor(Disable Gate Driver & Stop TIM8 PWM Generation)
 * @param
 * @return
 */
void BLDCDisable(void) {
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);	//EN_GATE = LOW
	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 0);
	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 0);
	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, 0);
	HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_2);
	HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_3);
}

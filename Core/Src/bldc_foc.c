/*
 * bldc_foc.c
 *
 *  Created on: Dec 23, 2020
 *      Author: Ryohei
 */

#include "bldc_foc.h"
#include "as5147.h"
#include <math.h>
#include <stdio.h>

//private define
#define ADC_CURT_SENSE_BUFFER_SIZE	((uint32_t)3)

//static variables
static uint16_t curt_sense_data[ADC_CURT_SENSE_BUFFER_SIZE] = {};

void BLDCVqConstControl(float vol_d, float vol_q) {
	float curt_u, curt_v, curt_w;	//Iu, Iv, Iw current[A]
	float curt_alpha, curt_beta; 	//I_alpha, I_beta current[A]
    float curt_d, curt_q;			//Id, Iq current[A]

    volatile float vol_u, vol_v, vol_w;
    volatile float vol_alpha, vol_beta;

    float theta, sinth, costh;
	uint16_t theta_data;

	theta_data = (AS5147Read(AS5147_ANGLECOM) & 0x3FFF);		//mask lower 14bit
	theta = fmodf(((float)theta_data + ((float)0x3FFF / 12) - 361), ((float)0x3FFF / 12)) * ((float)(2 * M_PI * 12) / 0x3FFF);
	sinth = sinf(theta);
	costh = cosf(theta);

	//dq -> alpha,beta
	vol_alpha = vol_d * costh - vol_q * sinth;
	vol_beta  = vol_d * sinth + vol_q * costh;

	//alpha,beta -> UVW
	vol_u = 0.81649658f * vol_alpha;
    vol_v = -0.40824829 * vol_alpha + 0.707106781 * vol_beta;
    vol_w = -0.40824829 * vol_alpha - 0.707106781 * vol_beta;

    /*
    vol_u = fmaxf(fminf(1249 - (vol_u + 6.0f) * 624.0f / 12.0f, 1249.0f), 0);
    vol_v = fmaxf(fminf(1249 - (vol_v + 6.0f) * 624.0f / 12.0f, 1249.0f), 0);
    vol_w = fmaxf(fminf(1249 - (vol_w + 6.0f) * 624.0f / 12.0f, 1249.0f), 0);
    */
    vol_u = fmaxf(fminf(624.0f + vol_u * 1249.0f / 12.0f, 1249.0f), 0);
    vol_v = fmaxf(fminf(624.0f + vol_v * 1249.0f / 12.0f, 1249.0f), 0);
    vol_w = fmaxf(fminf(624.0f + vol_w * 1249.0f / 12.0f, 1249.0f), 0);

//    printf("%f %f %f %f\n", theta, vol_u, vol_v, vol_w);

    //output PWM
	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, vol_u);
	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, vol_v);
	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, vol_w);
}

/*
 * Start current sensing using DMA transfer
 * @param
 * @return
 * @note	Triggered by tim8(at the center of center aligned PWM.
 * 			Data will be automatically transfered to the variable by DMA
 */
void BLDCStartCurrentSense(void) {
	HAL_ADC_Start_DMA(&hadc1, (uint32_t *)curt_sense_data, ADC_CURT_SENSE_BUFFER_SIZE);
}

/*
 * ADC Conversion Complete Callback(unique function of HAL)
 * @param
 * @return
 * @note	for debugging
 */
/*
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
	printf("%d %d %d\n", curt_sense_data[0], curt_sense_data[1], curt_sense_data[2]);
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

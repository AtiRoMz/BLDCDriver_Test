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

int32_t idx = 0;
float g_curt[1000][6];

//private define
#define ADC_CURT_SENSE_BUFFER_SIZE	((uint32_t)3)
#define ADC_MOV_AVE_SIZE			5

//static variables
static uint16_t curt_sense_data[ADC_CURT_SENSE_BUFFER_SIZE] = {};
static uint16_t curt_sense_data_log[ADC_CURT_SENSE_BUFFER_SIZE][ADC_MOV_AVE_SIZE] = {};
static uint32_t curt_sense_data_ave[ADC_CURT_SENSE_BUFFER_SIZE] = {};
static uint32_t curt_sense_data_offset[ADC_CURT_SENSE_BUFFER_SIZE] = {};

void BLDCVqConstControl(float vol_d, float vol_q) {
	float curt_u, curt_v, curt_w;	//Iu, Iv, Iw current[A]
	float curt_alpha, curt_beta; 	//I_alpha, I_beta current[A]
    float curt_d, curt_q;			//Id, Iq current[A]

    static volatile float vol_u, vol_v, vol_w;
    volatile float vol_alpha, vol_beta;
    const float pwm_max  	= __HAL_TIM_GET_AUTORELOAD(&htim8);
    const float pwm_half 	= __HAL_TIM_GET_AUTORELOAD(&htim8) / 2.0f;
    const float pwm_per_vol = __HAL_TIM_GET_AUTORELOAD(&htim8) / 18.0f;		//TODO:18.0 -> measured voltage

    float theta, sinth, costh;
	uint16_t theta_data;

	theta_data = (AS5147Read(AS5147_ANGLECOM) & 0x3FFF);		//mask lower 14bit
	theta = fmodf(((float)theta_data + ((float)0x3FFF / 12) - 361), ((float)0x3FFF / 12)) * ((float)(2 * M_PI * 12) / 0x3FFF);
	sinth = sinf(theta);
	costh = cosf(theta);

	for (int8_t i = 0; i < ADC_CURT_SENSE_BUFFER_SIZE; i++) {
		curt_sense_data_ave[i] = 0;
		for (int16_t j = 0; j < ADC_MOV_AVE_SIZE; j++) {
			curt_sense_data_ave[i] += curt_sense_data_log[i][j];
		}
		curt_sense_data_ave[i] /= ADC_MOV_AVE_SIZE;
	}

	//current control
	curt_u = ((int)curt_sense_data_ave[0] - (int)curt_sense_data_offset[0]) * 3.3f / 4096.0f * 25.0f;	//TODO:use amplifier gain
	curt_v = ((int)curt_sense_data_ave[1] - (int)curt_sense_data_offset[1]) * 3.3f / 4096.0f * 25.0f;
	curt_w = ((int)curt_sense_data_ave[2] - (int)curt_sense_data_offset[2]) * 3.3f / 4096.0f * 25.0f;
	/*
	if 		(vol_u >= vol_v && vol_u >= vol_w) {curt_u = -curt_v - curt_w;}
	else if (vol_v >= vol_w && vol_v >= vol_u) {curt_v = -curt_w - curt_u;}
	else if (vol_w >= vol_u && vol_w >= vol_v) {curt_w = -curt_u - curt_v;}
	*/

	//current UVW -> alpha,beta
	curt_alpha = 0.8169496580928f * (curt_u - 0.5 * (curt_v + curt_w));
	curt_beta  = 0.7071067811866f * (curt_v - curt_w);

	//current alpha,beta -> dq
	curt_d =  curt_alpha * costh + curt_beta * sinth;
	curt_q = -curt_alpha * sinth + curt_beta * costh;


	//Vd,Vq control
	//dq -> alpha,beta
	vol_alpha = vol_d * costh - vol_q * sinth;
	vol_beta  = vol_d * sinth + vol_q * costh;

	//alpha,beta -> UVW
	vol_u =  0.81649658f * vol_alpha;
    vol_v = -0.40824829f * vol_alpha + 0.707106781 * vol_beta;
    vol_w = -0.40824829f * vol_alpha - 0.707106781 * vol_beta;

	if (5000 <= idx && idx < 6000) {
		g_curt[idx - 5000][0] = vol_u;
		g_curt[idx - 5000][1] = vol_v;
		g_curt[idx - 5000][2] = vol_w;
		g_curt[idx - 5000][3] = curt_u;
		g_curt[idx - 5000][4] = curt_v;
		g_curt[idx - 5000][5] = curt_w;
		idx++;
	} else if (idx < 5000) {
		idx++;
	}

    vol_u = fmaxf(fminf(pwm_half + vol_u * pwm_per_vol, pwm_max), 0);
    vol_v = fmaxf(fminf(pwm_half + vol_v * pwm_per_vol, pwm_max), 0);
    vol_w = fmaxf(fminf(pwm_half + vol_w * pwm_per_vol, pwm_max), 0);

    //output PWM
	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, (uint16_t)vol_u);
	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, (uint16_t)vol_v);
	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, (uint16_t)vol_w);
}

/*
 * ADC Conversion Complete Callback(unique function of HAL)
 * @param
 * @return
 * @note	for debugging
 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
	static int16_t index = 0;
	for (int8_t i = 0; i < ADC_CURT_SENSE_BUFFER_SIZE; i++) {
		curt_sense_data_log[i][index] = curt_sense_data[i];
	}
	index = (index + 1) % ADC_MOV_AVE_SIZE;

//	printf("%d %d %d\n", curt_sense_data[0], curt_sense_data[1], curt_sense_data[2]);
}

void BLDCGetCurrentSenseOffset(void) {
	const int32_t num_offset = 100;

 	curt_sense_data_offset[0] = curt_sense_data_offset[1] = curt_sense_data_offset[2] = 0;
 	for (int32_t i = 0; i < num_offset; i++) {
 		curt_sense_data_offset[0] += curt_sense_data[0];
 		curt_sense_data_offset[1] += curt_sense_data[1];
 		curt_sense_data_offset[2] += curt_sense_data[2];
 		for (volatile int32_t j = 0; j < 1000; j++) {;}		//short wait
 	}
 	curt_sense_data_offset[0] = (int)(curt_sense_data_offset[0] / num_offset + 0.50f);	//ROUND(curt_sense_data_offset / num_offset)
 	curt_sense_data_offset[1] = (int)(curt_sense_data_offset[1] / num_offset + 0.50f);
 	curt_sense_data_offset[2] = (int)(curt_sense_data_offset[2] / num_offset + 0.50f);
 	printf("%d %d %d\n", (int)curt_sense_data_offset[0], (int)curt_sense_data_offset[1], (int)curt_sense_data_offset[2]);
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
 * Enable BLDC Motor(Enable Gate Driver & Start TIM8 PWM Generation)
 * @param
 * @return
 */
void BLDCEnable(void) {
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);		//EN_GATE = HIGH
	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 0);
	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 0);
	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, 0);
	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, __HAL_TIM_GET_AUTORELOAD(&htim8) - 1);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);
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
	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, __HAL_TIM_GET_AUTORELOAD(&htim8) - 1);
	HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_2);
	HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_3);
	HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_4);
}

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
float g_curt[3000];

//private define
#define ADC_CURT_SENSE_BUFFER_SIZE	((uint32_t)3)

//static variables
static uint16_t curt_sense_data[ADC_CURT_SENSE_BUFFER_SIZE] = {};
static float curt_sense_data_offset[ADC_CURT_SENSE_BUFFER_SIZE] = {};

void BLDCVqConstControl(float vol_d, float vol_q) {
	float curt_u, curt_v, curt_w;	//Iu, Iv, Iw current[A]
	float curt_alpha, curt_beta; 	//I_alpha, I_beta current[A]
    float curt_d, curt_q;			//Id, Iq current[A]

    static volatile float vol_u, vol_v, vol_w, vol_max;
    static volatile float vol_alpha, vol_beta;
    const float pwm_max = __HAL_TIM_GET_AUTORELOAD(&htim8);

    float theta, sinth, costh;
	uint16_t theta_data;

	theta_data = (AS5147Read(AS5147_ANGLECOM) & 0x3FFF);		//mask lower 14bit
	theta = fmodf(((float)theta_data + ((float)0x3FFF / 12) - 361), ((float)0x3FFF / 12)) * ((float)(2 * M_PI * 12) / 0x3FFF);
	sinth = sinf(theta);
	costh = cosf(theta);

	//current control
	curt_u = (1.65f - ((float)curt_sense_data[0] - curt_sense_data_offset[0]) * 3.3f / 4096.0f) * 12.5f;
	curt_v = (1.65f - ((float)curt_sense_data[1] - curt_sense_data_offset[1]) * 3.3f / 4096.0f) * 12.5f;
	curt_w = (1.65f - ((float)curt_sense_data[2] - curt_sense_data_offset[2]) * 3.3f / 4096.0f) * 12.5f;
	/*
	if 		(vol_u >= vol_v && vol_u >= vol_w) {curt_u = -curt_v - curt_w;}
	else if (vol_v >= vol_w && vol_v >= vol_u) {curt_v = -curt_w - curt_u;}
	else if (vol_w >= vol_u && vol_w >= vol_v) {curt_w = -curt_u - curt_v;}
	*/
	if (idx < 3000) {
		g_curt[idx] = curt_u + curt_v + curt_w;
		idx++;
	}

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

    vol_u = fmaxf(fminf(pwm_max / 2.0f + vol_u * pwm_max / 12.0f, pwm_max), 0);
    vol_v = fmaxf(fminf(pwm_max / 2.0f + vol_v * pwm_max / 12.0f, pwm_max), 0);
    vol_w = fmaxf(fminf(pwm_max / 2.0f + vol_w * pwm_max / 12.0f, pwm_max), 0);

//    printf("%f %f %f %f\n", theta, vol_u, vol_v, vol_w);

    //output PWM
	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, (uint16_t)vol_u);
	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, (uint16_t)vol_v);
	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, (uint16_t)vol_w);
}

void BLDCGetCurrentSenseOffset(void) {
	const int32_t num_offset = 10000;

	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, __HAL_TIM_GET_AUTORELOAD(&htim8) - 1);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);
	HAL_Delay(10);

 	curt_sense_data_offset[0] = curt_sense_data_offset[1] = curt_sense_data_offset[2] = 0;
 	for (int32_t i = 0; i < num_offset; i++) {
 		curt_sense_data_offset[0] += curt_sense_data[0];
 		curt_sense_data_offset[1] += curt_sense_data[1];
 		curt_sense_data_offset[2] += curt_sense_data[2];
 	}
 	curt_sense_data_offset[0] /= num_offset;
 	curt_sense_data_offset[1] /= num_offset;
 	curt_sense_data_offset[2] /= num_offset;
 	printf("%f %f %f\n", curt_sense_data_offset[0], curt_sense_data_offset[1], curt_sense_data_offset[2]);
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

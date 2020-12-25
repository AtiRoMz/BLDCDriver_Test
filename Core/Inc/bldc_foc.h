/*
 * bldc_foc.h
 *
 *  Created on: Dec 23, 2020
 *      Author: Ryohei
 */

#ifndef INC_BLDC_FOC_H_
#define INC_BLDC_FOC_H_

#include "main.h"
#include "tim.h"
#include "spi.h"
#include "adc.h"

extern int32_t idx;
extern float g_curt[1000][6];

//prototype declaration
void BLDCVqConstControl(float, float);
void BLDCGetCurrentSenseOffset(void);
void BLDCStartCurrentSense(void);
void BLDCEnable(void);
void BLDCDisable(void);

#endif /* INC_BLDC_FOC_H_ */

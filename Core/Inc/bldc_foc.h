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

//prototype declaration
void BLDCStartCurrentSense(void);
void BLDCEnable(void);
void BLDCDisable(void);

#endif /* INC_BLDC_FOC_H_ */

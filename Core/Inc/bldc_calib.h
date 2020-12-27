/*
 * bldc_calib.h
 *
 *  Created on: Dec 16, 2020
 *      Author: Ryohei
 */

#ifndef INC_BLDC_CALIB_H_
#define INC_BLDC_CALIB_H_

#include "main.h"
#include "tim.h"

typedef enum {
	BLDC_UtoV = 0,
	BLDC_UtoW,
	BLDC_VtoW,
	BLDC_VtoU,
	BLDC_WtoU,
	BLDC_WtoV,
} bldc_conduction_phase;

//prototype declaration
void BLDCCalibZeroPos(void);
void BLDC120DegConduction(bldc_conduction_phase, float);
void BLDCFree(void);

#endif /* INC_BLDC_CALIB_H_ */

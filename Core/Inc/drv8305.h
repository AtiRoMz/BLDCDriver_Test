/*
 * drv8305.h
 *
 *  Created on: 2020/12/23
 *      Author: Ryohei
 */

#ifndef INC_DRV8305_H_
#define INC_DRV8305_H_

#include "main.h"
#include "spi.h"

//SPI Handler
#define DRV8305_SPI_HANDLER		hspi2

//CS Pin
#define DRV8305_CS_PORT			GPIOB
#define DRV8305_CS_PIN			GPIO_PIN_12

//prototype declaration
void DRV8305Init(void);
uint16_t DRV8305Write(uint8_t, uint16_t);
uint16_t DRV8305Read(uint8_t);

//register table
//status registers(R)
#define DRV8305_WARNING_WATCHDOG_RESET		0x01
#define DRV8305_OV_VDS_FAULT				0x02
#define DRV8305_IC_FAULT					0x03
#define DRV8305_VGS_FAULT					0x04
//control registers(R/W)
#define DRV8305_HS_GATE_DRIVER_CONTROL		0x05
#define DRV8305_LS_GATE_DRIVER_CONTROL		0x06
#define DRV8305_GATE_DRIVER_CONTROL			0x07
#define DRV8305_IC_OPERATION				0x09
#define DRV8305_SHUNT_AMP_CONTROL			0x0A
#define DRV8305_VOL_REG_CONTROL				0x0B
#define DRV8305_VDS_SENSE_CONTROL			0x0C

#endif /* INC_DRV8305_H_ */

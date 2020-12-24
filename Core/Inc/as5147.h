/*
 * as5147.h
 *
 *  Created on: Dec 14, 2020
 *      Author: Ryohei
 */

#ifndef INC_AS5147_H_
#define INC_AS5147_H_

#include "main.h"
#include "spi.h"

//SPI Handler
#define AS5147_SPI_HANDLER		hspi3

//CS Pin
#define AS5147_CS_PORT			GPIOA
#define AS5147_CS_PIN			GPIO_PIN_15

//prototype declaration
float AS5147GetAngle(void);
uint16_t AS5147Read(uint16_t);

//register table
//volatile
#define AS5147_NOP				0x0000
#define AS5147_ERREF			0x0001
#define AS5147_PROG				0x0003
#define AS5147_DIAAGC			0x3FFC
#define AS5147_MAG				0x3FFD
#define AS5147_ANGLEUNC			0x3FFE
#define AS5147_ANGLECOM			0x3FFF
//non-volatile
#define AS5147_ZPOSM			0x0016
#define AS5147_ZPOSL			0x0017
#define AS5147_SETTINGS1		0x0018
#define AS5147_SETTINGS2		0x0019
#define AS5147_RED				0x001A


#endif /* INC_AS5147_H_ */

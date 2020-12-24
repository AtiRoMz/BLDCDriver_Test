/*
 * as5147.c
 *
 *  Created on: Dec 14, 2020
 *      Author: Ryohei
 */

#include "as5147.h"
#include <stdio.h>
#include <math.h>

//private function declaration
static void AS5147Select(void);
static void AS5147Deselect(void);
static uint16_t AS5147AddParityBit(uint16_t);

float AS5147GetAngle(void) {
	uint16_t angle_data;
	angle_data = AS5147Read(AS5147_ANGLECOM) & 0x3FFF;		//mask lower 14bit
	return ((float)angle_data * 2 * M_PI) / 0x3FFF;			//2pi = 14bit
}

/*
 * Read Register
 * @param	address : AS5147 regigster address
 * @return	16 bit data
 */
uint16_t AS5147Read(uint16_t address) {
	uint8_t txdata[2] = {((AS5147AddParityBit(address | 0x4000) & 0xFF00) >> 8), (AS5147AddParityBit(address | 0x4000) & 0x00FF)};
	uint8_t rxdata[2] = {};
	AS5147Select();
	HAL_SPI_TransmitReceive(&AS5147_SPI_HANDLER, txdata, rxdata, 2, 1);
	AS5147Deselect();
	return ((rxdata[0] << 8) | rxdata[1]);
}

/*
 * Select
 * @param
 * @return
 */
static void AS5147Select(void) {
	HAL_GPIO_WritePin(AS5147_CS_PORT, AS5147_CS_PIN, GPIO_PIN_RESET);
}

/*
 * Deselect
 * @param
 * @return
 */
static void AS5147Deselect(void) {
	HAL_GPIO_WritePin(AS5147_CS_PORT, AS5147_CS_PIN, GPIO_PIN_SET);
}

/*
 * Parity Calculation
 * @param	raw_data
 * @return	parity bit + raw_data
 */
static uint16_t AS5147AddParityBit(uint16_t raw_data) {
	uint16_t parity = raw_data;
	parity ^= parity >> 8;
	parity ^= parity >> 4;
	parity ^= parity >> 2;
	parity ^= parity >> 1;

	return (raw_data | ((parity & 0x0001) << 15));
}

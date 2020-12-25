/*
 * drv8305.c
 *
 *  Created on: 2020/12/23
 *      Author: Ryohei
 */

#include "drv8305.h"
#include "bldc_foc.h"

//private function declaration
static void DRV8305Select(void);
static void DRV8305Deselect(void);

/*
 * Initialize DRV8305
 * @param
 * @return
 */
void DRV8305Init(void) {
	DRV8305Write(DRV8305_GATE_DRIVER_CONTROL, 0x0296);	//6 PWM mode -> 3 PWM mode
	DRV8305Write(DRV8305_SHUNT_AMP_CONTROL, 0x072A);	//Current Amp : x10 -> x40, current sense calibration on
	BLDCGetCurrentSenseOffset();						//get current sense offset(calibration)
	DRV8305Write(DRV8305_SHUNT_AMP_CONTROL, 0x002A);	//Current Amp : x10 -> x40, current sense calibration off
}

/*
 * Write Register
 * @param	address : DRV8305 regigster address(8bit)
 * 			data : data to write(11bit)
 * @return
 */
uint16_t DRV8305Write(uint8_t address, uint16_t data) {
	uint16_t adr_data = (address << 11) | (data & 0x07FF);		//0(1bit) + address(4bit) + data(11bit) = 16bit
	uint8_t txdata[2] = {adr_data >> 8, adr_data & 0xFF};		//{msb8bit, lsb8bit}
	uint8_t rxdata[2] = {};
	DRV8305Select();
	HAL_SPI_TransmitReceive(&DRV8305_SPI_HANDLER, txdata, rxdata, 2, 1);
	DRV8305Deselect();
	return (((rxdata[0] & 0x07) << 8) | rxdata[1]);
}

/*
 * Read Register
 * @param	address : DRV8305 regigster address(8bit)
 * @return	data in the designated address(11bit)
 */
uint16_t DRV8305Read(uint8_t address) {
	uint8_t txdata[2] = {(address << 3) | 0x80, 0x00};
	uint8_t rxdata[2] = {};
	DRV8305Select();
	HAL_SPI_TransmitReceive(&DRV8305_SPI_HANDLER, txdata, rxdata, 2, 1);
	DRV8305Deselect();
	return (((rxdata[0] & 0x07) << 8) | rxdata[1]);
}

/*
 * Select
 * @param
 * @return
 */
static void DRV8305Select(void) {
	HAL_GPIO_WritePin(DRV8305_CS_PORT, DRV8305_CS_PIN, GPIO_PIN_RESET);
}

/*
 * Deselect
 * @param
 * @return
 */
static void DRV8305Deselect(void) {
	HAL_GPIO_WritePin(DRV8305_CS_PORT, DRV8305_CS_PIN, GPIO_PIN_SET);
}

/**
	*
	*	@file  : eeprom.h
	*	@brief : Simple library that uses HAL-s I2C functions and adds necessary delay defined in datasheet
	*	@note	 : DEV_ADDR and DEV_DELAY should be edited with values given in the device datasheet
	*
	*/


#include "stm32f1xx_hal.h"

#define DEV_ADDR 0xA0
#define DEV_DELAY 6

extern  I2C_HandleTypeDef hi2c1;

extern uint8_t	data[65];

void EEPROM_Write (uint16_t page, uint16_t offset, uint8_t *data, uint16_t size);
void EEPROM_PageErase (uint16_t page);
void EEPROM_Read (uint16_t page, uint16_t offset, uint8_t *data, uint16_t size);

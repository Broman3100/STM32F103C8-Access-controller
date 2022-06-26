#include "eeprom.h"


void EEPROM_Write (uint16_t page, uint16_t offset, uint8_t *data, uint16_t size)
{
		HAL_I2C_Mem_Write(&hi2c1, DEV_ADDR, page<<6 | offset, 2, data, size, 1000);  // write the data to the EEPROM
		HAL_Delay(DEV_DELAY);
}

void EEPROM_PageErase (uint16_t page)
{
		HAL_I2C_Mem_Write(&hi2c1, DEV_ADDR, page<<6, 2, data, 64, 1000);
		HAL_Delay(DEV_DELAY); 
}

void EEPROM_Read (uint16_t page, uint16_t offset, uint8_t *data, uint16_t size)
{
		HAL_I2C_Mem_Read(&hi2c1, DEV_ADDR, page<<6 | offset, 2, data, size, 1000);
}

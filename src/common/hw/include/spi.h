/*
 * i2c.h
 *
 *  Created on: 2020. 12. 20.
 *      Author: WANG
 */

#ifndef SRC_COMMON_HW_INCLUDE_SPI_H_
#define SRC_COMMON_HW_INCLUDE_SPI_H_

#include "hw_def.h"


#ifdef _USE_HW_SPI

bool spiInit(void);
bool SPI_ByteRead(uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size);
void I2C_Write(uint16_t DevAddress, uint8_t data, uint16_t Size);
void I2C_Read(uint16_t DevAddress, uint8_t *pData, uint16_t Size);
bool I2C_ByteWrite(uint8_t DevAddress, uint8_t MemAddress, uint8_t bitStart, uint8_t length, uint8_t data);
void I2C_BitWrite(uint8_t DevAddress, uint8_t MemAddress, uint8_t bitNum, uint8_t data);


#endif

#endif /* SRC_COMMON_HW_INCLUDE_I2C_H_ */

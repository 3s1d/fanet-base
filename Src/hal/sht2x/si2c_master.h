/*
 * i2c_master.h
 *
 *  Created on: 25 Sep 2019
 *      Author: sid
 */

#ifndef HAL_SHT2X_SI2C_MASTER_H_
#define HAL_SHT2X_SI2C_MASTER_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>

typedef struct
{
	GPIO_TypeDef* sdaPort;
	GPIO_TypeDef* sclPort;
	uint16_t sdaPin;
	uint16_t sclPin;
} si2c_t;

si2c_t si2cInit(GPIO_TypeDef* sdaPort, uint16_t sdaPin, GPIO_TypeDef* sclPort, uint16_t sclPin);
bool si2cWrite(si2c_t *si2c, uint8_t device, uint8_t addr, uint8_t *data, uint16_t len);
bool si2cRead(si2c_t *si2c, uint8_t device, uint8_t addr, uint8_t addrLen, uint8_t *data, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif /* HAL_SHT2X_SI2C_MASTER_H_ */

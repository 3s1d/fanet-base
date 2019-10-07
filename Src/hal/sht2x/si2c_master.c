/*
 * i2c_master.c
 *
 *  Created on: 25 Sep 2019
 *      Author: sid
 */

#include <stdbool.h>

#include "cmsis_os.h"

#include "main.h"

#include "../sht2x/si2c_master.h"


//note: private helper do not feature a null ptr check
//note2: GPIO_PIN_SET -> hi-z, GPIO_PIN_RESET -> low
//note3: does not yet support busy port indication by slaves (SCL kept low)

void si2cDelay_half(void)
{
	asm("nop");
	asm("nop");
	asm("nop");
	asm("nop");
	asm("nop");
}

void si2cDelay(void)
{
	si2cDelay_half();
	si2cDelay_half();
}

si2c_t si2cInit(GPIO_TypeDef* sdaPort, uint16_t sdaPin, GPIO_TypeDef* sclPort, uint16_t sclPin)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	si2c_t i2c = {.sdaPort = sdaPort, .sdaPin = sdaPin, .sclPort = sclPort, .sclPin = sclPin};

	GPIO_InitStructure.Pin = sdaPin;
	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_OD;
	GPIO_InitStructure.Pull = GPIO_PULLUP;
	GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(sdaPort, &GPIO_InitStructure);

	GPIO_InitStructure.Pin = sclPin;
	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_OD;
	GPIO_InitStructure.Pull = GPIO_PULLUP;
	GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(sclPort, &GPIO_InitStructure);

	return i2c;
}

void si2cStart(si2c_t *si2c)
{
	HAL_GPIO_WritePin(si2c->sdaPort, si2c->sdaPin, GPIO_PIN_SET);
	si2cDelay();
	HAL_GPIO_WritePin(si2c->sclPort, si2c->sclPin, GPIO_PIN_SET);
	si2cDelay();
	HAL_GPIO_WritePin(si2c->sdaPort, si2c->sdaPin, GPIO_PIN_RESET);
	si2cDelay();
	HAL_GPIO_WritePin(si2c->sclPort, si2c->sclPin, GPIO_PIN_RESET);
	si2cDelay();
}

void si2cStop(si2c_t *si2c)
{
	HAL_GPIO_WritePin(si2c->sdaPort, si2c->sdaPin, GPIO_PIN_RESET);
	si2cDelay();
	HAL_GPIO_WritePin(si2c->sclPort, si2c->sclPin, GPIO_PIN_SET);
	si2cDelay();
	HAL_GPIO_WritePin(si2c->sdaPort, si2c->sdaPin, GPIO_PIN_SET);
	si2cDelay();
}

bool si2cWaitAck(si2c_t *si2c)
{
	HAL_GPIO_WritePin(si2c->sdaPort, si2c->sdaPin, GPIO_PIN_SET);
	si2cDelay_half();
	HAL_GPIO_WritePin(si2c->sclPort, si2c->sclPin, GPIO_PIN_SET);
	si2cDelay_half();
	uint16_t loop = 0;
	while(HAL_GPIO_ReadPin(si2c->sdaPort, si2c->sdaPin) == GPIO_PIN_SET)
	{
		if(++loop > 254)
		{
			si2cStop(si2c);
			return false;
		}
		osThreadYield();
	}
	HAL_GPIO_WritePin(si2c->sclPort, si2c->sclPin, GPIO_PIN_RESET);
	si2cDelay_half();
	return true;
}

void si2cAck(si2c_t *si2c, bool ack)
{
	HAL_GPIO_WritePin(si2c->sclPort, si2c->sclPin, GPIO_PIN_RESET);
	si2cDelay_half();
	HAL_GPIO_WritePin(si2c->sdaPort, si2c->sdaPin, ack ? GPIO_PIN_RESET : GPIO_PIN_SET);
	si2cDelay_half();
	HAL_GPIO_WritePin(si2c->sclPort, si2c->sclPin, GPIO_PIN_SET);
	si2cDelay();
	HAL_GPIO_WritePin(si2c->sclPort, si2c->sclPin, GPIO_PIN_RESET);
}

void si2cTxByte(si2c_t *si2c, uint8_t txd)
{
	HAL_GPIO_WritePin(si2c->sclPort, si2c->sclPin, GPIO_PIN_RESET);
	for (uint8_t bit=0; bit<8; bit++)
	{
		HAL_GPIO_WritePin(si2c->sdaPort, si2c->sdaPin, ((txd & 0x80) == 0x80) ? GPIO_PIN_SET : GPIO_PIN_RESET);
		txd <<= 1;

		si2cDelay_half();
		HAL_GPIO_WritePin(si2c->sclPort, si2c->sclPin, GPIO_PIN_SET);
		si2cDelay_half();
		HAL_GPIO_WritePin(si2c->sclPort, si2c->sclPin, GPIO_PIN_RESET);
	}
}

uint8_t si2cRxByte(si2c_t *si2c, bool ack)
{
	uint8_t rxd = 0;
	HAL_GPIO_WritePin(si2c->sdaPort, si2c->sdaPin, GPIO_PIN_SET);
	for (uint8_t i=0; i<8; i++)
	{
		HAL_GPIO_WritePin(si2c->sclPort, si2c->sclPin, GPIO_PIN_RESET);
		si2cDelay_half();
		HAL_GPIO_WritePin(si2c->sclPort, si2c->sclPin, GPIO_PIN_SET);
		si2cDelay_half();
		rxd <<= 1;
		rxd |= HAL_GPIO_ReadPin(si2c->sdaPort, si2c->sdaPin);
	}

	si2cAck(si2c, ack);
	return rxd;
}

bool si2cWrite(si2c_t *si2c, uint8_t device, uint8_t addr, uint8_t *data, uint16_t len)
{
	if(si2c == NULL)
		return false;

	si2cStart(si2c);

	si2cTxByte(si2c, device);
	if(si2cWaitAck(si2c) == false)
	{
		si2cStop(si2c);
		return false;
	}

	si2cTxByte(si2c, addr);
	if(si2cWaitAck(si2c) == false)
	{
		si2cStop(si2c);
		return false;
	}


	for(uint16_t i=0; i<len && data != NULL; i++)
	{
		si2cTxByte(si2c, data[i]);
		if(si2cWaitAck(si2c) == false)
		{
			si2cStop(si2c);
			return false;
		}
	}

	si2cStop(si2c);
	return true;
}

//note: currently only supports 0 and 1 byte address length
bool si2cRead(si2c_t *si2c, uint8_t device, uint8_t addr, uint8_t addrLen,  uint8_t *data, uint16_t len)
{
	if(si2c == NULL || data == NULL)
		return false;

	if(addrLen > 0)
	{
		si2cStart(si2c);

		si2cTxByte(si2c, device);
		if(si2cWaitAck(si2c) == false)
		{
			si2cStop(si2c);
			return false;
		}

		si2cTxByte(si2c, addr);
		if(si2cWaitAck(si2c) == false)
		{
			si2cStop(si2c);
			return false;
		}
	}

	si2cStart(si2c);
	si2cTxByte(si2c, device | 1);					//read bit
	if(si2cWaitAck(si2c) == false)
	{
		si2cStop(si2c);
		return false;
	}
	for(uint16_t i=0; i<len; i++)
		data[i] = si2cRxByte(si2c, i != len-1);

	si2cStop(si2c);
	return true;
}

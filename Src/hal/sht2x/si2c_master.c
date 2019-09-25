/*
 * i2c_master.c
 *
 *  Created on: 25 Sep 2019
 *      Author: sid
 */

#include <stdbool.h>
#include <stdio.h>		//tbr

#include "cmsis_os.h"

#include "main.h"

#include "../sht2x/si2c_master.h"


//note: private helper do not feature a null ptr check
//note2: GPIO_PIN_SET -> hi-z, GPIO_PIN_RESET -> low

void SDA_IN(void)
{
//	GPIO_InitTypeDef GPIO_InitStruct;
//	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;    //
//	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
//	GPIO_InitStruct.GPIO_Pin = PIN_SDA;
//	GPIO_Init(I2C_PORT, &GPIO_InitStruct);
}

void SDA_OUT(void)
{
//	GPIO_InitTypeDef GPIO_InitStruct;
//	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
//	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
//	GPIO_InitStruct.GPIO_Pin = PIN_SDA;
//	GPIO_Init(I2C_PORT, &GPIO_InitStruct);
}

void si2cDelay(void)
{
	osDelay(1);
	//todo 2x half
}

void si2cDelay_half(void)
{
	osDelay(1);
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
	SDA_OUT();
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
	SDA_OUT();
	HAL_GPIO_WritePin(si2c->sdaPort, si2c->sdaPin, GPIO_PIN_RESET);
	si2cDelay();
	HAL_GPIO_WritePin(si2c->sclPort, si2c->sclPin, GPIO_PIN_SET);
	si2cDelay();
	HAL_GPIO_WritePin(si2c->sdaPort, si2c->sdaPin, GPIO_PIN_SET);
	si2cDelay();
}

bool si2cWaitAck(si2c_t *si2c)
{
	SDA_IN();
	HAL_GPIO_WritePin(si2c->sdaPort, si2c->sdaPin, GPIO_PIN_SET);
	si2cDelay_half();
	HAL_GPIO_WritePin(si2c->sclPort, si2c->sclPin, GPIO_PIN_SET);
	si2cDelay_half();
	uint16_t loop = 0;
	while(HAL_GPIO_ReadPin(si2c->sdaPort, si2c->sdaPin) == GPIO_PIN_SET)
	{
		if(++loop > 1000)									//todo test how small we can go....
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

void si2cAck(si2c_t *si2c)
{
	SDA_OUT();
	HAL_GPIO_WritePin(si2c->sdaPort, si2c->sdaPin, GPIO_PIN_RESET);
	si2cDelay_half();
	HAL_GPIO_WritePin(si2c->sclPort, si2c->sclPin, GPIO_PIN_SET);
	si2cDelay_half();
	HAL_GPIO_WritePin(si2c->sclPort, si2c->sclPin, GPIO_PIN_RESET);
}

void si2cNack(si2c_t *si2c)
{
	HAL_GPIO_WritePin(si2c->sclPort, si2c->sclPin, GPIO_PIN_RESET);
	SDA_OUT();
	HAL_GPIO_WritePin(si2c->sdaPort, si2c->sdaPin, GPIO_PIN_SET);
	si2cDelay_half();
	HAL_GPIO_WritePin(si2c->sclPort, si2c->sclPin, GPIO_PIN_SET);
	si2cDelay_half();
	HAL_GPIO_WritePin(si2c->sclPort, si2c->sclPin, GPIO_PIN_RESET);
}

void si2cTxByte(si2c_t *si2c, uint8_t txd)
{
	SDA_OUT();
	HAL_GPIO_WritePin(si2c->sclPort, si2c->sclPin, GPIO_PIN_RESET);
	for (uint8_t bit=0; bit<8; bit++)
	{
		HAL_GPIO_WritePin(si2c->sdaPort, si2c->sdaPin, ((txd & 0x80) == 0x80) ? GPIO_PIN_SET : GPIO_PIN_RESET);

		txd <<= 1;
		si2cDelay_half();
		HAL_GPIO_WritePin(si2c->sclPort, si2c->sclPin, GPIO_PIN_SET);
		si2cDelay_half();
		HAL_GPIO_WritePin(si2c->sclPort, si2c->sclPin, GPIO_PIN_RESET);
		si2cDelay_half();
	}
}

uint8_t si2cRxByte(si2c_t *si2c, bool ack)
{
	uint8_t rxd = 0;
	SDA_IN();
	HAL_GPIO_WritePin(si2c->sdaPort, si2c->sdaPin, GPIO_PIN_SET);
	for (uint8_t i=0; i<8; i++)
	{
		HAL_GPIO_WritePin(si2c->sclPort, si2c->sclPin, GPIO_PIN_RESET);
		si2cDelay_half();
		HAL_GPIO_WritePin(si2c->sclPort, si2c->sclPin, GPIO_PIN_SET);
		si2cDelay_half();
		rxd <<= 1;
		rxd |= HAL_GPIO_ReadPin(si2c->sdaPort, si2c->sdaPin);
		si2cDelay_half();
	}
	if (ack)
		si2cAck(si2c);
	else
		si2cNack(si2c);
	return rxd;
}

bool si2cWrite(si2c_t *si2c, uint8_t device, uint8_t addr, uint8_t data)
{
	if(si2c == NULL)
		return false;

	si2cStart(si2c);

	si2cTxByte(si2c, device);
	if(si2cWaitAck(si2c) == false)
{
printf("dev nack\n");
		return false;
}

	si2cTxByte(si2c, addr);
	if(si2cWaitAck(si2c) == false)
{
printf("addr nack\n");
		return false;
}

	si2cTxByte(si2c, data);
	if(si2cWaitAck(si2c) == false)
{
printf("data nack\n");					//this might be ok???
		return false;
}

	si2cStop(si2c);
	return true;
}

bool si2cRead(si2c_t *si2c, uint8_t device, uint8_t addr, uint8_t *data)
{
	if(si2c == NULL || data == NULL)
		return false;

	si2cStart(si2c);

	si2cTxByte(si2c, device);
	if(si2cWaitAck(si2c) == false)
{
printf("dev nack\n");
		return false;
}

	si2cTxByte(si2c, addr);
	if(si2cWaitAck(si2c) == false)
{
printf("addr nack\n");
		return false;
}

	si2cStart(si2c);
	si2cTxByte(si2c, device | 1);					//read bit
	if(si2cWaitAck(si2c) == false)
{
printf("dev nack\n");
		return false;
}

	*data = si2cRxByte(si2c, false);					//no ACK

	si2cStop(si2c);
	return true;
}

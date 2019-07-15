/*
 * replay.cpp
 *
 *  Created on: 13 Jul 2019
 *      Author: sid
 */

#include "stm32l4xx.h"

#include "clamp.h"
#include "common.h"

#include "../hal/wind.h"
#include "fanet.h"

#include "print.h"

bool Replay::isVaild(void)
{
	/* not initialized */
	if(payloadLength == 0 || payload == nullptr || type == FanetFrame::TYPE_NONE || type == 0xFF)
		return false;

	/* all wind situation or no sensor present */
	if(windSector == 0xFF || wind.sensorPresent == false)
		return true;

	/* don't display in case of wind and no wind landmark or no wind and wind landmark*/
	float windSpd = wind.getSpeed_2min_avg();							//kmph
	if((windSpd > 4.0f) == (windSector == 0))
		return false;

	/* check wind sector */
	float windCrs = wind.getDir_2min_avg() + 22.5f;							//[0+22.5..360+22.5]
	if(windCrs >= 360.0f)
		windCrs -= 360.0f;
	int16_t idx = static_cast<int16_t>(windCrs * 8.0f / 360.0f);
	clamp(idx, (const int16_t)0, (const int16_t)7);
	return !!(windSector & (1<<idx));
}

FanetFrame *Replay::toFrame(void)
{
	/* generate frame w/o knowing whats in */
	FanetFrame *frm = new FanetFrame();
	frm->setType(type);
	frm->forward = forward;
	frm->payloadLength = payloadLength;
	frm->payload = new uint8_t[frm->payloadLength];
	memcpy(frm->payload, payload, frm->payloadLength);

	debug_printf("RPF type%02X\n", type);
	return frm;
}

void Replay::init(uint8_t windSector, FanetFrame::FrameType_t type, bool forward, uint8_t *payload, uint8_t len)
{
	/* clean */
	if(this->payload != nullptr)
		delete this->payload;
	this->payload = nullptr;
	this->type = FanetFrame::TYPE_NONE;
	this->payloadLength = 0;

	if(len > 150)									//limit so that 12 fill fit into one page
		return;

	/* header */
	this->windSector = windSector;
	this->type = type;
	this->forward = forward;
	this->payloadLength = len;

	/* payload */
	if(len == 0 || payload == nullptr)
		return;
	this->payload = new uint8_t[len];
	memcpy(this->payload, payload, this->payloadLength);

	debug_printf("RPF wind%02X type%02X fw%d len%d\n", this->windSector, this->type, this->forward, this->payloadLength);
}

bool Replay::write(uint32_t addr)
{
	/* header */
	uint64_t container = 	((uint64_t) ((uint8_t)type)) << 0 |
				((uint64_t) ((uint8_t)forward)) << 8 |
				((uint64_t) ((uint8_t)windSector)) << 16 |
				((uint64_t) ((uint8_t)payloadLength)) << 24;

	/* mark unused */
	if(payloadLength == 0 || payload == nullptr  || type == FanetFrame::TYPE_NONE)
		container = UINT64_MAX;

	if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, addr, container) != HAL_OK)
		return false;
	addr += 8;

	/* done writing */
	if(container == UINT64_MAX)
		return true;

	/* fill payload */
	uint16_t contPos = 0;
	container = 0;
	for(uint16_t i=0; i<payloadLength; i++)
	{
		container |= ((uint64_t) payload[i]) << (contPos*8);

		/* container full */
		if(++contPos == 8)
		{
			if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, addr, container) != HAL_OK)
				return false;
			addr += 8;
			container = 0;
			contPos = 0;
		}
	}
	/* flush rest */
	if(contPos != 0)
	{
		if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, addr, container) != HAL_OK)
			return false;
		addr += 8;
		container = 0;
		contPos = 0;
	}

	return true;
}

void Replay::load(uint32_t addr)
{
	__IO uint64_t *ptr = (__IO uint64_t*)addr;

	/* header correct */
	if(*ptr > 0x00000000FFFFFFFF)
	{
		init(0, FanetFrame::TYPE_NONE, false, nullptr, 0);
		return;
	}

	init((*ptr>>16)&0xFF, static_cast<FanetFrame::FrameType_t>(*ptr&0xFF), (*ptr>>8)&0xFF, (uint8_t *) (ptr+1), (*ptr>>24) & 0xFF);
}


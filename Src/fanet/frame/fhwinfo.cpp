/*
 * fhwinfo.cpp
 *
 *  Created on: 19 Oct 2019
 *      Author: sid
 */
#include <stdlib.h>

#include "common.h"

#include "../fanet.h"
#include "fhwinfo.h"

/* handle payload */
int16_t FanetFrameHwInfo::serialize(uint8_t*& buffer)
{
	payloadLength = 5;
	if(payload != nullptr)
		delete [] payload;
	payload = new uint8_t[payloadLength];
	if(payload == nullptr)
		return -1;

	payload[0] = type;
	uint16_t *datePtr = (uint16_t*)&payload[1];

	char date[3] = {'\0'};
	date[2] = 0;
	/* year */
	date[0] = BUILD[8];
	date[1] = BUILD[9];
	const uint16_t year = atoi(date) - 19;

	/* month */
	date[0] = BUILD[10];
	date[1] = BUILD[11];
	const uint16_t month = atoi(date);

	/* day */
	date[0] = BUILD[12];
	date[1] = BUILD[13];
	const uint16_t day = atoi(date);
	*datePtr = (year<<9) | (month<<5) | (day<<0);

	/* debug flag */
#ifdef DEBUG
	*datePtr |= 1<<15;
#else
	*datePtr &= 0x7FFF;
#endif

	uint16_t *timePtr = (uint16_t*)&payload[3];
	*timePtr = ((osKernelSysTick() / 30000) << 4) | (numVolatileRPF & 0x000F);

	return FanetFrame::serialize(buffer);
}

void FanetFrameHwInfo::decode(const uint8_t *payload, const uint16_t len, bool isBroadcast)
{
	/* hw info request */
	//note: has to be unicast
	if(isBroadcast == false && len > 0 && payload != nullptr && payload[0] == 0)
		fanet.sendHwInfo();
}

/*
 * fname.cpp
 *
 *  Created on: Sep 21, 2018
 *      Author: sid
 */

#include <string.h>

#include "fname.h"

int16_t FanetFrameName::serialize(uint8_t*& buffer)
{
	const char *name = "todo";

	/* prepare storage */
	if(payload != nullptr)
		delete [] payload;
	payloadLength = strlen(name);
	payload = new uint8_t[payloadLength];

	/* copy pilot name */
	//note: zero termination not required
	memcpy(payload, name, payloadLength * sizeof(char));

	return FanetFrame::serialize(buffer);
}

void FanetFrameName::decode(const uint8_t *payload, const uint16_t len, FanetNeighbor *neighbor)
{
	if(len == 0 || payload == nullptr || neighbor == nullptr)
		return;

	neighbor->setName((const char *)payload, len);
}



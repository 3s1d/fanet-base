/*
 * fmsg.cpp
 *
 *  Created on: Sep 25, 2018
 *      Author: sid
 */

#include "print.h"

#include "fmsg.h"

FanetFrameMessage::FanetFrameMessage(const FanetMacAddr &dest, char *text) : FanetFrame(dest)
{
	payloadLength = 1;
	if(text != nullptr && strlen(text) > 0)
		payloadLength += strlen(text);
	payload = new uint8_t[payloadLength];
	payload[0] = normal;

	/* add text */
	//note: no need for zero-termination
	if(payloadLength > 0)
		memcpy(&payload[1], text, payloadLength-1);
	_type = FanetFrame::TYPE_MESSAGE;

}

/* handle payload */
int16_t FanetFrameMessage::serialize(uint8_t*& buffer)
{
	/* nothing to do here. payload already ready */
	return FanetFrame::serialize(buffer);
}

void FanetFrameMessage::decode(const uint8_t *payload, const uint16_t len, FanetNeighbor *neighbor)
{
	if(len == 0 || payload == nullptr || neighbor == nullptr)
		return;

	/* push message forward to neighbor */
	neighbor->setMessage((const char *) &payload[1], len-1);
}


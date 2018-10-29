/*
 * fgndtracking.cpp
 *
 *  Created on: Sep 27, 2018
 *      Author: sid
 */

#include "common.h"

#include "../../phy/coordinate.h"
#include "../fanet.h"
#include "fgndtracking.h"

int16_t FanetFrameGndTracking::serialize(uint8_t*& buffer)
{
	return -10;							//not implemented

#if 0
	/* prepare storage */
	if(payload != nullptr)
		delete [] payload;
	payload = new uint8_t[FANET_GROUNDTRACKING_SIZE];
	payloadLength = FANET_GROUNDTRACKING_SIZE;

	/* position */
	FanetFrame::coord2payload_absolut(flightCtl.gnss_rad, payload);

	/* state */
	payload[6] = (fanet.state&0x0F)<<4 | !!settings.onlineTracking;

	return FanetFrame::serialize(buffer);
#endif
}

void FanetFrameGndTracking::decode(const uint8_t *payload, const uint16_t len, FanetNeighbor *neighbor)
{
	if(neighbor == nullptr || payload == nullptr || len == 0)
		return;

	/* position */
	Coordinate2D pos;
	payload2coord_absolute(payload, pos);
	//note: do not overwrite pos here

	/* state */
	neighbor->setTrackingType(static_cast<FanetDef::status_t>((payload[6]>>4) & 0x0F));

	/* additional stuff */
	//note: ignoring online tracking
	if(neighbor->pos.latitude != 0.0f || neighbor->pos.longitude != 0.0f)
	{
		float dt = (osKernelSysTick() - neighbor->lastTrackUpdate) / 1000.0f;
		if(dt == 0.0f)
			dt = 0.01f;
		float dx = neighbor->pos.distanceTo(pos);
		neighbor->speed_kmh = (neighbor->speed_kmh*0.75f) + (0.25f*mps2kmph(dx/dt));
		neighbor->heading_rad = neighbor->pos.courseTo(pos);
	}
	neighbor->climb_mps = 0.0f;						//no altitude information available
	neighbor->lastTrackUpdate = osKernelSysTick();

	/* updating pos */
	neighbor->setPosition(pos);
}

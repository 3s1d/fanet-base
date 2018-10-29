/*
 * ftracking.cpp
 *
 *  Created on: Sep 20, 2018
 *      Author: sid
 */

#include "common.h"
#include "clamp.h"
#include "constrain.h"

#include "ftracking.h"

int FanetFrameTracking::payload2altitude(uint16_t *buf)
{
	int alt = *buf&0x7FF;
	if(*buf & 1<<11)
		alt*=4;

	return alt;
}

void FanetFrameTracking::altitude2payload(int altitude, uint16_t *buf)
{
	if(buf == nullptr)
		return;

	clamp(altitude, 0, 8190);

	if(altitude > 2047)
		((uint16_t*)buf)[0] = ((altitude+2)/4) | (1<<11);				//set scale factor
	else
		((uint16_t*)buf)[0] = altitude;
}

/* handle payload */
int16_t FanetFrameTracking::serialize(uint8_t*& buffer)
{
	return -10;			//not implemented
#if 0
	/* prepare storage */
	if(payload != nullptr)
		delete [] payload;
	payload = new uint8_t[FANET_TRACKING_SIZE];

	/* position */
	FanetFrame::coord2payload_absolut(flightCtl.gnss_rad, payload);

	/* altitude set the lower 12bit */
	altitude2payload((int)roundf(flightCtl.qnh_m), &((uint16_t*)payload)[3]);

	/* online tracking */
	((uint16_t*)payload)[3] |= !!settings.onlineTracking<<15;
	/* aircraft type */
	((uint16_t*)payload)[3] |= (settings.aircraft&0x7)<<12;

	/* Speed */
	int speed2 = constrain((int)roundf(flightCtl.speed_kmph*2.0f), 0, 635);
	if(speed2 > 127)
		payload[8] = ((speed2+2)/5) | (1<<7);						//set scale factor
	else
		payload[8] = speed2;

	/* Climb */
	int climb10 = constrain((int)roundf(pressure.climbAvg*10.0f), -315, 315);
	if(abs(climb10) > 63)
		payload[9] = ((climb10 + (climb10>=0?2:-2))/5) | (1<<7);			//set scale factor
	else
		payload[9] = climb10 & 0x7F;

	/* Heading */
	payload[10] = constrain((int)roundf(flightCtl.heading_rad*256.0f/360.0f), 0, 255);

#if 0	/* Turn rate */
	if(!isnan(turnrate))
	{
		int turnr4 = constrain((int)roundf(turnrate*4.0f), -255, 255);
		if(abs(turnr4) > 63)
			payload[11] = ((turnr4 + (turnr4>=0?2:-2))/4) | (1<<7);			//set scale factor
		else
			payload[11] = turnr4 & 0x7f;
		payloadLength = FANET_TRACKING_SIZE;
	}
	else
#endif
	{
		payloadLength = FANET_TRACKING_SIZE - 1;
	}

	return FanetFrame::serialize(buffer);
#endif
}

/* static */
void FanetFrameTracking::decode(const uint8_t *payload, const uint16_t len, FanetNeighbor *neighbor)
{
	if(neighbor == nullptr || payload == nullptr || len == 0)
		return;

	/* position */
	Coordinate2D pos;
	payload2coord_absolute(payload, pos);
	float alt = payload2altitude(&((uint16_t*)payload)[3]);
	neighbor->setPosition(pos, alt);

	/* aircraft */
	neighbor->setTrackingType(static_cast<FanetDef::aircraft_t>((((uint16_t*)payload)[3]>>12) & 0x7));

	/* additional stuff */
	//note: ignoring online tracking
	neighbor->lastTrackUpdate = osKernelSysTick();
	neighbor->speed_kmh = payload2ufloat(payload[8], 5.0f) / 2.0f;
	neighbor->climb_mps = payload2sfloat(payload[9], 5.0f) / 10.0f;
	neighbor->heading_rad = (((float)payload[10])/256.0f) * M_2PI_f;
}

/*
 * fframe.cpp
 *
 *  Created on: Jul 4, 2018
 *      Author: sid
 */

#include <math.h>
#include <string.h>

#include "clamp.h"
#include "common.h"
#include "print.h"

#include "fframe.h"
#include "fmac.h"
#include "frame/fhwinfo.h"
#include "frame/fmsg.h"
#include "frame/fgndtracking.h"
#include "frame/fname.h"
#include "frame/fremotecfg.h"
#include "frame/fservice.h"
#include "frame/ftracking.h"


/*
 * Frame
 */

uint16_t FanetFrame::coord2payload_compressed(float rad)
{
	const float ref_deg = rad2deg(rad);
	const float deg_round = std::round(ref_deg);
	const bool deg_odd = ((int)deg_round) & 1;
	const float decimal = ref_deg - deg_round;
	int dec_int = (int)(decimal*32767.0f);
	clamp(dec_int, -16383, 16383);

	return ((dec_int&0x7FFF) | (!!deg_odd<<15));
}

void FanetFrame::coord2payload_absolut(const Coordinate2D &coord, uint8_t *buf)
{
	if(buf == nullptr)
		return;

	int32_t lat_i = std::round(rad2deg(coord.latitude) * 93206.0f);
	int32_t lon_i = std::round(rad2deg(coord.longitude) * 46603.0f);

	buf[0] = ((uint8_t*)&lat_i)[0];
	buf[1] = ((uint8_t*)&lat_i)[1];
	buf[2] = ((uint8_t*)&lat_i)[2];

	buf[3] = ((uint8_t*)&lon_i)[0];
	buf[4] = ((uint8_t*)&lon_i)[1];
	buf[5] = ((uint8_t*)&lon_i)[2];
}

float FanetFrame::payload2coord_compressed(const uint16_t *buf, float ref)
{
	if(buf == nullptr)
		return NAN;

	/* decode buffer */
	const bool odd = !!((1<<15) & *buf);
	const int16_t sub_deg_int = (*buf&0x7FFF) | (1<<14&*buf)<<1;
	const float sub_deg = sub_deg_int / 32767.0f;


	/* retrieve coordinate */
	const float ref_deg = rad2deg(ref);
	float mycood_rounded = roundf(ref_deg);
	const bool mycoord_isodd = ((int)mycood_rounded) & 1;

	/* target outside our segment. estimate where it is in */
	if(mycoord_isodd != odd)
	{
		/* adjust deg segment */
		const float mysub_deg = ref_deg - mycood_rounded;
		if(sub_deg > mysub_deg)
			mycood_rounded--;
		else
			mycood_rounded++;
	}

	return deg2rad(mycood_rounded + sub_deg);
}

void FanetFrame::payload2coord_absolute(const uint8_t *buf, Coordinate2D &pos)
{
	if(buf == nullptr)
		return;

	/* integer values */
	int lati = buf[2]<<16 | buf[1]<<8 | buf[0];
	if(lati & 0x00800000)
		lati |= 0xFF000000;
	int loni = buf[5]<<16 | buf[4]<<8 | buf[3];
	if(loni & 0x00800000)
		loni |= 0xFF000000;

	pos.latitude = deg2rad((float)lati / 93206.0f);
	pos.longitude = deg2rad((float)loni / 46603.0f);
}

float FanetFrame::payload2ufloat(uint8_t buf, float scale)
{
	float value = (float)(buf&0x7F);
	if(buf & 1<<7)
		return value*scale;
	else
		return value;
}

float FanetFrame::payload2sfloat(uint8_t buf, float scale)
{
	int8_t value8 = (buf&0x7F) | (buf&(1<<6))<<1;
	if(buf & 1<<7)
		return ((float)value8) * scale;
	else
		return (float) value8;
}

int16_t FanetFrame::serialize(uint8_t*& buffer)
{
	if(src.id <= 0 || src.id >= 0xFFFF || src.manufacturer <= 0 || src.manufacturer>=0xFE)
		return -2;

	int blength = MAC_FRM_MIN_HEADER_LENGTH + payloadLength;

	/* extended header? */
	if(ackRequested || dest.id != 0 || dest.manufacturer != 0 || signature != 0 || geoForward)
		blength++;

	/* none broadcast frame */
	if(dest.id != 0 || dest.manufacturer != 0)
		blength += MAC_FRM_ADDR_LENGTH;

	/* signature */
	if(signature != 0)
		blength += MAC_FRM_SIGNATURE_LENGTH;

	/* frame to long */
	if(blength > 255)
		return -1;

	/* get memory */
	buffer = new uint8_t[blength];
	int idx = 0;

	/* header */
	if(geoForward)
		forward = false;
	buffer[idx++] = !!(ackRequested || dest.id != 0 || dest.manufacturer != 0 || signature != 0 || geoForward)<<MAC_FRM_HEADER_EXTHEADER_BIT |
			!!forward<<MAC_FRM_HEADER_FORWARD_BIT | (type & MAC_FRM_HEADER_TYPE_MASK);
	buffer[idx++] = src.manufacturer & 0x000000FF;
	buffer[idx++] = src.id & 0x000000FF;
	buffer[idx++] = (src.id>>8) & 0x000000FF;

	/* extended header */
	if(buffer[0] & 1<<MAC_FRM_HEADER_EXTHEADER_BIT)
		buffer[idx++] = (ackRequested & 3)<<MAC_FRM_EXTHEADER_ACK_BIT0 |
				!!(dest.id != 0 || dest.manufacturer != 0)<<MAC_FRM_EXTHEADER_UNICAST_BIT |
				!!signature<<MAC_FRM_EXTHEADER_SIGNATURE_BIT |
				!!geoForward<<MAC_FRM_EXTHEADER_GEOFORWARD_BIT;

	/* extheader and unicast -> add destination addr */
	if((buffer[0] & 1<<MAC_FRM_HEADER_EXTHEADER_BIT) && (buffer[4] & 1<<MAC_FRM_EXTHEADER_UNICAST_BIT))
	{
		buffer[idx++] = dest.manufacturer & 0x000000FF;
		buffer[idx++] = dest.id & 0x000000FF;
		buffer[idx++] = (dest.id>>8) & 0x000000FF;
	}

	/* extheader and signature -> add signature */
	if((buffer[0] & 1<<MAC_FRM_HEADER_EXTHEADER_BIT) && (buffer[4] & 1<<MAC_FRM_EXTHEADER_SIGNATURE_BIT))
	{
		buffer[idx++] = signature & 0x000000FF;
		buffer[idx++] = (signature>>8) & 0x000000FF;
		buffer[idx++] = (signature>>16) & 0x000000FF;
		buffer[idx++] = (signature>>24) & 0x000000FF;
	}

	/* fill payload */
	for(int i=0; i<payloadLength && idx<blength; i++)
		buffer[idx++] = payload[i];

	return blength;
}

void FanetFrame::decodePayload(FanetNeighbor *neighbor)
{
	//note: only decode position

	if(type == TYPE_TRACKING)
		FanetFrameTracking::decode(payload, payloadLength, neighbor);
	else if(type == TYPE_GROUNDTRACKING)
		FanetFrameGndTracking::decode(payload, payloadLength, neighbor);
	else if(type == TYPE_SERVICE)
		FanetFrameService::decode(payload, payloadLength, neighbor);
	else if(type == TYPE_NAME)
		FanetFrameName::decode(payload, payloadLength, neighbor);
	else if(type == TYPE_MESSAGE)
		FanetFrameMessage::decode(payload, payloadLength, neighbor);
	else if(type == TYPE_REMOTECONFIG)
		FanetFrameRemoteConfig::decode(this);
	else if(type == TYPE_LANDMARK)
		{ /* don't care */ }
	else if(type == TYPE_HWINFO)
		FanetFrameHwInfo::decode(payload, payloadLength, dest == FanetMacAddr());
	else
		debug_printf("unable to decode type %d\n", type);

	//debug_printf("frm%d %02x:%04x @ %.3f,%.3f,%.f\n", type, src.manufacturer, src.id,
	//		neighbor->pos.latitude, neighbor->pos.longitude, neighbor->pos.altitude);
}

void FanetFrame::requestAck(bool en)
{
	if(en)
	{
		ackRequested = forward ? FRM_ACK_TWOHOP : FRM_ACK_SINGLEHOP;
		numTx = MAC_TX_RETRANSMISSION_RETRYS;
	}
	else
	{
		ackRequested = FRM_NOACK;
		numTx = 0;
	}
}

FanetFrame::FanetFrame(int16_t length, uint8_t *data) : type(_type)
{
	int16_t payload_start = MAC_FRM_MIN_HEADER_LENGTH;

	/* header */
	forward = !!(data[0] & (1<<MAC_FRM_HEADER_FORWARD_BIT));
	_type = static_cast<FrameType_t>(data[0] & MAC_FRM_HEADER_TYPE_MASK);
	src.manufacturer = data[1];
	src.id = data[2] | (data[3]<<8);

	/* extended header */
	if(data[0] & 1<<MAC_FRM_HEADER_EXTHEADER_BIT)
	{
		payload_start++;

		/* ack type */
		ackRequested = (data[4] >> MAC_FRM_EXTHEADER_ACK_BIT0) & 3;

		/* unicast */
		if(data[4] & (1<<MAC_FRM_EXTHEADER_UNICAST_BIT))
		{
			dest.manufacturer = data[5];
			dest.id = data[6] | (data[7]<<8);

			payload_start += MAC_FRM_ADDR_LENGTH;
		}

		/* signature */
		if(data[4] & (1<<MAC_FRM_EXTHEADER_SIGNATURE_BIT))
		{
			signature = data[payload_start] | (data[payload_start+1]<<8) | (data[payload_start+2]<<16) | (data[payload_start+3]<<24);
			payload_start += MAC_FRM_SIGNATURE_LENGTH;
		}

		/* geoforward */
		geoForward = !!(data[4] & (1<<MAC_FRM_EXTHEADER_GEOFORWARD_BIT));
	}

	/* payload */
	payloadLength = length - payload_start;
	if(payloadLength > 0)
	{
		payload = new uint8_t[payloadLength];
		memcpy(payload, &data[payload_start], payloadLength);
	}
}

FanetFrame::FanetFrame(const FanetMacAddr &dest) : dest(dest), type(_type)
{
	src = fmac.addr;
	forward = true;				//always use forward for uni cast frames
}

FanetFrame::FanetFrame() : type(_type)
{
	src = fmac.addr;
}


/* copy a frame */
FanetFrame::FanetFrame(FanetFrame *frm) : type(_type)
{
	src = frm->src;
	dest = frm->dest;

	ackRequested = frm->ackRequested;
	forward = frm->forward;

	signature = frm->signature;

	_type = frm->type;
	if(frm->payloadLength > 0 && frm->payload != nullptr)
	{
		payloadLength = frm->payloadLength;
		payload = new uint8_t[payloadLength];
		memcpy(payload, frm->payload, payloadLength * sizeof(uint8_t));
	}

	numTx = frm->numTx;
	nextTx = frm->nextTx;

	rssi = frm->rssi;
}

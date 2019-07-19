/*
 * fremotecfg.cpp
 *
 *  Created on: 13 Jul 2019
 *      Author: sid
 */

#include <math.h>

#include "common.h"
#include "print.h"

#include "../../misc/sha1.h"
#include "../fanet.h"
#include "fremotecfg.h"


/* handle payload */
int16_t FanetFrameRemoteConfig::serialize(uint8_t*& buffer)
{
	return -10;							//not implemented
}

void FanetFrameRemoteConfig::decode(FanetFrame *frm)
{
	/* decode key set? */
	if(strlen(fanet.key) == 0)
		return;

	/* check signature */
	SHA1_CTX ctx;
	sha1_init(&ctx);
	BYTE hash[SHA1_BLOCK_SIZE];

	/* pseudo header */
	uint8_t phdr[4];
	phdr[0] = frm->type & 0x3F;
	phdr[1] = frm->src.manufacturer;
	phdr[2] = frm->src.id & 0xFF;
	phdr[3] = frm->src.id >> 8;
	sha1_update(&ctx, phdr, sizeof(phdr));

	/* payload */
	sha1_update(&ctx, frm->payload, frm->payloadLength);

	/* load pre shared key */
	sha1_update(&ctx, (uint8_t*) fanet.key, std::min(strlen(fanet.key), (size_t)FANET_KEY_SIZE));

	/* sign */
	sha1_final(&ctx, hash);
	uint32_t signature;
	memcpy(&signature, hash, sizeof(uint32_t));

	/* ignore incorrectly signed frames */
	if(signature != frm->signature)
		return;

	/*
	 * Evaluate payload
	 */
	bool success = false;
	if(frm->payloadLength > 0 && frm->payload[0] == FANET_RC_POSITION)
		success = position(&frm->payload[1], frm->payloadLength - 1);
	else if(frm->payloadLength > 0 && frm->payload[0] >= FANET_RC_REPLAY_LOWER && frm->payload[0] <= FANET_RC_REPLAY_UPPER)
		success = replayFeature(frm->payload[0] - FANET_RC_REPLAY_LOWER, &frm->payload[1], frm->payloadLength - 1);
	else if(frm->payloadLength > 0 && frm->payload[0] >= FANET_RC_GEOFENCE_LOWER && frm->payload[0] <= FANET_RC_GEOFENCE_UPPER)
		success = geofenceFeature(frm->payload[0] - FANET_RC_GEOFENCE_LOWER, &frm->payload[1], frm->payloadLength - 1);
	else if(frm->payloadLength > 1 && frm->payload[0] == FANET_RC_REQUST)
		request(frm->payload[1], frm->src);					//success unchanged as it'll generate a packet on it's own

	/* generate RC ACK */
	if(success)
	{
		FanetFrame *rfrm = new FanetFrame(fmac.addr);
		if(rfrm == nullptr)
			return;

		/* manually construct frame */
		rfrm->setType(FanetFrame::TYPE_REMOTECONFIG);
		rfrm->dest = frm->src;
		rfrm->forward = false;
		rfrm->ackRequested = false;
		rfrm->payloadLength = 2;
		rfrm->payload = new uint8_t[rfrm->payloadLength];
		rfrm->payload[0] = FANET_RC_ACK;
		rfrm->payload[1] = frm->payload[0];

		if(fmac.transmit(rfrm) != 0)
			delete rfrm;
	}
}

bool FanetFrameRemoteConfig::position(uint8_t *payload, uint16_t payload_length)
{
	/* remove our position */
	if(payload_length == 0)
		return fanet.writePosition(Coordinate3D(), 0.0f);

	/* to little information */
	if(payload_length < 8)
		return false;

	/* update position */
	Coordinate2D nPos;
	FanetFrame::payload2coord_absolute(payload, nPos);
	int16_t alt = (((int8_t)payload[6]) + 109) * 25;
	float head = (((float)payload[7])/256.0f) * 360.0f;
	return fanet.writePosition(Coordinate3D(nPos.latitude, nPos.longitude, alt), head);
}

bool FanetFrameRemoteConfig::replayFeature(uint16_t num, uint8_t *payload, uint16_t len)
{
	/* out of bound */
	if(num >= NELEM(fanet.replayFeature))
		return false;

	if(len < 2)
		fanet.replayFeature[num].init(0, static_cast<FanetFrame::FrameType_t>(0), false, nullptr, 0);	//remove feature
	else
		fanet.replayFeature[num].init(payload[0], static_cast<FanetFrame::FrameType_t>(payload[1]&0x3F), !!(payload[1]&0x40),
				&payload[2], len-2);								// copy feature

	return fanet.writeReplayFeatures();
}

bool FanetFrameRemoteConfig::geofenceFeature(uint16_t num, uint8_t *payload, uint16_t len)
{
	debug_printf("gf %d\n", num);
	Coordinate2D pos;
	if(len < 2+6+4+4)											//min 2 altitudes, 3 positions
		return false;
	uint16_t idx = 0;
	int16_t btm = (((int8_t)payload[idx++]) + 109) * 25;
	int16_t top = (((int8_t)payload[idx++]) + 109) * 25;
	FanetFrame::payload2coord_absolute(&payload[idx], pos);
	idx += 6;
	debug_printf("%d-%dm %.5f,%.5f\n", btm, top, rad2deg(pos.latitude), rad2deg(pos.longitude));

	for(uint16_t i=idx; i<len-3; i+=4)
	{
		float lat = FanetFrame::payload2coord_compressed((uint16_t *)&payload[i], pos.latitude);
		float lon = FanetFrame::payload2coord_compressed((uint16_t *)&payload[i+2], pos.longitude);
		debug_printf("%.5f,%.5f\n", rad2deg(lat), rad2deg(lon));
	}
	return false;
}

void FanetFrameRemoteConfig::request(uint8_t subtype, FanetMacAddr &addr)
{
	/* generate frame */
	FanetFrame *rfrm = new FanetFrame(fmac.addr);
	if(rfrm == nullptr)
		return;
	rfrm->setType(FanetFrame::TYPE_REMOTECONFIG);
	rfrm->dest = addr;
	rfrm->forward = false;
	rfrm->ackRequested = false;

	if(subtype == FANET_RC_POSITION)
	{
		if(fanet.position != Coordinate3D())
		{
			/* tell position */
			rfrm->payloadLength = 1+6+2;
			rfrm->payload = new uint8_t[rfrm->payloadLength];
			rfrm->payload[0] = FANET_RC_POSITION;
			FanetFrame::coord2payload_absolut(fanet.position, &rfrm->payload[1]);
			rfrm->payload[7] = static_cast<uint8_t>(static_cast<int16_t>(std::round(fanet.position.altitude/25.0f)) - 109);
			rfrm->payload[8] = static_cast<uint8_t>(std::round(fanet.heading/360.0f*256.0f));
		}
		else
		{
			/* no position present */
			rfrm->payloadLength = 1;
			rfrm->payload = new uint8_t[rfrm->payloadLength];
			rfrm->payload[0] = FANET_RC_POSITION;
		}
	}
	else if(subtype >= FANET_RC_REPLAY_LOWER && subtype <= FANET_RC_REPLAY_UPPER)
	{
		const uint16_t idx = subtype-FANET_RC_REPLAY_LOWER;
		if(fanet.replayFeature[idx].payloadLength > 0)
		{
			rfrm->payloadLength = 1+2+fanet.replayFeature[idx].payloadLength;
			rfrm->payload = new uint8_t[rfrm->payloadLength];
			rfrm->payload[0] = subtype;
			rfrm->payload[1] = fanet.replayFeature[idx].windSector;
			rfrm->payload[2] = fanet.replayFeature[idx].type | (fanet.replayFeature[idx].forward<<6);
			memcpy(&rfrm->payload[3], fanet.replayFeature[idx].payload, fanet.replayFeature[idx].payloadLength);
		}
		else
		{
			/* no position present */
			rfrm->payloadLength = 1;
			rfrm->payload = new uint8_t[rfrm->payloadLength];
			rfrm->payload[0] = subtype;
		}
	}
	//todo geofaence

	/* send if filed */
	if(rfrm->payloadLength == 0 || fmac.transmit(rfrm) != 0)
		delete rfrm;
}

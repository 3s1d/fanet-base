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
	if(num >= fanet.numReplayFeatures())
		return false;

	Replay& rf = fanet.getReplayFeature_locked(num);
	if(len < 2)
		rf.init(0, static_cast<FanetFrame::FrameType_t>(0), false, nullptr, 0);	//remove feature
	else
		rf.init(payload[0], static_cast<FanetFrame::FrameType_t>(payload[1]&0x3F), !!(payload[1]&0x40),	&payload[2], len-2); // copy feature
	return fanet.releaseReplayFeature(num);
}

bool FanetFrameRemoteConfig::geofenceFeature(uint16_t num, uint8_t *payload, uint16_t len)
{
	/* out of bounce */
	if(num >= fanet.numGeoFences())
		return false;

	/* remove */
	if(len == 0)
	{
		GeoFence& gf = fanet.getGeoFence_locked(num);
		gf.remove();
		fanet.releaseGeoFence();
		return true;
	}

	/* min 2 altitudes, 3 positions */
	if(len < 2+6+4+4)
		return false;

	/* altitudes */
	uint16_t idx = 0;
	int16_t btm = (((int8_t)payload[idx++]) + 109) * 25;
	int16_t top = (((int8_t)payload[idx++]) + 109) * 25;

	/* prepare fence */
	GeoFence& gf = fanet.getGeoFence_locked(num);
	gf.init((len-idx-2) / 4, top, btm);

	/* reference */
	Coordinate2D pos;
	FanetFrame::payload2coord_absolute(&payload[idx], pos);
	idx += 6;
	uint8_t numVert = 0;
	gf.add(numVert++, pos);

	/* other vertices */
	for(uint16_t i=idx; i<len-3; i+=4)
	{
		pos = Coordinate2D(FanetFrame::payload2coord_compressed((uint16_t *)&payload[i], pos.latitude),
				FanetFrame::payload2coord_compressed((uint16_t *)&payload[i+2], pos.longitude));
		gf.add(numVert++, pos);
	}
	fanet.releaseGeoFence();

	return fanet.writeGeoFences();
}

void FanetFrameRemoteConfig::request(uint8_t subtype, FanetMacAddr &addr)
{
	/* generate frame */
	FanetFrame *rfrm = new FanetFrame(addr);
	if(rfrm == nullptr)
		return;
	rfrm->setType(FanetFrame::TYPE_REMOTECONFIG);
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
	}
	else if(subtype >= FANET_RC_REPLAY_LOWER && subtype <= FANET_RC_REPLAY_UPPER)
	{
		Replay& rf = fanet.getReplayFeature_locked(subtype - FANET_RC_REPLAY_LOWER);
		if(rf.payloadLength > 0)
		{
			rfrm->payloadLength = 1 + 2 + rf.payloadLength;
			rfrm->payload = new uint8_t[rfrm->payloadLength];
			rfrm->payload[0] = subtype;
			rfrm->payload[1] = rf.windSector;
			rfrm->payload[2] = rf.type | (rf.forward<<6);
			memcpy(&rfrm->payload[3], rf.payload, rf.payloadLength);
		}
		fanet.releaseReplayFeature();
	}
	else if(subtype >= FANET_RC_GEOFENCE_LOWER && subtype <= FANET_RC_GEOFENCE_UPPER)
	{
		GeoFence& gf = fanet.getGeoFence_locked(subtype - FANET_RC_GEOFENCE_LOWER);
		if(gf.num > 0 && gf.vertex != nullptr)
		{
			rfrm->payloadLength = 1 + 2 + gf.num*4+2;
			rfrm->payload = new uint8_t[rfrm->payloadLength];
			uint16_t idx = 0;
			rfrm->payload[idx++] = subtype;
			rfrm->payload[idx++] = static_cast<uint8_t>(static_cast<int16_t>(std::round(gf.floor/25.0f)) - 109);
			rfrm->payload[idx++] = static_cast<uint8_t>(static_cast<int16_t>(std::round(gf.ceiling/25.0f)) - 109);

			/* vertices */
			coord2payload_absolut(gf.vertex[0], &rfrm->payload[idx]);
			idx += 6;
			for(uint16_t i=1; i<gf.num && idx<rfrm->payloadLength; i++)
			{
				*((uint16_t *)&rfrm->payload[idx]) = coord2payload_compressed(gf.vertex[i].latitude);
				*((uint16_t *)&rfrm->payload[idx+2]) = coord2payload_compressed(gf.vertex[i].longitude);
				idx += 4;
			}
		}
		fanet.releaseGeoFence();
	}

	/* not present / unknown */
	if(rfrm->payloadLength == 0 || rfrm->payload == nullptr)
	{
		rfrm->payloadLength = 1;
		if(rfrm->payload != nullptr)
			delete[] rfrm->payload;
		rfrm->payload = new uint8_t[rfrm->payloadLength];
		rfrm->payload[0] = subtype;
	}

	/* send if filled */
	if(rfrm->payloadLength == 0 || fmac.transmit(rfrm) != 0)
		delete rfrm;
}

/*
 * fremotecfg.cpp
 *
 *  Created on: 13 Jul 2019
 *      Author: sid
 */

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
		success = geofenceFeature(frm->payload[0] - FANET_RC_REPLAY_LOWER, &frm->payload[1], frm->payloadLength - 1);

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
	debug_printf("TODO\n");
	return false;
}

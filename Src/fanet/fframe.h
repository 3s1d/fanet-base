/*
 * fframe.h
 *
 *  Created on: Jul 4, 2018
 *      Author: sid
 */

#ifndef VARIO_HAL_FANET_FFRAME_H_
#define VARIO_HAL_FANET_FFRAME_H_

/*
 * Frame
 */
#define MAC_FRM_MIN_HEADER_LENGTH		4
#define MAC_FRM_ADDR_LENGTH			3
#define MAC_FRM_SIGNATURE_LENGTH		4

/* Header Byte */
#define MAC_FRM_HEADER_EXTHEADER_BIT		7
#define MAC_FRM_HEADER_FORWARD_BIT		6
#define MAC_FRM_HEADER_TYPE_MASK		0x3F

/* Extended Header Byte */
#define MAC_FRM_EXTHEADER_ACK_BIT1		7
#define MAC_FRM_EXTHEADER_ACK_BIT0		6
#define MAC_FRM_EXTHEADER_UNICAST_BIT		5
#define MAC_FRM_EXTHEADER_SIGNATURE_BIT		4
//bits 3-0 reserved

#define FRM_NOACK				0
#define FRM_ACK_SINGLEHOP			1
#define FRM_ACK_TWOHOP				2

#include <stdio.h>
#include <stdint.h>

#include "../phy/coordinate.h"
#include "fmacaddr.h"
#include "fneighbor.h"

//todo hide stuff?
class FanetFrame
{
public:
	enum FrameType_t : uint8_t
	{
		TYPE_ACK = 0,
		TYPE_TRACKING = 1,
		TYPE_NAME = 2,
		TYPE_MESSAGE = 3,
		TYPE_SERVICE = 4,
		TYPE_LANDMARK = 5,
		TYPE_REMOTECONFIG = 6,
		TYPE_GROUNDTRACKING = 7,
	};

protected:
	FrameType_t _type = TYPE_ACK;

public:
	/* general stuff */
	static uint16_t coord2payload_compressed(float ref_deg);
	static void coord2payload_absolut(const Coordinate3D &coord, uint8_t *buf);
	static float payload2coord_compressed(const uint16_t *buf, float ref_deg);
	static void payload2coord_absolute(const uint8_t *buf, Coordinate2D &pos);
	static float payload2ufloat(uint8_t buf, float scale);
	static float payload2sfloat(uint8_t buf, float scale);

	/* addresses */
	FanetMacAddr src;
	FanetMacAddr dest = FanetMacAddr();	//broadcast by default

	//ack and forwards (also geo based) will be handled by mac...
	int16_t ackRequested = FRM_NOACK;
	bool forward = false;

	uint32_t signature = 0;

	/* payload */
	const FrameType_t &type;		//used for identification
	int16_t payloadLength = 0;
	uint8_t *payload = nullptr;

	/* Transmit stuff */
	int16_t numTx = 0;
	uint32_t nextTx = 0;			//used for backoff

	/* Received stuff */
	int16_t rssi = 0;

	virtual int16_t serialize(uint8_t*& buffer);		//virtual is used to be overloaded by inherited class and still be castable
	void decodePayload(FanetNeighbor *neighbor);
	void requestAck(bool en);


	FanetFrame(const FanetMacAddr &dest);
	FanetFrame(FanetFrame *frm);
	FanetFrame();
	virtual ~FanetFrame() { if(payload != nullptr) delete [] payload; }		//virtual: calls inherited class and than this here

	/* deserialize packet */
	FanetFrame(int16_t length, uint8_t *data);

	void setType(FrameType_t newType) { _type = newType; }

	inline bool operator == (const FanetFrame& frm) const
	{
		if(src != frm.src)
			return false;

		if(dest != frm.dest)
			return false;

		if(type != frm.type)
			return false;

		if(payloadLength != frm.payloadLength)
			return false;

		for(int i=0; i<payloadLength; i++)
			if(payload[i] != frm.payload[i])
				return false;

		return true;
	}

	inline bool operator != (const FanetFrame& frm) const {	return !(*this == frm);	}
};





#endif /* VARIO_HAL_FANET_FFRAME_H_ */

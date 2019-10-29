/*
 * fhwinfo.h
 *
 *  Created on: 19 Oct 2019
 *      Author: sid
 */

#ifndef FANET_FRAME_FHWINFO_H_
#define FANET_FRAME_FHWINFO_H_

#include "../fframe.h"

class FanetFrameHwInfo : public FanetFrame
{
private:
	static const uint8_t type = 0x01;		//Windstation
	const uint8_t numVolatileRPF;
public:
	FanetFrameHwInfo(uint8_t numVolRPF = 0) : FanetFrame(), numVolatileRPF(numVolRPF) { _type = FanetFrame::TYPE_HWINFO; }

	/* handle payload */
	int16_t serialize(uint8_t*& buffer);
	static void decode(const uint8_t *payload, const uint16_t len, bool isBroadcast);
};



#endif /* FANET_FRAME_FHWINFO_H_ */

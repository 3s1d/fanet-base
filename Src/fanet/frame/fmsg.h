/*
 * fmsg.h
 *
 *  Created on: Sep 25, 2018
 *      Author: sid
 */

#ifndef VARIO_HAL_FANET_FRAME_FMSG_H_
#define VARIO_HAL_FANET_FRAME_FMSG_H_

#include "../fframe.h"

class FanetFrameMessage : public FanetFrame
{
private:
	enum subtype : uint8_t
	{
		normal = 0
	};

public:
	FanetFrameMessage(const FanetMacAddr &dest, char *text);

	/* handle payload */
	int16_t serialize(uint8_t*& buffer);
	static void decode(const uint8_t *payload, const uint16_t len, FanetNeighbor *neighbor);
};



#endif /* VARIO_HAL_FANET_FRAME_FMSG_H_ */

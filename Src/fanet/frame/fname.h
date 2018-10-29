/*
 * fname.h
 *
 *  Created on: Sep 21, 2018
 *      Author: sid
 */

#ifndef VARIO_HAL_FANET_FRAME_FNAME_H_
#define VARIO_HAL_FANET_FRAME_FNAME_H_

#include "../fframe.h"

class FanetFrameName : public FanetFrame
{
public:
	FanetFrameName() : FanetFrame() { _type = FanetFrame::TYPE_NAME; }
	FanetFrameName(const FanetMacAddr &dest) : FanetFrame(dest) { _type = FanetFrame::TYPE_NAME; }

	/* handle payload */
	int16_t serialize(uint8_t*& buffer);
	static void decode(const uint8_t *payload, const uint16_t len, FanetNeighbor *neighbor);
};



#endif /* VARIO_HAL_FANET_FRAME_FNAME_H_ */

/*
 * f2dtracking.h
 *
 *  Created on: Sep 27, 2018
 *      Author: sid
 */

#ifndef VARIO_HAL_FANET_FRAME_FGNDTRACKING_H_
#define VARIO_HAL_FANET_FRAME_FGNDTRACKING_H_

#include "../fframe.h"

#define FANET_GROUNDTRACKING_SIZE			7

class FanetFrameGndTracking : public FanetFrame
{
public:
	FanetFrameGndTracking() : FanetFrame() { _type = FanetFrame::TYPE_GROUNDTRACKING; }

	/* handle payload */
	int16_t serialize(uint8_t*& buffer);
	static void decode(const uint8_t *payload, const uint16_t len, FanetNeighbor *neighbor);
};


#endif /* VARIO_HAL_FANET_FRAME_FGNDTRACKING_H_ */

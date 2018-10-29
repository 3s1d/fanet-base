/*
 * tracking.h
 *
 *  Created on: Jul 5, 2018
 *      Author: sid
 */

#ifndef VARIO_HAL_FANET_FRAME_FTRACKING_H_
#define VARIO_HAL_FANET_FRAME_FTRACKING_H_

#define FANET_TRACKING_SIZE			12

#include <stdint.h>
#include <math.h>


#include "../fframe.h"

class FanetFrameTracking : public FanetFrame
{
public:
	FanetFrameTracking() : FanetFrame() { _type = FanetFrame::TYPE_TRACKING; }
	FanetFrameTracking(FanetFrame *frm) : FanetFrame(frm) { }
	~FanetFrameTracking() { }

	static int payload2altitude(uint16_t *buf);
	static void altitude2payload(int altitude, uint16_t *buf);

	/* handle payload */
	int16_t serialize(uint8_t*& buffer);
	static void decode(const uint8_t *payload, const uint16_t len, FanetNeighbor *neighbor);
};



#endif /* VARIO_HAL_FANET_FRAME_FTRACKING_H_ */

/*
 * fack.h
 *
 *  Created on: Jul 5, 2018
 *      Author: sid
 */

#ifndef VARIO_HAL_FANET_FRAME_FACK_H_
#define VARIO_HAL_FANET_FRAME_FACK_H_

#include "../fframe.h"

class FanetFrameAck : public FanetFrame
{
public:
	FanetFrameAck(FanetMacAddr &dest) : FanetFrame(dest) { _type = FanetFrame::TYPE_ACK; }
	~FanetFrameAck() { }
};




#endif /* VARIO_HAL_FANET_FRAME_FACK_H_ */

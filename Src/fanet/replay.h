/*
 * replay.h
 *
 *  Created on: 13 Jul 2019
 *      Author: sid
 */

#ifndef FANET_REPLAY_H_
#define FANET_REPLAY_H_

#include "../serial/serial_interface.h"
#include "fframe.h"

class Replay
{
	friend class Serial_Interface;
	friend class FanetFrameRemoteConfig;

private:
	uint8_t *payload = nullptr;
	uint8_t payloadLength = 0;

	uint8_t windSector = 0;
	FanetFrame::FrameType_t type = FanetFrame::TYPE_NONE;
	bool forward = false;

public:
	Replay() { }
	~Replay() {if(payload != nullptr) delete payload; }

	/* output */
	bool isVaild(void);
	FanetFrame *toFrame(void);

	/* management */
	void init(uint8_t windSector, FanetFrame::FrameType_t type, bool forward, uint8_t *payload, uint8_t len);
	bool write(uint32_t addr);
	void load(uint32_t addr);

	FanetFrame::FrameType_t getType(void) { return type; }
};


#endif /* FANET_REPLAY_H_ */

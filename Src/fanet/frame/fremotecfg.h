/*
 * fremotecfg.h
 *
 *  Created on: 1 Jul 2019
 *      Author: sid
 */

#ifndef FANET_FRAME_FREMOTECFG_H_
#define FANET_FRAME_FREMOTECFG_H_

#define FANET_RC_ACK				0
#define FANET_RC_REQUST				1
#define FANET_RC_POSITION			2
#define FANET_RC_GEOFENCE_LOWER			4
#define FANET_RC_GEOFENCE_UPPER			8
#define FANET_RC_REPLAY_LOWER			9
#define FANET_RC_REPLAY_UPPER			33

#include "../fframe.h"

class FanetFrameRemoteConfig : public FanetFrame
{
private:
	static bool position(uint8_t *payload, uint16_t payload_length);
	static bool replayFeature(uint16_t num, uint8_t *payload, uint16_t len);
	static bool geofenceFeature(uint16_t num, uint8_t *payload, uint16_t len);
	static void request(uint8_t suytype, FanetMacAddr &addr);
public:
	FanetFrameRemoteConfig() : FanetFrame() { _type = FanetFrame::TYPE_REMOTECONFIG; }

	/* handle payload */
	int16_t serialize(uint8_t*& buffer);
	static void decode(FanetFrame *frm);
};


#endif /* FANET_FRAME_FREMOTECFG_H_ */

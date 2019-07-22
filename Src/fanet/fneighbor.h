/*
 * fneighbor.h
 *
 *  Created on: Jul 4, 2018
 *      Author: sid
 */

#ifndef VARIO_HAL_FANET_FNEIGHBOR_H_
#define VARIO_HAL_FANET_FNEIGHBOR_H_

#define FNEIGHBOR_MAX_TIMEOUT_MS			250000		//4min 10sek

#include <string.h>

#include "cmsis_os.h"

#include "../phy/coordinate.h"
#include "fanet_defs.h"
#include "fmacaddr.h"

//#define FNEIGHBOR_DEBUG

class FanetNeighbor
{
private:
	uint32_t lastSeen;

	/* 2D/3D tracking */
	Coordinate3D _pos = Coordinate3D();
	FanetDef::aircraft_t _aircraft = FanetDef::NONE;
	FanetDef::status_t _status = FanetDef::AIRBORNE;

public:
	const FanetMacAddr addr;
	char name[32] = {'\0'};

	/* 2D/3D position/tracking */
	const Coordinate3D &pos;
	const FanetDef::aircraft_t &aircraft;
	const FanetDef::status_t &status;

	/* additional 3D tracking here */
	//note: could be extrapolated from 2d tracking as well
	uint32_t lastTrackUpdate = 0;
	float climb_mps = 0.0f;					//wind gust / 10 [km/h] 4 service
	float speed_kmh = 0.0f;					//wind speed 4 service
	float heading_rad = 0.0f;				//wind direction 4 service

	/* message */
	char *msg = nullptr;
	uint32_t msgReceived = 0;
	bool msgRead = false;

	FanetNeighbor(const FanetMacAddr &addr);

	~FanetNeighbor()
	{
#if defined(DEBUG) && defined(FNEIGHBOR_DEBUG)
		printf("%lu ~fn%02x:%04x\n", osKernelSysTick(), addr.manufacturer, addr.id);
#endif
		if(msg != nullptr)
			delete [] msg;
	}

	void seen(void) { lastSeen = osKernelSysTick(); }
	bool isAround(void) { return lastSeen + FNEIGHBOR_MAX_TIMEOUT_MS > osKernelSysTick(); }
	uint32_t wasSeen(void) { return lastSeen; }

	bool isAirborne(void) { return status == FanetDef::AIRBORNE && aircraft != FanetDef::NONE; }

	void setTrackingType(FanetDef::aircraft_t ac) {_aircraft = ac; _status = FanetDef::AIRBORNE; }
	void setTrackingType(FanetDef::status_t stat);
	void setPosition(Coordinate3D &newPos) {_pos = newPos; }
	void setPosition(Coordinate2D &newPos) {_pos = newPos; _pos.altitude = 0.0f; }
	void setPosition(Coordinate2D &newPos, float altitude) {_pos = newPos; _pos.altitude = altitude; }
	bool hasPosition(void) { return pos != Coordinate3D(); }

	void setName(const char *newName) { setName(newName, strlen(newName)); };
	void setName(const char *newName, uint16_t len);

	void setMessage(const char *text, uint16_t len);
};



#endif /* VARIO_HAL_FANET_FNEIGHBOR_H_ */

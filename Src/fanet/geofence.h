/*
 * geofence.h
 *
 *  Created on: 22 Jul 2019
 *      Author: sid
 */

#ifndef FANET_GEOFENCE_H_
#define FANET_GEOFENCE_H_

#include "../phy/coordinate.h"

class GeoFence
{
private:
	/* area */
	Coordinate2D *vertex = nullptr;
	uint8_t num = 0;
	float ceiling = 0.0f;
	float floor = 0.0f;

	float horizontalDistance(const Coordinate3D &poi);
	float verticalDistance(const Coordinate3D &poi);
public:
	GeoFence();
	~GeoFence() { if(vertex != nullptr) delete vertex; }

	void init(uint8_t num, float ceiling, float floor);
	bool add(uint8_t idx, Coordinate2D &pos);
	bool inside(Coordinate3D &poi);
};



#endif /* FANET_GEOFENCE_H_ */

/*
 * geofence.cpp
 *
 *  Created on: 22 Jul 2019
 *      Author: sid
 */

#include <math.h>
#include <algorithm>

#include "common.h"
#include "../phy/angle.h"
#include "../phy/triangle.h"
#include "geofence.h"

void GeoFence::init(uint8_t num, float ceiling, float floor)
{
	/* free old vertices */
	if(vertex != nullptr)
		delete vertex;

	this->num = num;
	this->ceiling = ceiling;
	this->floor = floor;
	vertex = new Coordinate2D[num];
}

bool GeoFence::add(uint8_t idx, Coordinate2D &pos)
{
	/* out of index */
	if(idx >= num)
		return false;

	vertex[idx] = pos;
	return true;
}

bool GeoFence::inside(Coordinate3D &poi)
{
	return verticalDistance(poi) < 0.0f && horizontalDistance(poi) < 0.0f;
}

float GeoFence::horizontalDistance(const Coordinate3D &poi)
{
	if(vertex == nullptr || num == 0)
		return NAN;

	float windingCount = 0.0f;
	float sinPoi_lat = std::sin(poi.latitude);
	float cosPoi_lat = std::cos(poi.latitude);
	float lastDst = 0.0f, lastAng = 0.0f;
	float firstDst = 0.0f, firstAng = 0.0f;
	float minDst = 0.0f;

	for(int i=0; i<num; i++)
	{
		float dx = cosPoi_lat * (poi.longitude - vertex[i].longitude);
		float dy = poi.latitude - vertex[i].latitude;
		float dst = std::sqrt(dx * dx + dy * dy);
		float ang = std::atan2(std::sin(poi.longitude-vertex[i].longitude) * std::cos(vertex[i].latitude),
			cosPoi_lat*std::sin(vertex[i].latitude) - sinPoi_lat*std::cos(vertex[i].latitude)*std::cos(poi.longitude-vertex[i].longitude));

		if(i==0)
		{
			/* only one element in list */
			if(num == 1)
				return dst * WGS84_A_RADIUS;

			/* first element */
			minDst = dst;
			firstDst = dst;
			firstAng = ang;
		}
		else
		{
			/* any other element */
			const float deltaAng = angle::toMinusPlusPi(ang - lastAng);
			windingCount += deltaAng;
			float dstfromline, relAng;
			triangle::getAltitudeAttachedAngle(dst, lastDst, deltaAng, dstfromline, relAng);
			if (dstfromline < minDst)
				minDst = dstfromline;
		}

		/* store state */
		lastDst = dst;
		lastAng = ang;
	}

	/* closing polygon */
	const float deltaAng = angle::toMinusPlusPi(firstAng - lastAng);
	windingCount += deltaAng;
	float dstfromline, relAng;
	triangle::getAltitudeAttachedAngle(firstDst, lastDst, deltaAng, dstfromline, relAng);
	if (dstfromline < minDst)
		minDst = dstfromline;

	//note: (minDst < -1.0) -> NZD021 AUCKLAND OCEANIC FIR Bugfix
	if (fabsf(windingCount) > M_PI_f && minDst < 1.0f)
		minDst = -minDst;

	return minDst * WGS84_A_RADIUS;
}

float GeoFence::verticalDistance(const Coordinate3D &poi)
{
	/* floor */
	float belowFloor = floor - poi.altitude;

	/* ceiling */
	float aboveCeiling = poi.altitude - ceiling;

	return std::max(belowFloor, aboveCeiling);
}

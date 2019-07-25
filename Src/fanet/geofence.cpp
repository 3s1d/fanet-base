/*
 * geofence.cpp
 *
 *  Created on: 22 Jul 2019
 *      Author: sid
 */

#include <stdio.h>
#include <math.h>
#include <algorithm>

#include "stm32l4xx.h"

#include "common.h"
#include "print.h"
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

void GeoFence::remove(void)
{
	num = 0;
	delete[] vertex;
	vertex = nullptr;
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
	if(poi == Coordinate3D())
		return false;

	return verticalDistance(poi) <= 0.0f && horizontalDistance(poi) <= 0.0f;
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

	//debug_printf("hor %.f\n", minDst* WGS84_A_RADIUS);
	return minDst * WGS84_A_RADIUS;
}

float GeoFence::verticalDistance(const Coordinate3D &poi)
{
	/* floor */
	float belowFloor = floor - poi.altitude;

	/* ceiling */
	float aboveCeiling = poi.altitude - ceiling;

//	debug_printf("vert %.f\n", std::max(belowFloor, aboveCeiling));
	return std::max(belowFloor, aboveCeiling);
}

bool GeoFence::isActive(void)
{
	return vertex != nullptr && num > 0;
}

bool GeoFence::write(uint32_t addr)
{
	/* header */
	uint64_t container = ((uint64_t) ((uint8_t)num)) << 0;
	if(num == 0 || vertex == nullptr)	// fence unused
		container = UINT64_MAX;
	if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, addr, container) != HAL_OK)
		return false;
	addr += 8;

	/* done writing */
	if(container == UINT64_MAX)
		return true;

	/* altitudes */
	container = 	((uint64_t) ((uint8_t *)&floor)[0]) << 0    | ((uint64_t) ((uint8_t *)&floor)[1]) << 8    |
			((uint64_t) ((uint8_t *)&floor)[2]) << 16   | ((uint64_t) ((uint8_t *)&floor)[3]) << 24   |
			((uint64_t) ((uint8_t *)&ceiling)[0]) << 32 | ((uint64_t) ((uint8_t *)&ceiling)[1]) << 40 |
			((uint64_t) ((uint8_t *)&ceiling)[2]) << 48 | ((uint64_t) ((uint8_t *)&ceiling)[3]) << 56;
	if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, addr, container) != HAL_OK)
		return false;
	addr += 8;

	/* vertices */
	for(uint16_t i=0; i<num && i<60; i++)
	{
		container = 	((uint64_t) ((uint8_t *)&vertex[i].latitude)[0]) << 0   | ((uint64_t) ((uint8_t *)&vertex[i].latitude)[1]) << 8   |
				((uint64_t) ((uint8_t *)&vertex[i].latitude)[2]) << 16  | ((uint64_t) ((uint8_t *)&vertex[i].latitude)[3]) << 24  |
				((uint64_t) ((uint8_t *)&vertex[i].longitude)[0]) << 32 | ((uint64_t) ((uint8_t *)&vertex[i].longitude)[1]) << 40 |
				((uint64_t) ((uint8_t *)&vertex[i].longitude)[2]) << 48 | ((uint64_t) ((uint8_t *)&vertex[i].longitude)[3]) << 56;
		if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, addr, container) != HAL_OK)
			return false;
		addr += 8;
	}

	return true;
}

void GeoFence::load(uint32_t addr)
{
	__IO uint64_t *ptr = (__IO uint64_t*)addr;

	/* num vertices */
	if(*ptr > 60)
	{
		/* empty */
		remove();
		return;
	}
	uint8_t num = *ptr++;

	/* altitudes */
	float *fptr = (float *) ptr;
	float floor = *fptr++;
	float ceiling = *fptr++;
	init(num, ceiling, floor);
	//debug_printf("Geo Fence %.f-%.fm (%d)\n", floor, ceiling, num);

	/* vertices */
	for(uint16_t i=0; i<num; i++)
	{
		float lat = *fptr++;
		float lon = *fptr++;
		Coordinate2D v = Coordinate2D(lat, lon);
		add(i, v);
		//debug_printf("#%d %.4f,%4f\n", i, rad2deg(lat), rad2deg(lon));
	}


}

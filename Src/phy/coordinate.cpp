/*
 * coordinate.cpp
 *
 *  Created on: Jun 13, 2018
 *      Author: sid
 */

#include <cmath>

#include "common.h"
#include "coordinate.h"

Coordinate3D& Coordinate3D::operator=(const Coordinate2D& other)
{
	latitude = other.latitude;
	longitude = other.longitude;
	//altitude stays untouched
	return *this;
}

Coordinate3D& Coordinate3D::operator=(const Coordinate3D& other)
{
	/* self-assignment check expected */
	if (this != &other)
	{
		latitude = other.latitude;
		longitude = other.longitude;
		altitude = other.altitude;
	}

	return *this;
}

float Coordinate3D::distanceTo(const Coordinate2D &target) const
{
	return (2.0f*std::asin(std::sqrt( std::pow((std::sin((latitude-target.latitude)/2.0f)), 2.0) +
			std::cos(latitude)*std::cos(target.latitude)*std::pow((std::sin((longitude-target.longitude)/2.0f)), 2.0) ))
			* WGS84_A_RADIUS);
}

float Coordinate3D::angleTo(const Coordinate2D &target)	const	//north aligned, mathematical direction
{
	return atan2f(sinf(longitude-target.longitude)*cosf(target.latitude),
			cosf(latitude)*sinf(target.latitude)-sinf(latitude)*cosf(target.latitude)*cosf(longitude-target.longitude));
}

float Coordinate3D::courseTo(const Coordinate2D &target) const
{
	/* angle to course */
	float course = M_2PI_f - angleTo(target);
	if(course >= M_2PI_f)
		course -= M_2PI_f;

	return course;
}

Coordinate2D &Coordinate2D::translate(float heading_rad, float distance_m)
{
	distance_m /= WGS84_A_RADIUS;

	float lat_origin = latitude;
	float lon_origin = longitude;
	latitude = asinf( sinf(lat_origin) * cosf(distance_m) + cosf(lat_origin) * sinf(distance_m) * cosf(heading_rad) );
	longitude = lon_origin + atan2f( sinf(heading_rad) * sinf(distance_m) * cosf(lat_origin),
			cosf(distance_m) - sinf(lat_origin) * sinf(latitude) );
	return *this;
}

Coordinate2D &Coordinate2D::operator=(const Coordinate2D& other)
{
	/* self-assignment check expected */
	if (this != &other)
	{
		latitude = other.latitude;
		longitude = other.longitude;
	}

	return *this;
}

Coordinate2D &Coordinate2D::operator()(float lat, float lon)
{
	latitude = lat;
	longitude = lon;
	return *this;
}

Coordinate2D &Coordinate2D::operator=(const Coordinate3D& other)
{
	latitude = other.latitude;
	longitude = other.longitude;
	return *this;
}

float Coordinate2D::distanceTo(const Coordinate2D &target) const
{
	return (2.0f*asinf(sqrtf( pow((sinf((latitude-target.latitude)/2.0f)), 2.0f) +
			cosf(latitude)*cosf(target.latitude)*pow((sinf((longitude-target.longitude)/2.0f)), 2.0f) )) * WGS84_A_RADIUS);
}

bool operator==(const Coordinate3D& lhs, const Coordinate3D& rhs)
{
	return lhs.latitude == rhs.latitude && lhs.longitude == rhs.longitude && lhs.altitude == rhs.altitude;
}

Coordinate2D operator-(const Coordinate2D& lhs, const Coordinate2D& rhs)
{
	return Coordinate2D(lhs.latitude-rhs.latitude, lhs.longitude-rhs.longitude);
}

bool operator!=(const Coordinate2D& lhs, const Coordinate2D& rhs)
{
	return (lhs.latitude!=rhs.latitude) || (lhs.longitude!=rhs.longitude);
}


Coordinate2D Coord::translate(const Coordinate2D &base, float heading_rad, float distance_m)
{
	Coordinate2D coord = base;
	coord.translate(heading_rad, distance_m);

	return coord;
}

/*
 * coordinate.h
 *
 *  Created on: Jun 13, 2018
 *      Author: sid
 */

#ifndef VARIO_PHY_COORDINATE_H_
#define VARIO_PHY_COORDINATE_H_

class Coordinate2D;

class Coordinate3D
{
public:
	float latitude;
	float longitude;
	float altitude;

	Coordinate3D(void) : latitude(0.0f), longitude(0.0f), altitude(0.0f) { }
	Coordinate3D(float lat, float lon, float alt=0.0f) : latitude(lat), longitude(lon), altitude(alt) { }

	/* operators */
	Coordinate3D &operator=(const Coordinate2D& other);
	Coordinate3D &operator=(const Coordinate3D& other);

	float distanceTo(const Coordinate2D &target) const;
	float angleTo(const Coordinate2D &target) const;
	float courseTo(const Coordinate2D &target) const;
};

class Coordinate2D
{
public:
	float latitude;
	float longitude;

	Coordinate2D(void) : latitude(0.0f), longitude(0.0f) { }
	Coordinate2D(float lat, float lon) : latitude(lat), longitude(lon) { }
	Coordinate2D(const Coordinate3D &c3d) : latitude(c3d.latitude), longitude(c3d.longitude) { }


	/* operators */
	Coordinate2D &operator=(const Coordinate2D& other);
	Coordinate2D &operator=(const Coordinate3D& other);
	Coordinate2D &translate(float heading_rad, float distance_m);
	Coordinate2D &operator()(float lat, float lon);

	float distanceTo(const Coordinate2D &target) const;
};

Coordinate2D operator-(const Coordinate2D& lhs, const Coordinate2D& rhs);
bool operator!=(const Coordinate2D& lhs, const Coordinate2D& rhs);
bool operator==(const Coordinate3D& lhs, const Coordinate3D& rhs);
bool operator==(const Coordinate2D& lhs, const Coordinate2D& rhs);


namespace Coord
{
	Coordinate2D translate(const Coordinate2D &base, float heading_rad, float distance_m);
}

#endif /* VARIO_PHY_COORDINATE_H_ */

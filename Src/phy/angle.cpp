/*
 * angle.cpp
 *
 *  Created on: Aug 14, 2018
 *      Author: sid
 */

#include "common.h"

#include "angle.h"

namespace angle
{
float diff(float source_rad, float target_rad)
{
	float a = target_rad - source_rad;
	return toMinusPlusPi(a);
}

float toMinusPlusPi(float input_rad)
{
	while(input_rad > M_PI_f)
		input_rad -= M_2PI_f;
	while(input_rad < -M_PI_f)
		input_rad += M_2PI_f;

	return input_rad;
}

float toZeroTwoPi(float input_rad)
{
	while(input_rad > M_2PI_f)
		input_rad -= M_2PI_f;
	while (input_rad < 0.0f)
		input_rad += M_2PI_f;

	return input_rad;
}

float course2ang_nav2math_rad_ExNy(float ang)
{
	return -(ang-M_PI_2_f);
}

float ang2course_math2nav_rad_ExNy(float ang)
{
	return M_PI_2_f-ang;
}

}


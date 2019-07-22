/*
 * triangle.cpp
 *
 *  Created on: Aug 31, 2018
 *      Author: sid
 */

#include <cmath>

#include "common.h"
#include "triangle.h"

/*
 * same notation as in http://www.edwilliams.org/avform.htm -> Some general spherical triangle formulae.
 *              c
           A -------B
            \       |
             \      |
              \b    |a
               \    |
                \   |
                 \  |
                  \C|
                   \|

    (The angle at B is not necessarily a right angle)

      sin(a)  sin(b)   sin(c)
      ----- = ------ = ------
      sin(A)  sin(B)   sin(C)

Given {A,b,c}:  // Two sides, included angle
   a=acos(cos(b)*cos(c)+sin(b)*sin(c)*cos(A))
   B=acos((cos(b) - cos(c)*cos(a))/(sin(c)*sin(a)))
   C=acos((cos(c) - cos(a)*cos(b))/(sin(a)*sin(b)))
 *
 */

/* expects (0..pi) for edges (-pi..pi) for the angle */
//return: h = altitude, Abh = angle between h and b
void triangle::getAltitudeAttachedAngle(double b, double c, float Aprime, float &h, float &Abh)
{
	/* swap, ensure b is the smaller edge */
	bool swapped = false;
	if (c < b)
	{
		float tmp = b;
		b = c;
		c = tmp;
		swapped = true;
	}

	/* angle less than 8deg, accepting an distance error < 1% -> massive performance boost */
	const double A = std::abs(Aprime);				//fold to (0..pi)
	if (A < 0.1415 || A >= M_PI)
	{
		h = b;
		Abh = swapped ? Aprime : 0.0f;				//swapped b,c?
		return;
	}

	/* missing information */
	const double a = acos( cos(b)*cos(c) + sin(b)*sin(c)*cos(A) );
	double C = (cos(c) - cos(a)*cos(b)) / (sin(a) * sin(b));
	if (C <= 0.0)							// =>90 degree, h does not intersect a
	{
		h = b;
		Abh = swapped ? Aprime : 0.0f;				//swapped b,c?
		return;
	}
	C = acos(C);

	/* building sub-triangle using b,C,B'. Where B' is perpendicular -> sin(B') = 1 */
	h = asin(sin(C) * sin(b));					//h = altitude of A and a
	if(b == 0.0)
	{
		Abh = swapped ? Aprime : 0.0f;				//swapped b,c?
		return;
	}
	Abh = acosf(h / ((float)b)) * sgn(Aprime);

	if(swapped)
		Abh = Aprime - Abh;					//swapped edges -> translate to other edge
}



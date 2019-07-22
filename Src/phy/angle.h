/*
 * angle.h
 *
 *  Created on: Aug 14, 2018
 *      Author: sid
 */

#ifndef VARIO_MISC_ANGLE_H_
#define VARIO_MISC_ANGLE_H_

namespace angle
{
float diff(float source_rad, float target_rad);
float toMinusPlusPi(float input_rad);
float toZeroTwoPi(float input_rad);
float course2ang_nav2math_rad_ExNy(float ang);
float ang2course_math2nav_rad_ExNy(float ang);
}



#endif /* VARIO_MISC_ANGLE_H_ */

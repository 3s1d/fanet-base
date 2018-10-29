/*
 * clamp.h
 *
 *  Created on: 29.06.2018
 *      Author: skytraxx
 */

#ifndef CLAMP_H_
#define CLAMP_H_


#ifndef __cplusplus
	#define clampf(val, minv, maxv)		val = fmaxf((minv), fminf((maxv), (val)))
	#define clamp(val, minv, maxv)		val = max((minv), min((maxv), (val)))
#else

#include <algorithm>

template<class T>
void clamp(T& val, const T& lo, const T& hi)
{
	val = std::max((lo), std::min((hi), (val)));
}

#endif



#endif /* CLAMP_H_ */

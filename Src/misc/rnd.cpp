/*
 * random.c
 *
 *  Created on: May 21, 2017
 *      Author: sid
 */

#include <stdint.h>

#include "rnd.h"

uint32_t rnd::get(uint32_t howbig)
{
	if(howbig==0)
		return 0;
	return rand() % howbig;
}

uint32_t rnd::get(uint32_t howsmall, uint32_t howbig)
{
	if (howsmall >= howbig)
		return howsmall;
	return get(howbig - howsmall) + howsmall;
}

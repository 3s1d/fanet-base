/*
 * power.h
 *
 *  Created on: 11 Jul 2019
 *      Author: sid
 */

#ifndef HAL_POWER_H_
#define HAL_POWER_H_

#include "config.h"

#ifdef DIRECTBAT
#define POWER_SOCINTERVALL		30000
#endif

namespace power
{
	extern bool psu;
	bool isSufficiant(void);

#ifdef DIRECTBAT
	float getSoc(void);
#endif

}



#endif /* HAL_POWER_H_ */

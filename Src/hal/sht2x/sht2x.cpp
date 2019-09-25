/*
 * sht2x.cpp
 *
 *  Created on: 25 Sep 2019
 *      Author: sid
 */

#include <stdbool.h>

#include "cmsis_os.h"
#include "main.h"

#include "../power.h"
#include "sht2x.h"


void sht2x_task(void const * argument)
{
	sht2x.init();

	while(1)
	{

		osDelay(12000/*0*/);			//wait two minutes
		if(power::isSufficiant())
			sht2x.handle();
	}
}


void Sht2x::init(void)
{
	si2c = si2cInit(SHTSDA_GPIO_Port, SHTSDA_Pin, SHTSCL_GPIO_Port, SHTSCL_Pin);
}

void Sht2x::handle(void)
{

}

Sht2x sht2x = Sht2x();

/*
 * power.cpp
 *
 *  Created on: 11 Jul 2019
 *      Author: sid
 */

#include <stdio.h>

#include "cmsis_os.h"
#include "stm32l4xx.h"
#include "adc.h"
#include "comp.h"

#include "clamp.h"

#include "power.h"

bool power::psu = false;
float power_vdd = 0.0f;
float power_soc = 0.0f;
uint32_t power_nextconv = 0;

/* Cap > 2.225V */
/* Vbat > 3.33V */
bool power::isSufficiant(void)
{
#ifdef DEBUG
	return true;
#endif

	/* power always sufficiant on PSU  */
	if(psu)
		return true;

#ifdef DIRECTBAT
	return power::getSoc() > 0.80f;
#else
	static uint32_t nextPwr = 0;
	static bool pwrSuf = false;
	uint32_t current = osKernelSysTick();

	/* use cache */
	if(current < nextPwr)
		return pwrSuf;
	nextPwr = current + 5000;

	/* generate new value */
	HAL_COMP_Start(&hcomp2);
	pwrSuf = !!HAL_COMP_GetOutputLevel(&hcomp2);
	HAL_COMP_Stop(&hcomp2);

	return pwrSuf;
#endif
}


#ifdef DIRECTBAT
float power::getSoc(void)
{
	if(power_nextconv > osKernelSysTick())
		return power_soc;

	/* do conversion */
	osMutexWait(adcMutex, osWaitForever);
	ADC_Select_VREF();
	if(HAL_ADC_Start(&hadc1) != HAL_OK)
	{
		osMutexRelease(adcMutex);
		return power_soc;
	}
	if(HAL_ADC_PollForConversion(&hadc1, 10) != HAL_OK)
	{
		HAL_ADC_Stop(&hadc1);
		osMutexRelease(adcMutex);
		return power_soc;
	}

	/* Get value */
	uint32_t adc = HAL_ADC_GetValue(&hadc1);			//vref

	/* adc shut down */
	HAL_ADC_Stop(&hadc1);
	osMutexRelease(adcMutex);

	/* filter voltages */
	float v = (3.0f * ((*VREFINT_CAL_ADDR)>>2) / adc);
	if(power_vdd == 0.0f)
		power_vdd = v;
	else
		power_vdd = 0.75f*power_vdd + 0.25f*v;

	/* compute SOC */
	power_soc = (power_vdd - 3.0f) / (3.4-3.0f);
	clamp(power_soc, 0.0f, 1.0f);

	/* prevent numerous conversions */
	power_nextconv = osKernelSysTick() + POWER_SOCINTERVALL;

	return power_soc;
}

bool power::critical(void)
{
	getSoc();
	return power_vdd > 0.0f && power_vdd < 2.9f;
}
#endif

/*
 * Windsensor.cpp
 *
 *  Created on: Sep 19, 2017
 *      Author: sid
 */

#include "../hal/wind.h"

#include <stdio.h>
#include <float.h>
#include <math.h>

#include "cmsis_os.h"
#include "adc.h"

#include "common.h"

uint16_t windTicker = 0;		//16bit is for sure atomic
Wind wind = Wind();

void wind_rtos(void)
{

}

void wind_irq(void)
{
	/* de-bounce sensor */
	//note: 10ms == 142 mph (misol)
	static uint32_t debounce = 0;
	uint32_t current = osKernelSysTick();
	if(current-debounce < 10 && current >= debounce)
		return;
	debounce = current;

	/* advance ticker */
	windTicker++;
}


void wind_task(void const * argument)
{
	/* let things become stable */
	osDelay(2000);

  	/* Calibrate ADC */
	HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);

	TickType_t xLastWakeTime = xTaskGetTickCount();
	while(1)
	{
		/* Wait for the next cycle */
		vTaskDelayUntil(&xLastWakeTime, portTICK_PERIOD_MS * 1000);

		/* handle wind stuff */
		wind.handle();
	}
}


float Wind::getDir_misol_deg(void)
{
	/* enable external pull-up */
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.Pin = WIND_DIR_PULL_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(WIND_DIR_PULL_GPIO_Port, &GPIO_InitStruct);
	HAL_GPIO_WritePin(WIND_DIR_PULL_GPIO_Port, WIND_DIR_PULL_Pin, GPIO_PIN_SET);

	/* ensure low power pin state */
	HAL_GPIO_WritePin(WIND_PWR_GPIO_Port, WIND_PWR_Pin, GPIO_PIN_RESET);

	//note: delays for about 50uS in order to let the signal rise. todo: not quite enough...
	//for(int i=0 ; i<16; i++)
	//	asm("nop");
	osDelay(1);

	if(HAL_ADC_Start_IT(&hadc1) != HAL_OK)
		return -1.0f;

	if(HAL_ADC_PollForConversion(&hadc1, 10) != HAL_OK)
		return -1.0f;

	/* Get value */
	uint32_t adc = HAL_ADC_GetValue(&hadc1);

	/* adc shut down */
	//note: EN pin set to input -> disable
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	HAL_GPIO_Init(WIND_DIR_PULL_GPIO_Port, &GPIO_InitStruct);
	HAL_ADC_Stop_IT(&hadc1);

	/* determine error in respect to all directions */
	float error[NELEM(wsd)];
	for(unsigned int i=0; i<NELEM(wsd); i++)
		error[i] = std::abs((float)adc - wsd[i].ratio);

	/* minimize error */
	float minError = FLT_MAX;
	int idx = -1;
	for(unsigned int i=0; i<NELEM(wsd); i++)
	{
		if(error[i] < minError)
		{
			minError = error[i];
			idx = i;
		}
	}

	/* error */
	if(idx < 0)
		return 0.0f;
	return wsd[idx].dir;
}

float Wind::getDir_davis_deg(void)
{
	/* ensure high power pin state */
	HAL_GPIO_WritePin(WIND_PWR_GPIO_Port, WIND_PWR_Pin, GPIO_PIN_SET);
	osDelay(1);

	if(HAL_ADC_Start_IT(&hadc1) != HAL_OK)
		return -1.0f;

	if(HAL_ADC_PollForConversion(&hadc1, 10) != HAL_OK)
		return -1.0f;

	/* Get value */
	uint32_t adc = HAL_ADC_GetValue(&hadc1);

	/* adc shut down */
	HAL_GPIO_WritePin(WIND_PWR_GPIO_Port, WIND_PWR_Pin, GPIO_PIN_RESET);
	HAL_ADC_Stop_IT(&hadc1);

	/* convert adc to angle */
	return (adc*360.0f)/1024.0f;
}

float Wind::getSpeed_kmh(uint16_t ticker, bool isMisol)
{
	/* wind speed */
	uint16_t dticks = ticker - ticker_last;			//note: type has to be the same as ticker and ticker_last for a correct overflow
	ticker_last = ticker;

	/* delta t */
	uint32_t current = osKernelSysTick();
	uint32_t dt = current - speed_last;
	speed_last = current;
	if(dt == 0)
		dt = 1;

	return ((isMisol?2400.0f:3621.015f) /dt) * dticks;
}

void Wind::handle(void)
{
	uint32_t current = osKernelSysTick();

	/* test sensor present */
	if(current >= sensorCheck_last+60000 || sensorCheck_last == 0)
	{
		/* sensor available check */
		//note: high -> try to set low, low try to set high
		GPIO_InitTypeDef GPIO_InitStruct = {0};
		GPIO_InitStruct.Pin = ISMISOL_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
		GPIO_InitStruct.Pull = (HAL_GPIO_ReadPin(ISMISOL_GPIO_Port, ISMISOL_Pin) == GPIO_PIN_SET) ? GPIO_PULLDOWN : GPIO_PULLUP;
		HAL_GPIO_Init(ISMISOL_GPIO_Port, &GPIO_InitStruct);
		osDelay(1);	//not needed (at least during debug), just to be sure....
		_sensorPresent = HAL_GPIO_ReadPin(ISMISOL_GPIO_Port, ISMISOL_Pin) == (GPIO_InitStruct.Pull == GPIO_PULLDOWN);

		/* reset to default state */
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(ISMISOL_GPIO_Port, &GPIO_InitStruct);

		sensorCheck_last = current;
	}

	/* avoid sensor update in case of not present sensor */
	if(sensorPresent == false)
		return;

	/* wind speed, average over 5 sec */
	if(current >= speedAvg_last+5000)
	{
		speed_db[speedIdx] = getSpeed_kmh(windTicker, isMisol());
		if(++speedIdx >= NELEM(speed_db))
			speedIdx = 0;
		speedAvg_last = current;
	}

	/* wind direction */
	dir_db[dirIdx] = isMisol() ? getDir_misol_deg() : getDir_davis_deg();
	if(++dirIdx >= NELEM(dir_db))
		dirIdx = 0;
}

float Wind::getDir_2min_avg(void)
{
	if(sensorPresent == false)
		return NAN;

	/* Calc Wind Direction
	 * You can't just take the average. Google "mean of circular quantities" for more info
	 * We will use the Mitsuta method because it doesn't require trig functions
	 * Based on: http://abelian.org/vlf/bearings.html
	 * Based on: http://stackoverflow.com/questions/1813483/averaging-angles-again
	 */

	float sum = dir_db[0];
	float D = dir_db[0];
	for(unsigned int i=1; i<NELEM(dir_db); i++)
	{
		float delta = dir_db[i] - D;

		if(delta < -180.0f)
			D += delta + 360.0f;
		else if(delta > 180.0f)
			D += delta - 360.0f;
		else
			D += delta;

		sum += D;
	}

	float dir_avg2m = sum / NELEM(dir_db);
	if(dir_avg2m >= 360.0f)
		dir_avg2m -= 360.0f;
	if(dir_avg2m < 0.0f)
		dir_avg2m += 360.0f;

	return dir_avg2m;
}

float Wind::getSpeed_2min_avg(void)
{
	if(sensorPresent == false)
		return NAN;

	float speed_sum = 0.0f;
	for(unsigned int i=0; i<NELEM(speed_db); i++)
		speed_sum += speed_db[i];

	return speed_sum / NELEM(speed_db);
}

float Wind::getSpeed_max(void)
{
	if(sensorPresent == false)
		return NAN;

	float speed_max = 0.0f;
	for(unsigned int i=0; i<NELEM(speed_db); i++)
		if(speed_db[i] > speed_max)
			speed_max = speed_db[i];

	return speed_max;
}

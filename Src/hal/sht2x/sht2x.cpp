/*
 * sht2x.cpp
 *
 *  Created on: 25 Sep 2019
 *      Author: sid
 */

#include <stdio.h>
#include <stdbool.h>

#include "cmsis_os.h"
#include "main.h"
#include "print.h"

#include "../power.h"
#include "si2c_master.h"
#include "sht2x.h"

uint32_t sht2x_forceConversion_next = 0;

void sht2x_task(void const * argument)
{
	sht2x.init();
	osDelay(1000);

	TickType_t xLastWakeTime = xTaskGetTickCount();
	while(1)
	{
		/* sensor conversion */
		//note: in case of forced single shot, temperature will be stores for one cycle (2min)
		const uint32_t current = osKernelSysTick();
#ifdef DIRECTBAT
		const bool doEval = sht2x_forceConversion_next < current || power::getSoc() > 0.2f;
#else
		const bool doEval = sht2x_forceConversion_next < current || power::isSufficiant();
#endif
#ifdef DIRECTBAT
		if(power::critical() == false)
#endif
		{
			sht2x.handle(doEval);
			if(doEval)
				sht2x_forceConversion_next = current + SHT2X_FORCECONVERSION_INTERVALL_MS;
		}

		/* Wait for the next cycle */
		vTaskDelayUntil(&xLastWakeTime, portTICK_PERIOD_MS * (SHT2X_FORCECONVERSION_INTERVALL_MS+5));
	}
}


void Sht2x::init(void)
{
	si2c = si2cInit(SHTSDA_GPIO_Port, SHTSDA_Pin, SHTSCL_GPIO_Port, SHTSCL_Pin);
	config();
}

void Sht2x::handle(bool eval)
{
	/* ignore temperature */
	if(eval == false)
	{
		//note: no race condition here. it gets only written in this thread
		if(std::isnan(temperature) == false || std::isnan(relHumidity) == false)
		{
			debug_printf("Removing SHT2x\n");
			osMutexWait(sensMutex, osWaitForever);
			temperature = NAN;
			relHumidity = NAN;
			osMutexRelease(sensMutex);
		}

		return;
	}

	/* update sensor data */
	if(measureTemp() == false || measureRH() == false)
		config();

	debug_printf("SHT2x %.1fdeg %.1fRH\n", temperature, relHumidity);
}


//calculates 8-Bit checksum with given polynomial
bool Sht2x::checkCrc(uint8_t *data, uint8_t len)
{
	uint8_t crc = 0;
	for(uint8_t byteCtr = 0; byteCtr < len-1; byteCtr++)
	{
		crc ^= (data[byteCtr]);
		for (int8_t bit = 8; bit > 0; bit--)
		{
			if (crc & 0x80)
				crc = (crc << 1) ^ SHT2X_POLYNOMIAL;
			else
				crc = (crc << 1);
		}
	}

	return (crc == data[len-1]);
}

bool Sht2x::measure(uint8_t what, uint16_t delay, uint16_t &result)
{
	uint8_t data[3];

	/* start conversion */
	if(si2cWrite(&si2c, SHT2X_ADDRESS, what, nullptr, 0) == false)
		return false;
	osDelay(delay);

	/* collect data */
	if(si2cRead(&si2c, SHT2X_ADDRESS, 0, 0, data, sizeof(data)) == false)
		return false;
	if(checkCrc(data, sizeof(data)) == false)
		return false;

	/* eval data */
	result = (data[0] << 8) + data[1];
	return true;
}

float Sht2x::calcTemperatureC(uint16_t rawTemp)
{
	rawTemp &= ~0x0003;          							// clear bits [1..0] (status bits)
											//-- calculate temperature [℃] --
	const float temperatureC = -46.85 + 175.72 / 65536 * (float)rawTemp; 		//T= -46.85 + 175.72 * ST/2^16
	return temperatureC;
}

float Sht2x::calcRH(uint16_t rawRH)
{
	rawRH &= ~0x0003;								// clear bits [1..0] (status bits)
											//-- calculate relative humidity [%RH] --
	const float humidityRH = ((float)rawRH * 0.00190735) - 6;			//humidityRH = -6.0 + 125.0/65536 * (float)u16sRH;
											//RH= -6 + 125 * SRH/2^16
	return humidityRH;
}

bool Sht2x::measureTemp(void)
{
	uint16_t rawTemp;
	if(measure(SHT2X_MEASUREMENT_T_nhm, 16, rawTemp) == false)
		return false;

	const float temp = calcTemperatureC(rawTemp);
	osMutexWait(sensMutex, osWaitForever);
	temperature = temp;
	osMutexRelease(sensMutex);

	return true;
}

bool Sht2x::measureRH(void)
{
	uint16_t rawRH;
	if(measure(SHT2X_MEASUREMENT_RH_nhm, 20, rawRH) == false)
		return false;

	const float rh = calcRH(rawRH);
	osMutexWait(sensMutex, osWaitForever);
	relHumidity = rh>100.0f ? 100.0f : rh;
	osMutexRelease(sensMutex);

	return true;
}

bool Sht2x::reset(void)
{
	return si2cWrite(&si2c, SHT2X_ADDRESS, SHT2X_SOFT_RESET, nullptr, 0);
}

bool Sht2x::config(void)
{
	/* ensure known state */
	osDelay(100);									//ensure any ongoing conversion is done
	if(reset() == false)
		return false;
	osDelay(20);

	uint8_t cfg = 0x83;								//11bit resolution for temp+RH
	return si2cWrite(&si2c, SHT2X_ADDRESS, SHT2X_WRITE_REG, &cfg, 1);
}

void Sht2x::get(float *temp, float *rh)
{
	osMutexWait(sensMutex, osWaitForever);
	if(temp != nullptr)
		*temp = temperature;
	if(rh != nullptr)
		*rh = relHumidity;
	osMutexRelease(sensMutex);
}

Sht2x sht2x = Sht2x();

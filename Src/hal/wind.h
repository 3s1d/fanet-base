/*
 * Windsensor.h
 *
 *  Created on: Sep 19, 2017
 *      Author: sid
 */

#ifndef WINDSOCK_WINDSENSOR_H_
#define WINDSOCK_WINDSENSOR_H_

#include "stm32l4xx.h"
#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

/* C */
void wind_rtos(void);
void wind_irq(void);
void wind_task(void const * argument);


#ifdef __cplusplus
}

/*
 * C++
 */


class Wind
{
private:
	/* sensor present */
	uint32_t sensorCheck_last = 0;
	bool _sensorPresent;

	/* speed */
	uint16_t ticker_last = 0;
	uint32_t speed_last = 0;
	uint32_t speedAvg_last = 0;
	uint16_t speedIdx = 0;

	/* direction */
	uint16_t dirIdx = 0;

	/* Misol direction converter */
	#define WSD_RES(a)	(1024.0f*((a)/((a)+wsd_resistork)))
	const float wsd_resistork = 4.7f;
	typedef struct wsd_t
	{
		float dir;
		float ratio;
	} wsd_t;
	const wsd_t wsd[16] =
	{
		{ 0.0f, WSD_RES(33.0f)},
		{ 22.5f, WSD_RES(6.57f)},
		{ 45.0f, WSD_RES(8.2f)},
		{ 67.5f, WSD_RES(0.891f)},
		{ 90.0f, WSD_RES(1.0f)},
		{ 112.5f, WSD_RES(0.688f)},
		{ 135.0f, WSD_RES(2.2f)},
		{ 157.5f, WSD_RES(1.41f)},
		{ 180.0f, WSD_RES(3.9f)},
		{ 202.5f, WSD_RES(3.14f)},
		{ 225.0f, WSD_RES(16.0f)},
		{ 247.5f, WSD_RES(14.12f)},
		{ 270.0f, WSD_RES(120.0f)},
		{ 292.5f, WSD_RES(42.12f)},
		{ 315.0f, WSD_RES(64.9f)},
		{ 337.5f, WSD_RES(21.88f)},
	};

	/* 2min */
	float dir_db[120] = {0.0f};		//every 1sec
	float speed_db[24] = {0.0f};		//every 5sec

	/* Sensor HAL */
	float getSpeed_kmh(uint16_t ticker, bool isMisol);
	float getDir_misol_deg(void);
	float getDir_davis_deg(void);
	bool isMisol(void) { return HAL_GPIO_ReadPin(ISMISOL_GPIO_Port, ISMISOL_Pin) == GPIO_PIN_SET; }

public:
	const bool &sensorPresent;

	Wind() : sensorPresent(_sensorPresent) { }

	void handle(void);
	float getDir_2min_avg(void);
	float getSpeed_2min_avg(void);
	float getSpeed_max(void);

};

extern Wind wind;

#endif

#endif /* WINDSOCK_WINDSENSOR_H_ */

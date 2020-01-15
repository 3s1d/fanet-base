/*
 * sht2x.h
 *
 *  Created on: 25 Sep 2019
 *      Author: sid
 */

#ifndef HAL_SHT2X_SHT2X_H_
#define HAL_SHT2X_SHT2X_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <math.h>

#include "../sht2x/si2c_master.h"

#define SHT2X_FORCECONVERSION_INTERVALL_MS		(20*60*1000)			//20min

void sht2x_task(void const * argument);

#ifdef __cplusplus
}

/*
 * C++
 */

class Sht2x
{
private:
	si2c_t si2c = {0};
	osMutexDef(sensMutex);
	osMutexId sensMutex = osMutexCreate(osMutex(sensMutex));
	float temperature = NAN;
	float relHumidity = NAN;

	bool checkCrc(uint8_t *data, uint8_t len);
	bool measure(uint8_t what, uint16_t delay, uint16_t &result);
	float calcTemperatureC(uint16_t rawTemp);
	float calcRH(uint16_t rawRH);

	bool measureRH(void);
	bool measureTemp(void);
	bool reset(void);
	bool config(void);

public:
	const uint16_t SHT2X_POLYNOMIAL = 0x0131;
	const uint8_t SHT2X_ADDRESS = 0x80;
	const uint8_t SHT2X_MEASUREMENT_RH = 0xE5;
	const uint8_t SHT2X_MEASUREMENT_T = 0xE3;
	const uint8_t SHT2X_MEASUREMENT_RH_nhm = 0xF5;
	const uint8_t SHT2X_MEASUREMENT_T_nhm = 0xF3;
	const uint8_t SHT2X_READ_REG = 0xE7;
	const uint8_t SHT2X_WRITE_REG = 0xE6;
	const uint8_t SHT2X_SOFT_RESET = 0xFE;

	Sht2x() { }
	void init(void);
	void handle(bool eval);

	void get(float *temp, float *rh);
};

extern Sht2x sht2x;

#endif


#endif /* HAL_SHT2X_SHT2X_H_ */

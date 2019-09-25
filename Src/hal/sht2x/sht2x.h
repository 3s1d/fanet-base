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

#include "../sht2x/si2c_master.h"

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
public:
	Sht2x() { }
	void init(void);
	void handle(void);

};

extern Sht2x sht2x;

#endif


#endif /* HAL_SHT2X_SHT2X_H_ */

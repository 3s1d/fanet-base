/*
 * random.h
 *
 *  Created on: May 21, 2017
 *      Author: sid
 */

#ifndef LIB_RANDOM_H_
#define LIB_RANDOM_H_

#include <stdint.h>
#include <stdlib.h>

namespace rnd
{

inline void seed(uint32_t dwSeed) { srand(dwSeed); }

/* The rand function returns the next pseudo-random number in the series. The value ranges from 0 to RAND_MAX */
uint32_t get(uint32_t howbig);
uint32_t get(uint32_t howsmall, uint32_t howbig);

}

#endif /* LIB_RANDOM_H_ */

/*
 * print.h
 *
 *  Created on: Apr 6, 2018
 *      Author: sid
 */

#ifndef PRINT_H_
#define PRINT_H_

#ifdef DEBUG
	#define debug_printf(...)	do { printf("%s:%d: ", __FILE__, __LINE__); printf(__VA_ARGS__); } while(0)
#else
	#define debug_printf(...) 	do { ((void)0U); } while(0)
#endif


#endif /* PRINT_H_ */

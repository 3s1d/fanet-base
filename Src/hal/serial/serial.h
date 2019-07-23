/*
 * serial.h
 *
 *  Created on: May 20, 2017
 *      Author: sid
 */

#ifndef COM_SERIAL_H_
#define COM_SERIAL_H_

#include "cmsis_os.h"

#include "stm32l4xx.h"
#include "stm32l4xx_hal_uart.h"

/* Queue */
#define SERIAL_QUEUE_SIZE			4
#define SERIAL_LINE				1

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);

#ifdef __cplusplus
extern "C" {
#endif


#include <stdbool.h>
#include "stm32l4xx.h"
#include "stm32l4xx_hal_uart.h"

#include "circular_buffer.h"

typedef struct serial_t
{
	UART_HandleTypeDef *uart;
	circBuf_t *rx_crc_buf;
	int pushed_cmds;
	int pulled_cmds;
	osMessageQId queueID;
} serial_t;

namespace serial
{

serial_t *init(UART_HandleTypeDef *uart);

bool poll(serial_t *serial, char *line, int num);

void print(const char *str, serial_t *serial = nullptr);

}

#ifdef __cplusplus
}
#endif

#endif /* COM_SERIAL_H_ */

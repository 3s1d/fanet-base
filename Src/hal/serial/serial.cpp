/*
 * serial.c
 *
 *  Created on: May 20, 2017
 *      Author: sid
 */

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "cmsis_os.h"

#include "stm32l4xx.h"
#include "stm32l4xx_hal_uart.h"

#include "circular_buffer.h"
#include "serial.h"

/*
 * serial input -> buffer
 */

CIRC_BUF_DEF(serial_uart_rx_buffer, 512);
serial_t serialPort = {0};

uint8_t serial_uart_inchr = '\0';
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart != serialPort.uart)
		return;

	circ_buf_push(&serial_uart_rx_buffer, serial_uart_inchr);
	if (serial_uart_inchr == '\n')
	{
		serialPort.pushed_cmds++;
		osMessagePut(serialPort.queueID, SERIAL_LINE, osWaitForever);
	}

	HAL_UART_Receive_IT(serialPort.uart, &serial_uart_inchr, 1);
}



namespace serial
{

serial_t *init(UART_HandleTypeDef *uart)
{
	serialPort.uart = uart;
	serialPort.rx_crc_buf = &serial_uart_rx_buffer;

	/* Queue */
	osMessageQDef(Serial_Queue, SERIAL_QUEUE_SIZE, uint16_t);
	serialPort.queueID = osMessageCreate (osMessageQ(Serial_Queue), NULL);

	/* enable uart receiver */
	HAL_UART_Receive_IT(serialPort.uart, &serial_uart_inchr, 1);

	return &serialPort;
}

bool poll(serial_t *serial, char *line, int num)
{
	if(serial == nullptr || serialPort.uart == nullptr)
		return false;

	/* ensure receiver is on */
	HAL_UART_Receive_IT(serial->uart, &serial_uart_inchr, 1);

	if(serial->pushed_cmds == serial->pulled_cmds)
		return false;

	int pos;
	for(pos=0; circ_buf_pop(serial->rx_crc_buf, (uint8_t *) &line[pos])==0 && pos < num; pos++)
	{
		if(line[pos] == '\n')
		{
			serial->pulled_cmds++;
			break;
		}
	}

	if(pos >= 0 && pos < num)
		line[pos] = '\0';

	return true;
}

void print(const char *str, serial_t *ser)
{
	/* use default */
	if(ser == nullptr)
		ser = &serialPort;

	if(ser == nullptr || ser->uart == nullptr || str == nullptr)
		return;

	while(HAL_UART_Transmit_IT(ser->uart, (uint8_t *)str, strlen(str)) == HAL_BUSY);
	while(ser->uart->gState != (__IO HAL_UART_StateTypeDef) HAL_UART_STATE_READY);
}

} //namespace

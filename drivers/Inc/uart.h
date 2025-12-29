/*
 * uart.h
 *
 *  Created on: Jun 27, 2025
 *      Author: abhin
 */

#ifndef UART_H_
#define UART_H_
char uart2_read(void);
int uart2_read_nonblocking(char *out);
void uart2_rxtx_init(void);
void uart2_write(int ch);   
int __io_putchar(int ch);

#endif /* UART_H_ */

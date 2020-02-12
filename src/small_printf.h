/*
 * small_printf.h
 *
 *  Created on: 30-Dec-2019
 *      Author: arun
 */

#ifndef SMALL_PRINTF_H_
#define SMALL_PRINTF_H_

#define PRINTF_USART USART1

void uart_putc(char dataByte);
void uart_puts(char *dataString);
void xtoa(unsigned long x, const unsigned long *dp);
void puth(unsigned n);
void uart_printf(char *format, ...);


#endif /* SMALL_PRINTF_H_ */

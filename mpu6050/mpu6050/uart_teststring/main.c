/*************************************************************************
Title:    Example program for the Interrupt controlled UART library
Author:   Peter Fleury <pfleury@gmx.ch>   http://tinyurl.com/peterfleury
File:     $Id: test_uart.c,v 1.7 2015/01/31 17:46:31 peter Exp $
Software: AVR-GCC 4.x
Hardware: AVR with built-in UART/USART

DESCRIPTION:
          This example shows how to use the UART library uart.c

*************************************************************************/
#include <stdlib.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>

#include "uart/uart.h"


/* define CPU frequency in Hz in Makefile */
#ifndef F_CPU
#error "F_CPU undefined, please define CPU frequency in Hz in Makefile"
#endif

/* Define UART buad rate here */
#define UART_BAUD_RATE      115200      


int main(void)
{    
    /*
     *  Initialize UART library, pass baudrate and AVR cpu clock
     *  with the macro 
     *  UART_BAUD_SELECT() (normal speed mode )
     *  or 
     *  UART_BAUD_SELECT_DOUBLE_SPEED() ( double speed mode)
     */
    uart_init( UART_BAUD_SELECT_DOUBLE_SPEED(UART_BAUD_RATE,F_CPU) ); 
    
    /*
     * now enable interrupt, since UART library is interrupt controlled
     */
    sei();
    while(1){
		uart_putc('U');
	}
    
}

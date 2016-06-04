/*************************************************************************
Title:    Interrupt UART library with receive/transmit circular buffers
Author:   Peter Fleury <pfleury@gmx.ch>   http://jump.to/fleury
File:     $Id: uart.c,v 1.6.2.1 2007/07/01 11:14:38 peter Exp $
Software: AVR-GCC 4.1, AVR Libc 1.4.6 or higher
Hardware: any AVR with built-in UART, 
License:  GNU General Public License 
          
DESCRIPTION:
    An interrupt is generated when the UART has finished transmitting or
    receiving a byte. The interrupt handling routines use circular buffers
    for buffering received and transmitted data.
    
    The UART_RX_BUFFER_SIZE and UART_TX_BUFFER_SIZE variables define
    the buffer size in bytes. Note that these variables must be a 
    power of 2.
    
USAGE:
    Refere to the header file uart.h for a description of the routines. 
    See also example test_uart.c.

NOTES:
    Based on Atmel Application Note AVR306
                    
LICENSE:
    Copyright (C) 2006 Peter Fleury

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
                        
*************************************************************************/
/************************************************************************
uart_available, uart_flush, uart1_available, and uart1_flush functions
were adapted from the Arduino HardwareSerial.h library by Tim Sharpe on 
11 Jan 2009.  The license info for HardwareSerial.h is as follows:

  HardwareSerial.cpp - Hardware serial library for Wiring
  Copyright (c) 2006 Nicholas Zambetti.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
  
  Modified 23 November 2006 by David A. Mellis
************************************************************************/
/************************************************************************
Changelog for modifications made by Tim Sharpe, starting with the current
  library version on his Web site as of 05/01/2009. 

Date        Description
=========================================================================
05/11/2009  Changed all existing UARTx_RECEIVE_INTERRUPT and UARTx_TRANSMIT_INTERRUPT
              macros to use the "_vect" format introduced in AVR-Libc
			  v1.4.0.  Had to split the 3290 and 6490 out of their existing
			  macro due to an inconsistency in the UART0_RECEIVE_INTERRUPT 
			  vector name (seems like a typo: USART_RX_vect for the 3290/6490
			  vice USART0_RX_vect for the others in the macro).
			Verified all existing macro register names against the device
			  header files in AVR-Libc v1.6.6 to catch any inconsistencies.
05/12/2009  Added support for 48P, 88P, 168P, and 328P by adding them to the
               existing 48/88/168 macro.
			Added Arduino-style available() and flush() functions for both
			supported UARTs.  Really wanted to keep them out of the library, so
			that it would be as close as possible to Peter Fleury's original
			library, but has scoping issues accessing internal variables from
			another program.  Go C!
05/13/2009  Changed Interrupt Service Routine label from the old "SIGNAL" to
               the "ISR" format introduced in AVR-Libc v1.4.0.

************************************************************************/
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include "uart.h"

/*
 *  constants and macros
 */
/* size of RX/TX buffers */
#define UART_RX_BUFFER_MASK (UART_RX_BUFFER_SIZE - 1)
#define UART_TX_BUFFER_MASK (UART_TX_BUFFER_SIZE - 1)

#if (UART_RX_BUFFER_SIZE & UART_RX_BUFFER_MASK)
#error RX buffer size is not a power of 2
#endif
#if (UART_TX_BUFFER_SIZE & UART_TX_BUFFER_MASK)
#error TX buffer size is not a power of 2
#endif

#if defined(__AVR_ATmega8__) || defined(__AVR_ATmega16__) || defined(__AVR_ATmega32__) || defined(__AVR_ATmega323__)

/* ATmega with one USART */
#define ATMEGA_USART
#define UART0_RECEIVE_INTERRUPT     USART_RXC_vect
#define UART0_TRANSMIT_INTERRUPT    USART_UDRE_vect
#define UART0_STATUS                UCSRA
#define UART0_CONTROL               UCSRB
#define UART0_DATA                  UDR
#define UART0_UDRIE                 UDRIE
#elif defined(__AVR_ATmega48__)     \
    ||  defined(__AVR_ATmega88__)   \
    ||  defined(__AVR_ATmega168__)  \
    ||  defined(__AVR_ATmega48P__)  \
    ||  defined(__AVR_ATmega88P__)  \
    ||  defined(__AVR_ATmega168P__) \
    ||  defined(__AVR_ATmega328P__)

    /* TLS-Added 48P/88P/168P/328P */
    /* ATmega with one USART */
#define ATMEGA_USART0
#define UART0_RECEIVE_INTERRUPT     USART_RX_vect
#define UART0_TRANSMIT_INTERRUPT    USART_UDRE_vect
#define UART0_STATUS                UCSR0A
#define UART0_CONTROL               UCSR0B
#define UART0_DATA                  UDR0
#define UART0_UDRIE                 UDRIE0
#else
#error "no UART definition for MCU available"
#endif

    /*
 *  module global variables
 */
    static volatile unsigned char   UART_TxBuf[UART_TX_BUFFER_SIZE];
static volatile unsigned char   UART_RxBuf[UART_RX_BUFFER_SIZE];
static volatile unsigned char   UART_TxHead;
static volatile unsigned char   UART_TxTail;
static volatile unsigned char   UART_RxHead;
static volatile unsigned char   UART_RxTail;
static volatile unsigned char   UART_LastRxError;

//ISR(UART0_RECEIVE_INTERRUPT)
#if defined(USART_RX_vect)
  ISR(USART_RX_vect)
#elif defined(USART0_RX_vect)
  ISR(USART0_RX_vect)
#elif defined(USART_RXC_vect)
  ISR(USART_RXC_vect) // ATmega8
#else
  #error "Don't know what the Data Received vector is called for Serial"
#endif

/*************************************************************************
Function: UART Receive Complete interrupt
Purpose:  called when the UART has received a character
**************************************************************************/
{
    unsigned char   tmphead;
    unsigned char   data;
    unsigned char   usr;
    unsigned char   lastRxError;

    /* read UART status register and UART data register */
    usr = UART0_STATUS;
    data = UART0_DATA;

    /* */
#if defined(ATMEGA_USART)
    lastRxError = (usr & (_BV(FE) | _BV(DOR)));
#elif defined(ATMEGA_USART0)
    lastRxError = (usr & (_BV(FE0) | _BV(DOR0)));
#endif

    /* calculate buffer index */
    tmphead = (UART_RxHead + 1) & UART_RX_BUFFER_MASK;

    if(tmphead == UART_RxTail) {

        /* error: receive buffer overflow */
        lastRxError = UART_BUFFER_OVERFLOW >> 8;
    }
    else {

        /* store new index */
        UART_RxHead = tmphead;

        /* store received data in buffer */
        UART_RxBuf[tmphead] = data;
    }

    UART_LastRxError = lastRxError;
}

//ISR(UART0_TRANSMIT_INTERRUPT)
#if defined(UART0_UDRE_vect)
ISR(UART0_UDRE_vect)
#elif defined(UART_UDRE_vect)
ISR(UART_UDRE_vect)
#elif defined(USART0_UDRE_vect)
ISR(USART0_UDRE_vect)
#elif defined(USART_UDRE_vect)
ISR(USART_UDRE_vect)
#else
  #error "Don't know what the Data Register Empty vector is called for Serial"
#endif
/*************************************************************************
Function: UART Data Register Empty interrupt
Purpose:  called when the UART is ready to transmit the next byte
**************************************************************************/
{
    unsigned char   tmptail;

    if(UART_TxHead != UART_TxTail) {

        /* calculate and store new buffer index */
        tmptail = (UART_TxTail + 1) & UART_TX_BUFFER_MASK;
        UART_TxTail = tmptail;

        /* get one byte from buffer and write it to UART */
        UART0_DATA = UART_TxBuf[tmptail];   /* start transmission */
    }
    else {

        /* tx buffer empty, disable UDRE interrupt */
        UART0_CONTROL &= ~_BV(UART0_UDRIE);
    }
}

/*************************************************************************
Function: uart_init()
Purpose:  initialize UART and set baudrate
Input:    baudrate using macro UART_BAUD_SELECT()
Returns:  none
**************************************************************************/
void uart_init(uint32_t    baudrate)
{
	uint16_t baud_setting = (F_CPU / 4 / baudrate - 1) / 2;

    UART_TxHead = 0;
    UART_TxTail = 0;
    UART_RxHead = 0;
    UART_RxTail = 0;
	// Try u2x mode first

	UART0_STATUS = 1 << U2X0;
	
	// hardcoded exception for 57600 for compatibility with the bootloader
	// shipped with the Duemilanove and previous boards and the firmware
	// on the 8U2 on the Uno and Mega 2560. Also, The baud_setting cannot
	// be > 4095, so switch back to non-u2x mode if the baud rate is too
	// low.
	if (((F_CPU == 16000000UL) && (baudrate == 57600)) || (baud_setting >4095))
	{
	  UART0_STATUS = 0;
	  baud_setting = (F_CPU / 8 / baudrate - 1) / 2;
	}

#if defined(ATMEGA_USART)
    UBRRH = (unsigned char)(baud_setting>>8);
    UBRRL = (unsigned char) baud_setting;

    /* Enable USART receiver and transmitter and receive complete interrupt */
    UART0_CONTROL = _BV(RXCIE) | (1 << RXEN) | (1 << TXEN);

    /* Set frame format: asynchronous, 8data, no parity, 1stop bit */
#ifdef URSEL
    UCSRC = (1 << URSEL) | (3 << UCSZ0);
#else
    UCSRC = (3 << UCSZ0);
#endif

#elif defined(ATMEGA_USART0)
	
	// assign the baud_setting, a.k.a. ubrr (USART Baud Rate Register)
	UBRR0H = baud_setting >> 8;
	UBRR0L = baud_setting;

    /* Enable USART receiver and transmitter and receive complete interrupt */
    UART0_CONTROL |= (_BV(RXEN0) | _BV(TXEN0) | _BV(RXCIE0));
	UART0_CONTROL &= ~_BV(UDRIE0);

    /* Set frame format: asynchronous, 8data, no parity, 1stop bit */
#ifdef URSEL0
    UCSR0C = (1 << URSEL0) | (3 << UCSZ00);
#else
    UCSR0C |= ((1 << UCSZ01) | (1 << UCSZ00));
#endif
#endif

}   /* uart_init */

/*************************************************************************
Function: uart_getc()
Purpose:  return byte from ringbuffer  
Returns:  lower byte:  received byte from ringbuffer
          higher byte: last receive error
**************************************************************************/
unsigned int uart_getc(void)
{
    unsigned char   tmptail;
    unsigned char   data;

    if(UART_RxHead == UART_RxTail) {
        return UART_NO_DATA;    /* no data available */
    }

    /* calculate /store buffer index */
    tmptail = (UART_RxTail + 1) & UART_RX_BUFFER_MASK;
    UART_RxTail = tmptail;

    /* get data from receive buffer */
    data = UART_RxBuf[tmptail];

    return (UART_LastRxError << 8) + data;

}   /* uart_getc */

/*************************************************************************
Function: uart_putc()
Purpose:  write byte to ringbuffer for transmitting via UART
Input:    byte to be transmitted
Returns:  none          
**************************************************************************/
void uart_putc(unsigned char   data)
{
    unsigned char   tmphead;

    tmphead = (UART_TxHead + 1) & UART_TX_BUFFER_MASK;

    while(tmphead == UART_TxTail) {
        ;   /* wait for free space in buffer */
    }

    UART_TxBuf[tmphead] = data;
    UART_TxHead = tmphead;

    /* enable UDRE interrupt */
    UART0_CONTROL |= _BV(UART0_UDRIE);

}           /* uart_putc */

/*************************************************************************
Function: uart_puts()
Purpose:  transmit string to UART
Input:    string to be transmitted
Returns:  none          
**************************************************************************/
void uart_puts(const char*     s)
{
    while(*s) {
        uart_putc(*s++);

    }

}   /* uart_puts */

/*************************************************************************
Function: uart_puts_p()
Purpose:  transmit string from program memory to UART
Input:    program memory string to be transmitted
Returns:  none
**************************************************************************/
void uart_puts_p(const char*     progmem_s)
{
    register char   c;

    while((c = pgm_read_byte(progmem_s++))) {
        uart_putc(c);

    }

}   /* uart_puts_p */

/*************************************************************************
Function: uart_available()
Purpose:  Determine the number of bytes waiting in the receive buffer
Input:    None
Returns:  Integer number of bytes in the receive buffer
**************************************************************************/
int uart_available(void)
{
    return (UART_RX_BUFFER_MASK + UART_RxHead - UART_RxTail) % UART_RX_BUFFER_MASK;
}   /* uart_available */

/*************************************************************************
Function: uart_flush()
Purpose:  Flush bytes waiting the receive buffer.  Actually ignores them.
Input:    None
Returns:  None
**************************************************************************/
void uart_flush(void)
{
    UART_RxHead = UART_RxTail;
}   /* uart_flush */

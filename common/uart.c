/*************************************************************************
Title:    Interrupt UART library with receive/transmit circular buffers
Author:   Peter Fleury <pfleury@gmx.ch>   http://jump.to/fleury
File:     $Id: uart.c,v 1.12 2014/01/08 21:58:12 peter Exp $
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
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <uart.h>
#include "snap.h"


/*
 *  module global variables
 */
#ifndef __SNAP__
static volatile unsigned char UART_RxBuf[UART_RX_BUFFER_SIZE];
static volatile unsigned char UART_RxHead;
static volatile unsigned char UART_RxTail;
static volatile unsigned char UART_LastRxError;
#endif

static volatile unsigned char UART_TxBuf[UART_TX_BUFFER_SIZE];
static volatile unsigned char UART_TxHead;
static volatile unsigned char UART_TxTail;



ISR (UART0_RECEIVE_INTERRUPT)	
/*************************************************************************
Function: UART Receive Complete interrupt
Purpose:  called when the UART has received a character
**************************************************************************/
{
    unsigned char data;
    unsigned char usr;
    unsigned char lastRxError;

#ifndef __SNAP__	
    unsigned char tmphead;
#endif 
 
    /* read UART status register and UART data register */ 
    usr  = UART0_STATUS;
    data = UART0_DATA;
    
    /* */

	#if  defined( ATMEGA_USART )
		lastRxError = (usr & (_BV(FE)|_BV(DOR)) );
	#elif defined( ATMEGA_USART0 )
		lastRxError = (usr & (_BV(FE0)|_BV(DOR0)) );
	#endif

#if defined ( __USE_SYNC__ )
 #if defined( ATMEGA_USART ) 
	if (usr & _BV(FE) && data==0x00)
 #elif defined( ATMEGA_USART0 )	
	if (usr & _BV(FE0) && data==0x00)
 #else
	#error Wrong U(S)ART!
 #endif
	
	{
		//assume we have a break -> start sync-process
		DISABLE_UART_RECEIVER;
		
		EIFR |= _BV(INTF1);
		
		//Set INT1 to trigger on falling edge
		INT1_ON_FALLING_EDGE;
		
		//INT1_ENABLE;
	}
    else
	{  
#endif	//__USE_SYNC__	  

#if defined ( __SNAP__ )
		if (!lastRxError)
			SnapReceiveChar(data);
#else
		/* calculate buffer index */ 
		tmphead = ( UART_RxHead + 1) & UART_RX_BUFFER_MASK;
    
		if ( tmphead == UART_RxTail ) {
			/* error: receive buffer overflow */
			lastRxError = UART_BUFFER_OVERFLOW >> 8;
		}else{
			/* store new index */
			UART_RxHead = tmphead;
			/* store received data in buffer */
			UART_RxBuf[tmphead] = data;
		}

		UART_LastRxError |= lastRxError;
#endif

#if defined ( __USE_SYNC__ )
	}
#endif
}


ISR (UART0_TRANSMIT_INTERRUPT)
/*************************************************************************
Function: UART Data Register Empty interrupt
Purpose:  called when the UART is ready to transmit the next byte
**************************************************************************/
{
    unsigned char tmptail;
    
    if ( UART_TxHead != UART_TxTail) {
        /* calculate and store new buffer index */
        tmptail = (UART_TxTail + 1) & UART_TX_BUFFER_MASK;
        UART_TxTail = tmptail;

#ifdef RS485_DIR_SWITCH		
		/*switch RX485-DIR-Line to high (enable Transmitter)*/
		RS485_DIR_TX_ON;
		
		UART0_CONTROL |= _BV(UART0_UDCIE);
#endif		
        /* get one byte from buffer and write it to UART */
        UART0_DATA = UART_TxBuf[tmptail];  /* start transmission */
    }else{
        /* tx buffer empty, disable UDRE interrupt */
        UART0_CONTROL &= ~_BV(UART0_UDRIE);
    }
}



#ifdef RS485_DIR_SWITCH
ISR (UART0_TRANSMIT_READY_INTERRUPT)
/*************************************************************************
Function: UART TX-Ready interrupt
Purpose:  called when the UART has completed tranmitting a byte
**************************************************************************/
{
    if ( UART_TxHead == UART_TxTail) {
		RS485_DIR_TX_OFF;
			
		UART0_CONTROL &= ~_BV(UART0_UDCIE);
	}			
}
#endif

/*************************************************************************
Function: uart_init()
Purpose:  initialize UART and set baudrate
Input:    baudrate using macro UART_BAUD_SELECT()
Returns:  none
**************************************************************************/
void uart_init(unsigned int baudrate)
{
    UART_TxHead = 0;
    UART_TxTail = 0;
#ifndef __SNAP__
    UART_RxHead = 0;
    UART_RxTail = 0;
#endif
    
#if defined (ATMEGA_USART)
    /* Set baud rate */
    if ( baudrate & 0x8000 )
    {
    	 UART0_STATUS = (1<<U2X);  //Enable 2x speed 
    	 baudrate &= ~0x8000;
    }
    UBRRH = (unsigned char)(baudrate>>8);
    UBRRL = (unsigned char) baudrate;
   
    /* Enable USART receiver and transmitter and receive complete interrupt */
    UART0_CONTROL = _BV(RXCIE)|(1<<RXEN)|(1<<TXEN);

	#ifdef RS485_DIR_SWITCH
		RS485_DIR_TX_OFF;
		RS485_DIR_SWITCH_DIR |= _BV(RS485_DIR_SWITCH_PIN);
	#endif
    
    /* Set frame format: asynchronous, 8data, no parity, 1stop bit */
    #ifdef URSEL
		UCSRC = (1<<URSEL)|(3<<UCSZ0);
    #else
		UCSRC = (3<<UCSZ0);
    #endif 
    
#elif defined (ATMEGA_USART0 )
    /* Set baud rate */
    if ( baudrate & 0x8000 ) 
    {
   		UART0_STATUS = (1<<U2X0);  //Enable 2x speed 
   		baudrate &= ~0x8000;
   	}
    UBRR0H = (unsigned char)(baudrate>>8);
    UBRR0L = (unsigned char) baudrate;

    /* Enable USART receiver and transmitter and receive complete interrupt */
    UART0_CONTROL = _BV(RXCIE0)|(1<<RXEN0)|(1<<TXEN0);

#ifdef RS485_DIR_SWITCH
	RS485_DIR_TX_OFF;
	RS485_DIR_SWITCH_DIR |= _BV(RS485_DIR_SWITCH_PIN);
#endif
    
    /* Set frame format: asynchronous, 8data, no parity, 1stop bit */
    #ifdef URSEL0
		UCSR0C = (1<<URSEL0)|(3<<UCSZ00);
    #else
		UCSR0C = (3<<UCSZ00);
    #endif 

#endif

}/* uart_init */




/*************************************************************************
Function: uart_putc()
Purpose:  write byte to ringbuffer for transmitting via UART
Input:    byte to be transmitted
Returns:  none          
**************************************************************************/
void uart_putc(unsigned char data)
{
    unsigned char tmphead;
    
    tmphead  = (UART_TxHead + 1) & UART_TX_BUFFER_MASK;
    
    while ( tmphead == UART_TxTail ){
        ;/* wait for free space in buffer */
    }
    
    UART_TxBuf[tmphead] = data;
    UART_TxHead = tmphead;

    /* enable UDRE interrupt */
    UART0_CONTROL |= _BV(UART0_UDRIE);

}/* uart_putc */


/*************************************************************************
Function: uart_puts()
Purpose:  transmit string to UART
Input:    string to be transmitted
Returns:  none          
**************************************************************************/
void uart_puts(const char *s )
{
    while (*s) 
      uart_putc(*s++);

}/* uart_puts */

/*************************************************************************
Function: uart_getc()
Purpose:  return byte from ringbuffer  
Returns:  lower byte:  received byte from ringbuffer
          higher byte: last receive error
**************************************************************************/
#ifndef __SNAP__
unsigned int uart_getc(void)
{    
    unsigned char tmptail;
    unsigned char data;


    if ( UART_RxHead == UART_RxTail ) {
        return UART_NO_DATA;   /* no data available */
    }
    
    /* calculate /store buffer index */
    tmptail = (UART_RxTail + 1) & UART_RX_BUFFER_MASK;
    UART_RxTail = tmptail; 
    
    /* get data from receive buffer */
    data = UART_RxBuf[tmptail];
    
    data = (UART_LastRxError << 8) + data;
    UART_LastRxError = 0;
    return data;

}/* uart_getc */
#endif


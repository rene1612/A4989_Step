//----------------------------------------------------------------------------------
//
// Copyright (c) 2003 Brian Low (mportobello@hotmail.com)
//
// Permission is hereby granted, free of charge, to any person obtaining a copy of
// this software and associated documentation files (the "Software"), to deal in
// the Software without restriction, including without limitation the rights to
// use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
// the Software, and to permit persons to whom the Software is furnished to do so,
// subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
// FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
// COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
// IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
// CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//
//----------------------------------------------------------------------------------

//----------------------------------------------------------------------------------
//
// SNAP Protocol Library
//
// Simple library for sending and receiving Snap packets.
//
// The library has the following features:
//	- easy to use
//	- collision avoidance (e.g. RS485 half-duplex networks)
//	- crc-16 error detection
//	- interrupt-based receiving
//	- blocking transmitting
//
// The Snap protocol is very flexible. See http://www.hth.com for more
// information. The library has the following restrictions to keep it simple:
//	- 3 byte addresses (16.7 million nodes)
//	- crc-16 error detection
//  - command-mode bits and protocol specific flags not supported
//	- 2 preamble bytes to improve packet start detection
//
//
//----------------------------------------------------------------------------------
#ifdef __SNAP__

#include <avr/io.h>
//#include <avr/signal.h>
#include <avr/interrupt.h>
#include <stdlib.h>
//#include "delay.h"
#include "uart.h"
#include "crc.h"
#include "snap.h"
#include "config.h"

#ifdef __BOOTLOADER__
 #include "prog.h"
#else
 #include "light_panel_node.h"
#endif


// Snap Receiver States
#define SNAP_STATE_START			0
#define SNAP_STATE_PREAMBLE1		0
#define SNAP_STATE_PREAMBLE2		1
#define SNAP_STATE_SYNC				2
#define SNAP_STATE_HEADER			3
#define SNAP_STATE_LENGTH			4
#define SNAP_STATE_DEST				5
#define SNAP_STATE_SOURCE			6
#define SNAP_STATE_DATA				7
#define SNAP_STATE_CRC1				8
#define SNAP_STATE_CRC2				9
#define	SNAP_STATE_PACKET_RECEIVED	99


// Snap receiver variables
unsigned char	SnapRxState;				// Current state of the receiver
unsigned char	SnapRxDataIndex;			// Index of next byte to write to SnapPacketData

// Address of this node
unsigned char SnapNodeAddress = 123;
static unsigned int crc;



//----------------------------------------------------------------------------------
// Initialize the Snap library
//  - set the device's address
//	- reset the receiver
//  - initial the UART
//----------------------------------------------------------------------------------
void SnapInit(unsigned char nodeAddress)
{
	// Save this node's address
	SnapNodeAddress = nodeAddress;

	// Reset receiver state
	SnapReset();

	// Configure uart pin (for detecting uart activity)
	//cbi(UART_DDR, UART_PIN);						// Set direction to input
	//sbi(UART_PORT, UART_PIN);						// enable pull-up resistors

	// Set seed for rand()
	//srand(nodeAddress);
}


//----------------------------------------------------------------------------------
// Reset the receiver discarding any existing data/packets received
//----------------------------------------------------------------------------------
void SnapDecodeReceived(unsigned char *pReg)
{
	unsigned char i;
	unsigned char* p_reg;
	uint8_t ack_nak_flag=SNAP_ACK;
	
	if (SnapPacketDataLength){
		
		p_reg=pReg+SnapPacketData[0];
		
		if (SnapPacketHeader & SNAP_RW_REG_MASK){
			
#ifdef __BOOTLOADER__
			if ((SnapPacketData[0]+SnapPacketDataLength-1)<=sizeof(struct PROG_ENV)){
#else
			if ((SnapPacketData[0]+SnapPacketDataLength-1)<=sizeof(MAIN_REGS)){
#endif
				//Register wird geschrieben
				for (i = 0; i<(SnapPacketDataLength-1); i++)
					p_reg[i] = SnapPacketData[i+1];
			
				ack_nak_flag = SNAP_ACK;
			}
			else
				ack_nak_flag = SNAP_NAK;
			
		}
		else{
			
#ifdef __BOOTLOADER__
			if ((SnapPacketData[0]+SnapPacketDataLength-1)<=sizeof(struct PROG_ENV)){
#else
			if ((SnapPacketData[0]+SnapPacketDataLength-1)<=sizeof(MAIN_REGS)){
#endif
				//Register wird gelesen
				if (SnapSend(SNAP_ACK, (unsigned char *)p_reg, SnapPacketData[1])==0xFF)
					ack_nak_flag=SNAP_NAK;
				else
					SnapPacketHeader&=~SNAP_ACK_MASK;
			}
			else
				ack_nak_flag = SNAP_NAK;
		}
	}
		
#ifndef __BOOTLOADER__
	if(SnapPacketDest!=BROADCAST_ADDR)
#endif
		if (SnapPacketHeader & SNAP_ACK_REQUEST){
			SnapSend(ack_nak_flag, NULL, 0);	
		}
		
	SnapReset();
}


//----------------------------------------------------------------------------------
// Reset the receiver discarding any existing data/packets received
//----------------------------------------------------------------------------------
void SnapReset(void)
{
	SnapRxState = SNAP_STATE_START;
	//SnapPacketDataLength = 0;
	//SnapPacketSource = 0;
	//SnapPacketHeader=0;
	crc = CRC_INITIAL_VALUE;
	//SnapPacketCRC=0;
}

//----------------------------------------------------------------------------------
// Returns non-zero if a complete packet has been received.
// Call SnapReset() when done with the packet to enable receiving the next packet.
//----------------------------------------------------------------------------------
inline char SnapPacketReceived(void)
{
	return( SnapRxState == SNAP_STATE_PACKET_RECEIVED );
}

//----------------------------------------------------------------------------------
// Process a single character as part of a Snap packet.
//
// To determine when a complete packet is received
//	if (SnapPacketReceived()) ...
//
// Once a complete packet is received, all subsequent incoming
// characters will be ignored until you set
//	SnapReset();
//
// This method is implemented as a state machine. As each
// character is received, if it is expected based on the Snap
// protocol then the state machine is advanced.
//----------------------------------------------------------------------------------
void SnapReceiveChar(unsigned char ch)
{

	if ((SnapRxState >= SNAP_STATE_HEADER) & (SnapRxState <= SNAP_STATE_DATA))
		crc = calcCRC16r(crc, ch);

	switch (SnapRxState)
	{
		case SNAP_STATE_PREAMBLE1: 	
									if (ch == SNAP_PREAMBLE1) 
									{
										SnapRxState++;
									}
									break;
									
		case SNAP_STATE_PREAMBLE2:	
									if (ch == SNAP_PREAMBLE2){
										SnapRxState++;
									}
									else{ 
										SnapRxState=0;
									}
									break;
									
		case SNAP_STATE_SYNC: 		
									if (ch == SNAP_SYNC){ 
										SnapRxState++;
									}
									else {
										SnapRxState=0; 
									}
									break;
		
		case SNAP_STATE_HEADER:
									SnapPacketHeader = ch;
									SnapRxState++;
									break;
									
		case SNAP_STATE_LENGTH:		SnapPacketDataLength = ch & 0x7F;
									SnapRxState++;
									break;
									
		case SNAP_STATE_DEST:		SnapPacketDest = ch;
#ifdef __BOOTLOADER__
									if (SnapPacketDest == SnapNodeAddress)
#else
									if (SnapPacketDest == SnapNodeAddress || SnapPacketDest == BROADCAST_ADDR)
#endif									
									{
										SnapRxState++;
									}
									else{
										SnapRxState=0;
									}
									break;
									
		case SNAP_STATE_SOURCE	:	SnapPacketSource = ch; SnapRxState++;
									SnapRxDataIndex = 0;
									if (SnapPacketDataLength == 0)
										SnapRxState++;
									break;

		case SNAP_STATE_DATA:		SnapPacketData[SnapRxDataIndex] = ch;
									SnapRxDataIndex++;
									if (SnapRxDataIndex >= SnapPacketDataLength)
										SnapRxState++;
									break;

		case SNAP_STATE_CRC1:		SnapPacketCRC = (ch << 8);
									SnapRxState++;
									break;

		case SNAP_STATE_CRC2:		SnapPacketCRC += ch;
									if (crc == SnapPacketCRC)
									{
#ifdef __BOOTLOADER__
										if (SnapPacketDest == SnapNodeAddress)
#else
										if (SnapPacketDest == SnapNodeAddress || SnapPacketDest == BROADCAST_ADDR)
#endif
										{
											SnapRxState = SNAP_STATE_PACKET_RECEIVED;
											return;
										}
									}
									
									SnapReset();
									break;
	}

}



//----------------------------------------------------------------------------------
// Send a SNAP packet.
//
// This method will block until the full packet is sent.
//
// Collision avoidance:
// This method waits the time is takes to transmit 1 full character. If no activity
// is detected the packet is transmitted. If activitiy is detected the method waits
// a random amount time after the last detected activity before transmitting.
//
// Returns 0 if successful. Returns 0xFF if data is larger than 512 bytes
//----------------------------------------------------------------------------------
char SnapSend(unsigned char ack_nack, unsigned char data[], unsigned char length)
{
	unsigned char c;
//	unsigned char n;
	unsigned char  i;
//	unsigned int  padding = 0;
	unsigned int  crc=CRC_INITIAL_VALUE;
	unsigned char header[4];


	// Header byte 2
	c = ack_nack;
	header[0] = c;

	// Determine the size of the packet. The protocol allows 0 to 512 bytes of
	// data but not all sizes within that are supported. In the header, if bit 3 = 0,
	// then bits 2..0 indicate the exact size in bytes. If bit 3 = 1, then bits 2..0
	// the size of the data is 2^(bits+3). If the data is greater than 7 bytes then
	// we will pick the next largest size.
	if (length > SNAP_MAX_LENGTH)
		return( 0xFF );

	header[1] = length;

	// Destination address (3 bytes)
//	header[2] = (unsigned char)(dest >> 16);
//	header[3] = (unsigned char)(dest >> 8);
//	header[4] = (unsigned char)(dest >> 0);
	header[2] = (unsigned char)SnapPacketSource;

	// Source address (3 bytes)
//	header[5] = (unsigned char)(SnapNodeAddress >> 16);
//	header[6] = (unsigned char)(SnapNodeAddress >> 8);
//	header[7] = (unsigned char)(SnapNodeAddress >> 0);
	header[3] = (unsigned char)SnapNodeAddress;

	// Check for clear line
	// Wait the time it take for 1 full character (10 bits) to be sent
/*	
	for (i=(1.0/(UART_BAUD/10))*1000000/MIN_DELAY_US; i>0; i--)
	{
		if (bit_is_set(UART_PIN, UART_RX))
			break;
		DELAY(MIN_DELAY_US);
	}

	// Is the line in use? If so wait for the end of the transmission
	// plus a random amount of time to avoid colliding with other
	// devices
	while (i > 0)
	{
		// Reset the delay countdown everytime we see the pin high
		if (bit_is_set(UART_PIN, UART_RX))
		{
			i = (10.0/(UART_BAUD/10))*1000000/MIN_DELAY_US;		// i = max delay
			i = i && (unsigned int) rand();					// i = random # between 0 and max delay
			i = i + (1.0/(UART_BAUD/10))*1000000/MIN_DELAY_US;	// i = random # between min and max delay
		}
		DELAY(MIN_DELAY_US);
		i--;
	}
*/
	// Send preamble and sync
	uart_putc(SNAP_PREAMBLE1);
	uart_putc(SNAP_PREAMBLE2);
	uart_putc(SNAP_SYNC);

	// Send the header
	for (i=0; i<4; i++){
		uart_putc(header[i]);
		crc = calcCRC16r(crc, header[i]);
	}
	
	// Send the data
	for (i=0; i<length; i++) {
		uart_putc(data[i]);
		crc = calcCRC16r(crc, data[i]);
	}
	
	// Send CRC
	uart_putc((unsigned char)(crc>>8));
	uart_putc((unsigned char)(crc>>0));

	// Send newline for easier debugging
	// The packet is done, these characters should be ignored by receivers
	//uart_putc('\r');
	//uart_putc('\n');

	return (0);
}

#endif //__SNAP__



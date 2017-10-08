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
// Header for the Snap library. See Snap.c for more information.
//
//----------------------------------------------------------------------------------

#ifndef __snap_h
#define __snap_h

// UART contants
//#define XTAL				16000000
//#define UART_BAUD			9600
//#define UART_BAUD_SELECT 	(XTAL/(UART_BAUD*16l)-1)
#define SNAP_MAX_LENGTH		128
#define SNAP_LENGTH_MASK	SNAP_MAX_LENGTH-1


// Snap constants
#define SNAP_PREAMBLE1		'!'				// Preamble byte 1
#define SNAP_PREAMBLE2		'#'				// Preamble byte 2
#define SNAP_SYNC			'T'				// Sync byte (start of SNAP packet)
#define SNAP_DAB			0x80			// Number of Destination Address Bytes -> 3
#define SNAP_SAB			0x20			// Number of Source Address Bytes -> 3
#define SNAP_PFB			0x00			// Protocol specific Flag Bytes -> 0, unused
#define SNAP_ACK_REQUEST	0x01			// Request ACK
#define SNAP_ACK			0x02			// Send ACK
#define SNAP_NAK			0x03			// Send NAK
#define SNAP_CMD			0x00			// Command mode bit -> 0
#define SNAP_EDM			0x00			// Error detection method -> none
#define SNAP_ACK_MASK		0x03			// Mask bits: ACK
#define SNAP_RW_REG_MASK	0x04			// Mask bits: REG RW
#define SNAP_CMD_MASK		0x80			// Mask bits: Command mode bit


// Received Snap Packet
unsigned char	SnapPacketDataLength;		// Number of bytes in SnapPacketData
unsigned char	SnapPacketDest;				// For completed packets, this will always be this node's address
unsigned char	SnapPacketSource;			// Address of the sender
unsigned char	SnapPacketHeader;				// Ack byte of the packet
unsigned int	SnapPacketCRC;
unsigned char 	SnapPacketData[SNAP_MAX_LENGTH];		// Data of the current packet


// Prototypes
void 			SnapInit(unsigned char);
void 			SnapDecodeReceived(unsigned char *);
void 			SnapReceiveChar(unsigned char);
char 			SnapSend(unsigned char, unsigned char data[], unsigned char);
void 			SnapReset(void);
char 			SnapPacketReceived(void);

#endif

 ///////////////////////////////////////////////////////////////////////////////////
 // Datei  	: 	  twi.h
 //	
	// Prozessor	: 	ATmega128
	//
	// Funktion	: 	 
	//
 // Projekt	: 	  
	//
	// Update	: 	   11.03.2004
 //
 // Compiler	: 	 
 //
 // Ersteller	:	 René Schönrock, Andreas Herrmann, 
	//
	// Bemerkung	:  keine
 ///////////////////////////////////////////////////////////////////////////////////


#ifndef __TWI_H_
 #define __TWI_H_


 ///////////////////////////////////////////////////////////////////////////////////
 //                  Diverse Definitionen und Konstanten
 ///////////////////////////////////////////////////////////////////////////////////

 /* TWSR values (not bits) */
 /* Master */
 #define TW_START		                 0x08
 #define TW_REP_START		             0x10

 /* Master Transmitter */
 #define TW_MT_SLA_ACK		            0x18
 #define TW_MT_SLA_NACK	           	0x20
 #define TW_MT_DATA_ACK	           	0x28
 #define TW_MT_DATA_NACK	          	0x30
 #define TW_MT_ARB_LOST	           	0x38
 
 /* Master Receiver */
 #define TW_MR_ARB_LOST		           0x38
 #define TW_MR_SLA_ACK		            0x40
 #define TW_MR_SLA_NACK		           0x48
 #define TW_MR_DATA_ACK		           0x50
 #define TW_MR_DATA_NACK	        	  0x58
 
 /* Slave Transmitter */
 // wird eigentlich nicht benötigt
 #define TW_ST_SLA_ACK		            0xA8
 #define TW_ST_ARB_LOST_SLA_ACK	    0xB0
 #define TW_ST_DATA_ACK		           0xB8
 #define TW_ST_DATA_NACK		          0xC0
 #define TW_ST_LAST_DATA	        	  0xC8
 
 /* Slave Receiver */
 // wird eigentlich nicht benötigt
 #define TW_SR_SLA_ACK	            	0x60
 #define TW_SR_ARB_LOST_SLA_ACK    	0x68
 #define TW_SR_GCALL_ACK		          0x70
 #define TW_SR_ARB_LOST_GCALL_ACK   0x78
 #define TW_SR_DATA_ACK	         	  0x80
 #define TW_SR_DATA_NACK	        	  0x88
 #define TW_SR_GCALL_DATA_ACK    	  0x90
 #define TW_SR_GCALL_DATA_NACK   	  0x98
 #define TW_SR_STOP	               	0xA0
 
 /* Misc */
 #define TW_NO_INFO		               0xF8
 #define TW_BUS_ERROR	             	0x00

 #define TW_STATUS_MASK	            ((1<<TWS7)|(1<<TWS6)|(1<<TWS5)|(1<<TWS4)|(1<<TWS3) )
 #define TW_STATUS	                	( TWSR & TW_STATUS_MASK )

 /*
  * R/~W bit in SLA+R/W address field.
  */
 #define TW_READ	1
 #define TW_WRITE	0


 #define TWI_START		TWCR=((1<<TWINT) | (1<<TWSTA) | (1<<TWEN))
 #define TWI_STOP	 	TWCR=((1<<TWINT) | (1<<TWSTO) | (1<<TWEN))
 #define TWI_CLI	  	TWCR=((1<<TWINT) | (1<<TWEN))
 #define TWI_NACK	  	TWCR=((1<<TWINT) | (1<<TWEN))
 #define TWI_ACK	  	TWCR=((1<<TWINT) | (1<<TWEN) | (1<<TWEA))
 
/*
 * Maximal number of iterations to wait for a device to respond for a
 * selection.  Should be large enough to allow for a pending write to
 * complete, but low enough to properly abort an infinite loop in case
 * a slave is broken or not present at all.  With 100 kHz TWI clock,
 * transfering the start condition and SLA+R/W packet takes about 10
 * µs.  The longest write period is supposed to not exceed ~ 10 ms.
 * Thus, normal operation should not require more than 100 iterations
 * to get the device to respond to a selection.
 */
 #define MAX_ITER        200

 typedef enum  {
                TWI_IDLE,
                TWI_MT,
                TWI_MR,
               }TWI_MODE;

   
 /////////////////////////////////////////////////////////////////////////////////////////////////////
 // 2 Wire Bus initialization
 // Generate Acknowledge Pulse: On
 // 2 Wire Bus Slave Address: 70h
 // General Call Recognition: Off
 // Bit Rate: 97,011 kHz  
 void init_twi(unsigned char frequency, unsigned char slave_addr, unsigned char flags);


/////////////////////////////////////////////////////////////////////////////////////////////////////
// 2 Wire Bus Start
//BOOL twi_start (void);
BOOL twi_start (void);


/////////////////////////////////////////////////////////////////////////////////////////////////////
// 2 Wire Bus Stop
// void twi_stop (void);


/////////////////////////////////////////////////////////////////////////////////////////////////////
// 2 Wire Bus 
BOOL twi_send_sla_byte (unsigned char sla);


/////////////////////////////////////////////////////////////////////////////////////////////////////
// 2 Wire Bus 
BOOL twi_send_byte (unsigned char byte_to_send);


/////////////////////////////////////////////////////////////////////////////////////////////////////
// 2 Wire Bus 
BOOL twi_receive_byte (unsigned char *byte_to_receive, unsigned char ack_flag);

#endif

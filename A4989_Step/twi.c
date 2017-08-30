 ///////////////////////////////////////////////////////////////////////////////////
 // Datei  	: 	  twi.c
 //	
 // Prozessor	: 	ATmega128
 //
 // Funktion	: 	 
 //
 // Projekt	: 	  
 //
 // Update	: 	   22.09.2004
 //
 // Compiler	: 	 
 //
 // Ersteller	:	 René Schönrock, Andreas Herrmann, 
 //
 // Bemerkung	:  keine
 ///////////////////////////////////////////////////////////////////////////////////


#include <stdio.h>
#include <avr/io.h>

#include "my_twi.h"




/////////////////////////////////////////////////////////////////////////////////////////////////////
// 2 Wire Bus initialization
// Generate Acknowledge Pulse: Off
// 2 Wire Bus Slave Address: 70h
// General Call Recognition: Off
// Bit Rate: 97,011 kHz  
void init_twi(unsigned char frequency, unsigned char slave_addr, unsigned char flags)
 {
  unsigned char sys_clk_devisor;
  sys_clk_devisor = (CLKPR & 0x0F);
  TWSR = 0x00;
  //TWBR = ( (F_CPU/sys_clk_devisor )/ ((unsigned long)frequency*1000) - 16) / 2;
  TWBR = ( F_CPU/((unsigned long)frequency*1000) - 16) / 2;
  TWAR = ((slave_addr&0x7F)<<1) | (flags&0x01);
  
  //#ifdef __ATMEGA2560__
  // TWAMR = 0;
  //#endif
  
  TWCR = _BV(TWEN);// | TWIE;
  //twi_mode = TWI_IDLE;
 }



/////////////////////////////////////////////////////////////////////////////////////////////////////
// 2 Wire Bus Start
BOOL twi_start (void)
 {
  unsigned char twst;

  TWI_START;                    /* send start condition */
  while (!(TWCR & _BV(TWINT)));  /* wait for transmission */
  twst = TW_STATUS;
  if ( twst == TW_START || twst == TW_REP_START)
   return TRUE;
  else
   return FALSE; 
 }


/////////////////////////////////////////////////////////////////////////////////////////////////////
// 2 Wire Bus 
BOOL twi_send_sla_byte (unsigned char sla)
 {
  unsigned char twst;
     // send stop condition 
  /* Note [8] */
  /* send SLA+W */
  TWDR = sla;
  TWI_CLI;                         /* clear interrupt to start transmission */
  while (!(TWCR & _BV(TWINT)));     /* wait for transmission */
  twst = TW_STATUS;
  if ( twst == TW_MT_SLA_ACK || twst == TW_MR_SLA_ACK)
   return TRUE;
  else
   return FALSE; 
 }


/////////////////////////////////////////////////////////////////////////////////////////////////////
// 2 Wire Bus 
BOOL twi_send_byte (unsigned char byte_to_send)
 {
  unsigned char twst;
  
  TWDR = byte_to_send;
  TWI_CLI;                         /* clear interrupt to start transmission */
  while (!(TWCR & _BV(TWINT)));         /* wait for transmission */
  twst = TW_STATUS;
  
  if ( twst == TW_MT_DATA_ACK)
   return TRUE;
  else
   return FALSE; 
 }


/////////////////////////////////////////////////////////////////////////////////////////////////////
// 2 Wire Bus 
BOOL twi_receive_byte (unsigned char *byte_to_receive, unsigned char ack_flag)
 {
  unsigned char twst;
  
  if (ack_flag)
   TWI_ACK;                         /* clear interrupt to start transmission with ACK*/
  else 
   TWI_NACK;                        /* clear interrupt to start transmission with NACK*/
   
  while (!(TWCR & _BV(TWINT)));          /* wait for transmission */
  twst = TW_STATUS;
  
  if (ack_flag)
   {
    if ( twst == TW_MR_DATA_ACK)
     {
      *byte_to_receive = TWDR;
      return TRUE;
     }
    else
     return FALSE;
   }
  else 
   {
    if ( twst == TW_MR_DATA_NACK)
     {
      *byte_to_receive = TWDR;
      return TRUE;
     }
    else
     return FALSE;
   }

 }



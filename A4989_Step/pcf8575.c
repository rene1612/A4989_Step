///////////////////////////////////////////////////////////////////////////////////
// Datei  	: 	  pcf8575.c
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
// Ersteller	:	 Ren� Sch�nrock, Andreas Herrmann, 
//
// Bemerkung	:  keine
///////////////////////////////////////////////////////////////////////////////////

#include <stdio.h>
#include <string.h>
#include <avr/io.h>
#include <avr/eeprom.h>
#include <util/delay.h>
#include <avr/interrupt.h>


#include "types.h"
#include "config.h"

#include "twi_master.h"
#include "pcf8575.h"
#include "pwm.h"
#include "state.h"

#include "A4989_Step.h"

ENC_PORT encoder;

/************************************************************************
 * @fn		void pcf8575_init (void)
 * @brief	
 *
 * Init-Function for Portexpander
 *
 * @note    
 *		
 */
void pcf8575_init (void)
{
	PCF8575_INT_DIR &= ~_BV(PCF8575_INT_PIN);	//Interupt-PIn auf Eingang

	MCUCR &= ~((1<<ISC11) | (1<<ISC10));	//falling edge of INT1
	MCUCR |= ((1<<ISC11) | (0<<ISC10));	//falling edge of INT1
	GICR |= _BV(INT1);						//INT1 activate
	
	encoder.word = 0x0000;
	
	pcf8575_read_from (PCF8575_TWI_DEV_ADDR, (unsigned char *)pcf8575.port);
	
	encoder.word = pcf8575.word;
	
	main_regs.full_current = (unsigned int)(encoder.val.current*CURRENT_SET_FACTOR) + (unsigned int)CURRENT_OFFSET_VAL;
	main_regs.standby_current = (unsigned int)(encoder.val.current_sb*CURRENT_SET_FACTOR) + (unsigned int)CURRENT_OFFSET_VAL;

	
}

/************************************************************************
 * @fn		pcf8575_write_to (BYTE addr, unsigned char *data)
 * @brief	Write to Portexpander
 *
 * Funktion schreibt count Datenbytes in den Portexpander
 * count muss eine gerade Zahl sein
 *
 * @note    
 *		
 */
BYTE pcf8575_write_to (BYTE addr, unsigned char *data)
{
	unsigned char buf[3];
	
	buf[0]=addr & ~(TRUE<<TWI_READ_BIT);
	
	buf[1]=data[0];
	buf[2]=data[1];
	
	TWI_Start_Transceiver_With_Data( (unsigned char *)buf, 3 );

	return TRUE;
}



/************************************************************************
 * @fn		pcf8575_read_from (BYTE addr, PBYTE destination, BYTE count)
 * @brief	Read from Portexpander
 *
 * Funktion lie�t count Datenbytes vom den Portexpander 
 * count muss eine gerade Zahl sein
 *
 * @note    
 *		
 */
BYTE pcf8575_read_from (BYTE addr, PBYTE destination)
{
	BYTE twi_addr;
  
	twi_addr = addr | (TRUE<<TWI_READ_BIT);
 
	TWI_Start_Transceiver_With_Data( &twi_addr, 3 ); 
	
	return TWI_Get_Data_From_Transceiver( (unsigned char *)destination, 2 );
}


/************************************************************************
 * @fn		ISR(INT1_vect)
 * @brief	TWI Interrupt Handler
 *
 * This function is the Interrupt Service Routine (ISR), and called when
 * the TWI interrupt is triggered; that is whenever a TWI event has occurred.
 *
 * @note    
 *		This function should not be called directly from the main application.
 */
ISR(INT1_vect)
{
	main_task_scheduler |= PROCESS_PCF8575;
	return;
}

/************************************************************************
 * @fn		unsigned char process_pcf8575(void)
 * @brief	Arbeitsloop f�r den Portexpander
 *
 *
 * @note    
 *		
 */
unsigned char process_pcf8575(void)
{
	unsigned char temp[2];

	GICR &= ~_BV(INT1);						//INT1 off
	
	//Interrupt ist aufgelaufen, zuerst Portexpander lesen
	//Portexpander lesen Eing�nge(Encoder) abfragen
	pcf8575_read_from (PCF8575_TWI_DEV_ADDR, (unsigned char *)temp);

	do {
		_delay_us(500);
		pcf8575_read_from (PCF8575_TWI_DEV_ADDR, (unsigned char *)pcf8575.port);

		if(pcf8575.port[0]!=temp[0] || pcf8575.port[1]!=temp[1])
		{
			temp[0]=pcf8575.port[0];
			temp[1]=pcf8575.port[1];
		}
		else
			break;
	}
	while (1);

	//pr�fen, welcher Encoder ge�ndert wurde
	
	if (pcf8575.val.encoder_a ^ encoder.val.current){
		encoder.val.current = pcf8575.val.encoder_a;
		
		//DAC f�r Stromeistellung updaten
		//set_DAC((unsigned int)(encoder.val.current*CURRENT_SET_FACTOR) + (unsigned int)CURRENT_OFFSET_VAL);
		main_regs.full_current = (unsigned int)(encoder.val.current*CURRENT_SET_FACTOR) + (unsigned int)CURRENT_OFFSET_VAL;
		set_StateMon(STATE_INFO_ENC_A);
	}
		
	if (pcf8575.val.encoder_b ^ encoder.val.current_sb){
		encoder.val.current_sb = pcf8575.val.encoder_b;
		
		if (encoder.val.current_sb <= encoder.val.current) {
			main_regs.standby_current = (unsigned int)(encoder.val.current_sb*CURRENT_SET_FACTOR) + (unsigned int)CURRENT_OFFSET_VAL;

			set_StateMon(STATE_INFO_ENC_B);
		}
	}

	if (pcf8575.val.encoder_c ^ encoder.val.microstep_decay){
		encoder.val.microstep_decay = pcf8575.val.encoder_c;
		
		set_StateMon(STATE_INFO_ENC_C);
	}

	//testhalber mit Encoder Fehler zur�cksetzen	
	if ( main_regs.sys_state == SYS_ERROR) {
		set_sys_state(SYS_OK);
	}

	GICR |= _BV(INT1);						//INT1 activate again
	return TRUE;
}


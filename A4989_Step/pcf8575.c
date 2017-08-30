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
// Ersteller	:	 René Schönrock, Andreas Herrmann, 
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
	PCF8575_INT_DIR &= ~_BV(PCF8575_INT);	//Interupt-PIn auf Eingang

	MCUCR &= ~((1<<ISC11) | (1<<ISC10));	//falling edge of INT1
	MCUCR |= ((1<<ISC11) | (0<<ISC10));	//falling edge of INT1
	GICR |= _BV(INT1);						//INT1 activate
	
	encoder.word = 0x0000;
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
 * Funktion ließt count Datenbytes vom den Portexpander 
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
 * @brief	Arbeitsloop für den Portexpander
 *
 *
 * @note    
 *		
 */
unsigned char process_pcf8575(void)
{
	uint8_t temp_port;
	unsigned int pcf8575_io_mask;
	
	//Interrupt ist aufgelaufen, zuerst Portexpander lesen
	//Portexpander lesen Eingänge(Encoder) abfragen
	pcf8575_read_from (PCF8575_TWI_DEV_ADDR, (unsigned char *)pcf8575.port);

	//prüfen, welcher Encoder geändert wurde
	
	if (pcf8575.val.encoder_a ^ encoder.val.current){
		encoder.val.current = pcf8575.val.encoder_a;
		
		//DAC für Stromeistellung updaten
		set_DAC((unsigned int)(encoder.val.current*CURRENT_SET_FACTOR) + (unsigned int)CURRENT_OFFSET_VAL);
	
		//Initialisierung des IO-Expanders Eingänge(Encoder), Ausgänge(LED)
		pcf8575_io_mask = ((pcf8575.val.encoder_a << 12) | main_regs.encoder_inp_mask);
		pcf8575_write_to (PCF8575_TWI_DEV_ADDR, (unsigned char *)&pcf8575_io_mask);
		
	}
		
	if (pcf8575.val.encoder_b ^ encoder.val.microstep_decay){
		encoder.val.microstep_decay = pcf8575.val.encoder_b;
		
		//Portpins setzen
		temp_port = PINB & 0x0F;
		temp_port |= ((encoder.val.microstep_decay & 0x03)<<A4989_MS1);
		temp_port |= ((encoder.val.microstep_decay & 0x08)<<(A4989_PFD2-3));
		temp_port |= ((encoder.val.microstep_decay & 0x04)<<(A4989_PFD1-2));
		PORTB = temp_port;
	}

	if (pcf8575.val.encoder_c ^ encoder.val.sync_rect_mode){
		encoder.val.sync_rect_mode = pcf8575.val.encoder_c;
		
		//Portpins setzen
		temp_port = PINA & 0xFE;
		temp_port |= (encoder.val.sync_rect_mode & 0x01);
		PORTA = temp_port;
	}
	
	return TRUE;
}


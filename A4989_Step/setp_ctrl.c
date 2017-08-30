/**
 * @file	step_ctrl.c
 * @brief	Firmware für den ATtiny25/45-Controller im VAGABUND-Beamer.
 *
 * @author	René Schönrock
 *
 *
 * @date	05.11.2007
 * @see		Datenblatt des  von Atmel
 * 
 *
 * @note	
 *
 * @todo	Berechnung und Nutzung der temperaturkompensierten Strahlerleistung
 */

#include <stdio.h>
#include <avr/io.h>

#include <avr/eeprom.h>
#include <avr/interrupt.h>

#include <avr/wdt.h>

#include <util/delay.h>

#include "step_ctrl.h"

/**
 * @fn		void init_Sys(void)
 * @brief	Systeminitialisierung
 *
 * Ausführliche Beschreibung
 *
 * @param   keine 
 * @return	keine
 */
void init_Sys(void)
 {
  PORTA = 0x00;
  DDRA = (STEP_CLK | STEP_CW_CCW | STEP_NRESET | STEP_ENABLE | STATE_LED);

  PORTB = 0x00;
  DDRB = 0;

  //if (eeprom_read_byte(&ee_reg.ctrl) & (1<<CTRL_EE_DEFAULTS))
  // {
    //Registerwerte aus dem EEProm restaurieren
    //eeprom_read_block ((void*)&reg, (const void*)&ee_reg, sizeof(REG));
  // }


  //wdt_enable(WDTO_60MS);

  //Interrupts anwerfen
  sei();
 }


/**
 * @fn		int main (void)
 * @brief	Hauptprogramm-Schleife
 *
 * Ausführliche Beschreibung
 *
 * @param   keine 
 * @return	Type int (never returns)
 */
int main (void)
 {
  unsigned char iic_data, tmp_ctrl_reg=0xFF;


	//Systeminitialisierug
	init_Sys();



	PORTA |=STEP_NRESET;


	//Endlosschleife 
	while(1)
	{
    	//wdt_reset();
		if (!(PINA & SWITCH_DOWN))
		{
			PORTA &= ~STEP_CW_CCW;
		}
		else
		{
			PORTA |= STEP_CW_CCW;
		}

    	//wdt_reset();
		if (!(PINA & SWITCH_UP))
		{
			PORTA &= ~STATE_LED;


			PORTA |= STEP_ENABLE;

		}
		else
		{
			PORTA |= STATE_LED;

			PORTA &= ~STEP_ENABLE;

		}


		_delay_ms(1);
		PORTA |= STEP_CLK;
		_delay_ms(1);
		PORTA &= ~STEP_CLK;
	}

  return 0;
 }


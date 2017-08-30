/**
 * @file	state.c
 * @author	Ren� Sch�nrock
 * @brief	Statusmonitor mittels LED.
 *
 *
 * @date	23.04.2007
 * @see		Datenblatt des ATtiny25/45 von Atmel
 *	
 * �ber die Status-LED (STATE) wird mittels Blinkkodes der aktuelle Systemstatus angezeigt.
 * Dazu kann im Register MONITOR_MODE_REG ein entspr. Modus ausgew�hlt werden.
 * 
 * verwendete Resourcen:
 *	- Timer 1
 *	- INT-Timer 1
 *	- Port PB5
 */

#include <stdio.h>
#include <avr/io.h>

#include <avr/interrupt.h>
 
#include "state.h"

#include "A4989_Step.h"


/**
 * @var		msec_counter
 * @brief	Z�hlervariable f�r den State-Timer
 */
unsigned int msec_counter;


/**
 * @var		state_mask
 * @brief	
 *	
 */
unsigned char state_mask;



/**
 * @fn		ISR(TIM2_OVF_vect)
 * @brief	Timer-ISR
 *
 * Ausf�hrliche Beschreibung
 *
 * @param	keine	
 *
 * @return	keine	
 *
 * @note	
 *
 */
ISR(TIMER2_OVF_vect)
{
	//Status mit LED anzeigen?
	if (main_regs.ctrl & (1<<REG_CTRL_SET_LED))
	{
		if (++msec_counter >= STATE_TIMER_RESOLUTION )
		{
			msec_counter = 0;

			state_mask<<=1;
			if (!state_mask)
				state_mask = 1;
				
			main_task_scheduler |= PROCESS_MONITOR_STATE;

		}
	}
}



/**
 * @fn		void init_StateMon(void)
 * @brief	Initialisierung des Statusmonitors
 *
 * Ausf�hrliche Beschreibung
 *
 * @param	keine	
 *
 * @return	keine
 *
 */
 void init_StateMon(void)
{
	TCCR2 = 0x0A;
  
	//GTCCR = 0;
	TIMSK = (1<<TOIE2);
  
	msec_counter = 0;
	state_mask=1;

	main_regs.monitor_led_state = STATE_OFF;
}



/**
 * @fn		unsigned char set_State(unsigned char new_state)
 * @brief	neuen Systemstatus setzen
 *
 * Ausf�hrliche Beschreibung
 *
 * @param	new_state
 *
 * @return	TRUE | FALSE
 *
 * @note	
 *
 */
 unsigned char set_State(unsigned char new_state)
{
	if( (new_state & 0x0F) <= (main_regs.monitor_led_state & 0x0F) )
	{
		main_regs.monitor_led_state = new_state;
		return 1;
	}
	return 0;
}


/**
 * @fn		unsigned char process_State(void)
 * @brief	neuen Systemstatus setzen
 *
 * Ausf�hrliche Beschreibung
 *
 * @param	none
 *
 * @return	TRUE | FALSE
 *
 * @note	
 *
 */
 unsigned char process_State(void)
{
		
	//do something
	
	return 1;
}


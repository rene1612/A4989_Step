/**
 * @file	state.c
 * @author	René Schönrock
 * @brief	Statusmonitor mittels LED.
 *
 *
 * @date	23.04.2007
 * @see		Datenblatt des ATtiny25/45 von Atmel
 *	
 * Über die Status-LED (STATE) wird mittels Blinkkodes der aktuelle Systemstatus angezeigt.
 * Dazu kann im Register MONITOR_MODE_REG ein entspr. Modus ausgewählt werden.
 * 
 * verwendete Resourcen:
 *	- Timer 1
 *	- INT-Timer 1
 *	- Port PB5
 */

#include <stdio.h>
#include <avr/io.h>

#include <avr/interrupt.h>
 #include "config.h"
#include "state.h"

#include "A4989_Step.h"
#include "pcf8575.h"


/**
 * @var		msec_counter
 * @brief	Zählervariable für den State-Timer
 */
unsigned int msec_counter;
unsigned int msec_counter_ticks;


/**
 * @var		state_mask
 * @brief	
 *	
 */
unsigned char state_mask;

/**
 * @var		state_mask
 * @brief	
 *	
 */
unsigned char view_timer;


/**
 * @var		old_state

 * @brief	
 *	
 */
unsigned char old_state;
;

/**
 * @fn		ISR(TIM2_OVF_vect)
 * @brief	Timer-ISR
 *
 * Ausführliche Beschreibung
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
			msec_counter_ticks++;

			state_mask<<=1;
			if (!state_mask)
				state_mask = 1;
				
			main_task_scheduler |= PROCESS_MONITOR_STATE;

		}
	}
}

/**
 * @fn		unsigned char set_StateMon(unsigned char new_state)
 * @brief	neuen Systemstatus setzen
 *
 * Ausführliche Beschreibung
 *
 * @param	new_state
 *
 * @return	TRUE | FALSE
 *
 * @note	
 *
 */
 unsigned char set_LED(unsigned char led)
{
	unsigned int pcf8575_io_mask;

	pcf8575_io_mask = ((~led << 12) | (main_regs.encoder_inp_mask & ENCODER_ALL_MASK));
	pcf8575_write_to (PCF8575_TWI_DEV_ADDR, (unsigned char *)&pcf8575_io_mask);
	return 1;
	//}
	//return 0;
}



/**
 * @fn		void init_StateMon(void)
 * @brief	Initialisierung des Statusmonitors
 *
 * Ausführliche Beschreibung
 *
 * @param	keine	
 *
 * @return	keine
 *
 */
 void init_StateMon(void)
{
	//TCCR2 = ((0<<COM21) | (0<<COM20) | (0<<WGM21) | (0<<WGM20) | (1<<CS22) | (1<<CS21) | (0<<CS20));
  
	//GTCCR = 0;
	TIMSK = (1<<TOIE2);
  
	msec_counter = 0;
	msec_counter_ticks = 0;
	state_mask=1;
	view_timer = 0;

	main_regs.monitor_led_state = STATE_OFF;
	old_state = STATE_OFF;
	
	set_LED(0);
}



/**
 * @fn		unsigned char set_StateMon(unsigned char new_state)
 * @brief	neuen Systemstatus setzen
 *
 * Ausführliche Beschreibung
 *
 * @param	new_state
 *
 * @return	TRUE | FALSE
 *
 * @note	
 *
 */
 unsigned char set_StateMon(unsigned char new_state)
{
	//if( (new_state & 0x0F) <= (main_regs.monitor_led_state & 0x0F) )
	//{
		old_state = main_regs.monitor_led_state;
		main_regs.monitor_led_state = new_state;
		
		if (new_state) {
			
			view_timer = 0;
			TCCR2 = ((0<<COM21) | (0<<COM20) | (0<<WGM21) | (0<<WGM20) | (1<<CS22) | (1<<CS21) | (0<<CS20));
			
		}
		return 1;
	//}
	//return 0;
}


/**
 * @fn		unsigned char process_State(void)
 * @brief	neuen Systemstatus setzen
 *
 * Ausführliche Beschreibung
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
	switch (main_regs.monitor_led_state)
	{
		case STATE_ON:
			set_LED(state_mask);
			return 0;

		case STATE_INFO_ENC_A:
			if (!view_timer)
				set_LED(pcf8575.val.encoder_a);
			if (++view_timer < 16)
				return 1;
			
			set_StateMon(old_state);
			break;
			
		case STATE_INFO_ENC_B:
			if (!view_timer)
				set_LED(pcf8575.val.encoder_b);
			if (++view_timer < 16)
				return 1;
			
			set_StateMon(old_state);
			break;
			
		case STATE_INFO_ENC_C:
			if (!view_timer)
				set_LED(pcf8575.val.encoder_c);
			if (++view_timer < 16)
				return 1;
			
			set_StateMon(old_state);
			break;

		case STATE_ERR_HEADSINK_TEMP:
		case STATE_ERR_VBUS_VOLTAGE_UT:
		case STATE_ERR_VBUS_VOLTAGE_LT:
			if (msec_counter_ticks&0x01)
				set_LED(main_regs.monitor_led_state&0x0F);
			else
				set_LED(0x00);
			
			break;
			
		case STATE_OFF:
			set_LED(0x00);
			TCCR2 = 0;
			break;
	}
	
	return 1;
}


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
uint8_t state_msec_counter;
uint8_t state_msec_counter_ticks;

/**
 * @var		state_mask
 * @brief	
 *	
 */
//unsigned char state_mask;

/**
 * @var		state_mask
 * @brief	
 *	
 */
unsigned char view_timer;

unsigned char led_display_buffer[2][8] ={{0x01,0x02,0x08,0x04},{0x04,0x08,0x02,0x01}};


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
	state_msec_counter = 0;
	state_msec_counter_ticks = 0;

	TIMSK |= (1<<TOIE2);
	
	//state_mask=1;
	view_timer = 0;

	main_regs.monitor_led_state = STATE_OFF;
	
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
	main_regs.monitor_led_state = new_state;
	
	if (new_state) {
		
		view_timer = 0;
		
	}
	return 1;
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
		case STATE_ON_FC:
			if (STEP_DIR_INPUT)
				set_LED(led_display_buffer[0][state_msec_counter_ticks&0x03]);
			else
				set_LED(led_display_buffer[1][state_msec_counter_ticks&0x03]);
			return 1;

		case STATE_ON_SBC:
//			if (view_timer&0x01) {
				if (STEP_DIR_INPUT)
					set_LED(led_display_buffer[0][(state_msec_counter_ticks&0x06)>>1]);
				else
					set_LED(led_display_buffer[1][(state_msec_counter_ticks&0x06)>>1]);
//			}
//			view_timer++;
			return 1;

		case STATE_INFO_ENC_A:
			if (!view_timer)
				set_LED(pcf8575.val.encoder_a);
			if (++view_timer < 16)
				return 1;
			
			set_StateMon(0x00);
			break;
			
		case STATE_INFO_ENC_B:
			if (!view_timer)
				set_LED(pcf8575.val.encoder_b);
			if (++view_timer < 16)
				return 1;
			
			set_StateMon(0x00);
			break;
			
		case STATE_INFO_ENC_C:
			if (!view_timer)
				set_LED(pcf8575.val.encoder_c);
			if (++view_timer < 16)
				return 1;
			
			set_StateMon(0x00);
			break;

		case STATE_ERR_HEADSINK_TEMP:
		case STATE_ERR_VBUS_VOLTAGE_UT:
		case STATE_ERR_VBUS_VOLTAGE_LT:
			if (state_msec_counter_ticks&0x01)
				set_LED(main_regs.monitor_led_state&0x0F);
			else
				set_LED(0x00);
			
			break;
			
		case STATE_OFF:
			set_LED(0x00);
			//TCCR2 = 0;
			break;
	}
	
	return 1;
}

/**
 * @fn		ISR(TIM2_OVF_vect)
 * @brief	Timer-ISR kommt alle 8,8 ms
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
	if (main_regs.monitor_led_state) {
		if (++state_msec_counter >= STATE_TIMER_RESOLUTION )
		{
			state_msec_counter = 0;
			state_msec_counter_ticks++;
			main_task_scheduler |= PROCESS_MONITOR_STATE;
		}
	}
}

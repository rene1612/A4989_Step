/************************************************************************
 * @file	A4989_Step.c
 * @brief	Firmware f�r den Controller 
 *
 * @author	Ren� Sch�nrock
 *
 *
 * @date	15.10.2010
 * @version	1.0
 * @bug		none.
 * @warning none.
 * @see		Datenblatt des  von Atmel
 *
 * @note	
 *
 * @todo
 *	
 * Details.
 */

 #include <stdio.h>
 #include <avr/io.h>
 #include <string.h>
 #include <avr/eeprom.h>
 #include <avr/interrupt.h>
 #include <util/delay.h>
 #include <avr/wdt.h>

 #include "types.h"
 #include "config.h"
// #include "prot.h"
 #include "uart.h"
 #include "twi_master.h"
 #include "pwm.h"
 #include "adc.h"
 #include "state.h"
 #include "pcf8575.h"


 #ifdef __SNAP__
	#include "snap.h"
 #endif


 #include "A4989_Step.h"


/**
 * @fn		void (*jump_to_app)(void) = __APPLICATION_START_ADDR__
 * @brief	Einsprungspunkt f�r die Applikation
 */
 void (*jump_to_app)(void) = __APPLICATION_START_ADDR__;

 /**
 * @fn		void (*jump_to_bootloader)(void) = __APPLICATION_START_ADDR__
 * @brief	Einsprungspunkt f�r den Bootloader
 */
void (*jump_to_bootloader)(void) = __BOOTLOADER_START_ADDR__;



/**
 * @var		
 * @brief	Registersatz im EEProm
 * @see		REG
 * @see		reg
 *	
 */
 unsigned char dev_addr EEMEM = DEFAULT_DEV_ADDR;
 unsigned char dev_addr_backup EEMEM = DEFAULT_DEV_ADDR;


/**
 * @var		main_ee_regs
 * @brief	Registersatz im EEProm
 * @see		REG
 * @see		reg
 *	
 */
MAIN_REGS main_ee_regs EEMEM =
{
	//!<RW CTRL Ein-/Ausschalten usw.  (1 BYTE )
	(0<<REG_CTRL_BOOT) | \
	(1<<REG_CTRL_AUTO_CURRENT) | \
	(1<<REG_CTRL_EE_DEFAULTS),

	SYS_OK,
	STATE_OFF,
	
	0x0FFF,
	
	DEFAULT_MAX_HEATSINK_TEMP,
	DEFAULT_UPPER_BV_THRESHOLD,
	DEFAULT_LOWER_BV_THRESHOLD,

	DEFAULT_STANDBY_TIMEOUT,
		
	0x0000,
	0x0000,
	0x00,
	0x0000,
	0,
	0,
	
	APP_UART_BAUD_RATE,
	
	//ab rel. 0x10 steht die Ger�tesignatur und die Softwareversion
	__DEV_SIGNATURE__,
	__SW_RELEASE__,
	__SW_RELEASE_DATE__,

};


/**
 * @var		main_regs
 * @brief	Registersatz im Ram (Arbeitsregister)
 * @see		MAIN_REGS
 * @see		main_ee_regs
 *
 */
MAIN_REGS main_regs;


SWITCH_PORT pcf8575;

uint8_t main_task_scheduler;
//uint8_t main_timer=0;
//uint8_t timer0_ov_ticks=0;
//signed char cur_heatsink_temperature;
//unsigned int cur_vbus_volage;
uint8_t standby_timer;
uint8_t standby_timout;

/************************************************************************
 * @fn		ISR(INT0_vect)
 * @brief	TWI Interrupt Handler
 *
 * This function is the Interrupt Service Routine (ISR), and called when
 * the STEP_ENABLE interrupt is triggered;
 *
 * @note    
 *		This function should not be called directly from the main application.
 */
ISR(INT0_vect)
{
	main_task_scheduler |= PROCESS_STEP_ENABLE;
	return;
}


/************************************************************************
 * @fn		ISR(INT2_vect)
 * @brief	Step_Dir Interrupt Handler
 *
 * This function is the Interrupt Service Routine (ISR), and called when
 * the STEP_DIR interrupt is triggered;
 *
 * @note    
 *		This function should not be called directly from the main application.
 */
ISR(INT2_vect)
{
	main_regs.step_dir = STEP_DIR_INPUT;
	return;
}


/************************************************************************
 * @fn		ISR(SIG_OVERFLOW1)
 * @brief	ISR f�r step-pulse erkennung (automatische stromabsenkung)
 *
 * Ausf�hrliche Beschreibung
 *
 * @param   keine 
 * @return	keine
 */
ISR(TIMER0_OVF_vect)
{
	if (main_regs.sys_state == SYS_ACTIVE_SBC) {
		OCR0 = 0xFF;
		standby_timer=TCNT0;
		TIMSK |= _BV(TOIE1);
		set_sys_state (SYS_ACTIVE_FC);
	}
}


/************************************************************************
 * @fn		ISR(SIG_OVERFLOW1)
 * @brief	ISR f�r step-pulse erkennung (automatische stromabsenkung)
 *
 * Ausf�hrliche Beschreibung
 *
 * @param   keine 
 * @return	keine
 */
ISR(TIMER1_OVF_vect)
{
	if (TCNT0 == standby_timer) {
		if (++standby_timout > main_regs.standby_timeout) {
			TIMSK &= ~_BV(TOIE1);
			set_sys_state (SYS_ACTIVE_SBC);
		}
	}
	else {
		standby_timer=TCNT0;
		standby_timout=0;
	}
}


/************************************************************************
 * @fn		void init_Sys(void)
 * @brief	Systeminitialisierung
 *
 * Ausf�hrliche Beschreibung
 *
 * @param   keine 
 * @return	keine
 */
void init_Sys(void)
{
	unsigned int ui_baudrate;

#ifdef __USE_WDT__
	//Watchdogtimer aktivieren  	WDTO_250MS
	//wdt_enable(WDTO_60MS);
	wdt_enable(WDTO_500MS);
	wdt_reset();
#else
	wdt_disable();
#endif

#if defined(__AVR_ATmega32__) || defined (__AVR_ATmega16__)
	//TCCR0 = ((0<<COM01) | (0<<COM00) | (0<<WGM01) | (0<<WGM00) | (1<<CS02) | (1<<CS01) | (0<<CS00));
	TIMSK = (_BV(TOIE0));
	
	STEP_ENA_INT_DIR &= ~_BV(STEP_ENA_INPUT_PIN);	//Interupt-PIn auf Eingang
	MCUCR &= ~((1<<ISC01) | (1<<ISC00));	//falling edge of INT0
	MCUCR |= ((0<<ISC01) | (1<<ISC00));	//falling edge of INT0
	GICR |= _BV(INT0) | _BV(INT2);						//INT0 & INT2 activate

#else
	#error Wrong AVR!
#endif

	DDRA=(_BV(A4989_EN) | _BV(A4989_SR));
	DDRB=(_BV(A4989_PFD2) | _BV(A4989_PFD1) | _BV(A4989_MS2) | _BV(A4989_MS1) | _BV(A4989_RESET));
	A4989_DISABLE;
	A4989_SR_ENABLE;	//Synchronus Rectification always on		
	A4989_RESET_ENABLE;
		
	if (eeprom_read_byte(&main_ee_regs.ctrl) & (1<<REG_CTRL_EE_DEFAULTS)) {
		//Registerwerte aus dem EEProm restaurieren
		eeprom_read_block ((void*)&main_regs, (const void*)&main_ee_regs, sizeof(MAIN_REGS));
	}
	
	
	main_regs.step_dir = STEP_DIR_INPUT;
	
	/* Einstellen der Baudrate */
	//uart_init( UART_BAUD_SELECT_DOUBLE_SPEED(APP_UART_BAUD_RATE, F_CPU) );
	if(main_regs.baudrate>MAX_BAUDRATE){
		ui_baudrate=UART_BAUD_SELECT(APP_UART_BAUD_RATE, F_CPU);
		main_regs.baudrate=APP_UART_BAUD_RATE;
	}
	
	if(main_regs.baudrate<MIN_BAUDRATE){
		ui_baudrate=UART_BAUD_SELECT(APP_UART_BAUD_RATE, F_CPU);
		main_regs.baudrate=APP_UART_BAUD_RATE;
	}

	else{
		if (main_regs.baudrate>=57600)
			ui_baudrate=UART_BAUD_SELECT_DOUBLE_SPEED(main_regs.baudrate, F_CPU);
		else
			ui_baudrate=UART_BAUD_SELECT(main_regs.baudrate, F_CPU);
	}

	//Interrupts anwerfen
	sei();

	init_ADC();
	
	uart_init( ui_baudrate );

	TWI_Master_Initialise();
	
	pcf8575_init ();
	pcf8575.word = 0xFFFF;
	
	init_DAC();
	
	init_StateMon();

#ifdef __SNAP__
	unsigned char nodeAddress1=eeprom_read_byte((uint8_t *)0x00);
	unsigned char nodeAddress2=eeprom_read_byte((uint8_t *)0x01);
	if(nodeAddress1!=nodeAddress2 || nodeAddress1==0 || nodeAddress1==0xFF)
		nodeAddress1=BACKUP_DEV_ADDR;
		
	SnapInit(nodeAddress1);
#endif

	main_task_scheduler = 0;

}


/************************************************************************
 * @fn		int set_sys_state (void)
 * @brief	
 *
 * Ausf�hrliche Beschreibung
 *
 * @param   enum SYS_STATE sys_state
 * @return	
 */
void set_sys_state (enum SYS_STATE sys_state) {
	uint8_t temp_port;
	
	switch (sys_state) {

		case SYS_OK:
		case SYS_ACTIVE_STOP:
			A4989_RESET_ENABLE;
			A4989_DISABLE;
			//DAC f�r Stromeistellung updaten
			set_DAC((unsigned int)0x0000);
			main_regs.sys_state = SYS_OK;
			set_StateMon(STATE_OFF);
			break;

		case SYS_ACTIVE_START:
			//Portpins setzen
			temp_port = PINB & 0x0F;
			temp_port |= ((encoder.val.microstep_decay & 0x03)<<A4989_MS1);
			temp_port |= ((encoder.val.microstep_decay & 0x08)<<(A4989_PFD2-3));
			temp_port |= ((encoder.val.microstep_decay & 0x04)<<(A4989_PFD1-2));
			PORTB = temp_port;
			A4989_RESET_DISABLE;
			A4989_ENABLE;
		
		case SYS_ACTIVE_SBC:
			set_DAC(main_regs.standby_current);
			main_regs.sys_state = SYS_ACTIVE_SBC;
			set_StateMon(STATE_ON_SBC);

			if ( main_regs.ctrl & _BV(REG_CTRL_AUTO_CURRENT) ) {
				TCNT0 = 0;
				OCR0 = 1;
				standby_timout=0;
				TCCR0 = ((0<<COM01) | (0<<COM00) | (1<<WGM01) | (0<<WGM00) | (1<<CS02) | (1<<CS01) | (0<<CS00));
				break;
			}

		case SYS_ACTIVE_FC:
			set_DAC(main_regs.full_current);
			main_regs.sys_state = SYS_ACTIVE_FC;
			set_StateMon(STATE_ON_FC);
			break;
		
		case SYS_ERROR:
			A4989_RESET_ENABLE;
			set_DAC((unsigned int)0x0000);
			main_regs.sys_state = SYS_ERROR;
			break;
	}
}

/************************************************************************
 * @fn		int main (void)
 * @brief	Hauptprogramm-Schleife
 *
 * Ausf�hrliche Beschreibung
 *
 * @param   keine 
 * @return	Type int (never returns)
 */
int main (void) {
	
	static unsigned char main_tmp_ctrl_reg=0x00;
	
	//Systeminitialisierug
	init_Sys();


	//Endlosschleife 
	while(1) {

#ifdef __USE_WDT__
		//Watchdog zur�cksetzen
		wdt_reset();
#endif


#ifdef __SNAP__
		if(SnapPacketReceived()){
			SnapDecodeReceived((unsigned char *)& main_regs.ctrl);
		}
#endif

		//mal schauen, ob sich im CTRL-Register was ge�ndert hat
		if( main_regs.ctrl^main_tmp_ctrl_reg ) {

			if( main_regs.ctrl & (1<<REG_CTRL_WR_EEREGS) ) { // EEProm-Register schreiben ?
			 
				//extrem Wichtig!!! sonst eventuell Loop mit massiven EEProm-Schreibaktionen
				main_regs.ctrl &= ~((1<<REG_CTRL_WR_EEREGS) | (1<<REG_CTRL_RESET)); 

				//Registerwerte in dem EEProm schreiben
				eeprom_update_block ((const void*)&main_regs, (void*)&main_ee_regs, sizeof(MAIN_REGS) ); //
			}

			if( main_regs.ctrl & (1<<REG_CTRL_RESET) ) { // Reset des Controllers ausl�sen ?
				_delay_ms(15);
				jump_to_app();
   			}

			if( main_regs.ctrl & (1<<REG_CTRL_BOOT) ) {
				main_regs.ctrl &= ~(1<<REG_CTRL_BOOT);
				
				wdt_enable(WDTO_15MS);
				while(1){};

				//jump_to_bootloader();
			}

			if( main_regs.ctrl & (1<<REG_CTRL_ERROR_RESET) ) {
				main_regs.ctrl &= ~(1<<REG_CTRL_ERROR_RESET);

				set_sys_state (SYS_OK);
			}

			if( main_regs.ctrl & (1<<REG_CTRL_ACTIVATE) ) {
				main_regs.ctrl &= ~(1<<REG_CTRL_ACTIVATE);

				set_sys_state (SYS_ACTIVE_START);
			}
			
			if( main_regs.ctrl & (1<<REG_CTRL_DEACTIVATE) ) {
				main_regs.ctrl &= ~(1<<REG_CTRL_DEACTIVATE);

				set_sys_state (SYS_ACTIVE_STOP);
			}
			   
			main_tmp_ctrl_reg = main_regs.ctrl;
		}
		
		
		 main_regs.cur_heatsink_temperature = get_Temperature(SENSOR_TH_HEADSINK, DEFAULT_RTH_VOR);
		 if (main_regs.cur_heatsink_temperature > main_regs.max_heatsink_temp && main_regs.sys_state != SYS_ERROR ) {
			set_sys_state (SYS_ERROR);
			set_StateMon(STATE_ERR_HEADSINK_TEMP);
		 }
		 
		 main_regs.cur_vbus_volage = get_VBusVoltage(SENSOR_POWER_VOLTAGE, DEFAULT_R1, DEFAULT_R2, DEFAULT_VREF);
		 if (main_regs.cur_vbus_volage)
		 {
			 if (main_regs.cur_vbus_volage < main_regs.lower_bv_threshold && main_regs.sys_state != SYS_ERROR ) {
				set_sys_state (SYS_ERROR);
				set_StateMon(STATE_ERR_VBUS_VOLTAGE_LT);
			 }

			 if (main_regs.cur_vbus_volage > main_regs.upper_bv_threshold && main_regs.sys_state != SYS_ERROR ) {
				set_sys_state (SYS_ERROR);
				set_StateMon(STATE_ERR_VBUS_VOLTAGE_UT);
			 }
		 }
		 
		 
		 if (main_task_scheduler & PROCESS_PCF8575){
			 if (process_pcf8575())
				main_task_scheduler &= ~PROCESS_PCF8575;
		 }
		 
		 if (main_task_scheduler & PROCESS_MONITOR_STATE){
			 if (process_State())
				main_task_scheduler &= ~PROCESS_MONITOR_STATE;
		 }

		 if (main_task_scheduler & PROCESS_STEP_ENABLE){
			 
			if (main_regs.sys_state != SYS_ERROR) {
				 if (!STEP_ENA_INPUT)
				 {
					set_sys_state (SYS_ACTIVE_START);
					//set_StateMon(STATE_ON);
				 }else
				 {
					set_sys_state (SYS_ACTIVE_STOP);
					//set_StateMon(STATE_OFF);
				 }
			}
			 
			main_task_scheduler &= ~PROCESS_STEP_ENABLE;
		 }

	};

	return 0;
}
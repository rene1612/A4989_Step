/************************************************************************
 * @file	A4989_Step.c
 * @brief	Firmware für den Controller 
 *
 * @author	René Schönrock
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
 * @brief	Einsprungspunkt für die Applikation
 */
void (*jump_to_app)(void) = __APPLICATION_START_ADDR__;



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
	(1<<REG_CTRL_EE_DEFAULTS),

	0,
	
	0x0FFF,
	
	APP_UART_BAUD_RATE,
	
	//ab rel. 0x10 steht die Gerätesignatur und die Softwareversion
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

/************************************************************************
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
//	TIMSK1 = ((1<<TOIE1));//(1<<TICIE1) | (1<<TOIE1)
	TCCR1A = ((0<<WGM11) | (1<<WGM10) | (1<<COM1A1) | (0<<COM1A0) | (1<<COM1B1) | (0<<COM1B0));
	TCCR1B = ((0<<ICNC1) | (0<<ICES1) | (0<<WGM13) | (1<<WGM12) | (0<<CS12) | (1<<CS11) | (1<<CS10));//clkI/O/64 (From prescaler)
	OCR1A	= 0x0010;
	OCR1B	= 0x0020;
	//ICR1 = 0x0010;

	//TIMSK2 = ((1<<TOIE2));//(1<<TICIE1) | (1<<TOIE1)
	//TCCR2A = ((1<<WGM21) | (1<<WGM20));
	//TCCR2B = ((0<<FOC2A) | (0<<FOC2B) | (1<<WGM22) | (0<<CS22) | (0<<CS21) | (1<<CS20));//clkI/O/8 (From prescaler)
	//OCR2A	= 0x40;
#else
	
#endif


	//PWM_DDR |= (PWM_MASK(PWM_CH_0) | PWM_MASK(PWM_CH_1) | PWM_MASK(PWM_CH_2) | PWM_MASK(PWM_CH_3) | PWM_MASK(PWM_CH_4) | PWM_MASK(PWM_CH_5));
	//EPWM_DDR |=	(EPWM_MASK(PWM_CH_6) | EPWM_MASK(PWM_CH_7));
	
	DDRA=(_BV(A4989_EN) | _BV(A4989_SR));
	DDRB=(_BV(A4989_PFD2) | _BV(A4989_PFD1) | _BV(A4989_MS2) | _BV(A4989_MS1) | _BV(A4989_RESET));
	//DDRD=()		
			
	if (eeprom_read_byte(&main_ee_regs.ctrl) & (1<<REG_CTRL_EE_DEFAULTS)) {
		//Registerwerte aus dem EEProm restaurieren
		eeprom_read_block ((void*)&main_regs, (const void*)&main_ee_regs, sizeof(MAIN_REGS));
	}
	
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
	
	uart_init( ui_baudrate );


	TWI_Master_Initialise();
	
	pcf8575_init ();
	pcf8575.word = 0xFFFF;
	
	init_DAC(3);
	
	init_ADC();
	
	init_StateMon();

#ifdef __SNAP__
	unsigned char nodeAddress1=eeprom_read_byte((uint8_t *)0x00);
	unsigned char nodeAddress2=eeprom_read_byte((uint8_t *)0x01);
	if(nodeAddress1!=nodeAddress2 || nodeAddress1==0 || nodeAddress1==0xFF)
		nodeAddress1=BACKUP_DEV_ADDR;
		
	SnapInit(nodeAddress1);
#endif

	main_task_scheduler = PROCESS_PCF8575;
	
	//Interrupts anwerfen
	sei();

}


/************************************************************************
 * @fn		int main (void)
 * @brief	Hauptprogramm-Schleife
 *
 * Ausführliche Beschreibung
 *
 * @param   keine 
 * @return	Type int (never returns)
 */
int main (void) {
	
	static unsigned char main_tmp_ctrl_reg=0x00;
	signed char cur_heatsink_temperature;
	unsigned int cur_vbus_volage;
	unsigned int pcf8575_io_mask;
	
	//Systeminitialisierug
	init_Sys();
	
	//Initialisierung des IO-Expanders Eingänge(Encoder), Ausgänge(LED)
	pcf8575_io_mask = (0xF000 | main_regs.encoder_inp_mask);
	pcf8575_write_to (PCF8575_TWI_DEV_ADDR, (unsigned char *)&pcf8575_io_mask);

	//Endlosschleife 
	while(1) {

#ifdef __USE_WDT__
		//Watchdog zurücksetzen
		wdt_reset();
#endif


#ifdef __SNAP__
		if(SnapPacketReceived()){
			SnapDecodeReceived((unsigned char *)& main_regs.ctrl);
		}
#endif

		//mal schauen, ob sich im CTRL-Register was geändert hat
		if( main_regs.ctrl^main_tmp_ctrl_reg ) {

			if( main_regs.ctrl & (1<<REG_CTRL_WR_EEREGS) ) { // EEProm-Register schreiben ?
			 
				//extrem Wichtig!!! sonst eventuell Loop mit massiven EEProm-Schreibaktionen
				main_regs.ctrl &= ~((1<<REG_CTRL_WR_EEREGS) | (1<<REG_CTRL_RESET)); 

				//Registerwerte in dem EEProm schreiben
				eeprom_update_block ((const void*)&main_regs, (void*)&main_ee_regs, sizeof(MAIN_REGS) ); //
			}

			if( main_regs.ctrl & (1<<REG_CTRL_RESET) ) { // Reset des Controllers auslösen ?
				_delay_ms(15);
				jump_to_app();
   			}
			   
			main_tmp_ctrl_reg = main_regs.ctrl;
		}
		
		
		 cur_heatsink_temperature = get_Temperature(SENSOR_TH_HEADSINK, DEFAULT_RTH_VOR);
		 if (cur_heatsink_temperature > 50) {
				jump_to_app();
		 }
		 
		 cur_vbus_volage = get_VBusVoltage(SENSOR_POWER_VOLTAGE, DEFAULT_R1, DEFAULT_R2, DEFAULT_VREF);
		 if (cur_vbus_volage < DEFAULT_LOWER_BV_THRESHOLD || cur_vbus_volage > DEFAULT_UPPER_BV_THRESHOLD) {
				//jump_to_app();
		 }
		 
		 if (main_task_scheduler & PROCESS_PCF8575){
			 if (process_pcf8575())
				main_task_scheduler &= ~PROCESS_PCF8575;
		 }
		 
		 if (main_task_scheduler & PROCESS_MONITOR_STATE){
			 if (process_State())
				main_task_scheduler &= ~PROCESS_MONITOR_STATE;
		 }

	};

	return 0;
}
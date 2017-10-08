/**
 * @file	prog.c
 * @author	René Schönrock
 * @brief	Firmwareprogrammierung .
 *
 *
 * @date	15.05.2015
 * @see		Datenblatt des ATmega8 von Atmel
 *	
 * Über den Bus kann mit Hilfe des Registerinterfaces eine Firmwareaktualisierung 
 * vorgenommen werden.
 * 
 * verwendete Resourcen:
 *	- keine
 */



#include <string.h>
#include <stdio.h>
#include <avr/io.h>
#include <avr/eeprom.h>
#include <avr/boot.h>
#include <inttypes.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>

#include <avr/wdt.h>
 
#include "config.h"

#include "uart.h"

#ifdef __SNAP__
#include "snap.h"
#endif

#include "prog.h"



uint8_t t2_overflow_counter;
uint8_t sec_flag=0;
uint8_t display_state_flag=0;

uint8_t led_test_channel=0;
uint8_t led_test_flag=0;

/**
 * @fn		void (*jump_to_app)(void) = __APPLICATION_START_ADDR__
 * @brief	Einsprungspunkt für die Applikation
 */
 void (*jump_to_app)(void) = __APPLICATION_START_ADDR__;

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
  * @struct	PROG_ENV prog_env
  * @brief	Enviroment für die Firmwareprogrammierung
  */
  struct PROG_ENV	prog_env = 
 {
	(1<<PROG_STATE_READY),						
	{},
	0,
	__BOOT_SIGNATURE__,
	__SW_RELEASE__,
	__SW_RELEASE_DATE__,

	DISPLAY_STATE,
	
	PROG_NONE,
	0,
	0,
	{},
 };



/**
 * @fn		int prog_ctrl(unsigned char prog_ctrl_reg)
 * @brief	Programmierung
 *
 * Ausführliche Beschreibung
 *
 * @param	cmd
 *
 * @return	TRUE | FALSE
 *
 * @note	steht in der Bootloader-Section.
 */
 unsigned char prog_ctrl(void) 
 {
  uint16_t i;
  uint8_t sreg;
  uint16_t page;
  unsigned char *buf = prog_env.mem_page;

  	sreg = SREG;	//Statusregister sichern

	//prog_env.state &= 0x0F;
	//prog_env.state |= (1<<PROG_STATE_BUSY);

  	cli();			// Disable interrupts.

	//eeprom_busy_wait ();

	//prog_env.ctrl = cmd;

	switch (prog_env.ctrl)
	{
		case WRITE_FPAGE:
			page = (prog_env.page_addr & ~(SPM_PAGESIZE-1));

//  			cli();			// Disable interrupts.
			boot_page_erase (page);
			sei();
			
			boot_spm_busy_wait ();      // Wait until the memory is erased.

			for (i=0; i<SPM_PAGESIZE; i+=2) {
				// Set up little-endian word.
				uint16_t w = *buf++;
				w += (*buf++) << 8;
  				cli();			// Disable interrupts.
				boot_page_fill (page + i, w);
				sei();
			}

  			cli();			// Disable interrupts.
			boot_page_write (page);     // Store buffer in flash page.
			sei();
			
			boot_spm_busy_wait();       // Wait until the memory is written.
			break;

		case ERASE_FPAGE:		//!<Page im Programmspeicher löschen
			page = (prog_env.page_addr & ~(SPM_PAGESIZE-1));
			boot_page_erase (page);
			sei();
			boot_spm_busy_wait ();      // Wait until the memory is erased.
			break;

		case WRITE_EPAGE:		//!<Page in den EEPromspeicher schreiben
			sei();
#ifdef __USE_WDT__
			wdt_enable(	WDTO_500MS);
#endif
//			wdt_disable();
			for (i=0; i<prog_env.size_data; i++) {
				wdt_reset();
				eeprom_update_byte ((void*)prog_env.page_addr+i, prog_env.mem_page[i]);
			}
			//eeprom_write_block ((const void*)prog_env.mem_page, (void*)prog_env.page_addr+1, prog_env.size_data);
			//Watchdogtimer aktivieren
#ifdef __USE_WDT__
			wdt_enable(WDTO_60MS);
#endif
			break;

/*
		case READ_CPU_SIGNATURE:	//!<CPU-Signatur auslesen
			prog_env.cpu_signature[0] = boot_sig_byte_get(0x0000);
			prog_env.cpu_signature[1] = boot_sig_byte_get(0x0002);
			prog_env.cpu_signature[2] = boot_sig_byte_get(0x0004);
			break;
*/
#if defined(__AVR_ATmega8__) || defined(__AVR_ATmega8A__)
		case READ_LOCK_BITS:		//!<Lock-Bits auslesen
 			prog_env.lock_bits = boot_lock_fuse_bits_get(GET_LOCK_BITS);
			break;

//		case WRITE_LOCK_BITS:	//!<Lock-Bits schreiben (vorsicht!!!)
//			boot_lock_bits_set(prog_env.lock_bits);
//			break;

		case READ_FUSE_BITS:		//!<Fuse-Bits auslesen
 			prog_env.fuse[0] = boot_lock_fuse_bits_get(GET_LOW_FUSE_BITS);
 			prog_env.fuse[1] = boot_lock_fuse_bits_get(GET_HIGH_FUSE_BITS);
 			prog_env.fuse[2] = boot_lock_fuse_bits_get(GET_EXTENDED_FUSE_BITS);
			break;
		//case WRITE_FUSE_BITS:		//!<Fuse-Bits schreiben (vorsicht!!!)
#else		
		case READ_LOCK_BITS:		//!<Lock-Bits auslesen
		case READ_FUSE_BITS:		//!<Fuse-Bits auslesen
#endif
		default:
			prog_env.state |= (1<<PROG_STATE_ERROR);
			SREG = sreg;
			return PROG_NONE;
	}

	// Re-enable interrupts (if they were ever enabled).
	SREG = sreg;
		
	return PROG_PENDING;
 }




/**
 * @fn		void start_app(void) 
 * @brief	Start der Applikation
 *
 * Ausführliche Beschreibung
 *
 *
 * @note	
 */
void start_app(void)
{

	//MCUCR = _BV(IVCE); // wechsel der Interrupt Vectoren 
	//MCUCR = 0; // Interrupts auf App-Section umschalten 

	//Watchdogtimer aktivieren
	// das bedeutet, dass die Applikation 15ms Zeit hat, um durchzustarten
	// sonst schlägt wieder der Bootloader auf
	// (Sicherheitsmechanismus, falls eine neue, geflashte Applikation nichts taugt)
//#ifdef __USE_WDT__
//	wdt_enable(WDTO_250MS);
//#endif

  	cli();			// Disable interrupts.

  	// Reenable RWW-section again. We need this if we want to jump back
  	// to the application after bootloading.
  	boot_rww_enable ();

	wdt_reset();
	
	jump_to_app();
}


/************************************************************************
 * @fn		ISR(SIG_OVERFLOW1)
 * @brief	ISR für 
 *
 * Ausführliche Beschreibung
 *
 * @param   keine 
 * @return	keine
 */
ISR(TIMER2_OVF_vect)
{
	if (++t2_overflow_counter >= 15 )
	{
		t2_overflow_counter=0;	
		sec_flag++;
	}
}


/************************************************************************
 * @fn		void display_state(void)
 * @brief	status des bootloader wird über rgb-led angezeigt
 *
 * Ausführliche Beschreibung
 *
 * @param   keine 
 * @return	keine
 */
void display_state(void) 
{
	if ((sec_flag^display_state_flag)&0x01){
		display_state_flag = sec_flag;
		
		switch(prog_env.state&0xF0){
			case (1<<PROG_STATE_READY):
				//LED_TOGGEL(HEART_BEET_LED);
				break;
				
			case (1<<PROG_STATE_ERROR):
				//LED_TOGGEL(ERROR_LED);
				break;
				
			case (1<<PROG_STATE_BUSY):
				//LED_TOGGEL(PROG_LED);
				break;
		}
	}
}



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
#if defined(__AVR_ATmega88__)
	MCUCR = _BV(IVCE); // enable wechsel der Interrupt Vectoren
	MCUCR = _BV(IVSEL); // Interrupts auf Boot Section umschalten
	
	prog_env.state = ((MCUSR & 0x0F) | _BV(PROG_STATE_READY));
	MCUSR &= 0xF0;
	
	TIMSK2 = ((1<<TOIE2));//(1<<TICIE1) | (1<<TOIE1)
	TCCR2A = ((1<<WGM21) | (1<<WGM20));
	TCCR2B = ((0<<FOC2A) | (0<<FOC2B) | (0<<WGM22) | (1<<CS22) | (1<<CS21) | (1<<CS20));//clkI/O/8 (From prescaler)
	
#elif defined(__AVR_ATmega8__) || defined(__AVR_ATmega8A__) || defined(__AVR_ATmega16__) || defined(__AVR_ATmega32__)
	GICR = _BV(IVCE); // enable wechsel der Interrupt Vectoren
	GICR = _BV(IVSEL); // Interrupts auf Boot Section umschalten
	
	prog_env.state = ((MCUCSR & 0x0F) | _BV(PROG_STATE_READY));
	MCUCSR &= 0xF0;
	
	TCCR2 = ((0<<FOC2) | (1<<WGM21) | (1<<WGM20) | (0<<COM21) | (0<<COM20) | (1<<CS22) | (1<<CS21) | (1<<CS20));//clkI/O/1024 (From prescaler)
	TIMSK |= (1<<TOIE2);
	
#else
	#error Wrong AVR!
#endif

	//Bootloader-Switch (Power-On- und ext. Reset starten die Applikation)
	//if ((prog_env.state & (_BV(PORF) | _BV(EXTRF))) && !(prog_env.state & _BV(WDRF))) {
	if ((prog_env.state & _BV(PORF)) && !(prog_env.state & _BV(WDRF))) {
		start_app();
	}
#ifdef __USE_WDT__
//Watchdogtimer aktivieren
//	wdt_enable(WDTO_250MS);
#else
//	wdt_disable();
#endif


	/* Einstellen der Baudrate und aktivieren der Interrupts */
	uart_init( UART_BAUD_SELECT(BOOT_UART_BAUD_RATE,F_CPU) );
	//uart_init( UART_BAUD_SELECT(BOOT_UART_BAUD_RATE,8294400UL) );

#ifdef __SNAP__
	//unsigned char nodeAddress=1;
	//unsigned char nodeAddress=dev_addr;
	unsigned char nodeAddress=eeprom_read_byte((uint8_t *)0x00);
	//unsigned char nodeAddress=2;
	SnapInit(nodeAddress);
#endif	
	//Interrupts anwerfen
	sei();

}


/**
 * @fn		int main(void) 
 * @brief	Programmierung
 *
 * Ausführliche Beschreibung
 *
 *
 * @note	steht in der Bootloader-Section.
 */
 int main(void) 
 {

	init_Sys();
	
	while (1) {

		if(SnapPacketReceived()){
			
			SnapDecodeReceived((unsigned char *)&prog_env.state);
			
			if (prog_env.ctrl > PROG_PENDING)
			{
				prog_env.ctrl= prog_ctrl();
				
				if(prog_env.ctrl == PROG_PENDING){
					prog_env.state &= 0x0F;
					prog_env.state |= (1<<PROG_STATE_BUSY);
				}
			}
			else
			{
				prog_env.state &= 0x0F;
				prog_env.state |= (1<<PROG_STATE_READY);
			}
			
		}
		
		switch(prog_env.boot_mode){
			case DISPLAY_STATE:
				display_state();
				break;
				
			case LED_TEST:
				//if (!led_test())
				//	prog_env.boot_mode = DISPLAY_STATE;
				break;

			case START_APP:
				_delay_ms(15);
				start_app();
				break;

			case BOOT_RESET:
				wdt_enable(WDTO_15MS);
				while(1){};
				break;
				
			default:
				break;
		}
		
		//Watchdog zurücksetzen
//#ifdef __USE_WDT__
		//Watchdog zurücksetzen
		wdt_reset();
		
//#endif
	}
	return 0;	//never happens
 }

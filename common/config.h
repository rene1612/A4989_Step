/**
 * @file	config.h
 * @author	Rene Schönrock
 * @brief	Configurations-Datei für den LED-Controller
 *
 * @date	06.01.2016
 * @see		
 */

#ifndef __CONFIG_H
#define __CONFIG_H

#define __BOOTLOADER_START_ADDR__	(void *)0x1C00
#define __APPLICATION_START_ADDR__	(void *)0x0000
#define __BOOT_CODE_START_ADDR__	(void *)0x1800


 #define __COMPILATION_DAY__		25UL
 #define __COMPILATION_MONTH__		8UL
 #define __COMPILATION_YEAR__		2018UL



/********************************************************************
 * @def		__DEV_SIGNATURE__
 * @brief	"eindeutige" Gerätekennung
 */
 #define __DEV_SIGNATURE__			0xAB

/********************************************************************
 * @def		__BOOT_SIGNATURE__
 * @brief	"eindeutige" Bootloaderkennung
 *
 * @see		
 */
 #define __BOOT_SIGNATURE__			0xCD

/********************************************************************
 * @def		__SW_RELEASE__
 * @brief	aktuelle Softwareversion
 * @note	gilt auch für das Bootloadertarget (Bootloader hat auch diese Versionsnummer) 
 * Formatierung:
 *	- Byte 0 -> Minor-Nr.
 *	- BYTE 1 -> Mayor-Nr.
 */
 #define __SW_RELEASE__				0x0096


/********************************************************************
 * @def		__SW_RELEASE__
 * @brief	Datum der aktuelle Softwareversion 
 * @note	gilt auch für das Bootloadertarget (Bootloader hat auch dieses Versionsdatum) 
 * Formatierung:
 *	- Byte 0 -> Tag
 *	- BYTE 1 -> Monat
 *	- BYTE 2 -> Jahr
 *	- BYTE 3 -> Jahr
 */
 #define __SW_RELEASE_DATE__	(__COMPILATION_YEAR__ + (__COMPILATION_MONTH__<<16) + (__COMPILATION_DAY__<<24))
 //#define __SW_RELEASE_DATE__		0x250807DAUL


/*********************************************************************
 * @def		DEFAULT_DEV_ADDR
 * @brief	Adressen 
 */
#ifndef DEFAULT_DEV_ADDR
	#define DEFAULT_DEV_ADDR	0x03
#endif
	
#define BROADCAST_ADDR		0xFF
#define BACKUP_DEV_ADDR		0xFE

#define PCF8575_TWI_DEV_ADDR		0x40


/**********************************************************************
 * @brief	Defaults
 */
#define APP_UART_BAUD_RATE     115200UL     /* Baudrate */
//#define APP_UART_BAUD_RATE     38400     /* Baudrate */
//#define APP_UART_BAUD_RATE     9600UL     /* Baudrate */

//#define BOOT_UART_BAUD_RATE     38400     /* Baudrate */
#define BOOT_UART_BAUD_RATE     115200UL     /* Baudrate */
//#define BOOT_UART_BAUD_RATE     9600UL     /* Baudrate */
//#define BOOT_UART_BAUD_RATE     2400UL     /* Baudrate */


#define MAX_BAUDRATE			115200UL
#define MIN_BAUDRATE			2400UL



#define A4989_EN		PA1
#define A4989_SR		PA0
#define A4989_PFD1		PB7
#define A4989_PFD2		PB4
#define A4989_MS1		PB5
#define A4989_MS2		PB6
#define A4989_RESET		PB3

#define A4989_ENABLE		(PORTA &= ~_BV(A4989_EN))
#define A4989_DISABLE		(PORTA |= _BV(A4989_EN))

#define A4989_RESET_ENABLE	(PORTB &= ~_BV(A4989_RESET))
#define A4989_RESET_DISABLE	(PORTB |= _BV(A4989_RESET))

#define A4989_SR_ENABLE		(PORTA &= ~_BV(A4989_SR))
#define A4989_SR_DISABLE	(PORTA |= _BV(A4989_SR))

#define STEP_DIR		PB2
#define STEP_ENA		PD2
#define STEP_SYNC		PB1
#define STEP_CLK		PB0

#define PCF8575_INT_PIN			PIND3
#define PCF8575_INT_DIR			DDRD
#define STEP_ENA_INT_DIR		DDRD
#define STEP_ENA_INT			INT0
#define STEP_ENA_INPUT_PIN		PIND2
#define STEP_ENA_INPUT			(PIND & (1<<STEP_ENA_INPUT_PIN))
#define STEP_DIR_INPUT_PIN		PINB2
#define STEP_DIR_INPUT			(PINB & (1<<STEP_DIR_INPUT_PIN))


#define ENCODER_A_MASK		0x000F
#define ENCODER_B_MASK		0x00F0
#define ENCODER_C_MASK		0x0F00
#define ENCODER_ALL_MASK	(ENCODER_A_MASK | ENCODER_B_MASK | ENCODER_C_MASK)

// Define input pin to be used to detect UART activity
#define UART_RXSCAN_DDR		DDRD
#define UART_RXSCAN_PORT	PORTD
#define UART_RXSCAN_PIN		PD0

/**
 * @brief	Defaults
 * @see		adc.c
 */
 #define DEFAULT_RTH_VOR			9960	//!<Vorwiderstandswert für den Termistor
 #define DEFAULT_ALPHA_1			20		//!<Alpha 1 für gleitendes Mittel
 #define DEFAULT_ALPHA_2			20		//!<Alpha 2 für gleitendes Mittel


 #define DEFAULT_STANDBY_CURRENT_TIMEOUT_MS	800		//!<Timeout in ms
 #define T1_TIMER_RESOLUTION	(F_CPU/1024)
 #define DEFAULT_STANDBY_CURRENT_TIMEOUT	(T1_TIMER_RESOLUTION*DEFAULT_STANDBY_CURRENT_TIMEOUT_MS)/1000		//!<Timeout in ms

 //#define DEFAULT_STANDBY_CURRENT_TIMEOUT_MS	800		//!<Timeout in ms
 //#define T2_TIMER_RESOLUTION	(F_CPU/128/256)
 //#define DEFAULT_STANDBY_CURRENT_TIMEOUT	(T2_TIMER_RESOLUTION*DEFAULT_STANDBY_CURRENT_TIMEOUT_MS)/1000		//!<Timeout in ms
 
 #define DEFAULT_MAX_HEATSINK_TEMP	60		//!<obere Temperaturschwelle für den Kühlkörper
 #define DEFAULT_UPPER_BV_THRESHOLD	50000	//!<obere Boardspannungsschwelle
 #define DEFAULT_LOWER_BV_THRESHOLD	26000	//!<untere Boardspannungsschwelle
 #define DEFAULT_R1					2200	//!<Spannungsteiler R1 in Ohm
 #define DEFAULT_R2					47000	//!<Spannungsteiler R2 in Ohm
 #define DEFAULT_VREF				3378	//!<Default-Referenzspannung in mV
 #define DEFAULT_CURENT_SHUNT		25		//!<Default Widerstand für die Strommessung in mOhm)
 #define DEFAULT_CURRENT_OFFSET		2000	//!<Offset für Motorstrom in mA (Minimalstrom)
 #define DEFAULT_MAX_CURRENT		10000	//!<Maximalstrom für Motorstrom in mA
 #define PWM_TIMER_TICKS_PER_1000MA	(((unsigned long)DEFAULT_CURENT_SHUNT*8*0x03FF)/DEFAULT_VREF)
 //#define CURRENT_OFFSET_VAL			(((DEFAULT_CURRENT_SHUNT * DEFAULT_CURRENT_OFFSET)/1000)*8)*(0xFFFF/DEFAULT_VREF)
 #define CURRENT_OFFSET_VAL			(((unsigned long)PWM_TIMER_TICKS_PER_1000MA * DEFAULT_CURRENT_OFFSET)/1000)
 #define CURRENT_SET_FACTOR			((unsigned long)(DEFAULT_MAX_CURRENT-DEFAULT_CURRENT_OFFSET)*PWM_TIMER_TICKS_PER_1000MA)/15000	//!<
 
#endif


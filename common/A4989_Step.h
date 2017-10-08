/*! \file       main.h
 *  \brief      Firmware für den Controller
 *  \author     René Schönrock
 *  \version    1.0
 *  \date       01.02.2011
 *  \bug        keine.
 *  \warning    keine.
 *  \note       
 *  \see        
 *  \todo       
 *
 *  Details.
 */


#ifndef  __A4989_DTEP_H_
 #define  __A4989_DTEP_H_

 #ifndef __ASSEMBLER__  
 #include "types.h"
 #endif
  
 ////////////////////////////////////////////////////////////////////////////////////////////////////////////
 //                  Diverse Definitionen und Konstanten
 ////////////////////////////////////////////////////////////////////////////////////////////////////////////
 
 
 
 
/*!
 * Main Reg-Defines
 */


 ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 //
 // Datenstructuren
 //
 ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

 #define PROCESS_PCF8575		0x01
 #define PROCESS_MONITOR_STATE	0x02
 #define PROCESS_STEP_ENABLE	0x04



#define TIMER0_250_MS (F_CPU/256/256/4)


/**
 * Bit-Defines für das Controllregister
 */
 #define REG_CTRL_ACTIVATE			0
 #define REG_CTRL_DEACTIVATE		1
 #define REG_CTRL_ERROR_RESET		2
 #define REG_CTRL_AUTO_CURRENT		3
 #define REG_CTRL_BOOT				4	//!<Bit zum 
 #define REG_CTRL_EE_DEFAULTS		5	//!<Sollen die Defaultwerte aus dem EEPROM geladen werden beim Start?
 #define REG_CTRL_WR_EEREGS			6	//!<EEProm-Register schreiben
 #define REG_CTRL_RESET				7	//!<Reset des Controllers auslösen


 typedef union   _SWITCH_PORT
 {
	 unsigned char port[2];
	 unsigned int word;
	 struct 
	 {
		unsigned int  encoder_a	:4;								//Datenleitungen (P0 am Portexpander)
		unsigned int  encoder_b	:4;								//Datenleitungen (P0 am Portexpander)
		unsigned int  encoder_c	:4;								//Datenleitungen (P0 am Portexpander)
		unsigned int  led_1		:1;								//Datenleitungen (P0 am Portexpander)
		unsigned int  led_2		:1;								//Datenleitungen (P0 am Portexpander)
		unsigned int  led_3		:1;								//Datenleitungen (P0 am Portexpander)
		unsigned int  led_4		:1;								//Datenleitungen (P0 am Portexpander)
	 }val;
 }SWITCH_PORT;

enum SYS_STATE{
	SYS_OK,
	SYS_ACTIVE_START,
	SYS_ACTIVE_FC,
	SYS_ACTIVE_SBC,
	SYS_ACTIVE_STOP,
	SYS_ERROR,
	};

/**
 * @struct	REG
 * @brief	Registersatz des Controllers.
 *
 * @note	Der Registersatz wird im RAM und im EEProm gehalten
 */
 typedef struct
 {
 /**
  * @var	unsigned char ctrl
  * @brief	Steuer-Register für den Controller
  *
  *	- Bit 0 -> Bit zum Ein-/Ausschalten 
  *	- Bit 5 -> Sollen die Defaultwerte aus dem EEPROM geladen werden beim Start?
  *	- Bit 6 -> EEProm-Register schreiben
  *	- Bit 7 -> Reset des Controllers auslösen
  *
  * @see	CTRL_REG
  */
  unsigned char		ctrl;
  
  enum SYS_STATE	sys_state;
  unsigned char		monitor_led_state;
  
  unsigned int		encoder_inp_mask;

  unsigned char		max_heatsink_temp;
 
  unsigned int		upper_bv_threshold;
  unsigned int		lower_bv_threshold;

  unsigned int		full_current;
  unsigned int		standby_current;
  signed char		cur_heatsink_temperature;
  unsigned int		cur_vbus_volage;
  
 /**
  * @var	uint32_t		baudrate
  * @brief	
  */
  uint32_t		baudrate;

 /**
  * @var	unsigned char dev_signature
  * @brief	Register mit der Gerätekennung
  * @see	__DEV_SIGNATURE__
  * @see	DEV_SIGNATURE_REG
  * @see	config.h
  */
  unsigned char		dev_signature;

 /**
  * @var	unsigned int sw_release
  * @brief	Register mit der Softwareversion
  * @see	__SW_RELEASE__
  * @see	SW_REL_REG
  * @see	config.h
  */
  unsigned int		sw_release;						

 /**
  * @var	unsigned int sw_release_date
  * @brief	Register mit dem Datum der Softwareversion
  * Formatierung:
  *	- Byte 0 -> Tag
  *	- BYTE 1 -> Monat
  *	- BYTE 2 -> Jahr
  *	- BYTE 3 -> Jahr
  * @see	__SW_RELEASE_DATE__
  * @see	SW_REL_DATE_REG
  * @see	config.h
  */
  unsigned long		sw_release_date;						
 }MAIN_REGS;




/**
 * @def		REG_ADDR_AUTO_INC
 * @brief	Flag im Registeradressbyte, welches eine automatische Incrementierung
 *			der Registeradresse nach einer Lese- oder Schreibaktion bewirkt.
 *
 * @note	Die Registeradresse wird vom Master bei jeder Schreibaktion als erstes Byte gesendet.
 * @see		fco_reg_addr
 * @see		TWI-WRITE-Mode
 */
 #define REG_ADDR_AUTO_INC	0x80


 ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 //
 // Funktionen(Prototypes)
 //
 ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void set_sys_state (enum SYS_STATE sys_state);

extern  MAIN_REGS main_regs;

extern SWITCH_PORT pcf8575;

extern uint8_t main_task_scheduler;

#endif  

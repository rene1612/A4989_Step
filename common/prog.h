/**
 * @file	prog.h
 * @author	René Schönrock
 * @brief	Firmwareprogrammierung des FCO-Controllers.
 *
 * @date	15.05.2009
 * @see		Datenblatt des ATmega88 von Atmel
 */

#ifndef __PROG_H__
 #define __PROG_H__

 


/**
 * Bit-Defines für das Statusregister
 */
 #define PROG_STATE_POR		PORF	//!<Bit zeigt an, dass .
 #define PROG_STATE_EXTR	EXTRF	//!<Bit zeigt an, dass .
 #define PROG_STATE_BOR		BORF	//!<Bit zeigt an, dass .
 #define PROG_STATE_WDR		WDRF	//!<Bit zeigt an, dass .
 #define PROG_STATE_READY	4		//!<Bit zeigt an, dass die Ausführung des Programmierkommandos abgeschlossen wurde.
 #define PROG_STATE_BUSY	5		//!<Bit zeigt an, dass das entspr. Programmierkommando gerade bearbeitet wird.
 #define PROG_STATE_ERROR	6		//!<Bit zeigt an, dass bei der Ausführung des Programmierkommandos ein Fehler aufgetreten ist.
 #define PROG_STATE_PROGRAMMING	7	//!<Bit zeigt an, dass die Programmierung aktiv ist.





/**
 * @enum BOOT_MODE
 * @brief	Kommandos für die Firmwareprogrammierung.
 *
 * @note	
 */
enum BOOT_MODE {
	DISPLAY_STATE,
	LED_TEST,
	START_APP,
	BOOT_RESET,
};


/**
 * @enum	PROG_CTRL
 * @brief	Kommandos für die Firmwareprogrammierung.
 *
 * @note	
 */
 enum PROG_CTRL
 {
	PROG_NONE = 0,		//!<keine Programmieraktion
	PROG_PENDING,		//!<Programmiervorgang läuft
	WRITE_FPAGE,		//!<Page in den Programmspeicher schreiben
	READ_FPAGE,			//!<Page aus den Programmspeicher lesen
	ERASE_FPAGE,		//!<Page im Programmspeicher löschen
	VERIFY_FPAGE,		//!<Page im Programmspeicher verifizieren

	WRITE_EPAGE,		//!<Page in den EEPromspeicher schreiben
	READ_EPAGE,			//!<Page aus den EEPromspeicher lesen
	ERASE_EPAGE,
	VERIFY_EPAGE,

	//READ_CPU_SIGNATURE,	//!<CPU-Signatur auslesen

	READ_LOCK_BITS,		//!<Lock-Bits auslesen
	WRITE_LOCK_BITS,	//!<Lock-Bits schreiben (vorsicht!!!)

	READ_FUSE_BITS,		//!<Fuse-Bits auslesen
	//WRITE_FUSE_BITS,	//!<Fuse-Bits schreiben (vorsicht!!!)

	//START_APP,			//!<Applikation starten

	PROG_ERROR=0xFF		//!<Fehler
 };




/**
 * @struct	PROG_ENV
 * @brief	Enviroment für die Firmwareprogrammieung des Beam-Controllers.
 *
 * @note	Der Registersatz wird im RAM gehalten
 */
 struct PROG_ENV
 {
 /**
  * @var	unsigned char state
  * @brief	Status der Firmwareprogrammieung
  */
  unsigned char		state;
   				
 /**
  * @var	unsigned char fuse
  * @brief	die Atmel-Fusebits
  * @see	Datenblatt des Atmel-Controller
  */
  unsigned char		fuse[3];						

 /**
  * @var	unsigned char lock_bits
  * @brief	die Atmel-Lockbits
  * @see	Datenblatt des Atmel-Controller
  */
  unsigned char		lock_bits;						

 /**
  * @var	unsigned char dev_signature
  * @brief	Register mit der Gerätekennung
  * @see	__DEV_SIGNATURE__
  * @see	DEV_SIGNATURE_REG
  * @see	config.h
  */
  unsigned char		boot_signature;

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

 /**
  * @var	enum BOOT_MODE	boot_mode
  * @brief	
  */
  enum BOOT_MODE	boot_mode;	

  /**
  * @var	unsigned char ctrl
  * @brief	Firmwareupload-Kommandos
  */
  enum PROG_CTRL	ctrl;						

/**
  * @var	unsigned int page_addr
  * @brief	Speicheradresse (FLASH, EEPROM)
  * @note	Wortadressen
  */
  unsigned int		page_addr;						

 /**
  * @var	unsigned char size_data
  * @brief	Anzahl der Datenbytes, die programmiert werden sollen
  */
  unsigned char		size_data;						

 /**
  * @var	unsigned char mem_page
  * @brief	Speicherpage für den Firmwareupload
  * @see	SPM_PAGESIZE
  * @note	SPM_PAGESIZE = 64 Bytes for ATmega88
  */
  unsigned char		mem_page[SPM_PAGESIZE];						
 };



 //extern struct PROG_ENV	prog_env;

/*
#define SIGRD	5
#define __BOOT_SIG_BYTE_GET      (_BV(SPMEN) | _BV(SIGRD))


#define boot_sig_byte_get(address)                   		\
(__extension__({                                           \
    uint8_t __result;                                      \
    __asm__ __volatile__                                   \
    (                                                      \
        "ldi r30, %3\n\t"                                  \
        "ldi r31, 0\n\t"                                   \
        "sts %1, %2\n\t"                                   \
        "lpm %0, Z\n\t"                                    \
        : "=r" (__result)                                  \
        : "i" (_SFR_MEM_ADDR(__SPM_REG)),                  \
          "r" ((uint8_t)__BOOT_SIG_BYTE_GET),             \
          "M" (address)                                    \
        : "r0", "r30", "r31"                               \
    );                                                     \
    __result;                                              \
}))
*/

#endif

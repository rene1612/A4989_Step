///////////////////////////////////////////////////////////////////////////////////
// Datei: 	  	pcf8575.h
//	
// Prozessor:	ATmega88
//
// Funktion: 	 
//
// Projekt: 	  
//
// Update: 	   	11.02.2009
//
// Compiler: 	 
//
// Ersteller:	René Schönrock, 
//
// Bemerkung:	keine
///////////////////////////////////////////////////////////////////////////////////
 
#ifndef __PCF8575_H_
 #define __PCF8575_H_
 
 typedef union   _ENC_PORT
 {
	 unsigned char port[2];
	 unsigned int word;
	 struct
	 {
		 unsigned int  current			:4;								//Datenleitungen (P0 am Portexpander)
		 unsigned int  microstep_decay	:4;								//Datenleitungen (P0 am Portexpander)
		 unsigned int  sync_rect_mode	:4;								//Datenleitungen (P0 am Portexpander)
		 unsigned int  ledn				:4;								//Datenleitungen (P0 am Portexpander)
	 }val;
 }ENC_PORT;
 
//=================================================================================
// pcf8575_write_to :  
//--------------------------------------------------------------------------------
// Eingabe :
//
// Ausgabe :
//--------------------------------------------------------------------------------
// Hinweis :        dauert ca. 350 uS
//=================================================================================
BYTE pcf8575_write_to (BYTE addr, unsigned char *data); 


//=================================================================================
// pcf8575_write_to :  
//--------------------------------------------------------------------------------
// Eingabe :
//
// Ausgabe :
//--------------------------------------------------------------------------------
// Hinweis :        dauert ca. 350 uS
//=================================================================================
BYTE pcf8575_read_from (BYTE addr, PBYTE destiantion); 


/************************************************************************
 * @fn		void pcf8575_init (void)
 * @brief	
 *
 * Init-Function for Portexpander
 *
 * @note    
 *		
 */
void pcf8575_init (void);
 
 
/************************************************************************
 * @fn		unsigned char process_pcf8575(void)
 * @brief	Arbeitsloop für den Portexpander
 *
 * This function is the Interrupt Service Routine (ISR), and called when
 * the TWI interrupt is triggered; that is whenever a TWI event has occurred.
 *
 * @note    
 *		
 */
unsigned char process_pcf8575(void);
 
#endif

 ///////////////////////////////////////////////////////////////////////////////////
	// Datei  	: 	  crc.h
	//
	// Prozessor	: 	ATmega128
	//
	// Funktion	: 	 CRC calculation routine
	//
 // Projekt	: 	  
	//
	// Update	: 	   03.05.2004
 //
 // Compiler	: 	 
 //
 // Ersteller	:	 René Schönrock, Andreas Herrmann, 
	//
	// Bemerkung	:  keine
 ///////////////////////////////////////////////////////////////////////////////////


#ifndef __CRC_H
 #define __CRC_H
 
 #include "types.h"

 /*crc-16 standard root*/
 
 #ifndef POLYNOMIAL
	#define POLYNOMIAL         0x8005
 #endif
 
 #ifndef REV_POLYNOMIAL
	#define REV_POLYNOMIAL     0xA001
 #endif
 
 #ifndef CRC_INITIAL_VALUE
	#define CRC_INITIAL_VALUE  (WORD)0x0000 
#endif

/**
 * @fn		unsigned int calcCRC16(unsigned int crc, unsigned int c)
 * @brief	Berechnet CRC-Prüfsumme
 *
 * Ausführliche Beschreibung
 *
 * @param	crc		aktueller CRC-Wert
 * @param	c		Char-Wert, der zur CRC hinzugerechnet werden soll
 *
 * @return	neuer CRC-Wert
 *
 * @note	steht in der Bootloader-Section.
 */
 unsigned int calcCRC16r(unsigned int crc, unsigned int c);



#endif  // CRC_H

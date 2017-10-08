 ///////////////////////////////////////////////////////////////////////////////////
 // Datei  	: 	  types.h
 //	
	// Prozessor	: 	ATmega162
	//
	// Funktion	: 	 
	//
 // Projekt	: 	  
	//
	// Update	: 	   15.01.2004
 //
 // Compiler	: 	 
 //
 // Ersteller	:	 René Schönrock, Andreas Herrmann, 
	//
	// Bemerkung	:  keine
 ///////////////////////////////////////////////////////////////////////////////////
 
#ifndef   __TYPES_H
 #define   __TYPES_H
 
 
 ///////////////////////////////////////////////////////////////////////////////////
 //                  Diverse Definitionen und Konstanten
 ///////////////////////////////////////////////////////////////////////////////////
 
 #define TRUE               1
 #define FALSE              0

 // Leitungszustand (High / Low)
 #define HIGH               1
 #define LOW                0
 
 #define ON                 1
 #define OFF                0

 #define OUTPUT             1
 #define INPUT              0

 #define OK	            0
 #define ERROR              1
 
 #define TOGGLE		    2
 
 ///////////////////////////////////////////////////////////////////////////////////
 //                       Typdefinitionen
 ///////////////////////////////////////////////////////////////////////////////////
 
 typedef unsigned char  BYTE;
 typedef BYTE          *PBYTE;
 
 typedef unsigned int   UINT;
 typedef UINT          *PUINT;
 
 typedef unsigned int   WORD;
 typedef WORD          *PWORD;
 
 typedef unsigned long  ULONG;
 typedef ULONG         *PULONG;

 typedef unsigned long  DWORD;
 typedef DWORD         *PDWORD;
 
 typedef long           LONG;
 typedef LONG          *PLONG;
 
 typedef int            INT;
 typedef int           *PINT;
 
 typedef char           CHAR;
 typedef char          *PCHAR;
 
 typedef void           VOID;
 typedef void          *PVOID;
 
 //typedef bit            BOOL;
 typedef unsigned char  BOOL;


 // datatype definitions macros
 typedef unsigned char  u08;
 typedef   signed char  s08;
 typedef unsigned short u16;
 typedef   signed short s16;
 typedef unsigned long  u32;
 typedef   signed long  s32;
 typedef unsigned long long u64;
 typedef   signed long long s64;

 ///////////////////////////////////////////////////////////////////////////////////
 //                              Definitionen
 ///////////////////////////////////////////////////////////////////////////////////
 
 
 
#endif
 

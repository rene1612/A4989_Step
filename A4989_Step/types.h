
#ifndef   __TYPES_H
 #define  __TYPES_H
 

//*********************************************************************************
//                       Typdefinitionen
//*********************************************************************************

typedef unsigned char  BYTE;
typedef BYTE          *PBYTE;

typedef unsigned int   UINT;
typedef UINT          *PUINT;

typedef unsigned int   WORD;
typedef WORD          *PWORD;

typedef unsigned long  ULONG;
typedef ULONG         *PULONG;

typedef long           LONG;
typedef LONG          *PLONG;

typedef int            INT;
typedef int           *PINT;

typedef char           CHAR;
typedef char          *PCHAR;

typedef void           VOID;
typedef void          *PVOID;

typedef unsigned char  BOOL;

//********************************************************************************
//                              Definitionen
//********************************************************************************



 // Unions für 16bit Wort
 typedef union
  {
   unsigned int w;
   unsigned char b[2];
  }WORD_UNION;



// Zusstandsdefinitionen

// Zustand (TRUE / FALSE)
#define TRUE           1
#define FALSE          0

// Leitungszustand (High / Low)
#define LOW            0
#define HIGH           1 
#define TOGEL          2 

// Zustand (ON / OFF)
#define ON             1
#define OFF            0

// Ereignis (START / STOP)
#define START          1
#define STOP           0

	// Richtungsdefinition (INPUT / OUTPUT)
#define INPUT             0x00
#define OUTPUT            0xff
                      
// Leitungszustand (High / Low)
#define LINE_LOW       0
#define LINE_HIGH      1 

// Returndefinition (OK / ERROR)
#define OK             1
#define ERROR          0

//andere Defines

//#define NULL           0x0000

#endif

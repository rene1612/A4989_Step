/**
 * @file	state.h
 * @author	René Schönrock
 * @brief	Status-Monitor.
 *
 *
 * @date	23.04.2007
 * @see		
 *	
 *
 * Anzeige des Systemstaus mittels LED.
 * Dabei werden verschiedene Statie mit verschiedenen Blink-Codes
 * visualisiert.
 */



 //extern unsigned char state;



/**
 * @def		STATE_TIMER_RESOLUTION
 */
 #define STATE_TIMER_RESOLUTION	12


/**
 * @def		STATE_PIN
 */
 //#define STATE_PIN	PB1			//!<Achtung: ist normalerweise der PWM-Ausgangspin für die Strahlerleistung
 #define STATE_PIN	PB5		//!<Achtung: ist DW- und Reset-Pin (nur für produktiv-Compilat verwenden



/**
 * Zustände
 */
 #define STATE_OFF					0xFF	//!<Keine Blinken (LED aus)
 	
 #define STATE_OK					0x0F	//!<Zustand alles OK (gleichmäßiges "langsames" Blinken Tastverhältnis 50/50)

 #define STATE_WARN					0xCC	//!<Zustand Warnung (gleichmäßiges "schnelles" Blinken Tastverhältnis 50/50)

 #define STATE_ERR_UNKNOWN			0xAA	//!<Zustand unbekannter Fehler (gleichmäßiges "sehr schnelles" Blinken Tastverhältnis 50/50)
 	
 #define STATE_ERR_HEADSINK_TEMP	0x01	//!<Zustand Fehler LED-Temperatur zu hoch (einmal kurzes blinken)	
 #define STATE_ERR_VBUS_VOLTAGE		0x05	//!<Zustand Fehler Umgebungstemperatur zu hoch? (zweimaliges kurzes blinken)	
 #define STATE_ERR_EXT_OFF		0x15	//!<Zustand "Fehler" externe Komponente (z.B. CPLD) hat den Strahler hardwaremäßig deaktiviert (dreimaliges kurzes blinken)	


/**
 * @fn		void init_StateMon(void)
 */
void init_StateMon(void);


/**
 * @fn		unsigned char set_State(unsigned char new_state)
 */
unsigned char set_State(unsigned char new_state);

/**
 * @fn		unsigned char set_State(unsigned char new_state)
 */
unsigned char set_State(unsigned char new_state);

/**
 * @fn		unsigned char process_State(void)
 */
unsigned char process_State(void);

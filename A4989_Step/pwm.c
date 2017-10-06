/**
 * @file	pwm.c
 * @author	René Schönrock
 * @brief	Digital-Analog-Konverter mittels PWM-Einheit.
 *
 *
 * @date	16.04.2007
 * @see		Datenblatt des ATtiny25/45 von Atmel
 *	
 * Die PWM-Einheit wird benutzt, um einen Digital-Analog-Konverter zu realisieren,
 * mit dessen Hilfe die Leistung des IR-Strahlers eingestellt wird.
 * 
 * verwendete Resourcen:
 *	- Timer 0
 *	- Port PB1
 */

 #include <stdio.h>
 #include <avr/io.h>

 //#include "beam_ctrl.h"

 #include "pwm.h"


/**
 * @var		TCCR0_clk_val
 * @brief	Taktteiler für den PWM-Timer
 */
 unsigned int TCCR1_clk_val;




/**
 * @fn		void init_DAC(unsigned char freq)
 * @brief	Initialisierung der PWM-Einheit
 *
 * Ausführliche Beschreibung
 *
 * @param	freq -> Frequnenz des PWM-Signals [kHz] (30Hz bis 30kHz)
 *
 * @return	keine
 *
 * @note	Die Frequenz freq des PWM-Signals sollte entspr. der RC-Tiefpassfilter(C15,R19) am Pin 2 (FBP) des LTC3783 eingestellt werden.
 * @note	Achtung: Die Pereiodendauer bzw. Frequenz des PWM-Signals kann nur mit gewisser Genauigkeit eingestellt werden (so grob).
 */
 void init_DAC(unsigned char freq)
 {
  TCNT0  = 0x00;

  TCCR1A = ((1<<COM1A1) | (0<<COM1A0) |(1<<WGM11) | (1<<WGM10)); //PWM-Mode 3, Clear OC1A on Compare Match, set OC1A at TOP

  TCCR1B = 0x00; //Timer aus

  OCR1A  = 0x0000;
  OCR1B  = 0x0000;
  DDRD  |= (1<<PD5); //Pin 14 Auf Ausgang (OC1A)

  TCCR1_clk_val = (freq & TCCR1_CLK_VAL_MASK) | (1<<WGM12);
  
  if (TCCR1_clk_val > 5)
   TCCR1_clk_val -= 2; 
 }



/**
 * @fn		void set_DAC(unsigned char dac_val)
 * @brief	Neuen Wert für den Digital-Analog-Konverter einstellen.
 *
 * Ausführliche Beschreibung
 *
 * @param	dac_val -> neuer Wert für den DAC
 *
 * @return	keine
 */
 void set_DAC(unsigned int dac_val)
 {
  if (dac_val)
   {
    OCR1A  = dac_val;
    TCCR1B |= TCCR1_clk_val; //Timer an
   }
  else
   {
    OCR1A  = 0x0000;
	TIFR |= (1<<OCF1B); //TOV0
	PORTD &= ~(1<<PD5);
    TCCR1B &= ~TCCR1_CLK_VAL_MASK; //Timer aus
   }
 }



/**
 * @fn		unsigned int get_DAC(void)
 * @brief	Aktuellen Wert des Digital-Analog-Konverter lesen.
 *
 * Ausführliche Beschreibung
 *
 * @param	keine
 *
 * @return	aktuell eingesteller Wert des Digital-Analog-Konverters
 */
 unsigned int get_DAC(void)
 {
  return OCR1A;
 }

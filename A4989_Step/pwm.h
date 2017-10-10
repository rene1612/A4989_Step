/**
 * @file	pwm.h
 * @author	René Schönrock
 *
 *
 * @date	13.04.2007
 * @see		
 *	
 * PWM-Routinen.
 */


/**
 * @def		TCCR1_CLK_VAL_MASK
 * @brief	wird benötigt zum ausmaskieren des Taktteilers im Control Register OCR1B für den PWM-Timer 1
 */
 #define TCCR1_CLK_VAL_MASK		0x07



//! Prototypes

 void init_DAC(void);

 void set_DAC(unsigned int dac_val);

 unsigned int get_DAC(void);

/**
 * @file	pwm.h
 * @author	Ren� Sch�nrock
 *
 *
 * @date	13.04.2007
 * @see		
 *	
 * PWM-Routinen.
 */


/**
 * @def		TCCR1_CLK_VAL_MASK
 * @brief	wird ben�tigt zum ausmaskieren des Taktteilers im Control Register OCR1B f�r den PWM-Timer 1
 */
 #define TCCR1_CLK_VAL_MASK		0x07



//! Prototypes

 void init_DAC(void);

 void set_DAC(unsigned int dac_val);

 unsigned int get_DAC(void);

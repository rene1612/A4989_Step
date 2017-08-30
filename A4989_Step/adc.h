/**
 * @file	adc.h
 * @author	Ren� Sch�nrock
 * @brief	ADC-Routinen.
 *
 * @date	13.04.2007
 * @see		
 *	
 */

#ifndef __ADC_H__
 #define __ADC_H__


/**
 * @def		ADC_MAX_CANNEL
 * @brief	max. Anzahl der ADC-Kan�le
 */
 #define ADC_MAX_CANNEL			2

/**
 * @def		TH_LED_CH1
 * @brief	Bitmaske f�r den Analogeingangspin (PA3 = Pin35 am Controller)
 * @note	Analoger Eingangspin f�r den LED-Thermistor (Kanal 1).
 */
 #define TH_HEADSINK_PIN_MASK			(1<<PA3)

/**
 * @def		POWER_VOLTAGE
 * @brief	Bitmaske f�r den Analogeingangspin (PA2 = Pin34 am Controller)
 * @note	Analoger Eingangspin f�r die Boardspannung.
 */
 #define POWER_VOLTAGE_PIN_MASK		(1<<PA2)



/**
 * @def		LED_SENSOR_CHANNEL_01
 * @brief	Relevante ADC-Kan�le f�r die ext. Thermistoren 
 *			Kanal f�r den ext. Headsink-Temperatursensor (Kanal3) 
 */
 #define SENSOR_TH_HEADSINK		0x03

/**
 * @def		SENSOR_POWER_VOLTAGE
 * @brief	Relevante ADC-Kan�le f�r die Boardspannung 
 *			Kanal f�r die Boardspannung 
 */
 #define SENSOR_POWER_VOLTAGE		0x02


/**
 * @def		ADC_VREF_TYPE_1_10V
 * @brief	Referenzspannungen f�r den ADC
 * @note	ben�tigt f�r den internen Temperatursensor
 */
 #define ADC_VREF_TYPE_1_10V  	0xC0

/**
 * @def		ADC_VREF_TYPE_2_56V
 * @brief	Referenzspannungen f�r den ADC
 * @note	wird nicht ben�tigt
 */
 #define ADC_VREF_AREF			0x00

/**
 * @def		ADC_VREF_TYPE_VCC
 * @brief	Referenzspannungen f�r den ADC
 * @note	ben�tigt f�r den ext. LED-Thermistor
 */
 #define ADC_VREF_AVCC			0x40



/**
 * @struct	ADC_CTRL
 * @brief	ADC_CTRL
 *
 * @note	
 */
 typedef struct
 {
 /**
  * @var	unsigned int	conv_result
  * @brief	 
  */
  unsigned int		conv_result;

 /**
  * @var	unsigned char 	cannel
  * @brief	 
  */
  unsigned char 	cannel;

 /**
  * @var	unsigned char 	flags
  * @brief	 
  */
  unsigned char 	flags;
 }ADC_CTRL;



//! Prototypes

 void init_ADC(void);

 signed char get_Temperature(unsigned char channel, unsigned int rth_vor);

 unsigned int get_VBusVoltage(unsigned char channel, unsigned int r1, unsigned int r2, unsigned int v_ref);


#endif

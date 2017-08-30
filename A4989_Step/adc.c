/**
 * @file	adc.c
 * @author	René Schönrock
 * @brief	Analog zu Digitalwandlung der enstpr. Temperatursensoren. und der Boardspannung
 *
 * @date	16.04.2007
 * @see		Datenblatt des ATtiny25/45 von Atmel
 *	
 * Die ADC-Einheit wird benutzt, um die Analogsignale vom Thermistor zu digitalisieren.
 * Weiterhin wird in diesem Modul die Boardspannung gewandelt.
 * 
 * verwendete Resourcen:
 *	- ADC-Kanäle 2,3
 *	- Port PB3
 */

 #include <stdio.h>
 #include <avr/io.h>
 #include <avr/eeprom.h>
 #include <avr/interrupt.h>
 #include <avr/pgmspace.h>
 #include <math.h>

 //#include "fco_ctrl.h"
 
 #include "thermistor.h"

 #include "adc.h"

/**
 * @var		unsigned char cur_adc_channel
 * @brief	
 * @see		
 * @see		
 *	
 */
 unsigned char cur_adc_channel;


/**
 * @var		ADC_CTRL adc_ctrl]
 * @brief	Array
 * @see		
 * @see		
 *	
 */
 ADC_CTRL adc_ctrl[ADC_MAX_CANNEL] = {
 	{
		0x0000,
		(SENSOR_TH_HEADSINK | ADC_VREF_AVCC),
		0,
	},
 	{
		0x0000,
		(SENSOR_POWER_VOLTAGE | ADC_VREF_AVCC),
		0,
	},
 };


/**
 * @fn		void init_ADC(void)
 * @brief	Initialisierung
 *
 * Ausführliche Beschreibung
 *
 * @param       keine 
 *
 * @return		keine	
 *
 * @note	
 */
 void init_ADC(void)
 {
	unsigned char j;

	ADMUX  = 0;
	ADCSRA = (1<<ADEN) | (1<<ADIE) | 0x07;
	//ADCSRB = 0;//(1<<ADTS2);
	//DIDR0  =  (TH_HEADSINK_PIN_MASK | POWER_VOLTAGE_PIN_MASK);
	cur_adc_channel = 0;
	ADMUX = adc_ctrl[cur_adc_channel].cannel;
	ADCSRA |=  (1<<ADSC);
	//ADCSRA |=  (1<<ADATE);

	for (j=0; j<MAX_SAMPLING_POINTS; j++)
		rh_thermistor[j] = pgm_read_dword_near(rh_thermistor_flash+j);
 }




/** 
 * @fn          signed char get_Temperature(void)
 * @brief       Lesen der Thermistor-Temperaturen.
 *
 * Ausführliche Beschreibung.
 *
 * @param       channel ADC-Kanal des zu wandelnden Thermistors
 * @param		rth_vor Vorwiderstandswert des entspr. Thermistors 
 * @return      Temperaturwert als signed char
 * @note        Die Temperaturwertermittlung wird duch das Kennlinienfeld rh_thermistor gestützt.
 *				Zwischen den einzelnen Schtützstellen wird interpoliert.
 * @note        Temperaturbereich: -30°C..+125°C
 * @note		Auflösung: 1°C Genauigkeit
 * @see			thermistor.h         
 * @see			rh_thermistor         
 * @see			rh_thermistor_flash         
 * @see			REG#rth_vor         
 */
signed char get_Temperature(unsigned char channel, unsigned int rth_vor)
{
  unsigned char i, j;
  unsigned long r_th;
  unsigned long sp1, sp2;
  unsigned char x=0;


	//res_pre = eeprom_read_word(&r_vor);//const uint16_t *addr
	for (i=0; i<ADC_MAX_CANNEL; i++) {


		//cli();
		if ( ((adc_ctrl[i].cannel & 0x0F) == channel) && adc_ctrl[i].flags ) {
			adc_ctrl[i].flags = 0;

			//mit dem gewandelten Analogwert wird als erstes der Widerstand des Thermistors berechnet
			//Achtung: Gleitkommmaarithmetik (Faktor 1000) (unsigned long) muss verwendet werden um die entspr Genauigkeit zu erreichen
			//		 (erzeugt eine Menge Programmcode)
			//r_th = (unsigned long) (((unsigned long)reg.rth_vor*1000)/((1023000UL/read_adc(LED_SENSOR_CHANNEL, ADC_VREF_AVCC)) - 1000));
			//r_th = (unsigned long) (((unsigned long)rth_vor*1000)/((1023000UL/adc_ctrl[i].conv_result) - 1000));
			r_th = (unsigned long) ((((unsigned long)rth_vor)*((1023000UL/adc_ctrl[i].conv_result) - 1000))/1000);

			//sei();


			//dann wird aus dem Kennlinienfeld rh_thermistor[] die passenden Schtützstellen ermittlet
			//und eine Interpolation vorgenommen

			for (j=0; j<MAX_SAMPLING_POINTS; j++) {

				//sp1 = pgm_read_dword_near(rh_thermistor+j);
				sp1 = rh_thermistor[j];
				if (r_th >= sp1) {
					if (j) {
						//sp2 = pgm_read_dword_near(rh_thermistor+(j-1));
						sp2 = rh_thermistor[j-1];
						x = (unsigned char)(((50 * (r_th - sp1))/(sp2 - sp1) + 5)/10);
					}
					break;
				}
			}

			return (-30 + (j*5) - x);
		}
	}

	//sei();
	return 0;
}



/** 
 * @fn          unsigned int get_VBusVoltage(unsigned char channel, unsigned int r1, unsigned int r2)
 * @brief       Lesen der Boardspannung.
 *
 * Ausführliche Beschreibung.
 *
 * @param       channel ADC-Kanal des zu wandelnden Eingangs
 * @param		 
 * @param		
 * @return      Spannungswert als signed char
 * @note        Spannungsbereich: 
 * @note		Auflösung: 
 * @see			REG#         
 */
unsigned int get_VBusVoltage(unsigned char channel, unsigned int r1, unsigned int r2, unsigned int v_ref)
{
	unsigned char i;
	unsigned int voltage;
	//unsigned char x=0;


	//res_pre = eeprom_read_word(&r_vor);//const uint16_t *addr
	for (i=0; i<ADC_MAX_CANNEL; i++) {

		if ( ((adc_ctrl[i].cannel & 0x0F) == channel) && adc_ctrl[i].flags ) {
			adc_ctrl[i].flags = 0;

			voltage = (unsigned int) (     ((((unsigned long)adc_ctrl[i].conv_result * v_ref)/1024) * (r1 + r2)) / r1 );

			return voltage;
		}
	}

	return 0;
}


/************************************************************************
 * @fn		ISR(ADC_vect)
 * @brief	ADC-Conversion Completed Interrupt
 *
 * Ausführliche Beschreibung
 *
 * @param   keine 
 * @return	keine
 */
ISR(ADC_vect)
{
	adc_ctrl[cur_adc_channel].conv_result = (unsigned int)(ADCL | (ADCH<<8));
	adc_ctrl[cur_adc_channel].flags |= 0x01;

	if (cur_adc_channel++ >= (ADC_MAX_CANNEL - 1))
		cur_adc_channel=0;
			
	ADMUX = adc_ctrl[cur_adc_channel].cannel;

	ADCSRA |=  (1<<ADSC);
		
	return;
}

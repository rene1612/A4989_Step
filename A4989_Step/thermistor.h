/**
 * @file	thermistor.h
 * @author	René Schönrock
 * @brief	Kennlinienfeld des verwendeten Thermistors.
 *
 *
 * @date	16.04.2007
 * @see		Datenblatt des entspr. Thermistors
 */




/**
 * @def		MAX_SAMPLING_POINTS
 * @brief	Anzahl der Stützstellen für das Kennlinienfeld des LED-Thermistors
 * @see		rh_thermistor
 */
 #define MAX_SAMPLING_POINTS	32



/**
 * @var		unsigned long int rh_thermistor_flash PROGMEM
 * @brief	Kennlinienfeld für den LED-Thermistor
 * @note	Widerstandswerte sind in Ohm angegeben
 * @note	Die Tabelle steht im Programmspeicher
 * @note	Wenn ein anderer Thermistor zum Einsatz käme, müsste diese Tabelle angepasst werden.
 * @see		REG#rth_vor		
 * @see		MAX_SAMPLING_POINTS		
 */
 const unsigned long int rh_thermistor_flash[MAX_SAMPLING_POINTS] PROGMEM =
  {
	113347,	//-30°C
	87558,	//-25°C
	68236,	//-20°C
	53649,	//-15°C
	42506,	//-10°C
	33892,	// -5°C
	27218,	//  0°C
	22021,	//  5°C
	17925,	// 10°C
	14673,	// 15°C
	12080,	// 20°C
	10000,	// 25°C (R0)
	8314,	// 30°C
	6947,	// 35°C
	5833,	// 40°C
	4916,	// 45°C
	4160,	// 50°C
	3535,	// 55°C
	3014,	// 60°C
	2586,	// 65°C
	2227,	// 70°C
	1924,	// 75°C
	1668,	// 80°C
	1452,	// 85°C
	1268,	// 90°C
	1109,	// 95°C
	973,	//100°C
	858,	//105°C
	758,	//110°C
	671,	//115°C
	596,	//120°C
	531		//125°C
  };


/**
 * @var		unsigned int rh_thermistor_flash PROGMEM
 * @brief	Kennlinienfeld für den LED-Thermistor
 * @note	Widerstandswerte sind in Ohm angegeben
 * @note	Die Tabelle steht im Programmspeicher
 * @note	Wenn ein anderer Thermistor zum Einsatz käme, müsste diese Tabelle angepasst werden.
 * @see		REG#rth_vor		
 * @see		MAX_SAMPLING_POINTS		
 */
 /*
 unsigned int rh_thermistor_flash[MAX_SAMPLING_POINTS] PROGMEM =
  {
	11335,	//-30°C
	8756,	//-25°C
	6824,	//-20°C
	5365,	//-15°C
	4251,	//-10°C
	3389,	// -5°C
	2722,	//  0°C
	2202,	//  5°C
	1793,	// 10°C
	1467,	// 15°C
	1208,	// 20°C
	1000,	// 25°C (R0)
	831,	// 30°C
	694,	// 35°C
	583,	// 40°C
	492,	// 45°C
	416,	// 50°C
	354,	// 55°C
	301,	// 60°C
	259,	// 65°C
	223,	// 70°C
	192,	// 75°C
	167,	// 80°C
	145,	// 85°C
	127,	// 90°C
	111,	// 95°C
	97,		//100°C
	86,		//105°C
	76,		//110°C
	67,		//115°C
	60,		//120°C
	53		//125°C
  };
*/


/**
 * @var		unsigned long int rh_thermistor
 * @brief	Kennlinienfeld für den LED-Thermistor
 * @note	Widerstandswerte sind in Ohm angegeben
 * @note	Die Tabelle zum Programmstart in den RAM kopiert
 * @note	Wenn ein anderer Thermistor zum Einsatz käme, müsste diese Tabelle angepasst werden.
 * @see		REG#rth_vor		
 * @see		MAX_SAMPLING_POINTS		
 * @see		rh_thermistor_flash		
 */
 unsigned long int rh_thermistor[MAX_SAMPLING_POINTS];

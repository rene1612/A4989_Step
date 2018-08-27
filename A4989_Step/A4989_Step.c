/************************************************************************
 * @file	A4989_Step.c
 * @brief	Firmware für den Controller 
 *
 * @author	René Schönrock
 *
 *
 * @date	15.10.2010
 * @version	1.0
 * @bug		none.
 * @warning none.
 * @see		Datenblatt des  von Atmel
 *
 * @note	
 *
 * @todo
 *	
 * Details.
 */

 #include <stdio.h>
 #include <avr/io.h>
 #include <string.h>
 #include <avr/eeprom.h>
 #include <avr/interrupt.h>
 #include <util/delay.h>
 #include <avr/wdt.h>

 #include "types.h"
 #include "config.h"
// #include "prot.h"
 #include "uart.h"
 #include "twi_master.h"
 #include "pwm.h"
 #include "adc.h"
 #include "state.h"
 #include "pcf8575.h"

#ifdef __MODBUS__
	#include "mb.h"
	#include "mbport.h"
	#include "mbconfig.h"

	/* ----------------------- Defines ------------------------------------------*/
	#define REG_INPUT_START 1000
	#define REG_INPUT_NREGS 4

	#define REG_HOLDING_START 2000
	#define REG_HOLDING_NREGS 4

	#define COILS_START 0
	#define COIL_CNT 1

	/* ----------------------- Static variables ---------------------------------*/
	static USHORT   usRegInputStart = REG_INPUT_START;
	static USHORT   usRegInputBuf[REG_INPUT_NREGS];
	static USHORT   usRegHoldingBuf[REG_HOLDING_NREGS]={0x0001,DEFAULT_RTH_VOR,0,0};
#endif

 #ifdef __SNAP__
	#include "snap.h"
 #endif


 #include "A4989_Step.h"


/**
 * @fn		void (*jump_to_app)(void) = __APPLICATION_START_ADDR__
 * @brief	Einsprungspunkt für die Applikation
 */
 void (*jump_to_app)(void) = __APPLICATION_START_ADDR__;

 /**
 * @fn		void (*jump_to_bootloader)(void) = __APPLICATION_START_ADDR__
 * @brief	Einsprungspunkt für den Bootloader
 */
void (*jump_to_bootloader)(void) = __BOOTLOADER_START_ADDR__;



/**
 * @var		
 * @brief	Registersatz im EEProm
 * @see		REG
 * @see		reg
 *	
 */
 unsigned char dev_addr EEMEM = DEFAULT_DEV_ADDR;
 unsigned char dev_addr_backup EEMEM = DEFAULT_DEV_ADDR;


/**
 * @var		main_ee_regs
 * @brief	Registersatz im EEProm
 * @see		REG
 * @see		reg
 *	
 */
MAIN_REGS main_ee_regs EEMEM =
{
	//!<RW CTRL Ein-/Ausschalten usw.  (1 BYTE )
	(0<<REG_CTRL_BOOT) | \
	(1<<REG_CTRL_AUTO_CURRENT) | \
	(1<<REG_CTRL_EE_DEFAULTS),

	SYS_OK,
	STATE_OFF,
	
	(0<<REG_SCAN_VBUS_LT) | (1<<REG_SCAN_VBUS_UT) | (1<<REG_SCAN_TEMP),
	
	0x0FFF,
	
	DEFAULT_MAX_HEATSINK_TEMP,
	DEFAULT_UPPER_BV_THRESHOLD,
	DEFAULT_LOWER_BV_THRESHOLD,
	
	DEFAULT_STANDBY_CURRENT_TIMEOUT,
	
	0x0000,
	0x0000,
	0x00,
	0x0000,
	0,
	
	APP_UART_BAUD_RATE,
	
	//ab rel. 0x10 steht die Gerätesignatur und die Softwareversion
	__DEV_SIGNATURE__,
	__SW_RELEASE__,
	__SW_RELEASE_DATE__,

};


/**
 * @var		main_regs
 * @brief	Registersatz im Ram (Arbeitsregister)
 * @see		MAIN_REGS
 * @see		main_ee_regs
 *
 */
MAIN_REGS main_regs;


SWITCH_PORT pcf8575;

uint8_t main_task_scheduler;
//uint8_t main_timer=0;
//uint8_t timer0_ov_ticks=0;
//signed char cur_heatsink_temperature;
//unsigned int cur_vbus_volage;
unsigned long step_counter;

uint8_t standby_timer;
uint16_t standby_timout;



/************************************************************************
 * @fn		ISR(INT0_vect)
 * @brief	TWI Interrupt Handler
 *
 * This function is the Interrupt Service Routine (ISR), and called when
 * the STEP_ENABLE interrupt is triggered;
 *
 * @note    
 *		This function should not be called directly from the main application.
 */
ISR(INT0_vect)
{
	main_task_scheduler |= PROCESS_STEP_ENABLE;
	return;
}

/************************************************************************
 * @fn		ISR(INT2_vect)
 * @brief	TWI Interrupt Handler
 *
 * This function is the Interrupt Service Routine (ISR), and called when
 * the STEP_ENABLE interrupt is triggered;
 *
 * @note    
 *		This function should not be called directly from the main application.
 */
ISR(INT2_vect)
{
	main_regs.step_dir = STEP_DIR_INPUT;
	return;
}


/************************************************************************
 * @fn		ISR(SIG_OVERFLOW1)
 * @brief	ISR für step-pulse erkennung (automatische stromabsenkung)
 *
 * Ausführliche Beschreibung
 *
 * @param   keine 
 * @return	keine
 */
ISR(TIMER0_COMP_vect)
{
	if (main_regs.sys_state == SYS_ACTIVE_SBC) {
		OCR0 = 0xFF;
		standby_timer=TCNT0;
		TIMSK |= _BV(TOIE1);
		set_sys_state (SYS_ACTIVE_FC);
		return;
	}
	
	step_counter += 0xFF;
}

/************************************************************************
 * @fn		ISR(SIG_OVERFLOW1)
 * @brief	ISR für step-pulse erkennung (automatische stromabsenkung)
 *
 * Ausführliche Beschreibung
 *
 * @param   keine 
 * @return	keine
 */

ISR(TIMER1_OVF_vect)
{
	if (TCNT0 == standby_timer) {
		if (++standby_timout > main_regs.standby_timeout) {
			TIMSK &= ~_BV(TOIE1);
			set_sys_state (SYS_ACTIVE_SBC);
		}
	}
	else {
		standby_timer=TCNT0;
		standby_timout=0;
	}
}





/************************************************************************
 * @fn		void init_Sys(void)
 * @brief	Systeminitialisierung
 *
 * Ausführliche Beschreibung
 *
 * @param   keine 
 * @return	keine
 */
void init_Sys(void)
{
	unsigned int ui_baudrate;
#ifdef __MODBUS__
    const UCHAR     ucSlaveID[] = { 0xAA, 0xBB, 0xCC };
    eMBErrorCode    eStatus;
#endif
    
    //CLKPR = (1 << CLKPCE); // enable a change to CLKPR
    //CLKPR = 0; // set the CLKDIV to 0 - was 0011b = div by 8 taking 8MHz to 1MHz

    
#ifdef __USE_WDT__
	//Watchdogtimer aktivieren  	WDTO_250MS
	//wdt_enable(WDTO_60MS);
	wdt_enable(WDTO_500MS);
	wdt_reset();
#else
	wdt_disable();
#endif

#if defined(__AVR_ATmega32__) || defined (__AVR_ATmega16__)
	//TCCR0 = ((0<<COM01) | (0<<COM00) | (0<<WGM01) | (0<<WGM00) | (1<<CS02) | (1<<CS01) | (0<<CS00));
	TIMSK |= (_BV(OCIE0));
	
	STEP_ENA_INT_DIR &= ~_BV(STEP_ENA_INPUT_PIN);	//Interupt-PIn auf Eingang
	MCUCR &= ~((1<<ISC01) | (1<<ISC00));	//falling edge of INT1
	MCUCR |= ((0<<ISC01) | (1<<ISC00));	//falling edge of INT1
	GICR |= _BV(INT0) | _BV(INT2);						//INT0 & INT2 activate
	TCCR2 = ((0<<COM21) | (0<<COM20) | (0<<WGM21) | (0<<WGM20) | (1<<CS22) | (0<<CS21) | (1<<CS20));

#else
	#error Wrong AVR!
#endif

	DDRA=(_BV(A4989_EN) | _BV(A4989_SR));
	DDRB=(_BV(A4989_PFD2) | _BV(A4989_PFD1) | _BV(A4989_MS2) | _BV(A4989_MS1) | _BV(A4989_RESET));
	A4989_DISABLE;
	A4989_SR_ENABLE;	//Synchronus Rectification always on		
	A4989_RESET_ENABLE;
		
	if (eeprom_read_byte(&main_ee_regs.ctrl) & (1<<REG_CTRL_EE_DEFAULTS)) {
		//Registerwerte aus dem EEProm restaurieren
		eeprom_read_block ((void*)&main_regs, (const void*)&main_ee_regs, sizeof(MAIN_REGS));
	}
	
	main_regs.step_dir = STEP_DIR_INPUT;
	
	/* Einstellen der Baudrate */
	//uart_init( UART_BAUD_SELECT_DOUBLE_SPEED(APP_UART_BAUD_RATE, F_CPU) );
	if(main_regs.baudrate>MAX_BAUDRATE){
		ui_baudrate=UART_BAUD_SELECT(APP_UART_BAUD_RATE, F_CPU);
		main_regs.baudrate=APP_UART_BAUD_RATE;
	}
	
	if(main_regs.baudrate<MIN_BAUDRATE){
		ui_baudrate=UART_BAUD_SELECT(APP_UART_BAUD_RATE, F_CPU);
		main_regs.baudrate=APP_UART_BAUD_RATE;
	}

	else{
		if (main_regs.baudrate>=57600)
			ui_baudrate=UART_BAUD_SELECT_DOUBLE_SPEED(main_regs.baudrate, F_CPU);
		else
			ui_baudrate=UART_BAUD_SELECT(main_regs.baudrate, F_CPU);
	}

	//Interrupts anwerfen
	sei();

	init_ADC();
	

	TWI_Master_Initialise();
	
	pcf8575_init ();
	pcf8575.word = 0xFFFF;
	
	init_DAC(3);
	
	init_StateMon();
	
#ifdef __MODBUS__
	unsigned char nodeAddress1=eeprom_read_byte((uint8_t *)0x00);
	unsigned char nodeAddress2=eeprom_read_byte((uint8_t *)0x01);
	if(nodeAddress1!=nodeAddress2 || nodeAddress1==0 || nodeAddress1==0xFF)
	nodeAddress1=BACKUP_DEV_ADDR;
	eStatus = eMBInit( MB_RTU, nodeAddress1, 0, main_regs.baudrate, MB_PAR_EVEN );
	eStatus = eMBSetSlaveID( 0x34, TRUE, ucSlaveID, 3 );
	/* Enable the Modbus Protocol Stack. */
	eStatus = eMBEnable(  );
#endif

#ifdef __SNAP__
	uart_init( ui_baudrate );
	unsigned char nodeAddress1=eeprom_read_byte((uint8_t *)0x00);
	unsigned char nodeAddress2=eeprom_read_byte((uint8_t *)0x01);
	if(nodeAddress1!=nodeAddress2 || nodeAddress1==0 || nodeAddress1==0xFF)
		nodeAddress1=BACKUP_DEV_ADDR;
		
	SnapInit(nodeAddress1);
#endif

	main_task_scheduler = 0;

}


/************************************************************************
 * @fn		int set_sys_state (void)
 * @brief	
 *
 * Ausführliche Beschreibung
 *
 * @param   enum SYS_STATE sys_state
 * @return	
 */
void set_sys_state (enum SYS_STATE sys_state) {
	uint8_t temp_port;
	
	switch (sys_state) {

		case SYS_OK:
		case SYS_ACTIVE_STOP:
			A4989_RESET_ENABLE;
			A4989_DISABLE;
			//DAC für Stromeistellung updaten
			set_DAC((unsigned int)0x0000);
			main_regs.sys_state = SYS_OK;
			set_StateMon(STATE_OFF);
			break;

		case SYS_ACTIVE_START:
			//Portpins setzen
			temp_port = PINB & 0x0F;
			temp_port |= ((encoder.val.microstep_decay & 0x03)<<A4989_MS1);
			temp_port |= ((encoder.val.microstep_decay & 0x08)<<(A4989_PFD2-3));
			temp_port |= ((encoder.val.microstep_decay & 0x04)<<(A4989_PFD1-2));
			PORTB = temp_port;
			A4989_RESET_DISABLE;
			A4989_ENABLE;
		
		case SYS_ACTIVE_SBC:
			set_DAC(main_regs.standby_current);
			main_regs.sys_state = SYS_ACTIVE_SBC;
			set_StateMon(STATE_ON_SBC);

			if ( main_regs.ctrl & _BV(REG_CTRL_AUTO_CURRENT) ) {
				step_counter = 0;
				standby_timout = 0;
				TCNT0 = 0;
				OCR0 = 1;
				TCCR0 = ((0<<COM01) | (0<<COM00) | (0<<WGM01) | (0<<WGM00) | (1<<CS02) | (1<<CS01) | (1<<CS00));
				break;
			}

		case SYS_ACTIVE_FC:
			set_DAC(main_regs.full_current);
			main_regs.sys_state = SYS_ACTIVE_FC;
			set_StateMon(STATE_ON_FC);
			break;
		
		case SYS_ERROR:
			A4989_RESET_ENABLE;
			set_DAC((unsigned int)0x0000);
			main_regs.sys_state = SYS_ERROR;
			break;
	}
}

/************************************************************************
 * @fn		int main (void)
 * @brief	Hauptprogramm-Schleife
 *
 * Ausführliche Beschreibung
 *
 * @param   keine 
 * @return	Type int (never returns)
 */
int main (void) {
	
	static unsigned char main_tmp_ctrl_reg=0x00;
	
	//Systeminitialisierug
	init_Sys();


	//Endlosschleife 
	while(1) {

#ifdef __USE_WDT__
		//Watchdog zurücksetzen
		wdt_reset();
#endif


#ifdef __SNAP__
		if(SnapPacketReceived()){
			SnapDecodeReceived((unsigned char *)& main_regs.ctrl);
		}
#endif

#ifdef __MODBUS__
        ( void )eMBPoll(  );
#endif

		//mal schauen, ob sich im CTRL-Register was geändert hat
		if( main_regs.ctrl^main_tmp_ctrl_reg ) {

			if( main_regs.ctrl & (1<<REG_CTRL_WR_EEREGS) ) { // EEProm-Register schreiben ?
			 
				//extrem Wichtig!!! sonst eventuell Loop mit massiven EEProm-Schreibaktionen
				main_regs.ctrl &= ~((1<<REG_CTRL_WR_EEREGS) | (1<<REG_CTRL_RESET)); 

				//Registerwerte in dem EEProm schreiben
				eeprom_update_block ((const void*)&main_regs, (void*)&main_ee_regs, sizeof(MAIN_REGS) ); //
			}

			if( main_regs.ctrl & (1<<REG_CTRL_RESET) ) { // Reset des Controllers auslösen ?
				_delay_ms(15);
				jump_to_app();
   			}

			if( main_regs.ctrl & (1<<REG_CTRL_BOOT) ) {
				main_regs.ctrl &= ~(1<<REG_CTRL_BOOT);
				
				wdt_enable(WDTO_15MS);
				while(1){};

				//jump_to_bootloader();
			}

			if( main_regs.ctrl & (1<<REG_CTRL_ERROR_RESET) ) {
				main_regs.ctrl &= ~(1<<REG_CTRL_ERROR_RESET);

				set_sys_state (SYS_OK);
			}

			if( main_regs.ctrl & (1<<REG_CTRL_ACTIVATE) ) {
				main_regs.ctrl &= ~(1<<REG_CTRL_ACTIVATE);

				set_sys_state (SYS_ACTIVE_START);
			}
			
			if( main_regs.ctrl & (1<<REG_CTRL_DEACTIVATE) ) {
				main_regs.ctrl &= ~(1<<REG_CTRL_DEACTIVATE);

				set_sys_state (SYS_ACTIVE_STOP);
			}
			   
			main_tmp_ctrl_reg = main_regs.ctrl;
		}
		
		
		 main_regs.cur_heatsink_temperature = get_Temperature(SENSOR_TH_HEADSINK, DEFAULT_RTH_VOR);
		 if ( (main_regs.scan_mask & (1<<REG_SCAN_TEMP)) && main_regs.cur_heatsink_temperature > main_regs.max_heatsink_temp) {
			if (main_regs.sys_state != SYS_ERROR) {
				set_sys_state (SYS_ERROR);
				set_StateMon(STATE_ERR_HEADSINK_TEMP);
			}
		 }
		 
		 main_regs.cur_vbus_volage = get_VBusVoltage(SENSOR_POWER_VOLTAGE, DEFAULT_R1, DEFAULT_R2, DEFAULT_VREF);
		 if ( main_regs.cur_vbus_volage)
		 {
			 if ((main_regs.scan_mask & (1<<REG_SCAN_VBUS_LT)) && main_regs.cur_vbus_volage < main_regs.lower_bv_threshold) {
				if (main_regs.sys_state != SYS_ERROR ) {
					set_sys_state (SYS_ERROR);
					set_StateMon(STATE_ERR_VBUS_VOLTAGE_LT);
				}
			 }

			 if ((main_regs.scan_mask & (1<<REG_SCAN_VBUS_UT)) && main_regs.cur_vbus_volage > main_regs.upper_bv_threshold) {
				if (main_regs.sys_state != SYS_ERROR ) {
					set_sys_state (SYS_ERROR);
					set_StateMon(STATE_ERR_VBUS_VOLTAGE_UT);
				}
			 }
		 }
		 
		 
		 if (main_task_scheduler & PROCESS_PCF8575){
			 if (process_pcf8575())
				main_task_scheduler &= ~PROCESS_PCF8575;
		 }
		 
		 if (main_task_scheduler & PROCESS_MONITOR_STATE){
			 if (process_State())
				main_task_scheduler &= ~PROCESS_MONITOR_STATE;
		 }

		 if (main_task_scheduler & PROCESS_STEP_ENABLE){
			 
			if (main_regs.sys_state != SYS_ERROR) {
				 if (!STEP_ENA_INPUT)
				 {
					set_sys_state (SYS_ACTIVE_START);
					//set_StateMon(STATE_ON);
				 }else
				 {
					set_sys_state (SYS_ACTIVE_STOP);
					//set_StateMon(STATE_OFF);
				 }
			}
			 
			main_task_scheduler &= ~PROCESS_STEP_ENABLE;
		 }

	};

	return 0;
}


#ifdef __MODBUS__
eMBErrorCode
eMBRegInputCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs )
{
	eMBErrorCode    eStatus = MB_ENOERR;
	int             iRegIndex;

	if( ( usAddress >= REG_INPUT_START )
	&& ( usAddress + usNRegs <= REG_INPUT_START + REG_INPUT_NREGS ) )
	{
		
		iRegIndex = ( int )( usAddress - usRegInputStart );
		while( usNRegs > 0 )
		{
			*pucRegBuffer++ =
			( unsigned char )( usRegInputBuf[iRegIndex] >> 8 );
			*pucRegBuffer++ =
			( unsigned char )( usRegInputBuf[iRegIndex] & 0xFF );
			iRegIndex++;
			usNRegs--;
		}
	}
	else
	{
		eStatus = MB_ENOREG;
	}

	return eStatus;
}

eMBErrorCode
eMBRegHoldingCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs,
eMBRegisterMode eMode )
{
	eMBErrorCode    eStatus = MB_ENOERR;
	int             iRegIndex;

	if( ( usAddress >= REG_HOLDING_START )
	&& ( usAddress + usNRegs <= REG_HOLDING_START + REG_HOLDING_NREGS ) )
	{
		iRegIndex = ( int )( usAddress - REG_HOLDING_START );
		while( usNRegs > 0 )
		{
			switch(eMode) {
				case MB_REG_WRITE:
				usRegHoldingBuf[iRegIndex] = *pucRegBuffer++;
				usRegHoldingBuf[iRegIndex] <<= 8;
				usRegHoldingBuf[iRegIndex] += *pucRegBuffer++;
				break;
				case MB_REG_READ:
				default:
				*pucRegBuffer++ =
				( unsigned char )( usRegHoldingBuf[iRegIndex] >> 8 );
				*pucRegBuffer++ =
				( unsigned char )( usRegHoldingBuf[iRegIndex] & 0xFF );
				break;
			}
			iRegIndex++;
			usNRegs--;
		}
	}
	else
	{
		eStatus = MB_ENOREG;
	}

	return eStatus;
}

/*
eMBErrorCode
eMBRegCoilsCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNCoils,
eMBRegisterMode eMode )
{
	eMBErrorCode    eStatus = MB_ENOERR;
	int             iRegIndex;
	USHORT			coil_addr;
	USHORT			cur_coil;
	
	coil_addr=usAddress-1;
	
	if( ( coil_addr >= COILS_START )
	&& ( coil_addr + usNCoils <= COILS_START + COIL_CNT ) )
	{
		iRegIndex = 0;
		cur_coil=0;
		while( usNCoils > 0 )
		{
			switch(eMode) {
				case MB_REG_WRITE:
				
				if (pucRegBuffer[iRegIndex]&_BV(cur_coil))
					*(CoilList[coil_addr].pPort) |= CoilList[coil_addr].mask;
				else
					*(CoilList[coil_addr].pPort) &= ~CoilList[coil_addr].mask;
					
				break;
				
				case MB_REG_READ:
				default:
				
				if(*(CoilList[coil_addr].pPin)&CoilList[coil_addr].mask)
					pucRegBuffer[iRegIndex] |= _BV(cur_coil);
				else
					pucRegBuffer[iRegIndex] &= ~_BV(cur_coil);
				
				break;
			}
			
			coil_addr++;
			cur_coil++;
			if(cur_coil>=8){
				iRegIndex++;
				cur_coil=0;
			}
			usNCoils--;
		}
	}
	else
	{
		eStatus = MB_ENOREG;
	}
	return eStatus;
}

eMBErrorCode
eMBRegDiscreteCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNDiscrete )
{
	return MB_ENOREG;
}
*/
#endif

/*
 * sp5KV3_utils.c
 *
 *  Created on: 27/10/2015
 *      Author: pablo
 */

#include "sp5KV5_8CH.h"

static uint8_t pv_paramLoad(uint8_t* data, uint8_t* addr, uint16_t sizebytes);
static uint8_t pv_paramStore(uint8_t* data, uint8_t* addr, uint16_t sizebytes);
static uint8_t pv_checkSum ( uint8_t *data,uint16_t sizebytes );

void u_uarts_ctl(uint8_t cmd)
{

static bool terminal_prendida = false;
static bool modem_prendido = false;

	switch(cmd) {

	case MODEM_PRENDER:
		// Si tengo el LM365 deshabilitado lo habilito
		if ( ! modem_prendido ) {
			cbi(UARTCTL_PORT, UARTCTL);	// Habilito el LM365
			modem_prendido = true;
		}
		break;

	case TERM_PRENDER:
		if ( ! terminal_prendida ) {
			cbi(UARTCTL_PORT, UARTCTL);	// Habilito el LM365
			terminal_prendida = true;
		}
		break;

	case MODEM_APAGAR:
		modem_prendido = false;
		if ( ! terminal_prendida ) {
			sbi(UARTCTL_PORT, UARTCTL);	// Deshabilito el LM365
		}
		break;

	case TERM_APAGAR:
		terminal_prendida = false;
		if ( ! modem_prendido ) {
			sbi(UARTCTL_PORT, UARTCTL);	// Deshabilito el LM365
		}
		break;
		break;
	}

}
//----------------------------------------------------------------------------------------
void u_panic( uint8_t panicCode )
{
char msg[16];

	snprintf_P( msg,sizeof(msg),PSTR("\r\nPANIC(%d)\r\n\0"), panicCode);
	FreeRTOS_write( &pdUART1,  msg, sizeof( msg) );
	vTaskDelay( ( TickType_t)( 20 / portTICK_RATE_MS ) );
	vTaskSuspendAll ();
	vTaskEndScheduler ();
	exit (1);
}
//----------------------------------------------------------------------------------------
void u_kick_Wdg( uint8_t wdgId )
{
	// Pone el correspondiente bit del wdg en 0.
	systemWdg &= ~wdgId ;

}
//------------------------------------------------------------------------------------
bool u_saveSystemParams(void)
{
	// Salva el systemVars en la EE y verifica que halla quedado bien.
	// Hago hasta 3 intentos.

bool retS = false;
uint8_t storeChecksum = 0;
uint8_t loadChecksum = 0;
uint8_t i;

	while ( xSemaphoreTake( sem_SYSVars, ( TickType_t ) 1 ) != pdTRUE )
		taskYIELD();

	for ( i=0; i<3; i++ ) {
		storeChecksum = pv_paramStore( (uint8_t *)&systemVars, (uint8_t *)EEADDR_SV, sizeof(systemVarsType));
		vTaskDelay( ( TickType_t)( 100 / portTICK_RATE_MS ) );
		pv_paramLoad( (uint8_t *)&tmpSV, (uint8_t *)EEADDR_SV, sizeof(systemVarsType));
		loadChecksum = pv_checkSum( (uint8_t *)&tmpSV,sizeof(systemVarsType));

		if ( loadChecksum == storeChecksum ) {
			retS = true;
			break;
		}
	}

	xSemaphoreGive( sem_SYSVars );
	return(retS);

}
//------------------------------------------------------------------------------------
bool u_loadSystemParams(void)
{
bool retS = false;
uint8_t i;

	// Leo la configuracion:  Intento leer hasta 3 veces.

	for ( i=0; i<3;i++) {
		retS =  pv_paramLoad( (uint8_t *)&systemVars, (uint8_t *)EEADDR_SV, sizeof(systemVarsType));
		if ( retS )
			break;
	}

	// Ajustes de inicio:
	strncpy_P(systemVars.dlg_ip_address, PSTR("000.000.000.000\0"),16);
	systemVars.csq = 0;
	systemVars.dbm = 0;
	systemVars.debugLevel = D_BASIC + D_GPRS;
	systemVars.wrkMode = WK_NORMAL;

	// Cuando arranca si la EE no esta inicializada puede dar cualquier cosa.
	// De este modo controlo el largo de los strings.
	systemVars.dlgId[DLGID_LENGTH - 1] = '\0';
	systemVars.apn[APN_LENGTH - 1] = '\0';
	systemVars.server_tcp_port[PORT_LENGTH - 1] = '\0';
	systemVars.server_ip_address[IP_LENGTH - 1] = '\0';
	systemVars.dlg_ip_address[IP_LENGTH - 1] = '\0';
	systemVars.serverScript[SCRIPT_LENGTH - 1] = '\0';
	systemVars.passwd[PASSWD_LENGTH - 1] = '\0';

	// Nombre de los canales
	systemVars.aChName[0][PARAMNAME_LENGTH - 1] = '\0';
	systemVars.aChName[1][PARAMNAME_LENGTH - 1] = '\0';
	systemVars.aChName[2][PARAMNAME_LENGTH - 1] = '\0';

	systemVars.dChName[0][PARAMNAME_LENGTH - 1] = '\0';
	systemVars.dChName[1][PARAMNAME_LENGTH - 1] = '\0';

	return(retS);

}
//------------------------------------------------------------------------------------
void u_loadDefaults(void)
{

// Configura el systemVars con valores por defecto.

	while ( xSemaphoreTake( sem_SYSVars, ( TickType_t ) 1 ) != pdTRUE )
		taskYIELD();

	systemVars.initByte = 0x49;
	strncpy_P(systemVars.dlgId, PSTR("DEF400\0"),DLGID_LENGTH);
	strncpy_P(systemVars.server_tcp_port, PSTR("80\0"),PORT_LENGTH	);
	strncpy_P(systemVars.passwd, PSTR("spymovil123\0"),PASSWD_LENGTH);
	strncpy_P(systemVars.serverScript, PSTR("/cgi-bin/sp5K/sp5K8CH.pl\0"),SCRIPT_LENGTH);

	systemVars.csq = 0;
	systemVars.dbm = 0;
	systemVars.gsmBand = 8;
	systemVars.wrkMode = WK_NORMAL;

	strncpy_P(systemVars.apn, PSTR("SPYMOVIL.VPNANTEL\0"),APN_LENGTH);

	// DEBUG
	systemVars.debugLevel = D_BASIC;

	strncpy_P(systemVars.server_ip_address, PSTR("192.168.1.9\0"),IP_LENGTH);

	// Defaults de los canales analogicos y timerpoll.
	u_analog_load_defaults();

	// Canales digitales
	u_digital_load_defaults();

	xSemaphoreGive( sem_SYSVars );


}
//------------------------------------------------------------------------------------
char *u_now(void)
{

	// Devuelve un puntero a un string con la fecha y hora formateadas para usar en
	// los mensajes de log.

RtcTimeType_t rtcDateTime;

	RTC_read(&rtcDateTime);
	rtcDateTime.year -= 2000;
	snprintf_P( nowStr,sizeof(nowStr), PSTR("%02d/%02d/%02d %02d:%02d:%02d\0"),rtcDateTime.day,rtcDateTime.month,rtcDateTime.year,rtcDateTime.hour,rtcDateTime.min,rtcDateTime.sec );
	return(nowStr);
}
//------------------------------------------------------------------------------------
void u_debugPrint(uint8_t debugCode, char *msg, uint16_t size)
{

	if ( (systemVars.debugLevel & debugCode) != 0) {
		FreeRTOS_write( &pdUART1, msg, size );
	}
}
//------------------------------------------------------------------------------------
void u_reset(void)
{
	wdt_enable(WDTO_30MS);
	while(1) {}
}
//------------------------------------------------------------------------------------
// FUNCIONES PRIVADAS
//------------------------------------------------------------------------------------
static uint8_t pv_paramLoad(uint8_t* data, uint8_t* addr, uint16_t sizebytes)
{
uint16_t i;
uint8_t checksum_stored=0;
uint8_t checksum=0;

	// load parameters
	eeprom_read_block(data, (uint8_t *)addr, sizebytes);
	// load checksum
	eeprom_read_block(&checksum_stored, (uint8_t *)(addr+sizebytes), sizeof(uint8_t));

	// calculate own checksum
	for(i=0;i<sizebytes;i++)
		checksum += data[i];
	checksum = ~checksum;

	if(checksum == checksum_stored)
		return true;
	else
		return false;
}
//------------------------------------------------------------------------------------
static uint8_t pv_paramStore(uint8_t* data, uint8_t* addr, uint16_t sizebytes)
{
	// Almacena un string de bytes en la eeprom interna del micro

uint16_t i;
uint8_t checksum=0;

	// calculate checksum
	for(i=0;i<sizebytes;i++)
		checksum += data[i];
	checksum = ~checksum;

	// store parameters
	 eeprom_write_block(data, (uint8_t *)addr, sizebytes);
	// store checksum
	eeprom_write_block(&checksum, (uint8_t *)(addr+sizebytes), sizeof(uint8_t));

	return(checksum);
}
//------------------------------------------------------------------------------------
static uint8_t pv_checkSum ( uint8_t *data,uint16_t sizebytes )
{
uint16_t i;
uint8_t checksum=0;

	// calculate checksum
	for(i=0;i<sizebytes;i++)
		checksum += data[i];
	checksum = ~checksum;
	return(checksum);
}
//------------------------------------------------------------------------------------

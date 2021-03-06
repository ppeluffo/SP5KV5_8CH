/*
 * sp5KV5_tkGprs_init_frame.c
 *
 *  Created on: 27 de abr. de 2017
 *      Author: pablo
 */

#include "../sp5KV5_8CH_tkGPRS/sp5KV5_tkGprs.h"

bool pv_send_init_frame(void);
bool pv_process_init_response(void);
static void pv_TX_init_frame(void);
static void pv_reconfigure_params(void);

static void pv_process_server_clock(void);
static uint8_t pv_process_timerPoll(void);
static uint8_t pv_process_digitalCh(uint8_t channel);
static uint8_t pv_process_analogCh(uint8_t channel);

//------------------------------------------------------------------------------------
bool gprs_init_frame(void)
{
	// Debo mandar el frame de init al server, esperar la respuesta, analizarla
	// y reconfigurarme.
	// Intento 3 veces antes de darme por vencido.
	// El socket puede estar abierto o cerrado. Lo debo determinar en c/caso y
	// si esta cerrado abrirlo.
	// Mientras espero la respuesta debo monitorear que el socket no se cierre

uint8_t intentos;
bool exit_flag = false;

// Entry:

	GPRS_stateVars.state = G_INIT_FRAME;

	if ( (systemVars.debugLevel & (D_BASIC + D_GPRS) ) != 0) {
		snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR("%s GPRS::iniframe:\r\n\0"), u_now());
		FreeRTOS_write( &pdUART1, gprs_printfBuff, sizeof(gprs_printfBuff) );
	}

	// Intenteo MAX_INIT_TRYES procesar correctamente el INIT
	for ( intentos = 0; intentos < MAX_INIT_TRYES; intentos++ ) {

		if ( ! pv_send_init_frame() ) {			// Intento madar el frame al servidor
			if ( (systemVars.debugLevel & (D_BASIC + D_GPRS) ) != 0) {
				snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR("%s GPRS::iniframe: Send retry\r\n\0"), u_now() );
				FreeRTOS_write( &pdUART1, gprs_printfBuff, sizeof(gprs_printfBuff) );
			}
			continue;
		}

		if ( ! pv_process_init_response() ) {	// Intento procesar la respuesta
			if ( ( systemVars.debugLevel & (D_BASIC + D_GPRS) ) != 0) {
				snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR("%s GPRS::iniframe: Rsp retry\r\n\0"), u_now() );
				FreeRTOS_write( &pdUART1, gprs_printfBuff, sizeof(gprs_printfBuff) );
			}
			continue;
		}

		// Aqui es que anduvo todo bien y debo salir para pasar al modo DATA
		if ( (systemVars.debugLevel & (D_BASIC + D_GPRS) ) != 0) {
			snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR("%s GPRS::iniframe: Init frame OK.\r\n\0"), u_now() );
			FreeRTOS_write( &pdUART1, gprs_printfBuff, sizeof(gprs_printfBuff) );
		}

		exit_flag = true;
		goto EXIT;

	}

	// Aqui es que no puede enviar/procesar el INIT correctamente
	if ( (systemVars.debugLevel & (D_BASIC + D_GPRS) ) != 0) {
		snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR("%s GPRS::iniframe: Init frame FAIL !!.\r\n\0"), u_now() );
		FreeRTOS_write( &pdUART1, gprs_printfBuff, sizeof(gprs_printfBuff) );
	}

// Exit
EXIT:

	return(exit_flag);

}
//------------------------------------------------------------------------------------
// FUNCIONES PRIVADAS
//------------------------------------------------------------------------------------
bool pv_send_init_frame(void)
{
	// Intento enviar 1 SOLO frame de init.
	// El socket puede estar cerrado por lo que reintento abrirlo hasta 3 veces.
	// Una vez que envie el INIT, salgo.
	// Al entrar, veo que el socket este cerrado.

uint8_t i;
bool exit_flag = false;

	for ( i = 0; i < MAX_TRYES_OPEN_SOCKET; i++ ) {

		//g_sleep(5);			// Espero 5s
		if ( g_socket_is_open() ) {
			pv_TX_init_frame();		// Escribo en el socket el frame de INIT
			exit_flag = true;
			break;
		}

		g_open_socket();
	}

	return(exit_flag);
}
//------------------------------------------------------------------------------------
bool pv_process_init_response(void)
{
	// Espero la respuesta al frame de INIT.
	// Si la recibo la proceso.
	// Salgo por timeout 10s o por socket closed.

uint8_t timeout;
bool exit_flag = false;

	for ( timeout = 0; timeout < 10; timeout++) {

		g_sleep(1);				// Espero 1s

		if ( ! g_socket_is_open() ) {		// El socket se cerro
			exit_flag = false;
			goto EXIT;
		}

		if ( strstr( gprsRx.buffer, "ERROR") != NULL ) {	// Recibi un ERROR de respuesta
			g_printRxBuffer();
			exit_flag = false;
			goto EXIT;
		}

		if ( strstr( gprsRx.buffer, "INIT_OK") != NULL ) {	// Respuesta correcta del server
			g_printRxBuffer();
			pv_reconfigure_params();
			exit_flag = true;
			goto EXIT;
		}

	}

// Exit:
EXIT:

	return(exit_flag);

}
//------------------------------------------------------------------------------------
static void pv_TX_init_frame(void)
{
	// Send Init Frame
	// GET /cgi-bin/sp5K8CH.pl?DLGID=SPY001&PASSWD=spymovil123&&INIT&CSQ=75 HTTP/1.1
	// Host: www.spymovil.com
	// Connection: close\r\r ( no mando el close )

uint16_t pos = 0;
uint8_t i;

	if ( (systemVars.debugLevel &  D_GPRS ) != 0) {
		snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR("%s GPRS::iniframe: Sent\r\n\0"), u_now() );
		FreeRTOS_write( &pdUART1, gprs_printfBuff, sizeof(gprs_printfBuff) );
	}

	// Trasmision: 1r.Parte.
	// HEADER:
	// Envio parcial ( no CR )
	memset( gprs_printfBuff, '\0', sizeof(gprs_printfBuff));
	FreeRTOS_ioctl( &pdUART0,ioctl_UART_CLEAR_RX_BUFFER, NULL, false);
	FreeRTOS_ioctl( &pdUART0,ioctl_UART_CLEAR_TX_BUFFER, NULL, false);
	g_flushRXBuffer();

	pos = snprintf_P( gprs_printfBuff,CHAR256,PSTR("GET " ));
	pos += snprintf_P( &gprs_printfBuff[pos],( sizeof(gprs_printfBuff) - pos ),PSTR("%s"), systemVars.serverScript );
	pos += snprintf_P( &gprs_printfBuff[pos],( sizeof(gprs_printfBuff) - pos ),PSTR("?DLGID=%s"), systemVars.dlgId );
	pos += snprintf_P( &gprs_printfBuff[pos],( sizeof(gprs_printfBuff) - pos ),PSTR("&PASSWD=%s"), systemVars.passwd );
	pos += snprintf_P( &gprs_printfBuff[pos],( sizeof(gprs_printfBuff) - pos ),PSTR("&IMEI=%s"), g_getImei() );
	pos += snprintf_P( &gprs_printfBuff[pos],( sizeof(gprs_printfBuff) - pos ),PSTR("&VER=%s\0"), SP5K_REV );
	pos += snprintf_P( &gprs_printfBuff[pos],( sizeof(gprs_printfBuff) - pos ),PSTR("&INIT&CSQ=%d\0"), systemVars.csq);

	// GPRS sent
	FreeRTOS_write( &pdUART0, gprs_printfBuff, sizeof(gprs_printfBuff) );

	// DEBUG & LOG
	if ( (systemVars.debugLevel &  D_GPRS ) != 0) {
		snprintf_P( &gprs_printfBuff[pos],( sizeof(gprs_printfBuff) - pos ),PSTR("\r\n\0") );
		FreeRTOS_write( &pdUART1, gprs_printfBuff, sizeof(gprs_printfBuff) );
	}

	// TAIL ( No mando el close ya que espero la respuesta y no quiero que el socket se cierre )
	memset(gprs_printfBuff, '\0', sizeof(gprs_printfBuff));
	pos = snprintf_P( gprs_printfBuff, ( sizeof(gprs_printfBuff) - pos ),PSTR(" HTTP/1.1\n") );
	pos += snprintf_P( &gprs_printfBuff[pos], ( sizeof(gprs_printfBuff) - pos ),PSTR("Host: www.spymovil.com\n" ));
	pos += snprintf_P( &gprs_printfBuff[pos], sizeof(gprs_printfBuff),PSTR("\n\n\0" ));

	// GPRS sent
	FreeRTOS_write( &pdUART0, gprs_printfBuff, sizeof(gprs_printfBuff) );

	// DEBUG & LOG
	if ( (systemVars.debugLevel &  D_GPRS ) != 0) {
		snprintf_P( &gprs_printfBuff[pos],( sizeof(gprs_printfBuff) - pos ),PSTR("\r\n\0") );
		FreeRTOS_write( &pdUART1, gprs_printfBuff, sizeof(gprs_printfBuff) );
	}

}
//------------------------------------------------------------------------------------
static void pv_reconfigure_params(void)
{

uint8_t saveFlag = 0;
uint8_t channel;

	// Proceso la respuesta del INIT para reconfigurar los parametros
	pv_process_server_clock();
/*	saveFlag += pv_process_timerPoll();
	// Canales analogicos.
	for ( channel = 0; channel < NRO_ANALOG_CHANNELS; channel++ ) {
		saveFlag += pv_process_analogCh(channel);
	}
	for ( channel = 0; channel < NRO_DIGITAL_CHANNELS; channel++ ) {
		saveFlag += pv_process_digitalCh(channel);
	}

	if ( saveFlag > 0 ) {

		if ( u_saveSystemParams() ) {

			// DEBUG & LOG
			if ( (systemVars.debugLevel &  D_GPRS ) != 0) {
				snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR("%s GPRS::iniframe: Save params OK\r\n\0"), u_now());
				FreeRTOS_write( &pdUART1, gprs_printfBuff, sizeof(gprs_printfBuff) );
			}

		}
	}
*/
}
//------------------------------------------------------------------------------------
static void pv_process_server_clock(void)
{
/* Extraigo el srv clock del string mandado por el server y si el drift con la hora loca
 * es mayor a 5 minutos, ajusto la hora local
 * La linea recibida es del tipo: <h1>INIT_OK:CLOCK=1402251122:PWRM=DISC:</h1>
 *
 */

char *p, *s;
uint8_t ret = 0;
char localStr[32];
char *stringp;
char *token;
char *delim = ",=:><";
char rtcStr[12];
uint8_t i;
char c;

	s = FreeRTOS_UART_getFifoPtr(&pdUART0);
	p = strstr(s, "CLOCK");
	if ( p == NULL ) {
		return;
	}

	// Copio el mensaje enviado a un buffer local porque la funcion strsep lo modifica.
	memset(localStr,'\0',32);
	memcpy(localStr,p,sizeof(localStr));

	stringp = localStr;
	token = strsep(&stringp,delim);			// CLOCK

	token = strsep(&stringp,delim);			// rtc
	memset(rtcStr, '\0', sizeof(rtcStr));
//	memcpy(rtcStr,token, sizeof(rtcStr));	// token apunta al comienzo del string con la hora
	for ( i = 0; i<12; i++) {
		c = *token;
		rtcStr[i] = c;
		c = *(++token);
		if ( c == '\0' )
			break;

	}

	RTC_str_to_date(rtcStr);

	if ( (systemVars.debugLevel & (D_BASIC + D_GPRS) ) != 0) {
		snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR("%s GPRS::iniframe: Update rtc to: %s\r\n\0"), u_now(), rtcStr );
		FreeRTOS_write( &pdUART1, gprs_printfBuff, sizeof(gprs_printfBuff) );
	}

}
//------------------------------------------------------------------------------------
static uint8_t pv_process_timerPoll(void)
{
//	La linea recibida es del tipo: <h1>INIT_OK:CLOCK=1402251122:TPOLL=600:PWRM=DISC:</h1>

char *p, *s;
uint8_t ret = 0;
char localStr[32];
char *stringp;
char *token;
char *delim = ",=:><";

	s = FreeRTOS_UART_getFifoPtr(&pdUART0);
	p = strstr(s, "TPOLL");
	if ( p == NULL ) {
		goto quit;
	}

	// Copio el mensaje enviado a un buffer local porque la funcion strsep lo modifica.
	memset(localStr,'\0',32);
	memcpy(localStr,p,sizeof(localStr));

	stringp = localStr;
	token = strsep(&stringp,delim);	// TPOLL

	token = strsep(&stringp,delim);	// timerPoll
	u_config_timerpoll(token);
	ret = 1;
	if ( (systemVars.debugLevel & (D_BASIC + D_GPRS) ) != 0) {
		snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR("%s GPRS::iniframe: Reconfig TPOLL\r\n\0"), u_now());
		FreeRTOS_write( &pdUART1, gprs_printfBuff, sizeof(gprs_printfBuff) );
	}

quit:

	return(ret);
}
//------------------------------------------------------------------------------------
static uint8_t pv_process_analogCh(uint8_t channel)
{
//	La linea recibida es del tipo:
//	<h1>INIT_OK:CLOCK=1402251122:TPOLL=600:TDIAL=10300:PWRM=DISC:A0=pA,0,20,0,6:A1=pB,0,20,0,10:A2=pC,0,20,0,10:D0=q0,1.00:D1=q1,1.00</h1>

uint8_t ret = 0;
char localStr[32];
char *stringp;
char *token;
char *delim = ",=:><";
char *chName,*s_iMin,*s_iMax,*s_mMin,*s_mMax;
char *s;
char needle[4];

	s = FreeRTOS_UART_getFifoPtr(&pdUART0);

	needle[0] = 'A';
	needle[1] = channel + 32;
	needle[2] = '=';
	needle[3] = '\0';

	stringp = strstr(s, needle);

	if ( stringp == NULL ) {
		ret = 0;
		goto quit;
	}

	// Copio el mensaje enviado a un buffer local porque la funcion strsep lo modifica.
	memset(localStr,'\0',32);
	memcpy(localStr,stringp,31);

	stringp = localStr;
	token = strsep(&stringp,delim);	//A0

	chName = strsep(&stringp,delim);	//name
	s_iMin = strsep(&stringp,delim);	//iMin
	s_iMax = strsep(&stringp,delim);	//iMax
	s_mMin = strsep(&stringp,delim);	//mMin
	s_mMax = strsep(&stringp,delim);	//mMax

	u_config_analog( channel, chName,s_iMin,s_iMax,s_mMin,s_mMax );
	ret = 1;
	if ( (systemVars.debugLevel & (D_BASIC + D_GPRS) ) != 0) {
		snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR("%s GPRS::iniframe: Reconfig A%d\r\n\0"), u_now(), channel);
		FreeRTOS_write( &pdUART1, gprs_printfBuff, sizeof(gprs_printfBuff) );
	}

quit:
	return(ret);
}
//--------------------------------------------------------------------------------------
static uint8_t pv_process_digitalCh(uint8_t channel)
{

//	La linea recibida es del tipo:
//	<h1>INIT_OK:CLOCK=1402251122:TPOLL=600:TDIAL=10300:PWRM=DISC:A0=pA,0,20,0,6:A1=pB,0,20,0,10:A2=pC,0,20,0,10:D0=q0,1.00:D1=q1,1.00</h1>

uint8_t ret = 0;
char localStr[32];
char *stringp;
char *token;
char *delim = ",=:><";
char *chName;
char *s;
char needle[4];

	s = FreeRTOS_UART_getFifoPtr(&pdUART0);

	needle[0] = 'D';
	needle[1] = channel + 32;
	needle[2] = '=';
	needle[3] = '\0';

	stringp = strstr(s, needle);

	if ( stringp == NULL ) {
		ret = 0;
		goto quit;
	}

	// Copio el mensaje enviado a un buffer local porque la funcion strsep lo modifica.
	memset(localStr,'\0',32);
	memcpy(localStr,stringp,31);

	stringp = localStr;
	token = strsep(&stringp,delim);	//D0

	chName = strsep(&stringp,delim);	//name

	u_config_digital( channel, chName );
	ret = 1;
	if ( (systemVars.debugLevel & (D_BASIC + D_GPRS) ) != 0) {
		snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR("%s GPRS::iniframe: Reconfig D%d\r\n\0"), u_now(), channel);
		FreeRTOS_write( &pdUART1, gprs_printfBuff, sizeof(gprs_printfBuff) );
	}

quit:

	return(ret);

}
//--------------------------------------------------------------------------------------

/*
 * sp5K_tkCmd.c
 *
 *  Created on: 27/12/2013
 *      Author: root
 */

#include "sp5KV5_8CH.h"
#include "sp5KV5_8CH_tkGPRS/sp5KV5_tkGprs.h"

static char cmd_printfBuff[CHAR128];
char *argv[16];

//----------------------------------------------------------------------------------
// FUNCIONES DE USO PRIVADO
//-----------------------------------------------------------------------------------
static void pv_snprintfP_OK(void );
static void pv_snprintfP_ERR(void);
static uint8_t pv_makeArgv(void);

bool pv_cmdWrDebugLevel(char *s);
bool pv_cmdWrkMode(char *s0, char *s1);
static void pv_readMemory(void);

//-----------------------------------------------------------------------------------
// FUNCIONES DE CMDMODE
//------------------------------------------------------------------------------------
static void cmdClearScreen(void);
static void cmdHelpFunction(void);
static void cmdResetFunction(void);
static void cmdStatusFunction(void);
static void cmdReadFunction(void);
static void cmdWriteFunction(void);
static void cmdRedialFunction(void);
static void cmdKillFunction(void);
//------------------------------------------------------------------------------------
void tkCmd(void * pvParameters)
{

uint8_t c;
uint8_t ticks;
( void ) pvParameters;

	while ( !startTask )
		vTaskDelay( ( TickType_t)( 100 / portTICK_RATE_MS ) );

	cmdlineInit();
	cmdlineSetOutputFunc(pvFreeRTOS_UART1_writeChar);

	cmdlineAddCommand((uint8_t *)("cls"), cmdClearScreen );
	cmdlineAddCommand((uint8_t *)("help"), cmdHelpFunction);
	cmdlineAddCommand((uint8_t *)("reset"), cmdResetFunction);
	cmdlineAddCommand((uint8_t *)("read"), cmdReadFunction);
	cmdlineAddCommand((uint8_t *)("write"), cmdWriteFunction);
	cmdlineAddCommand((uint8_t *)("status"), cmdStatusFunction);
	cmdlineAddCommand((uint8_t *)("redial"), cmdRedialFunction);
	cmdlineAddCommand((uint8_t *)("kill"), cmdKillFunction);

	// Espero la notificacion para arrancar
	vTaskDelay( ( TickType_t)( 500 / portTICK_RATE_MS ) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("starting tkCmd..\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	ticks = 1;
	FreeRTOS_ioctl( &pdUART1,ioctlSET_TIMEOUT, &ticks, false );

	// loop
	for( ;; )
	{
		u_kick_Wdg(WDG_CMD);

		c = '\0';	// Lo borro para que luego del un CR no resetee siempre el timer.
		// el read se bloquea 50ms. lo que genera la espera.
		while ( FreeRTOS_read( &pdUART1, &c, 1 ) == 1 ) {
			cmdlineInputFunc(c);
		}

		/* run the cmdline execution functions */
		cmdlineMainLoop();

	}
}
//------------------------------------------------------------------------------------
static void cmdClearScreen(void)
{
	// ESC [ 2 J
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("\x1B[2J\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
}
//------------------------------------------------------------------------------------
static void cmdHelpFunction(void)
{

	memset( &cmd_printfBuff, '\0', sizeof(cmd_printfBuff));
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("\r\nSpymovil %s %s %dch %s %s\r\n\0"), SP5K_MODELO, SP5K_VERSION, NRO_ANALOG_CHANNELS, SP5K_REV, SP5K_DATE);
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("Available commands are:\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("-cls\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("-reset {memory}\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("-status\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("-redial\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("-write\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  rtc YYMMDDhhmm\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR( "  wrkmode [service | monitor {sqe|frame}]\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  timerpoll, dlgid, gsmband\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  debuglevel +/-{none,basic,mem,output,data,gprs,digital,all} \r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  loglevel (none, info, all)\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  analog {0..8} aname imin imax mmin mmax\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  cspan {0..8} {value}\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  digital {0..8} dname\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  ina {0,1,2} conf {value}\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  apn, port, ip, script, passwd\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  save\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  mcp {regAddr} {regValue}\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  ee addr string\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  gprspwr {0|1},gprssw {0|1} \r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  atcmd {cmd}\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("-read\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  mcp regAddr\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  din {0|1}\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  rtc, frame\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  ee {addr}{lenght}\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  ina {0,1,2} {conf|chXshv|chXbusv|mfid|dieid}\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  an {0..8}\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  din {0..3} \r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  defaults \r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  frame,memory,gprs\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("-kill { digital,gprs,data }\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

}
//------------------------------------------------------------------------------------
static void cmdResetFunction(void)
{
	pv_makeArgv();

	// Reset memory ??
	if (!strcmp_P( strupr(argv[1]), PSTR("MEMORY\0"))) {
		FF_rewind();
	}

	cmdClearScreen();
	// RESET
	u_reset();

}
//------------------------------------------------------------------------------------
static void cmdRedialFunction(void)
{
	// Envio un mensaje a la tk_Gprs para que recargue la configuracion y disque al server
	// Notifico en modo persistente. Si no puedo me voy a resetear por watchdog. !!!!
	while ( xTaskNotify(xHandle_tkGprsRx,TK_REDIAL , eSetBits ) != pdPASS ) {
		vTaskDelay( ( TickType_t)( 100 / portTICK_RATE_MS ) );
	}

}
//------------------------------------------------------------------------------------
static void cmdStatusFunction(void)
{

RtcTimeType_t rtcDateTime;
uint16_t pos;
uint8_t channel;
StatBuffer_t pxFFStatBuffer;
st_data_frame data_frame;

	memset( &cmd_printfBuff, '\0', sizeof(cmd_printfBuff));
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("\r\nSpymovil %s %s %dch %s %s\r\n\0"), SP5K_MODELO, SP5K_VERSION, NRO_ANALOG_CHANNELS, SP5K_REV, SP5K_DATE);
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	// Last reset info
	pos = snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("Wdg (0x%X"),wdgStatus.resetCause);
	if (wdgStatus.resetCause & 0x01 ) {
		pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR(" PORF"));
	}
	if (wdgStatus.resetCause & 0x02 ) {
		pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR(" EXTRF"));
	}
	if (wdgStatus.resetCause & 0x04 ) {
		pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR(" BORF"));
	}
	if (wdgStatus.resetCause & 0x08 ) {
		pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR(" WDRF"));
	}
	if (wdgStatus.resetCause & 0x10 ) {
		pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR(" JTRF"));
	}
	pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR(" )\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	/* DlgId */
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("dlgid: %s\r\n\0"), systemVars.dlgId );
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	/* Fecha y Hora */
	RTC_read(&rtcDateTime);
	pos = snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("rtc: %02d/%02d/%04d "),rtcDateTime.day,rtcDateTime.month, rtcDateTime.year );
	pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR("%02d:%02d:%02d\r\n\0"),rtcDateTime.hour,rtcDateTime.min, rtcDateTime.sec );
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	/* SERVER */
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR(">Server:\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	/* APN */
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  apn: %s\r\n\0"), systemVars.apn );
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	/* SERVER IP:SERVER PORT */
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  server ip:port: %s:%s\r\n\0"), systemVars.server_ip_address,systemVars.server_tcp_port );
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	/* SERVER SCRIPT */
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  server script: %s\r\n\0"), systemVars.serverScript );
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	/* SERVER PASSWD */
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  passwd: %s\r\n\0"), systemVars.passwd );
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	// MODEM ---------------------------------------------------------------------------------------
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR(">Modem:\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	/* Modem band */
	pos = snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  band: "));
	switch ( systemVars.gsmBand) {
	case 0:
		pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR("(900)"));
		break;
	case 1:
		pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR("(1800)"));
		break;
	case 2:
		pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR("dual band (900/1800)"));
		break;
	case 3:
		pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR("pcs (1900)"));
		break;
	case 4:
		pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR("gsm (850)"));
		break;
	case 5:
		pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR("dual band (1900/850)"));
		break;
	case 6:
		pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR("triband (900/1800/1900)"));
		break;
	case 7:
		pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR("triband (850/1800/1900)"));
		break;
	case 8:
		pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR("cuatriband (850/900/1800/1900)"));
		break;
	}
	pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR("\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	/* DLGIP */
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  dlg ip: %s\r\n\0"), systemVars.dlg_ip_address );
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	/* CSQ */
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  signalQ: csq=%d, dBm=%d\r\n\0"), systemVars.csq, systemVars.dbm );
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	// GPRS STATE
	pos = snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  state: "));
	switch (GPRS_stateVars.state) {
	case G_ESPERA_APAGADO:
		pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR("await_off\r\n"));
		break;
	case G_PRENDER:
		pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR("prendiendo\r\n"));
		break;
	case G_CONFIGURAR:
		pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR("configurando\r\n"));
		break;
	case G_MON_SQE:
		pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR("mon_sqe\r\n"));
		break;
	case G_GET_IP:
		pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR("ip\r\n"));
		break;
	case G_INIT_FRAME:
		pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR("init frame\r\n"));
		break;
	case G_DATA:
		pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR("data\r\n"));
		break;
	default:
		pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR("ERROR\r\n"));
		break;
	}
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	// SYSTEM ---------------------------------------------------------------------------------------
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR(">System:\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	/* Memoria */
	FF_stat(&pxFFStatBuffer);
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  memory: wrPtr=%d,rdPtr=%d,delPtr=%d,Free=%d,4del=%d \r\n"), pxFFStatBuffer.HEAD,pxFFStatBuffer.RD, pxFFStatBuffer.TAIL,pxFFStatBuffer.rcdsFree,pxFFStatBuffer.rcds4del );
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	/* WRK mode (NORMAL / SERVICE) */
	pos = snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  wrkmode: "));
	switch (systemVars.wrkMode) {
	case WK_NORMAL:
		pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR("normal\r\n"));
		break;
	case WK_MONITOR_SQE:
		pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR("monitor_sqe\r\n"));
		break;
	default:
		pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR("ERROR\r\n"));
		break;
	}
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	/* DebugLevel */
	pos = snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  debugLevel: "));
	if ( systemVars.debugLevel == D_NONE) {
		pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR("none") );
	} else {
		if ( (systemVars.debugLevel & D_BASIC) != 0) { pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR("+basic")); }
		if ( (systemVars.debugLevel & D_DATA) != 0) { pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR("+data")); }
		if ( (systemVars.debugLevel & D_GPRS) != 0) { pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR("+gprs")); }
		if ( (systemVars.debugLevel & D_MEM) != 0)   { pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR("+mem")); }
		if ( (systemVars.debugLevel & D_DIGITAL) != 0)  { pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR("+digital")); }
		if ( (systemVars.debugLevel & D_DEBUG) != 0)  { pos += snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR("+debug")); }
	}
	snprintf_P( &cmd_printfBuff[pos],sizeof(cmd_printfBuff),PSTR("\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	/* CONFIG */
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR(">Config:\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  timerPoll: [%d s]\r\n\0"),systemVars.timerPoll );
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

	/* Configuracion de canales analogicos */
	for ( channel = 0; channel < NRO_ANALOG_CHANNELS; channel++) {
		snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  a%d{%d-%dmA/%d,%d},%d,%s\r\n\0"),channel, systemVars.Imin[channel],systemVars.Imax[channel],systemVars.Mmin[channel],systemVars.Mmax[channel],systemVars.coef_calibracion[channel],systemVars.aChName[channel] );
		FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	}

	/* Configuracion de canales digitales */
	for ( channel = 0; channel < NRO_DIGITAL_CHANNELS; channel++) {
		snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("  d%d{%s}\r\n\0"),channel, systemVars.dChName[channel] );
		FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	}

	// Valores actuales: leo e imprimo
	u_read_dataframe(&data_frame);
	u_print_dataframe(&data_frame);

}
//------------------------------------------------------------------------------------
static void cmdKillFunction(void)
{
uint8_t argc;

	argc = pv_makeArgv();

	// KILL DATA
	if (!strcmp_P( strupr(argv[1]), PSTR("DATA\0"))) {
		vTaskSuspend( xHandle_tkData );
		return;
	}

	// KILL DIGITAL
	if (!strcmp_P( strupr(argv[1]), PSTR("DIGITAL\0"))) {
		vTaskSuspend( xHandle_tkDigitalIn );
		return;
	}

	// KILL GPRS
	if (!strcmp_P( strupr(argv[1]), PSTR("GPRS\0"))) {
		vTaskSuspend( xHandle_tkGprs );
		return;
	}

	pv_snprintfP_OK();
	return;
}
//------------------------------------------------------------------------------------
static void cmdReadFunction(void)
{
uint8_t argc;
char datetime[24];
bool retS = false;
uint8_t regValue;
uint16_t raw_val;
float mag_val;
char *p;

	argc = pv_makeArgv();

	// EE
	// read ee address length
	if (!strcmp_P( strupr(argv[1]), PSTR("EE\0"))) {
		memset(cmd_printfBuff, '\0', sizeof(cmd_printfBuff));
		retS = EE_test_read( argv[2], cmd_printfBuff, argv[3] );
		if ( retS ) {
			// El string leido lo devuelve en cmd_printfBuff por lo que le agrego el CR.
			snprintf_P( &cmd_printfBuff[atoi(argv[3])], sizeof(cmd_printfBuff),PSTR( "\r\n\0"));
			FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
		}
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	// RTC
	if (!strcmp_P( strupr(argv[1]), PSTR("RTC\0"))) {
		if ( RTC_date_to_str( datetime, sizeof(datetime) ) != -1 ) {
			snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("OK\r\n%s\r\n\0"), datetime );
		} else {
			snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("ERROR\r\n\0"));
		}
		FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
		return;
	}

	// MCP
	// read mcp addr
	if (!strcmp_P( strupr(argv[1]), PSTR("MCP\0"))) {
		retS = MCP_read( MCP0_ADDR, atoi(argv[2]), &regValue );

		if (retS ) {
			snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("OK\r\n[reg 0X%03x]=[0X%03x]\r\n\0"),atoi(argv[3]),regValue);
		} else {
			snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("ERROR\r\n\0"));
		}
		FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
		return;
	}

	// DIN
	if (!strcmp_P( strupr(argv[1]), PSTR("DIN\0"))) {
		u_read_digitalIn(atoi(argv[2]) );
		return;
	}

	// DEFAULT (load default configuration)
	if (!strcmp_P( strupr(argv[1]), PSTR("DEFAULTS\0"))) {
		u_loadDefaults();
		return;
	}

	// FRAME
	// read frame
	if (!strcmp_P( strupr(argv[1]), PSTR("FRAME\0"))) {
		u_tkData_read_frame(false);
		pv_snprintfP_OK();
		return;
	}

	// MEMORY
	if (!strcmp_P( strupr(argv[1]), PSTR("MEMORY\0"))) {
		pv_readMemory();
		return;
	}

	// INA
	if (!strcmp_P( strupr(argv[1]), PSTR("INA\0"))) {
		u_read_INA3221(argv[2], argv[3]);
		return;
	}

	// AN { 0..8}
	if (!strcmp_P( strupr(argv[1]), PSTR("AN\0"))) {
		u_read_analogIn( atoi(argv[2]),false, &raw_val, &mag_val );
		snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("CH[%02d] raw=%d,mag=%.02f\r\n\0"),atoi(argv[2]),raw_val, mag_val );
		FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
		return;
	}

	// GPRS RSP.
	if (!strcmp_P( strupr(argv[1]), PSTR("GPRS\0"))) {
		p = FreeRTOS_UART_getFifoPtr(&pdUART0);
		FreeRTOS_write( &pdUART1, "rx->", sizeof("rx->")  );
		FreeRTOS_write( &pdUART1, p, UART0_RXBUFFER_LEN );
		FreeRTOS_write( &pdUART1, "\r\n\0", sizeof("\r\n\0")  );
		return;
	}

	// CMD NOT FOUND
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("ERROR\r\nCMD NOT DEFINED\r\n"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	return;
}
//------------------------------------------------------------------------------------
static void cmdWriteFunction(void)
{
bool retS = false;
uint8_t argc;

	argc = pv_makeArgv();

	// SAVE
	if (!strcmp_P( strupr(argv[1]), PSTR("SAVE\0"))) {
		retS = u_saveSystemParams();
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	// PASSWD
	if (!strcmp_P( strupr(argv[1]), PSTR("PASSWD\0"))) {
		if ( argv[2] == NULL ) {
			retS = false;
		} else {
			memset(systemVars.passwd, '\0', sizeof(systemVars.passwd));
			memcpy(systemVars.passwd, argv[2], sizeof(systemVars.passwd));
			systemVars.passwd[PASSWD_LENGTH - 1] = '\0';
			retS = true;
		}
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	// DLGID
	if (!strcmp_P( strupr(argv[1]), PSTR("DLGID\0"))) {
		if ( argv[2] == NULL ) {
			retS = false;
		} else {
			memcpy(systemVars.dlgId, argv[2], sizeof(systemVars.dlgId));
			systemVars.dlgId[DLGID_LENGTH - 1] = '\0';
			retS = true;
		}
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	// APN
	if (!strcmp_P( strupr(argv[1]), PSTR("APN\0"))) {
		if ( argv[2] == NULL ) {
			retS = false;
		} else {
			memset(systemVars.apn, '\0', sizeof(systemVars.apn));
			memcpy(systemVars.apn, argv[2], sizeof(systemVars.apn));
			systemVars.apn[APN_LENGTH - 1] = '\0';
			retS = true;
		}
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	// SERVER PORT
	if (!strcmp_P( strupr(argv[1]), PSTR("PORT\0"))) {
		if ( argv[2] == NULL ) {
			retS = false;
		} else {
			memset(systemVars.server_tcp_port, '\0', sizeof(systemVars.server_tcp_port));
			memcpy(systemVars.server_tcp_port, argv[2], sizeof(systemVars.server_tcp_port));
			systemVars.server_tcp_port[PORT_LENGTH - 1] = '\0';
			retS = true;
		}
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	// SERVER IP
	if (!strcmp_P( strupr(argv[1]), PSTR("IP\0"))) {
		if ( argv[2] == NULL ) {
			retS = false;
		} else {
			memset(systemVars.server_ip_address, '\0', sizeof(systemVars.server_ip_address));
			memcpy(systemVars.server_ip_address, argv[2], sizeof(systemVars.server_ip_address));
			systemVars.server_ip_address[IP_LENGTH - 1] = '\0';
			retS = true;
		}
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	// SERVER SCRIPT
	if (!strcmp_P( strupr(argv[1]), PSTR("SCRIPT\0"))) {
		if ( argv[2] == NULL ) {
			retS = false;
		} else {
			memset(systemVars.serverScript, '\0', sizeof(systemVars.serverScript));
			memcpy(systemVars.serverScript, argv[2], sizeof(systemVars.serverScript));
			systemVars.serverScript[SCRIPT_LENGTH - 1] = '\0';
			retS = true;
		}
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	/* DEBUGLEVEL */
	if (!strcmp_P( strupr(argv[1]), PSTR("DEBUGLEVEL\0"))) {
		retS = pv_cmdWrDebugLevel(argv[2]);
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	/* WRKMODE */
	if (!strcmp_P( strupr(argv[1]), PSTR("WRKMODE\0"))) {
		retS = pv_cmdWrkMode(argv[2],argv[3]);
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	// CANALES ANALOGICOS
	// analog {0..8} aname imin imax mmin mmax
	if (!strcmp_P( strupr(argv[1]), PSTR("ANALOG\0"))) {
		u_config_analog( atoi(argv[2]), argv[3], argv[4], argv[5], argv[6], argv[7] );
		pv_snprintfP_OK();
		return;
	}

	// CANALES DIGITALES
	if (!strcmp_P( strupr(argv[1]), PSTR("DIGITAL\0"))) {
		u_config_digital( atoi(argv[2]),argv[3]);
		pv_snprintfP_OK();
		return;
	}

	// TIMERPOLL
	if (!strcmp_P( strupr(argv[1]), PSTR("TIMERPOLL\0"))) {
		retS = u_config_timerpoll(argv[2]);
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	// RTC
	if (!strcmp_P( strupr(argv[1]), PSTR("RTC\0"))) {
		retS = RTC_str_to_date(argv[2]);
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	// INA
	//  ina {0,1,2} conf {value}
	if ( !strcmp_P( strupr(argv[1]), PSTR("INA\0")) && !strcmp_P( strupr(argv[3]), PSTR("CONF\0"))) {
		INA3221_test_write( atoi(argv[2]), "0", argv[4]);
		pv_snprintfP_OK();
		return;
	}

	// CSPAN
	// Coeficinte de correccion del conversor INA.
	if (!strcmp_P( strupr(argv[1]), PSTR("CSPAN\0"))) {
		u_config_cspan(argv[2], argv[3]);
		pv_snprintfP_OK();
		return;
	}

	//----------------------------------------------------------------------
	// COMANDOS USADOS PARA DIAGNOSTICO
	// DEBEMOS ESTAR EN MODO SERVICE
	//----------------------------------------------------------------------

	// GSMBAND:
	// Debo estar en modo service ya que para que tome el valor debe resetearse
	if (!strcmp_P( strupr(argv[1]), PSTR("GSMBAND\0"))) {
		if ( argv[2] == NULL ) {
			retS = false;
		} else {
			systemVars.gsmBand = atoi(argv[2]);
			retS = true;
		}
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	// EE: write ee pos string
	if (!strcmp_P( strupr(argv[1]), PSTR("EE\0"))) {
		retS = EE_test_write( argv[2], argv[3]);
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	// gprsPWR
	if (!strcmp_P( strupr(argv[1]), PSTR("GPRSPWR\0"))) {
		switch( atoi(argv[2]) ) {
		case 0:
			IO_modem_hw_pwr_off();
			retS = true;
			break;
		case 1:
			IO_modem_hw_pwr_on();
			retS = true;
			break;
		default:
			retS = false;
			break;
		}
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	// gprsSW
	if (!strcmp_P( strupr(argv[1]), PSTR("GPRSSW\0"))) {
		switch( atoi(argv[2]) ) {
		case 0:
			IO_modem_sw_switch_low();
			retS = true;
			break;
		case 1:
			IO_modem_sw_switch_high();
			retS = true;
			break;
		default:
			retS = false;
			break;
		}
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	// MCP
	// write mcp addr value
	if (!strcmp_P( strupr(argv[1]), PSTR("MCP\0"))) {
		retS = MCP_write( MCP0_ADDR, atoi(argv[2]), atoi(argv[3]) );
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	// ATCMD
	// Envia un comando al modem.
	if (!strcmp_P( strupr(argv[1]), PSTR("ATCMD\0"))) {
		snprintf( cmd_printfBuff,sizeof(cmd_printfBuff),"%s\r\0",argv[2] );
		FreeRTOS_ioctl( &pdUART0,ioctl_UART_CLEAR_RX_BUFFER, NULL, false);
		FreeRTOS_ioctl( &pdUART0,ioctl_UART_CLEAR_TX_BUFFER, NULL, false);
		FreeRTOS_write( &pdUART0, cmd_printfBuff, sizeof(cmd_printfBuff) );

		snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("sent->%s\r\n\0"),argv[2] );
		FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
		return;
	}

	// CMD NOT FOUND
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("ERROR\r\nCMD NOT DEFINED\r\n"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
	return;
}
//------------------------------------------------------------------------------------
// FUNCIONES PRIVADAS
//-------------------------------------------------------------------------------------
bool pv_cmdWrkMode(char *s0, char *s1)
{
bool retS = false;

	if ((!strcmp_P(strupr(s0), PSTR("MONITOR")))) {

		if ((!strcmp_P( strupr(s1), PSTR("SQE")))) {
			systemVars.wrkMode = WK_MONITOR_SQE;
			retS = true;
			goto quit;
		}

	}

quit:

	return(retS);
}
//------------------------------------------------------------------------------------
bool pv_cmdWrDebugLevel(char *s)
{

	if ((!strcmp_P( strupr(s), PSTR("NONE")))) {
		systemVars.debugLevel = D_NONE;
		return(true);
	}

	if ((!strcmp_P( strupr(s), PSTR("+BASIC")))) {
		systemVars.debugLevel += D_BASIC;
		return(true);
	}

	if ((!strcmp_P( strupr(s), PSTR("-BASIC")))) {
		if ( ( systemVars.debugLevel & D_BASIC) != 0 ) {
			systemVars.debugLevel -= D_BASIC;
			return(true);
		}
	}

	if ((!strcmp_P( strupr(s), PSTR("+DATA")))) {
		systemVars.debugLevel += D_DATA;
		return(true);
	}

	if ((!strcmp_P( strupr(s), PSTR("-DATA")))) {
		if ( ( systemVars.debugLevel & D_DATA) != 0 ) {
			systemVars.debugLevel -= D_DATA;
			return(true);
		}
	}

	if ((!strcmp_P( strupr(s), PSTR("+MEM")))) {
		systemVars.debugLevel += D_MEM;
		return(true);
	}

	if ((!strcmp_P( strupr(s), PSTR("-MEM")))) {
		if ( ( systemVars.debugLevel & D_MEM) != 0 ) {
			systemVars.debugLevel -= D_MEM;
			return(true);
		}
	}

	if ((!strcmp_P( strupr(s), PSTR("+GPRS")))) {
		systemVars.debugLevel += D_GPRS;
		return(true);
	}

	if ((!strcmp_P( strupr(s), PSTR("-GPRS")))) {
		if ( ( systemVars.debugLevel & D_GPRS) != 0 ) {
			systemVars.debugLevel -= D_GPRS;
			return(true);
		}
	}

	if ((!strcmp_P( strupr(s), PSTR("+DIGITAL")))) {
		systemVars.debugLevel += D_DIGITAL;
		return(true);
	}

	if ((!strcmp_P( strupr(s), PSTR("-DIGITAL")))) {
		if ( ( systemVars.debugLevel & D_DIGITAL) != 0 ) {
			systemVars.debugLevel -= D_DIGITAL;
			return(true);
		}
	}

	if ((!strcmp_P( strupr(s), PSTR("+DEBUG")))) {
		systemVars.debugLevel += D_DEBUG;
		return(true);
	}

	if ((!strcmp_P( strupr(s), PSTR("-DEBUG")))) {
		if ( ( systemVars.debugLevel & D_DEBUG) != 0 ) {
			systemVars.debugLevel -= D_DEBUG;
			return(true);
		}
	}
	if ((!strcmp_P( strupr(s), PSTR("ALL")))) {
		systemVars.debugLevel = D_DATA + D_GPRS + D_MEM + D_DIGITAL + D_DEBUG;
		return(true);
	}

	return(false);
}
//------------------------------------------------------------------------------------
static uint8_t pv_makeArgv(void)
{
// A partir de la linea de comando, genera un array de punteros a c/token
//
char *token = NULL;
char parseDelimiters[] = " ";
int i = 0;

	// inicialmente todos los punteros deben apuntar a NULL.
	memset(argv, 0, sizeof(argv) );

	// Genero los tokens delimitados por ' '.
	token = strtok(SP5K_CmdlineBuffer, parseDelimiters);
	argv[i++] = token;
	while ( (token = strtok(NULL, parseDelimiters)) != NULL ) {
		argv[i++] = token;
		if (i == 16) break;
	}
	return(( i - 1));
}
//------------------------------------------------------------------------------------
static void pv_snprintfP_OK(void )
{
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("OK\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
}
//------------------------------------------------------------------------------------
static void pv_snprintfP_ERR(void)
{
	snprintf_P( cmd_printfBuff,sizeof(cmd_printfBuff),PSTR("ERROR\r\n\0"));
	FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );
}
//------------------------------------------------------------------------------------
static void pv_readMemory(void)
{
	// Leemos la memoria e imprimo los datos.
	// El problema es que si hay muchos datos puede excederse el tiempo de watchdog y
	// resetearse el dlg.
	// Para esto, cada 32 registros pateo el watchdog.

StatBuffer_t pxFFStatBuffer;
st_data_frame data_frame;
size_t bRead;
uint16_t rcds = 0;

	FF_seek();
	while(1) {
		bRead = FF_fread( &data_frame, sizeof(data_frame));

		if ( bRead == 0) {
			break;
		}

		if ( ( rcds++ % 32) == 0 ) {
			u_kick_Wdg(WDG_CMD);
		}

		// imprimo
		FF_stat(&pxFFStatBuffer);
		snprintf_P( cmd_printfBuff, sizeof(cmd_printfBuff), PSTR("RD:[%d/%d/%d][%d/%d] "), pxFFStatBuffer.HEAD,pxFFStatBuffer.RD, pxFFStatBuffer.TAIL,pxFFStatBuffer.rcdsFree,pxFFStatBuffer.rcds4del);
		FreeRTOS_write( &pdUART1, cmd_printfBuff, sizeof(cmd_printfBuff) );

		u_print_dataframe(&data_frame);
	}
}
//------------------------------------------------------------------------------------

/*
 * sp5KV3_tkControl.c
 *
 *  Created on: 7/4/2015
 *      Author: pablo
 *
 *  Tareas de control generales del SP5K
 *  - Recibe un mensaje del timer del led para indicar si debe prender o apagarlo.
 */

#include "sp5KV5_8CH.h"
#include "sp5KV5_8CH_tkGPRS/sp5KV5_tkGprs.h"

static char ctl_printfBuff[CHAR128];

static void pv_tkControl_init(void);
static void pv_init_show_reset_cause(void);
static void pv_check_leds(void);
static void pv_check_daily_reset(void);
static void pv_check_wdg(void);

//------------------------------------------------------------------------------------
void tkControl(void * pvParameters)
{

( void ) pvParameters;

	// Aqui solo controlo la terminal por lo que no me importa tanto el watchdog.
	// Lo controlo en LedTiltWdg

	pv_tkControl_init();
	pv_init_show_reset_cause();

	snprintf_P( ctl_printfBuff,sizeof(ctl_printfBuff),PSTR("-----------------\r\n\0"));
	FreeRTOS_write( &pdUART1, ctl_printfBuff, sizeof(ctl_printfBuff) );
	snprintf_P( ctl_printfBuff,sizeof(ctl_printfBuff),PSTR("starting tkControl..\r\n\0"));
	FreeRTOS_write( &pdUART1, ctl_printfBuff, sizeof(ctl_printfBuff) );

	// Loop
    for( ;; )
    {

    	u_kick_Wdg(WDG_CTL);

	   	// Espero 1 segundo para revisar todo.
        vTaskDelay( ( TickType_t)( 1000 / portTICK_RATE_MS ) );

        // Reviso los sistemas perifericos.
        pv_check_leds();
        pv_check_daily_reset();
        pv_check_wdg();

    }

}
//------------------------------------------------------------------------------------
static void pv_check_wdg(void)
{
	// Cada tarea periodicamente pone su wdg flag en 0. Esto hace que al chequearse c/3s
	// deban estar todas en 0 para asi resetear el wdg del micro.

static u08 l_timer = 5;


	if (l_timer-- > 0 ) {
		wdt_reset();
		return;
	}

	l_timer = 5;
	if ( systemWdg == 0 ) {
		systemWdg = WDG_CTL + WDG_CMD + WDG_DIN;
	} else {
		while(1);
	}

}
//------------------------------------------------------------------------------------
static void pv_check_leds(void)
{

static uint8_t count = 3;

	// Los leds flashean c/3s solo si la terminal esta prendida.
	// Siempre los apago para no correr riesgo que queden prendidos.

	if ( --count > 0 ) {
		return;
	}

	count = 3;
   	// Prendo.
   	IO_set_led_KA_logicBoard();				// Led de KA de la placa logica
   	// El led del modem lo maneja el modem

   	// no es necesario ya que lo que demora las MCP son suficientes.
   	//vTaskDelay( 1 );

   	// Apago
   	IO_clear_led_KA_logicBoard();

 }
//------------------------------------------------------------------------------------
static void  pv_check_daily_reset(void)
{
	// Todos los dias debo resetearme para restaturar automaticamente posibles
	// problemas.

static uint32_t ticks_to_reset = 86400; // Segundos en 1 dia.

	while ( --ticks_to_reset > 0 ) {
		return;
	}

	snprintf_P( ctl_printfBuff,sizeof(ctl_printfBuff),PSTR("%s CTL::reset: Daily Reset !!\r\n\0"), u_now() );
	u_debugPrint( D_BASIC, ctl_printfBuff, sizeof(ctl_printfBuff) );
	vTaskDelay( ( TickType_t)( 1000 / portTICK_RATE_MS ) );
	wdt_enable(WDTO_30MS);
	while(1) {}

}
//------------------------------------------------------------------------------------
// FUNCIONES DE INIT
//------------------------------------------------------------------------------------
static void pv_tkControl_init(void)
{

uint8_t ffRcd;
StatBuffer_t pxFFStatBuffer;
int8_t loadParamStatus = false;
uint16_t recSize;

	vTaskDelay( ( TickType_t)( 500 / portTICK_RATE_MS ) );

	MCP_init(0);				// Esto prende la terminal.
	IO_term_pwr_on();
	u_uarts_ctl(TERM_PRENDER);

	// Load systemVars
	if  ( u_loadSystemParams() == true ) {
		loadParamStatus = true;
	} else {
		u_loadDefaults();
		u_saveSystemParams();
		loadParamStatus = false;
	}

	// Configuro el ID en el bluetooth: debe hacerse antes que nada
	snprintf_P( ctl_printfBuff,sizeof(ctl_printfBuff),PSTR("AT+NAME%s\r\n"),systemVars.dlgId);
	FreeRTOS_write( &pdUART1, ctl_printfBuff, sizeof(ctl_printfBuff) );
	vTaskDelay( ( TickType_t)( 2000 / portTICK_RATE_MS ) );

	// Mensaje de load Status.
	if ( loadParamStatus ) {
		snprintf_P( ctl_printfBuff,sizeof(ctl_printfBuff),PSTR("Load config OK.\r\n\0") );
	} else {
		snprintf_P( ctl_printfBuff,sizeof(ctl_printfBuff),PSTR("Load config ERROR: defaults !!\r\n\0") );
	}
	FreeRTOS_write( &pdUART1, ctl_printfBuff, sizeof(ctl_printfBuff) );

	u_kick_Wdg(WDG_CTL);

	// Inicializo la memoria EE ( fileSysyem)
	ffRcd = FF_fopen();
	FF_stat(&pxFFStatBuffer);
	if ( pxFFStatBuffer.errno != pdFF_ERRNO_NONE ) {
		snprintf_P( ctl_printfBuff,sizeof(ctl_printfBuff),PSTR("FSInit ERROR (%d)[%d]\r\n\0"),ffRcd, pxFFStatBuffer.errno);
	} else {
		snprintf_P( ctl_printfBuff,sizeof(ctl_printfBuff),PSTR("FSInit OK\r\nMEMsize=%d, wrPtr=%d,rdPtr=%d,delPtr=%d,Free=%d,4del=%d\r\n\0"),FF_MAX_RCDS, pxFFStatBuffer.HEAD,pxFFStatBuffer.RD, pxFFStatBuffer.TAIL,pxFFStatBuffer.rcdsFree,pxFFStatBuffer.rcds4del);
	}
	FreeRTOS_write( &pdUART1, ctl_printfBuff, sizeof(ctl_printfBuff) );

	// Tamanio de registro de memoria
	recSize = sizeof(st_data_frame);
	snprintf_P( ctl_printfBuff,sizeof(ctl_printfBuff),PSTR("RCD size %d bytes.\r\n\0"),recSize);
	FreeRTOS_write( &pdUART1, ctl_printfBuff, sizeof(ctl_printfBuff) );

	// Habilito al resto de las tareas a arrancar.
	startTask = true;

}
//------------------------------------------------------------------------------------
static void pv_init_show_reset_cause(void)
{
uint8_t pos;

	// Muestro la razon del ultimo reseteo

	pos = snprintf_P( ctl_printfBuff,sizeof(ctl_printfBuff),PSTR("Init code (0x%X"),wdgStatus.resetCause);
	if (wdgStatus.resetCause & 0x01 ) {
		pos += snprintf_P( &ctl_printfBuff[pos],sizeof(ctl_printfBuff),PSTR(" PORF"));
	}
	if (wdgStatus.resetCause & 0x02 ) {
		pos += snprintf_P( &ctl_printfBuff[pos],sizeof(ctl_printfBuff),PSTR(" EXTRF"));
	}
	if (wdgStatus.resetCause & 0x04 ) {
		pos += snprintf_P( &ctl_printfBuff[pos],sizeof(ctl_printfBuff),PSTR(" BORF"));
	}
	if (wdgStatus.resetCause & 0x08 ) {
		pos += snprintf_P( &ctl_printfBuff[pos],sizeof(ctl_printfBuff),PSTR(" WDRF"));
	}
	if (wdgStatus.resetCause & 0x10 ) {
		pos += snprintf_P( &ctl_printfBuff[pos],sizeof(ctl_printfBuff),PSTR(" JTRF"));
	}
	pos += snprintf_P( &ctl_printfBuff[pos],sizeof(ctl_printfBuff),PSTR(" )\r\n\0"));
	FreeRTOS_write( &pdUART1, ctl_printfBuff, sizeof(ctl_printfBuff) );

}
//------------------------------------------------------------------------------------
// FUNCIONES PUBLICAS DE USO GENERAL
//------------------------------------------------------------------------------------


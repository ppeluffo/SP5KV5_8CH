/*
 * sp5KV5_8CH_tkData.c
 *
 *  Created on: 8 de mar. de 2018
 *      Author: pablo
 */

#include "sp5KV5_8CH.h"

//------------------------------------------------------------------------------------
// PROTOTIPOS
static bool pv_tkData_guardar_BD(void);

// VARIABLES LOCALES
static char data_printfBuff[CHAR256];
static st_analog_frame analog_frame;
static st_data_frame data_frame;

//------------------------------------------------------------------------------------
void tkData(void * pvParameters)
{

( void ) pvParameters;

uint32_t waiting_ticks;
TickType_t xLastWakeTime;

	// Espero la notificacion para arrancar
	while ( !startTask )
		vTaskDelay( ( TickType_t)( 100 / portTICK_RATE_MS ) );

	snprintf_P( data_printfBuff,sizeof(data_printfBuff),PSTR("starting tkData..\r\n\0"));
	FreeRTOS_write( &pdUART1, data_printfBuff, sizeof(data_printfBuff) );

	// Configuro los INA para promediar en 128 valores.
	u_analog_init();

    // Initialise the xLastWakeTime variable with the current time.
    xLastWakeTime = xTaskGetTickCount();

    // Al arrancar poleo a los 5s
    waiting_ticks = (uint32_t)(5) * 1000 / portTICK_RATE_MS;

	// loop
	for( ;; )
	{
		vTaskDelayUntil( &xLastWakeTime, waiting_ticks ); // Da el tiempo para entrar en tickless.

		// Leo analog,digital,rtc,salvo en BD e imprimo.
		u_tkData_read_frame(true);

		while ( xSemaphoreTake( sem_SYSVars, ( TickType_t ) 1 ) != pdTRUE )
			taskYIELD();
		waiting_ticks = (uint32_t)(systemVars.timerPoll) * 1000 / portTICK_RATE_MS;
		xSemaphoreGive( sem_SYSVars );

	}

}
//------------------------------------------------------------------------------------
void u_tkData_read_frame(bool saveInBD )
{
	// Funcion usada para leer los datos de todos los modulos, guardarlos en memoria
	// e imprimirlos.
	// La usa por un lado tkData en forma periodica y desde el cmd line cuando se
	// da el comando read frame.

static bool primer_frame = true;
bool retS = false;

	// Leo los canales analogicos.( uso analog_frame ya que la funcion requiere los campos
	// mag y raw pero luego solo me sirven las mag.
	u_analog_polear(&analog_frame);
	memcpy(data_frame.an_mag_val, analog_frame.mag_val, sizeof(analog_frame.mag_val));

	// Leo los canales digitales y los dejo en 0.
	u_read_digital_frame(&data_frame.digital_frame, true);

	// Agrego el timestamp
	RTC_read(&data_frame.rtc);

	// Para no incorporar el error de los contadores en el primer frame lo descarto
	if ( primer_frame ) {
		primer_frame = false;
		return;
	}

	// Si me invocaron por modo comando, no salvo en BD
	if ( saveInBD ) {
		retS = pv_tkData_guardar_BD();
		if ( !retS ) {
			snprintf_P( data_printfBuff, sizeof(data_printfBuff) , PSTR("Data BD save ERROR !!\r\n\0") );
			FreeRTOS_write( &pdUART1, data_printfBuff, sizeof(data_printfBuff) );
			return;
		}
	}

	u_print_dataframe(&data_frame);

}
//------------------------------------------------------------------------------------
static bool pv_tkData_guardar_BD(void)
{

	// Solo los salvo en la BD si estoy en modo normal.
	// En otros casos ( service, monitor_frame, etc, no.
size_t bytes_written;
StatBuffer_t pxFFStatBuffer;

	if ( (systemVars.debugLevel & D_DATA) != 0) {
		snprintf_P( data_printfBuff,sizeof(data_printfBuff),PSTR("%s aDATA::bd:\r\n\0"),u_now() );
		FreeRTOS_write( &pdUART1, data_printfBuff, sizeof(data_printfBuff) );
	}

	// Guardo en BD
	bytes_written = FF_fwrite( &data_frame, sizeof(data_frame));
	FF_stat(&pxFFStatBuffer);

	if ( bytes_written != sizeof(data_frame) ) {
		// Error de escritura ??
		if ( (systemVars.debugLevel & (D_BASIC + D_DATA) ) != 0) {
			snprintf_P( data_printfBuff,sizeof(data_printfBuff),PSTR("%s aDATA::bd: WR ERROR: (%d)\r\n\0"),u_now(),pxFFStatBuffer.errno);
			FreeRTOS_write( &pdUART1, data_printfBuff, sizeof(data_printfBuff) );
			return(false);
		}

	} else {

		// Stats de memoria
		if ( (systemVars.debugLevel & (D_BASIC + D_DATA) ) != 0) {
			snprintf_P( data_printfBuff, sizeof(data_printfBuff), PSTR("%s aDATA::bd: MEM [%d/%d/%d][%d/%d]\r\n\0"),u_now(), pxFFStatBuffer.HEAD,pxFFStatBuffer.RD, pxFFStatBuffer.TAIL,pxFFStatBuffer.rcdsFree,pxFFStatBuffer.rcds4del);
			FreeRTOS_write( &pdUART1, data_printfBuff, sizeof(data_printfBuff) );
			return(true);
		}
	}

	return(true);

}
//------------------------------------------------------------------------------------
// FUNCIONES PUBLICAS
//------------------------------------------------------------------------------------
void u_read_dataframe(st_data_frame *dst_data_frame)
{
	memcpy(dst_data_frame, &data_frame, sizeof( st_data_frame));

}
//------------------------------------------------------------------------------------
void u_print_dataframe(st_data_frame *data_frame)
{
	// Imprime el frame en consola

uint8_t pos;
uint8_t channel;

	// HEADER
	pos = snprintf_P( data_printfBuff, sizeof(data_printfBuff), PSTR("frame: " ) );
	// timeStamp.
	pos += snprintf_P( &data_printfBuff[pos], ( sizeof(data_printfBuff) - pos ),PSTR( "%04d%02d%02d,"),data_frame->rtc.year,data_frame->rtc.month,data_frame->rtc.day );
	pos += snprintf_P( &data_printfBuff[pos], ( sizeof(data_printfBuff) - pos ), PSTR("%02d%02d%02d"),data_frame->rtc.hour,data_frame->rtc.min, data_frame->rtc.sec );

	// Valores analogicos
	for ( channel = 0; channel < NRO_ANALOG_CHANNELS; channel++) {
		pos += snprintf_P( &data_printfBuff[pos], ( sizeof(data_printfBuff) - pos ), PSTR(",%s=%.02f"),systemVars.aChName[channel],data_frame->an_mag_val[channel] );
	}

	// Valores digitales
	for ( channel = 0; channel < NRO_DIGITAL_CHANNELS; channel++) {
		pos += snprintf_P( &data_printfBuff[pos], ( sizeof(data_printfBuff) - pos ), PSTR(",%s_L=%d,%s_T=%d"),systemVars.dChName[channel],data_frame->digital_frame.level[channel],systemVars.dChName[channel],data_frame->digital_frame.ticks_time_H[channel] );
	}

	// TAIL
	pos += snprintf_P( &data_printfBuff[pos], ( sizeof(data_printfBuff) - pos ), PSTR("\r\n\0") );

	// Imprimo
	FreeRTOS_write( &pdUART1, data_printfBuff, sizeof(data_printfBuff) );
}
//------------------------------------------------------------------------------------

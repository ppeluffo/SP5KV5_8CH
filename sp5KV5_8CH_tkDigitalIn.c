/*
 * sp5KV3_tkDigitalIn.c
 *
 *  Mido pulsos.
 *  En la configuracion solo configuro el nombre.
 *  Tenemos 4 entradas de las que medimos:
 *  - Nivel del pin.
 *  - Cantidad de pulsos
 *  - Tiempo HIGH / LOW.
 *
 *  Poleo cada 100ms de modo que la granularidad del tiempo es esta.
 *  El nivel y los tiempos los tengo leyendo los pines_L
 *  Los pulsos los tengo de leer los latches.
 *
 */


#include "sp5KV5_8CH.h"

static void pv_clearLatchs(void);
static void pv_poll_dInputs(void);

static char dIn_printfBuff[CHAR128];	// Buffer de impresion

st_digital_frame digital_frame;
static uint16_t total_ticks;

//------------------------------------------------------------------------------------
void tkDigitalIn(void * pvParameters)
{

( void ) pvParameters;
uint8_t channel;

	while ( !startTask )
		vTaskDelay( ( TickType_t)( 100 / portTICK_RATE_MS ) );

	snprintf_P( dIn_printfBuff,sizeof(dIn_printfBuff),PSTR("starting tkDigitalIn..\r\n\0"));
	FreeRTOS_write( &pdUART1, dIn_printfBuff, sizeof(dIn_printfBuff) );

	// Inicializo los latches borrandolos
	pv_clearLatchs();
	total_ticks = 0;

	for ( channel = 0; channel < NRO_DIGITAL_CHANNELS; channel++ ) {
		digital_frame.level[channel] = 0;
		digital_frame.ticks_time_H[channel] = 0;
	}

	for( ;; )
	{

		u_kick_Wdg(WDG_DIN);
		vTaskDelay( ( TickType_t)( 100 / portTICK_RATE_MS ) );
		pv_poll_dInputs();

	}

}
//------------------------------------------------------------------------------------
static void pv_poll_dInputs(void)
{

uint8_t channel;

	// Incremento los ticks dentro del intervalo.
	total_ticks++;

	// Paso 1: Leo los niveles logicos de las 4 entradas
	digital_frame.level[0] = IO_read_din0_level();
	digital_frame.level[1] = IO_read_din1_level();
	digital_frame.level[2] = IO_read_din2_level();
	digital_frame.level[3] = IO_read_din3_level();

	// Paso 2: Tiempo que el pin esta en LOW
	// Mido el tiempo en intervalos de 100ms que la señal esta en LOW.
	// Normalmente ( flotando ) estaria en HIGH
	for ( channel = 0; channel < NRO_DIGITAL_CHANNELS; channel++ ) {
		if ( digital_frame.level[channel] == 1 ) {
			digital_frame.ticks_time_H[channel]++;
		}
	}

	// Siempre borro los latches para evitar la posibilidad de quedar colgado.
	pv_clearLatchs();
	return;

}
//------------------------------------------------------------------------------------
static void pv_clearLatchs(void)
{
	// Pongo un pulso 1->0->1 en Q0/Q1 pin para resetear el latch
	// En reposo debe quedar en H.

	IO_clr_CLRD();
	taskYIELD();
	IO_set_CLRD();
}
//------------------------------------------------------------------------------------
// FUNCIONES PUBLICAS
//------------------------------------------------------------------------------------
void u_read_digital_frame( st_digital_frame *dst_frame, bool reset_counters )
{

	// Copia el frame digital al puntero dst.
	// Primero debo ajustar los ticks.
	// El tema es que por ej. conte 599 ticks pero deberian habers sido teoricamente 600,
	// entonces falta 1 tick lo que indica que en algun momento la entrada estubo baja pero
	// en realidad fue que por un tema del clock, el intervalo fue unos ms. menos.
	// Para que esto no pase, debo corregirlo.
	// Teoricamente deberia haber contado systemVars.timerPoll * 10 ticks en un intervalo.
	// pero lo que realmente conte fue total_ticks.
	// Ajusto los ticks_time_H de c/entrada con estos valores.


uint8_t channel;
float ajuste_ticks = systemVars.timerPoll * 10 / total_ticks;

	for ( channel = 0; channel < NRO_DIGITAL_CHANNELS; channel++ ) {
		digital_frame.ticks_time_H[channel] = (uint16_t) ( ajuste_ticks * digital_frame.ticks_time_H[channel] );
	}

	memcpy( dst_frame, &digital_frame, sizeof(st_digital_frame));

	if ( reset_counters) {
		total_ticks = 0;
		for ( channel = 0; channel < NRO_DIGITAL_CHANNELS; channel++ ) {
			digital_frame.ticks_time_H[channel] = 0;
		}
	}

}
//------------------------------------------------------------------------------------
void u_read_digitalIn( uint8_t channel)
{
	// Leo el valor de una entrada analogica.
	// Debo determinar segun cual canal, que INA corresponde.

uint8_t pin;

	switch ( channel ) {
	case 0:
		pin = IO_read_din0_level();
		break;
	case 1:
		pin = IO_read_din1_level();
		break;
	case 2:
		pin = IO_read_din2_level();
		break;
	case 3:
		pin = IO_read_din3_level();
		break;
	}

	snprintf_P( dIn_printfBuff,sizeof(dIn_printfBuff),PSTR("DIN_%d=%d\r\n\0"), channel, pin);
	FreeRTOS_write( &pdUART1, dIn_printfBuff, sizeof(dIn_printfBuff) );

}
//------------------------------------------------------------------------------------
bool u_config_digital ( uint8_t channel, char *s_dname )
{

	while ( xSemaphoreTake( sem_SYSVars, ( TickType_t ) 1 ) != pdTRUE )
		taskYIELD();

	if ( ( channel >=  0) && ( channel < NRO_DIGITAL_CHANNELS) ) {
		snprintf_P( systemVars.dChName[channel], PARAMNAME_LENGTH, PSTR("%s\0"),s_dname );
	}

	xSemaphoreGive( sem_SYSVars );
	return(true);

}
//------------------------------------------------------------------------------------
void u_digital_load_defaults(void)
{

uint8_t channel;

	for ( channel = 0; channel < NRO_DIGITAL_CHANNELS; channel++) {
		snprintf_P( systemVars.dChName[channel], PARAMNAME_LENGTH, PSTR("D%d\0"),channel );
	}

}
//------------------------------------------------------------------------------------

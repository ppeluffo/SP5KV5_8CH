/*
 * sp5KV5_8CH_analog.c
 *
 *  Created on: 10 de mar. de 2018
 *      Author: pablo
 *
 *  Funciones propias del sistema analogico.
 *
 */

#include "sp5KV5_8CH.h"

// Este factor es porque la resistencia shunt es de 7.3 por lo que con 20mA llegamos hasta 3646 y no a 4096
#define FACTOR_CORRECCION_RSHUNT	3646

//------------------------------------------------------------------------------------
// FUNCIONES PUBLICAS
//------------------------------------------------------------------------------------
void u_analog_init(void)
{

	// Configuro los 3 INA para promediar en 128 medidas ( el registro 0x00, CONF = 0x7927 )

char res[3];

	res[0] = ( 0x7929 & 0xFF00 ) >> 8;
	res[1] = ( 0x7927 & 0x00FF );
	INA3221_write( INA3221_ADDR_0, 0x00, res, 2 );
	INA3221_write( INA3221_ADDR_1, 0x00, res, 2 );
	INA3221_write( INA3221_ADDR_2, 0x00, res, 2 );

}
//------------------------------------------------------------------------------------
void u_analog_polear(st_analog_frame *analog_frame )
{
	// Lee los datos de todos los canales analogicos.
	// Completa los valores raw y mag ya que usa u_read_analogIn que hace la conversion
	// raw->mag

uint8_t channel;

	for ( channel = 0; channel < NRO_ANALOG_CHANNELS; channel++ ) {
		u_read_analogIn(channel,false, &analog_frame->raw_val[channel], &analog_frame->mag_val[channel] );
	}

}
//------------------------------------------------------------------------------------
void u_analog_load_defaults(void)
{

uint8_t channel;

	systemVars.timerPoll = 60;

	for ( channel = 0; channel < NRO_ANALOG_CHANNELS; channel++) {
		systemVars.coef_calibracion[channel] = 3646;
		systemVars.Imin[channel] = 0;
		systemVars.Imax[channel] = 20;
		systemVars.Mmin[channel] = 0;
		systemVars.Mmax[channel] = 6;
		snprintf_P( systemVars.aChName[channel], PARAMNAME_LENGTH, PSTR("A%d\0"),channel );
	}

}
//------------------------------------------------------------------------------------
bool u_config_analog( uint8_t channel, char *s_aname, char *s_imin, char *s_imax, char *s_mmin, char *s_mmax )
{
	// Configura los canales analogicos. Se usa desde el modo comando o desde online gprs.
	// u_config_analog( channel aname imin imax mmin mmax )

bool retS = false;

	while ( xSemaphoreTake( sem_SYSVars, ( TickType_t ) 1 ) != pdTRUE )
		taskYIELD();

	if ( ( channel >=  0) && ( channel < NRO_ANALOG_CHANNELS) ) {
		snprintf_P( systemVars.aChName[channel], PARAMNAME_LENGTH, PSTR("%s\0"),s_aname );
		if ( s_imin != NULL ) { systemVars.Imin[channel] = atoi(s_imin); }
		if ( s_imax != NULL ) { systemVars.Imax[channel] = atoi(s_imax); }
		if ( s_mmin != NULL ) { systemVars.Mmin[channel] = atoi(s_mmin); }
		if ( s_mmax != NULL ) { systemVars.Mmax[channel] = atoi(s_mmax); }
		retS =true;
	}

	xSemaphoreGive( sem_SYSVars );
	return(retS);

}
//----------------------------------------------------------------------------------------
bool u_config_timerpoll( char *s_timerpoll )
{

	// El tiempo de poleo debe estar entre 10s y 3600s

	while ( xSemaphoreTake( sem_SYSVars, ( TickType_t ) 1 ) != pdTRUE )
		taskYIELD();

	systemVars.timerPoll = atoi(s_timerpoll);

	if ( systemVars.timerPoll < 10 )
		systemVars.timerPoll = 10;

	if ( systemVars.timerPoll > 3600 )
		systemVars.timerPoll = 300;

	xSemaphoreGive( sem_SYSVars );
	return(true);
}
//----------------------------------------------------------------------------------------
void u_config_cspan(char *s_channel, char *s_span)
{
uint8_t channel;
uint16_t span;

	while ( xSemaphoreTake( sem_SYSVars, ( TickType_t ) 1 ) != pdTRUE )
		taskYIELD();

	channel = atoi(s_channel);
	span = atoi(s_span);
	systemVars.coef_calibracion[channel] = span;

	xSemaphoreGive( sem_SYSVars );

}
//----------------------------------------------------------------------------------------
void u_read_analogIn(uint8_t channel, bool debug_flag, uint16_t *raw_val, float *mag_val )
{
	// Leo el valor de una entrada analogica.

uint8_t ina_id;
uint8_t ina_reg;
uint16_t an_raw_val;
float an_mag_val;
float I,M,P;
uint16_t D;

	//snprintf_P( debug_printfBuff,sizeof(debug_printfBuff),PSTR("DEBUG1: channel=%d, flag=%d\r\n"),channel,debug_flag );
	//FreeRTOS_write( &pdUART1, debug_printfBuff, sizeof(debug_printfBuff) );

	switch ( channel ) {
	case 0:
		ina_id = 0; ina_reg = INA3221_CH1_SHV;break;
	case 1:
		ina_id = 0; ina_reg = INA3221_CH2_SHV;break;
	case 2:
		ina_id = 0; ina_reg = INA3221_CH3_SHV;break;
	case 3:
		ina_id = 1; ina_reg = INA3221_CH1_SHV;break;
	case 4:
		ina_id = 1; ina_reg = INA3221_CH2_SHV;break;
	case 5:
		ina_id = 1; ina_reg = INA3221_CH3_SHV;break;
	case 6:
		ina_id = 2; ina_reg = INA3221_CH1_SHV;break;
	case 7:
		ina_id = 2; ina_reg = INA3221_CH2_SHV;break;
	case 8:
		ina_id = 2; ina_reg = INA3221_CH3_SHV;break;
	}

	// Leo el valor del INA.
	an_raw_val = INA3221_test_read( ina_id, ina_reg );

	//snprintf_P( debug_printfBuff,sizeof(debug_printfBuff),PSTR("DEBUG: an_raw_val=%d\r\n"),an_raw_val );
	//FreeRTOS_write( &pdUART1, debug_printfBuff, sizeof(debug_printfBuff) );

	*raw_val = an_raw_val;

	// Convierto el raw_value a la magnitud
	// Calculo la corriente medida en el canal
	I = (float)( an_raw_val) * 20 / ( systemVars.coef_calibracion[channel] + 1);

	// Calculo la magnitud
	P = 0;
	D = systemVars.Imax[channel] - systemVars.Imin[channel];

	//snprintf_P( debug_printfBuff,sizeof(debug_printfBuff),PSTR("DEBUG: D=%d, Imax=%d, Imin=%d\r\n"),D,systemVars.Imax[channel],systemVars.Imin[channel] );
	//FreeRTOS_write( &pdUART1, debug_printfBuff, sizeof(debug_printfBuff) );

	an_mag_val = 0.0;
	if ( D != 0 ) {
		// Pendiente
		P = (float) ( systemVars.Mmax[channel]  -  systemVars.Mmin[channel] ) / D;
		// Magnitud
		M = (float) (systemVars.Mmin[channel] + ( I - systemVars.Imin[channel] ) * P);
		an_mag_val = M;

	} else {
		// Error: denominador = 0.
		an_mag_val = -999.0;
	}

	//snprintf_P( debug_printfBuff,sizeof(debug_printfBuff),PSTR("DEBUG: P= %.02f, Mmax=%d, Mmin=%d\r\n"),P, systemVars.Mmax[channel],systemVars.Mmin[channel] );
	//FreeRTOS_write( &pdUART1, debug_printfBuff, sizeof(debug_printfBuff) );
	//snprintf_P( debug_printfBuff,sizeof(debug_printfBuff),PSTR("DEBUG: channel=%d, raw=%d, D=%d, P=%.02f, M=%.02f, mag=%0.2f\r\n"),channel,an_raw_val,D,P,M,an_mag_val );
	//FreeRTOS_write( &pdUART1, debug_printfBuff, sizeof(debug_printfBuff) );

	*mag_val = an_mag_val;

	if ( debug_flag ) {
		//memset(debug_printfBuff, '\0', sizeof(debug_printfBuff));
		snprintf_P( debug_printfBuff,sizeof(debug_printfBuff),PSTR("ch_%d: raw=%d, mag=%.02f\r\n\0"),channel,an_raw_val,*mag_val );
		FreeRTOS_write( &pdUART1, debug_printfBuff, sizeof(debug_printfBuff) );
	}

}
//------------------------------------------------------------------------------------
uint16_t u_read_INA3221(char *s_inaId, char *s_inaReg)
{
	// Interface al driver INA3221 para leer un registro indicado por nombre.
	// read ina {0,1,2} {conf|chxshv|chxbusv|mfid|dieid}

uint8_t ina_id;

	ina_id = atoi(s_inaId);

//	snprintf_P( debug_printfBuff,sizeof(debug_printfBuff),PSTR("INA %d, REG=%s\r\n\0"),ina_id, strupr( s_inaReg ) );
//	FreeRTOS_write( &pdUART1, debug_printfBuff, sizeof(debug_printfBuff) );


	if (!strcmp_P( strupr( s_inaReg ), PSTR("CONF\0"))) {
		return( INA3221_test_read( ina_id, INA3231_CONF_ADDR ) );
	}

	if (!strcmp_P( strupr( s_inaReg ), PSTR("CH1SHV\0"))) {
		return( INA3221_test_read( ina_id, INA3221_CH1_SHV ) );
	}

	if (!strcmp_P( strupr( s_inaReg ), PSTR("CH1BUSV\0"))) {
		return( INA3221_test_read( ina_id, INA3221_CH1_BUSV ) );
	}

	if (!strcmp_P( strupr( s_inaReg ), PSTR("CH2SHV\0"))) {
		return( INA3221_test_read( ina_id, INA3221_CH2_SHV ) );
	}

	if (!strcmp_P( strupr( s_inaReg ), PSTR("CH2BUSV\0"))) {
		return( INA3221_test_read( ina_id, INA3221_CH2_BUSV ) );
	}

	if (!strcmp_P( strupr( s_inaReg ), PSTR("CH3SHV\0"))) {
		return( INA3221_test_read( ina_id, INA3221_CH3_SHV ) );
	}

	if (!strcmp_P( strupr( s_inaReg ), PSTR("CH3BUSV\0"))) {
		return( INA3221_test_read( ina_id, INA3221_CH3_BUSV ) );
	}

	if (!strcmp_P( strupr( s_inaReg ), PSTR("MFID\0"))) {
		return( INA3221_test_read( ina_id, INA3221_MFID ) );
	}

	if (!strcmp_P( strupr( s_inaReg ), PSTR("DIEID\0"))) {
		return( INA3221_test_read( ina_id, INA3221_DIEID ) );
	}

	return(0);

}
//----------------------------------------------------------------------------------



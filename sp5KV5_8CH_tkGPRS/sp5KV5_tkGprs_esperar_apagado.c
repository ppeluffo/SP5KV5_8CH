/*
 * sp5KV5_tkGprs_esperar_apagado.c
 *
 *  Created on: 28 de abr. de 2017
 *      Author: pablo
 */

#include "../sp5KV5_8CH_tkGPRS/sp5KV5_tkGprs.h"


static int32_t waiting_time;

static void pv_calcular_tiempo_espera(void);
static bool pv_procesar_signals_espera( bool *exit_flag );

static bool starting_flag = true;

//------------------------------------------------------------------------------------
bool gprs_esperar_apagado(void)
{
	// Calculo el tiempo a esperar y espero. El intervalo no va a considerar el tiempo
	// posterior de proceso.

bool exit_flag = false;

// Entry:

	GPRS_stateVars.state = G_ESPERA_APAGADO;
	u_uarts_ctl(MODEM_APAGAR);

	// Secuencia para apagar el modem y dejarlo en modo low power.
	if ( (systemVars.debugLevel & (D_BASIC + D_GPRS) ) != 0) {
		snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR("%s GPRS::wait: Apago modem\r\n\0"), u_now());
		FreeRTOS_write( &pdUART1, gprs_printfBuff, sizeof(gprs_printfBuff) );
	}

	GPRS_stateVars.modem_prendido = false;
	strncpy_P(systemVars.dlg_ip_address, PSTR("000.000.000.000\0"),16);
	systemVars.csq = 0;
	systemVars.dbm = 0;
	// Para que no consuma

	// Apago por SW.
	IO_modem_sw_switch_high();
	vTaskDelay( (portTickType)( 100 / portTICK_RATE_MS ) );
	IO_modem_sw_switch_low();
	vTaskDelay( (portTickType)( 1000 / portTICK_RATE_MS ) );
	IO_modem_sw_switch_high();

	// Apago por HW.
	IO_modem_hw_pwr_off();
	vTaskDelay( (portTickType)( 2000 / portTICK_RATE_MS ) );
	IO_modem_hw_pwr_on();

	pv_calcular_tiempo_espera();

	// Espera
	do {

		// Espero 1s.
		vTaskDelay( (portTickType)( 1000 / portTICK_RATE_MS ) );
		waiting_time--;

		// Analizo si termine el periodo y debo salir
		if ( waiting_time == 0 ) {

			// 1: Si estoy en un arranque salgo y listo.
			if ( starting_flag == true ) {
				starting_flag = false;
				break;
			}
		}

		// PROCESO LAS SEÑALES
		if ( pv_procesar_signals_espera( &exit_flag )) {
			// Si recibi alguna senal, debo salir.
			goto EXIT;
		}

	} while (waiting_time > 0);

	//
	exit_flag = bool_CONTINUAR;

EXIT:

	// No espero mas y salgo del estado prender.
	waiting_time = -1;
	return(exit_flag);

}
//------------------------------------------------------------------------------------
static void pv_calcular_tiempo_espera(void)
{

	// Cuando arranco ( la primera vez) solo espero 10s y disco por primera vez
	if ( starting_flag ) {
		waiting_time = 10;
		return;
	}

	// Calculo el tiempo que voy a tener que estar esperando
	// NORMAL:
	switch ( systemVars.wrkMode ) {
	case WK_NORMAL:
		waiting_time = 30;
		break;

	case WK_MONITOR_SQE:
		// Debo prender cuanto antes ( 10s ) para ir a monitorear el sqe
		waiting_time = 10;
		break;

	default:
		waiting_time = 15;
		break;
	}

	if ( (systemVars.debugLevel & (D_BASIC + D_GPRS) ) != 0) {
		snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR("%s GPRS::wait: %lu s\r\n\0"), u_now(), waiting_time );
		FreeRTOS_write( &pdUART1, gprs_printfBuff, sizeof(gprs_printfBuff) );
	}

}
//------------------------------------------------------------------------------------
static bool pv_procesar_signals_espera( bool *exit_flag )
{

bool ret_f = false;

	if ( GPRS_stateVars.signal_redial) {
		// Salgo a discar inmediatamente.
		*exit_flag = bool_CONTINUAR;
		ret_f = true;
		goto EXIT;
	}


	ret_f = false;
EXIT:

	GPRS_stateVars.signal_redial = false;

	return(ret_f);
}
//------------------------------------------------------------------------------------
// FUNCIONES PUBLICAS
//----------------------------------------------------------------------------------------

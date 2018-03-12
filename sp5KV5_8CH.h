/*
 * sp5K.h
 *
 * Created on: 27/12/2013
 *      Author: root
 */

#ifndef SP5K_H_
#define SP5K_H_

#include <avr/io.h>			/* include I/O definitions (port names, pin names, etc) */
//#include <avr/signal.h>		/* include "signal" names (interrupt names) */
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/eeprom.h>
#include <stdarg.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <avr/sleep.h>
#include <string.h>
#include <compat/deprecated.h>
#include <util/twi.h>
#include <util/delay.h>
#include <ctype.h>
#include <util/delay.h>
#include <avr/cpufunc.h>

#include <l_adc7828.h>
#include <l_file.h>
#include <l_rtc.h>
#include <l_iopines.h>
#include <l_mcp.h>
#include <l_ina3221.h>

#include "sp5Klibs/avrlibdefs.h"
#include "sp5Klibs/avrlibtypes.h"
#include "sp5Klibs/global.h"			// include our global settings
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "list.h"
#include "croutine.h"
#include "semphr.h"
#include "timers.h"
#include "limits.h"

#include "FRTOS-IO.h"

#include "cmdline.h"

// DEFINICION DEL TIPO DE SISTEMA
//----------------------------------------------------------------------------
#define SP5K_REV "6.0.0"
#define SP5K_DATE "@ 20180312"

#define SP5K_MODELO "sp5KV5_8CH HW:avr1284P R5.0"
#define SP5K_VERSION "FW:FRTOS8"

#define NRO_ANALOG_CHANNELS		8
#define NRO_DIGITAL_CHANNELS 	4

#define CHAR64		64
#define CHAR128	 	128
#define CHAR256	 	256

//----------------------------------------------------------------------------
// TASKS
/* Stack de las tareas */
#define tkCmd_STACK_SIZE		512
#define tkControl_STACK_SIZE	512
#define tkDigitalIn_STACK_SIZE	512
#define tkData_STACK_SIZE		512
#define tkGprs_STACK_SIZE		512
#define tkGprsRx_STACK_SIZE		512

/* Prioridades de las tareas */
#define tkCmd_TASK_PRIORITY	 		( tskIDLE_PRIORITY + 1 )
#define tkControl_TASK_PRIORITY	 	( tskIDLE_PRIORITY + 1 )
#define tkDigitalIn_TASK_PRIORITY 	( tskIDLE_PRIORITY + 1 )
#define tkData_TASK_PRIORITY 		( tskIDLE_PRIORITY + 1 )
#define tkGprs_TASK_PRIORITY 		( tskIDLE_PRIORITY + 1 )
#define tkGprsRx_TASK_PRIORITY 		( tskIDLE_PRIORITY + 1 )

/* Prototipos de tareas */
void tkCmd(void * pvParameters);
void tkControl(void * pvParameters);
void tkDigitalIn(void * pvParameters);
void tkData(void * pvParameters);
void tkGprsTx(void * pvParameters);
//void tkGprsInit(void);
void tkGprsRx(void * pvParameters);

TaskHandle_t xHandle_tkCmd, xHandle_tkControl, xHandle_tkDigitalIn, xHandle_tkData, xHandle_tkGprs, xHandle_tkGprsRx;

bool startTask;
typedef struct {
	uint8_t resetCause;
	uint8_t mcusr;
} wdgStatus_t;

wdgStatus_t wdgStatus;

// Mensajes entre tareas
#define TK_PARAM_RELOAD			0x01	// param reload
#define TK_READ_FRAME			0x02	// to tkAnalogIN: (mode service) read a frame
#define TK_FRAME_READY			0x08	//
#define TK_REDIAL				0x10	//

//------------------------------------------------------------------------------------

xSemaphoreHandle sem_SYSVars;
#define MSTOTAKESYSVARSSEMPH ((  TickType_t ) 10 )

typedef enum { WK_NORMAL = 0, WK_MONITOR_SQE  } t_wrkMode;
typedef enum { D_NONE = 0, D_BASIC = 1, D_DATA = 2, D_GPRS = 4, D_MEM = 8, D_DIGITAL = 16, D_DEBUG = 32 } t_debug;
typedef enum { MODEM_PRENDER = 0, MODEM_APAGAR, TERM_PRENDER, TERM_APAGAR } t_uart_ctl;

#define DLGID_LENGTH		12
#define APN_LENGTH			32
#define PORT_LENGTH			7
#define IP_LENGTH			24
#define SCRIPT_LENGTH		64
#define PASSWD_LENGTH		15
#define PARAMNAME_LENGTH	5

// Estructura para manejar los canales digitales.
typedef struct {
	uint8_t level[NRO_DIGITAL_CHANNELS];		// nivel logico de la entrada
	uint16_t ticks_time_H[NRO_DIGITAL_CHANNELS];
} st_digital_frame;

// Estructura para manejar los canales analogicos
typedef struct {
	uint16_t raw_val[NRO_ANALOG_CHANNELS];
	float mag_val[NRO_ANALOG_CHANNELS];
} st_analog_frame;

// Estructura de datos
typedef struct {
	RtcTimeType_t rtc;
	float an_mag_val[NRO_ANALOG_CHANNELS];
	st_digital_frame digital_frame;
} st_data_frame;

typedef struct {
	// Variables de trabajo.
	// Tamanio: 302 bytes para 3 canales.

	uint8_t dummyBytes;
	uint8_t initByte;

	char dlgId[DLGID_LENGTH];
	char apn[APN_LENGTH];
	char server_tcp_port[PORT_LENGTH];
	char server_ip_address[IP_LENGTH];
	char dlg_ip_address[IP_LENGTH];
	char serverScript[SCRIPT_LENGTH];
	char passwd[PASSWD_LENGTH];

	uint8_t csq;
	uint8_t dbm;

	uint16_t timerPoll;

	t_wrkMode wrkMode;

	uint8_t logLevel;		// Nivel de info que presentamos en display.
	uint8_t debugLevel;		// Indica que funciones debugear.
	uint8_t gsmBand;

	// Configuracion de Canales analogicos
	char aChName[NRO_ANALOG_CHANNELS][PARAMNAME_LENGTH];
	uint16_t coef_calibracion[NRO_ANALOG_CHANNELS];
	uint8_t Imin[NRO_ANALOG_CHANNELS];				// Coeficientes de conversion de I->magnitud (presion)
	uint8_t Imax[NRO_ANALOG_CHANNELS];
	int16_t Mmin[NRO_ANALOG_CHANNELS];
	int16_t Mmax[NRO_ANALOG_CHANNELS];

	// Configuracion de canales digitales
	char dChName[NRO_DIGITAL_CHANNELS][PARAMNAME_LENGTH];

} systemVarsType;	// 315 bytes

systemVarsType systemVars,tmpSV;

uint32_t ticks;

#define EEADDR_SV 32		// Direccion inicio de la EE de escritura del systemVars.

//------------------------------------------------------------------------------------
// FUNCIONES DE USO GENERAL.
//------------------------------------------------------------------------------------
// utils
void u_uarts_ctl(uint8_t cmd);
void u_panic( uint8_t panicCode );
void u_kick_Wdg( uint8_t wdgId );
bool u_saveSystemParams(void);
bool u_loadSystemParams(void);
void u_loadDefaults(void);
char *u_now(void);
void u_debugPrint(uint8_t debugCode, char *msg, uint16_t size);
void u_reset(void);
// tkData
void u_tkData_read_frame(bool saveInBD );
void u_read_dataframe(st_data_frame *dst_data_frame);
void u_print_dataframe(st_data_frame *data_frame);
// Analog
void u_analog_init(void);
void u_analog_polear(st_analog_frame *analog_frame );
void u_analog_load_defaults(void);
bool u_config_analog( uint8_t channel, char *s_aname, char *s_imin, char *s_imax, char *s_mmin, char *s_mmax );
bool u_config_timerpoll( char *s_timerpoll );
void u_config_cspan(char *s_channel, char *s_span);
uint16_t u_read_INA3221(char *s_inaId, char *s_inaReg);
void u_read_analogIn(uint8_t channel, bool debug_flag, uint16_t *raw_val, float *mag_val );
// tkDigital
void u_read_digital_frame( st_digital_frame *dst_frame, bool reset_counters );
void u_read_digitalIn( uint8_t channel );
void u_digital_load_defaults(void);
bool u_config_digital (uint8_t channel, char *s_dname );
// tkGprs

char nowStr[32];
char debug_printfBuff[CHAR128];

//------------------------------------------------------------------------------------
// PANIC CODES
#define P_AIN_TIMERSTART		1
#define P_AIN_TIMERCREATE		2

//------------------------------------------------------------------------------------
// WATCHDOG
uint8_t systemWdg;

#define WDG_CTL			0x01
#define WDG_CMD			0x02
#define WDG_DIN			0x04
#define WDG_AIN			0x10
//#define WDG_GPRS		0x20
//#define WDG_GPRSRX	0x40

//------------------------------------------------------------------------------------

#endif /* SP5K_H_ */

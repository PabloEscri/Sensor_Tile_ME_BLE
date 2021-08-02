/*
 * RTC_APP.h
 *
 *  Created on: 6 mar. 2021
 *      Author: pablo
 */

#ifndef RTC_APP_H_
#define RTC_APP_H_
#include "stm32l4xx_hal.h"
#include <string.h>


typedef struct
{
	int dia;
	int mes;
	int year;
	int segundo;
	int minuto;
	int hora;
}momento_t;
void RTC_Init(void);


int RTC_configurar_dia_desde_cadena(const char * dia_str);

int RTC_obtener_hora_str_formato_SD(char * buffer);
int RTC_obtener_dia_str(char * buffer);
int RTC_obtener_momento(momento_t * momento);
int RTC_obtener_hora_str(char * buffer);
int RTC_obtener_fecha_str(char * buffer);

#endif /* RTC_APP_H_ */

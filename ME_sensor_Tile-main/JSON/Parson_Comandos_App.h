/*
 * Parson_Comandos_App.h
 *
 *  Created on: 5 dic. 2020
 *      Author: pablo
 */

#ifndef PARSON_COMANDOS_APP_H_
#define PARSON_COMANDOS_APP_H_

#include"parson.h"
#include <string.h>
//#include "stm32l4xx_hal_uart.h"
#include "stm32l4xx_hal.h"
/* @param pData Este es el byte que acaba de recibirse.
 * @param maximum_len Indica cuantos caracteres como máximo puede tener tu JSON antes de desbordar.
 * @param resetear Si tiene el valor 1 -> Las variables que cuentan llaves de apertura y cierre se resetean a 0. Si vale 0 -> No afecta.
 *
 * @return 0 si el JSON no ha empezado (JSON no abierto aún)
 * @return 1 si el JSON está incompleto (JSON en curso)
 * @return 2 si se ha cerrado el JSON (JSON completo)
 * @return 3 si se ha sobrepasado el size de tu buffer (JSON demasiado grande)
 * @return 4 si se han reseteado los contadores de {} (Se acaba de reiniciar los contadores)
 * */
int check_JSON(const char pData, const int maximum_len, const int resetear);


/*Hay que revisar lo de hacer un free de la memoria.*/

/* @param json_file Puntero a un buffer que contenga un JSON.
 * @param vector_comandos Puntero a un vector de punteros. Ejemplo de parámetro: const char * vector[7] = {"SD","GD","SS","GS","GL","LS","DL"};
 * @param resetear Si tiene el valor 1 -> Las variables que cuentan llaves de apertura y cierre se resetean a 0. Si vale 0 -> No afecta.
 *
 * @return 0 si el JSON no contiene una instrucción "c". Esto es, no contiene un comando. -> Formato JSON Incorrecto para esta App.
 * @return 1 si el JSON contiene una instrucción "c" pero el comando no se reconoce. Ejemplo: {"c":"hola"} . "hola" no es un comando valido.
 * @return 2 si se ha reconocido el comando.
 * */
int parsear_comandos_json(const char * json_file, const char ** vector_comandos, const int numero_comandos);

/*
* @return -6 si el JSON está completo pero no se ha recibido un comando válido.
* @return -5 si el JSON está completo pero Falta instrucción "c".
* @return -4 si el JSON no ha empezado (JSON no abierto aún)
* @return -3 si el JSON está incompleto (JSON en curso)
* @return -2 si se ha sobrepasado el size de tu buffer (JSON demasiado grande)
* @return -1 si se han reseteado los contadores de {} (Se acaba de reiniciar los contadores)
* @return  n El número de comando de tu vector de comandos que ha sido recibido.
*/
int process_mesage(const char pData, char * buffer_json, const uint32_t buffer_size, const char ** vector_comandos, const int numero_comandos, const int resetear);

#endif /* PARSON_COMANDOS_APP_H_ */

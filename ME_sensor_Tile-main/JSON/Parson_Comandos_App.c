/*
 * Parson_Comandos_App.c
 *
 *  Created on: 5 dic. 2020
 *      Author: pablo
 */
#include "Parson_Comandos_App.h"


extern UART_HandleTypeDef huart1;
#define PRINT(...)		//do{char texto[20]; int size = sprintf(texto,__VA_ARGS__); HAL_UART_Transmit(&huart1, texto, size, 0xFFFF); }while(0);


/*Hay que revisar lo de hacer un free de la memoria.*/

/* @param json_file Puntero a un buffer que contenga un JSON.
 * @param vector_comandos Puntero a un vector de punteros. Ejemplo de parámetro: const char * vector[7] = {"SD","GD","SS","GS","GL","LS","DL"};
 * @param resetear Si tiene el valor 1 -> Las variables que cuentan llaves de apertura y cierre se resetean a 0. Si vale 0 -> No afecta.
 *
 * @return -1 si el JSON no contiene una instrucción "c". Esto es, no contiene un comando. -> Formato JSON Incorrecto para esta App.
 * @return -2 si el JSON contiene una instrucción "c" pero el comando no se reconoce. Ejemplo: {"c":"hola"} . "hola" no es un comando valido.
 * @return n si se ha reconocido el comando.
 * */

#define parsear_comandos_json_MSG_ON 0

int parsear_comandos_json(const char * json_file, const char ** vector_comandos, const int numero_comandos)
{

	int status_Comando_Json = -1;

	JSON_Value * val_1;
	val_1 = json_parse_string(json_file);

	//Chequear si es un objeto
	JSON_Object *root_object;
	root_object = json_value_get_object(val_1);

	if(json_object_has_value(root_object, "c") != 0 )
	{
		JSON_Value  * valor_2 = json_object_dotget_value(root_object, "c");

		//Falta un IF de solo si es una cadena!!!!!!!!!!!!!!!!!!!!!!!!!!!
		 const char * buffer_ptr = json_value_get_string (valor_2);

		for(int i = 0; i<numero_comandos;i++ )
		{
			if(strcmp(vector_comandos[i],buffer_ptr) == 0)
			{

				status_Comando_Json = i;
				break;
			}
		}
		if(status_Comando_Json < 0)
		{
			#if parsear_comandos_json_MSG_ON
				PRINT("COMANDO NO VALIDO\r\n")
			#endif
			status_Comando_Json = -2;
		}


	}
	else
	{
		#if parsear_comandos_json_MSG_ON
			PRINT("NO COMANDO RECIBIDO\r\n")
		#endif
		status_Comando_Json = -1;
	}
	return status_Comando_Json;
}


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

#define check_JSON_MSG_ON 0

int check_JSON(const char pData, const int maximum_len, const int resetear)
{
		static int num_llaves_abiertas = 0;	// Contador de llaves abiertas
		static int num_llaves_cerradas = 0; // Contador de llaves cerradas
		static int index_buffer = 0;		// Contador de tamaño de buffer
		int Json_current_status = 1; // JSON está en curso

		if(resetear != 1)			// Si no hay un reset por parte del usuario...
		{

			if((char)pData == '{')
			{
				num_llaves_abiertas++;
			}

			if(num_llaves_abiertas > 0)
			{
				index_buffer++ ;

				if((char)pData == '}')
				{
					num_llaves_cerradas++;
					if(num_llaves_cerradas == num_llaves_abiertas)
					{
						#if check_JSON_MSG_ON
							PRINT("CJ: JSON  COMPLETO\r\n");
						#endif
						index_buffer = 0;
						num_llaves_abiertas = 0;
						num_llaves_cerradas = 0;
						Json_current_status = 2;  // JSON completo
					}
				}

				if(index_buffer == maximum_len)
				{
					#if check_JSON_MSG_ON
						PRINT("CJ: JSON EXCESIVO GRANDE\r\n");
					#endif
					index_buffer = 0;
					num_llaves_abiertas = 0;
					num_llaves_cerradas = 0;
					Json_current_status = 3;  // JSON demasiado grande
				}
			}
			else
			{
				#if check_JSON_MSG_ON
					PRINT("CJ: JSON NO ABIERTO\r\n");
				#endif
				Json_current_status = 0; // JSON no se ha abierto
			}
		}
		else
		{
			#if check_JSON_MSG_ON
				PRINT("CJ: JSON RESETEADO\r\n");
			#endif
			index_buffer = 0;
			num_llaves_abiertas = 0;
			num_llaves_cerradas = 0;
			Json_current_status = 4;  // JSON Reseteado
		}

		return Json_current_status;
}

/*@param pData Es el nuevo byte que se acaba de recibir
* @param buffer_json es el Buffer externo donde se guardan todos los bytes recibidos por pData
* @param buffer_size Tamaño del buffer. En caso de excederse este buffer se resetea la interfaz por seguridad.
* @param vector_comandos Contiene todos los posibles comandos que puedan ser recibidos.
* @param numero_comnados Indica cuantos comandos tiene el parámetro vector_comandos
* @param resetear Si es 1 se resetea la Interfaz, si es 0 no se resetea la interfaz.
*
* @return -6 si el JSON está completo pero no se ha recibido un comando válido.
* @return -5 si el JSON está completo pero Falta instrucción "c".
* @return -4 si el JSON no ha empezado (JSON no abierto aún)
* @return -3 si el JSON está incompleto (JSON en curso)
* @return -2 si se ha sobrepasado el size de tu buffer (JSON demasiado grande)
* @return -1 si se han reseteado los contadores de {} (Se acaba de reiniciar los contadores)
*/
/*
* @return n Número de comando de tu vector de comandos que ha sido recibido.
*/
#define process_message_MSG_ON 0
int process_mesage(char pData,  char * buffer_json, const uint32_t buffer_size, const char ** vector_comandos, const int numero_comandos, const int resetear)
{
	int process_message =-6; // Inicio del estado de la función
	static int index = 0;
	int JSON_status = check_JSON(pData, buffer_size, resetear);

	switch(JSON_status)
	{
		case 0: //JSON NO ABIERTO
			process_message = -4;
#if process_message_MSG_ON
			PRINT("PM: JSON NO ABIERTO \r\n");
			#endif
		break;
		case 1: //JSON ABIERTO Y EN CURS0. JSON INCOMPLETO.
#if process_message_MSG_ON
			PRINT("PM: JSON ABIERTO \r\n");
#endif
			buffer_json[index] = pData;
			index++;
			process_message = -3;
		break;
		case 2://JSON TERMINADO
#if process_message_MSG_ON
			PRINT("PM: JSON TERMINADO \r\n");
#endif
			buffer_json[index] = pData;
			index = 0;
			int status = parsear_comandos_json(buffer_json, vector_comandos, numero_comandos);
			switch(status)
			{
				case -1:
					// No contiene Instrucción "c"
#if process_message_MSG_ON
					PRINT("PM: FALTA INSTRUCCION \"C\" \r\n");
#endif
					process_message = -5;
				break;
				case -2:
					// Contiene Instrucción "c" pero COMANDO NO VÁLIDO
					process_message = -6;
#if process_message_MSG_ON
					PRINT("PM: COMANDO NO VALIDO \r\n");
#endif
				break;
				default: // Devuelve número de comando encontrado:
					process_message = status;
#if process_message_MSG_ON
					PRINT("PM: COMANDO ENCONTRADO\r\n");
#endif
				break;
			}

		break;
		case 3://JSON DEMASIADO GRANDE
			index = 0;
			process_message = -2;
#if process_message_MSG_ON
			PRINT("PM: JSON DEMASIADO GRANDE \r\n");
#endif
			break;
		case 4: // Contadores reseteados
			index = 0;
			process_message = -1;
#if process_message_MSG_ON
			PRINT("PM: CONTADORES RESETEADOS \r\n");
#endif
		break;
		}
	return process_message;
}

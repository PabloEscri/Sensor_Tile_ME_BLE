/**
  ******************************************************************************
  * @file    sensor_service.c
  * @author  SRA
  * @version v2.4.0
  * @date    31-Jul-2020
  * @brief   Add bluetooth services using vendor specific profiles.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
#include <stdio.h>
#include "main.h"
#include "sensor_service.h"
#include "bluenrg_utils.h"
#include "bluenrg_l2cap_aci.h"
#include "uuid_ble_service.h"
#include "BLE_APP.h"
#include "datalog_application.h"
#include "RTC_APP.h"
/* Exported variables ---------------------------------------------------------*/
int connected = FALSE;
uint8_t set_connectable = TRUE;

/* Imported Variables -------------------------------------------------------------*/
extern uint32_t ConnectionBleStatus;

extern TIM_HandleTypeDef    TimCCHandle;

extern uint8_t bdaddr[6];

/* Private variables ------------------------------------------------------------*/
static uint16_t HWServW2STHandle;
static uint16_t EnvironmentalCharHandle;
static uint16_t LedCharHandle;

static uint16_t ConfigServW2STHandle;
static uint16_t ConfigCharHandle;

static uint16_t ConsoleW2STHandle;
static uint16_t TermCharHandle;
static uint16_t StdErrCharHandle;

static uint8_t LastStderrBuffer[W2ST_CONSOLE_MAX_CHAR_LEN];
static uint8_t LastStderrLen;
static uint8_t LastTermBuffer[W2ST_CONSOLE_MAX_CHAR_LEN];
static uint8_t LastTermLen;

static uint8_t  EnvironmentalCharSize=2; /* Size for Environmental BLE characteristic */

static uint16_t connection_handle = 0;

/* Private functions ------------------------------------------------------------*/
static void GAP_ConnectionComplete_CB(uint8_t addr[6], uint16_t handle);
static void GAP_DisconnectionComplete_CB(void);
static void ConfigCommandParsing(uint8_t * att_data, uint8_t data_length);

/* Private define ------------------------------------------------------------*/

#ifdef ACC_BLUENRG_CONGESTION
#define ACI_GATT_UPDATE_CHAR_VALUE safe_aci_gatt_update_char_value
static int32_t breath;


/* @brief  Update the value of a characteristic avoiding (for a short time) to
 *         send the next updates if an error in the previous sending has
 *         occurred.
 * @param  servHandle The handle of the service
 * @param  charHandle The handle of the characteristic
 * @param  charValOffset The offset of the characteristic
 * @param  charValueLen The length of the characteristic
 * @param  charValue The pointer to the characteristic
 * @retval tBleStatus Status
 */
tBleStatus safe_aci_gatt_update_char_value(uint16_t servHandle, 
                                           uint16_t charHandle,
                                           uint8_t charValOffset,
                                           uint8_t charValueLen,   
                                           const uint8_t *charValue)
{
  tBleStatus ret = BLE_STATUS_INSUFFICIENT_RESOURCES;
  
  if (breath > 0) 
  {
    breath--;
  } 
  else 
  {
    ret = aci_gatt_update_char_value(servHandle,charHandle,charValOffset,charValueLen,charValue);
    
    if (ret != BLE_STATUS_SUCCESS)
    {
      breath = ACC_BLUENRG_CONGESTION_SKIP;
    }
  }
  
  return (ret);
}

#else /* ACC_BLUENRG_CONGESTION */
#define ACI_GATT_UPDATE_CHAR_VALUE aci_gatt_update_char_value
#endif /* ACC_BLUENRG_CONGESTION */


/**
* @brief  Add the Config service using a vendor specific profile
* @param  None
* @retval tBleStatus Status
*/
static uint16_t ConfigServW2STHandle;
static uint16_t ConfigCharHandle,ConfigCharHandle2;

tBleStatus Add_ConfigW2ST_Service(void)
{
#if 0
	tBleStatus ret;

	  uint8_t uuid[16];

	  COPY_CONFIG_SERVICE_UUID(uuid);
	  ret = aci_gatt_add_serv(UUID_TYPE_128,  uuid, PRIMARY_SERVICE, 6,&ConfigServW2STHandle);

	  if (ret != BLE_STATUS_SUCCESS)
	    goto fail;
	  //Lectura
	  COPY_CONFIG_W2ST_CHAR_UUID(uuid);
	  ret =  aci_gatt_add_char(ConfigServW2STHandle, UUID_TYPE_128, uuid, 20 /* Max Dimension */,
	                           CHAR_PROP_NOTIFY| CHAR_PROP_WRITE_WITHOUT_RESP,
	                           ATTR_PERMISSION_NONE,
							   GATT_DONT_NOTIFY_EVENTS,
	                           16, 1, &ConfigCharHandle);

	  if (ret != BLE_STATUS_SUCCESS)
	  {
	    goto fail;
	  }
	  char vector[9] = {'R','X',' ','B','U','F','F','E','R'};
	  aci_gatt_update_char_value(ConfigServW2STHandle, ConfigCharHandle, 0, 9,vector);
	  //Escritura
	  COPY_CONFIG_W3ST_CHAR_UUID(uuid);
	  	  ret =  aci_gatt_add_char(ConfigServW2STHandle, UUID_TYPE_128, uuid, 20 /* Max Dimension */,
	  	                           CHAR_PROP_WRITE_WITHOUT_RESP,
	  	                           ATTR_PERMISSION_NONE,
	  	                           GATT_NOTIFY_ATTRIBUTE_WRITE ,
	  	                           16, 1, &ConfigCharHandle2);

	 if (ret != BLE_STATUS_SUCCESS)
	 {
	  	    goto fail;
	 }

#endif
	  return BLE_STATUS_SUCCESS;

	fail:
	  //STLBLE_PRINTF("Error while adding Configuration service.\n");
	  return BLE_STATUS_ERROR;
}

/**
 * @brief  Update Stderr characteristic value
 * @param  uint8_t *data string to write
 * @param  uint8_t lenght lengt of string to write
 * @retval tBleStatus      Status
 */
tBleStatus Stderr_Update(uint8_t *data,uint8_t length)
{

  return BLE_STATUS_SUCCESS;
}

/**
* @brief  Update Terminal characteristic value
* @param  uint8_t *data string to write
* @param  uint8_t lenght lengt of string to write
* @retval tBleStatus      Status
*/
tBleStatus Term_Update(uint8_t *data,uint8_t length)
{

  return BLE_STATUS_SUCCESS;
}

/**
 * @brief  Update Stderr characteristic value after a read request
 * @param None
 * @retval tBleStatus      Status
 */
static tBleStatus Stderr_Update_AfterRead(void)
{
  
  return BLE_STATUS_SUCCESS;
}

/**
* @brief  Update Terminal characteristic value after a read request
* @param None
* @retval tBleStatus      Status
*/
static tBleStatus Term_Update_AfterRead(void)
{
  
  
  return BLE_STATUS_SUCCESS;
}

/* @brief  Send a notification for answering to a configuration command for Accelerometer events
 * @param  uint32_t Feature Feature calibrated
 * @param  uint8_t Command Replay to this Command
 * @param  uint8_t data result to send back
 * @retval tBleStatus Status
 */
tBleStatus Config_Notify(uint32_t Feature,uint8_t Command,uint8_t data)
{

  return BLE_STATUS_SUCCESS;
}

/**
* @brief  Add the HW Features service using a vendor specific profile
* @param  None
* @retval tBleStatus Status
*/
tBleStatus Add_HWServW2ST_Service(void)
{
	 tBleStatus ret;

	  uint8_t uuid[16];

	  COPY_CONFIG_SERVICE_UUID(uuid);
	  ret = aci_gatt_add_serv(UUID_TYPE_128,  uuid, PRIMARY_SERVICE, 8,&ConfigServW2STHandle);

	  if (ret != BLE_STATUS_SUCCESS)
	    goto fail;

	  COPY_CONFIG_W2ST_CHAR_UUID(uuid);
	  ret =  aci_gatt_add_char(ConfigServW2STHandle, UUID_TYPE_128, uuid, 20 /* Max Dimension */,
	                           CHAR_PROP_NOTIFY| CHAR_PROP_READ,
	                           ATTR_PERMISSION_NONE,
							   GATT_DONT_NOTIFY_EVENTS,
	                           16, 1, &ConfigCharHandle);

	  if (ret != BLE_STATUS_SUCCESS)
	  {
	    goto fail;
	  }
	  char vector[9] = {'R','X',' ','B','U','F','F','E','R'};
	 	  aci_gatt_update_char_value(ConfigServW2STHandle, ConfigCharHandle, 0, 9,vector);

	  COPY_CONFIG_W3ST_CHAR_UUID(uuid);
	 	  	  ret =  aci_gatt_add_char(ConfigServW2STHandle, UUID_TYPE_128, uuid, 20 /* Max Dimension */,
	 	  	                           CHAR_PROP_WRITE_WITHOUT_RESP,
	 	  	                           ATTR_PERMISSION_NONE,
	 	  	                           GATT_NOTIFY_ATTRIBUTE_WRITE ,
	 	  	                           16, 1, &ConfigCharHandle2);

	 	 if (ret != BLE_STATUS_SUCCESS)
	 	 {
	 	  	    goto fail;
	 	 }




	  return BLE_STATUS_SUCCESS;
	  fail:
	    //STLBLE_PRINTF("Error while adding Configuration service.\n");
	    return BLE_STATUS_ERROR;
}


/**
* @brief  Update Environmental characteristic value
* @param  int32_t Press Pressure in mbar
* @param  uint16_t Hum humidity RH (Relative Humidity) in thenths of %
* @param  int16_t Temp2 Temperature in tenths of degree second sensor
* @param  int16_t Temp1 Temperature in tenths of degree first sensor
* @retval tBleStatus   Status
*/
tBleStatus Environmental_Update(int32_t Press,uint16_t Hum,int16_t Temp2,int16_t Temp1)
{
  
  return BLE_STATUS_SUCCESS;
}

/**
* @brief  Update LEDs characteristic value
* @param  uint8_t LedStatus LEDs status 0/1 (off/on)
* @retval tBleStatus   Status
*/
tBleStatus LED_Update(uint8_t LedStatus)
{

  return BLE_STATUS_SUCCESS;
}


/**
 * @brief  Puts the device in connectable mode.
 * @param  None 
 * @retval None
 */
void setConnectable(void)
{  

}

/**
 * @brief  This function is called when there is a LE Connection Complete event.
 * @param  uint8_t addr[6] Address of peer device
 * @param  uint16_t handle Connection handle
 * @retval None
 */
static void GAP_ConnectionComplete_CB(uint8_t addr[6], uint16_t handle)
{  
  

}

/**
 * @brief  This function is called when the peer device get disconnected.
 * @param  None 
 * @retval None
 */
static void GAP_DisconnectionComplete_CB(void)
{
	StartBeaconing();
}



static void GAP_Scan( uint8_t evt_type, uint8_t bdaddr_type,uint8_t * bdaddr,uint8_t data_length,uint8_t *data_RSSI, int8_t rssi)
{

	char guardar_baddr[50];
	//write_SD_BT("-------------------------------------------------\r\n");
	char escribir_sd[100];
	 RTC_obtener_hora_str(escribir_sd);
	 write_SD_BT(escribir_sd);
	//sprintf(guardar_baddr,"Addr: %x:%x:%x:%x:%x:%x;",bdaddr[6],bdaddr[5],bdaddr[4],bdaddr[3],bdaddr[2],bdaddr[1] );
	 sprintf(guardar_baddr,";%x:%x:%x:%x:%x:%x;",bdaddr[5],bdaddr[4],bdaddr[3],bdaddr[2],bdaddr[1],bdaddr[0] );
	write_SD_BT(guardar_baddr);
	//write_SD_BT("Evt_type: ");
	 switch(evt_type)
	 {
	 	 	 case(ADV_IND):
				write_SD_BT("ADV_IND");
			 break;
	 	 	 case(ADV_DIRECT_IND):
				write_SD_BT("ADV_DIRECT_IND");
	 	 	 break;
	 	 	 case(ADV_SCAN_IND):
				write_SD_BT("ADV_SCAN_IND");
	 	 	 break;
	 	 	 case(ADV_NONCONN_IND):
				write_SD_BT("ADV_NONCONN_IND");
	 	     break;
	 	 	 case(SCAN_RSP):
				write_SD_BT("SCAN_RSP");
	 	 	break;



	 }
	 write_SD_BT(";");
	 sprintf(guardar_baddr,"RSSI:%x-%d",rssi,rssi);
	 write_SD_BT(guardar_baddr);
	 write_SD_BT(";");
	 //write_SD_BT("Mensaje: ");
	 uint8_t mensaje[200];
	 for(int i = 0; i <= data_length; i++)
	 {
		 sprintf(mensaje, "%x-", data_RSSI[i]);
		 write_SD_BT(mensaje);

	 }


	 write_SD_BT("\r\n");
	 cerrar_SD_BT();
	 abrir_SD_BT();


}
/**
* @brief  This function is called when there is a Bluetooth Read request
* @param  uint16_t handle Handle of the attribute
* @retval None
*/
void Read_Request_CB(uint16_t handle)
{

}

/**
* @brief  This function is called when there is a change on the gatt attribute
* With this function it's possible to understand if one application 
* is subscribed or not to the one service
* @param uint16_t att_handle Handle of the attribute
* @param uint8_t *att_data attribute data
* @param uint8_t data_length length of the data
* @retval None
*/

extern Estados_Uc_t estado_uc_actual;
static const char * vector_comandos[7] = {"SD","GD","SS","GS","GL","LS","DL"};


void Attribute_Modified_CB(uint16_t attr_handle, uint8_t * att_data, uint8_t data_length)
{
	static char pbuffer[200];



			char texto[40];
			for(int i = 0 ; i < data_length; i++)
			{
				int size = sprintf(texto, "%c",att_data[i] );
				int numero_comando = process_mesage(texto[0], pbuffer, 200, vector_comandos, 7, 0);


				//HAL_UART_Transmit(&huart1, &pbuffer[i], 1, 0xFFFF);

				if(numero_comando >= 0)
				{

					JSON_Value * val_1 = json_parse_string(pbuffer);


					//Chequear si es un objeto
					JSON_Object *root_object = json_value_get_object(val_1);



					JSON_Value  * valor_2 = json_object_dotget_value(root_object, "c");

					char respuesta[40];
					int size;

					char vector[9] = {'S','D',' ',' ',' ',' ',' ',' ',' '};
					switch(numero_comando)
					{
						case 0:
						{

							//PRINT("SD\r\n")
							char * respuest_sd_t = "";
							char * respuest_sd_m = "";

							/*Se meterá asi la hora: (hora)(minuto)(segundo)(dia)(mes)(año) -> 23050202062020  =   23:05:02/02/06/2020*/
							if(json_object_has_value(root_object, "t") != 0 )
							{
								char aux[5]= {'x','x','\0','x','\0'};
								JSON_Value  * valor_3 = json_object_dotget_value(root_object, "t");
								//Falta un IF de solo si es una cadena!!!!!!!!!!!!!!!!!!!!!!!!!!!
								char * t_ptr = json_value_get_string (valor_3);
								if(strlen(t_ptr) == 12)
								{
									//PRINT("Tiempo: ")
									for(int i = 0; i<5;i++)
									{
										aux[0] = t_ptr[2*i];
										aux[1] = t_ptr[2*i+1];
										//PRINT(aux)
										if(i< 2)
										{
											//PRINT(":")
										}
										else
										{
											//PRINT("/")
										}
									}
									aux[0] = t_ptr[10];
									aux[1] = t_ptr[11];

									respuest_sd_t = "T: OK";

									RTC_configurar_dia_desde_cadena(t_ptr);
									//RTC_obtener_hora_str(hora);
									//sprintf(respuesta, "%d:%d:%d/%d/%d/%d", dia.hora_actual.hora,dia.hora_actual.minuto,dia.hora_actual.segundo,dia.dia_actual.dia,dia.dia_actual.mes,dia.dia_actual.year);

								}
								else
								{
									//PRINT("FORMATO DE HORA INCORRECTO.\r\n")
									respuest_sd_t = "T: ERROR";
								}
							}
							if(json_object_has_value(root_object, "m") != 0 )
							{
								JSON_Value  * valor_4 = json_object_dotget_value(root_object, "m");
								//Falta un IF de solo si es una cadena!!!!!!!!!!!!!!!!!!!!!!!!!!!
								char * m_ptr = json_value_get_string (valor_4);
								switch(atoi(m_ptr))
								{
									case 0:

										//PRINT("MODO BAJO CONSUMO. \r\n")
										respuest_sd_m = "M: DORMIDO";

									break;
									case 1:

										//PRINT("MODO STREAMING. \r\n")
										respuest_sd_m = "M: ESPERA_COMANDOS";
										estado_uc_actual = ESPERA_COMANDOS;

									break;
									case 2:

										//PRINT("MODO LOGGING. \r\n")
										respuest_sd_m = "M: LOGGING";
										estado_uc_actual = TOMANDO_DATOS;
									break;
									default:

										//PRINT("MODO INCORRECTO. \r\n")
										respuest_sd_m = "M: ERROR";
									break;
								}

							}
							size = sprintf(respuesta,"%s %s",respuest_sd_m,respuest_sd_t);
							aci_gatt_update_char_value(ConfigServW2STHandle, ConfigCharHandle, 0, size,respuesta);

						break;
						}
						case 1:
							//GD
							if(json_object_has_value(root_object, "p") != 0 )
							{
								JSON_Value  * valor_p = json_object_dotget_value(root_object, "p");
								char * gd_ptr = json_value_get_string (valor_p);
								if(strcmp(gd_ptr,"e")==0)
								{
										switch(estado_uc_actual)
										{
											case( ESPERA_COMANDOS):
												aci_gatt_update_char_value(ConfigServW2STHandle, ConfigCharHandle, 0, strlen("ESPERA_COMANDOS"),"ESPERA_COMANDOS");
											break;
											case( TOMANDO_DATOS):
												aci_gatt_update_char_value(ConfigServW2STHandle, ConfigCharHandle, 0, strlen("TOMANDO_DATOS"),"TOMANDO_DATOS");
											break;
											case( DORMIDO ):
												aci_gatt_update_char_value(ConfigServW2STHandle, ConfigCharHandle, 0, strlen("DORMIDO"),"DORMIDO");
											break;
										}


								}
								else if(strcmp(gd_ptr,"f")==0){
									char fecha[20];
									RTC_obtener_fecha_str(fecha);
									aci_gatt_update_char_value(ConfigServW2STHandle, ConfigCharHandle, 0, strlen(fecha),fecha);
								}
								else if(strcmp(gd_ptr,"h")==0)
								{
									char hora[20];
									RTC_obtener_hora_str(hora);
									aci_gatt_update_char_value(ConfigServW2STHandle, ConfigCharHandle, 0, strlen(hora),hora);
								}
								else if(strcmp(gd_ptr,"b")==0)
								{

									aci_gatt_update_char_value(ConfigServW2STHandle, ConfigCharHandle, 0, strlen("80%"),"80%");
								}
							}

						break;
						case 2:
							//PRINT("SS\r\n")
							size = sprintf(vector,"SS");
							aci_gatt_update_char_value(ConfigServW2STHandle, ConfigCharHandle, 0, size,vector);

						break;
						case 3:
							//PRINT("GS\r\n")
							size = sprintf(vector,"GS");
							aci_gatt_update_char_value(ConfigServW2STHandle, ConfigCharHandle, 0, size,vector);
						break;
						case 4:
							//PRINT("GL\r\n")
							size = sprintf(vector,"GL");
							aci_gatt_update_char_value(ConfigServW2STHandle, ConfigCharHandle, 0, size,vector);
						break;
						case 5:
							//PRINT("LS\r\n")
							size = sprintf(vector,"LS");
							aci_gatt_update_char_value(ConfigServW2STHandle, ConfigCharHandle, 0, size,vector);
						break;
						case 6:
							//PRINT("DL\r\n")
							size = sprintf(vector,"DL");
							aci_gatt_update_char_value(ConfigServW2STHandle, ConfigCharHandle, 0, size,vector);
						break;

					}
				}
			}


}

/**
 * @brief  This function makes the parsing of the Configuration Commands
 * @param uint8_t *att_data attribute data
 * @param uint8_t data_length length of the data
 * @retval None
 */
static void ConfigCommandParsing(uint8_t * att_data, uint8_t data_length)
{
}

/**
* @brief  This function is called whenever there is an ACI event to be processed.
* @note   Inside this function each event must be identified and correctly
*         parsed.
* @param  void *pckt Pointer to the ACI packet
* @retval None
*/
void HCI_Event_CB(void *pckt)
{
  hci_uart_pckt *hci_pckt = pckt;
  hci_event_pckt *event_pckt = (hci_event_pckt*)hci_pckt->data;
  
  if(hci_pckt->type != HCI_EVENT_PKT)
  {
    return;
  }
  
  switch(event_pckt->evt)
  {
    
  case EVT_DISCONN_COMPLETE:
    {
      GAP_DisconnectionComplete_CB();
    }
    break;
  case EVT_LE_ADVERTISING_REPORT:
        {
        	GAP_DisconnectionComplete_CB();
        }
     break;

  case EVT_LE_META_EVENT:
    {
      evt_le_meta_event *evt = (void *)event_pckt->data;
      
      switch(evt->subevent)
      {
      case EVT_LE_CONN_COMPLETE:
        {
          evt_le_connection_complete *cc = (void *)evt->data;
          GAP_ConnectionComplete_CB(cc->peer_bdaddr, cc->handle);
        }
        break;

      case EVT_LE_ADVERTISING_REPORT:
            {
            	le_advertising_info *info = (le_advertising_info *) (evt->data + 1);
            	int8_t rssi;
            	rssi = *(info->data_RSSI + info->data_length);
            	rssi = *(info->data_RSSI + info->data_length);

            	le_advertising_info * var = (le_advertising_info *) (evt->data + 1);
            	//le_advertising_info * var = (void *)evt->data;
            	GAP_Scan(var->evt_type, var->bdaddr_type, var->bdaddr, var->data_length, var->data_RSSI,rssi);

            }
         break;
      }
    }
    break;
  case EVT_VENDOR:
    {
      evt_blue_aci *blue_evt = (void*)event_pckt->data;
      switch(blue_evt->ecode)
      {
      /*case EVT_BLUE_HAL_INITIALIZED:
      {
    	  uint8_t att_data = 0; uint8_t data_length= 1;
    	  ConfigCommandParsing(&att_data,data_length);//No hacer nada esto es para reboot
      }
      break;*/
      case EVT_BLUE_GAP_PROCEDURE_COMPLETE:
      	  {
      		evt_gap_procedure_complete * var = (void *)blue_evt->data;
      		if(var->procedure_code == GAP_GENERAL_DISCOVERY_PROC)
      		{
      			write_SD_BT("\r\n\r\n\r\n++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++\r\n");
                int ret = aci_gap_start_general_discovery_proc(0x4000,0x4000,STATIC_RANDOM_ADDR, 1);
                cerrar_SD_BT();
                abrir_SD_BT();

      		}


      		//evt_gap_device_found *gap_package = (evt_gap_device_found * )blue_evt->data;
      		//GAP_Scan( gap_package->evt_type, gap_package->bdaddr_type, gap_package->bdaddr,  gap_package->data_length, gap_package->data_RSSI);
      		 //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_14, GPIO_PIN_RESET);

      	  }
      	 break;
      case EVT_BLUE_GATT_READ_PERMIT_REQ:
        {
          evt_gatt_read_permit_req *pr = (void*)blue_evt->data; 
          Read_Request_CB(pr->attr_handle);                    
        }
        break;
      case EVT_BLUE_GATT_ATTRIBUTE_MODIFIED:
        {
          evt_gatt_attr_modified_IDB05A1 *evt = (evt_gatt_attr_modified_IDB05A1*)blue_evt->data;
          Attribute_Modified_CB(evt->attr_handle, evt->att_data,evt->data_length);
        }
        break;

      case EVT_LE_ADVERTISING_REPORT:
      {
    	  //evt_gap_device_found *gap_package = (evt_gap_device_found * )blue_evt->data;

    	  //GAP_Scan( gap_package->evt_type, gap_package->bdaddr_type, gap_package->bdaddr,  gap_package->data_length, gap_package->data_RSSI);

      }
      break;

      }
    }
    break;
  }
}

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/

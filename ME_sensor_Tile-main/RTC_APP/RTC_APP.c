/*
 * RTC_APP.c
 *
 *  Created on: 6 mar. 2021
 *      Author: pablo
 */
#include "RTC_APP.h"

RTC_HandleTypeDef hrtc;


void RTC_Init(void)
{

	// Ver esto     WRITE_REG( RTC->BKP31R, 0x0 );


	  /* USER CODE BEGIN RTC_Init 0 */

	  /* USER CODE END RTC_Init 0 */

	  RTC_TimeTypeDef sTime = {0};
	  RTC_DateTypeDef sDate = {0};

	  /* USER CODE BEGIN RTC_Init 1 */

	  /* USER CODE END RTC_Init 1 */
	  /** Initialize RTC Only
	  */
	  hrtc.Instance = RTC;
	  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
	  hrtc.Init.AsynchPrediv = 127;
	  hrtc.Init.SynchPrediv = 255;
	  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
	  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
	  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
	  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
	  if (HAL_RTC_Init(&hrtc) != HAL_OK)
	  {
	    Error_Handler();
	  }

	  /* USER CODE BEGIN Check_RTC_BKUP */

	  /* USER CODE END Check_RTC_BKUP */

	  /** Initialize RTC and set the Time and Date
	  */
	  sTime.Hours = 0x6;
	  sTime.Minutes = 0x0;
	  sTime.Seconds = 0x0;
	  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
	  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
	  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
	  {
	    Error_Handler();
	  }
	  sDate.WeekDay = RTC_WEEKDAY_SATURDAY;
	  sDate.Month = RTC_MONTH_MARCH;
	  sDate.Date = 0x7;
	  sDate.Year = 0x21;

	  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
	  {
	    Error_Handler();
	  }
	  /* USER CODE BEGIN RTC_Init 2 */

	  /* USER CODE END RTC_Init 2 */


}

/*Se meterá asi la hora: (hora)(minuto)(segundo)(dia)(mes)(año) -> 230502020620  =   23:05:02/02/06/20(año2020 en este caso)     */
int RTC_configurar_dia_desde_cadena(const char * dia_str)
{
	RTC_TimeTypeDef sTime;
	RTC_DateTypeDef sDate;

	char aux[2]  = {'x','\0'};
	char aux2[2] = {'x','\0'};
	uint8_t num;

    if(strlen(dia_str)==12)
    {
		for(int i = 0 ; i < 6; i++)
		{
			aux[0]  = dia_str[i*2];
			aux2[0] = dia_str[i*2+1];
			num = (atoi(aux)<<4) + atoi(aux2);

			switch(i)
			{
			    case 0:
					sTime.Hours = num;
				break;
				case 1:
					sTime.Minutes = num;
				break;
					case 2:
					sTime.Seconds = num;
				break;
				case 3:
					sDate.Date = num;
				break;
				case 4:
					sDate.Month = num;
				break;
				case 5:
					sDate.Year = num;
				break;
			  }
		}
		  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
		  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
		  sDate.WeekDay = RTC_WEEKDAY_MONDAY;

		  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
		  {
		      //PRINT("ERROR_SETTING_TIME")
		  }
		  else
		  {
			  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
			  {
				  //PRINT("ERROR_SETTING_DATE")
			  }
		  }


		return 0;
    }
	return 1;
}


int RTC_obtener_hora_str_formato_SD(char * buffer)
{
	RTC_TimeTypeDef sTime;
	RTC_DateTypeDef sDate;


	HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BCD);
	HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BCD);



	buffer[0] = (sTime.Hours / 16) + 48;
	buffer[1] = (sTime.Hours % 16) + 48;
	buffer[2] = '_';
	buffer[3] = (sTime.Minutes / 16) + 48;
	buffer[4] = (sTime.Minutes % 16) + 48;
	buffer[5] = '_';
	buffer[6] = (sTime.Seconds / 16) + 48;
	buffer[7] = (sTime.Seconds % 16) + 48;
	buffer[8] = '\0';

	//sprintf(&dia_str[0], "%d:%d:%d/%d/%d/%d",sTime.Hours, sTime.Minutes,sTime.Seconds,sDate.Date,sDate.Month,sDate.Year);


	return 0;

}



int RTC_obtener_dia_str(char * buffer)
{
	RTC_TimeTypeDef sTime;
		RTC_DateTypeDef sDate;


		HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BCD);
		HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BCD);



		buffer[0] = (sTime.Hours / 16) + 48;
		buffer[1] = (sTime.Hours % 16) + 48;
		buffer[2] = ';';
		buffer[3] = (sTime.Minutes / 16) + 48;
		buffer[4] = (sTime.Minutes % 16) + 48;
		buffer[5] = ';';
		buffer[6] = (sTime.Seconds / 16) + 48;
		buffer[7] = (sTime.Seconds % 16) + 48;
		buffer[8] = '-';
		buffer[9] = (sDate.Date / 16) + 48;
		buffer[10] = (sDate.Date % 16) + 48;
		buffer[11] = '-';
		buffer[12] = (sDate.Month / 16) + 48;
		buffer[13] = (sDate.Month % 16) + 48;
		buffer[14] = '-';
		buffer[15]= '2';
		buffer[16]= '0';
		buffer[17] = (sDate.Year / 16) + 48;
		buffer[18] = (sDate.Year % 16) + 48;
		buffer[19] = '\0';
		return 0;
}


int RTC_obtener_hora_str(char * buffer)
{
	RTC_TimeTypeDef sTime;
	RTC_DateTypeDef sDate;


	HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BCD);
	HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BCD);



	buffer[0] = (sTime.Hours / 16) + 48;
	buffer[1] = (sTime.Hours % 16) + 48;
	buffer[2] = ':';
	buffer[3] = (sTime.Minutes / 16) + 48;
	buffer[4] = (sTime.Minutes % 16) + 48;
	buffer[5] = ':';
	buffer[6] = (sTime.Seconds / 16) + 48;
	buffer[7] = (sTime.Seconds % 16) + 48;
	buffer[8] = '\0';

	//sprintf(&dia_str[0], "%d:%d:%d/%d/%d/%d",sTime.Hours, sTime.Minutes,sTime.Seconds,sDate.Date,sDate.Month,sDate.Year);


	return 0;

}
int RTC_obtener_fecha_str(char * buffer)
{
	RTC_TimeTypeDef sTime;
	RTC_DateTypeDef sDate;


	HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BCD);
	HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BCD);

	buffer[0] = (sDate.Date / 16) + 48;
	buffer[1] = (sDate.Date % 16) + 48;
	buffer[2] = '/';
	buffer[3] = (sDate.Month / 16) + 48;
	buffer[4] = (sDate.Month % 16) + 48;
	buffer[5] = '/';
	buffer[6] = (sDate.Year / 16) + 48;
	buffer[7] = (sDate.Year % 16) + 48;
	buffer[8] = '\0';

	//sprintf(&dia_str[0], "%d:%d:%d/%d/%d/%d",sTime.Hours, sTime.Minutes,sTime.Seconds,sDate.Date,sDate.Month,sDate.Year);


	return 0;

}

int RTC_obtener_momento(momento_t * momento)
{
	 RTC_TimeTypeDef sTime1 = {0};
	 RTC_DateTypeDef sDate1 = {0};
	 HAL_RTC_GetTime(&hrtc, &sTime1, RTC_FORMAT_BCD);
	 HAL_RTC_GetDate(&hrtc, &sDate1, RTC_FORMAT_BCD);
	 momento->segundo = RTC_Bcd2ToByte(sTime1.Seconds);
	 momento->minuto = RTC_Bcd2ToByte(sTime1.Minutes);
	 momento->hora = RTC_Bcd2ToByte(sTime1.Hours);
	 momento->dia = RTC_Bcd2ToByte(sDate1.Date);
	 momento->mes = RTC_Bcd2ToByte(sDate1.Month);
	 momento->year = RTC_Bcd2ToByte(sDate1.Year);
	return 0;

}

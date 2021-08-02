/**
  ******************************************************************************
  * @file    datalog_application.c
  * @author  System Research & Applications Team - Catania Lab.
  * @version V4.1.0
  * @date    26-May-2020
  * @brief   This file provides a set of functions to handle the datalog
  *          application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2020 STMicroelectronics</center></h2>
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

/* Includes ------------------------------------------------------------------*/

#include "datalog_application.h"
#include "sensor_service.h"

#include <stdio.h>
#include <math.h>
    
/* FatFs includes component */
#include "ff_gen_drv.h"

#include "sd_diskio_SensorTile.h"
#include<string.h>
/* Private variables -------------------------------------------------------------*/
   
/* FatFs function common result code */
FRESULT res;

/* File write/read counts */
uint32_t byteswritten, bytesread;

/* File system object for SD card logical drive */
FATFS SDFatFs;



/* SD card logical drive path */
char SDPath[4];

volatile uint8_t pHeader[169];
volatile uint8_t pAudioHeader[44];

/* Imported Variables -------------------------------------------------------------*/
 uint16_t Audio_OUT_Buff[];
  volatile int  index_buff;


char MemsDataFileName[];

/* Exported Variables -------------------------------------------------------------*/
volatile uint8_t SD_Log_Enabled = 1;
volatile uint8_t SD_CardNotPresent = 0;

uint16_t sdcard_file_counter = 0;

char newLine[] = "\r\n";

/* Private function prototypes ---------------------------------------------------*/
static uint32_t csvProcess_HeaderInit(void);
static void floatToInt( float in, int32_t *out_int, int32_t *out_dec, int32_t dec_prec );

static uint32_t WavProcess_HeaderInit(void);
static uint32_t WavProcess_HeaderUpdate(uint32_t len);

static void CreateFileName(char *FileName, uint16_t Counter);








/* File object */
FIL sensors_file;


/**
  * @brief  Start SD-Card demo
  * @param  None
  * @retval None
  */
char sensors_file_name[50] = {0};
void DATALOG_SD_Init(void)
{
  //char SDPath[4];
    
  if(FATFS_LinkDriver(&SD_Driver, SDPath) == 0)
  {
    /* Register the file system object to the FatFs module */
	  if(f_mount(&SDFatFs, (TCHAR const*)SDPath, 0) != FR_OK)
	    {
		  while(1);
	    }

    }
  }

void write_SD(char * cadena)
{
	/* char file_name[30] = {0};
	 sprintf(file_name, "%s%.3d%s", "UESE", 3, ".csv");
	 if(f_open(&MyFile, (char const*)file_name, FA_OPEN_APPEND| FA_WRITE) == FR_OK)
	{
		 //char header2[100];
		 //strcpy(header2,cadena);*/

		 uint32_t byteswritten;
		 if(f_write(&sensors_file, cadena, strlen(cadena), (void *)&byteswritten) == FR_OK)
		 {
			// f_close (&sensors_file);
		 }
	//}

}
void cerrar_SD(void)
{
	f_close (&sensors_file);
}
  
void abrir_SD(void)
{

	f_open(&sensors_file, (char const*)sensors_file_name,  FA_OPEN_APPEND| FA_WRITE);
}

void abrir_SD_new(char * cadena)
{

	sprintf(sensors_file_name, "%s%s", cadena, ".csv");
	f_open(&sensors_file, (char const*)sensors_file_name,  FA_OPEN_APPEND| FA_WRITE);
}





FIL BT_file;
char BT_file_name[50] = {0};
void write_SD_BT(char * cadena)
{
	/* char file_name[30] = {0};
	 sprintf(file_name, "%s%.3d%s", "UESE", 3, ".csv");
	 if(f_open(&MyFile, (char const*)file_name, FA_OPEN_APPEND| FA_WRITE) == FR_OK)
	{
		 //char header2[100];
		 //strcpy(header2,cadena);*/

		 uint32_t byteswritten;
		 if(f_write(&BT_file, cadena, strlen(cadena), (void *)&byteswritten) == FR_OK)
		 {
			// f_close (&sensors_file);
		 }
	//}
}
void cerrar_SD_BT(void)
{
	f_close (&BT_file);
}

void abrir_SD_BT(void)
{

	f_open(&BT_file, (char const*)BT_file_name,  FA_OPEN_APPEND| FA_WRITE);
}

void abrir_SD_new_BT(char * cadena)
{

	sprintf(BT_file_name, "%s%s", cadena, ".csv");
	f_open(&BT_file, (char const*)BT_file_name,  FA_OPEN_APPEND| FA_WRITE);
}








































/**
  * @brief  Start SD-Card demo
  * @param  None
  * @retval None
  */
uint8_t DATALOG_SD_Log_Enable(uint8_t AccessControl)
{
  //static uint16_t sdcard_file_counter = 0;
  
  static uint32_t delay,d=0;
  
  /* written byte count */
  uint32_t byteswritten;
  
  static char file_name[40] = {0};
  
  csvProcess_HeaderInit();
  
  /* SD SPI CS Config */
  SD_IO_CS_Init();
  

    
    CreateFileName(file_name,sdcard_file_counter);

    sdcard_file_counter++;

    delay = HAL_GetTick();  
    d=HAL_GetTick();
    do
    {
      d=HAL_GetTick();
    }while(d - delay < 100 );
    //HAL_Delay(100);
    
    if(f_open(&sensors_file, (char const*)file_name, FA_CREATE_ALWAYS | FA_WRITE) != FR_OK)
    {
      if(sdcard_file_counter > MAX_TRIALS_OPENS_SD)
      {
        sdcard_file_counter = 0;
        SD_CardNotPresent= 1;
      }
      
      return 0;
    }
    

    
    //BytesToWrite =sprintf((char *)BufferToWrite,"Open File OK\r\n");
    //Term_Update(BufferToWrite,BytesToWrite);
    //HAL_Delay(100);

    if(f_write(&sensors_file, (const void*)pHeader, sizeof(pHeader), (void *)&byteswritten) != FR_OK)
    {
      //BytesToWrite =sprintf((char *)BufferToWrite,"Write Failed\r\n");
      //Term_Update(BufferToWrite,BytesToWrite);
      return 0;
    }
  
  return 1;
}

/**
  * @brief  Start SD-Card demo
  * @param  None
  * @retval None
  */
uint8_t DATALOG_SD_LogAudio_Enable(void)
{
  
    return 0;

}

/**
  * @brief  .
  * @param  None
  * @retval None
  */
void DATALOG_SD_DeInit(void)
{
  FATFS_UnLinkDriver(SDPath);
}

/**
  * @brief  Disable SDCard Log
  * @param  None
  * @retval None
  */
void DATALOG_SD_Log_Disable(void)
{
  f_close(&sensors_file);
  
  /* SD SPI Config */
  SD_IO_CS_DeInit();
}

/**
  * @brief  Disable SDCard Log
  * @param  None
  * @retval None
  */
void DATALOG_SD_LogAudio_Disable(void)
{

}

/**
* @brief Highest priority interrupt handler routine
* @param None
* @retval None
*/
void writeAudio_on_sd(void)
{ 
  
}

/**
  * @brief  Write New Line to file
  * @param  None
  * @retval None
  */
void DATALOG_SD_NewLine(void)
{
  uint32_t byteswritten; /* written byte count */
  f_write(&sensors_file, (const void*)&newLine, 2, (void *)&byteswritten);
}



/**
* @brief  write on uSD some data
* @param  None
* @retval None
*/
void uSdWriteSpeedTest(char* myData)
{
  /* Write data to the file on the SDCard */
  if(SD_Log_Enabled) 
  {
    uint32_t size; 
    size = 8192; 
    res = f_write(&sensors_file, myData, size, (void *)&byteswritten);
  }
}

/**
  * @brief  Initialize the wave header file
  * @param  pHeader: Header Buffer to be filled
  * @param  pWaveFormatStruct: Pointer to the wave structure to be filled.
  * @retval 0 if passed, !0 if failed.
  */
static uint32_t csvProcess_HeaderInit(void)
{
  
	pHeader[0]=  'h';
	  pHeader[1]=  'h';
	  pHeader[2]=  ':';
	  pHeader[3]=  'm';
	  pHeader[4]=  'm';
	  pHeader[5]=  ':';
	  pHeader[6]=  's';
	  pHeader[7]=  's';
	  pHeader[8]=  ' ';
	  pHeader[9]=  ',';

	  pHeader[10]= 'A';
	  pHeader[11]= 'c';
	  pHeader[12]= 'c';
	  pHeader[13]= 'X';
	  pHeader[14]= ' ';
	  pHeader[15]= '[';
	  pHeader[16]= 'm';
	  pHeader[17]= 'g';
	  pHeader[18]= ']';
	  pHeader[19]= ',';

	  pHeader[20]= 'A';
	  pHeader[21]= 'c';
	  pHeader[22]= 'c';
	  pHeader[23]= 'Y';
	  pHeader[24]= ' ';
	  pHeader[25]= '[';
	  pHeader[26]= 'm';
	  pHeader[27]= 'g';
	  pHeader[28]= ']';
	  pHeader[29]= ',';

	  pHeader[30]= 'A';
	  pHeader[31]= 'c';
	  pHeader[32]= 'c';
	  pHeader[33]= 'Z';
	  pHeader[34]= ' ';
	  pHeader[35]= '[';
	  pHeader[36]= 'm';
	  pHeader[37]= 'g';
	  pHeader[38]= ']';
	  pHeader[39]= ',';

	  pHeader[40]= 'G';
	  pHeader[41]= 'y';
	  pHeader[42]= 'r';
	  pHeader[43]= 'o';
	  pHeader[44]= 'X';
	  pHeader[45]= ' ';
	  pHeader[46]= '[';
	  pHeader[47]= 'm';
	  pHeader[48]= 'd';
	  pHeader[49]= 'p';
	  pHeader[50]= 's';
	  pHeader[51]= ']';
	  pHeader[52]= ',';

	  pHeader[53]= 'G';
	  pHeader[54]= 'y';
	  pHeader[55]= 'r';
	  pHeader[56]= 'o';
	  pHeader[57]= 'Y';
	  pHeader[58]= ' ';
	  pHeader[59]= '[';
	  pHeader[60]= 'm';
	  pHeader[61]= 'd';
	  pHeader[62]= 'p';
	  pHeader[63]= 's';
	  pHeader[64]= ']';
	  pHeader[65]= ',';

	  pHeader[66]= 'G';
	  pHeader[67]= 'y';
	  pHeader[68]= 'r';
	  pHeader[69]= 'o';
	  pHeader[70]= 'Z';
	  pHeader[71]= ' ';
	  pHeader[72]= '[';
	  pHeader[73]= 'm';
	  pHeader[74]= 'd';
	  pHeader[75]= 'p';
	  pHeader[76]= 's';
	  pHeader[77]= ']';
	  pHeader[78]= ',';

	  pHeader[79]= 'M';
	  pHeader[80]= 'a';
	  pHeader[81]= 'g';
	  pHeader[82]= 'X';
	  pHeader[83]= ' ';
	  pHeader[84]= '[';
	  pHeader[85]= 'm';
	  pHeader[86]= 'g';
	  pHeader[87]= 'a';
	  pHeader[88]= 'u';
	  pHeader[89]= 's';
	  pHeader[90]= 's';
	  pHeader[91]= ']';
	  pHeader[92]= ',';

	  pHeader[93]= 'M';
	  pHeader[94]= 'a';
	  pHeader[95]= 'g';
	  pHeader[96]= 'Y';
	  pHeader[97]= ' ';
	  pHeader[98]= '[';
	  pHeader[99]= 'm';
	  pHeader[100]= 'g';
	  pHeader[101]= 'a';
	  pHeader[102]= 'u';
	  pHeader[103]= 's';
	  pHeader[104]= 's';
	  pHeader[105]= ']';
	  pHeader[106]= ',';

	  pHeader[107]= 'M';
	  pHeader[108]= 'a';
	  pHeader[109]= 'g';
	  pHeader[110]= 'Z';
	  pHeader[111]= ' ';
	  pHeader[112]= '[';
	  pHeader[113]= 'm';
	  pHeader[114]= 'g';
	  pHeader[115]= 'a';
	  pHeader[116]= 'u';
	  pHeader[117]= 's';
	  pHeader[118]= 's';
	  pHeader[119]= ']';
	  pHeader[120]= ',';

	  pHeader[121]= 'P';
	  pHeader[122]= ' ';
	  pHeader[123]= '[';
	  pHeader[124]= 'm';
	  pHeader[125]= 'B';
	  pHeader[126]= ']';
	  pHeader[127]= ',';

	  pHeader[128]= 'T';
	  pHeader[129]= '1';
	  pHeader[130]= ' ';
	  pHeader[131]= '[';
	  pHeader[132]= '°';
	  pHeader[133]= 'C';
	  pHeader[134]= ']';
	  pHeader[135]= ',';

	  pHeader[136]= 'T';
	  pHeader[137]= '2';
	  pHeader[138]= ' ';
	  pHeader[139]= '[';
	  pHeader[140]= '°';
	  pHeader[141]= 'C';
	  pHeader[142]= ']';
	  pHeader[143]= ',';

	  pHeader[144]= 'H';
	  pHeader[145]= ' ';
	  pHeader[146]= '[';
	  pHeader[147]= '%';
	  pHeader[148]= ']';
	  pHeader[149]= ',';

	  pHeader[150]= 'q';
	  pHeader[151]= 'i';
	  pHeader[152]= ',';
	  pHeader[153]= 'q';
	  pHeader[154]= 'j';
	  pHeader[155]= ',';
	  pHeader[156]= 'q';
	  pHeader[157]= 'k';
	  pHeader[158]= ',';
	  pHeader[159]= 'q';
	  pHeader[160]= 's';
	  pHeader[161]= ',';

	  pHeader[162]= 'B';
	  pHeader[163]= 'a';
	  pHeader[164]= 't';
	  pHeader[165]= 't';
	  pHeader[166]= '%';

	  pHeader[167]= '\r';
	  pHeader[168]= '\n';
  /* Return 0 if all operations are OK */
  return 0;
}

/**
  * @brief  Initialize the wave header file
  * @param  pHeader: Header Buffer to be filled
  * @param  pWaveFormatStruct: Pointer to the wave structure to be filled.
  * @retval 0 if passed, !0 if failed.
  */
static uint32_t WavProcess_HeaderInit(void)
{
  
  /* Return 0 if all operations are OK */
  return 0;
}

/**
  * @brief  Initialize the wave header file
  * @param  pHeader: Header Buffer to be filled
  * @param  pWaveFormatStruct: Pointer to the wave structure to be filled.
  * @retval 0 if passed, !0 if failed.
  */
static uint32_t WavProcess_HeaderUpdate(uint32_t len)
{

  return 0;
}

/**
* @brief  Splits a float into two integer values.
* @param  in the float value as input
* @param  out_int the pointer to the integer part as output
* @param  out_dec the pointer to the decimal part as output
* @param  dec_prec the decimal precision to be used
* @retval None
*/
static void floatToInt( float in, int32_t *out_int, int32_t *out_dec, int32_t dec_prec )
{
  *out_int = (int32_t)in;
  
  if(in >= 0.0f)
  {
    in = in - (float)(*out_int);
  }
  else
  {
    in = (float)(*out_int) - in;
  }
  *out_dec = (int32_t)trunc(in * pow(10, dec_prec));
}

/**
* @brief  Create file name for mems data loggimg.
* @param  FileName Name of the create file
* @retval None
*/
static void CreateFileName(char *FileName, uint16_t Counter)
{

	 sprintf(FileName, "%d.txt",Counter);
}

 /* ALLMEMS1_ENABLE_SD_CARD_LOGGING */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

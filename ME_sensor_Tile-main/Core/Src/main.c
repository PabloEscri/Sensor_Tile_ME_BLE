
/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/*Librerías para los sensores de enviromental (Presión, he desactivado el de temperatura) y para acelerómetro y giroscopio.*/
#include "SensorTile_env_sensors.h"
#include "SensorTile_motion_sensors.h"
#include "BLE_APP.h"
#include "datalog_application.h"
#include "ff.h"
#include "RTC_APP.h"
#include <string.h>
#include "cmsis_os.h"

osThreadId defaultTaskHandle;
void StartDefaultTask(void const * argument);
osThreadId defaultTaskHandle2;
void StartDefaultTask2(void const * argument);
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */
extern RTC_HandleTypeDef hrtc;
void MX_RTC_Init(void);
static void MX_GPIO_Init(void);
/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

TIM_HandleTypeDef htim2;
void MX_TIM2_Init(void);


/**
  * @brief  The application entry point.
  * @retval int
  */
FIL MyFile;
const TCHAR path[] = "ST.txt";
BYTE mode = FA_CREATE_NEW ;
extern FATFS SDFatFs;
extern char SDPath[4];   /* SD logical drive path */



Estados_Uc_t estado_uc_actual = ESPERA_COMANDOS;


typedef struct
{
	int segundos;
	int minutos;
	int horas;
} my_time;
#define DATA_NUMBER 60
BLE_Estados_t ESTADO_BLE = CENTRAL;
BLE_Estados_t ESTADO_BLE_ant = PERIPHERAL;




int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
 HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOG_CLK_ENABLE();
  HAL_PWREx_EnableVddIO2();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin : PG12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

#ifdef MY_FREERTOS




  /* USER CODE BEGIN SysInit */
  MX_GPIO_Init();
  /* USER CODE END SysInit */

  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);
  osThreadDef(defaultTask2, StartDefaultTask2, osPriorityNormal, 0, 128);
  defaultTaskHandle2 = osThreadCreate(osThread(defaultTask2), NULL);

    /* USER CODE BEGIN RTOS_THREADS */
    /* add threads, ... */
    /* USER CODE END RTOS_THREADS */

    /* Start scheduler */
    osKernelStart();
#endif

  //Inicializamos el sensor de presión. He desactivado el de temperatura
  BSP_ENV_SENSOR_Init(LPS22HB_0, ENV_PRESSURE);

  //Leemos el identificador del sensor de presión.
  uint8_t LPS22HB_Id;
  BSP_ENV_SENSOR_ReadID(LPS22HB_0, &LPS22HB_Id);
  //Leemos el valor del sensor de presión.
  float Value;
  BSP_ENV_SENSOR_GetValue(LPS22HB_0, ENV_PRESSURE, &Value);


   // Inicializamos el sensor de LSM6DSM y el sensor LSM303AGR_MAG

  BSP_MOTION_SENSOR_Init(LSM6DSM_0, MOTION_GYRO|MOTION_ACCELERO);
  BSP_MOTION_SENSOR_Init(LSM303AGR_MAG_0, MOTION_MAGNETO);

  uint8_t LSM6DSM_Id, LSM303AGR_MAG_Id;
  BSP_MOTION_SENSOR_ReadID(LSM6DSM_0, &LSM6DSM_Id);
  BSP_MOTION_SENSOR_ReadID(LSM303AGR_MAG_0, &LSM303AGR_MAG_Id);

//Habilito los sensores, TODO: Mirar si es necesario
  BSP_MOTION_SENSOR_Enable(LSM6DSM_0, MOTION_GYRO);
  BSP_MOTION_SENSOR_Enable(LSM6DSM_0, MOTION_ACCELERO);
  BSP_MOTION_SENSOR_Enable(LSM303AGR_MAG_0, MOTION_MAGNETO);

  //Leo los sensores.
  BSP_MOTION_SENSOR_Axes_t Axes;
  BSP_MOTION_SENSOR_GetAxes(LSM6DSM_0,MOTION_GYRO ,&Axes);
  BSP_MOTION_SENSOR_GetAxes(LSM6DSM_0,MOTION_ACCELERO ,&Axes);
  BSP_MOTION_SENSOR_GetAxes(LSM303AGR_MAG_0,MOTION_MAGNETO ,&Axes);


//HAL_Delay(100);
  RTC_Init();
  RTC_TimeTypeDef sTime1 = {0};
    RTC_DateTypeDef sDate1 = {0};
    int minutos;
    int horas;
    my_time registro[DATA_NUMBER];
    int segundos_ant=0;
    int contador =0;
    int continua =1;






    momento_t my_momento;
    RTC_obtener_momento(&my_momento);
    uint32_t tick;
    BSP_MOTION_SENSOR_Axes_t Axes_A;
    BSP_MOTION_SENSOR_Axes_t Axes_G;
    BSP_MOTION_SENSOR_Axes_t Axes_M;
    char escribir_sd[100];

    MX_TIM2_Init();


    DATALOG_SD_Init();
    uint32_t tick_ant=0;
    uint32_t tick_ant2 = 0;
    char nombre_fichero_aux[100];

    while (1)
  {
    	/*f_mkdir("sub1");
    	abrir_SD_new("sub1/mis");
    	write_SD("Fecha;Gyro;Accel;Magnet\r\n");
    	cerrar_SD();*/


#if 1
    	if(estado_uc_actual == ESPERA_COMANDOS)
    	{
    		if(ESTADO_BLE != PERIPHERAL)
    		{
    			Init_BlueNRG_Stack(PERIPHERAL);
    			Init_BlueNRG_Custom_Services(PERIPHERAL);
    			ESTADO_BLE = PERIPHERAL;
    			ESTADO_BLE_ant = PERIPHERAL;
    		}
    		while(estado_uc_actual == ESPERA_COMANDOS)
    		{
    			 hci_user_evt_proc();
    		}
    		RTC_obtener_dia_str(escribir_sd);

    		cerrar_SD();
    		f_mkdir(escribir_sd);
    		sprintf(nombre_fichero_aux,"%s/%s%s",escribir_sd,escribir_sd,"_sen");



    		abrir_SD_new(nombre_fichero_aux);
    		write_SD("Fecha(dd:mm:yy);Timestamp(ms);Gyro(rad/s);Accel(m/s^2);Magnet(B)\r\n");
    		cerrar_SD();
    		cerrar_SD_BT();

    		sprintf(nombre_fichero_aux,"%s/%s%s",escribir_sd,escribir_sd,"_BT");
    		abrir_SD_new_BT(nombre_fichero_aux);
    		write_SD_BT("Dispositivos_BT\r\n");
    		cerrar_SD_BT();
    	    tick_ant = HAL_GetTick();
    	    tick_ant2 = HAL_GetTick();
    	}
    	else if(estado_uc_actual == TOMANDO_DATOS)
		{
			tick = HAL_GetTick();
			  //Leo los sensores.
			  RTC_obtener_hora_str(escribir_sd);
			  write_SD(escribir_sd);
			  BSP_MOTION_SENSOR_GetAxes(LSM6DSM_0,MOTION_GYRO ,&Axes_G);
			  BSP_MOTION_SENSOR_GetAxes(LSM6DSM_0,MOTION_ACCELERO ,&Axes_A);
			  BSP_MOTION_SENSOR_GetAxes(LSM303AGR_MAG_0,MOTION_MAGNETO ,&Axes_M);
			  sprintf(escribir_sd,";%ld;%d;%d;%d;%d;%d;%d;%d;%d;%d\r\n",tick,Axes_G.x,Axes_G.y,Axes_G.z,Axes_A.x,Axes_A.y,Axes_A.z,Axes_M.x,Axes_M.y,Axes_M.z);

			  write_SD(escribir_sd);



			  if((ESTADO_BLE == PERIPHERAL) && (ESTADO_BLE_ant == CENTRAL))
			  {
				  ESTADO_BLE_ant = ESTADO_BLE;
				  Init_BlueNRG_Stack(PERIPHERAL);
				  Init_BlueNRG_Custom_Services(PERIPHERAL);
			  }
			  else if((ESTADO_BLE == CENTRAL)&&(ESTADO_BLE_ant == PERIPHERAL))
			  {


				  ESTADO_BLE_ant = ESTADO_BLE;
				  //Inicio modo scanning
				  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_13, GPIO_PIN_SET);
				  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_13, GPIO_PIN_RESET);
				  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_13, GPIO_PIN_SET);
				  Init_BlueNRG_Stack(CENTRAL);
				  Init_BlueNRG_Custom_Services(CENTRAL);
				  __HAL_TIM_CLEAR_FLAG(&htim2, TIM_SR_UIF);
				  HAL_TIM_Base_Start_IT(&htim2);

			  }

			  if(abs(HAL_GetTick() - tick_ant2) > 40000) //Cada 1 minuto cambio de role
			  {
				  ESTADO_BLE = CENTRAL;
				  tick_ant2 = HAL_GetTick();


			  }

			  if(abs(HAL_GetTick() - tick_ant) > 5000) //Cada 5 segundos escribo
			  {
				  cerrar_SD();
				  tick_ant = HAL_GetTick();
				  abrir_SD();
			  }
			  else
			  {
				HAL_Delay(20);
			  }
			  hci_user_evt_proc();

		}


#endif
  }
}


void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
	osDelay(50);
    HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_12);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_14, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_14, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_14, GPIO_PIN_RESET);


  }
  /* USER CODE END 5 */
}
void StartDefaultTask2(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
	osDelay(10);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_13, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_13, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_13, GPIO_PIN_RESET);
  }
  /* USER CODE END 5 */
}


void MX_RTC_Init(void)
{

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

/**
  * @brief System Clock Configuration
  * @retval None
  */

#if 1//SD + BLE todo funcionando
 //BLE
 void SystemClock_Config(void)
 {
   RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
   RCC_OscInitTypeDef RCC_OscInitStruct = {0};
   RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

   __HAL_RCC_GPIOC_CLK_ENABLE();
   __HAL_RCC_PWR_CLK_ENABLE();
   HAL_PWR_EnableBkUpAccess();
   __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);



   /* Enable the LSE Oscilator */
   RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE;
   RCC_OscInitStruct.LSEState = RCC_LSE_ON;
   if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
   {
     while(1);
   }

   /* Enable the CSS interrupt in case LSE signal is corrupted or not present */
   HAL_RCCEx_DisableLSECSS();

   /* Enable MSI Oscillator and activate PLL with MSI as source */
   RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_MSI;
   RCC_OscInitStruct.MSIState            = RCC_MSI_ON;
   RCC_OscInitStruct.HSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
   RCC_OscInitStruct.MSIClockRange       = RCC_MSIRANGE_11;
   RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
   RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_MSI;
   RCC_OscInitStruct.PLL.PLLM            = 6;
   RCC_OscInitStruct.PLL.PLLN            = 40;
   RCC_OscInitStruct.PLL.PLLP            = 7;
   RCC_OscInitStruct.PLL.PLLQ            = 4;
   RCC_OscInitStruct.PLL.PLLR            = 4;
   if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
   {
     while(1);
   }

   /* Enable MSI Auto-calibration through LSE */
   HAL_RCCEx_EnableMSIPLLMode();

   RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
   PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
   PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
   if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
   {
     Error_Handler();
   }


   /* Select MSI output as USB clock source */
   PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USB;
   PeriphClkInitStruct.UsbClockSelection = RCC_USBCLKSOURCE_MSI;
   HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct);

   /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
   clocks dividers */
   RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
   RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
   RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
   RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
   RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
   if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
   {
     while(1);
   }
 }

#endif



/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  if (htim->Instance == TIM2) {
	  HAL_TIM_Base_Stop(&htim2);
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_13, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_13, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_13, GPIO_PIN_SET);
	  ESTADO_BLE = PERIPHERAL;
	  aci_gap_terminate_gap_procedure(GAP_GENERAL_DISCOVERY_PROC);


   }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}






void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 80000000 * 4; //1 minuto
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV2;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */
  __HAL_TIM_CLEAR_FLAG(&htim2, TIM_SR_UIF);
  /* USER CODE END TIM2_Init 2 */

}

//#define PLACA_GRANDE

#ifdef PLACA_GRANDE
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET);

  /*Configure GPIO pin : PC1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}
#endif
#define PLACA_PEQUENA
#ifdef PLACA_PEQUENA
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_14, GPIO_PIN_SET); //SWCLK el de abajo

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_13, GPIO_PIN_SET);

  /*Configure GPIO pin : PA14 */
  GPIO_InitStruct.Pin = GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}
#endif












/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}


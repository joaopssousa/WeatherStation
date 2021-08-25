/**
  ******************************************************************************
  * @file    main.c
  * @author  MCD Application Team
  * @brief   this is the main!
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2018 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "radio.h"
#include "debug.h"
#include "hw.h"
#include "low_power_manager.h"
#include "lora.h"
#include "bsp.h"
#include "timeServer.h"
#include "vcom.h"
#include "version.h"
#include "ble.h"
#include "station.h"
#include "battery_monitor.h"
#include "fw_update_app.h"
#include "com.h"
#include "irradiator_sensor.h"


#define MAX_MEASURES					5	// Quantidade de medidas do sensor de irradiação para calcular a media

#define USED_POWER TX_POWER_0

#define LOW_POWER_DISABLE

#define DEBUG_SD 1
#define PRINT_SD_CARD(X) do{ if(DEBUG_SD>0) { X } }while(0);

_Bool delay_flag = RESET;
char buffer_tag[50];
#define LOG_FILE "STORE.TXT"

//#define SDCARD_IN_USE 		//to use SD Card enable this


/*--------------------*/


#define LORAWAN_MAX_BAT   254


#define USED_POWER TX_POWER_0

#define LOW_POWER_DISABLE

/*!
 * Defines the application data transmission duty cycle. value in [ms].
 */
#define APP_TX_DUTYCYCLE                            60000	// 1 min

/*!
 * LoRaWAN Adaptive Data Rate
 * @note Please note that when ADR is enabled the end-device should be static
 */
#define LORAWAN_ADR_STATE LORAWAN_ADR_OFF //modified - old - ON
/*!
 * LoRaWAN Default data Rate Data Rate
 * @note Please note that LORAWAN_DEFAULT_DATA_RATE is used only when ADR is disabled
 */
#define LORAWAN_DEFAULT_DATA_RATE DR_3 //modified - old - DR_0
/*!
 * LoRaWAN application port
 * @note do not use 224. It is reserved for certification
 */
#define LORAWAN_APP_PORT                            2
#define LORAWAN_CHANGE_CLASS_PORT					3

/*!
 * Commands to change class
 *
 */
#define CHANGE_TO_CLASS_A 							0
#define CHANGE_TO_CLASS_B 							1
#define CHANGE_TO_CLASS_C 							2


/*!
 * LoRaWAN default endNode class port
 */
#define LORAWAN_DEFAULT_CLASS                       CLASS_C
/*!
 * LoRaWAN default confirm state
 */
#define LORAWAN_DEFAULT_CONFIRM_MSG_STATE           LORAWAN_UNCONFIRMED_MSG
/*!
 * User application data buffer size
 */
#define LORAWAN_APP_DATA_BUFF_SIZE                  64


/******************************** WeatherStation **********************************************/

#define DATA_BUFF_SIZE                  64  // Tamanho do buffer de dados para envio

uint8_t send_battery_voltage_flag = LORA_RESET;

// User application data
uint8_t AppDataBuff[DATA_BUFF_SIZE];

double vbat;
uint16_t vbat_int;

uint32_t mediaIrradiator;


// Estrutura do dados para envio
Sensor_AppData AppData = { AppDataBuff, 0, 0 };

// Buffer de envio
char Buffer_to_send[sizeof(Estation_Parameters)] = { 0 };

// Retorno dos parametros definidos em station.h
extern Estation_Parameters Parameters;

/***********************************************************************************************/

static uint8_t LORA_GetBatteryLevel (void);



/************************* SD CArd Function prototypes *****************************************/
static void mount_sd_card();
static void REMOVE_and_OPENAGAIN(const char* arq);
static void VERIFY_OPEN(const char* arq);
static void SAVE_ON_CARD();
static void REMOVE_FROM_CARD();
/***********************************************************************************************/



/* call back when LoRa endNode has received a frame*/
static void LORA_RxData(lora_AppData_t *AppData);

/* call back when LoRa endNode has just joined*/
static void LORA_HasJoined(void);

/* call back when LoRa endNode has just switch the class*/
static void LORA_ConfirmClass(DeviceClass_t Class);

/* call back when server needs endNode to send a frame*/
static void LORA_TxNeeded(void);

/* callback to get the battery level in % of full charge (254 full charge, 0 no charge)*/
static uint8_t LORA_GetBatteryLevel(void);

/* LoRa endNode send request*/
static void Send(void *context);

/* start the tx process*/
static void LoraStartTx(TxEventType_t EventType);

/* tx timer callback function*/
static void OnTxTimerEvent(void *context);

/* tx timer callback function*/
static void LoraMacProcessNotify(void);


/* Private variables ---------------------------------------------------------*/
/* load Main call backs structure*/
static LoRaMainCallback_t LoRaMainCallbacks = { LORA_GetBatteryLevel,
                                                HW_GetTemperatureLevel,
                                                HW_GetUniqueId,
                                                HW_GetRandomSeed,
                                                LORA_RxData,
                                                LORA_HasJoined,
                                                LORA_ConfirmClass,
                                                LORA_TxNeeded,
                                                LoraMacProcessNotify
                                              };
LoraFlagStatus LoraMacProcessRequest = LORA_RESET;
LoraFlagStatus AppProcessRequest = LORA_RESET;


static TimerEvent_t TxTimer;


/* !
 *Initialises the Lora Parameters
 */
static  LoRaParam_t LoRaParamInit = {LORAWAN_ADR_STATE,
                                     LORAWAN_DEFAULT_DATA_RATE,
                                     LORAWAN_PUBLIC_NETWORK
                                    };

/* Private functions ---------------------------------------------------------*/

/************************
 * SD Card Functions
 ************************/

int size_list(FIL File){
	char frase_f[10] = { 0 };
	int index = 0;
	while(f_gets(frase_f, bytesread, &File) != 0){
		index++;
	}
	return index;
}

static void mount_sd_card(){
	if(f_mount(&SDFatFS, (const TCHAR *)&SDPath, 1) != FR_OK)
	  {
		  // TODO Acionar flag ou alerta de ausencia de cartão ou erro de montagem
		  PRINT_SD_CARD(PRINTF("\r\nErro ao montar o cartao\r\n");)
		 // Error_Handler();
	  }
}

static void REMOVE_and_OPENAGAIN(const char* arq){

	res = f_unlink(arq);
	if(res == FR_LOCKED){
		f_close(&SDFile); 		// Fecha
		f_unlink(arq);			// Depois apaga
	}
	else if(res == FR_NO_FILE){
		return; 	// Não há arquivo existente com o nome informado
	}

	if(f_open(&SDFile, arq, FA_OPEN_APPEND | FA_READ | FA_WRITE) != FR_OK)
	{
		// TODO Imprimir os erros e tratar na uart
	  Error_Handler();
	}

	f_sync(&SDFile);
}

static void VERIFY_OPEN(const char* arq){
	res = f_open(&SDFile, arq, FA_OPEN_APPEND | FA_READ | FA_WRITE) != FR_OK;
	if(res == FR_OK){
		PRINT_SD_CARD(PRINTF("FR_OK \n");)
		return;
	}
	else if(res == FR_LOCKED){
		PRINT_SD_CARD(PRINTF("FR_LOCKED \n");)
		return;
	}
	else if(res == FR_DISK_ERR){
		PRINT_SD_CARD(PRINTF("FR_DISK_ERR \n");)
		Error_Handler();
	}
	else{
		PRINT_SD_CARD(PRINTF("Error to open the log file on the SD Card \n Reset the board \n");)
		Error_Handler();
	}
}

static void SAVE_ON_CARD(){
	//delayed_store_flag++; 	// Contagem de TAGs atrasadas ao envio

	// Se não há conexão entre o gateway, armazena no cartão SD para envio posterior
//	PRINT_SD_CARD(PRINTF("===> Escrita no cartao. Count = %d\r\n", delayed_store_flag);)
//	f_write(&SDFile, store_TAG[last_TAG].N_TAG, sizeof(store_TAG[last_TAG].N_TAG), (void *)&byteswritten);
////	f_sync(&SDFile);	// Um ou outro
//	f_close(&SDFile);
}

static void REMOVE_FROM_CARD(){
	// Remove do cartão SD e armazena estrutura para envio da Lora
	//TODO Generalizar a função colocando um argumento para receber o dado que estava no cartão

//	f_gets(buffer_tag, bytesread, &SDFile);
//	memcpy(tag_to_lora.N_TAG, buffer_tag, sizeof(buffer_tag));
//	delayed_store_flag--;
//	PRINT_SD_CARD(PRINTF("===> Removida do cartão. Count = %d\r\n", delayed_store_flag);)
}

/************* End of Sd card functions *****************/


/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{


  flags_ble.all_flags=RESET;						/* Reset all bluetooth flags */

  HAL_Init();										/* STM32 HAL library initialization*/


  SystemClock_Config();								/* Configure the system clock*/


  HW_Init();										/* Configure the hardware*/

  init_station();									/* Initialize WeatherStation Peripherals */

  init_irradiator();								/* Initialize Irradiator sensor */

  init_battery_monitor();							/* Initialize Battery monitor */

  HAL_TIM_Base_Start_IT(&htim3);

  mount_sd_card();									/* Mount and prepare SD Card */

  LPM_SetOffMode(LPM_APPLI_Id, LPM_Disable);		/* Disable Stand-by mode */

	PRINTF("APP_VERSION= %02X.%02X.%02X.%02X\r\n",
			(uint8_t)(__APP_VERSION >> 24), (uint8_t)(__APP_VERSION >> 16),
			(uint8_t)(__APP_VERSION >> 8), (uint8_t)__APP_VERSION);
	PRINTF("MAC_VERSION= %02X.%02X.%02X.%02X\r\n",
			(uint8_t)(__LORA_MAC_VERSION >> 24),
			(uint8_t)(__LORA_MAC_VERSION >> 16),
			(uint8_t)(__LORA_MAC_VERSION >> 8), (uint8_t)__LORA_MAC_VERSION);


  LORA_Init(&LoRaMainCallbacks, &LoRaParamInit);	/* Configure the Lora Stack*/

  LORA_Join();

  LoraStartTx(TX_ON_TIMER);

  uint8_t buffer_time[6];

  flags_ble.all_flags=0;
  flagsStation.all_flags=0;

  while (1)
  {

	if((flagsStation.receive_measure_irrad > 0) && (!flagsStation.active_irradiator)) {
		PRINTF("Irradiador presente...\n");
		flagsStation.active_irradiator = 1;
	}

	if(flagsStation.read_sensors)
	{
		flagsStation.read_sensors=0;
		PRINTF("Leitura dos Sensores\r\n");
		read_sensors(&Parameters);
		PRINTF("Leitura da tensão da bateria\r\n");
		vbat = get_battery_voltage();
		vbat_int = (uint16_t)(double)(vbat*100);

	}

	if(flagsStation.receive_measure_irrad)
	{
		flagsStation.receive_measure_irrad=0;

		measures += getIntMeasure();

		count_measures++;

		/* Calcula a media a cada 5 medidas do sensor de irradiacao */
		if(count_measures == MAX_MEASURES) {
			count_measures = 0;
			mediaIrradiator = mediaCalculator(MAX_MEASURES);

			PRINTF("Average of the last 5 measurements of the radiator:%.2f W/m2\n", mediaIrradiator);
		}
	}

	if (flagsStation.pluviometer)
	{
		flagsStation.pluviometer=0;
		get_time_now((uint8_t*)&buffer_time);
		// Inicio de outro dia, zera-se o contador de precipitação.
		if ((buffer_time[3] == 23) && (buffer_time[4] == 59) && buffer_time[5] > 30)
		{
			pluviometer_count = 0;
		}
	}

	if (flags_ble.enable_handler){
		flags_ble.enable_handler = 0;
		HAL_TIM_Base_Stop(&htim2);
		ble_handler((uint8_t*)&message_ble);					// Aciona o handler para selecionar a mensagem de resposta.
	}

	if (flags_ble.update_mode){
		__get_PRIMASK();

		flags_ble.update_mode = RESET;

		//Clear Usart to receive new firmware
		HAL_NVIC_DisableIRQ(USART1_IRQn);
		HAL_UART_AbortReceive_IT(&huart1);
		HAL_UART_DeInit(&huart1);
		HAL_Delay(1);
		COM_Init();
		HAL_Delay(1);
		COM_Flush();
		//Enter in Update Mode
		FW_UPDATE_Run();

		//ReEnable Ble Interrupts
		MX_USART1_UART_Init();
		HAL_UART_Receive_IT(&huart1, rx_byte_uart1, 1);
	}

	//Send WeatherStation Data
    if (AppProcessRequest == LORA_SET)
    {
    	AppProcessRequest = LORA_RESET;
    	Send(NULL);
    }

    if (LoraMacProcessRequest == LORA_SET)
    {
    	PRINTF("LoraMacProcessRequest\r\n");
    	LoraMacProcessRequest = LORA_RESET;
    	LoRaMacProcess();
    }
    //If a flag is set at this point, mcu must not enter low power and must loop
    //DISABLE_IRQ();

     /*if an interrupt has occurred after DISABLE_IRQ, it is kept pending
     * and cortex will not enter low power anyway
     * */
    if ((LoraMacProcessRequest != LORA_SET) && (AppProcessRequest != LORA_SET))
    {
#ifndef LOW_POWER_DISABLE
      LPM_EnterLowPower();
#endif
    }

    //ENABLE_IRQ();

  }
}

void LoraMacProcessNotify(void)
{
  LoraMacProcessRequest = LORA_SET;
}


static void LORA_HasJoined(void)
{
  LORA_RequestClass(LORAWAN_DEFAULT_CLASS);
}


static void Send(void *context) {

	if (LORA_JoinStatus() != LORA_SET) {
		/*Not joined, try again later*/
		LORA_Join();
		return;
	}

	TVL1(PRINTF("SEND REQUEST\n\r");)

	get_time_now(AppData.Buff);

	AppData.Port = LORAWAN_APP_PORT;

	memcpy(&(AppData.Buff[6]),Buffer_to_send,sizeof(Estation_Parameters));

	AppData.Buff[19]= (vbat_int>>8)&0xFF;
	AppData.Buff[20]= vbat_int&0xFF;
	AppData.BuffSize = sizeof(Estation_Parameters)+8;

	if(flagsStation.active_irradiator) {
		AppData.BuffSize += 2;
		AppData.Buff[21]= (mediaIrradiator>>8)&0xFF;
		AppData.Buff[22]= mediaIrradiator&0xFF;
	}

	LORA_send((lora_AppData_t*)&AppData, LORAWAN_DEFAULT_CONFIRM_MSG_STATE);

}


static void LORA_RxData(lora_AppData_t *AppData)
{

  PRINTF("PACKET RECEIVED ON PORT %d\n\r", AppData->Port);

  switch (AppData->Port)
  {
    case LORAWAN_CHANGE_CLASS_PORT:
    	/*this port switches the class*/
		if (AppData->BuffSize == 1)
		{
		switch (AppData->Buff[0])
		{
		  case CHANGE_TO_CLASS_A:
		  {
			LORA_RequestClass(CLASS_A);
			break;
		  }
		  case CHANGE_TO_CLASS_B:
		  {
			LORA_RequestClass(CLASS_B);
			break;
		  }
		  case CHANGE_TO_CLASS_C:
		  {
			LORA_RequestClass(CLASS_C);
			break;
		  }
		  default:
			break;
		}
		}
		break;
    case LORAWAN_APP_PORT:
    	//TODO Atualizar Data e Hora do dispositivo
    	if(AppData->BuffSize == 7)
    	{
    		DateTime_Update(AppData->Buff);
			PRINTF("DATE-TIME UPDATED \n\r");
		}
    	break;
    default:
    	break;
  }
}

static void OnTxTimerEvent(void *context)
{
  /*Wait for next tx slot*/
	TimerStart(&TxTimer);
	AppProcessRequest = LORA_SET;
}

static void LoraStartTx(TxEventType_t EventType)
{
  if (EventType == TX_ON_TIMER)
  {
    /* send everytime timer elapses */
    TimerInit(&TxTimer, OnTxTimerEvent);
    TimerSetValue(&TxTimer,  APP_TX_DUTYCYCLE);
    OnTxTimerEvent(NULL);
  }

}

static void LORA_ConfirmClass(DeviceClass_t Class)
{
  PRINTF("switch to class %c done\n\r", "ABC"[Class]);

  /*Optionnal*/
  /*informs the server that switch has occurred ASAP*/
  AppData.BuffSize = 0;
  AppData.Port = LORAWAN_APP_PORT;

  LORA_send((lora_AppData_t*)&AppData, LORAWAN_UNCONFIRMED_MSG);
}

static void LORA_TxNeeded(void)
{
  AppData.BuffSize = 0;
  AppData.Port = LORAWAN_APP_PORT;

  LORA_send((lora_AppData_t*)&AppData, LORAWAN_UNCONFIRMED_MSG);
}

/**
  * @brief This function return the battery level
  * @param none
  * @retval the battery level  1 (very low) to 254 (fully charged)
  */
uint8_t LORA_GetBatteryLevel(void)
{
  return 0xFF;
}

static uint8_t store_weather_data(void){
	return 0;
}

/*****************************END OF FILE*****************************/

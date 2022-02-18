/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2013 Semtech

Description: MCU RTC timer

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Miguel Luis and Gregory Cristian
*/
/**
  ******************************************************************************
  * @file    hw_rtc.c
  * @author  MCD Application Team
  * @brief   driver for RTC
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
#include <math.h>
#include <time.h>
#include "hw.h"
#include "low_power_manager.h"
#include "systime.h"
#include "station.h"

/* Private typedef -----------------------------------------------------------*/
typedef struct
{
  uint32_t  Rtc_Time; /* Reference time */

  RTC_TimeTypeDef RTC_Calndr_Time; /* Reference time in calendar format */

  RTC_DateTypeDef RTC_Calndr_Date; /* Reference date in calendar format */

} RtcTimerContext_t;

/* Private define ------------------------------------------------------------*/
#define MAGIC_NUMBER_CALENDAR_POSITION BKPSRAM_BASE
#define BKP_HOURS_POSITION    BKPSRAM_BASE+5
#define BKP_MINUTES_POSITION  BKPSRAM_BASE+6
#define BKP_SECONDS_POSITION  BKPSRAM_BASE+7
#define BKP_YEAR_POSITION     BKPSRAM_BASE+8
#define BKP_MONTH_POSITION    BKPSRAM_BASE+9
#define BKP_DAY_POSITION      BKPSRAM_BASE+10
#define BKP_WEEK_DAY_POSITION BKPSRAM_BASE+11

#define MEMO_NUMBER			  0x32F2

//sum of first 3 letters of the week day
#define MON 298
#define TUE 302
#define WED 288
#define THU 305
#define FRI 289
#define SAT 296
#define SUN 310

//sum of first 3 letters of the month
#define JAN 281
#define FEB 269
#define MAR 288
#define APR 291
#define MAY 295
#define JUN 301
#define JUL 299
#define AUG 285
#define SEP 296
#define OCT 294
#define NOV 307
#define DEC 268



/* MCU Wake Up Time */
#define MIN_ALARM_DELAY               3 /* in ticks */

/* subsecond number of bits */
#define N_PREDIV_S                 10

/* Synchonuous prediv  */
#define PREDIV_S                  7999//((1<<N_PREDIV_S)-1) //1023

/* Asynchonuous prediv   */
#define PREDIV_A                  124//(1<<(15-N_PREDIV_S))-1 //31

/* Sub-second mask definition  */
#define HW_RTC_ALARMSUBSECONDMASK (N_PREDIV_S<<RTC_ALRMASSR_MASKSS_Pos)

/* RTC Time base in us */
#define USEC_NUMBER               1000000
#define MSEC_NUMBER               (USEC_NUMBER/1000)
#define RTC_ALARM_TIME_BASE       (USEC_NUMBER>>N_PREDIV_S)

#define COMMON_FACTOR        3
#define CONV_NUMER                (MSEC_NUMBER>>COMMON_FACTOR)
#define CONV_DENOM                (1<<(N_PREDIV_S-COMMON_FACTOR))

#define DAYS_IN_LEAP_YEAR                        ( ( uint32_t )  366U )
#define DAYS_IN_YEAR                             ( ( uint32_t )  365U )
#define SECONDS_IN_1DAY                          ( ( uint32_t )86400U )
#define SECONDS_IN_1HOUR                         ( ( uint32_t ) 3600U )
#define SECONDS_IN_1MINUTE                       ( ( uint32_t )   60U )
#define MINUTES_IN_1HOUR                         ( ( uint32_t )   60U )
#define HOURS_IN_1DAY                            ( ( uint32_t )   24U )

#define  DAYS_IN_MONTH_CORRECTION_NORM     ((uint32_t) 0x99AAA0 )
#define  DAYS_IN_MONTH_CORRECTION_LEAP     ((uint32_t) 0x445550 )

#define DIVC( X, N )                                ( ( ( X ) + ( N ) -1 ) / ( N ) )
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/*!
 * \brief Indicates if the RTC is already Initalized or not
 */
static bool HW_RTC_Initalized = false;

/*!
 * \brief compensates MCU wakeup time
 */

static bool McuWakeUpTimeInitialized = false;

/*!
 * \brief compensates MCU wakeup time
 */

static int16_t McuWakeUpTimeCal = 0;

/*!
 * Number of days in each month on a normal year
 */
static const uint8_t DaysInMonth[] = { 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };

/*!
 * Number of days in each month on a leap year
 */
static const uint8_t DaysInMonthLeapYear[] = { 31, 29, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };

static RTC_HandleTypeDef RtcHandle = {0};

static RTC_AlarmTypeDef RTC_AlarmStructure;

/*!
 * Keep the value of the RTC timer when the RTC alarm is set
 * Set with the HW_RTC_SetTimerContext function
 * Value is kept as a Reference to calculate alarm
 */
static RtcTimerContext_t RtcTimerContext;

/* Private function prototypes -----------------------------------------------*/

static void HW_RTC_SetConfig(void);

static void HW_RTC_SetAlarmConfig(void);

static void HW_RTC_StartWakeUpAlarm(uint32_t timeoutValue);

static uint64_t HW_RTC_GetCalendarValue(RTC_DateTypeDef *RTC_DateStruct, RTC_TimeTypeDef *RTC_TimeStruct);

static void set_date_time_on_start(void);

static void get_datetime_from_compilation(RTC_TimeTypeDef *RTC_TimeStruct_Real,	RTC_DateTypeDef *RTC_DateStruct_Real);

static uint8_t get_month(void);
static uint8_t get_week_day(void);


/* Exported functions ---------------------------------------------------------*/

/*!
 * @brief Initializes the RTC timer
 * @note The timer is based on the RTC
 * @param none
 * @retval none
 */
void HW_RTC_Init(void)
{
	if (HW_RTC_Initalized == false) {
		HW_RTC_SetConfig();
		//HW_RTC_SetAlarmConfig();
		HW_RTC_SetTimerContext();
		HW_RTC_Initalized = true;
	}
}

/*!
 * @brief Configures the RTC timer
 * @note The timer is based on the RTC
 * @param none
 * @retval none
 */
static void HW_RTC_SetConfig(void)
{
	RtcHandle.Instance = RTC;
	RtcHandle.Init.HourFormat = RTC_HOURFORMAT_24;
	RtcHandle.Init.AsynchPrediv = PREDIV_A; /*RTC_ASYNCH_PREDIV; */
	RtcHandle.Init.SynchPrediv = PREDIV_S; /*RTC_SYNCH_PREDIV; */
	RtcHandle.Init.OutPut = RTC_OUTPUT_DISABLE;
	RtcHandle.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
	RtcHandle.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;

	PRINTF("\r\nENTROU RTC\r\n");
	if(HAL_RTC_Init(&RtcHandle)!=HAL_OK){
		while(1);
	}

	set_date_time_on_start();

	HAL_RTCEx_SetSmoothCalib(&RtcHandle, RTC_SMOOTHCALIB_PERIOD_32SEC, RTC_SMOOTHCALIB_PLUSPULSES_RESET, 130);
	HAL_RTCEx_EnableBypassShadow(&RtcHandle);
}


/*!
 * @brief calculates the wake up time between wake up and mcu start
 * @note resulotion in RTC_ALARM_TIME_BASE in timer ticks
 * @param none
 * @retval none
 */
void HW_RTC_setMcuWakeUpTime(void)
{
  RTC_TimeTypeDef RTC_TimeStruct;
  RTC_DateTypeDef RTC_DateStruct;

  TimerTime_t now, hit;
  int16_t McuWakeUpTime;

  if ((McuWakeUpTimeInitialized == false) &&
      (HAL_NVIC_GetPendingIRQ(RTC_Alarm_IRQn) == 1))
  {
    /* warning: works ok if now is below 30 days
       it is ok since it's done once at first alarm wake-up*/
    McuWakeUpTimeInitialized = true;
    now = (uint32_t) HW_RTC_GetCalendarValue(&RTC_DateStruct, &RTC_TimeStruct);

    HAL_RTC_GetAlarm(&RtcHandle, &RTC_AlarmStructure, RTC_ALARM_A, RTC_FORMAT_BIN);
    hit = RTC_AlarmStructure.AlarmTime.Seconds +
          60 * (RTC_AlarmStructure.AlarmTime.Minutes +
                60 * (RTC_AlarmStructure.AlarmTime.Hours +
                      24 * (RTC_AlarmStructure.AlarmDateWeekDay)));
    hit = (hit << N_PREDIV_S) + (PREDIV_S - RTC_AlarmStructure.AlarmTime.SubSeconds);

    McuWakeUpTime = (int16_t)((now - hit));
    McuWakeUpTimeCal += McuWakeUpTime;
  }
}

int16_t HW_RTC_getMcuWakeUpTime(void)
{
  return McuWakeUpTimeCal;
}

/*!
 * @brief returns the wake up time in ticks
 * @param none
 * @retval wake up time in ticks
 */
uint32_t HW_RTC_GetMinimumTimeout(void)
{
  return (MIN_ALARM_DELAY);
}

/*!
 * @brief converts time in ms to time in ticks
 * @param [IN] time in milliseconds
 * @retval returns time in timer ticks
 */
uint32_t HW_RTC_ms2Tick(TimerTime_t timeMilliSec)
{
  /*return( ( timeMicroSec / RTC_ALARM_TIME_BASE ) ); */
  return (uint32_t)((((uint64_t)timeMilliSec) * CONV_DENOM) / CONV_NUMER);
}

/*!
 * @brief converts time in ticks to time in ms
 * @param [IN] time in timer ticks
 * @retval returns time in milliseconds
 */
TimerTime_t HW_RTC_Tick2ms(uint32_t tick)
{
  /*return( ( timeMicroSec * RTC_ALARM_TIME_BASE ) ); */
  uint32_t seconds = tick >> N_PREDIV_S;
  tick = tick & PREDIV_S;
  return ((seconds * 1000) + ((tick * 1000) >> N_PREDIV_S));
}

/*!
 * @brief Set the alarm
 * @note The alarm is set at now (read in this funtion) + timeout
 * @param timeout Duration of the Timer ticks
 */
void HW_RTC_SetAlarm(uint32_t timeout)
{
  /* we don't go in Low Power mode for timeout below MIN_ALARM_DELAY */
  if ((MIN_ALARM_DELAY + McuWakeUpTimeCal) < ((timeout - HW_RTC_GetTimerElapsedTime())))
  {
    LPM_SetStopMode(LPM_RTC_Id, LPM_Enable);
  }
  else
  {
    LPM_SetStopMode(LPM_RTC_Id, LPM_Disable);
  }

  /*In case stop mode is required */
  if (LPM_GetMode() == LPM_StopMode)
  {
    timeout = timeout -  McuWakeUpTimeCal;
  }

  HW_RTC_StartWakeUpAlarm(timeout);
}

/*!
 * @brief Get the RTC timer elapsed time since the last Alarm was set
 * @param none
 * @retval RTC Elapsed time in ticks
 */
uint32_t HW_RTC_GetTimerElapsedTime(void)
{
  RTC_TimeTypeDef RTC_TimeStruct;
  RTC_DateTypeDef RTC_DateStruct;

  uint32_t CalendarValue = (uint32_t) HW_RTC_GetCalendarValue(&RTC_DateStruct, &RTC_TimeStruct);

  return ((uint32_t)(CalendarValue - RtcTimerContext.Rtc_Time));
}

/*!
 * @brief Get the RTC timer value
 * @param none
 * @retval RTC Timer value in ticks
 */
uint32_t HW_RTC_GetTimerValue(void)
{
  RTC_TimeTypeDef RTC_TimeStruct;
  RTC_DateTypeDef RTC_DateStruct;

  uint32_t CalendarValue = (uint32_t) HW_RTC_GetCalendarValue(&RTC_DateStruct, &RTC_TimeStruct);

  return (CalendarValue);
}

/*!
 * @brief Stop the Alarm
 * @param none
 * @retval none
 */
void HW_RTC_StopAlarm(void)
{
  /* Disable the Alarm A interrupt */
  HAL_RTC_DeactivateAlarm(&RtcHandle, RTC_ALARM_A);
  /* Clear RTC Alarm Flag */
  __HAL_RTC_ALARM_CLEAR_FLAG(&RtcHandle, RTC_FLAG_ALRAF);
  /* Clear the EXTI's line Flag for RTC Alarm */
  __HAL_RTC_ALARM_EXTI_CLEAR_FLAG();
}

/*!
 * @brief RTC IRQ Handler on the RTC Alarm
 * @param none
 * @retval none
 */
void HW_RTC_IrqHandler(void)
{
  RTC_HandleTypeDef *hrtc = &RtcHandle;
  /* enable low power at irq*/
  LPM_SetStopMode(LPM_RTC_Id, LPM_Enable);

  /* Clear the EXTI's line Flag for RTC Alarm */
  __HAL_RTC_ALARM_EXTI_CLEAR_FLAG();

  /* Get the AlarmA interrupt source enable status */
  if (__HAL_RTC_ALARM_GET_IT_SOURCE(hrtc, RTC_IT_ALRA) != RESET)
  {
    /* Get the pending status of the AlarmA Interrupt */
    if (__HAL_RTC_ALARM_GET_FLAG(hrtc, RTC_FLAG_ALRAF) != RESET)
    {
      /* Clear the AlarmA interrupt pending bit */
      __HAL_RTC_ALARM_CLEAR_FLAG(hrtc, RTC_FLAG_ALRAF);
      /* AlarmA callback */
      HAL_RTC_AlarmAEventCallback(hrtc);
    }
  }
}


/*!
 * @brief a delay of delay ms by polling RTC
 * @param delay in ms
 * @retval none
 */
void HW_RTC_DelayMs(uint32_t delay)
{
  TimerTime_t delayValue = 0;
  TimerTime_t timeout = 0;

  delayValue = HW_RTC_ms2Tick(delay);

  /* Wait delay ms */
  timeout = HW_RTC_GetTimerValue();
  while (((HW_RTC_GetTimerValue() - timeout)) < delayValue)
  {
    __NOP();
  }
}

/*!
 * @brief set Time Reference set also the RTC_DateStruct and RTC_TimeStruct
 * @param none
 * @retval Timer Value
 */
uint32_t HW_RTC_SetTimerContext(void)
{
  RtcTimerContext.Rtc_Time = (uint32_t) HW_RTC_GetCalendarValue(&RtcTimerContext.RTC_Calndr_Date, &RtcTimerContext.RTC_Calndr_Time);
  return (uint32_t) RtcTimerContext.Rtc_Time;
}

/*!
 * @brief Get the RTC timer Reference
 * @param none
 * @retval Timer Value in  Ticks
 */
uint32_t HW_RTC_GetTimerContext(void)
{
  return RtcTimerContext.Rtc_Time;
}
/* Private functions ---------------------------------------------------------*/

/*!
 * @brief configure alarm at init
 * @param none
 * @retval none
 */
static void HW_RTC_SetAlarmConfig(void)
{
  HAL_RTC_DeactivateAlarm(&RtcHandle, RTC_ALARM_A);
}

/*!
 * @brief start wake up alarm
 * @note  alarm in RtcTimerContext.Rtc_Time + timeoutValue
 * @param timeoutValue in ticks
 * @retval none
 */
static void HW_RTC_StartWakeUpAlarm(uint32_t timeoutValue)
{
  uint16_t rtcAlarmSubSeconds = 0;
  uint16_t rtcAlarmSeconds = 0;
  uint16_t rtcAlarmMinutes = 0;
  uint16_t rtcAlarmHours = 0;
  uint16_t rtcAlarmDays = 0;
  RTC_TimeTypeDef RTC_TimeStruct = RtcTimerContext.RTC_Calndr_Time;
  RTC_DateTypeDef RTC_DateStruct = RtcTimerContext.RTC_Calndr_Date;

  HW_RTC_StopAlarm();

  /*reverse counter */
  rtcAlarmSubSeconds =  PREDIV_S - RTC_TimeStruct.SubSeconds;
  rtcAlarmSubSeconds += (timeoutValue & PREDIV_S);
  /* convert timeout  to seconds */
  timeoutValue >>= N_PREDIV_S;  /* convert timeout  in seconds */

  /*convert microsecs to RTC format and add to 'Now' */
  rtcAlarmDays =  RTC_DateStruct.Date;
  while (timeoutValue >= SECONDS_IN_1DAY)
  {
    timeoutValue -= SECONDS_IN_1DAY;
    rtcAlarmDays++;
  }

  /* calc hours */
  rtcAlarmHours = RTC_TimeStruct.Hours;
  while (timeoutValue >= SECONDS_IN_1HOUR)
  {
    timeoutValue -= SECONDS_IN_1HOUR;
    rtcAlarmHours++;
  }

  /* calc minutes */
  rtcAlarmMinutes = RTC_TimeStruct.Minutes;
  while (timeoutValue >= SECONDS_IN_1MINUTE)
  {
    timeoutValue -= SECONDS_IN_1MINUTE;
    rtcAlarmMinutes++;
  }

  /* calc seconds */
  rtcAlarmSeconds =  RTC_TimeStruct.Seconds + timeoutValue;

  /***** correct for modulo********/
  while (rtcAlarmSubSeconds >= (PREDIV_S + 1))
  {
    rtcAlarmSubSeconds -= (PREDIV_S + 1);
    rtcAlarmSeconds++;
  }

  while (rtcAlarmSeconds >= SECONDS_IN_1MINUTE)
  {
    rtcAlarmSeconds -= SECONDS_IN_1MINUTE;
    rtcAlarmMinutes++;
  }

  while (rtcAlarmMinutes >= MINUTES_IN_1HOUR)
  {
    rtcAlarmMinutes -= MINUTES_IN_1HOUR;
    rtcAlarmHours++;
  }

  while (rtcAlarmHours >= HOURS_IN_1DAY)
  {
    rtcAlarmHours -= HOURS_IN_1DAY;
    rtcAlarmDays++;
  }

  if (RTC_DateStruct.Year % 4 == 0)
  {
    if (rtcAlarmDays > DaysInMonthLeapYear[ RTC_DateStruct.Month - 1 ])
    {
      rtcAlarmDays = rtcAlarmDays % DaysInMonthLeapYear[ RTC_DateStruct.Month - 1 ];
    }
  }
  else
  {
    if (rtcAlarmDays > DaysInMonth[ RTC_DateStruct.Month - 1 ])
    {
      rtcAlarmDays = rtcAlarmDays % DaysInMonth[ RTC_DateStruct.Month - 1 ];
    }
  }

  /* Set RTC_AlarmStructure with calculated values*/
  RTC_AlarmStructure.AlarmTime.SubSeconds = PREDIV_S - rtcAlarmSubSeconds;
  RTC_AlarmStructure.AlarmSubSecondMask  = HW_RTC_ALARMSUBSECONDMASK;
  RTC_AlarmStructure.AlarmTime.Seconds = rtcAlarmSeconds;
  RTC_AlarmStructure.AlarmTime.Minutes = rtcAlarmMinutes;
  RTC_AlarmStructure.AlarmTime.Hours   = rtcAlarmHours;
  RTC_AlarmStructure.AlarmDateWeekDay    = (uint8_t)rtcAlarmDays;
  RTC_AlarmStructure.AlarmTime.TimeFormat   = RTC_TimeStruct.TimeFormat;
  RTC_AlarmStructure.AlarmDateWeekDaySel   = RTC_ALARMDATEWEEKDAYSEL_DATE;
  RTC_AlarmStructure.AlarmMask       = RTC_ALARMMASK_NONE;
  RTC_AlarmStructure.Alarm = RTC_ALARM_A;
  RTC_AlarmStructure.AlarmTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  RTC_AlarmStructure.AlarmTime.StoreOperation = RTC_STOREOPERATION_RESET;

  /* Set RTC_Alarm */
  HAL_RTC_SetAlarm_IT(&RtcHandle, &RTC_AlarmStructure, RTC_FORMAT_BIN);
}


/*!
 * @brief get current time from calendar in ticks
 * @param pointer to RTC_DateStruct
 * @param pointer to RTC_TimeStruct
 * @retval time in ticks
 */
static uint64_t HW_RTC_GetCalendarValue(RTC_DateTypeDef *RTC_DateStruct, RTC_TimeTypeDef *RTC_TimeStruct)
{
  uint64_t calendarValue = 0;
  uint32_t first_read;
  uint32_t correction;
  uint32_t seconds;

  /* Get Time and Date*/
  HAL_RTC_GetTime(&RtcHandle, RTC_TimeStruct, RTC_FORMAT_BIN);

  /* make sure it is correct due to asynchronus nature of RTC*/
  do
  {
    first_read = LL_RTC_TIME_GetSubSecond(RTC);
    HAL_RTC_GetDate(&RtcHandle, RTC_DateStruct, RTC_FORMAT_BIN);
    HAL_RTC_GetTime(&RtcHandle, RTC_TimeStruct, RTC_FORMAT_BIN);

  }
  while (first_read != LL_RTC_TIME_GetSubSecond(RTC));

  /* calculte amount of elapsed days since 01/01/2000 */
  seconds = DIVC((DAYS_IN_YEAR * 3 + DAYS_IN_LEAP_YEAR) * RTC_DateStruct->Year, 4);

  correction = ((RTC_DateStruct->Year % 4) == 0) ? DAYS_IN_MONTH_CORRECTION_LEAP : DAYS_IN_MONTH_CORRECTION_NORM ;

  seconds += (DIVC((RTC_DateStruct->Month - 1) * (30 + 31), 2) - (((correction >> ((RTC_DateStruct->Month - 1) * 2)) & 0x3)));

  seconds += (RTC_DateStruct->Date - 1);

  /* convert from days to seconds */
  seconds *= SECONDS_IN_1DAY;

  seconds += ((uint32_t)RTC_TimeStruct->Seconds +
              ((uint32_t)RTC_TimeStruct->Minutes * SECONDS_IN_1MINUTE) +
              ((uint32_t)RTC_TimeStruct->Hours * SECONDS_IN_1HOUR)) ;



  calendarValue = (((uint64_t) seconds) << N_PREDIV_S) + (PREDIV_S - RTC_TimeStruct->SubSeconds);

  return (calendarValue);
}

/*!
 * \brief Get system time
 * \param [IN]   pointer to ms
 *
 * \return uint32_t seconds
 */
uint32_t HW_RTC_GetCalendarTime(uint16_t *mSeconds)
{
  RTC_TimeTypeDef RTC_TimeStruct ;
  RTC_DateTypeDef RTC_DateStruct;
  uint32_t ticks;

  uint64_t calendarValue = HW_RTC_GetCalendarValue(&RTC_DateStruct, &RTC_TimeStruct);

  uint32_t seconds = (uint32_t)(calendarValue >> N_PREDIV_S);

  ticks = (uint32_t) calendarValue & PREDIV_S;

  *mSeconds = HW_RTC_Tick2ms(ticks);

  return seconds;
}

void HW_RTC_BKUPWrite(uint32_t Data0, uint32_t Data1)
{
  HAL_RTCEx_BKUPWrite(&RtcHandle, RTC_BKP_DR0, Data0);
  HAL_RTCEx_BKUPWrite(&RtcHandle, RTC_BKP_DR1, Data1);
}

void HW_RTC_BKUPRead(uint32_t *Data0, uint32_t *Data1)
{
  *Data0 = HAL_RTCEx_BKUPRead(&RtcHandle, RTC_BKP_DR0);
  *Data1 = HAL_RTCEx_BKUPRead(&RtcHandle, RTC_BKP_DR1);
}

TimerTime_t RtcTempCompensation(TimerTime_t period, float temperature)
{
  float k = RTC_TEMP_COEFFICIENT;
  float kDev = RTC_TEMP_DEV_COEFFICIENT;
  float t = RTC_TEMP_TURNOVER;
  float tDev = RTC_TEMP_DEV_TURNOVER;
  float interim = 0.0;
  float ppm = 0.0;

  if (k < 0.0f)
  {
    ppm = (k - kDev);
  }
  else
  {
    ppm = (k + kDev);
  }
  interim = (temperature - (t - tDev));
  ppm *=  interim * interim;

  // Calculate the drift in time
  interim = ((float) period * ppm) / 1000000;
  // Calculate the resulting time period
  interim += period;
  interim = floor(interim);

  if (interim < 0.0f)
  {
    interim = (float)period;
  }

  // Calculate the resulting period
  return (TimerTime_t) interim;
}

void get_time_now (uint8_t* buffer_datetime)
{
	RTC_DateTypeDef sDate;
	RTC_TimeTypeDef sTime;
	unsigned char *p;

	HAL_RTC_GetTime(&RtcHandle, &sTime, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&RtcHandle, &sDate, RTC_FORMAT_BIN);

	//get first position from sDate struct
    p = (unsigned char*)&sDate;
    p++; //ignores WeekDay

    //put date in order month, day, year on buffer
    for (int i=0;i<sizeof(RTC_DateTypeDef)-1;i++)
    {
    	buffer_datetime[i]=*p++;
    }

    //get first position from sTime struct
    p = (unsigned char*)&sTime;

    //put time in order hour, minute, second on buffer
    for (int i=3;i<6;i++)
	{
    	buffer_datetime[i]=*p++;
	}
}

/*!
 * @brief Update the RTC timer from RxData
 * @note The timer is based on the RTC
 * @param buffer_datetime_real	RxData buffer
 * @retval none
 */
void DateTime_Update(uint8_t* buffer_datetime_real) {
	RTC_TimeTypeDef RTC_TimeStruct_Real;
	RTC_DateTypeDef RTC_DateStruct_Real;

	// [Wd, Mo, D, Y, H, M, S]

	RTC_DateStruct_Real.Year = buffer_datetime_real[3]; // 0-infinite
	RTC_DateStruct_Real.Month = buffer_datetime_real[1]; // 0-12
	RTC_DateStruct_Real.Date = buffer_datetime_real[2]; // 1-31
	RTC_DateStruct_Real.WeekDay = buffer_datetime_real[0]; // Seg 1 ... Dom 7
	HAL_RTC_SetDate(&RtcHandle, &RTC_DateStruct_Real, RTC_FORMAT_BIN);

	/*at 0:0:0*/
	RTC_TimeStruct_Real.Hours = buffer_datetime_real[4]; // 0-23
	RTC_TimeStruct_Real.Minutes = buffer_datetime_real[5]; // 0-59
	RTC_TimeStruct_Real.Seconds = buffer_datetime_real[6]; // 0-59
	RTC_TimeStruct_Real.TimeFormat = 0;
	RTC_TimeStruct_Real.SubSeconds = 0;
	RTC_TimeStruct_Real.StoreOperation = RTC_DAYLIGHTSAVING_NONE;
	RTC_TimeStruct_Real.DayLightSaving = RTC_STOREOPERATION_RESET;
	HAL_RTC_SetTime(&RtcHandle, &RTC_TimeStruct_Real, RTC_FORMAT_BIN);

	HAL_RTCEx_BKUPWrite(&RtcHandle, RTC_BKP_DR3, MEMO_NUMBER);
	write_sram_bckp(MEMO_NUMBER, MAGIC_NUMBER_CALENDAR_POSITION, _16BITS);

}

uint32_t HW_RTC_Read_Data(uint32_t position)
{
	for (int i=RTC_BKP_DR0; i< RTC_BKP_DR19;i++){
		PRINTF("bckp%d: %ld\r\n",i,  HAL_RTCEx_BKUPRead(&RtcHandle, i));
	}
	return HAL_RTCEx_BKUPRead(&RtcHandle, position);
}

void HW_RTC_Write_Data(uint32_t position, uint32_t data)
{
	HAL_PWR_EnableBkUpAccess();
	HAL_RTCEx_BKUPWrite(&RtcHandle, position, data);
	HAL_PWR_DisableBkUpAccess();
}

void write_time_to_backup(void)
{
    RTC_TimeTypeDef RTC_TimeStruct;
    RTC_DateTypeDef RTC_DateStruct;
    uint32_t first_read;
    HAL_RTC_GetTime(&RtcHandle, &RTC_TimeStruct, RTC_FORMAT_BIN);


    /* make sure it is correct due to asynchronus nature of RTC*/
	do
	{
		first_read = LL_RTC_TIME_GetSubSecond(RTC);
		HAL_RTC_GetDate(&RtcHandle, &RTC_DateStruct, RTC_FORMAT_BIN);
		HAL_RTC_GetTime(&RtcHandle, &RTC_TimeStruct, RTC_FORMAT_BIN);

	}while (first_read != LL_RTC_TIME_GetSubSecond(RTC));


    write_sram_bckp(RTC_TimeStruct.Hours, BKP_HOURS_POSITION, _8BITS);
    write_sram_bckp(RTC_TimeStruct.Minutes, BKP_MINUTES_POSITION, _8BITS);
    write_sram_bckp(RTC_TimeStruct.Seconds, BKP_SECONDS_POSITION, _8BITS);
    write_sram_bckp(RTC_DateStruct.Year, BKP_YEAR_POSITION, _8BITS);
    write_sram_bckp(RTC_DateStruct.Month, BKP_MONTH_POSITION, _8BITS);
    write_sram_bckp(RTC_DateStruct.Date, BKP_DAY_POSITION, _8BITS);
    write_sram_bckp(RTC_DateStruct.WeekDay, BKP_WEEK_DAY_POSITION, _8BITS);
}


static uint8_t get_week_day(void){
	uint16_t weekday = __TIMESTAMP__[0]+__TIMESTAMP__[1]+__TIMESTAMP__[2];

	switch (weekday){
	case MON:
		return RTC_WEEKDAY_MONDAY;
	case TUE:
		return RTC_WEEKDAY_TUESDAY;
	case WED:
		return RTC_WEEKDAY_WEDNESDAY;
	case THU:
		return RTC_WEEKDAY_THURSDAY;
	case FRI:
		return RTC_WEEKDAY_FRIDAY;
	case SAT:
		return RTC_WEEKDAY_SATURDAY;
	case SUN:
		return RTC_WEEKDAY_SUNDAY;
	}
	return -1;

}

static uint8_t get_month(void){
	uint16_t month = __TIMESTAMP__[4]+__TIMESTAMP__[5]+__TIMESTAMP__[6];
	switch (month){
	case JAN:
		return RTC_MONTH_JANUARY;
	case FEB:
		return RTC_MONTH_FEBRUARY;
	case MAR:
		return RTC_MONTH_MARCH;
	case APR:
		return RTC_MONTH_APRIL;
	case MAY:
		return RTC_MONTH_MAY;
	case JUN:
		return RTC_MONTH_JUNE;
	case JUL:
		return RTC_MONTH_JULY;
	case AUG:
		return RTC_MONTH_AUGUST;
	case SEP:
		return RTC_MONTH_SEPTEMBER;
	case OCT:
		return RTC_MONTH_OCTOBER;
	case NOV:
		return RTC_MONTH_NOVEMBER;
	case DEC:
		return RTC_MONTH_DECEMBER;
	}
	return -1;
}

static void get_datetime_from_compilation(RTC_TimeTypeDef *RTC_TimeStruct_Real,	RTC_DateTypeDef *RTC_DateStruct_Real){
	char s[2];

	s[0] = __TIMESTAMP__[11];
	s[1] = __TIMESTAMP__[12];

	RTC_TimeStruct_Real->Hours = atoi(s);
	RTC_TimeStruct_Real->Hours = 9;
	RTC_TimeStruct_Real->Hours = 10;
	PRINTF("%s -> Hr: %d \r\n", s, RTC_TimeStruct_Real->Hours);

	s[0] = __TIMESTAMP__[14];
	s[1] = __TIMESTAMP__[15];
	RTC_TimeStruct_Real->Minutes = atoi(s);
	RTC_TimeStruct_Real->Minutes = 10;
	PRINTF("%s -> Min: %d \r\n", s, RTC_TimeStruct_Real->Minutes);

	s[0] = __TIMESTAMP__[17];
	s[1] = __TIMESTAMP__[18];
	RTC_TimeStruct_Real->Seconds = atoi(s);
	RTC_TimeStruct_Real->Seconds = 00;
	PRINTF("%s -> Sec: %d \r\n", s, RTC_TimeStruct_Real->Seconds);

	s[0] = __TIMESTAMP__[8];
	s[1] = __TIMESTAMP__[9];
	RTC_DateStruct_Real->Date = atoi(s);
	RTC_DateStruct_Real->Date = 13;
	PRINTF("%s -> Dt: %d \r\n",s, RTC_DateStruct_Real->Date);

	s[0] = __TIMESTAMP__[22];
	s[1] = __TIMESTAMP__[23];
	RTC_DateStruct_Real->Year = atoi(s);
	RTC_DateStruct_Real->Year = 22;
	PRINTF("%s -> Yr: %d \r\n", s, RTC_DateStruct_Real->Year);

	RTC_DateStruct_Real->Month = get_month();
	RTC_DateStruct_Real->Month = RTC_MONTH_JANUARY;
	PRINTF("Month: %d \r\n", RTC_DateStruct_Real->Month);

	RTC_DateStruct_Real->WeekDay = get_week_day();
	RTC_DateStruct_Real->WeekDay = RTC_WEEKDAY_THURSDAY;
	PRINTF("WeekDay: %d \r\n", RTC_DateStruct_Real->WeekDay);

}

static void set_date_time_on_start(void){
	RTC_TimeTypeDef RTC_TimeStruct;
	RTC_DateTypeDef RTC_DateStruct;
	if ((read_sram_bckp(MAGIC_NUMBER_CALENDAR_POSITION, _16BITS)) != MEMO_NUMBER) {
		PRINTF("ENTROU set_date_time_on_start\r\n");
		write_sram_bckp(0, PLUVIOMETER_CNT_REGISTER, _16BITS);
		get_datetime_from_compilation(&RTC_TimeStruct, &RTC_DateStruct);
		RTC_TimeStruct.StoreOperation = RTC_DAYLIGHTSAVING_NONE;
		RTC_TimeStruct.DayLightSaving = RTC_STOREOPERATION_RESET;
		HAL_RTC_SetTime(&RtcHandle, &RTC_TimeStruct, RTC_FORMAT_BIN);
		HAL_RTC_SetDate(&RtcHandle, &RTC_DateStruct, RTC_FORMAT_BIN);

		write_sram_bckp(MEMO_NUMBER, MAGIC_NUMBER_CALENDAR_POSITION, _16BITS);
		HAL_RTCEx_BKUPWrite(&RtcHandle, RTC_BKP_DR3, MEMO_NUMBER);
	}
	else{
		RTC_TimeStruct.Hours = read_sram_bckp(BKP_HOURS_POSITION, _8BITS);
		RTC_TimeStruct.Minutes = read_sram_bckp(BKP_MINUTES_POSITION, _8BITS);
		RTC_TimeStruct.Seconds = read_sram_bckp(BKP_SECONDS_POSITION, _8BITS);
		RTC_TimeStruct.StoreOperation = RTC_DAYLIGHTSAVING_NONE;
		RTC_TimeStruct.DayLightSaving = RTC_STOREOPERATION_RESET;
		HAL_RTC_SetTime(&RtcHandle, &RTC_TimeStruct, RTC_FORMAT_BIN);

		RTC_DateStruct.Year = read_sram_bckp(BKP_YEAR_POSITION, _8BITS);
		RTC_DateStruct.Month = read_sram_bckp(BKP_MONTH_POSITION, _8BITS);
		RTC_DateStruct.Date = read_sram_bckp(BKP_DAY_POSITION, _8BITS);
		RTC_DateStruct.WeekDay = read_sram_bckp(BKP_WEEK_DAY_POSITION, _8BITS);
		HAL_RTC_SetDate(&RtcHandle, &RTC_DateStruct, RTC_FORMAT_BIN);


	}
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/


#include <math.h>
#include "board.h"
#include "rtc-board.h"

#include "stm32f10x.h"
#include "stm32f10x_rtc.h"
#
#include "vsn.h"
#include "vsntime.h"
#include "vsndriversconf.h"
#include <time.h>
#include <stdio.h>



//Local defines
/* RTC prescaler: set RTC period to 1 sec */
/* RTC period = RTCCLK/RTC_PR = (32.768 KHz)/(32767+1)
 *
 * RTC Prescaler is set to 32 so RTC clock is set to 1kHz = > T=1ms
 */
#define RTC_PRESCALER_DEFAULT 32
#define RTC_TICK_TO_MS (1.007080)
#define RTC_MS_TO_TICK (0.992970)
#define MICRO_SECOND_IN_SECOND   1000000
#define MILI_SECOND_IN_SECOND    1000
#define MICRO_SECOND_IN_MILI_SECOND 1000
// This is the default time from which RTC starts counting:
// * 1. Jan 2012, 00:00:00
#define DEFAULT_RTC_TIME   1325376000

//Max value of alarm timeout in ms
//Max time that you can set alarm to accoriding to lowapp code(rtc-board.c - RtcComputeTimerTimeToAlarmTick) is
//RTC_ALARM_MAX_NUMBER_OF_DAYS*SecondsInDay*RTC_ALARM_TICK_DURATION = 1181 s = 20min
#define MAX_ALARM_TIMEOUT 1181250




// Macros for optional logging functionality
#ifndef VSNTIME_DEBUG
#define VSNTIME_DEBUG 0
#endif

// Uncomment to get the filenames in the debug output
// #define FILENAME_IN_OUTPUT

#if VSNTIME_DEBUG
#ifdef FILENAME_IN_OUTPUT
#define PRINTF(...)     \
		(printf("%s (%4d) ",__FILE__,__LINE__), printf(__VA_ARGS__))
#else
#define PRINTF(...)     \
        printf(__VA_ARGS__)
#endif
#else
#define PRINTF(...)
#endif

#define UNUSED(x)		(void)x;

// Local constants
// RTC and Backup Domain initialization constant.
// * This value is written to the Backup Register 1 after RTC
// * initialization and is checked in each call of vsnTime_initRtc()
static const uint16_t rtcInitConst = 0xB1B2;

// Local variables
static FlagStatus rtcInitFlag = RESET;
static FlagStatus rtcPrescalerDef = RESET;
static uint16_t rtcInteruptSetting = 0;
// This flag is set by the uptime interrupt routine, It can be used
// * to check if the interrupt has happened between the read of the uptime
// * counters
static uint8_t upTimeInterruptFlag;
/*!
 * \brief Indicates if the RTC is already Initialized or not
 */
static bool RtcInitalized = FALSE;

/*!
 * \brief Flag to disable the LowPower Mode even if the timestamps until the
 * next event is long enough to allow Low Power mode
 */
static bool LowPowerDisableDuringTask = FALSE;

/*!
 * Flag used to indicates a the MCU has waken-up from an external IRQ
 */
volatile bool NonScheduledWakeUp = FALSE;

/*!
 * \brief Flag to indicate if the timestamps until the next event is long enough
 * to set the MCU into low power mode
 */
static bool RtcTimerEventAllowsLowPower = FALSE;

/*!
 * Current ALARM Start time value
 */
static uint32_t RtcAlarmStartTime;

/*!
 * \brief Indicates if the RTC Wake Up Time is calibrated or not
 */
static bool WakeUpTimeInitialized = FALSE;

/*!
 * \brief Hold the Wake-up time duration in ms
 */
volatile uint32_t McuWakeUpTime = 0;


static uint32_t RtcGetCounter (void);



void RtcInit( void )
{
	if(RtcInitalized == FALSE)
	{
	/* Enable PWR and BKP clocks */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);
	/* Check if RTC is already initialized */
	//if (BKP_ReadBackupRegister(BKP_DR1) != rtcInitConst) {
		/* Backup data register value is not correct or not yet programmed (when
		 the program is executed the first time) */
		/* RTC Configuration */
		/* Allow access to BKP Domain */
		PWR_BackupAccessCmd(ENABLE);
		/* Reset Backup Domain */
		BKP_DeInit();
		/* Enable LSE */
		RCC_LSEConfig(RCC_LSE_ON);
		/* Wait till LSE is ready */
		while (RCC_GetFlagStatus(RCC_FLAG_LSERDY) == RESET) {
		}
		/* Select LSE as RTC Clock Source */
		RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);
		/* Enable RTC Clock */
		RCC_RTCCLKCmd(ENABLE);
		/* Wait for RTC registers synchronization */
		RTC_WaitForSynchro();
		/* Wait until last write operation on RTC registers has finished */
		RTC_WaitForLastTask();
		/* Enable the RTC ALARM Interrupt */
		RTC_ITConfig(RTC_IT_ALR, ENABLE);
		/* Wait until last write operation on RTC registers has finished */
		RTC_WaitForLastTask();
		/* Set RTC prescaler: set RTC period to 1sec */
		RTC_SetPrescaler(RTC_PRESCALER_DEFAULT); /* RTC period = RTCCLK/RTC_PR = (32.768 KHz)/(32767+1) */
		/* Wait until last write operation on RTC registers has finished */
		RTC_WaitForLastTask();
		/* Start counting uptime from 0 */
		RTC_SetCounter(0);
		RTC_WaitForLastTask();
		/* Set the RTC Initialized Flag */
		RtcInitalized = TRUE;
	}
}

void RtcSetTimeout( uint32_t timeout )
{
	uint32_t now = 0;
	uint32_t alarm = 0;

	/* Wait for RTC registers synchronization */
	RTC_WaitForSynchro();
	/* Wait until last write operation on RTC registers has finished */
	RTC_WaitForLastTask();
	//Disable ALARM Interrupt
	RTC_ITConfig(RTC_IT_ALR, 0);
	//-------------Calculating value for next Alarm------------------//
	//1 tick is equal to 1.007ms -> For faster calculation we round it to 1 tick = 1ms
	//Check if timeout is longer than max timeout value
	if(timeout > ((uint32_t) MAX_ALARM_TIMEOUT))
		//Limit timeout value to MAX_ALARM_TIMEOUT
		timeout = ((uint32_t) MAX_ALARM_TIMEOUT);
	//Read current value of RTC counter
	now = RtcGetCounter();
	//Add now and timeout values together. If overflow occurs alarm should be set
	//to right value as well, since RTC counter is counting UP
	alarm = (uint32_t)(now + timeout);
	//------------Set RTC ALARM------------------------------------//
	/* Wait until last write operation on RTC registers has finished */
	RTC_WaitForLastTask();
	//Set Alarm
	RTC_SetAlarm(alarm);
	/* Wait until last write operation on RTC registers has finished */
	RTC_WaitForLastTask();
	//Clear any pending Alarm bit
	RTC_ClearITPendingBit(RTC_IT_ALR);
	/* Wait until last write operation on RTC registers has finished */
	RTC_WaitForLastTask();
	//Enable ALARM Interrupt
	RTC_ITConfig(RTC_IT_ALR, 1);
	//Save now value to RtcAlarmStartTime to use it later to calculate elapsed time
	RtcAlarmStartTime = now;
}

//I have not implemented this function so far
TimerTime_t RtcGetAdjustedTimeoutValue( uint32_t timeout )
{
	return timeout;
}

TimerTime_t RtcGetTimerValue( void )
{
	return RtcGetCounter();
}

TimerTime_t RtcGetElapsedAlarmTime( void )
{
	uint32_t currentTime = 0, elapsedTime = 0;
	currentTime = RtcGetCounter();
	//Check if overflow occurred
	if(currentTime < RtcAlarmStartTime)
	{	//Overflow occurred
		elapsedTime = (currentTime + (0xFFFFFFFF - RtcAlarmStartTime));
	}
	else
	{	//No overflow
		elapsedTime = currentTime - RtcAlarmStartTime;
	}
	return elapsedTime;
}

TimerTime_t RtcComputeFutureEventTime( TimerTime_t futureEventInTime )
{
	return (RtcGetCounter() + futureEventInTime);
}

TimerTime_t RtcComputeElapsedTime( TimerTime_t eventInTime )
{
	uint32_t elapsedTime = 0, currentTime = 0;
	if(eventInTime == 0)
	{
		return 0;
	}

	currentTime = RtcGetCounter();
	//Chek if overflow occured
	if(currentTime < eventInTime)
	{
		//Overflow occured
		elapsedTime = currentTime + (0xFFFFFFFF - eventInTime);
	}
	else
	{
		//There was not overflow
		elapsedTime = currentTime-eventInTime;
	}
	return elapsedTime;
}

void BlockLowPowerDuringTask ( bool status )
{
	if(status == TRUE)
	{
		RtcRecoverMcuStatus();
	}
	LowPowerDisableDuringTask = status;
}

//This function has not been implemented jet
//So far it does nothing
void RtcEnterLowPowerStopMode( void )
{
	if((LowPowerDisableDuringTask == FALSE) && (RtcTimerEventAllowsLowPower == TRUE))
	{
		BoardDeInitMcu();
	}
}

/*!
 * \brief Restore the MCU to its normal operation mode
 */
void RtcRecoverMcuStatus( void )
{
	//Check if ALARM IRQ occured
	if(RTC_GetITStatus(RTC_IT_ALR) != RESET)
	{
		RTC_ClearITPendingBit(RTC_IT_ALR);
	}
	else
	{
		//Something else woke up MCU
		NonScheduledWakeUp = TRUE;
	}


	/*check the clk source and set to full speed if we are coming from sleep mode
	if( ( __HAL_RCC_GET_SYSCLK_SOURCE( ) == RCC_SYSCLKSOURCE_STATUS_HSI ) ||
		( __HAL_RCC_GET_SYSCLK_SOURCE( ) == RCC_SYSCLKSOURCE_STATUS_MSI ) )
	{
		BoardInitMcu( );
	}*/
}
// I still need to implement this
void RTC_Alarm_IRQHandler( void )
{
	return;
}

//Local functions
/*!
 * \brief Reads value of RTC Counter and transfer it to ms
 *
 */
static uint32_t RtcGetCounter (void)
{
	uint16_t cntl, cnth;
	uint16_t cnth1 = RTC->CNTH;
	uint16_t cntl1 = RTC->CNTL;

	uint16_t cnth2 = RTC->CNTH;
	uint16_t cntl2 = RTC->CNTL;

	if(cntl1 != cntl2)
	{
		/* overflow occurred between reads of cntl, hence it
		 * couldn't have occurred before the first read. */
		cntl = cntl1;
		cnth = cnth1;
	}
	else
	{
		/* no overflow between reads of cntl, hence the
		 * values between the reads are correct */
		cntl = cntl2;
		cnth = cnth2;
	}

	return (((uint32_t) cnth) << 16 | ((uint32_t) cntl));
}




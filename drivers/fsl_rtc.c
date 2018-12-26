/*
 * The Clear BSD License
 * Copyright 2017 NXP
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided
 *  that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS LICENSE.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "fsl_rtc.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/* Component ID definition, used by tools. */
#ifndef FSL_COMPONENT_ID
#define FSL_COMPONENT_ID "platform.drivers.rtc_1"
#endif

#define SECONDS_IN_A_DAY (86400U)
#define SECONDS_IN_A_HOUR (3600U)
#define SECONDS_IN_A_MINUTE (60U)
#define DAYS_IN_A_YEAR (365U)
#define YEAR_RANGE_START (1970U)
#define YEAR_RANGE_END (2099U)
   
/*******************************************************************************
 * Variables
 ******************************************************************************/   
static volatile uint32_t s_CurrentTimeSeconds;
static volatile uint32_t s_AlarmTimeSeconds;
static rtc_alarm_callback_t s_RtcAlarmCallback;

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
/*!
 * @brief Checks whether the date and time passed in is valid
 *
 * @param datetime Pointer to structure where the date and time details are stored
 *
 * @return Returns false if the date & time details are out of range; true if in range
 */
static bool RTC_CheckDatetimeFormat(const rtc_datetime_t *datetime);

/*!
 * @brief Converts time data from datetime to seconds
 *
 * @param datetime Pointer to datetime structure where the date and time details are stored
 *
 * @return The result of the conversion in seconds
 */
static uint32_t RTC_ConvertDatetimeToSeconds(const rtc_datetime_t *datetime);

/*!
 * @brief Converts time data from seconds to a datetime structure
 *
 * @param seconds  Seconds value that needs to be converted to datetime format
 * @param datetime Pointer to the datetime structure where the result of the conversion is stored
 */
static void RTC_ConvertSecondsToDatetime(uint32_t seconds, rtc_datetime_t *datetime);

/*******************************************************************************
 * Code
 ******************************************************************************/
static bool RTC_CheckDatetimeFormat(const rtc_datetime_t *datetime)
{
    assert(datetime);

    /* Table of days in a month for a non leap year. First entry in the table is not used,
     * valid months start from 1
     */
    uint8_t daysPerMonth[] = {0U, 31U, 28U, 31U, 30U, 31U, 30U, 31U, 31U, 30U, 31U, 30U, 31U};

    /* Check year, month, hour, minute, seconds */
    if ((datetime->year < YEAR_RANGE_START) || (datetime->year > YEAR_RANGE_END) || (datetime->month > 12U) ||
        (datetime->month < 1U) || (datetime->hour >= 24U) || (datetime->minute >= 60U) || (datetime->second >= 60U))
    {
        /* If not correct then error*/
        return false;
    }

    /* Adjust the days in February for a leap year */
    if ((((datetime->year & 3U) == 0) && (datetime->year % 100 != 0)) || (datetime->year % 400 == 0))
    {
        daysPerMonth[2] = 29U;
    }

    /* Check the validity of the day */
    if ((datetime->day > daysPerMonth[datetime->month]) || (datetime->day < 1U))
    {
        return false;
    }

    return true;
}

static uint32_t RTC_ConvertDatetimeToSeconds(const rtc_datetime_t *datetime)
{
    assert(datetime);

    /* Number of days from begin of the non Leap-year*/
    /* Number of days from begin of the non Leap-year*/
    uint16_t monthDays[] = {0U, 0U, 31U, 59U, 90U, 120U, 151U, 181U, 212U, 243U, 273U, 304U, 334U};
    uint32_t seconds;

    /* Compute number of days from 1970 till given year*/
    seconds = (datetime->year - 1970U) * DAYS_IN_A_YEAR;
    /* Add leap year days */
    seconds += ((datetime->year / 4) - (1970U / 4));
    /* Add number of days till given month*/
    seconds += monthDays[datetime->month];
    /* Add days in given month. We subtract the current day as it is
     * represented in the hours, minutes and seconds field*/
    seconds += (datetime->day - 1);
    /* For leap year if month less than or equal to Febraury, decrement day counter*/
    if ((!(datetime->year & 3U)) && (datetime->month <= 2U))
    {
        seconds--;
    }

    seconds = (seconds * SECONDS_IN_A_DAY) + (datetime->hour * SECONDS_IN_A_HOUR) +
              (datetime->minute * SECONDS_IN_A_MINUTE) + datetime->second;

    return seconds;
}

static void RTC_ConvertSecondsToDatetime(uint32_t seconds, rtc_datetime_t *datetime)
{
    assert(datetime);

    uint32_t x;
    uint32_t secondsRemaining, days;
    uint16_t daysInYear;
    /* Table of days in a month for a non leap year. First entry in the table is not used,
     * valid months start from 1
     */
    uint8_t daysPerMonth[] = {0U, 31U, 28U, 31U, 30U, 31U, 30U, 31U, 31U, 30U, 31U, 30U, 31U};

    /* Start with the seconds value that is passed in to be converted to date time format */
    secondsRemaining = seconds;

    /* Calcuate the number of days, we add 1 for the current day which is represented in the
     * hours and seconds field
     */
    days = secondsRemaining / SECONDS_IN_A_DAY + 1;

    /* Update seconds left*/
    secondsRemaining = secondsRemaining % SECONDS_IN_A_DAY;

    /* Calculate the datetime hour, minute and second fields */
    datetime->hour = secondsRemaining / SECONDS_IN_A_HOUR;
    secondsRemaining = secondsRemaining % SECONDS_IN_A_HOUR;
    datetime->minute = secondsRemaining / 60U;
    datetime->second = secondsRemaining % SECONDS_IN_A_MINUTE;

    /* Calculate year */
    daysInYear = DAYS_IN_A_YEAR;
    datetime->year = YEAR_RANGE_START;
    while (days > daysInYear)
    {
        /* Decrease day count by a year and increment year by 1 */
        days -= daysInYear;
        datetime->year++;

        /* Adjust the number of days for a leap year */
        if (datetime->year & 3U)
        {
            daysInYear = DAYS_IN_A_YEAR;
        }
        else
        {
            daysInYear = DAYS_IN_A_YEAR + 1;
        }
    }

    /* Adjust the days in February for a leap year */
    if (!(datetime->year & 3U))
    {
        daysPerMonth[2] = 29U;
    }

    for (x = 1U; x <= 12U; x++)
    {
        if (days <= daysPerMonth[x])
        {
            datetime->month = x;
            break;
        }
        else
        {
            days -= daysPerMonth[x];
        }
    }

    datetime->day = days;
}

void RTC_Init(RTC_Type *base, const rtc_config_t *config)
{
    assert(config);
    
    uint32_t srcClock_Hz = 0;
#if defined(RTC_CLOCKS)
#if !(defined(FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL) && FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL)
    CLOCK_EnableClock(kCLOCK_Rtc0);
#endif /* FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL */
#endif /* RTC_CLOCKS */
    
    /* Clear Real-Time Interrupt Flag to 0 */
    base->SC |= RTC_SC_RTIF_MASK;
    
    /* Select Real-Time Clock Source and Clock Prescaler */
    base->SC &= ~(RTC_SC_RTCLKS_MASK | RTC_SC_RTCPS_MASK);
    base->SC |=  RTC_SC_RTCLKS(config->clockSource) | RTC_SC_RTCPS(config->prescaler);
    
    /* Get RTC clock frequency */
    if(config->clockSource == kRTC_ExternalClock)
    {
      srcClock_Hz = CLOCK_GetFreq(kCLOCK_Osc0ErClk);
    }
        
    if(config->clockSource == kRTC_LPOCLK)
    {
       srcClock_Hz = CLOCK_GetFreq(kCLOCK_LpoClk);
    } 
    
    if(config->clockSource == kRTC_ICSIRCLK)
    {
       srcClock_Hz = CLOCK_GetFreq(kCLOCK_ICSInternalRefClk);
    } 
    
    if(config->clockSource == kRTC_BusClock)
    {
       srcClock_Hz = CLOCK_GetFreq(kCLOCK_BusClk);
    }
        
    /* Set RTC module value */
    RTC_SetModuloValue(base, USEC_TO_COUNT(config->time_us, (srcClock_Hz/ RTC_GetDivideValue(base))));
}

void RTC_Deinit(RTC_Type *base)
{
#if defined(RTC_CLOCKS)
#if !(defined(FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL) && FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL)
    /* Gate the module clock */
    CLOCK_DisableClock(kCLOCK_Rtc0);
#endif /* FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL */
#endif /* RTC_CLOCKS */
}

void RTC_GetDefaultConfig(rtc_config_t *config)
{
    assert(config);

    config->clockSource = kRTC_BusClock;
    
    config->prescaler = kRTC_ClockDivide_16_2048;
   
    /* Configure RTC to interrupt every 1000000us */
    config->time_us = 1000000U;
}

uint32_t RTC_GetDivideValue(RTC_Type *base)
{
    uint32_t clocks, divide, prescale;
    
    /* Get RTCLKS and RTCPS configuration value */
    clocks = (uint32_t)((base->SC & RTC_SC_RTCLKS_MASK) >> RTC_SC_RTCLKS_SHIFT);
    prescale = (uint32_t)((base->SC & RTC_SC_RTCPS_MASK) >> RTC_SC_RTCPS_SHIFT);
    
    assert(prescale > 0U);
    
    if((clocks & 1U) == 0U)
    {
        divide = 1U << (prescale - 1U);
    }
    if((clocks & 1U) == 1U)
    {
       if (prescale < 6U)
       {
           divide = 1U << (prescale + 6U);
       }
       else if(prescale == 6U)
       {
           divide = 100U;
       }
       else 
       {
           divide = 1000U;
       }
    }
    return divide;
}

status_t RTC_SetDatetime(rtc_datetime_t *datetime)
{
    assert(datetime);

    /* Return error if the time provided is not valid */
    if (!(RTC_CheckDatetimeFormat(datetime)))
    {
        return kStatus_InvalidArgument;
    }
     /* Set current time seconds */
    s_CurrentTimeSeconds = RTC_ConvertDatetimeToSeconds(datetime);
    return kStatus_Success;
}

void RTC_GetDatetime(rtc_datetime_t *datetime)
{     
   assert(datetime);
  
   /* Get current data time */
   RTC_ConvertSecondsToDatetime(s_CurrentTimeSeconds, datetime);
}

void RTC_SetAlarm(uint32_t second)
{
    /* Set alarm time seconds */
    s_AlarmTimeSeconds = second + s_CurrentTimeSeconds;
     
}

void RTC_GetAlarm(rtc_datetime_t *datetime)
{
    assert(datetime);
    
    /* Get alarm data time */
    RTC_ConvertSecondsToDatetime(s_AlarmTimeSeconds, datetime);
}

void RTC_SetAlarmCallback(rtc_alarm_callback_t callback)
{
    s_RtcAlarmCallback = callback;
}

void RTC_DriverIRQHandler(void)
{
    if(s_AlarmTimeSeconds == s_CurrentTimeSeconds)
    {
         if (s_RtcAlarmCallback != NULL)
         {
             s_RtcAlarmCallback();
         }
    }
   
    if (RTC_GetInterruptFlags(RTC) & kRTC_InterruptFlag)
    {   
        s_CurrentTimeSeconds ++;
        /* Clear second interrupt flag */
        RTC_ClearInterruptFlags(RTC, kRTC_InterruptFlag);
    } 
}

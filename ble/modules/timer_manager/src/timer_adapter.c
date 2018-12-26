/*
 * The Clear BSD License
 * Copyright (c) 2016, NXP Semiconductors, N.V.
 * All rights reserved.
 *
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
 * o Neither the name of Freescale Semiconductor, Inc. nor the names of its
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
#include "timer_manager.h"
#include "timer_adapter.h"
#include "power_manager.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/
static ta_callback_t ta_callback;
/*******************************************************************************
 * Code
 ******************************************************************************/

/*!
 * @brief Check if ke time has passed.
 *
 * @param[in] time     time value to check
 *
 * @return if time has passed or not
 */
static inline bool TA_IsTimerPast(uint32_t time)
{
    return ((uint32_t)(time - TA_GetTimestamp()) > TM_TIMER_DELAY_MAX);
}

void TA_Init(ta_callback_t callback)
{
    RTC_Init(RTC);
    if (NVIC_GetPendingIRQ(RTC_FR_IRQn))
    {
        NVIC_ClearPendingIRQ(RTC_FR_IRQn);
    }

    ta_callback = callback;

    /* Enable RTC free running interrupt */
    RTC_EnableInterrupts(RTC, kRTC_FreeRunningInterruptEnable);
    /* Enable at the NVIC */
    NVIC_EnableIRQ(RTC_FR_IRQn);
}

uint32_t TA_GetTimestamp(void)
{
    return RTC_GetFreeRunningCount(RTC);
}

void TA_StartTimer(uint32_t time)
{
    RTC_SetFreeRunningInterruptThreshold(RTC, time);
    if (!(RTC->CNT2_CTRL & RTC_CNT2_CTRL_CNT2_EN_MASK))
    {
        RTC_FreeRunningEnable(RTC, true);
    }

    /* Check that the timer did not expire before HW prog */
    if (TA_IsTimerPast(time))
    {
        /* Timer already expired, so set the pending interrupt flag */
        NVIC_SetPendingIRQ(RTC_FR_IRQn);
    }
}

void TA_StopTimer(void)
{
    RTC_FreeRunningEnable(RTC, false);
    NVIC_ClearPendingIRQ(RTC_FR_IRQn);
}

void RTC_FR_IRQHandler(void)
{
    if (ta_callback != NULL)
    {
        ta_callback(RTC_GetFreeRunningInterruptThreshold(RTC));
    }
}

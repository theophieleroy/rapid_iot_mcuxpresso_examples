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

#include "timer_manager.h"
#include "ke_queue.h"

#ifdef NXP_BLE_STACK
#include "TimersManagerInternal.h"
#include "TimersManager.h"
#endif

#ifdef NXP_BLE_STACK

void TMR_Init()
{
    TM_Init();
}

/* To be mapped to real hw timer */
tmrTimerID_t TMR_AllocateTimer(void)
{
    return TM_AllocateTimer();
}

tmrErrCode_t TMR_FreeTimer(tm_timer_id_t timerID)
{
    TM_FreeTimer(timerID);

    return gTmrSuccess_c;
}

tmrErrCode_t TMR_StartTimer(
    tm_timer_id_t timerID, tm_timer_type_t type, uint32_t ms, tm_callback_t callback, void *param)
{
    uint32_t cycles = TA_MILLISECOND(ms);

    return (tmrErrCode_t)TM_SetTimer(timerID, type, cycles, callback, param);
}

tmrErrCode_t TMR_StartSingleShotTimer(tmrTimerID_t timerID,
                                      tmrTimeInMilliseconds_t timeInMilliseconds,
                                      pfTmrCallBack_t callback,
                                      void *param)
{
    return (tmrErrCode_t)TMR_StartTimer(timerID, kTM_SingleShotTimer, timeInMilliseconds, (tm_callback_t)callback,
                                        param);
}

tmrErrCode_t TMR_StopTimer(tmrTimerID_t timerID)
{
    return (tmrErrCode_t)TM_ClearTimer(timerID);
}

tmrErrCode_t TMR_StartIntervalTimer(tmrTimerID_t timerID,
                                    tmrTimeInMilliseconds_t timeInMilliseconds,
                                    pfTmrCallBack_t callback,
                                    void *param)
{
    return (tmrErrCode_t)TMR_StartTimer(timerID, kTM_IntervalTimer, timeInMilliseconds, (tm_callback_t)callback, param);
}

bool_t TMR_IsTimerActive(tmrTimerID_t timerID)
{
    return 1;
}
/* FIX ME using normal timer instead of Low Power Timer*/
tmrErrCode_t TMR_StartLowPowerTimer(
    tm_timer_id_t timerID, tm_timer_type_t timerType, uint32_t timeInMilliseconds, tm_callback_t callback, void *param)
{
    return (tmrErrCode_t)TMR_StartTimer(timerID, timerType, timeInMilliseconds, callback, (void *)param);
}

#endif

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*!
 * @brief timer table. All allocated timers are stored here.
 * A timer's ID is it's index in this table.
 */
static tm_timer_table_entry_t s_tmTimerTable[TM_TOTAL_TIMERS];

/*!
 * @brief timer status stable.
 * If an entry is == NULL, the timer is not allocated.
 */
static tm_timer_status_t s_tmTimerStatusTable[TM_TOTAL_TIMERS];

/*! @brief Timer list. */
static struct co_list s_tmTimerList;

/*******************************************************************************
 * Code
 ******************************************************************************/

/*!
* @brief     Returns the timer entry address
* @param[in] timerID - the timer ID
* @return    the timer entry address
*/
static inline tm_timer_table_entry_t *TM_GetTimerEntryAddress(tm_timer_id_t timerID)
{
    assert(timerID < TM_TOTAL_TIMERS);
    return &s_tmTimerTable[timerID];
}

/*!
* @brief     Set the timer status
* @param[in] timerID - the timer ID
* @param[in] status - the status of the timer
*/
void TM_SetTimerStatus(tm_timer_id_t timerID, tm_timer_status_t status)
{
    assert(timerID < TM_TOTAL_TIMERS);
    s_tmTimerStatusTable[timerID] = status;
}

/*!
* @brief     Returns the timer status
* @param[in] timerID - the timer ID
* @return    see definition of tm_timer_status_t
*/
tm_timer_status_t TM_GetTimerStatus(tm_timer_id_t timerID)
{
    assert(timerID < TM_TOTAL_TIMERS);
    return s_tmTimerStatusTable[timerID];
}

/*!
 * @brief Compare timer ID
 *
 * @param[in] timer           Timer value
 * @param[in] id              Timer ID
 *
 * @return timer insert or not
 ****************************************************************************************
 */
static inline bool TM_CompareTimerId(struct co_list_hdr const *timer, uint32_t id)
{
    /* insert the timer just before the first one older */
    return (id == ((tm_timer_table_entry_t *)timer)->timerID);
}

/*!
 * @brief Compare timer absolute expiration time.
 *
 * @param[in] timerA Timer to compare.
 * @param[in] timerB Timer to compare.
 *
 * @return true if timerA will expire before timerB.
 */
static inline bool TM_CompareTargetTime(struct co_list_hdr const *timerA, struct co_list_hdr const *timerB)
{
    uint32_t targetTimeA = ((tm_timer_table_entry_t *)timerA)->targetCycles;
    uint32_t targetTimeB = ((tm_timer_table_entry_t *)timerB)->targetCycles;
    return ((uint32_t)(targetTimeA - targetTimeB) > TM_TIMER_DELAY_MAX);
}

/*!
 * @brief   Handles the expired timer
 */
static void TM_ExpiredTimerHandler(uint32_t time)
{
    assert(time == ((tm_timer_table_entry_t *)s_tmTimerList.first)->targetCycles);
    tm_timer_table_entry_t *timer = (tm_timer_table_entry_t *)ke_queue_pop(&s_tmTimerList);
    assert(timer != NULL);

    TM_SetTimerStatus(timer->timerID, kTm_statusInactive);

    if (timer->callback != NULL)
    {
        timer->callback(timer->param);
    }

    if (timer->timerType == kTM_IntervalTimer)
    {
        timer->targetCycles = time + timer->intervalInCycles;

        /* Insert in stored timer list */
        ke_queue_insert(&s_tmTimerList, (struct co_list_hdr *)timer, TM_CompareTargetTime);
    }

    /* check the next timer */
    timer = (tm_timer_table_entry_t *)s_tmTimerList.first;
    if (timer == NULL)
    {
        /* no more timers, disable HW irq and leave */
        TA_StopTimer();
    }
    else
    {
        TA_StartTimer(timer->targetCycles);
    }
}

void TM_Init(void)
{
    co_list_init(&s_tmTimerList);
    TA_Init(TM_ExpiredTimerHandler);
}

tm_timer_id_t TM_AllocateTimer(void)
{
    uint8_t i = TM_TOTAL_TIMERS;
    uint32_t primask = DisableGlobalIRQ();
    for (i = 0U; i < TM_TOTAL_TIMERS; i++)
    {
        if (TM_GetTimerStatus(i) == kTm_statusFree)
        {
            TM_SetTimerStatus(i, kTm_statusInactive);
            break;
        }
    }
    EnableGlobalIRQ(primask);
    return i;
}

void TM_FreeTimer(tm_timer_id_t timerID)
{
    assert(timerID < TM_TOTAL_TIMERS);
    uint32_t primask = DisableGlobalIRQ();
    tm_timer_table_entry_t *timer = TM_GetTimerEntryAddress(timerID);
    timer->next = NULL;
    timer->timerID = TM_TOTAL_TIMERS;
    timer->timerType = kTM_SingleShotTimer;
    timer->callback = NULL;
    timer->param = NULL;
    timer->intervalInCycles = 0;
    timer->targetCycles = 0;
    TM_SetTimerStatus(timerID, kTm_statusFree);
    EnableGlobalIRQ(primask);
}

tm_err_code_t TM_SetTimer(
    tm_timer_id_t timerID, tm_timer_type_t type, uint32_t cycles, tm_callback_t callback, void *param)
{
    bool hwProg = false; /* Indicate if the HW will have to be reprogrammed */
    uint32_t primask = DisableGlobalIRQ();

    /* Check if requested timer is first of the list of pending timer */
    tm_timer_table_entry_t *first = (tm_timer_table_entry_t *)s_tmTimerList.first;

    /* Delay shall not be more than maximum allowed */
    if (cycles > TM_TIMER_DELAY_MAX)
    {
        cycles = TM_TIMER_DELAY_MAX;
    }

    if ((first != NULL) && (first->timerID == timerID))
    {
        hwProg = true;
    }

    /* Extract the timer from the list if required */
    tm_timer_table_entry_t *timer =
        (tm_timer_table_entry_t *)ke_queue_extract(&s_tmTimerList, TM_CompareTimerId, timerID);
    if (timer == NULL)
    {
        /* the timer is not in the list */
        timer = TM_GetTimerEntryAddress(timerID);
        timer->timerID = timerID;
    }

    timer->timerType = type;
    timer->callback = callback;
    timer->param = param;
    timer->intervalInCycles = cycles;
    if (type == kTM_AbsoluteTimer)
    {
        timer->targetCycles = cycles;
    }
    else
    {
        timer->targetCycles = TA_GetTimestamp() + cycles;
    }

    /* Insert in stored timer list */
    ke_queue_insert(&s_tmTimerList, (struct co_list_hdr *)timer, TM_CompareTargetTime);

    /* Check if HW timer set needed */
    if (hwProg || (timer == (tm_timer_table_entry_t *)s_tmTimerList.first))
    {
        TA_StartTimer(((tm_timer_table_entry_t *)s_tmTimerList.first)->targetCycles);
    }

    TM_SetTimerStatus(timerID, kTm_statusActive);
    EnableGlobalIRQ(primask);

    return kTM_Success;
}

tm_err_code_t TM_ClearTimer(tm_timer_id_t timerID)
{
    tm_err_code_t err = kTM_Success;
    uint32_t primask = DisableGlobalIRQ();
    tm_timer_table_entry_t *timer = (tm_timer_table_entry_t *)s_tmTimerList.first;

    if (timer == NULL)
    {
        err = kTM_OutOfRange;
    }
    else
    {
        if (timerID == timer->timerID)
        {
            /* Timer found and first to expire, pop it */
            ke_queue_pop(&s_tmTimerList);

            /* check the next timer */
            timer = (tm_timer_table_entry_t *)s_tmTimerList.first;
            if (timer == NULL)
            {
                TA_StopTimer();
            }
            else
            {
                /* set the following timer if have */
                TA_StartTimer(timer->targetCycles);
            }
        }
        else
        {
            timer = (tm_timer_table_entry_t *)ke_queue_extract(&s_tmTimerList, TM_CompareTimerId, timerID);
            if (timer == NULL)
            {
                err = kTM_InvalidId;
            }
        }
    }

    EnableGlobalIRQ(primask);
    return err;
}

tm_err_code_t TM_UpdateInterval(tm_timer_id_t timerID, uint32_t cycles)
{
    tm_err_code_t err = kTM_Success;
    uint32_t primask = DisableGlobalIRQ();
    tm_timer_table_entry_t *timer = TM_GetTimerEntryAddress(timerID);

    if (timer == NULL)
    {
        /* the timer is not in the list */
        err = kTM_OutOfRange;
    }
    else
    {
        timer->intervalInCycles = cycles;
    }

    EnableGlobalIRQ(primask);
    return err;
}

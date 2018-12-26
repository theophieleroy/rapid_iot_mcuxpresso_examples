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

#ifndef _TIMER_MANAGER_H_
#define _TIMER_MANAGER_H_

#include "fsl_common.h"
#include "timer_adapter.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*! @brief Total number of timers (user define) */
#ifdef MBED_PORT
#define TM_TOTAL_TIMERS (12)
#else
#define TM_TOTAL_TIMERS (10)
#endif

/*! @brief Maximum timer value */
#define TM_TIMER_DELAY_MAX (0x7FFFFFFF)

/*!
 * @brief   Error code
 */
typedef enum _tm_err_code
{
    kTM_Success,
    kTM_InvalidId,
    kTM_OutOfRange
} tm_err_code_t;

/*!
 * @brief   Timer type
 */
typedef enum _tm_timer_type
{
    kTM_SingleShotTimer,
    kTM_IntervalTimer,
    kTM_AbsoluteTimer
} tm_timer_type_t;

/*!
 * @brief   Timer status
 */
typedef enum _tm_timer_status
{
    kTm_statusFree,
    kTm_statusActive,
    kTm_statusInactive
} tm_timer_status_t;

/*!
 * @brief   Timer ID
 */
typedef uint8_t tm_timer_id_t;

/*!
 * @brief   Timer callback function
 */
typedef void (*tm_callback_t)(void *param);

/*!
 * @brief   One entry in the main timer table.
 */
typedef struct _tm_timer_table_entry
{
    struct _tm_timer_table_entry *next;
    tm_timer_id_t timerID;
    tm_timer_type_t timerType;
    uint32_t intervalInCycles; /*!< The timer's original duration, in cycles. Used to reset intervnal timers.*/
    uint32_t targetCycles;     /*!< When the HW counter reaches this value, the timer has expired.*/
    tm_callback_t callback;    /*!< Pointer to the callback function */
    void *param;               /*!< Parameter to the callback function */
} tm_timer_table_entry_t;

#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus */

/*******************************************************************************
 * API
 ******************************************************************************/

/*!
 * @brief   initialize the timer manager
 */
void TM_Init(void);

/*!
 * @brief   allocate a timer
 *
 * @return  the ID of the timer
 */
tm_timer_id_t TM_AllocateTimer(void);

/*!
 * @brief     Free a timer
 * @param[in] timerID - the ID of the timer
 *
 * @details   Safe to call even if the timer is running.
 */
void TM_FreeTimer(tm_timer_id_t timerID);

/*!
* @brief     Set the timer status
* @param[in] timerID - the timer ID
* @param[in] status - the status of the timer
*/
void TM_SetTimerStatus(tm_timer_id_t timerID, tm_timer_status_t status);

/*!
* @brief     Returns the timer status
* @param[in] timerID - the timer ID
* @return    see definition of tm_timer_status_t
*/
tm_timer_status_t TM_GetTimerStatus(tm_timer_id_t timerID);
/*!
 * @brief     Set a specified timer
 *
 * @param[in] timerID - the ID of the timer
 * @param[in] type - the type of the timer
 * @param[in] cycles - time expressed in clock source cycles
 * @param[in] callback - callback function
 * @param[in] param - parameter to callback function
 *
 * @return    the error code
 * @details   When the timer expires, the callback function is called in
 *            non-interrupt context. If the timer is already running when
 *            this function is called, it will be stopped and restarted.
 */
tm_err_code_t TM_SetTimer(
    tm_timer_id_t timerID, tm_timer_type_t type, uint32_t cycles, tm_callback_t callback, void *param);

/*!
 * @brief     Clear a specified timer
 * @param[in] timerID - the ID of the timer
 *
 * @return   the error code
 * @details  Associated timer callback function is not called, even if the timer
 *           expires. Does not frees the timer. Safe to call anytime, regardless
 *           of the state of the timer.
 */
tm_err_code_t TM_ClearTimer(tm_timer_id_t timerID);

/*!
 * @brief     Update timer interval.
 * @param[in] timerID - the ID of the timer
 *
 * @return   the error code
 * @details  Update interval for the timer with the type TM_IntervalTimer.
 */
tm_err_code_t TM_UpdateInterval(tm_timer_id_t timerID, uint32_t cycles);

#if defined(__cplusplus)
}
#endif /* __cplusplus */

#endif /* _TIMER_MANAGER_H_ */

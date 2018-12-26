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

#ifndef _TIMER_ADAPTER_H_
#define _TIMER_ADAPTER_H_

#include "fsl_common.h"
#include "fsl_clock.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*! @brief Typecast the macro argument into milliseconds */
#define TA_MILLISECOND(n) ((n)*CLOCK_GetFreq(kCLOCK_32KClk) / 1000U)

/*! @brief Typecast the macro argument into seconds */
#define TA_SECOND(n) ((n)*CLOCK_GetFreq(kCLOCK_32KClk))

/*! @brief Timer adapter callback function. */
typedef void (*ta_callback_t)(uint32_t time);

#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus */

/*******************************************************************************
 * API
 ******************************************************************************/

/*!
 * @brief   initialize the timer adapter
 */
void TA_Init(ta_callback_t callback);

/*!
 * @brief   Get a time-stamp value
 */
uint32_t TA_GetTimestamp(void);

/*!
 * @brief   Start a timer
 * @param[in] time           Timer value
 */
void TA_StartTimer(uint32_t time);

/*!
 * @brief   Stop timer
 * @param[in] time           Timer value
 */
void TA_StopTimer(void);

#if defined(__cplusplus)
}
#endif /* __cplusplus */

#endif /* _TIMER_ADAPTER_H_ */

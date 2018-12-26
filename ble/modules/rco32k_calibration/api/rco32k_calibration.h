/*
 * The Clear BSD License
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
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

#ifndef _RCO32K_CALIBRATION_H_
#define _RCO32K_CALIBRATION_H_

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define RCO32K_1ST_EDGE (1U)
#define RCO32K_17TH_EDGE (17U)

/*!
 * @brief RCO32K calibration environment
 *
 * This structure holds the environment for calibrating RCO32K.
 */
typedef struct _rco32k_calib_env
{
    /* application defined */
    CTIMER_Type *ctimer; /*!< stores which ctimer is used for calibrating rco32k */
    uint32_t interval;   /*!< the intervel between 2 calibrations, configured by application */

    /* internal use, application can ignore them */
    int32_t hwCodeFlag; /*!< 1 to increase, -1 to decrease value in CAU_RCO_CAP_CFG */
    uint32_t firstVal;  /*!< captured counter value at 1st 32k edge */
    uint32_t lastVal;   /*!< captured counter value at last 32k edge */
    uint8_t cnt;        /*!< the counter used to record 32k cycle count */
} rco32k_calib_env_t;

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @brief Initialize a timer to start RCO32K software calibration.
 *
 * @param interval Calibration's interval. Unit: 1ms
 */
void RCO32K_InitSwCalib(uint32_t interval);

/*!
 * @brief Update the interval of RCO32K software calibration.
 *
 * The interval can be changed at run-time by calling this function.
 *
 * @param interval Calibration's interval. Unit: 1ms
 */
void RCO32K_UpdateSwCalibInterval(uint32_t interval);

#if defined(__cplusplus)
}
#endif

#endif /* _RCO32K_CALIBRATION_H_ */

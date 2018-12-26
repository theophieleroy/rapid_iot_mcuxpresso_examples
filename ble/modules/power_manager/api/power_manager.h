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

#ifndef _POWER_MANAGER_H_
#define _POWER_MANAGER_H_

#include "fsl_common.h"
#include "fsl_power.h"

/*! @addtogroup power_manager */
/*! @{ */

/*! @file */

/*! @brief bit definition for power mode requsets */
enum
{
    kPmReqUserPolicy = (1U << 0U),   /*!< request from user app policy */
    kPmReqBle = (1U << 1U),          /*!< request from ble */
    kPmReqGpio = (1U << 2U),         /*!< request from GPIO */
    kPmReqUserEvents = (1U << 3U),   /*!< request from user event */
    kPmReqCTIMER0 = (1U << 4U),      /*!< request from CTIMER0 */
    kPmReqCTIMER1 = (1U << 5U),      /*!< request from CTIMER1 */
    kPmReqCTIMER2 = (1U << 6U),      /*!< request from CTIMER2 */
    kPmReqCTIMER3 = (1U << 7U),      /*!< request from CTIMER3 */
    kPmReqRTC = (1U << 8U),          /*!< request from RTC */
    kPmReqTimerManager = (1U << 9U), /*!< request from Timer Manager */
    kPmReqAdc = (1U << 10U),         /*!< request from ADC */
};

/*! @brief Power manager environment */
typedef struct _power_manager_env
{
    uint32_t req[4U]; /*!< hold power mode requests made by PM_SetReq() */
} power_manager_env_t;

/*******************************************************************************
 * Variables
 ******************************************************************************/
extern power_manager_env_t g_pmEnv;

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus */

/*!
 * @brief Request the chip to stay in Active/Sleep/Power Down0 mode.
 */
static inline void PM_SetReq(power_mode_t pm, uint32_t bits)
{
    uint32_t regPrimask = DisableGlobalIRQ();

    g_pmEnv.req[kPmActive] &= ~bits;
    g_pmEnv.req[kPmSleep] &= ~bits;
    g_pmEnv.req[kPmPowerDown0] &= ~bits;
    g_pmEnv.req[pm] |= bits;

    EnableGlobalIRQ(regPrimask);
}

/*!
 * @brief Clear the request for staying in Active/Sleep/Power Down0 mode.
 */
static inline void PM_ClrReq(uint32_t bits)
{
    uint32_t regPrimask = DisableGlobalIRQ();

    g_pmEnv.req[kPmActive] &= ~bits;
    g_pmEnv.req[kPmSleep] &= ~bits;
    g_pmEnv.req[kPmPowerDown0] &= ~bits;

    EnableGlobalIRQ(regPrimask);
}

/*!
 * @brief Power Manager's initialization.
 */
void PM_Init(void);

/*!
 * @brief Power management.
 */
void PM_PowerManagement(void);

#if defined(__ICCARM__)
__weak void APP_WakeupRestore(void);
__weak void BOARD_WakeupRestore(void);
#elif defined(__CC_ARM)
__weak void APP_WakeupRestore(void) __attribute__((used));
__weak void BOARD_WakeupRestore(void) __attribute__((used));
#else /* __GUNC__ */
void APP_WakeupRestore(void) __attribute__((weak));
void BOARD_WakeupRestore(void) __attribute__((weak));
#endif

#if defined(__cplusplus)
}
#endif /* __cplusplus */

/*! @} */

#endif /* _POWER_MANAGER_H_ */

/**
 * @}
 */

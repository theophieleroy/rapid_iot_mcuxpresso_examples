/*
 * The Clear BSD License
 * Copyright (c) 2017, NXP
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

#ifndef __SRTM_RTC_SERVICE_H__
#define __SRTM_RTC_SERVICE_H__

#include "srtm_service.h"

/*!
 * @addtogroup srtm_service
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/** @brief Switch to disable RTC service debugging messages. */
#ifndef SRTM_RTC_SERVICE_DEBUG_OFF
#define SRTM_RTC_SERVICE_DEBUG_OFF (0)
#endif

#if SRTM_RTC_SERVICE_DEBUG_OFF
#undef SRTM_DEBUG_VERBOSE_LEVEL
#define SRTM_DEBUG_VERBOSE_LEVEL SRTM_DEBUG_VERBOSE_NONE
#endif

#define SRTM_RTC_ALARM_DISABLED (0x0U)
#define SRTM_RTC_ALARM_ENABLED (0x1U)

#define SRTM_RTC_ALARM_NOT_PENDING (0x0U)
#define SRTM_RTC_ALARM_PENDING (0x1U)

/*! @brief SRTM RTC datetime structure */
SRTM_PACKED_BEGIN struct _srtm_rtc_alarm
{
    uint32_t seconds; /* Seconds since 01-01-1970 00:00:00 */
    uint8_t enabled;
    uint8_t pending;
} SRTM_PACKED_END;

SRTM_PACKED_BEGIN struct _srtm_rtc_payload
{
    uint8_t reserved;
    uint8_t retCode;
    struct _srtm_rtc_alarm alarm;
} SRTM_PACKED_END;

/**
* @brief SRTM RTC adapter structure pointer.
*/
typedef struct _srtm_rtc_adapter *srtm_rtc_adapter_t;

/**
* @brief SRTM RTC adapter structure
*/
struct _srtm_rtc_adapter
{
    /* Bound service */
    srtm_service_t service;

    /* Interfaces implemented by RTC service. */
    srtm_status_t (*notifyAlarm)(srtm_service_t service);

    /* Interface implemented by RTC adapter */
    srtm_status_t (*getTime)(srtm_rtc_adapter_t adapter, uint32_t *pSeconds);
    srtm_status_t (*setTime)(srtm_rtc_adapter_t adapter, uint32_t seconds);
    srtm_status_t (*getAlarm)(srtm_rtc_adapter_t adapter, struct _srtm_rtc_alarm *pAlarm);
    srtm_status_t (*setAlarm)(srtm_rtc_adapter_t adapter, const struct _srtm_rtc_alarm *pAlarm);
    srtm_status_t (*enableAlarm)(srtm_rtc_adapter_t adapter, bool enable);
};

/*******************************************************************************
 * API
 ******************************************************************************/
#ifdef __cplusplus
extern "C" {
#endif

/*!
 * @brief Create RTC service.
 *
 * @param adapter RTC adapter to handle real rtc operations.
 * @return SRTM service handle on success and NULL on failure.
 */
srtm_service_t SRTM_RtcService_Create(srtm_rtc_adapter_t adapter);

/*!
 * @brief Destroy RTC service.
 *
 * @param service SRTM service to destroy.
 */
void SRTM_RtcService_Destroy(srtm_service_t service);

/*!
 * @brief Reset RTC service. This is used to reset RTC to initial state.
 *
 * @param service SRTM service to reset.
 * @param core Identify which core is to be reset.
 */
void SRTM_RtcService_Reset(srtm_service_t service, srtm_peercore_t core);

#ifdef __cplusplus
}
#endif

/*! @} */

#endif /* __SRTM_RTC_SERVICE_H__ */

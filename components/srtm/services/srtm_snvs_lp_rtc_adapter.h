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
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR ADAPTERS;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __SRTM_SNVS_LP_RTC_ADAPTER_H__
#define __SRTM_SNVS_LP_RTC_ADAPTER_H__

#include "srtm_rtc_service.h"
#include "fsl_snvs_lp.h"

/*!
 * @addtogroup srtm_service
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * API
 ******************************************************************************/
#ifdef __cplusplus
extern "C" {
#endif

/*!
 * @brief Create SNVS_LP RTC adapter.
 *
 * @param base SNVS base address.
 * @return SRTM RTC adapter on success or NULL on failure.
 */
srtm_rtc_adapter_t SRTM_SnvsLpRtcAdapter_Create(SNVS_Type *base);

/*!
 * @brief Destroy SNVS_LP RTC adapter.
 *
 * @param adapter SNVS_LP RTC adapter to destroy.
 */
void SRTM_SnvsLpRtcAdapter_Destroy(srtm_rtc_adapter_t adapter);

/*!
 * @brief Notify alarm event.
 *
 * @param adapter SNVS_LP RTC adapter to destroy.
 */
void SRTM_SnvsLpRtcAdapter_NotifyAlarm(srtm_rtc_adapter_t adapter);

#ifdef __cplusplus
}
#endif

/*! @} */

#endif /* __SRTM_SNVS_LP_RTC_ADAPTER_H__ */

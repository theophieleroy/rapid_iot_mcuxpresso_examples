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

#ifndef __SRTM_RPMSG_ENDPOINT_H__
#define __SRTM_RPMSG_ENDPOINT_H__

#include "srtm_channel.h"
#include "rpmsg_lite.h"

/*!
 * @addtogroup srtm_channel
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/**
* @brief SRTM RPMsg endpoint channel configuration fields
*/
typedef struct _srtm_rpmsg_endpoint_config
{
    struct rpmsg_lite_instance *rpmsgHandle; /*!< RPMsg handle initialized by app */
    unsigned long localAddr; /*!< RPMsg local endpoint address */
    unsigned long peerAddr; /*!< RPMsg peer endpoint address */
    const char *epName; /*!< RPMsg endpoint name for name service announcement */
} srtm_rpmsg_endpoint_config_t;

/**
* @brief SRTM RPMsg endpoint channel RX callback function type.
*/
typedef int (*srtm_rpmsg_endpoint_rx_cb_t)(srtm_channel_t channel, void *payload, int payloadLen,
                                           unsigned long src, void *param);

/*******************************************************************************
 * API
 ******************************************************************************/
#ifdef __cplusplus
extern "C" {
#endif

/*!
 * @brief Create RPMsg endpoint channel.
 *
 * @param config SRTM RPMsg endpoint configuration.
 * @return SRTM channel handle on success and NULL on failure.
 */
srtm_channel_t SRTM_RPMsgEndpoint_Create(srtm_rpmsg_endpoint_config_t *config);

/*!
 * @brief Destroy RPMsg endpoint channel.
 *
 * @param channel SRTM channel to destroy.
 */
void SRTM_RPMsgEndpoint_Destroy(srtm_channel_t channel);

/*!
 * @brief Override RPMsg endpoint channel RX handler.
 *
 * By default, the RX messages are posted to dispatcher for SRTM
 * request/response/notification handling. This function is to change the default
 * behavior for functionality extension.
 *
 * @param channel SRTM channel to override RX handler.
 * @param callback User function to handle RX message.
 * @param param User parameter to be used in callback.
 * @return SRTM_Status_Success on success and others on failure.
 */
srtm_status_t SRTM_RPMsgEndpoint_OverrideRxHandler(srtm_channel_t channel,
                                                   srtm_rpmsg_endpoint_rx_cb_t callback,
                                                   void *param);

#ifdef __cplusplus
}
#endif

/*! @} */

#endif /* __SRTM_RPMSG_ENDPOINT_H__ */

/*
 * The Clear BSD License
 * Copyright (c) 2018, NXP
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

#ifndef __SRTM_RPMSG_EMU_H__
#define __SRTM_RPMSG_EMU_H__

#include "srtm_rpmsg_endpoint.h"

/*!
 * @addtogroup srtm_channel
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/**
* @brief SRTM RPMsg endpoint channel create/destroy callback function type.
*/
typedef void (*srtm_rpmsg_endpoint_hook_t)(srtm_channel_t channel, srtm_rpmsg_endpoint_config_t *config, void *param);

/**
* @brief SRTM RPMsg endpoint channel TX callback function type.
*/
typedef void (*srtm_rpmsg_endpoint_tx_cb_t)(srtm_channel_t channel, void *payload, int payloadLen,
                                            void *param);

/*******************************************************************************
 * API
 ******************************************************************************/
#ifdef __cplusplus
extern "C" {
#endif

/*!
 * @brief Set callback to be called when channel created.
 *
 * @param callback User function to be called when channel created.
 * @param param User parameter to be used in callback.
 */
void SRTM_RPMsgEndpoint_SetCreateHook(srtm_rpmsg_endpoint_hook_t callback, void *param);

/*!
 * @brief Set callback to be called when channel destroyed.
 *
 * @param callback User function to be called when channel destroyed.
 * @param param User parameter to be used in callback.
 */
void SRTM_RPMsgEndpoint_SetDestroyHook(srtm_rpmsg_endpoint_hook_t callback, void *param);

/*!
 * @brief Emulate received data from channel.
 *
 * @param channel SRTM channel received data from.
 * @param data start address of received data.
 * @param len data length in bytes.
 * @return RL_RELEASE on data buffer used or RL_HOLD on data buffer to be processed.
 */
int SRTM_RPMsgEndpoint_RecvData(srtm_channel_t channel, void *data, uint32_t len);

/*!
 * @brief Override RPMsg endpoint channel TX handler.
 *
 * This function is to provide callback for unit test application.
 *
 * @param channel SRTM channel to override TX handler.
 * @param callback User function to handle TX message.
 * @param param User parameter to be used in callback.
 * @return SRTM_Status_Success on success and others on failure.
 */
srtm_status_t SRTM_RPMsgEndpoint_OverrideTxHandler(srtm_channel_t channel,
                                                   srtm_rpmsg_endpoint_tx_cb_t callback,
                                                   void *param);

#ifdef __cplusplus
}
#endif

/*! @} */

#endif /* __SRTM_RPMSG_EMU_H__ */

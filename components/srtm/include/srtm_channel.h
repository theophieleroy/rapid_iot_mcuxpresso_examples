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

#ifndef __SRTM_CHANNEL_H__
#define __SRTM_CHANNEL_H__

#include "srtm_defs.h"

/*!
 * @addtogroup srtm
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
 * @brief Destroy SRTM channel.
 *
 * @param channel SRTM channel to start.
 */
void SRTM_Channel_Destroy(srtm_channel_t channel);

/*!
 * @brief Let SRTM channel start working.
 *
 * @param channel SRTM channel to start.
 * @return SRTM_Status_Success on success and others on failure.
 */
srtm_status_t SRTM_Channel_Start(srtm_channel_t channel);

/*!
 * @brief Let SRTM channel stop working.
 *
 * @param channel SRTM channel to stop.
 * @return SRTM_Status_Success on success and others on failure.
 */
srtm_status_t SRTM_Channel_Stop(srtm_channel_t channel);

/*!
 * @brief Send data via SRTM channel.
 *
 * @param channel SRTM channel to send data.
 * @param data start address of data to send.
 * @param len data length in bytes.
 * @return SRTM_Status_Success on success and others on failure.
 */
srtm_status_t SRTM_Channel_SendData(srtm_channel_t channel, void *data, uint32_t len);

#ifdef __cplusplus
}
#endif

/*! @} */

#endif /* __SRTM_CHANNEL_H__ */

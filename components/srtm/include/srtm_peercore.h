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

#ifndef __SRTM_PEERCORE_H__
#define __SRTM_PEERCORE_H__

#include "srtm_defs.h"

/*!
 * @addtogroup srtm
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/**
* @brief SRTM peer core state
*/
typedef enum _srtm_peercore_state
{
    SRTM_PeerCore_State_Inactive = 0x00U,  /*!< Peer core is not ready to communicate */
    SRTM_PeerCore_State_Activating,        /*!< Peer core wakeup in progress */
    SRTM_PeerCore_State_Activated,         /*!< Peer core is ready to communicate */
    SRTM_PeerCore_State_Deactivating,      /*!< Peer core is going to suspend */
    SRTM_PeerCore_State_Deactivated,       /*!< Peer core suspended and not ready to communicate */
} srtm_peercore_state_t;

/**
* @brief SRTM peer core wakeup callback function
*/
typedef srtm_status_t (*srtm_peercore_wakeup_cb_t)(srtm_peercore_t core, void *param);

/*******************************************************************************
 * API
 ******************************************************************************/
#ifdef __cplusplus
extern "C" {
#endif

/*!
 * @brief Create SRTM peer core object.
 *
 * @param id SRTM peer core ID allocated by application.
 * @return SRTM peer core handle, or NULL on failure.
 */
srtm_peercore_t SRTM_PeerCore_Create(uint32_t id);

/*!
 * @brief Destroy SRTM peer core object.
 *
 * @param core SRTM peer core handle.
 */
void SRTM_PeerCore_Destroy(srtm_peercore_t core);

/*!
 * @brief Get SRTM peer core ID.
 *
 * @param core SRTM peer core handle.
 * @return SRTM peer core ID.
 */
uint32_t SRTM_PeerCore_GetID(srtm_peercore_t core);

/*!
 * @brief Start SRTM peer core communication.
 *
 * @param core SRTM peer core handle.
 * @return SRTM_Status_Success on success and others on failure.
 */
srtm_status_t SRTM_PeerCore_Start(srtm_peercore_t core);

/*!
 * @brief Stop SRTM peer core communication.
 *
 * @param core SRTM peer core handle.
 * @return SRTM_Status_Success on success and others on failure.
 */
srtm_status_t SRTM_PeerCore_Stop(srtm_peercore_t core);

/*!
 * @brief Activate the SRTM peer core.
 *
 * @param core SRTM peer core handle.
 * @return SRTM_Status_Success on success and others on failure.
 */
srtm_status_t SRTM_PeerCore_Activate(srtm_peercore_t core);

/*!
 * @brief Deactivate the SRTM peer core.
 *
 * @param core SRTM peer core handle.
 * @param wakeup SRTM peer core wakeup function.
 * @param param SRTM peer core wakeup parameter.
 * @return SRTM_Status_Success on success and others on failure.
 */
srtm_status_t SRTM_PeerCore_Deactivate(srtm_peercore_t core, srtm_peercore_wakeup_cb_t wakeup,
                                       void *param);

/*!
 * @brief Add communication channel to the SRTM peer core.
 *
 * @param core SRTM peer core handle.
 * @param channel SRTM channel to add.
 * @return SRTM_Status_Success on success and others on failure.
 */
srtm_status_t SRTM_PeerCore_AddChannel(srtm_peercore_t core, srtm_channel_t channel);

/*!
 * @brief Remove communication channel from the SRTM peer core.
 *
 * @param core SRTM peer core handle.
 * @param channel SRTM channel to remove.
 * @return SRTM_Status_Success on success and others on failure.
 */
srtm_status_t SRTM_PeerCore_RemoveChannel(srtm_peercore_t core, srtm_channel_t channel);

/*!
 * @brief Get SRTM peer core state.
 *
 * @param core SRTM peer core handle.
 * @return SRTM peer core state.
 */
srtm_peercore_state_t SRTM_PeerCore_GetState(srtm_peercore_t core);

/*!
 * @brief Set SRTM peer core state.
 *
 * @param core SRTM peer core handle.
 * @param state SRTM peer core state to run into.
 * @return SRTM_Status_Success on success and others on failure.
 */
srtm_status_t SRTM_PeerCore_SetState(srtm_peercore_t core, srtm_peercore_state_t state);

#ifdef __cplusplus
}
#endif

/*! @} */

#endif /* __SRTM_PEERCORE_H__ */

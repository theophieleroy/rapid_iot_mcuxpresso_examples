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

#ifndef __SRTM_PEERCORE_STRUCT_H__
#define __SRTM_PEERCORE_STRUCT_H__

#include "srtm_defs.h"
#include "srtm_list.h"
#include "srtm_mutex.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/**
* @brief SRTM peer core struct
*/
struct _srtm_peercore
{
    uint32_t id;          /*!< SRTM peer core ID */
    srtm_list_t node;     /*!< SRTM peer core list node to link to a list */
    srtm_list_t channels; /*!< SRTM channel list to maintain channels added to the peer core */
    srtm_list_t pendingQ; /*!< Pending messages queue */

    srtm_dispatcher_t dispatcher;
    srtm_mutex_t mutex;

    bool started;
    srtm_peercore_state_t state;
    srtm_peercore_wakeup_cb_t wakeupFunc;
    void *wakeupParam;
};

/*******************************************************************************
 * API
 ******************************************************************************/

/*! @} */

#endif /* __SRTM_PEERCORE_STRUCT_H__ */

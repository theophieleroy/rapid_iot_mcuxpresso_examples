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

#ifndef __SRTM_CAN_SERVICE_H__
#define __SRTM_CAN_SERVICE_H__

#include "srtm_service.h"

/*!
 * @addtogroup srtm_service
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/** @brief Switch to disable Can service debugging messages. */
#ifndef SRTM_CAN_SERVICE_DEBUG_OFF
#define SRTM_CAN_SERVICE_DEBUG_OFF (0)
#endif

#if SRTM_CAN_SERVICE_DEBUG_OFF
#undef SRTM_DEBUG_VERBOSE_LEVEL
#define SRTM_DEBUG_VERBOSE_LEVEL SRTM_DEBUG_VERBOSE_NONE
#endif

typedef enum
{
    SRTM_CanEventReverse = 0U,
    SRTM_CanEventDrive = 1U
} srtm_can_event_t;

/**
* @brief SRTM Can adapter structure pointer.
*/
typedef struct _srtm_can_adapter *srtm_can_adapter_t;

/**
* @brief SRTM Can adapter structure
*/
struct _srtm_can_adapter
{
    /* Bound service */
    srtm_service_t service;

    /* Interfaces implemented by Can adapter. */
    srtm_status_t (*registerEvent)(srtm_can_adapter_t adapter, uint32_t index, uint32_t userParam);
    srtm_status_t (*unregisterEvent)(srtm_can_adapter_t adapter, uint32_t index, uint32_t userParam);
};

/**
* @brief SRTM Can payload structure
*/
SRTM_ANON_DEC_BEGIN
SRTM_PACKED_BEGIN struct _srtm_can_payload
{
    uint8_t index;
    uint8_t retCode;
    uint32_t event;
    uint32_t userParam;
} SRTM_PACKED_END;
SRTM_ANON_DEC_END

/*******************************************************************************
 * API
 ******************************************************************************/
#ifdef __cplusplus
extern "C" {
#endif

/*!
 * @brief Create can service.
 *
 * @param can can adapter to provide real can features.
 * @return SRTM service handle on success and NULL on failure.
 */
srtm_service_t SRTM_CanService_Create(srtm_can_adapter_t can);

/*!
 * @brief Destroy can service.
 *
 * @param service SRTM service to destroy.
 */
void SRTM_CanService_Destroy(srtm_service_t service);

/*!
 * @brief Reset can service. This is used to stop sending events and return to initial state.
 *
 * @param service SRTM service to reset.
 * @param core Identify which core is to be reset.
 */
void SRTM_CanService_Reset(srtm_service_t service, srtm_peercore_t core);

/*!
 * @brief Send CAN event to all registered clients, and wait for response until time out.
 *  Note: the function cannot be called in ISR or SRTM dispatcher task.
 *
 * @param service SRTM CAN service.
 * @param index CAN instance the event corresponds to.
 * @param event CAN event to be sent.
 * @param timeout Maximum milliseconds to wait for each client, must bigger than 0. 
 * @return SRTM_Status_Success on success and others on failure.
 */
srtm_status_t SRTM_CanService_SendEvent(srtm_service_t service, uint32_t index, srtm_can_event_t event,
                                        uint32_t timeout);

#ifdef __cplusplus
}
#endif

/*! @} */

#endif /* __SRTM_CAN_SERVICE_H__ */

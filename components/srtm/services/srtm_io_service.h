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

#ifndef __SRTM_IO_SERVICE_H__
#define __SRTM_IO_SERVICE_H__

#include "srtm_service.h"

/*!
 * @addtogroup srtm_service
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/** @brief Switch to disable IO service debugging messages. */
#ifndef SRTM_IO_SERVICE_DEBUG_OFF
#define SRTM_IO_SERVICE_DEBUG_OFF (0)
#endif

#if SRTM_IO_SERVICE_DEBUG_OFF
#undef SRTM_DEBUG_VERBOSE_LEVEL
#define SRTM_DEBUG_VERBOSE_LEVEL SRTM_DEBUG_VERBOSE_NONE
#endif

/*! @brief SRTM IO service output value */
typedef enum _srtm_io_value
{
    SRTM_IoValueLow = 0U,
    SRTM_IoValueHigh,
} srtm_io_value_t;

/*! @brief SRTM IO service input event */
typedef enum _srtm_io_event
{
    SRTM_IoEventNone = 0U, /* Ignore the event */
    SRTM_IoEventRisingEdge,
    SRTM_IoEventFallingEdge,
    SRTM_IoEventEitherEdge,
    SRTM_IoEventLowLevel,
    SRTM_IoEventHighLevel,
} srtm_io_event_t;

/**
* @brief SRTM IO service set output value function type.
*/
typedef srtm_status_t (*srtm_io_service_set_output_t)(srtm_service_t service, srtm_peercore_t core,
                                                      uint16_t ioId, srtm_io_value_t ioValue);

/**
* @brief SRTM IO service get input value function type.
*/
typedef srtm_status_t (*srtm_io_service_get_input_t)(srtm_service_t service, srtm_peercore_t core,
                                                     uint16_t ioId, srtm_io_value_t *pIoValue);


/**
* @brief SRTM IO service configure input event function type.
*/
typedef srtm_status_t (*srtm_io_service_conf_input_t)(srtm_service_t service, srtm_peercore_t core,
                                                      uint16_t ioId, srtm_io_event_t event, bool wakeup);

/*******************************************************************************
 * API
 ******************************************************************************/
#ifdef __cplusplus
extern "C" {
#endif

/*!
 * @brief Create IO service.
 *
 * @return SRTM service handle on success and NULL on failure.
 */
srtm_service_t SRTM_IoService_Create(void);

/*!
 * @brief Destroy IO service.
 *
 * @param service SRTM service to destroy.
 */
void SRTM_IoService_Destroy(srtm_service_t service);

/*!
 * @brief Reset IO service.
 *  This is used to stop all IO operations and return to initial state for corresponding core.
 *  Registered pins are kept unchanged.
 *
 * @param service SRTM service to reset.
 * @param core Identify which core is to be reset
 */
void SRTM_IoService_Reset(srtm_service_t service, srtm_peercore_t core);

/*!
 * @brief Register IO service pin. Only registered pin will be serviced.
 *
 * @param service SRTM IO service handle.
 * @param ioId IO pin identification.
 * @param setOutput IO pin set output value callback.
 * @param getInput IO pin get input value callback.
 * @param confIEvent IO pin configure input event callback.
 * @param param user callback parameter.
 * @return SRTM_Status_Success on success and others on failure.
 */
srtm_status_t SRTM_IoService_RegisterPin(srtm_service_t service, uint16_t ioId, srtm_io_service_set_output_t setOutput,
                                         srtm_io_service_get_input_t getInput, srtm_io_service_conf_input_t confIEvent,
                                         void *param);

/*!
 * @brief Unregister IO service pin. The operation cannot work when service is running.
 *
 * @param service SRTM IO service handle.
 * @param ioId IO pin identification.
 * @return SRTM_Status_Success on success and others on failure.
 */
srtm_status_t SRTM_IoService_UnregisterPin(srtm_service_t service, uint16_t ioId);

/*!
 * @brief Notify Input event to peer core. This function must be called by application after peer core configured
 *  input event.
 *
 * @param service SRTM IO service.
 * @param ioId IO pin identification.
 * @return SRTM_Status_Success on success and others on failure.
 */
srtm_status_t SRTM_IoService_NotifyInputEvent(srtm_service_t service, uint16_t ioId);

#ifdef __cplusplus
}
#endif

/*! @} */

#endif /* __SRTM_IO_SERVICE_H__ */

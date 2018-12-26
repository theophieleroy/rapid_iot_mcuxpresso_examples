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

#ifndef __SRTM_KEYPAD_SERVICE_H__
#define __SRTM_KEYPAD_SERVICE_H__

#include "srtm_service.h"

/*!
 * @addtogroup srtm_service
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/** @brief Switch to disable keypad service debugging messages. */
#ifndef SRTM_KEYPAD_SERVICE_DEBUG_OFF
#define SRTM_KEYPAD_SERVICE_DEBUG_OFF (0)
#endif

#if SRTM_KEYPAD_SERVICE_DEBUG_OFF
#undef SRTM_DEBUG_VERBOSE_LEVEL
#define SRTM_DEBUG_VERBOSE_LEVEL SRTM_DEBUG_VERBOSE_NONE
#endif

/*! @brief SRTM keypad value */
typedef enum _srtm_keypad_value
{
    SRTM_KeypadValueReleased = 0U,
    SRTM_KeypadValuePressed,
} srtm_keypad_value_t;

/*! @brief SRTM keypad service input event */
typedef enum _srtm_keypad_event
{
    SRTM_KeypadEventNone = 0U, /* Ignore the event */
    SRTM_KeypadEventPress,
    SRTM_KeypadEventRelease,
    SRTM_KeypadEventPressOrRelease,
} srtm_keypad_event_t;

/**
* @brief SRTM keypad service configure keypad event function type.
*/
typedef srtm_status_t (*srtm_keypad_service_conf_t)(srtm_service_t service, srtm_peercore_t core,
                                                    uint8_t keyIdx, srtm_keypad_event_t event, bool wakeup);

/*******************************************************************************
 * API
 ******************************************************************************/
#ifdef __cplusplus
extern "C" {
#endif

/*!
 * @brief Create keypad service.
 *
 * @return SRTM service handle on success and NULL on failure.
 */
srtm_service_t SRTM_KeypadService_Create(void);

/*!
 * @brief Destroy keypad service.
 *
 * @param service SRTM service to destroy.
 */
void SRTM_KeypadService_Destroy(srtm_service_t service);

/*!
 * @brief Reset keypad service.
 *  This is used to stop all keypad operations and return to initial state for corresponding core.
 *  Registered keys are kept unchanged.
 *
 * @param service SRTM service to reset.
 * @param core Identify which core is to be reset
 */
void SRTM_KeypadService_Reset(srtm_service_t service, srtm_peercore_t core);

/*!
 * @brief Register keypad service key. Only registered key will be serviced.
 *
 * @param service SRTM keypad service handle.
 * @param keyIdx Keypad key index.
 * @param confKEvent Keypad configure event callback.
 * @param param user callback parameter.
 * @return SRTM_Status_Success on success and others on failure.
 */
srtm_status_t SRTM_KeypadService_RegisterKey(srtm_service_t service, uint8_t keyIdx,
                                             srtm_keypad_service_conf_t confKEvent, void *param);

/*!
 * @brief Unregister keypad service pin. The operation cannot work when service is running.
 *
 * @param service SRTM keypad service handle.
 * @param keyIdx Keypad key index.
 * @return SRTM_Status_Success on success and others on failure.
 */
srtm_status_t SRTM_KeypadService_UnregisterPin(srtm_service_t service, uint8_t keyIdx);

/*!
 * @brief Notify keypad event to peer core. This function must be called by application after peer core configured
 *  keypad event.
 *
 * @param service SRTM KEYPAD service.
 * @param keyIdx Keypad index.
 * @param value Keypad event value.
 * @return SRTM_Status_Success on success and others on failure.
 */
srtm_status_t SRTM_KeypadService_NotifyKeypadEvent(srtm_service_t service, uint8_t keyIdx, srtm_keypad_value_t value);

#ifdef __cplusplus
}
#endif

/*! @} */

#endif /* __SRTM_KEYPAD_SERVICE_H__ */

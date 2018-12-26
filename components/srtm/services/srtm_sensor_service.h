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

#ifndef __SRTM_SENSOR_SERVICE_H__
#define __SRTM_SENSOR_SERVICE_H__

#include "srtm_service.h"

/*!
 * @addtogroup srtm_service
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/** @brief Switch to disable Sensor service debugging messages. */
#ifndef SRTM_SENSOR_SERVICE_DEBUG_OFF
#define SRTM_SENSOR_SERVICE_DEBUG_OFF (0)
#endif

#if SRTM_SENSOR_SERVICE_DEBUG_OFF
#undef SRTM_DEBUG_VERBOSE_LEVEL
#define SRTM_DEBUG_VERBOSE_LEVEL SRTM_DEBUG_VERBOSE_NONE
#endif

typedef enum
{
    SRTM_SensorTypePedometer = 0
} srtm_sensor_type_t;

/**
* @brief SRTM Sensor adapter structure pointer.
*/
typedef struct _srtm_sensor_adapter *srtm_sensor_adapter_t;

/**
* @brief SRTM Sensor adapter structure
*/
struct _srtm_sensor_adapter
{
    /* Bound service */
    srtm_service_t service;

    /* Interfaces implemented by Sensor service. */
    srtm_status_t (*updateState)(srtm_service_t service, srtm_sensor_type_t type, uint8_t index);
    srtm_status_t (*reportData)(srtm_service_t service, srtm_sensor_type_t type, uint8_t index,
                                uint8_t *data, uint32_t dataLen);

    /* Interfaces implemented by Sensor adapter. */
    srtm_status_t (*enableStateDetector)(srtm_sensor_adapter_t adapter, srtm_sensor_type_t type,
                                         uint8_t index, bool enable);
    srtm_status_t (*enableDataReport)(srtm_sensor_adapter_t adapter, srtm_sensor_type_t type,
                                      uint8_t index, bool enable);
    srtm_status_t (*setPollDelay)(srtm_sensor_adapter_t adapter, srtm_sensor_type_t type,
                                  uint8_t index, uint32_t millisec);
};

/**
* @brief SRTM Sensor payload structure
*/
SRTM_ANON_DEC_BEGIN
SRTM_PACKED_BEGIN struct _srtm_sensor_payload
{
    uint8_t type;
    uint8_t index;
    union {
        uint8_t enable;
        uint8_t retCode;
        uint32_t pollDelay;
        uint32_t data;
    };
} SRTM_PACKED_END;
SRTM_ANON_DEC_END

/*******************************************************************************
 * API
 ******************************************************************************/
#ifdef __cplusplus
extern "C" {
#endif

/*!
 * @brief Create sensor service.
 *
 * @param sensor sensor adapter to provide real sensor features.
 * @return SRTM service handle on success and NULL on failure.
 */
srtm_service_t SRTM_SensorService_Create(srtm_sensor_adapter_t sensor);

/*!
 * @brief Destroy sensor service.
 *
 * @param service SRTM service to destroy.
 */
void SRTM_SensorService_Destroy(srtm_service_t service);

/*!
 * @brief Reset sensor service. This is used to stop all sensor operations and return to initial state.
 *
 * @param service SRTM service to reset.
 * @param core Identify which core is to be reset.
 */
void SRTM_SensorService_Reset(srtm_service_t service, srtm_peercore_t core);

#ifdef __cplusplus
}
#endif

/*! @} */

#endif /* __SRTM_SENSOR_SERVICE_H__ */

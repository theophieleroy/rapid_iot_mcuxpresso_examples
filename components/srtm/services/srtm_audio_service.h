/*
 * The Clear BSD License
 * Copyright (c) 2017-2018, NXP
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

#ifndef __SRTM_AUDIO_SERVICE_H__
#define __SRTM_AUDIO_SERVICE_H__

#include "srtm_service.h"

/*!
 * @addtogroup srtm_service
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/** @brief Switch to disable Audio service debugging messages. */
#ifndef SRTM_AUDIO_SERVICE_DEBUG_OFF
#define SRTM_AUDIO_SERVICE_DEBUG_OFF (0)
#endif

#if SRTM_AUDIO_SERVICE_DEBUG_OFF
#undef SRTM_DEBUG_VERBOSE_LEVEL
#define SRTM_DEBUG_VERBOSE_LEVEL SRTM_DEBUG_VERBOSE_NONE
#endif

/* The preallocated prcedure messages for use in ISR. */
#ifndef SRTM_AUDIO_SERVICE_CONFIG_PROC_NUMBER
#define SRTM_AUDIO_SERVICE_CONFIG_PROC_NUMBER (4U)
#endif

typedef enum
{
    SRTM_AudioDirRx = 0,
    SRTM_AudioDirTx = 1,
} srtm_audio_dir_t;

typedef enum
{
    SRTM_AudioStateClosed = 0,
    SRTM_AudioStateOpened,
    SRTM_AudioStateStarted,
    SRTM_AudioStatePaused,
} srtm_audio_state_t;

/**
* @brief SRTM SAI adapter structure pointer.
*/
typedef struct _srtm_sai_adapter *srtm_sai_adapter_t;

/**
* @brief SRTM Audio Codec adapter structure pointer.
*/
typedef struct _srtm_codec_adapter *srtm_codec_adapter_t;

/**
* @brief SRTM SAI adapter structure
*/
struct _srtm_sai_adapter
{
    /* Bound service */
    srtm_service_t service;

    /* Interfaces implemented by Audio service. */
    srtm_status_t (*periodDone)(srtm_service_t service, srtm_audio_dir_t dir, uint8_t index, uint32_t periodIdx);

    /* Interfaces implemented by SAI adapter. */
    srtm_status_t (*open)(srtm_sai_adapter_t adapter, srtm_audio_dir_t dir, uint8_t index);
    srtm_status_t (*start)(srtm_sai_adapter_t adapter, srtm_audio_dir_t dir, uint8_t index);
    srtm_status_t (*pause)(srtm_sai_adapter_t adapter, srtm_audio_dir_t dir, uint8_t index);
    srtm_status_t (*restart)(srtm_sai_adapter_t adapter, srtm_audio_dir_t dir, uint8_t index);
    srtm_status_t (*stop)(srtm_sai_adapter_t adapter, srtm_audio_dir_t dir, uint8_t index);
    srtm_status_t (*close)(srtm_sai_adapter_t adapter, srtm_audio_dir_t dir, uint8_t index);
    srtm_status_t (*setParam)(srtm_sai_adapter_t adapter,
                              srtm_audio_dir_t dir,
                              uint8_t index,
                              uint8_t format,
                              uint8_t channels,
                              uint32_t srate);
    srtm_status_t (*setBuf)(srtm_sai_adapter_t adapter,
                            srtm_audio_dir_t dir,
                            uint8_t index,
                            uint8_t *bufAddr,
                            uint32_t bufSize,
                            uint32_t periodSize,
                            uint32_t periodIdx);
    srtm_status_t (*suspend)(srtm_sai_adapter_t adapter, srtm_audio_dir_t dir, uint8_t index);
    srtm_status_t (*resume)(srtm_sai_adapter_t adapter, srtm_audio_dir_t dir, uint8_t index);
    srtm_status_t (*getBufOffset)(srtm_sai_adapter_t adapter, srtm_audio_dir_t dir, uint8_t index, uint32_t *pOffset);
    srtm_status_t (*periodReady)(srtm_sai_adapter_t adapter, srtm_audio_dir_t dir, uint8_t index, uint32_t periodIdx);
};

/**
* @brief SRTM Audio Codec adapter structure
*/
struct _srtm_codec_adapter
{
    /* Interfaces implemented by Audio Codec adapter. */
    srtm_status_t (*setParam)(srtm_codec_adapter_t adapter, uint8_t index, uint8_t format, uint32_t srate);
    srtm_status_t (*setReg)(srtm_codec_adapter_t adapter, uint32_t reg, uint32_t regVal);
    srtm_status_t (*getReg)(srtm_codec_adapter_t adapter, uint32_t reg, uint32_t *pRegVal);
};

/**
* @brief SRTM Audio payload structure
*/
SRTM_ANON_DEC_BEGIN
SRTM_PACKED_BEGIN struct _srtm_audio_payload
{
    uint8_t index;
    union
    {
        uint8_t format;
        uint8_t retCode;
    };
    uint8_t channels;
    union
    {
        uint32_t bufOffset;
        uint32_t srate;
    };
    union
    {
        uint32_t bufAddr;
        uint32_t reg;
    };
    union
    {
        uint32_t bufSize;
        uint32_t regVal;
    };
    uint32_t periodSize;
    uint32_t periodIdx;
} SRTM_PACKED_END;
SRTM_ANON_DEC_END

/*******************************************************************************
 * API
 ******************************************************************************/
#ifdef __cplusplus
extern "C" {
#endif

/*!
 * @brief Create audio service.
 *
 * @param sai digital audio driver adapter.
 * @param codec analog audio codec adapter.
 * @return SRTM service handle on success and NULL on failure.
 */
srtm_service_t SRTM_AudioService_Create(srtm_sai_adapter_t sai, srtm_codec_adapter_t codec);

/*!
 * @brief Destroy audio service.
 *
 * @param service SRTM service to destroy.
 */
void SRTM_AudioService_Destroy(srtm_service_t service);

/*!
 * @brief Reset audio service. This is used to stop all audio operations and return to initial state
 *  for corresponding core.
 *
 * @param service SRTM service to reset.
 * @param core Identify which core is to be reset
 */
void SRTM_AudioService_Reset(srtm_service_t service, srtm_peercore_t core);

#ifdef __cplusplus
}
#endif

/*! @} */

#endif /* __SRTM_AUDIO_SERVICE_H__ */

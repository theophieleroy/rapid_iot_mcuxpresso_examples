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
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR ADAPTERS;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __SRTM_WM8524_ADAPTER_H__
#define __SRTM_WM8524_ADAPTER_H__

#include "srtm_audio_service.h"
#include "fsl_wm8524.h"

/*!
 * @addtogroup srtm_service
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/
typedef struct _srtm_wm8524_config
{
    wm8524_config_t config;
} srtm_wm8524_config_t;

/*******************************************************************************
 * API
 ******************************************************************************/
#ifdef __cplusplus
extern "C" {
#endif

/*!
 * @brief Create WM8524 adapter.
 *
 * @param driver WM8524 driver handle.
 * @param config WM8524 driver configuration.
 * @return SRTM WM8524 adapter on success or NULL on failure.
 */
srtm_codec_adapter_t SRTM_Wm8524Adapter_Create(wm8524_handle_t *driver, srtm_wm8524_config_t *config);

/*!
 * @brief Destroy WM8524 adapter.
 *
 * @param adapter WM8524 adapter to destroy.
 */
void SRTM_Wm8524Adapter_Destroy(srtm_codec_adapter_t adapter);

#ifdef __cplusplus
}
#endif

/*! @} */

#endif /* __SRTM_WM8524_ADAPTER_H__ */

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

#include <string.h>

#include "srtm_wm8524_adapter.h"
#include "srtm_heap.h"

#include "fsl_wm8524.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/* WM8524 adapter */
typedef struct _srtm_wm8524_adapter
{
    struct _srtm_codec_adapter adapter;
    srtm_wm8524_config_t config;
    wm8524_handle_t *driver;
} * srtm_wm8524_adapter_t;

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/
/* Currently only 1 audio instance is adequate, so index is just ignored */
static srtm_status_t SRTM_CodecAdapter_SetParam(srtm_codec_adapter_t adapter,
                                                uint8_t index,
                                                uint8_t format,
                                                uint32_t srate)
{
    return SRTM_Status_Success;
}
static srtm_status_t SRTM_CodecAdapter_SetReg(srtm_codec_adapter_t adapter, uint32_t reg, uint32_t regVal)
{
    return SRTM_Status_Success;
}

static srtm_status_t SRTM_CodecAdapter_GetReg(srtm_codec_adapter_t adapter, uint32_t reg, uint32_t *pRegVal)
{
    return SRTM_Status_Success;
}
srtm_codec_adapter_t SRTM_Wm8524Adapter_Create(wm8524_handle_t *driver, srtm_wm8524_config_t *config)
{
    srtm_wm8524_adapter_t handle;

    assert(driver && config);

    SRTM_DEBUG_MESSAGE(SRTM_DEBUG_VERBOSE_INFO, "%s\r\n", __func__);

    handle = (srtm_wm8524_adapter_t)SRTM_Heap_Malloc(sizeof(struct _srtm_wm8524_adapter));
    assert(handle);

    handle->driver = driver;
    memcpy(&handle->config, config, sizeof(struct _srtm_wm8524_config));

    /* Adapter interfaces. */
    handle->adapter.setParam = SRTM_CodecAdapter_SetParam;
    handle->adapter.setReg = SRTM_CodecAdapter_SetReg;
    handle->adapter.getReg = SRTM_CodecAdapter_GetReg;

    return &handle->adapter;
}

void SRTM_Wm8524Adapter_Destroy(srtm_codec_adapter_t adapter)
{
    srtm_wm8524_adapter_t handle = (srtm_wm8524_adapter_t)adapter;

    assert(adapter);

    SRTM_DEBUG_MESSAGE(SRTM_DEBUG_VERBOSE_INFO, "%s\r\n", __func__);

    SRTM_Heap_Free(handle);
}

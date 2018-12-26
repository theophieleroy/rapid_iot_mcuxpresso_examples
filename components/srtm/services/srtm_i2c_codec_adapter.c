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

#include <string.h>

#include "srtm_i2c_codec_adapter.h"
#include "srtm_heap.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/* Codec adapter */
typedef struct _srtm_i2c_codec_adapter
{
    struct _srtm_codec_adapter adapter;
    srtm_i2c_codec_config_t config;
    codec_handle_t *driver;
    uint32_t srate;
    uint8_t format;
} *srtm_i2c_codec_adapter_t;

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/
static const uint8_t codecFormatMap[] = {16U, 24U};

/*******************************************************************************
 * Code
 ******************************************************************************/
/* Currently only 1 audio instance is adequate, so index is just ignored */
static srtm_status_t SRTM_I2CCodecAdapter_SetParam(srtm_codec_adapter_t adapter, uint8_t index,
                                                   uint8_t format, uint32_t srate)
{
    srtm_i2c_codec_adapter_t handle = (srtm_i2c_codec_adapter_t)adapter;

    SRTM_DEBUG_MESSAGE(SRTM_DEBUG_VERBOSE_INFO, "%s: %d. fmt %d, srate %d\r\n", __func__, index, format, srate);

    if (format >= ARRAY_SIZE(codecFormatMap))
    {
        SRTM_DEBUG_MESSAGE(SRTM_DEBUG_VERBOSE_ERROR, "%s: unsupported format %d!\r\n", __func__, format);
        return SRTM_Status_InvalidParameter;
    }

    if (handle->srate != srate || handle->format != codecFormatMap[format])
    {
        /* Only set codec when configuration changes. */
        CODEC_SetFormat(handle->driver, handle->config.mclk, srate, codecFormatMap[format]);
        handle->srate = srate;
        handle->format = codecFormatMap[format];
    }

    return SRTM_Status_Success;
}

static srtm_status_t SRTM_I2CCodecAdapter_SetReg(srtm_codec_adapter_t adapter, uint32_t reg, uint32_t regVal)
{
    srtm_i2c_codec_adapter_t handle = (srtm_i2c_codec_adapter_t)adapter;
    status_t status;

    SRTM_DEBUG_MESSAGE(SRTM_DEBUG_VERBOSE_INFO, "%s: %d, %d\r\n", __func__, reg, regVal);

    if (handle->config.writeRegMap)
    {
        status = handle->config.writeRegMap(handle->driver, reg, regVal);
    }
    else
    {
        status = CODEC_I2C_WriteReg(handle->driver->slaveAddress, handle->config.addrType, reg, handle->config.regWidth,
                                    regVal, handle->driver->I2C_SendFunc);
    }
    return status == kStatus_Success ? SRTM_Status_Success : SRTM_Status_Error;
}

static srtm_status_t SRTM_I2CCodecAdapter_GetReg(srtm_codec_adapter_t adapter, uint32_t reg, uint32_t *pRegVal)
{
    srtm_i2c_codec_adapter_t handle = (srtm_i2c_codec_adapter_t)adapter;
    status_t status;

    assert(pRegVal);

    SRTM_DEBUG_MESSAGE(SRTM_DEBUG_VERBOSE_INFO, "%s: %d\r\n", __func__, reg);

    *pRegVal = 0; /* Clear high bytes. */
    if (handle->config.readRegMap)
    {
        status = handle->config.readRegMap(handle->driver, reg, pRegVal);
    }
    else
    {
        status = CODEC_I2C_ReadReg(handle->driver->slaveAddress, handle->config.addrType, reg, handle->config.regWidth,
                                   (void *)pRegVal, handle->driver->I2C_ReceiveFunc);
    }
    return status == kStatus_Success ? SRTM_Status_Success : SRTM_Status_Error;
}

srtm_codec_adapter_t SRTM_I2CCodecAdapter_Create(codec_handle_t *driver, srtm_i2c_codec_config_t *config)
{
    srtm_i2c_codec_adapter_t handle;

    assert(driver && config);

    SRTM_DEBUG_MESSAGE(SRTM_DEBUG_VERBOSE_INFO, "%s\r\n", __func__);

    handle = (srtm_i2c_codec_adapter_t)SRTM_Heap_Malloc(sizeof(struct _srtm_i2c_codec_adapter));
    assert(handle);

    handle->driver = driver;
    memcpy(&handle->config, config, sizeof(struct _srtm_i2c_codec_config));
    handle->srate = 0;
    handle->format = 0;

    /* Adapter interfaces. */
    handle->adapter.setParam = SRTM_I2CCodecAdapter_SetParam;
    handle->adapter.setReg = SRTM_I2CCodecAdapter_SetReg;
    handle->adapter.getReg = SRTM_I2CCodecAdapter_GetReg;

    return &handle->adapter;
}

void SRTM_I2CCodecAdapter_Destroy(srtm_codec_adapter_t adapter)
{
    srtm_i2c_codec_adapter_t handle = (srtm_i2c_codec_adapter_t)adapter;

    assert(adapter);

    SRTM_DEBUG_MESSAGE(SRTM_DEBUG_VERBOSE_INFO, "%s\r\n", __func__);

    SRTM_Heap_Free(handle);
}

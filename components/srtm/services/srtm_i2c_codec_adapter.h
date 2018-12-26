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
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR ADAPTERS;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __SRTM_I2C_CODEC_ADAPTER_H__
#define __SRTM_I2C_CODEC_ADAPTER_H__

#include "srtm_audio_service.h"
#include "fsl_codec_common.h"

/*!
 * @addtogroup srtm_service
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/
typedef status_t (*srtm_i2c_read_reg_map_t)(void *handle, uint32_t reg, uint32_t *val);
typedef status_t (*srtm_i2c_write_reg_map_t)(void *handle, uint32_t reg, uint32_t val);

typedef struct _srtm_i2c_codec_config
{
    uint32_t mclk;
    codec_reg_addr_t addrType;
    codec_reg_width_t regWidth;
    srtm_i2c_read_reg_map_t readRegMap;
    srtm_i2c_write_reg_map_t writeRegMap;
} srtm_i2c_codec_config_t;

/*******************************************************************************
 * API
 ******************************************************************************/
#ifdef __cplusplus
extern "C" {
#endif

/*!
 * @brief Create I2C Codec adapter.
 *
 * @param driver Codec driver handle.
 * @param config Codec driver configuration.
 * @return SRTM Codec adapter on success or NULL on failure.
 */
srtm_codec_adapter_t SRTM_I2CCodecAdapter_Create(codec_handle_t *driver, srtm_i2c_codec_config_t *config);

/*!
 * @brief Destroy I2C Codec adapter.
 *
 * @param adapter Codec adapter to destroy.
 */
void SRTM_I2CCodecAdapter_Destroy(srtm_codec_adapter_t adapter);

#ifdef __cplusplus
}
#endif

/*! @} */

#endif /* __SRTM_I2C_CODEC_ADAPTER_H__ */

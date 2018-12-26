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

#ifndef __SRTM_SAI_SDMA_ADAPTER_H__
#define __SRTM_SAI_SDMA_ADAPTER_H__

#include "srtm_audio_service.h"
#include "fsl_sai_sdma.h"
#include "fsl_common.h"
/*!
 * @addtogroup srtm_service
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define SRTM_SAI_CLK_FREQ                                                                 \
    CLOCK_GetPllFreq(kCLOCK_AudioPll1Ctrl) / (CLOCK_GetRootPreDivider(kCLOCK_RootSai3)) / \
        (CLOCK_GetRootPostDivider(kCLOCK_RootSai3))
#define SRTM_SAI_SDMA_MAX_LOCAL_BUF_PERIODS (4)
#define SRTM_SAI_SDMA_MAX_LOCAL_PERIOD_ALIGNMENT (4U)
#define SRTM_SAI_SDMA_MAX_LOCAL_PERIOD_ALIGNMENT_MASK (SRTM_SAI_SDMA_MAX_LOCAL_PERIOD_ALIGNMENT - 1)
typedef struct _srtm_sai_sdma_config
{
    sai_config_t config;
    uint8_t dataLine; /* SAI data line number for transaction */
    uint8_t watermark;
    uint32_t mclk;
    uint32_t bclk;
    uint32_t dmaChannel;
    uint8_t ChannelPriority;
    bool stopOnSuspend;
    uint32_t eventSource;
    uint32_t threshold; /* threshold period number: under which will trigger periodDone notification. */
    sdma_context_data_t txcontext;
    sdma_context_data_t rxcontext;
} srtm_sai_sdma_config_t;

typedef struct _srtm_sai_sdma_local_buf
{
    uint8_t *buf;
    uint32_t bufSize;   /* bytes of the whole local buffer */
    uint32_t periods;   /* periods in local buffer */
    uint32_t threshold; /* Threshold period number: under which will trigger copy from share buf to local buf
                           in playback case. */
} srtm_sai_sdma_local_buf_t;
/*******************************************************************************
 * API
 ******************************************************************************/
#ifdef __cplusplus
extern "C" {
#endif

/*!
 * @brief Create SAI SDMA adapter.
 *
 * @param sai SAI base address.
 * @param dma DMA base address.
 * @param txConfig SAI Tx channel configuration.
 * @param rxConfig SAI Rx channel configuration.
 * @return SRTM SAI SDMA adapter on success or NULL on failure.
 */
srtm_sai_adapter_t SRTM_SaiSdmaAdapter_Create(I2S_Type *sai,
                                              SDMAARM_Type *dma,
                                              srtm_sai_sdma_config_t *txConfig,
                                              srtm_sai_sdma_config_t *rxConfig);

/*!
 * @brief Destroy SAI SDMA adapter.
 *
 * @param adapter SAI SDMA adapter to destroy.
 */
void SRTM_SaiSdmaAdapter_Destroy(srtm_sai_adapter_t adapter);

/*!
* @brief Set local buffer to use in DMA transfer. If local buffer is set, the audio data will be copied
* from shared buffer to local buffer and then transfered to I2S interface. Otherwise the data will be
* transfered from shared buffer to I2S interface directly.
* NOTE: it must be called before service start.
*
* @param adapter SAI SDMA adapter to set.
* @param localBuf Local buffer information to be set to the adapter TX path.
*/
void SRTM_SaiSdmaAdapter_SetTxLocalBuf(srtm_sai_adapter_t adapter, srtm_sai_sdma_local_buf_t *localBuf);
/*******************************************************************************
 * Definitions from other files
 ******************************************************************************/

#ifdef __cplusplus
}
#endif

/*! @} */

#endif /* __SRTM_SAI_SDMA_ADAPTER_H__ */

/*
 * The Clear BSD License
 * Copyright (c) 2018, Freescale Semiconductor, Inc.
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
 * o Neither the name of Freescale Semiconductor, Inc. nor the names of its
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
#ifndef _FSL_PDM_SDMA_H_
#define _FSL_PDM_SDMA_H_

#include "fsl_pdm.h"
#include "fsl_sdma.h"

/*!
 * @addtogroup pdm_sdma
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*! @name Driver version */
/*@{*/
#define FSL_PDM_SDMA_DRIVER_VERSION (MAKE_VERSION(2, 0, 0)) /*!< Version 2.0.0 */
/*@}*/

typedef struct _pdm_sdma_handle pdm_sdma_handle_t;

/*! @brief PDM eDMA transfer callback function for finish and error */
typedef void (*pdm_sdma_callback_t)(PDM_Type *base, pdm_sdma_handle_t *handle, status_t status, void *userData);

/*! @brief PDM DMA transfer handle, users should not touch the content of the handle.*/
struct _pdm_sdma_handle
{
    sdma_handle_t *dmaHandle;     /*!< DMA handler for PDM send */
    uint8_t nbytes;               /*!< eDMA minor byte transfer count initially configured. */
    uint8_t fifoWidth;            /*!< fifo width */
    uint8_t endChannel;           /*!< The last enabled channel */
    uint8_t channelNums;          /*!< total channel numbers */
    uint8_t count;                /*!< The transfer data count in a DMA request */
    uint32_t state;               /*!< Internal state for PDM eDMA transfer */
    uint32_t eventSource;         /*!< PDM event source number */
    pdm_sdma_callback_t callback; /*!< Callback for users while transfer finish or error occurs */
    void *userData;               /*!< User callback parameter */
    sdma_buffer_descriptor_t bdPool[PDM_XFER_QUEUE_SIZE]; /*!< BD pool for SDMA transfer. */
    pdm_transfer_t pdmQueue[PDM_XFER_QUEUE_SIZE];         /*!< Transfer queue storing queued transfer. */
    size_t transferSize[PDM_XFER_QUEUE_SIZE];             /*!< Data bytes need to transfer */
    volatile uint8_t queueUser;                           /*!< Index for user to queue transfer. */
    volatile uint8_t queueDriver;                         /*!< Index for driver to get the transfer data and size */
};

/*******************************************************************************
 * APIs
 ******************************************************************************/
#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @name eDMA Transactional
 * @{
 */

/*!
 * @brief Initializes the PDM eDMA handle.
 *
 * This function initializes the PDM DMA handle, which can be used for other PDM master transactional APIs.
 * Usually, for a specified PDM instance, call this API once to get the initialized handle.
 *
 * @param base PDM base pointer.
 * @param handle PDM eDMA handle pointer.
 * @param base PDM peripheral base address.
 * @param callback Pointer to user callback function.
 * @param userData User parameter passed to the callback function.
 * @param dmaHandle eDMA handle pointer, this handle shall be static allocated by users.
 * @param dma request source.
 */
void PDM_TransferCreateHandleSDMA(PDM_Type *base,
                                  pdm_sdma_handle_t *handle,
                                  pdm_sdma_callback_t callback,
                                  void *userData,
                                  sdma_handle_t *dmaHandle,
                                  uint32_t eventSource);

/*!
 * @brief Performs a non-blocking PDM receive using eDMA.
 *
 * @note This interface returns immediately after the transfer initiates. Call
 * the PDM_GetReceiveRemainingBytes to poll the transfer status and check whether the PDM transfer is finished.
 *
 * @param base PDM base pointer
 * @param handle PDM eDMA handle pointer.
 * @param xfer Pointer to DMA transfer structure.
 * @retval kStatus_Success Start a PDM eDMA receive successfully.
 * @retval kStatus_InvalidArgument The input argument is invalid.
 * @retval kStatus_RxBusy PDM is busy receiving data.
 */
status_t PDM_TransferReceiveSDMA(PDM_Type *base, pdm_sdma_handle_t *handle, pdm_transfer_t *xfer);

/*!
 * @brief Aborts a PDM receive using eDMA.
 *
 * @param base PDM base pointer
 * @param handle PDM eDMA handle pointer.
 */
void PDM_TransferAbortReceiveSDMA(PDM_Type *base, pdm_sdma_handle_t *handle);

/*!
 * @brief PDM channel configurations.
 *
 * @param base PDM base pointer.
 * @param handle PDM eDMA handle pointer.
 * @param channel channel number.
 * @param config channel configurations.
 */
void PDM_SetChannelConfigSDMA(PDM_Type *base,
                              pdm_sdma_handle_t *handle,
                              uint32_t channel,
                              const pdm_channel_config_t *config);

/*! @} */

#if defined(__cplusplus)
}
#endif

/*!
 * @}
 */
#endif

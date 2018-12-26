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

#include "fsl_pdm_sdma.h"

/*******************************************************************************
 * Definitations
 ******************************************************************************/

/*<! Structure definition for uart_sdma_private_handle_t. The structure is private. */
typedef struct _pdm_sdma_private_handle
{
    PDM_Type *base;
    pdm_sdma_handle_t *handle;
} pdm_sdma_private_handle_t;

/* Base pointer array */
static PDM_Type *const s_pdmBases[] = PDM_BASE_PTRS;

/*<! Private handle only used for internally. */
static pdm_sdma_private_handle_t s_sdmaPrivateHandle[ARRAY_SIZE(s_pdmBases)];

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
/*!
 * @brief Get the instance number for PDM.
 *
 * @param base PDM base pointer.
 */
extern uint32_t PDM_GetInstance(PDM_Type *base);

/*!
 * @brief PDM SDMA callback for send.
 *
 * @param handle pointer to pdm_sdma_handle_t structure which stores the transfer state.
 * @param userData Parameter for user callback.
 * @param done If the DMA transfer finished.
 * @param tcds The TCD index.
 */
static void PDM_SDMACallback(sdma_handle_t *handle, void *userData, bool done, uint32_t tcds);
/*******************************************************************************
* Code
******************************************************************************/
static void PDM_SDMACallback(sdma_handle_t *handle, void *userData, bool done, uint32_t tcds)
{
    pdm_sdma_private_handle_t *privHandle = (pdm_sdma_private_handle_t *)userData;
    pdm_sdma_handle_t *pdmHandle = privHandle->handle;

    /* If finished a blcok, call the callback function */
    memset(&pdmHandle->pdmQueue[pdmHandle->queueDriver], 0, sizeof(pdm_transfer_t));
    pdmHandle->queueDriver = (pdmHandle->queueDriver + 1) % PDM_XFER_QUEUE_SIZE;
    if (pdmHandle->callback)
    {
        (pdmHandle->callback)(privHandle->base, pdmHandle, kStatus_PDM_Idle, pdmHandle->userData);
    }

    /* If all data finished, just stop the transfer */
    if (pdmHandle->pdmQueue[pdmHandle->queueDriver].data == NULL)
    {
        PDM_TransferAbortReceiveSDMA(privHandle->base, pdmHandle);
    }
}

void PDM_TransferCreateHandleSDMA(PDM_Type *base,
                                  pdm_sdma_handle_t *handle,
                                  pdm_sdma_callback_t callback,
                                  void *userData,
                                  sdma_handle_t *dmaHandle,
                                  uint32_t eventSource)
{
    assert(handle && dmaHandle);

    uint32_t instance = PDM_GetInstance(base);

    /* Zero the handle */
    memset(handle, 0, sizeof(*handle));

    /* Set pdm base to handle */
    handle->dmaHandle = dmaHandle;
    handle->callback = callback;
    handle->userData = userData;
    handle->eventSource = eventSource;
    handle->fifoWidth = FSL_FEATURE_PDM_FIFO_WIDTH;

    /* Set PDM state to idle */
    handle->state = kStatus_PDM_Idle;

    s_sdmaPrivateHandle[instance].base = base;
    s_sdmaPrivateHandle[instance].handle = handle;

    /* Need to use scatter gather */
    SDMA_InstallBDMemory(dmaHandle, handle->bdPool, PDM_XFER_QUEUE_SIZE);

    /* Install callback for Tx dma channel */
    SDMA_SetCallback(dmaHandle, PDM_SDMACallback, &s_sdmaPrivateHandle[instance]);
}

void PDM_SetChannelConfigSDMA(PDM_Type *base,
                              pdm_sdma_handle_t *handle,
                              uint32_t channel,
                              const pdm_channel_config_t *config)
{
    assert(NULL != config);

    /* channel configurations */
    PDM_SetChannelConfig(base, channel, config);

    /* record end channel number */
    handle->endChannel = channel;
    /* increase totoal enabled channel number */
    handle->channelNums++;
    /* increase count pre channel numbers */
    handle->count = handle->channelNums * (base->FIFO_CTRL & PDM_FIFO_CTRL_FIFOWMK_MASK);
}

status_t PDM_TransferReceiveSDMA(PDM_Type *base, pdm_sdma_handle_t *handle, pdm_transfer_t *xfer)
{
    assert(handle && xfer);

    sdma_transfer_config_t config = {0};
    uint32_t startAddr = PDM_GetDataRegisterAddress(base, handle->endChannel - (handle->channelNums - 1U));
    sdma_peripheral_t perType = kSDMA_PeripheralMultiFifoPDM;

    /* Check if input parameter invalid */
    if ((xfer->data == NULL) || (xfer->dataSize == 0U))
    {
        return kStatus_InvalidArgument;
    }

    if (handle->pdmQueue[handle->queueUser].data)
    {
        return kStatus_PDM_QueueFull;
    }

    /* Update queue state  */
    handle->transferSize[handle->queueUser] = xfer->dataSize;
    handle->pdmQueue[handle->queueUser].data = xfer->data;
    handle->pdmQueue[handle->queueUser].dataSize = xfer->dataSize;

    /* Prepare sdma configure */
    SDMA_PrepareTransfer(&config, startAddr, (uint32_t)xfer->data, handle->fifoWidth, handle->fifoWidth,
                         handle->count * handle->fifoWidth, xfer->dataSize, handle->eventSource, perType,
                         kSDMA_PeripheralToMemory);

    /* multi fifo configurations */
    SDMA_SetMultiFifoConfig(&config, handle->channelNums, FSL_FEATURE_PDM_FIFO_OFFSET / sizeof(uint32_t) - 1U);
    /* enable sw done for PDM */
    SDMA_EnableSwDone(base, &config, 0, perType);

    if (handle->queueUser == PDM_XFER_QUEUE_SIZE - 1U)
    {
        SDMA_ConfigBufferDescriptor(&handle->bdPool[handle->queueUser], startAddr, (uint32_t)xfer->data,
                                    config.destTransferSize, xfer->dataSize, true, true, true,
                                    kSDMA_PeripheralToMemory);
    }
    else
    {
        SDMA_ConfigBufferDescriptor(&handle->bdPool[handle->queueUser], startAddr, (uint32_t)xfer->data,
                                    config.destTransferSize, xfer->dataSize, true, true, false,
                                    kSDMA_PeripheralToMemory);
    }

    handle->queueUser = (handle->queueUser + 1) % PDM_XFER_QUEUE_SIZE;

    if (handle->state != kStatus_PDM_Busy)
    {
        SDMA_SubmitTransfer(handle->dmaHandle, &config);

        /* Start DMA transfer */
        SDMA_StartTransfer(handle->dmaHandle);
    }

    handle->state = kStatus_PDM_Busy;

    /* Enable DMA enable bit */
    PDM_EnableDMA(base, true);
    /* enable PDM */
    PDM_Enable(base, true);

    return kStatus_Success;
}

void PDM_TransferAbortReceiveSDMA(PDM_Type *base, pdm_sdma_handle_t *handle)
{
    assert(handle);

    /* Disable dma */
    SDMA_AbortTransfer(handle->dmaHandle);

    /* Disable DMA enable bit */
    PDM_EnableDMA(base, false);

    /* Set the handle state */
    handle->state = kStatus_PDM_Idle;
}

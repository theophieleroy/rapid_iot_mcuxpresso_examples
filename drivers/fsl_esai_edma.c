/*
 * The Clear BSD License
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
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

#include "fsl_esai_edma.h"

/* Component ID definition, used by tools. */
#ifndef FSL_COMPONENT_ID
#define FSL_COMPONENT_ID "platform.drivers.esai_edma"
#endif


/*******************************************************************************
 * Definitations
 ******************************************************************************/
/* Used for 32byte aligned */
#define STCD_ADDR(address) (edma_tcd_t *)(((uint32_t)address + 32) & ~0x1FU)

/*<! Structure definition for uart_edma_private_handle_t. The structure is private. */
typedef struct _esai_edma_private_handle
{
    ESAI_Type *base;
    esai_edma_handle_t *handle;
} esai_edma_private_handle_t;

enum _esai_edma_transfer_state
{
    kESAI_Busy = 0x0U, /*!< ESAI is busy */
    kESAI_Idle,        /*!< Transfer is done. */
};

/*<! Private handle only used for internally. */
static esai_edma_private_handle_t s_edmaPrivateHandle[FSL_FEATURE_SOC_ESAI_COUNT][2];

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
/*!
 * @brief Get the instance number for ESAI.
 *
 * @param base ESAI base pointer.
 */
extern uint32_t ESAI_GetInstance(ESAI_Type *base);

extern void ESAI_AnalysisSlot(esai_slot_format_t slotFormat, uint8_t *slotLen, uint8_t *dataLen);

/*!
 * @brief ESAI EDMA callback for send.
 *
 * @param handle pointer to esai_edma_handle_t structure which stores the transfer state.
 * @param userData Parameter for user callback.
 * @param done If the DMA transfer finished.
 * @param tcds The TCD index.
 */
static void ESAI_TxEDMACallback(edma_handle_t *handle, void *userData, bool done, uint32_t tcds);

/*!
 * @brief ESAI EDMA callback for receive.
 *
 * @param handle pointer to esai_edma_handle_t structure which stores the transfer state.
 * @param userData Parameter for user callback.
 * @param done If the DMA transfer finished.
 * @param tcds The TCD index.
 */
static void ESAI_RxEDMACallback(edma_handle_t *handle, void *userData, bool done, uint32_t tcds);

/*******************************************************************************
* Code
******************************************************************************/
static void ESAI_TxEDMACallback(edma_handle_t *handle, void *userData, bool done, uint32_t tcds)
{
    esai_edma_private_handle_t *privHandle = (esai_edma_private_handle_t *)userData;
    esai_edma_handle_t *esaiHandle = privHandle->handle;

    /* If finished a blcok, call the callback function */
    memset(&esaiHandle->esaiQueue[esaiHandle->queueDriver], 0, sizeof(esai_transfer_t));
    esaiHandle->queueDriver = (esaiHandle->queueDriver + 1) % ESAI_XFER_QUEUE_SIZE;
    if (esaiHandle->callback)
    {
        (esaiHandle->callback)(privHandle->base, esaiHandle, kStatus_ESAI_TxIdle, esaiHandle->userData);
    }

    /* If all data finished, just stop the transfer */
    if (esaiHandle->esaiQueue[esaiHandle->queueDriver].data == NULL)
    {
        ESAI_TransferAbortSendEDMA(privHandle->base, esaiHandle);
    }
}

static void ESAI_RxEDMACallback(edma_handle_t *handle, void *userData, bool done, uint32_t tcds)
{
    esai_edma_private_handle_t *privHandle = (esai_edma_private_handle_t *)userData;
    esai_edma_handle_t *esaiHandle = privHandle->handle;

    /* If finished a blcok, call the callback function */
    memset(&esaiHandle->esaiQueue[esaiHandle->queueDriver], 0, sizeof(esai_transfer_t));
    esaiHandle->queueDriver = (esaiHandle->queueDriver + 1) % ESAI_XFER_QUEUE_SIZE;
    if (esaiHandle->callback)
    {
        (esaiHandle->callback)(privHandle->base, esaiHandle, kStatus_ESAI_RxIdle, esaiHandle->userData);
    }

    /* If all data finished, just stop the transfer */
    if (esaiHandle->esaiQueue[esaiHandle->queueDriver].data == NULL)
    {
        ESAI_TransferAbortReceiveEDMA(privHandle->base, esaiHandle);
    }
}

void ESAI_TransferTxCreateHandleEDMA(ESAI_Type *base,
                                     esai_edma_handle_t *handle,
                                     esai_edma_callback_t callback,
                                     void *userData,
                                     edma_handle_t *dmaHandle)
{
    assert(handle && dmaHandle);

    uint32_t instance = ESAI_GetInstance(base);

    /* Zero the handle */
    memset(handle, 0, sizeof(*handle));

    /* Set esai base to handle */
    handle->dmaHandle = dmaHandle;
    handle->callback = callback;
    handle->userData = userData;

    /* Set ESAI state to idle */
    handle->state = kESAI_Idle;

    s_edmaPrivateHandle[instance][0].base = base;
    s_edmaPrivateHandle[instance][0].handle = handle;

    /* Need to use scatter gather */
    EDMA_InstallTCDMemory(dmaHandle, STCD_ADDR(handle->tcd), ESAI_XFER_QUEUE_SIZE);

    /* Install callback for Tx dma channel */
    EDMA_SetCallback(dmaHandle, ESAI_TxEDMACallback, &s_edmaPrivateHandle[instance][0]);
}

void ESAI_TransferRxCreateHandleEDMA(ESAI_Type *base,
                                     esai_edma_handle_t *handle,
                                     esai_edma_callback_t callback,
                                     void *userData,
                                     edma_handle_t *dmaHandle)
{
    assert(handle && dmaHandle);

    uint32_t instance = ESAI_GetInstance(base);

    /* Zero the handle */
    memset(handle, 0, sizeof(*handle));

    /* Set esai base to handle */
    handle->dmaHandle = dmaHandle;
    handle->callback = callback;
    handle->userData = userData;

    /* Set ESAI state to idle */
    handle->state = kESAI_Idle;

    s_edmaPrivateHandle[instance][1].base = base;
    s_edmaPrivateHandle[instance][1].handle = handle;

    /* Need to use scatter gather */
    EDMA_InstallTCDMemory(dmaHandle, STCD_ADDR(handle->tcd), ESAI_XFER_QUEUE_SIZE);

    /* Install callback for Tx dma channel */
    EDMA_SetCallback(dmaHandle, ESAI_RxEDMACallback, &s_edmaPrivateHandle[instance][1]);
}

void ESAI_TransferTxSetFormatEDMA(
    ESAI_Type *base, esai_edma_handle_t *handle, esai_format_t *format, uint32_t hckClockHz, uint32_t hclkSourceClockHz)
{
    assert(handle && format);

    /* Configure the audio format to ESAI registers */
    ESAI_TxSetFormat(base, format, hckClockHz, hclkSourceClockHz);

    /* Get the tranfer size from format, this should be used in EDMA configuration */
    ESAI_AnalysisSlot(format->slotType, &handle->slotLen, &handle->bitWidth);
    handle->sectionMap = format->sectionMap;

    /* Update the data channel ESAI used */
    handle->count = FSL_FEATURE_ESAI_FIFO_SIZEn(base) - ((base->TFCR & ESAI_TFCR_TFWM_MASK) >> ESAI_TFCR_TFWM_SHIFT);
}

void ESAI_TransferRxSetFormatEDMA(
    ESAI_Type *base, esai_edma_handle_t *handle, esai_format_t *format, uint32_t hckClockHz, uint32_t hclkSourceClockHz)
{
    assert(handle && format);

    /* Configure the audio format to ESAI registers */
    ESAI_RxSetFormat(base, format, hckClockHz, hclkSourceClockHz);

    /* Get the tranfer size from format, this should be used in EDMA configuration */
    ESAI_AnalysisSlot(format->slotType, &handle->slotLen, &handle->bitWidth);
    handle->sectionMap = format->sectionMap;

    /* Update the data channel ESAI used */
    handle->count = ((base->TFCR & ESAI_TFCR_TFWM_MASK) >> ESAI_TFCR_TFWM_SHIFT);
}

status_t ESAI_TransferSendEDMA(ESAI_Type *base, esai_edma_handle_t *handle, esai_transfer_t *xfer)
{
    assert(handle && xfer);

    edma_transfer_config_t config = {0};
    uint32_t destAddr = ESAI_TxGetDataRegisterAddress(base);

    /* Check if input parameter invalid */
    if ((xfer->data == NULL) || (xfer->dataSize == 0U))
    {
        return kStatus_InvalidArgument;
    }

    if (handle->esaiQueue[handle->queueUser].data)
    {
        return kStatus_ESAI_QueueFull;
    }

    /* Change the state of handle */
    handle->state = kESAI_Busy;

    /* Update the queue state */
    handle->transferSize[handle->queueUser] = xfer->dataSize;
    handle->esaiQueue[handle->queueUser].data = xfer->data;
    handle->esaiQueue[handle->queueUser].dataSize = xfer->dataSize;
    handle->queueUser = (handle->queueUser + 1) % ESAI_XFER_QUEUE_SIZE;

    /* Prepare edma configure */
    EDMA_PrepareTransfer(&config, xfer->data, handle->bitWidth / 8U, (void *)destAddr, handle->bitWidth / 8U,
                         handle->count * handle->bitWidth / 8U, xfer->dataSize, kEDMA_MemoryToPeripheral);

    /* Store the initially configured eDMA minor byte transfer count into the ESAI handle */
    handle->nbytes = handle->count * handle->bitWidth / 8U;

    EDMA_SubmitTransfer(handle->dmaHandle, &config);

    /* Start DMA transfer */
    EDMA_StartTransfer(handle->dmaHandle);

    /* Enable ESAI Tx clock */
    ESAI_TxEnable(base, handle->sectionMap);

    return kStatus_Success;
}

status_t ESAI_TransferReceiveEDMA(ESAI_Type *base, esai_edma_handle_t *handle, esai_transfer_t *xfer)
{
    assert(handle && xfer);

    edma_transfer_config_t config = {0};
    uint32_t srcAddr = ESAI_RxGetDataRegisterAddress(base);

    /* Check if input parameter invalid */
    if ((xfer->data == NULL) || (xfer->dataSize == 0U))
    {
        return kStatus_InvalidArgument;
    }

    if (handle->esaiQueue[handle->queueUser].data)
    {
        return kStatus_ESAI_QueueFull;
    }

    /* Change the state of handle */
    handle->state = kESAI_Busy;

    /* Update queue state  */
    handle->transferSize[handle->queueUser] = xfer->dataSize;
    handle->esaiQueue[handle->queueUser].data = xfer->data;
    handle->esaiQueue[handle->queueUser].dataSize = xfer->dataSize;
    handle->queueUser = (handle->queueUser + 1) % ESAI_XFER_QUEUE_SIZE;

    /* Prepare edma configure */
    EDMA_PrepareTransfer(&config, (void *)srcAddr, handle->bitWidth / 8U, xfer->data, handle->bitWidth / 8U,
                         handle->count * handle->bitWidth / 8U, xfer->dataSize, kEDMA_PeripheralToMemory);

    /* Store the initially configured eDMA minor byte transfer count into the ESAI handle */
    handle->nbytes = handle->count * handle->bitWidth / 8U;

    EDMA_SubmitTransfer(handle->dmaHandle, &config);

    /* Start DMA transfer */
    EDMA_StartTransfer(handle->dmaHandle);

    /* Enable ESAI Rx clock */
    ESAI_RxEnable(base, handle->sectionMap);

    return kStatus_Success;
}

void ESAI_TransferAbortSendEDMA(ESAI_Type *base, esai_edma_handle_t *handle)
{
    assert(handle);

    /* Disable dma */
    EDMA_AbortTransfer(handle->dmaHandle);

    /* Disable Tx */
    ESAI_TxEnable(base, 0x0);

    /* Set the handle state */
    handle->state = kESAI_Idle;
}

void ESAI_TransferAbortReceiveEDMA(ESAI_Type *base, esai_edma_handle_t *handle)
{
    assert(handle);

    /* Disable dma */
    EDMA_AbortTransfer(handle->dmaHandle);

    /* Disable Rx */
    ESAI_RxEnable(base, 0x0);

    /* Set the handle state */
    handle->state = kESAI_Idle;
}

status_t ESAI_TransferGetSendCountEDMA(ESAI_Type *base, esai_edma_handle_t *handle, size_t *count)
{
    assert(handle);

    status_t status = kStatus_Success;

    if (handle->state != kESAI_Busy)
    {
        status = kStatus_NoTransferInProgress;
    }
    else
    {
        *count = (handle->transferSize[handle->queueDriver] -
                  (uint32_t)handle->nbytes *
                      EDMA_GetRemainingMajorLoopCount(handle->dmaHandle->base, handle->dmaHandle->channel));
    }

    return status;
}

status_t ESAI_TransferGetReceiveCountEDMA(ESAI_Type *base, esai_edma_handle_t *handle, size_t *count)
{
    assert(handle);

    status_t status = kStatus_Success;

    if (handle->state != kESAI_Busy)
    {
        status = kStatus_NoTransferInProgress;
    }
    else
    {
        *count = (handle->transferSize[handle->queueDriver] -
                  (uint32_t)handle->nbytes *
                      EDMA_GetRemainingMajorLoopCount(handle->dmaHandle->base, handle->dmaHandle->channel));
    }

    return status;
}

/*
 * Copyright (c) 2017, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
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

#include "fsl_uart_sdma.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/* Component ID definition, used by tools. */
#ifndef FSL_COMPONENT_ID
#define FSL_COMPONENT_ID "platform.drivers.iuart_sdma"
#endif

/*<! Structure definition for uart_sdma_private_handle_t. The structure is private. */
typedef struct _uart_sdma_private_handle
{
    UART_Type *base;
    uart_sdma_handle_t *handle;
} uart_sdma_private_handle_t;

/* UART SDMA transfer handle. */
enum _uart_sdma_tansfer_states
{
    kUART_TxIdle, /* TX idle. */
    kUART_TxBusy, /* TX busy. */
    kUART_RxIdle, /* RX idle. */
    kUART_RxBusy  /* RX busy. */
};

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*<! Private handle only used for internally. */
static UART_Type *const s_uartSdmaBases[] = UART_BASE_PTRS;
static uart_sdma_private_handle_t s_sdmaPrivateHandle[ARRAY_SIZE(s_uartSdmaBases)];

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*!
 * @brief UART SDMA send finished callback function.
 *
 * This function is called when UART SDMA send finished. It disables the UART
 * TX SDMA request and sends @ref kStatus_UART_TxIdle to UART callback.
 *
 * @param handle The SDMA handle.
 * @param param Callback function parameter.
 */
static void UART_SendSDMACallback(sdma_handle_t *handle, void *param, bool transferDone, uint32_t tcds);

/*!
 * @brief UART SDMA receive finished callback function.
 *
 * This function is called when UART SDMA receive finished. It disables the UART
 * RX SDMA request and sends @ref kStatus_UART_RxIdle to UART callback.
 *
 * @param handle The SDMA handle.
 * @param param Callback function parameter.
 */
static void UART_ReceiveSDMACallback(sdma_handle_t *handle, void *param, bool transferDone, uint32_t tcds);

/*!
 * @brief Get the UART instance from peripheral base address.
 *
 * @param base UART peripheral base address.
 * @return UART instance.
 */
extern uint32_t UART_GetInstance(UART_Type *base);

/*******************************************************************************
 * Code
 ******************************************************************************/

static void UART_SendSDMACallback(sdma_handle_t *handle, void *param, bool transferDone, uint32_t tcds)
{
    assert(param);

    uart_sdma_private_handle_t *uartPrivateHandle = (uart_sdma_private_handle_t *)param;

    if (transferDone)
    {
        UART_TransferAbortSendSDMA(uartPrivateHandle->base, uartPrivateHandle->handle);

        if (uartPrivateHandle->handle->callback)
        {
            uartPrivateHandle->handle->callback(uartPrivateHandle->base, uartPrivateHandle->handle, kStatus_UART_TxIdle,
                                                uartPrivateHandle->handle->userData);
        }
    }
}

static void UART_ReceiveSDMACallback(sdma_handle_t *handle, void *param, bool transferDone, uint32_t tcds)
{
    assert(param);

    uart_sdma_private_handle_t *uartPrivateHandle = (uart_sdma_private_handle_t *)param;

    if (transferDone)
    {
        /* Disable transfer. */
        UART_TransferAbortReceiveSDMA(uartPrivateHandle->base, uartPrivateHandle->handle);

        if (uartPrivateHandle->handle->callback)
        {
            uartPrivateHandle->handle->callback(uartPrivateHandle->base, uartPrivateHandle->handle, kStatus_UART_RxIdle,
                                                uartPrivateHandle->handle->userData);
        }
    }
}

void UART_TransferCreateHandleSDMA(UART_Type *base,
                                   uart_sdma_handle_t *handle,
                                   uart_sdma_transfer_callback_t callback,
                                   void *userData,
                                   sdma_handle_t *txSdmaHandle,
                                   sdma_handle_t *rxSdmaHandle,
                                   uint32_t eventSourceTx,
                                   uint32_t eventSourceRx)
{
    assert(handle);

    uint32_t instance = UART_GetInstance(base);

    memset(handle, 0, sizeof(*handle));

    handle->rxState = kUART_RxIdle;
    handle->txState = kUART_TxIdle;

    if (rxSdmaHandle)
    {
        rxSdmaHandle->eventSource = eventSourceRx;
    }

    if (txSdmaHandle)
    {
        txSdmaHandle->eventSource = eventSourceTx;
    }

    handle->rxSdmaHandle = rxSdmaHandle;
    handle->txSdmaHandle = txSdmaHandle;

    handle->callback = callback;
    handle->userData = userData;

    s_sdmaPrivateHandle[instance].base = base;
    s_sdmaPrivateHandle[instance].handle = handle;

    /* Configure TX. */
    if (txSdmaHandle)
    {
        SDMA_SetCallback(handle->txSdmaHandle, UART_SendSDMACallback, &s_sdmaPrivateHandle[instance]);
    }

    /* Configure RX. */
    if (rxSdmaHandle)
    {
        SDMA_SetCallback(handle->rxSdmaHandle, UART_ReceiveSDMACallback, &s_sdmaPrivateHandle[instance]);
    }
}

status_t UART_SendSDMA(UART_Type *base, uart_sdma_handle_t *handle, uart_transfer_t *xfer)
{
    assert(handle);
    assert(handle->txSdmaHandle);
    assert(xfer);
    assert(xfer->data);
    assert(xfer->dataSize);

    sdma_transfer_config_t xferConfig;
    status_t status;
    sdma_peripheral_t perType = kSDMA_PeripheralTypeUART;

    /* If previous TX not finished. */
    if (kUART_TxBusy == handle->txState)
    {
        status = kStatus_UART_TxBusy;
    }
    else
    {
        handle->txState = kUART_TxBusy;
        handle->txDataSizeAll = xfer->dataSize;

#if defined(FSL_FEATURE_SOC_SPBA_COUNT) && (FSL_FEATURE_SOC_SPBA_COUNT > 0)
        bool isSpba = SDMA_IsPeripheralInSPBA((uint32_t)base);
        /* Judge if the instance is located in SPBA */
        if (isSpba)
        {
            perType = kSDMA_PeripheralTypeUART_SP;
        }
#endif /* FSL_FEATURE_SOC_SPBA_COUNT */

        /* Prepare transfer. */
        SDMA_PrepareTransfer(&xferConfig, (uint32_t)xfer->data, (uint32_t) & (base->UTXD), sizeof(uint8_t),
                             sizeof(uint8_t), sizeof(uint8_t), xfer->dataSize, handle->txSdmaHandle->eventSource,
                             perType, kSDMA_MemoryToPeripheral);

        /* Submit transfer. */
        SDMA_SubmitTransfer(handle->txSdmaHandle, &xferConfig);

        SDMA_StartTransfer(handle->txSdmaHandle);

        /* Enable UART TX SDMA. */
        UART_EnableTxDMA(base, true);
        status = kStatus_Success;
    }

    return status;
}

status_t UART_ReceiveSDMA(UART_Type *base, uart_sdma_handle_t *handle, uart_transfer_t *xfer)
{
    assert(handle);
    assert(handle->rxSdmaHandle);
    assert(xfer);
    assert(xfer->data);
    assert(xfer->dataSize);

    sdma_transfer_config_t xferConfig;
    status_t status;
    sdma_peripheral_t perType = kSDMA_PeripheralTypeUART;

    /* If previous RX not finished. */
    if (kUART_RxBusy == handle->rxState)
    {
        status = kStatus_UART_RxBusy;
    }
    else
    {
        handle->rxState = kUART_RxBusy;
        handle->rxDataSizeAll = xfer->dataSize;

#if defined(FSL_FEATURE_SOC_SPBA_COUNT) && (FSL_FEATURE_SOC_SPBA_COUNT > 0)
        bool isSpba = SDMA_IsPeripheralInSPBA((uint32_t)base);
        /* Judge if the instance is located in SPBA */
        if (isSpba)
        {
            perType = kSDMA_PeripheralTypeUART_SP;
        }
#endif /* FSL_FEATURE_SOC_SPBA_COUNT */

        /* Prepare transfer. */
        SDMA_PrepareTransfer(&xferConfig, (uint32_t) & (base->URXD), (uint32_t)xfer->data, sizeof(uint8_t),
                             sizeof(uint8_t), sizeof(uint8_t), xfer->dataSize, handle->rxSdmaHandle->eventSource,
                             perType, kSDMA_PeripheralToMemory);

        /* Submit transfer. */
        SDMA_SubmitTransfer(handle->rxSdmaHandle, &xferConfig);

        SDMA_StartTransfer(handle->rxSdmaHandle);

        /* Enable UART RX SDMA. */
        UART_EnableRxDMA(base, true);

        status = kStatus_Success;
    }

    return status;
}

void UART_TransferAbortSendSDMA(UART_Type *base, uart_sdma_handle_t *handle)
{
    assert(handle);
    assert(handle->txSdmaHandle);

    /* Disable UART TX SDMA. */
    UART_EnableTxDMA(base, false);

    /* Stop transfer. */
    SDMA_AbortTransfer(handle->txSdmaHandle);

    handle->txState = kUART_TxIdle;
}

void UART_TransferAbortReceiveSDMA(UART_Type *base, uart_sdma_handle_t *handle)
{
    assert(handle);
    assert(handle->rxSdmaHandle);

    /* Disable UART RX SDMA. */
    UART_EnableRxDMA(base, false);

    /* Stop transfer. */
    SDMA_AbortTransfer(handle->rxSdmaHandle);

    handle->rxState = kUART_RxIdle;
}

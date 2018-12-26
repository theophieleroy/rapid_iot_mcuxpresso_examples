/*
 * The Clear BSD License
 * Copyright (c) 2016, NXP Semiconductors, N.V.
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

#include "rwip.h"
#include "fsl_usart.h"
#include "uart_adapter.h"

/*******************************************************************************
 * Variables
 ******************************************************************************/
/* USART adapter handle */
static usart_handle_t s_uartHandle;

/* UART adapter user callback information */
static user_usart_cb_t s_userData;

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*!
 * @brief UART adapter user callback.
 *
 * This function is called back when the expected size data has been received from or
 * sent through UART adapter.
 */
static void ADAPTER_UartUserCallback(USART_Type *base, usart_handle_t *handle, status_t status, void *userData);

/*******************************************************************************
 * Code
 ******************************************************************************/

static void ADAPTER_UartUserCallback(USART_Type *base, usart_handle_t *handle, status_t status, void *userData)
{
    if (kStatus_USART_TxIdle == status)
    {
        if ((userData) && (((user_usart_cb_t *)(handle->userData))->usart_tx_callback))
        {
            ((user_usart_cb_t *)(handle->userData))->usart_tx_callback();
        }
    }

    if (kStatus_USART_RxIdle == status)
    {
        if ((userData) && (((user_usart_cb_t *)(handle->userData))->usart_rx_callback))
        {
            ((user_usart_cb_t *)(handle->userData))->usart_rx_callback();
        }
    }
}

void ADAPTER_UartInit(void)
{
    /*
     * config.baudRate_Bps = 115200U;
     * config.parityMode = kUART_ParityDisabled;
     * config.stopBitCount = kUART_OneStopBit;
     * config.loopback = false;
     * config.enableTx = false;
     * config.enableRx = false;
     */
    usart_config_t config;
    USART_GetDefaultConfig(&config);
    config.baudRate_Bps = DEMO_USART_BAUDRATE;
    config.enableTx = true;
    config.enableRx = true;

    USART_Init(DEMO_USART, &config, DEMO_USART_CLK_FREQ);

    // update Flexcomm fractional divider to correct the baud rate
    CLOCK_SetFRGClock(DEMO_USART == USART0 ? kCLOCK_DivFrg0 : kCLOCK_DivFrg1,
                      FLEXCOMM_CLK(DEMO_USART_CLK_FREQ, DEMO_USART_BAUDRATE));

    s_userData.usart_tx_callback = NULL;
    s_userData.usart_rx_callback = NULL;
    USART_TransferCreateHandle(DEMO_USART, &s_uartHandle, ADAPTER_UartUserCallback, &s_userData);
}

void ADAPTER_UartTransmitInt(uint8_t *buf, uint32_t size, void (*tx_callback)(void))
{
    usart_transfer_t sendXfer;

    sendXfer.data = buf;
    sendXfer.dataSize = size;
    ((user_usart_cb_t *)(s_uartHandle.userData))->usart_tx_callback = tx_callback;

    USART_TransferSendNonBlocking(DEMO_USART, &s_uartHandle, &sendXfer);
}

void ADAPTER_UartReceiveInt(uint8_t *buf, uint32_t size, void (*rx_callback)(void))
{
    usart_transfer_t receiveXfer;

    receiveXfer.data = buf;
    receiveXfer.dataSize = size;
    ((user_usart_cb_t *)(s_uartHandle.userData))->usart_rx_callback = rx_callback;

    USART_TransferReceiveNonBlocking(DEMO_USART, &s_uartHandle, &receiveXfer, NULL);
}

/*
 * The Clear BSD License
 * Copyright (c) 2013, Freescale Semiconductor, Inc.
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

#include "utilities/fsl_assert.h"
#include <stdlib.h>
#include <string.h>
#include "uart/fsl_uart_types.h"
#include "uart/fsl_uart_driver.h"
#include "bootloader_common.h"
#include "fsl_uart_driver_unit_test.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/

/* See fsl_uart_driver_unit_test.h for documentation of this function.*/
status_t uart_async_driver_test(uint32_t uartInstance)
{
    uint8_t sourceBuff[20] = {10,  13,  'U', 'A', 'R', 'T', ' ', ' ', 'A', 'S',
                              'Y', 'N', 'C', ' ', 'T', 'e', 's', 't', 10,  13};
    uint32_t txCurrentByteCount;
    uint32_t readByteCount;
    uint8_t rxBuff[10] = {0};
    uint8_t *pTxBuff;

    uint32_t bytesTransmittedCount = 0;
    uint32_t bytesReceivedCount = 0;
    uint32_t flagRxDone = 0;

    readByteCount = sizeof(rxBuff);

    uart_user_config_t uartConfig;
    uartConfig.baudRate = 9600;
    uartConfig.bitCountPerChar = kUart8BitsPerChar;
    uartConfig.parityMode = kUartParityDisabled;
    uartConfig.stopBitCount = kUartOneStopBit;

    /* instantiate variable uartState of type uart_state_t*/
    uart_state_t uartState;

    /* init the uart module with instance number and config structure*/
    uart_init(uartInstance, &uartConfig, &uartState);

    while (1)
    {
        if (uart_get_transmit_status(&uartState, &bytesTransmittedCount) != kStatus_UART_TxBusy)
        {
            /* if rx done, then use the rx buffer as the tx buffer and send out the rx data*/
            if (flagRxDone == 1)
            {
                flagRxDone = 0;
                pTxBuff = rxBuff;
                txCurrentByteCount = sizeof(rxBuff);
            }
            /* else if rx not done, then update the tx buffer with default data*/
            else
            {
                /* insert uart instance number within source buff  */
                sourceBuff[6] = '0' + uartInstance;
                /* update txBuff with sourceBuff*/
                pTxBuff = sourceBuff;
                txCurrentByteCount = sizeof(sourceBuff);
            }

            uart_send_data_async(&uartState, pTxBuff, txCurrentByteCount);
        }

        if (uart_get_receive_status(&uartState, &bytesReceivedCount) != kStatus_UART_RxBusy)
        {
            uart_receive_data_async(&uartState, rxBuff, readByteCount);
            if (bytesReceivedCount == readByteCount)
            {
                /* check if receive buffer starts with an 'x' or 'X', this is the escape sequence*/
                /* from this test*/
                if ((rxBuff[0] == 'x') || (rxBuff[0] == 'X'))
                {
                    /* abort sending and receiving data */
                    uart_abort_sending_data(&uartState);
                    uart_abort_receiving_data(&uartState);

                    /* shut down the uart*/
                    uart_shutdown(&uartState);

                    return kStatus_Success;
                }

                /* set the Rx done flag to indicate rx is done */
                flagRxDone = 1;
                /* clear the bytesReceivedCount*/
                bytesReceivedCount = 0;
            }
        }
    }
}

/*******************************************************************************
 * EOF
 ******************************************************************************/

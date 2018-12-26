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

#ifndef _UART_ADAPTER_H_
#define _UART_ADAPTER_H_

#include "fsl_common.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define DEMO_USART USART0
#define DEMO_USART_BAUDRATE 115200
#define DEMO_USART_CLK_FREQ CLOCK_GetFreq(kCLOCK_BusClk)

typedef struct
{
    void (*usart_tx_callback)(void); /*!< tx complete callback */
    void (*usart_rx_callback)(void); /*!< rx complete callback */
} user_usart_cb_t;

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus */

/*!
 * @brief Initialize the UART adapter.
 */
void ADAPTER_UartInit(void);

/*!
 * @brief Start a data reception.
 *
 * This function receives data from the UART adapter.
 *
 * @param bufptr Pointer to the receive buffer.
 * @param size Size of the expected reception.
 * @param callback Pointer to the function called back when transfer finished.
 * @param dummy Dummy data pointer returned to callback when reception is finished.
 */
void ADAPTER_UartTransmitInt(uint8_t *buf, uint32_t size, void (*tx_callback)(void));

/*!
 * @brief Starts a data transmission.
 *
 * This function transmit data to the UART adapter
 *
 * @param bufptr Pointer to the transmit buffer.
 * @param size Size of the transmission.
 * @param callback Pointer to the function called back when transfer finished.
 * @param dummy Dummy data pointer returned to callback when transmission is finished.
 */
void ADAPTER_UartReceiveInt(uint8_t *buf, uint32_t size, void (*rx_callback)(void));

#if defined(__cplusplus)
}
#endif /* __cplusplus */

#endif /* _UART_ADAPTER_H_ */

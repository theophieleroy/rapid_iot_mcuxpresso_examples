/*
 * The Clear BSD License
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
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
#ifndef __FSL_UART_RTOS_H__
#define __FSL_UART_RTOS_H__

#include "fsl_uart.h"
#include <os.h>

/*!
 * @addtogroup uart
 * @{
 */

/*! @file*/

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/*! @brief UART µCOS-III driver version */
#define FSL_UART_UCOSIII_DRIVER_VERSION (MAKE_VERSION(2, 0, 0)) /*!< Version 2.0.0. */

struct rtos_uart_config
{
    UART_Type *base;
    uint32_t srcclk;
    uint32_t baudrate;
    uart_parity_mode_t parity;
    uart_stop_bit_count_t stopbits;
    uint8_t *buffer;
    uint32_t buffer_size;
};

typedef struct _uart_rtos_handle
{
    UART_Type *base;
    struct _uart_transfer tx_xfer;
    struct _uart_transfer rx_xfer;
    OS_SEM rx_sem;
    OS_SEM tx_sem;
#define RTOS_UART_COMPLETE 0x1
    OS_FLAG_GRP rx_event;
    OS_FLAG_GRP tx_event;
    void *t_state; /* transactional layer state */
} uart_rtos_handle_t;

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @name UART RTOS Operation
 * @{
 */

/*!
 * @brief Initializes a UART instance for operation in RTOS.
 *
 * @param handle The RTOS UART handle, the pointer to allocated space for RTOS context.
 * @param uart_t_handle The pointer to an allocated space where to store transactional layer internal state.
 * @param cfg The pointer to the parameters required to configure the UART after initialization.
 * @return 0 Succeed, others fail.
 */
int UART_RTOS_Init(uart_rtos_handle_t *handle, uart_handle_t *t_handle, const struct rtos_uart_config *cfg);

/*!
 * @brief Deinitializes a UART instance for operation.
 *
 * This function deinitializes the UART module, sets all register values to reset value,
 * and releases the resources.
 *
 * @param handle The RTOS UART handle.
 */
int UART_RTOS_DeInit(uart_rtos_handle_t *handle);

/*!
 * @name UART transactional Operation
 * @{
 */

/*!
 * @brief Sends data in the background.
 *
 * This function sends data. It is a synchronous API.
 * If the hardware buffer is full, the task is in the blocked state.
 *
 * @param handle The RTOS UART handle.
 * @param buffer The pointer to buffer to send.
 * @param length The number of bytes to send.
 */
int UART_RTOS_Send(uart_rtos_handle_t *handle, const uint8_t *buffer, uint32_t length);

/*!
 * @brief Receives data. 
 *
 * This function receives data from UART. It is a synchronous API. If any data is immediately available,
 * it is returned immediately and the number of bytes received.
 *
 * @param handle The RTOS UART handle.
 * @param buffer The pointer to buffer where to write received data.
 * @param length The number of bytes to receive.
 * @param received The pointer to variable of a size_t where the number of received data is filled.
 */
int UART_RTOS_Receive(uart_rtos_handle_t *handle, uint8_t *buffer, uint32_t length, size_t *received);

/* @} */

#if defined(__cplusplus)
}
#endif

/*! @}*/

#endif /* __FSL_UART_RTOS_H__ */

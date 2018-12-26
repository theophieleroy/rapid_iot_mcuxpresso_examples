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
#ifndef __FSL_LPSCI_RTOS_H__
#define __FSL_LPSCI_RTOS_H__

#include "fsl_lpsci.h"
#include <ucos_ii.h>

/*!
 * @addtogroup lpsci
 * @{
 */

/*! @file*/

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/*! @brief LPSCI uCOS II driver version */
#define FSL_LPSCI_UCOSII_DRIVER_VERSION (MAKE_VERSION(2, 0, 0)) /*!< Version 2.0.0. */

struct rtos_lpsci_config
{
    LPSCI_Type *base;
    uint32_t srcclk;
    uint32_t baudrate;
    lpsci_parity_mode_t parity;
    lpsci_stop_bit_count_t stopbits;
    uint8_t *buffer;
    uint32_t buffer_size;
};

typedef struct _lpsci_rtos_handle
{
    LPSCI_Type *base;
    struct _lpsci_transfer tx_xfer;
    struct _lpsci_transfer rx_xfer;
    OS_EVENT *rx_sem;
    OS_EVENT *tx_sem;
#define RTOS_LPSCI_COMPLETE 0x1
    OS_FLAG_GRP *rx_event;
    OS_FLAG_GRP *tx_event;
    void *t_state; /* transactional layer state */
} lpsci_rtos_handle_t;

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @name LPSCI RTOS Operation
 * @{
 */

/*!
 * @brief Initializes an LPSCI instance for operation in RTOS.
 *
 * @param handle The RTOS LPSCI handle, the pointer to allocated space for RTOS context.
 * @param lpsci_t_handle The pointer to allocated space where to store transactional layer internal state.
 * @param cfg The pointer to the parameters required to configure the LPSCI after initialization.
 * @return 0 succeed, others failed
 */
int LPSCI_RTOS_Init(lpsci_rtos_handle_t *handle, lpsci_handle_t *t_handle, const struct rtos_lpsci_config *cfg);

/*!
 * @brief Deinitializes an LPSCI instance for operation.
 *
 * This function deinitializes the LPSCI modulem, set all register value to reset value
 * and releases the resources.
 *
 * @param handle The RTOS LPSCI handle.
 */
int LPSCI_RTOS_Deinit(lpsci_rtos_handle_t *handle);

/*!
 * @name LPSCI transactional Operation
 * @{
 */

/*!
 * @brief Send data in background.
 *
 * This function sends data. It is synchronous API.
 * If the HW buffer is full, the task is in the blocked state.
 *
 * @param handle The RTOS LPSCI handle.
 * @param buffer The pointer to buffer to send.
 * @param length The number of bytes to send.
 */
int LPSCI_RTOS_Send(lpsci_rtos_handle_t *handle, const uint8_t *buffer, uint32_t length);

/*!
 * @brief Receives data. It is synchronous API.
 *
 * This function receives data from LPSCI. If any data is immediately available
 * it will be returned imidiately and the number of bytes received.
 *
 * @param handle The RTOS LPSCI handle.
 * @param buffer The pointer to buffer where to write received data.
 * @param length The number of bytes to receive.
 * @param received The pointer to variable of size_t where the number of received data will be filled.
 */
int LPSCI_RTOS_Receive(lpsci_rtos_handle_t *handle, uint8_t *buffer, uint32_t length, size_t *received);

/* @} */

#if defined(__cplusplus)
}
#endif

/*! @}*/

#endif /* __FSL_LPSCI_RTOS_H__ */

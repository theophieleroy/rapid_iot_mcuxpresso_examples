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

#ifndef __FSL_LPUART_HAL_H__
#define __FSL_LPUART_HAL_H__

#include "bootloader_common.h"
#include "lpuart/fsl_lpuart_types.h"
#include "fsl_device_registers.h"
#include <stdint.h>
#include <stdbool.h>

/*!
 * @addtogroup lpuart_hal
 * @{
 */

/////////////////////////////////////////////////////////////////////////////
// API
/////////////////////////////////////////////////////////////////////////////

#if defined(__cplusplus)
extern "C" {
#endif

//! @name LPUART HAL Driver
//@{

/*!
 * @brief Reset LPUART controller
 *
 * @param   baseAddr Module base address.
 */
void lpuart_hal_reset(LPUART_Type *baseAddr);

/*!
 * @brief Initialize LPUART controller
 *  Default setting: 8-bit data, no parity, one stop bit
 *
 * @param   baseAddr Module base address.
 * @param   lpuartSourceClockInHz      LPUART source input clock in Hz.
 * @param   baudRate      LPUART desired baud rate.
 * @return  An error code or kStatus_Success
 */
status_t lpuart_hal_init(LPUART_Type *baseAddr, uint32_t lpuartSourceClockInHz, uint32_t baudRate);

/*!
 * @brief Configure LPUART baud rate
 *  In some LPUART instances it is required that the transmitter/receiver should be disabled
 *  before calling this function.
 *  Generally this may be applied to all LPUARTs to ensure safe operation
 *
 * @param   baseAddr Module base address.
 * @param   sourceClockInHz      LPUART source input clock in Hz.
 * @param   desiredBaudRate      LPUART desired baud rate.
 * @return  An error code or kStatus_Success
 */
status_t lpuart_hal_set_baud_rate(LPUART_Type *baseAddr, uint32_t sourceClockInHz, uint32_t desiredBaudRate);

/*!
 * @brief Set the LPUART baud rate modulo divisor
 *
 * @param   baseAddr Module base address.
 * @param   baudRateDivisor The baud rate modulo division "SBR"
 * @return  An error code or kStatus_Success
 */
status_t lpuart_hal_set_baud_divisor(LPUART_Type *baseAddr, uint32_t baudRateDivisor);

/*!
 * @brief Configure number of bits per char in LPUART controller
 *  In some LPUART instances it is required that the transmitter/receiver should be disabled
 *  before calling this function.
 *  Generally this may be applied to all LPUARTs to ensure safe operation
 *
 * @param   baseAddr Module base address.
 * @param   bitCountPerChar   Number of bits per char (8, 9, or
 *                            10, depending on the uart instance)
 * @return  An error code or kStatus_Success
 */
status_t lpuart_hal_configure_bit_count_per_char(LPUART_Type *baseAddr, lpuart_bit_count_per_char_t bitCountPerChar);

/*!
 * @brief Configure parity mode in LPUART controller
 *  In some LPUART instances it is required that the transmitter/receiver should be disabled
 *  before calling this function.
 *  Generally this may be applied to all LPUARTs to ensure safe operation
 *
 * @param   baseAddr Module base address.
 * @param   parityModeType  Parity mode (enabled, disable, odd, even - see parity_mode_t struct)
 */
void lpuart_hal_configure_parity_mode(LPUART_Type *baseAddr, lpuart_parity_mode_t parityModeType);

/*!
 * @brief Configure number of stop bits in LPUART controller
 *  In some LPUART instances it is required that the transmitter/receiver should be disabled
 *  before calling this function.
 *  Generally this may be applied to all LPUARTs to ensure safe operation
 *
 * @param   baseAddr Module base address.
 * @param   stopBitCount      Number of stop bits (1 or 2 - see uart_stop_bit_count_t struct)
 * @return  An error code (an unsupported setting in some uarts) or kStatus_Success
 */
status_t lpuart_hal_configure_stop_bit_count(LPUART_Type *baseAddr, lpuart_stop_bit_count_t stopBitCount);

/*!
 * @brief Configure transmit and receive inversion control in LPUART controller
 *  This function should only be called when the LPUART is between transmit and receive packets
 *
 * @param   baseAddr Module base address.
 * @param   rxInvert     Enable (1) or disable (0) receive inversion
 * @param   txInvert     Enable (1) or disable (0) transmit inversion
 */
void lpuart_hal_configure_tx_rx_inversion(LPUART_Type *baseAddr, bool rxInvert, bool txInvert);

/*!
 * @brief Enable LPUART transmitter
 *
 * @param   baseAddr Module base address.
 */
static inline void lpuart_hal_enable_transmitter(LPUART_Type *baseAddr)
{
    LPUART_SET_CTRL(baseAddr, LPUART_CTRL_TE_MASK);
}

/*!
 * @brief Disable LPUART transmitter
 *
 * @param   baseAddr Module base address.
 */
static inline void lpuart_hal_disable_transmitter(LPUART_Type *baseAddr)
{
    LPUART_CLR_CTRL(baseAddr, LPUART_CTRL_TE_MASK);
}

/*!
 * @brief Enable LPUART receiver
 *
 * @param   baseAddr Module base address.
 */
static inline void lpuart_hal_enable_receiver(LPUART_Type *baseAddr)
{
    LPUART_SET_CTRL(baseAddr, LPUART_CTRL_RE_MASK);
}

/*!
 * @brief Disable LPUART receiver
 *
 * @param   baseAddr Module base address.
 */
static inline void lpuart_hal_disable_receiver(LPUART_Type *baseAddr)
{
    LPUART_SET_CTRL(baseAddr, LPUART_CTRL_RE_MASK);
}

/*!
 * @brief Send LPUART 8-bit char
 *
 * @param   lpuartInstance    LPUART Instance.
 * @param   data        data to send (8-bit)
 */
static inline void lpuart_hal_putchar(LPUART_Type *baseAddr, uint8_t data)
{
    // put 8-bit data into the lpuart data register (low-8bit)
    LPUART_WR_DATA(baseAddr, data);
}

/*!
 * @brief Get LPUART 8-bit char
 *
 * @param   baseAddr Module base address.
 * @param   readData    data read from receive (8-bit)
 */
static inline uint8_t lpuart_hal_getchar(LPUART_Type *baseAddr)
{
    return (uint8_t)LPUART_RD_DATA(baseAddr); // read 8-bit data from data register
}

/*!
 * @brief  Get LPUART Transmit data register empty flag
 *
 * @param   baseAddr Module base address.
 * @return  Status of Transmit data register empty flag, sets when transmit buffer is empty
 */
static inline bool lpuart_hal_is_transmit_data_register_empty(LPUART_Type *baseAddr)
{
    return LPUART_RD_STAT_TDRE(baseAddr);
}

/*!
 * @brief  Get LPUART Transmission complete flag
 *
 * @param   baseAddr Module base address.
 * @return  Status of Transmission complete flag, sets when transmitter is idle
 *          (transmission activity complete)
 */
static inline bool lpuart_hal_is_transmission_complete(LPUART_Type *baseAddr)
{
    return LPUART_RD_STAT_TC(baseAddr);
}

/*!
 * @brief  Get LPUART Receive data register full flag
 *
 * @param   baseAddr Module base address.
 * @return  Status of Receive data register full flag, sets when the receive data buffer is full
 */
static inline bool lpuart_hal_is_receive_data_register_full(LPUART_Type *baseAddr)
{
    return LPUART_RD_STAT_RDRF(baseAddr);
}

/*!
 * @brief  Enables LPUART recieve interrupt
 *
 * @param   baseAddr Module base address.
 */
static inline void lpuart_hal_enable_receive_interrupt(LPUART_Type *baseAddr)
{
    LPUART_BWR_CTRL_RIE(baseAddr, 1);
}

//@}

#if defined(__cplusplus)
}
#endif

/*! @}*/

#endif // __FSL_LPUART_HAL_H__
////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////

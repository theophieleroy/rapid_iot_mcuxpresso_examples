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
#ifndef __FSL_LPUART_TYPES_H__
#define __FSL_LPUART_TYPES_H__

#include <stdint.h>

/*!
 * @addtogroup lpuart_types
 * @{
 */

////////////////////////////////////////////////////////////////////////////////
// Definitions
////////////////////////////////////////////////////////////////////////////////

//! @brief Error codes for the LPUART driver.
enum _lpuart_errors
{
    kStatus_LPUART_BaudRateCalculationError = 1, //!< LPUART Baud Rate calculation error out of range.
    kStatus_LPUART_BaudRatePercentDiffExceeded,  //!< LPUART Baud Rate exceeds percentage difference
    kStatus_LPUART_BitCountNotSupported,         //!< LPUART bit count config not supported.
    kStatus_LPUART_StopBitCountNotSupported,     //!< LPUART stop bit count config not supported.
    kStatus_LPUART_RxStandbyModeError,           //!< LPUART unable to place receiver in standby mode.
    kStatus_LPUART_ClearStatusFlagError,         //!< LPUART clear status flag error.
    kStatus_LPUART_MSBFirst_NotSupported,        //!< LPUART MSB first feature not supported.
    kStatus_LPUART_Resync_NotSupported,          //!< LPUART resync disable operation not supported.
    kStatus_LPUART_Tx_NotDisabled,               //!< LPUART Transmitter not disabled before enabling feature
    kStatus_LPUART_Rx_NotDisabled,               //!< LPUART Receiver not disabled before enabling feature
    kStatus_LPUART_Tx_or_Rx_NotDisabled,         //!< LPUART Transmitter or Receiver not disabled
    kStatus_LPUART_Tx_Busy,                      //!< LPUART transmit still in progress.
    kStatus_LPUART_Rx_Busy,                      //!< LPUART receive still in progress.
    kStatus_LPUART_NoTransmitInProgress,         //!< LPUART no transmit in progress.
    kStatus_LPUART_NoReceiveInProgress,          //!< LPUART no receive in progress.
    kStatus_LPUART_Invalid_Instance_Number,      //!< Invalid LPUART instance number
    kStatus_LPUART_Invalid_BitSetting,           //!< Invalid setting for desired LPUART register bit field
    kStatus_LPUART_OverSampling_NotSupported,    //!< LPUART oversampling not supported.
    kStatus_LPUART_BothEdge_NotSupported,        //!< LPUART both edge sampling not supported.
    kStatus_LPUART_Timeout,                      //!< LPUART transfer timed out.
};

//! @brief LPUART number of stop bits
typedef enum
{
    kLpuartOneStopBit, //!< one stop bit
    kLpuartTwoStopBit, //!< two stop bits
} lpuart_stop_bit_count_t;

//! @brief LPUART parity mode
typedef enum
{
    kLpuartParityDisabled = 0x0, //!< parity disabled
    kLpuartParityEven = 0x2,     //!< parity enabled, type even, bit setting: PE|PT = 10
    kLpuartParityOdd = 0x3,      //!< parity enabled, type odd,  bit setting: PE|PT = 11
} lpuart_parity_mode_t;

//! @brief LPUART number of bits in a character
typedef enum
{
    kLpuart8BitsPerChar,  //!< 8-bit data characters
    kLpuart9BitsPerChar,  //!< 9-bit data characters
    kLpuart10BitsPerChar, //!< 10-bit data characters
} lpuart_bit_count_per_char_t;

/*! @}*/

#endif // __FSL_LPUART_TYPES_H__
////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////

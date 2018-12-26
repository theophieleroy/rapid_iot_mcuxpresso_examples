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
#include "fsl_lpuart_hal.h"
#include "bootloader_common.h"
#include "fsl_device_registers.h"

#if BL_CONFIG_LPUART
////////////////////////////////////////////////////////////////////////////////
// Definitions
////////////////////////////////////////////////////////////////////////////////

enum
{
    //! The lower the OSR value the better chance we have at hitting a larger multitude
    //! of baud rates in the 0 - 115200 range, minimum for LPUART is 4
    kLPUART_OSR_Value = 4U
};

////////////////////////////////////////////////////////////////////////////////
// Code
////////////////////////////////////////////////////////////////////////////////

// See fsl_lpuart_hal.h for documentation of this function.
void lpuart_hal_reset(LPUART_Type *baseAddr)
{
#if FSL_FEATURE_LPUART_HAS_SOFT_RESET_SUPPORT
    // Soft Reset can reset all registers, except the Global Register, So
    //  we need to clear GLOBAL manually.
    LPUART_WR_GLOBAL_RST(baseAddr, 0x1U);
    LPUART_WR_GLOBAL(baseAddr, 0u);
#else
    // Restore those control and configuration registers which are used to
    // be operated
    LPUART_WR_CTRL(baseAddr, 0u);
    LPUART_WR_BAUD(baseAddr, 0x0f000004u);
#endif
}

// See fsl_lpuart_hal.h for documentation of this function.
status_t lpuart_hal_init(LPUART_Type *baseAddr, uint32_t lpuartSourceClockInHz, uint32_t baudRate)
{
    // first, disable the LPUART transmitter and receiver
    lpuart_hal_disable_transmitter(baseAddr);
    lpuart_hal_disable_receiver(baseAddr);

    //*******************************************************
    // calculate baud rate settings
    //*******************************************************
    if (lpuart_hal_set_baud_rate(baseAddr, lpuartSourceClockInHz, baudRate) != kStatus_Success)
    {
        return kStatus_LPUART_BaudRateCalculationError;
    }

    //*******************************************************
    // configure number of bits in a char
    //*******************************************************
    // 8-bit data characters
    if (lpuart_hal_configure_bit_count_per_char(baseAddr, kLpuart8BitsPerChar) != kStatus_Success)
    {
        return kStatus_LPUART_BitCountNotSupported;
    }

    //*******************************************************
    // configure the parity enable/type
    //*******************************************************
    // parity disabled
    lpuart_hal_configure_parity_mode(baseAddr, kLpuartParityDisabled);

    //*******************************************************
    // configure the number of stop bits
    //*******************************************************
    // one stop bit
    if (lpuart_hal_configure_stop_bit_count(baseAddr, kLpuartOneStopBit) != kStatus_Success)
    {
        return kStatus_LPUART_StopBitCountNotSupported;
    }

    //*******************************************************
    // configure tx and rx inversions
    //*******************************************************
    // receive & transmit data not inverted
    lpuart_hal_configure_tx_rx_inversion(baseAddr, 0, 0);

    // finally, enable the LPUART transmitter and receiver
    lpuart_hal_enable_transmitter(baseAddr);
    lpuart_hal_enable_receiver(baseAddr);

    return kStatus_Success;
}

// See fsl_lpuart_hal.h for documentation of this function.
status_t lpuart_hal_set_baud_rate(LPUART_Type *baseAddr, uint32_t sourceClockInHz, uint32_t desiredBaudRate)
{
    // Calculate SBR rounded to nearest whole number, 14.4 = 14, 14.6 = 15
    // This gives the least error possible
    uint32_t sbr = (uint32_t)(((sourceClockInHz * 10) / (desiredBaudRate * kLPUART_OSR_Value)) + 5) / 10;
    uint32_t calculatedBaud = (uint32_t)(sourceClockInHz / (sbr * kLPUART_OSR_Value));
    uint32_t baudDiff = MAX(calculatedBaud, desiredBaudRate) - MIN(calculatedBaud, desiredBaudRate);

    // next, check to see if actual baud rate is within 3% of desired baud rate
    // based on the best calculate osr value
    if (baudDiff < ((desiredBaudRate / 100) * 3))
    {
        // program the osr value (bit value is one less than actual value)
        LPUART_WR_BAUD(baseAddr,
                       LPUART_BAUD_OSR(kLPUART_OSR_Value - 1) | LPUART_BAUD_SBR(sbr) | LPUART_BAUD_BOTHEDGE_MASK);
    }
    else
    {
        // Unacceptable baud rate difference of more than 3%
        return kStatus_LPUART_BaudRatePercentDiffExceeded;
    }

    return kStatus_Success;
}

// See fsl_lpuart_hal.h for documentation of this function.
status_t lpuart_hal_set_baud_divisor(LPUART_Type *baseAddr, uint32_t baudRateDivisor)
{
    // check to see if baudRateDivisor is out of range of register bits

    // program the sbr (baudRateDivisor) value to the BAUD registers
    LPUART_BWR_BAUD_SBR(baseAddr, baudRateDivisor);

    return kStatus_Success;
}

// See fsl_lpuart_hal.h for documentation of this function.
status_t lpuart_hal_configure_bit_count_per_char(LPUART_Type *baseAddr, lpuart_bit_count_per_char_t bitCountPerChar)
{
// configure number of bits in a char
#if FSL_FEATURE_LPUART_HAS_10BIT_DATA_SUPPORT
    if ((bitCountPerChar) == kLpuart10BitsPerChar)
    {
        LPUART_SET_BAUD(baseAddr, LPUART_BAUD_M10_MASK); // set M10 for 10-bit mode, M bit in C1 is don't care
    }
    else
    {
        LPUART_BWR_CTRL_M(baseAddr, bitCountPerChar);    // config 8- (M=0) or 9-bits (M=1)
        LPUART_CLR_BAUD(baseAddr, LPUART_BAUD_M10_MASK); // clear M10 to make sure not 10-bit mode
    }
#endif

    return kStatus_Success;
}

// See fsl_lpuart_hal.h for documentation of this function.
void lpuart_hal_configure_parity_mode(LPUART_Type *baseAddr, lpuart_parity_mode_t parityModeType)
{
    // configure the parity enable/type
    if ((parityModeType) == 0)
    {
        // parity disabled, hence parity type is don't care
        LPUART_CLR_CTRL(baseAddr, LPUART_CTRL_PE_MASK);
    }
    else
    {
        // parity enabled
        LPUART_SET_CTRL(baseAddr, LPUART_CTRL_PE_MASK);
        // parity odd/even depending on parity mode setting
        LPUART_BWR_CTRL_PT(baseAddr, (parityModeType)&0x1);
    }
}

// See fsl_lpuart_hal.h for documentation of this function.
status_t lpuart_hal_configure_stop_bit_count(LPUART_Type *baseAddr, lpuart_stop_bit_count_t stopBitCount)
{
#if FSL_FEATURE_LPUART_HAS_STOP_BIT_CONFIG_SUPPORT
    // configure the number of stop bits
    LPUART_BWR_BAUD_SBNS(baseAddr, stopBitCount);

    return kStatus_Success;
#else
    // stop bit configuration not supported, only one stop bit is supported
    if (stopBitCount != kLpuartOneStopBit)
    {
        return kStatus_LPUART_StopBitCountNotSupported;
    }
    else
    {
        return kStatus_Success;
    }
#endif
}

// See fsl_lpuart_hal.h for documentation of this function.
void lpuart_hal_configure_tx_rx_inversion(LPUART_Type *baseAddr, bool rxInvert, bool txInvert)
{
    // configure tx and rx inversions
    // 0 - receive data not inverted, 1 - receive data inverted
    LPUART_BWR_STAT_RXINV(baseAddr, rxInvert);
    // 0 - transmit data not inverted, 1 - transmit data inverted
    LPUART_BWR_CTRL_TXINV(baseAddr, txInvert);
}

#endif // BL_CONFIG_LPUART

////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////

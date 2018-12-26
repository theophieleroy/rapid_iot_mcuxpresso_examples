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
#include "uart/hal/fsl_uart_hal.h"
#include "fsl_uart_hal_unit_test.h"
#include "bootloader_common.h"
#include "fsl_device_registers.h"
#include "utilities/fsl_assert.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/

/* See fsl_uart_hal.h for documentation of this function. */
uint32_t uart_hal_test_get_baud_rate(uint32_t uartInstance, uint32_t sourceClockInHz)
{
    assert(uartInstance < UART_INSTANCE_COUNT);

    uint32_t sbr;
    uint32_t baudRate;

#if FSL_FEATURE_UART_HAS_LOW_POWER_UART_SUPPORT
    if (uartInstance == 0)
    {
        sbr = ((uint32_t)HW_UART0_BDH.B.SBR << 8);
        sbr |= ((uint32_t)HW_UART0_BDL.B.SBR);
        baudRate = sourceClockInHz / (sbr * (HW_UART0_C4.B.OSR + 1));
    }
    else
    {
        sbr = ((uint32_t)HW_UART_BDH(uartInstance).B.SBR << 8);
        sbr |= ((uint32_t)HW_UART_BDL(uartInstance).B.SBR);
        baudRate = sourceClockInHz / (sbr * 16);
    }
#else
    sbr = ((uint32_t)HW_UART_BDH(uartInstance).B.SBR << 8);
    sbr |= ((uint32_t)HW_UART_BDL(uartInstance).B.SBR);

    /* first calculate the divider value */
    /* note, a x32 multiplication factor is needed for sbr to account*/
    /* for the brfa register value, which has a x32 multiplication inherent in the value*/
    /* then for the final baud rate calculation, we will need to multiply by a factor of 32 (since*/
    /* the divider has a x32 multiplication factored into it).*/
    /* doing it this way avoids using float */
    uint32_t divider;
    uint8_t brfa;

    brfa = (uint8_t)HW_UART_C4(uartInstance).B.BRFA;
    divider = 16 * (32 * sbr + brfa);

    /* first calcuate numerator to avoid using float*/
    baudRate = (sourceClockInHz * 32) / divider;
#endif
    return baudRate;
}

/* See fsl_uart_hal.h for documentation of this function. */
uint32_t uart_hal_test_get_baud_rate_divisor(uint32_t uartInstance)
{
    assert(uartInstance < UART_INSTANCE_COUNT);

    uint32_t sbr;
#if FSL_FEATURE_UART_HAS_LOW_POWER_UART_SUPPORT
    if (uartInstance == 0)
    {
        sbr = ((uint32_t)HW_UART0_BDH.B.SBR << 8);
        sbr |= ((uint32_t)HW_UART0_BDL.B.SBR);
    }
    else
#endif
    {
        sbr = ((uint32_t)HW_UART_BDH(uartInstance).B.SBR << 8);
        sbr |= ((uint32_t)HW_UART_BDL(uartInstance).B.SBR);
    }

    return sbr;
}

#if FSL_FEATURE_UART_HAS_BAUD_RATE_FINE_ADJUST_SUPPORT
/* See fsl_uart_hal.h for documentation of this function. */
uint8_t uart_hal_test_get_baud_rate_fine_adjust(uint32_t uartInstance)
{
    assert(uartInstance < UART_INSTANCE_COUNT);

    return HW_UART_C4(uartInstance).B.BRFA;
}
#endif

#if FSL_FEATURE_UART_HAS_BAUD_RATE_OVER_SAMPLING_SUPPORT
/* See fsl_uart_hal.h for documentation of this function. */
uint8_t uart_hal_test_get_oversampling_ratio(uint32_t uartInstance)
{
    assert(uartInstance < UART_INSTANCE_COUNT);

    if (uartInstance == 0)
    {
        return HW_UART0_C4.B.OSR;
    }
    else
    {
        return 0xFF; /* return an unsupported value to indicate an unsupported feature*/
    }
}
#endif

#if FSL_FEATURE_UART_HAS_BOTH_EDGE_SAMPLING_SUPPORT
/* See fsl_uart_hal.h for documentation of this function. */
bool uart_hal_test_get_both_edge_sampling_setting(uint32_t uartInstance)
{
    assert(uartInstance < UART_INSTANCE_COUNT);

    if (uartInstance == 0)
    {
        return HW_UART0_C5.B.BOTHEDGE;
    }
    else
    {
        /* unsupported feature of this instance number, return 0*/
        return 0;
    }
}
#endif

/* See fsl_uart_hal.h for documentation of this function. */
uart_bit_count_per_char_t uart_hal_test_get_bit_count_per_char_config(uint32_t uartInstance)
{
    assert(uartInstance < UART_INSTANCE_COUNT);

#if FSL_FEATURE_UART_HAS_LOW_POWER_UART_SUPPORT
    if (uartInstance == 0)
    {
        if (HW_UART0_C4.B.M10 == 1)
        {
            return kUart10BitsPerChar;
        }
        else if (HW_UART0_C1.B.M == 1)
        {
            return kUart9BitsPerChar;
        }
        else
        {
            return kUart8BitsPerChar;
        }
    }
    else
#endif
    {
        if (HW_UART_C1(uartInstance).B.M == 1)
        {
            return kUart9BitsPerChar;
        }
        else
        {
            return kUart8BitsPerChar;
        }
    }
}

/* See fsl_uart_hal.h for documentation of this function. */
uart_parity_mode_t uart_hal_test_get_parity_mode_config(uint32_t uartInstance)
{
    assert(uartInstance < UART_INSTANCE_COUNT);

#if FSL_FEATURE_UART_HAS_LOW_POWER_UART_SUPPORT
    if (uartInstance == 0)
    {
        if (HW_UART0_C1.B.PE == 0)
        {
            return kUartParityDisabled; /* Parity is disable, parity type is don't care*/
        }
        else if (HW_UART0_C1.B.PT == 1)
        {
            return kUartParityOdd; /* Parity enable, set to odd*/
        }
        else
        {
            return kUartParityEven; /* Parity enable, set to even*/
        }
    }
    else
#endif
    {
        if (HW_UART_C1(uartInstance).B.PE == 0)
        {
            return kUartParityDisabled; /* Parity is disable, parity type is don't care*/
        }
        else if (HW_UART_C1(uartInstance).B.PT == 1)
        {
            return kUartParityOdd; /* Parity enable, set to odd*/
        }
        else
        {
            return kUartParityEven; /* Parity enable, set to even*/
        }
    }
}

/* See fsl_uart_hal.h for documentation of this function. */
uart_stop_bit_count_t uart_hal_test_get_stop_bit_count(uint32_t uartInstance)
{
    assert(uartInstance < UART_INSTANCE_COUNT);

#if FSL_FEATURE_UART_HAS_STOP_BIT_CONFIG_SUPPORT
    if (uartInstance == 0)
    {
        if (HW_UART0_BDH.B.SBNS == 1)
        {
            return kUartTwoStopBit;
        }
        else
        {
            return kUartOneStopBit;
        }
    }
    else /* UART instances 1 or 2*/
    {
        if (HW_UART_BDH(uartInstance).B.SBNS == 1)
        {
            return kUartTwoStopBit;
        }
        else
        {
            return kUartOneStopBit;
        }
    }
#else
    /* stop bit configuration not supported, only one stop bit is supported*/
    return kUartOneStopBit;
#endif
}

/* See fsl_uart_hal.h for documentation of this function. */
void uart_hal_test_get_tx_rx_inversion_config(uint32_t uartInstance, uint32_t *rxInvert, uint32_t *txInvert)
{
    assert(uartInstance < UART_INSTANCE_COUNT);

/* get tx and rx inversions configuration*/
#if FSL_FEATURE_UART_HAS_LOW_POWER_UART_SUPPORT
    if (uartInstance == 0)
    {
        /* 0 - receive data not inverted, 1 - receive data inverted */
        *rxInvert = (uint32_t)HW_UART0_S2.B.RXINV;
        /* 0 - transmit data not inverted, 1 - transmit data inverted*/
        *txInvert = (uint32_t)HW_UART0_C3.B.TXINV;
    }
    else
#endif
    {
        /* 0 - receive data not inverted, 1 - receive data inverted */
        *rxInvert = (uint32_t)HW_UART_S2(uartInstance).B.RXINV;
        /* 0 - transmit data not inverted, 1 - transmit data inverted */
        *txInvert = (uint32_t)HW_UART_C3(uartInstance).B.TXINV;
    }
}

/* See fsl_uart_hal.h for documentation of this function. */
bool uart_hal_test_is_loopback_operation_config(uint32_t uartInstance)
{
    assert(uartInstance < UART_INSTANCE_COUNT);

/* get configuration of uart in loopback mode or not*/
#if FSL_FEATURE_UART_HAS_LOW_POWER_UART_SUPPORT
    if (uartInstance == 0)
    {
        if (HW_UART0_C1.B.LOOPS == 0)
        {
            return 0; /* loopback mode disabled*/
        }
        else
        {
            if (HW_UART0_C1.B.RSRC == 1)
            {
                return 0; /* set for singlewire mode, not loopback mode*/
            }
            else
            {
                return 1; /* loopback mode enabled*/
            }
        }
    }
    else
#endif
    {
        /* get configuration of uart in loopback mode or not*/
        if (HW_UART_C1(uartInstance).B.LOOPS == 0)
        {
            return 0; /* loopback mode disabled*/
        }
        else
        {
            if (HW_UART_C1(uartInstance).B.RSRC == 1)
            {
                return 0; /* set for singlewire mode, not loopback mode*/
            }
            else
            {
                return 1; /* loopback mode enabled*/
            }
        }
    }
}

/* See fsl_uart_hal.h for documentation of this function. */
bool uart_hal_test_is_singlewire_operation_config(uint32_t uartInstance)
{
    assert(uartInstance < UART_INSTANCE_COUNT);

/* get configuration of uart in singlewire mode or not*/
#if FSL_FEATURE_UART_HAS_LOW_POWER_UART_SUPPORT
    if (uartInstance == 0)
    {
        if (HW_UART0_C1.B.LOOPS == 0)
        {
            return 0; /* loopback/singlewire mode disabled*/
        }
        else
        {
            if (HW_UART0_C1.B.RSRC == 1)
            {
                return 1; /* set for singlewire mode, not loopback mode*/
            }
            else
            {
                return 0; /* loopback mode enabled, not singlewire*/
            }
        }
    }
    else
#endif
    {
        /* get configuration of uart in singlewire mode or not*/
        if (HW_UART_C1(uartInstance).B.LOOPS == 0)
        {
            return 0; /* loopback/singlewire mode disabled*/
        }
        else
        {
            if (HW_UART_C1(uartInstance).B.RSRC == 1)
            {
                return 1; /* set for singlewire mode, not loopback mode*/
            }
            else
            {
                return 0; /* loopback mode enabled, not singlewire*/
            }
        }
    }
}

/* See fsl_uart_hal.h for documentation of this function. */
uart_singlewire_txdir_t uart_hal_test_get_txdir_config(uint32_t uartInstance)
{
    assert(uartInstance < UART_INSTANCE_COUNT);

/* get configuration of uart transmit direction bit (input or output) */
#if FSL_FEATURE_UART_HAS_LOW_POWER_UART_SUPPORT
    if (uartInstance == 0)
    {
        if (HW_UART0_C3.B.TXDIR == 1)
        {
            return kUartSinglewireTxdirOut;
        }
        else
        {
            return kUartSinglewireTxdirIn;
        }
    }
    else
#endif
    {
        if (HW_UART_C3(uartInstance).B.TXDIR == 1)
        {
            return kUartSinglewireTxdirOut;
        }
        else
        {
            return kUartSinglewireTxdirIn;
        }
    }
}

/* See fsl_uart_hal.h for documentation of this function. */
void uart_hal_test_get_idle_line_detect_config(uint32_t uartInstance, uart_idle_line_config_t *getConfig)
{
    assert(uartInstance < UART_INSTANCE_COUNT);

/* Configure the idle line detection configuration as follows: */
/* configure the ILT to bit count after start bit or stop bit*/
/* configure RWUID to set or not set IDLE status bit upon detection of an idle character*/
/* when recevier in standby*/
#if FSL_FEATURE_UART_HAS_LOW_POWER_UART_SUPPORT
    if (uartInstance == 0)
    {
        getConfig->idleLineType = HW_UART0_C1.B.ILT;
        getConfig->rxWakeIdleDetect = HW_UART0_S2.B.RWUID;
    }
    else
#endif
    {
        getConfig->idleLineType = HW_UART_C1(uartInstance).B.ILT;
        getConfig->rxWakeIdleDetect = HW_UART_S2(uartInstance).B.RWUID;
    }
}

/* See fsl_uart_hal.h for documentation of this function. */
uart_break_char_length_t uart_hal_test_get_break_char_transmit_length(uint32_t uartInstance)
{
    assert(uartInstance < UART_INSTANCE_COUNT);

/* Get configuration of BRK13 - Break Character transmit length configuration*/
/* UART break character length setting: */
/* 0 - minimum 10-bit times (default), */
/* 1 - minimum 13-bit times*/
#if FSL_FEATURE_UART_HAS_LOW_POWER_UART_SUPPORT
    if (uartInstance == 0)
    {
        if (HW_UART0_S2.B.BRK13 == 1)
        {
            return kUartBreakChar13BitMinimum;
        }
        else
        {
            return kUartBreakChar10BitMinimum;
        }
    }
    else
#endif
    {
        if (HW_UART_S2(uartInstance).B.BRK13 == 1)
        {
            return kUartBreakChar13BitMinimum;
        }
        else
        {
            return kUartBreakChar10BitMinimum;
        }
    }
}

/* See fsl_uart_hal.h for documentation of this function. */
uart_break_char_length_t uart_hal_test_get_break_char_detect_length(uint32_t uartInstance)
{
    assert(uartInstance < UART_INSTANCE_COUNT);

/* Configure LBKDE - Break Character detect length configuration*/
/* UART break character length setting: */
/* 0 - minimum 10-bit times (default), */
/* 1 - minimum 13-bit times*/
#if FSL_FEATURE_UART_HAS_LOW_POWER_UART_SUPPORT
    if (uartInstance == 0)
    {
        if (HW_UART0_S2.B.LBKDE == 1)
        {
            return kUartBreakChar13BitMinimum;
        }
        else
        {
            return kUartBreakChar10BitMinimum;
        }
    }
    else
#endif
    {
        if (HW_UART_S2(uartInstance).B.LBKDE == 1)
        {
            return kUartBreakChar13BitMinimum;
        }
        else
        {
            return kUartBreakChar10BitMinimum;
        }
    }
}

/* See fsl_uart_hal.h for documentation of this function. */
bool uart_hal_test_is_send_break_set(uint32_t uartInstance)
{
    assert(uartInstance < UART_INSTANCE_COUNT);

/* Get configuration of SBK - Send Break*/
/* UART send break character setting: */
/* 0 - normal transmitter operation, */
/* 1 - Queue break character(s) to be sent*/
#if FSL_FEATURE_UART_HAS_LOW_POWER_UART_SUPPORT
    if (uartInstance == 0)
    {
        return HW_UART0_C2.B.SBK;
    }
    else
#endif
    {
        return HW_UART_C2(uartInstance).B.SBK;
    }
}

/* See fsl_uart_hal.h for documentation of this function. */
bool uart_hal_test_is_match_address_mode2_enabled(uint32_t uartInstance)
{
    assert(uartInstance < UART_INSTANCE_COUNT);

#if FSL_FEATURE_UART_HAS_LOW_POWER_UART_SUPPORT
    if (uartInstance == 0)
    {
        return ((uint32_t)UART0_RD_C4 >> 6) & 0x1;
    }
    else /* UART instances 1 or 2*/
    {
        /* unsupported feature of this instance number, return 0*/
        return 0;
    }
#else
    return ((uint32_t)UART_RD_C4(uartInstance) >> 6) & 0x1;
#endif
}

/* See fsl_uart_hal.h for documentation of this function. */
bool uart_hal_test_is_match_address_mode1_enabled(uint32_t uartInstance)
{
    assert(uartInstance < UART_INSTANCE_COUNT);

#if FSL_FEATURE_UART_HAS_LOW_POWER_UART_SUPPORT
    if (uartInstance == 0)
    {
        return ((uint32_t)UART0_RD_C4 >> 7) & 0x1;
    }
    else /* UART instances 1 or 2*/
    {
        /* unsupported feature of this instance number, return 0*/
        return 0;
    }
#else
    return ((uint32_t)UART_RD_C4(uartInstance) >> 7) & 0x1;
#endif
}

uint8_t uart_hal_test_get_match_address_register(uint32_t uartInstance, uint32_t maRegister)
{
    assert(uartInstance < UART_INSTANCE_COUNT);

#if FSL_FEATURE_UART_HAS_LOW_POWER_UART_SUPPORT
    if (uartInstance == 0)
    {
        if (maRegister == 1)
        {
            return UART0_RD_MA1;
        }
        else if (maRegister == 2)
        {
            return UART0_RD_MA2;
        }
        else
        {
            /* unsupported register, return 0*/
            return 0;
        }
    }
    else /* UART instances 1 or 2*/
    {
        /* unsupported feature of this instance number, return 0*/
        return 0;
    }
#else
    if (maRegister == 1)
    {
        return UART_RD_MA1(uartInstance);
    }
    else if (maRegister == 2)
    {
        return UART_RD_MA2(uartInstance);
    }
    else
    {
        /* unsupported register, return 0*/
        return 0;
    }
#endif
}

/* See fsl_uart_hal.h for documentation of this function. */
bool uart_hal_test_is_send_msb_first_operation_enabled(uint32_t uartInstance)
{
    assert(uartInstance < UART_INSTANCE_COUNT);

#if FSL_FEATURE_UART_HAS_LOW_POWER_UART_SUPPORT
    if (uartInstance == 0)
    {
        /* If this bit is set, UART reverses the order of the bits that are */
        /* transmitted and received on the wire.*/
        return HW_UART0_S2.B.MSBF;
    }
    else
    {
        /* unsupported feature of this instance number, return 0*/
        return 0;
    }
#else
    /* If this bit is set, UART reverses the order of the bits that */
    /* are transmitted and received on the wire.*/
    return HW_UART_S2(uartInstance).B.MSBF;
#endif
}

/* See fsl_uart_hal.h for documentation of this function. */
bool uart_hal_test_is_receive_resync_disabled(uint32_t uartInstance)
{
    assert(uartInstance < UART_INSTANCE_COUNT);

#if FSL_FEATURE_UART_HAS_RX_RESYNC_SUPPORT
    if (uartInstance == 0)
    {
        /* When set, disables the resynchronization of the received data word when a data one */
        /* followed by data zero transition is detected. */
        return HW_UART0_C5.B.RESYNCDIS;
    }
    else
#endif
    {
        /* unsupported feature of this instance number, return 0*/
        return 0;
    }
}

#if FSL_FEATURE_UART_HAS_MODEM_SUPPORT
/* See fsl_uart_hal.h for documentation of this function. */
bool uart_hal_test_is_receiver_rts_enabled(uint32_t uartInstance)
{
    assert(uartInstance < UART_INSTANCE_COUNT);

    return HW_UART_MODEM(uartInstance).B.RXRTSE;
}

/* See fsl_uart_hal.h for documentation of this function. */
bool uart_hal_test_is_transmitter_rts_enabled(uint32_t uartInstance)
{
    assert(uartInstance < UART_INSTANCE_COUNT);

    return HW_UART_MODEM(uartInstance).B.TXRTSE;
}

/* See fsl_uart_hal.h for documentation of this function. */
bool uart_hal_test_get_transmitter_rts_polarity_config(uint32_t uartInstance)
{
    assert(uartInstance < UART_INSTANCE_COUNT);

    /* Return the configuration of the transmitter rts polarity: 0=active low, 1=active high*/
    return HW_UART_MODEM(uartInstance).B.TXRTSPOL;
}

/* See fsl_uart_hal.h for documentation of this function. */
bool uart_hal_test_is_transmitter_cts_enabled(uint32_t uartInstance)
{
    assert(uartInstance < UART_INSTANCE_COUNT);

    return HW_UART_MODEM(uartInstance).B.TXCTSE;
}
#endif /* FSL_FEATURE_UART_MODEM_SUPPORT*/

#if FSL_FEATURE_UART_HAS_IR_SUPPORT
/* See fsl_uart_hal.h for documentation of this function. */
bool uart_hal_test_is_infrared_enabled(uint32_t uartInstance)
{
    assert(uartInstance < UART_INSTANCE_COUNT);

    return HW_UART_IR(uartInstance).B.IREN;
}

/* See fsl_uart_hal.h for documentation of this function. */
uart_ir_tx_pulsewidth_t uart_hal_test_get_infrared_pulse_config(uint32_t uartInstance)
{
    assert(uartInstance < UART_INSTANCE_COUNT);

    if (HW_UART_IR(uartInstance).B.TNP == 0x0)
    {
        return kUartIrThreeSixteenthsWidth;
    }
    else if (HW_UART_IR(uartInstance).B.TNP == 0x1)
    {
        return kUartIrOneSixteenthWidth;
    }
    else if (HW_UART_IR(uartInstance).B.TNP == 0x2)
    {
        return kUartIrOneThirtysecondsWidth;
    }
    else
    {
        return kUartIrOneFourthWidth;
    }
}
#endif /* FSL_FEATURE_UART_IR_SUPPORT*/

#if FSL_FEATURE_UART_HAS_FIFO
/* See fsl_uart_hal.h for documentation of this function. */
bool uart_hal_test_is_tx_fifo_enabled(uint32_t uartInstance)
{
    assert(uartInstance < UART_INSTANCE_COUNT);

    return HW_UART_PFIFO(uartInstance).B.TXFE;
}

/* See fsl_uart_hal.h for documentation of this function. */
bool uart_hal_test_is_rx_fifo_enabled(uint32_t uartInstance)
{
    assert(uartInstance < UART_INSTANCE_COUNT);

    return HW_UART_PFIFO(uartInstance).B.RXFE;
}
#endif /* FSL_FEATURE_UART_HAS_FIFO*/

/*******************************************************************************
 * EOF
 ******************************************************************************/

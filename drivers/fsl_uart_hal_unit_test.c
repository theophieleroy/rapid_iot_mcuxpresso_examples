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
#include "bootloader_common.h"
#include "fsl_device_registers.h"
#include "utilities/fsl_assert.h"
#include "fsl_uart_hal_unit_test.h"
#include <stdlib.h>
#include <string.h>
#include "clock/fsl_clock_manager.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static void uart_configure_tx_rx_enable(uint32_t uartInstance, bool enable);

/*******************************************************************************
 * Variables
 ******************************************************************************/
/* save off various instances of this struct  */
static uart_test_status_t s_uartTestStatus[UART_INSTANCE_COUNT];

#if FSL_FEATURE_UART_HAS_FIFO
/* variables for getting fifo sizes*/
static uint8_t txFifoSize[UART_INSTANCE_COUNT] = {0};
static uint8_t rxFifoSize[UART_INSTANCE_COUNT] = {0};
#endif

uint32_t g_failCount;

/*******************************************************************************
 * Code
 ******************************************************************************/
size_t __write(int handle, const unsigned char *buf, size_t size);

void main(void)
{
    uint32_t uartInstance;
    uint32_t uartInstanceCount;

    uartInstanceCount = UART_INSTANCE_COUNT;

/*******************************************************************************
 * The following initializes the hardware such as clock gates and if
 * applicable clock configuration and io muxing
 * This is very specific to the hardware itself - the MCU and if applicable
 * the development board (Freedom or Tower).
 * As such, we need to set up the hardware based specifically on the MCU being
 * used. For future MCUs, we will need to add the specific hardware configuration.
 ******************************************************************************/

/* For KL25, the hal test is linked to flash and cannot be linked to the platform library.
 * As such, we won't be able to use the clock_manager code and will have to manually
 * ungate the clock.
 */
#if (defined(CPU_MKL25Z32VFM4) || defined(CPU_MKL25Z64VFM4) || defined(CPU_MKL25Z128VFM4) || \
     defined(CPU_MKL25Z32VFT4) || defined(CPU_MKL25Z64VFT4) || defined(CPU_MKL25Z128VFT4) || \
     defined(CPU_MKL25Z32VLH4) || defined(CPU_MKL25Z64VLH4) || defined(CPU_MKL25Z128VLH4) || \
     defined(CPU_MKL25Z32VLK4) || defined(CPU_MKL25Z64VLK4) || defined(CPU_MKL25Z128VLK4))
    /* first, need to enable clocks to the uart modules, else cannot access registers*/
    /* SoC set up like IOMUX, clocks, etc*/
    init_hardware();
    /* ungate clocks to the UART modules*/
    SIM_SET_SCGC4(SIM_SCGC4_UART0_MASK | SIM_SCGC4_UART1_MASK | SIM_SCGC4_UART2_MASK);
/* For other chips that are built ot run from RAM, we'll use the clock manager to
 * ungate the clocks as we can link the platform library
 */
#else
    /* ungate uart module clock*/
    uint32_t i;
    for (i = 0; i < UART_INSTANCE_COUNT; i++)
    {
        clock_manager_set_gate(kClockModuleUART, i, true);
    }
#endif

    for (uartInstance = 0; uartInstance < uartInstanceCount; uartInstance++)
    {
        /* Clear the test status struct for this (and each) instance.*/
        memset(&s_uartTestStatus[uartInstance], 0, sizeof(uart_test_status_t));

        /**********************************************/
        /* Test enabling and disabling transmitter and receiver*/
        /**********************************************/
        uart_hal_enable_transmitter(uartInstance);
        uart_hal_enable_receiver(uartInstance);
        if (uart_hal_is_transmitter_enabled(uartInstance) != 1)
        {
            s_uartTestStatus[uartInstance].B.testStatusTxEnable = 1;
        }
        if (uart_hal_is_receiver_enabled(uartInstance) != 1)
        {
            s_uartTestStatus[uartInstance].B.testStatusRxEnable = 1;
        }

        /* wait till TX Complete flag set to make sure we can disable transmitter*/
        /* other transmitter might stay enabled, as found during testing*/
        while (!uart_hal_is_transmission_complete(uartInstance))
        {
        }

        uart_hal_disable_transmitter(uartInstance);
        uart_hal_disable_receiver(uartInstance);
        if (uart_hal_is_transmitter_enabled(uartInstance) != 0)
        {
            s_uartTestStatus[uartInstance].B.testStatusTxEnable = 1;
        }
        if (uart_hal_is_receiver_enabled(uartInstance) != 0)
        {
            s_uartTestStatus[uartInstance].B.testStatusRxEnable = 1;
        }

        /**********************************************/
        /* Test Stop bit count*/
        /**********************************************/
        uart_configure_tx_rx_enable(uartInstance, 0);

        /* configure stop bit count to kUartTwoStopBit, and check the status returned*/
        /* should get an error for devices that don't support stop bit config*/
        if (uart_hal_configure_stop_bit_count(uartInstance, kUartTwoStopBit) != kStatus_Success)
        {
#if FSL_FEATURE_UART_HAS_STOP_BIT_CONFIG_SUPPORT
            /* should not get an error on uarts that support stop bit config*/
            s_uartTestStatus[uartInstance].B.testStatusSBNS = 1;
#endif
        }
        else
        {
            /* check to see if stop bit configured to kUartTwoStopBit*/
            if (uart_hal_test_get_stop_bit_count(uartInstance) != kUartTwoStopBit)
            {
                s_uartTestStatus[uartInstance].B.testStatusSBNS = 1;
            }
        }

        /* configure stop bit count to kUartOneStopBit, and check the status returned, */
        /* should not get an error since all uarts support one stop bit*/
        if (uart_hal_configure_stop_bit_count(uartInstance, kUartOneStopBit) != kStatus_Success)
        {
            /* should not get an error*/
            s_uartTestStatus[uartInstance].B.testStatusSBNS = 1;
        }
        else
        {
            /* check to see if stop bit configured to kUartOneStopBit*/
            if (uart_hal_test_get_stop_bit_count(uartInstance) != kUartOneStopBit)
            {
                s_uartTestStatus[uartInstance].B.testStatusSBNS = 1;
            }
        }

        uart_configure_tx_rx_enable(uartInstance, 1);

        /**********************************************/
        /* Test Rx active edge interrupt enable*/
        /**********************************************/
        /* set the bit*/
        uart_hal_enable_rx_active_edge_interrupt(uartInstance);
        /* check to see if set*/
        if (uart_hal_is_rx_active_edge_interrupt_enabled(uartInstance != 1))
        {
            s_uartTestStatus[uartInstance].B.testStatusRXEDGIE = 1;
        }
        /* clear the bit*/
        uart_hal_disable_rx_active_edge_interrupt(uartInstance);
        /* check to see if cleared*/
        if (uart_hal_is_rx_active_edge_interrupt_enabled(uartInstance) != 0)
        {
            s_uartTestStatus[uartInstance].B.testStatusRXEDGIE = 1;
        }

        /**********************************************/
        /* Test break detect interrupt enable*/
        /**********************************************/
        /* set the bit*/
        uart_hal_enable_break_detect_interrupt(uartInstance);
        /* check to see if set*/
        if (uart_hal_is_break_detect_interrupt_enabled(uartInstance) != 1)
        {
            s_uartTestStatus[uartInstance].B.testStatusLBKDIE = 1;
        }
        /* clear the bit*/
        uart_hal_disable_break_detect_interrupt(uartInstance);
        /* check to see if cleared*/
        if (uart_hal_is_break_detect_interrupt_enabled(uartInstance) != 0)
        {
            s_uartTestStatus[uartInstance].B.testStatusLBKDIE = 1;
        }

        /**********************************************/
        /* Test parity mode */
        /**********************************************/
        uart_configure_tx_rx_enable(uartInstance, 0);

        /* set the PE and PT bits, kUartParityEven*/
        uart_hal_configure_parity_mode(uartInstance, kUartParityOdd);
        if (uart_hal_test_get_parity_mode_config(uartInstance) != kUartParityOdd)
        {
            s_uartTestStatus[uartInstance].B.testStatusParityConfig = 1;
        }

        /* clear the PT bit, but keep PE set, kUartParityEven*/
        uart_hal_configure_parity_mode(uartInstance, kUartParityEven);
        /* check to see if PT cleared*/
        if (uart_hal_test_get_parity_mode_config(uartInstance) != kUartParityEven)
        {
            s_uartTestStatus[uartInstance].B.testStatusParityConfig = 1;
        }

        /* clear the PE and PT bits, kUartParityDisabled*/
        uart_hal_configure_parity_mode(uartInstance, kUartParityDisabled);
        /* check to see if PE and PT cleared*/
        if (uart_hal_test_get_parity_mode_config(uartInstance) != kUartParityDisabled)
        {
            s_uartTestStatus[uartInstance].B.testStatusParityConfig = 1;
        }

        uart_configure_tx_rx_enable(uartInstance, 1);

        /**********************************************/
        /* Test idle line select config*/
        /**********************************************/
        uart_idle_line_config_t idleLineGetConfig;
        uart_idle_line_config_t idleLineSetConfig;

        uart_configure_tx_rx_enable(uartInstance, 0);

        /* set the config bits*/
        idleLineSetConfig.idleLineType = 1;
        idleLineSetConfig.rxWakeIdleDetect = 1;
        uart_hal_configure_idle_line_detect(uartInstance, &idleLineSetConfig);
        uart_hal_test_get_idle_line_detect_config(uartInstance, &idleLineGetConfig);

        /* check to see if config bits set*/
        if ((idleLineGetConfig.idleLineType != 1) || (idleLineGetConfig.rxWakeIdleDetect != 1))
        {
            s_uartTestStatus[uartInstance].B.testStatusIdleLineDetectConfig = 1;
        }

        /* clear the config bits*/
        idleLineSetConfig.idleLineType = 0;
        idleLineSetConfig.rxWakeIdleDetect = 0;
        uart_hal_configure_idle_line_detect(uartInstance, &idleLineSetConfig);
        uart_hal_test_get_idle_line_detect_config(uartInstance, &idleLineGetConfig);

        /* check to see if config bits clear*/
        if ((idleLineGetConfig.idleLineType != 0) || (idleLineGetConfig.rxWakeIdleDetect != 0))
        {
            s_uartTestStatus[uartInstance].B.testStatusIdleLineDetectConfig = 1;
        }

        uart_configure_tx_rx_enable(uartInstance, 1);

        /**********************************************/
        /* Test WAKE - receiver wake up select*/
        /**********************************************/
        /* set receiver wake for kUartAddrMarkWake*/
        uart_hal_select_receiver_wakeup_method(uartInstance, kUartAddrMarkWake);
        if (uart_hal_get_receiver_wakeup_method(uartInstance) != kUartAddrMarkWake)
        {
            s_uartTestStatus[uartInstance].B.testStatusWAKE = 1;
        }

        /* set receiver wake for kUartIdleLineWake*/
        uart_hal_select_receiver_wakeup_method(uartInstance, kUartIdleLineWake);
        if (uart_hal_get_receiver_wakeup_method(uartInstance) != kUartIdleLineWake)
        {
            s_uartTestStatus[uartInstance].B.testStatusWAKE = 1;
        }

        /**********************************************/
        /* Test bit count per char*/
        /**********************************************/
        uart_configure_tx_rx_enable(uartInstance, 0);

        /* set for kUart10BitsPerChar, not supported for all uart instances*/
        if (uart_hal_configure_bit_count_per_char(uartInstance, kUart10BitsPerChar) == kStatus_Success)
        {
/* only applicable in supported uart instances */
#if FSL_FEATURE_UART_HAS_10BIT_DATA_SUPPORT
            if ((uart_hal_test_get_bit_count_per_char_config(uartInstance) != kUart10BitsPerChar) &&
                (uartInstance == 0))
            {
                s_uartTestStatus[uartInstance].B.testStatusBitCount = 1;
            }
#endif
            /* if other uart instances return kStatus_Success this is an error since */
            /* these do not support 10-bit mode*/
            if (uartInstance != 0)
            {
                s_uartTestStatus[uartInstance].B.testStatusBitCount = 1;
            }
        }
        else
        {
/* in this case, non-supported uart instances should return an error*/
/* But if supported uart instances returned an error, this is a failure*/
#if FSL_FEATURE_UART_HAS_10BIT_DATA_SUPPORT
            if (uartInstance == 0)
            {
                s_uartTestStatus[uartInstance].B.testStatusBitCount = 1;
            }
#endif
        }

        /* set for kUart9BitsPerChar*/
        if (uart_hal_configure_bit_count_per_char(uartInstance, kUart9BitsPerChar) == kStatus_Success)
        {
            if (uart_hal_test_get_bit_count_per_char_config(uartInstance) != kUart9BitsPerChar)
            {
                s_uartTestStatus[uartInstance].B.testStatusBitCount = 1;
            }
        }
        else
        {
            s_uartTestStatus[uartInstance].B.testStatusBitCount = 1;
        }

        /* set for kUart8BitsPerChar*/
        if (uart_hal_configure_bit_count_per_char(uartInstance, kUart8BitsPerChar) == kStatus_Success)
        {
            if (uart_hal_test_get_bit_count_per_char_config(uartInstance) != kUart8BitsPerChar)
            {
                s_uartTestStatus[uartInstance].B.testStatusBitCount = 1;
            }
        }
        else
        {
            s_uartTestStatus[uartInstance].B.testStatusBitCount = 1;
        }

        uart_configure_tx_rx_enable(uartInstance, 1);

        /**********************************************/
        /* Test loopback mode*/
        /**********************************************/
        uart_configure_tx_rx_enable(uartInstance, 0);

        /* configure/enable loopback mode*/
        uart_hal_configure_loopback_mode(uartInstance, 1);
        if (uart_hal_test_is_loopback_operation_config(uartInstance) != 1)
        {
            s_uartTestStatus[uartInstance].B.testStatusLoopbackMode = 1;
        }

        /* disable loopback mode*/
        uart_hal_configure_loopback_mode(uartInstance, 0);
        if (uart_hal_test_is_loopback_operation_config(uartInstance) != 0)
        {
            s_uartTestStatus[uartInstance].B.testStatusLoopbackMode = 1;
        }

        uart_configure_tx_rx_enable(uartInstance, 1);

        /**********************************************/
        /* Test wait mode operation*/
        /**********************************************/
        uart_configure_tx_rx_enable(uartInstance, 0);

        /* configure wait mode, kUartStops in wait mode*/
        uart_hal_configure_wait_mode_operation(uartInstance, kUartStops);
        if (uart_hal_get_wait_mode_operation_config(uartInstance) != kUartStops)
        {
            s_uartTestStatus[uartInstance].B.testStatusWaitMode = 1;
        }

        /* configure wait mode, kUartOperates in wait mode*/
        uart_hal_configure_wait_mode_operation(uartInstance, kUartOperates);
        if (uart_hal_get_wait_mode_operation_config(uartInstance) != kUartOperates)
        {
            s_uartTestStatus[uartInstance].B.testStatusWaitMode = 1;
        }

        uart_configure_tx_rx_enable(uartInstance, 1);

        /**********************************************/
        /* Test put receiver in standby or normal (not standby) mode*/
        /**********************************************/
        /* try placing receiver in standby mode*/
        /* Some uart instances warn against placing the receiver in standby if wake detect is set */
        /* for idle and the receiver's current state is idle*/
        if (uart_hal_put_receiver_in_standby_mode(uartInstance) == kStatus_UART_RxStandbyModeError)
        {
            /* At this point UARTx_C1[WAKE] is cleared indicating wake on idle and the*/
            /* receiver is in the idle state. To place receiver in standby, set */
            /* UARTx_C1[WAKE] to wake on address-mark wakeup*/
            uart_hal_select_receiver_wakeup_method(uartInstance, kUartAddrMarkWake);
            /* now try placing receiver in standby mode again, if it returns error, */
            /* set test status flag*/
            if (uart_hal_put_receiver_in_standby_mode(uartInstance) == kStatus_UART_RxStandbyModeError)
            {
                s_uartTestStatus[uartInstance].B.testStatusRxWakeControl = 1;
            }
            /* set wake back to idle wake*/
            uart_hal_select_receiver_wakeup_method(uartInstance, kUartIdleLineWake);
        }

        if (uart_hal_is_receiver_in_standby(uartInstance) != 1)
        {
            s_uartTestStatus[uartInstance].B.testStatusRxWakeControl = 1;
        }

        uart_hal_put_receiver_in_normal_mode(uartInstance);
        if (uart_hal_is_receiver_in_standby(uartInstance) != 0)
        {
            s_uartTestStatus[uartInstance].B.testStatusRxWakeControl = 0;
        }

        /**********************************************/
        /* Test enable/disable idle_line_interrupt*/
        /**********************************************/
        uart_hal_enable_idle_line_interrupt(uartInstance);
        if (uart_hal_is_idle_line_interrupt_enabled(uartInstance) != 1)
        {
            s_uartTestStatus[uartInstance].B.testStatusILIE = 1;
        }

        uart_hal_disable_idle_line_interrupt(uartInstance);
        if (uart_hal_is_idle_line_interrupt_enabled(uartInstance) != 0)
        {
            s_uartTestStatus[uartInstance].B.testStatusILIE = 1;
        }

        /**********************************************/
        /* Test enable/disable rx_data_register_full_interrupt*/
        /**********************************************/
        uart_hal_enable_rx_data_register_full_interrupt(uartInstance);
        if (uart_hal_is_receive_data_full_interrupt_enabled(uartInstance) != 1)
        {
            s_uartTestStatus[uartInstance].B.testStatusRIE = 1;
        }

        uart_hal_disable_rx_data_register_full_interrupt(uartInstance);
        if (uart_hal_is_receive_data_full_interrupt_enabled(uartInstance) != 0)
        {
            s_uartTestStatus[uartInstance].B.testStatusRIE = 1;
        }

        /**********************************************/
        /* Test enable/disable transmission_complete_interrupt*/
        /**********************************************/
        uart_hal_enable_transmission_complete_interrupt(uartInstance);
        if (uart_hal_is_transmission_complete_interrupt_enabled(uartInstance) != 1)
        {
            s_uartTestStatus[uartInstance].B.testStatusTCIE = 1;
        }

        uart_hal_disable_transmission_complete_interrupt(uartInstance);
        if (uart_hal_is_transmission_complete_interrupt_enabled(uartInstance) != 0)
        {
            s_uartTestStatus[uartInstance].B.testStatusTCIE = 1;
        }

        /**********************************************/
        /* Test enable/disable tx_data_register_empty_interrupt*/
        /**********************************************/
        uart_hal_enable_tx_data_register_empty_interrupt(uartInstance);
        if (uart_hal_is_tx_data_register_empty_interrupt_enabled(uartInstance) != 1)
        {
            s_uartTestStatus[uartInstance].B.testStatusTIE = 1;
        }

        uart_hal_disable_tx_data_register_empty_interrupt(uartInstance);
        if (uart_hal_is_tx_data_register_empty_interrupt_enabled(uartInstance) != 0)
        {
            s_uartTestStatus[uartInstance].B.testStatusTIE = 1;
        }

        /**********************************************/
        /* Test msb enable/disable (applicable to some uart instances) */
        /**********************************************/
        uart_configure_tx_rx_enable(uartInstance, 0);

        /* test msb enable*/
        if (uart_hal_configure_send_msb_first_operation(uartInstance, 1) == kStatus_Success)
        {
#if FSL_FEATURE_UART_HAS_LOW_POWER_UART_SUPPORT
            /* for KL25Z, only uart0 (low power uart) supports MSB first operation, */
            /* so check for this instance*/
            if ((uart_hal_test_is_send_msb_first_operation_enabled(uartInstance) != 1) && (uartInstance == 0))
#else
            /* MSB first operation is supported on all uart instances of this SoC*/
            if ((uart_hal_test_is_send_msb_first_operation_enabled(uartInstance) != 1))
#endif
            {
                /* set flag since msb bit should be set*/
                s_uartTestStatus[uartInstance].B.testStatusMSBF = 1;
            }
#if FSL_FEATURE_UART_HAS_LOW_POWER_UART_SUPPORT
            /* non-supported uart instances should not return a success*/
            if (uartInstance != 0)
            {
                /* if uart1 or uart2 return kStatus_Success this is an error since these do not*/
                /* support msb first mode*/
                s_uartTestStatus[uartInstance].B.testStatusMSBF = 1;
            }
#endif
        }
        else
        {
#if FSL_FEATURE_UART_HAS_LOW_POWER_UART_SUPPORT
            /* in this case, uart1 and uart2 should return an error*/
            /* But if uart0 returned an error, this is a failure*/
            if (uartInstance == 0)
#endif
            /* if MSB first is supported, but an error was returned, set status flag*/
            {
                s_uartTestStatus[uartInstance].B.testStatusMSBF = 1;
            }
        }

        /* test msb disable*/
        if (uart_hal_configure_send_msb_first_operation(uartInstance, 0) == kStatus_Success)
        {
#if FSL_FEATURE_UART_HAS_LOW_POWER_UART_SUPPORT
            /* for KL25Z, only uart0 (low power uart) supports MSB first operation, */
            /* so check for this instance*/
            if ((uart_hal_test_is_send_msb_first_operation_enabled(uartInstance) != 0) && (uartInstance == 0))
#else
            /* MSB first operation is supported on all uart instances of this SoC*/
            if ((uart_hal_test_is_send_msb_first_operation_enabled(uartInstance) != 0))
#endif
            {
                /* set flag since msb bit should be cleared*/
                s_uartTestStatus[uartInstance].B.testStatusMSBF = 1;
            }
#if FSL_FEATURE_UART_HAS_LOW_POWER_UART_SUPPORT
            /* non-supported uart instances should not return a success*/
            if (uartInstance != 0)
            {
                /* if uart1 or uart2 return kStatus_Success this is an error since these do*/
                /* not support msb first mode*/
                s_uartTestStatus[uartInstance].B.testStatusMSBF = 1;
            }
#endif
        }
        else
        {
#if FSL_FEATURE_UART_HAS_LOW_POWER_UART_SUPPORT
            /* in this case, uart1 and uart2 should return an error*/
            /* But if uart0 returned an error, this is a failure*/
            if (uartInstance == 0)
#endif
            /* if MSB first is supported, but an error was returned, set status flag*/
            {
                s_uartTestStatus[uartInstance].B.testStatusMSBF = 1;
            }
        }

        uart_configure_tx_rx_enable(uartInstance, 1);

        /**********************************************/
        /* Test tx and rx data inversion*/
        /**********************************************/
        uint32_t rxInvert = 0;
        uint32_t txInvert = 0;
        /* invert tx and rx*/
        uart_hal_configure_tx_rx_inversion(uartInstance, 1, 1);
        /* get the invert state of the rx and tx invert bits*/
        uart_hal_test_get_tx_rx_inversion_config(uartInstance, &rxInvert, &txInvert);
        if (rxInvert != 1)
        {
            s_uartTestStatus[uartInstance].B.testStatusRXINV = 1;
        }
        if (txInvert != 1)
        {
            s_uartTestStatus[uartInstance].B.testStatusTXINV = 1;
        }

        /* disable invert tx and rx*/
        uart_hal_configure_tx_rx_inversion(uartInstance, 0, 0);
        /* get the invert state of the rx and tx invert bits*/
        uart_hal_test_get_tx_rx_inversion_config(uartInstance, &rxInvert, &txInvert);
        if (rxInvert != 0)
        {
            s_uartTestStatus[uartInstance].B.testStatusRXINV = 1;
        }
        if (txInvert != 0)
        {
            s_uartTestStatus[uartInstance].B.testStatusTXINV = 1;
        }

        /**********************************************/
        /* Test LIN Break Detection Enable*/
        /**********************************************/
        uart_hal_set_break_char_detect_length(uartInstance, kUartBreakChar13BitMinimum);
        if (uart_hal_test_get_break_char_detect_length(uartInstance) != kUartBreakChar13BitMinimum)
        {
            s_uartTestStatus[uartInstance].B.testStatusLBKDE = 1;
        }

        uart_hal_set_break_char_detect_length(uartInstance, kUartBreakChar10BitMinimum);
        if (uart_hal_test_get_break_char_detect_length(uartInstance) != kUartBreakChar10BitMinimum)
        {
            s_uartTestStatus[uartInstance].B.testStatusLBKDE = 1;
        }

        /**********************************************/
        /* Test Break Char Generation Length*/
        /**********************************************/
        uart_configure_tx_rx_enable(uartInstance, 0);

        uart_hal_set_break_char_transmit_length(uartInstance, kUartBreakChar13BitMinimum);
        if (uart_hal_test_get_break_char_transmit_length(uartInstance) != kUartBreakChar13BitMinimum)
        {
            s_uartTestStatus[uartInstance].B.testStatusBRK13 = 1;
        }

        uart_hal_set_break_char_transmit_length(uartInstance, kUartBreakChar10BitMinimum);
        if (uart_hal_test_get_break_char_transmit_length(uartInstance) != kUartBreakChar10BitMinimum)
        {
            s_uartTestStatus[uartInstance].B.testStatusBRK13 = 1;
        }

        uart_configure_tx_rx_enable(uartInstance, 1);

        /**********************************************/
        /* Test enable/disable parity_error_interrupt*/
        /**********************************************/
        uart_hal_enable_parity_error_interrupt(uartInstance);
        if (uart_hal_is_parity_error_interrupt_enabled(uartInstance) != 1)
        {
            s_uartTestStatus[uartInstance].B.testStatusPEIE = 1;
        }

        uart_hal_disable_parity_error_interrupt(uartInstance);
        if (uart_hal_is_parity_error_interrupt_enabled(uartInstance) != 0)
        {
            s_uartTestStatus[uartInstance].B.testStatusPEIE = 1;
        }

        /**********************************************/
        /* Test enable/disable framing_error_interrupt*/
        /**********************************************/
        uart_hal_enable_framing_error_interrupt(uartInstance);
        if (uart_hal_is_framing_error_interrupt_enabled(uartInstance) != 1)
        {
            s_uartTestStatus[uartInstance].B.testStatusFEIE = 1;
        }

        uart_hal_disable_framing_error_interrupt(uartInstance);
        if (uart_hal_is_framing_error_interrupt_enabled(uartInstance) != 0)
        {
            s_uartTestStatus[uartInstance].B.testStatusFEIE = 1;
        }

        /**********************************************/
        /* Test enable/disable noise_error_interrupt*/
        /**********************************************/
        uart_hal_enable_noise_error_interrupt(uartInstance);
        if (uart_hal_is_noise_error_interrupt_enabled(uartInstance) != 1)
        {
            s_uartTestStatus[uartInstance].B.testStatusNEIE = 1;
        }

        uart_hal_disable_noise_error_interrupt(uartInstance);
        if (uart_hal_is_noise_error_interrupt_enabled(uartInstance) != 0)
        {
            s_uartTestStatus[uartInstance].B.testStatusNEIE = 1;
        }

        /**********************************************/
        /* Test enable/disable overrun_error_interrupt*/
        /**********************************************/
        uart_hal_enable_rx_overrun_interrupt(uartInstance);
        if (uart_hal_is_rx_overrun_interrupt_enabled(uartInstance) != 1)
        {
            s_uartTestStatus[uartInstance].B.testStatusORIE = 1;
        }

        uart_hal_disable_rx_overrun_interrupt(uartInstance);
        if (uart_hal_is_rx_overrun_interrupt_enabled(uartInstance) != 0)
        {
            s_uartTestStatus[uartInstance].B.testStatusORIE = 1;
        }

        /**********************************************/
        /* Test uart_hal_configure_interrupts*/
        /**********************************************/
        uart_interrupt_config_t interruptConfig;

        /* set all interrupt configs*/
        interruptConfig.linBreakDetect = 1;
        interruptConfig.rxActiveEdge = 1;
        interruptConfig.transmitDataRegisterEmpty = 1;
        interruptConfig.transmitComplete = 1;
        interruptConfig.receiverDataRegisterFull = 1;
        interruptConfig.idleLine = 1;
        interruptConfig.receiverOverrun = 1;
        interruptConfig.noiseErrorFlag = 1;
        interruptConfig.frameErrorFlag = 1;
        interruptConfig.parityErrorFlag = 1;
#if FSL_FEATURE_UART_HAS_FIFO
        interruptConfig.txFifoOverflow = 1;
        interruptConfig.rxFifoUnderflow = 1;
#endif

        uart_hal_configure_interrupts(uartInstance, &interruptConfig);

        g_failCount = 0;
        if (uart_hal_is_break_detect_interrupt_enabled(uartInstance) != 1)
        {
            g_failCount++;
        }
        if (uart_hal_is_rx_active_edge_interrupt_enabled(uartInstance) != 1)
        {
            g_failCount++;
        }
        if (uart_hal_is_tx_data_register_empty_interrupt_enabled(uartInstance) != 1)
        {
            g_failCount++;
        }
        if (uart_hal_is_transmission_complete_interrupt_enabled(uartInstance) != 1)
        {
            g_failCount++;
        }
        if (uart_hal_is_receive_data_full_interrupt_enabled(uartInstance) != 1)
        {
            g_failCount++;
        }
        if (uart_hal_is_idle_line_interrupt_enabled(uartInstance) != 1)
        {
            g_failCount++;
        }
        if (uart_hal_is_rx_overrun_interrupt_enabled(uartInstance) != 1)
        {
            g_failCount++;
        }
        if (uart_hal_is_noise_error_interrupt_enabled(uartInstance) != 1)
        {
            g_failCount++;
        }
        if (uart_hal_is_framing_error_interrupt_enabled(uartInstance) != 1)
        {
            g_failCount++;
        }
        if (uart_hal_is_parity_error_interrupt_enabled(uartInstance) != 1)
        {
            g_failCount++;
        }
#if FSL_FEATURE_UART_HAS_FIFO
        if (uart_hal_is_tx_fifo_overflow_interrupt_enabled(uartInstance) != 1)
        {
            g_failCount++;
        }
        if (uart_hal_is_rx_fifo_underflow_interrupt_enabled(uartInstance) != 1)
        {
            g_failCount++;
        }
#endif
        if (g_failCount != 0)
        {
            s_uartTestStatus[uartInstance].B.testStatusConfigAllInterrupts = 1;
        }

        /* clear all interrupt configs*/
        interruptConfig.linBreakDetect = 0;
        interruptConfig.rxActiveEdge = 0;
        interruptConfig.transmitDataRegisterEmpty = 0;
        interruptConfig.transmitComplete = 0;
        interruptConfig.receiverDataRegisterFull = 0;
        interruptConfig.idleLine = 0;
        interruptConfig.receiverOverrun = 0;
        interruptConfig.noiseErrorFlag = 0;
        interruptConfig.frameErrorFlag = 0;
        interruptConfig.parityErrorFlag = 0;
#if FSL_FEATURE_UART_HAS_FIFO
        interruptConfig.txFifoOverflow = 0;
        interruptConfig.rxFifoUnderflow = 0;
#endif

        uart_hal_configure_interrupts(uartInstance, &interruptConfig);

        g_failCount = 0;
        if (uart_hal_is_break_detect_interrupt_enabled(uartInstance) != 0)
        {
            g_failCount++;
        }
        if (uart_hal_is_rx_active_edge_interrupt_enabled(uartInstance) != 0)
        {
            g_failCount++;
        }
        if (uart_hal_is_tx_data_register_empty_interrupt_enabled(uartInstance) != 0)
        {
            g_failCount++;
        }
        if (uart_hal_is_transmission_complete_interrupt_enabled(uartInstance) != 0)
        {
            g_failCount++;
        }
        if (uart_hal_is_receive_data_full_interrupt_enabled(uartInstance) != 0)
        {
            g_failCount++;
        }
        if (uart_hal_is_idle_line_interrupt_enabled(uartInstance) != 0)
        {
            g_failCount++;
        }
        if (uart_hal_is_rx_overrun_interrupt_enabled(uartInstance) != 0)
        {
            g_failCount++;
        }
        if (uart_hal_is_noise_error_interrupt_enabled(uartInstance) != 0)
        {
            g_failCount++;
        }
        if (uart_hal_is_framing_error_interrupt_enabled(uartInstance) != 0)
        {
            g_failCount++;
        }
        if (uart_hal_is_parity_error_interrupt_enabled(uartInstance) != 0)
        {
            g_failCount++;
        }
#if FSL_FEATURE_UART_HAS_FIFO
        if (uart_hal_is_tx_fifo_overflow_interrupt_enabled(uartInstance) != 0)
        {
            g_failCount++;
        }
        if (uart_hal_is_rx_fifo_underflow_interrupt_enabled(uartInstance) != 0)
        {
            g_failCount++;
        }
#endif
        if (g_failCount != 0)
        {
            s_uartTestStatus[uartInstance].B.testStatusConfigAllInterrupts = 1;
        }

        /**********************************************/
        /* Test singlewire mode config*/
        /**********************************************/
        uart_configure_tx_rx_enable(uartInstance, 0);

        /* enable singlewire mode*/
        uart_hal_configure_singlewire_mode(uartInstance, 1);
        if (uart_hal_test_is_singlewire_operation_config(uartInstance) != 1)
        {
            s_uartTestStatus[uartInstance].B.testStatusSinglewireMode = 1;
        }

        /* disable singlewire mode*/
        uart_hal_configure_singlewire_mode(uartInstance, 0);
        if (uart_hal_test_is_singlewire_operation_config(uartInstance) != 0)
        {
            s_uartTestStatus[uartInstance].B.testStatusSinglewireMode = 1;
        }

        uart_configure_tx_rx_enable(uartInstance, 1);

        /**********************************************/
        /* Test singlewire mode txdir config*/
        /**********************************************/
        /* configure txdir direction to output (essentially set the bit)*/
        uart_hal_configure_txdir_in_singlewire_mode(uartInstance, kUartSinglewireTxdirOut);
        if (uart_hal_test_get_txdir_config(uartInstance) != kUartSinglewireTxdirOut)
        {
            s_uartTestStatus[uartInstance].B.testStatusTXDIR = 1;
        }

        /* configure txdir to in*/
        uart_hal_configure_txdir_in_singlewire_mode(uartInstance, kUartSinglewireTxdirIn);
        if (uart_hal_test_get_txdir_config(uartInstance) != kUartSinglewireTxdirIn)
        {
            s_uartTestStatus[uartInstance].B.testStatusTXDIR = 1;
        }

        /**********************************************/
        /* Test DMA enable settings*/
        /**********************************************/
        /* enable the DMA for tx and rx*/
        uart_hal_configure_dma(uartInstance, 1, 1);
        /* check to make sure tx and rx DMA was enabled*/
        if (uart_hal_is_txdma_enabled(uartInstance) != 1)
        {
            s_uartTestStatus[uartInstance].B.testStatusTDMAE = 1;
        }
        if (uart_hal_is_rxdma_enabled(uartInstance) != 1)
        {
            s_uartTestStatus[uartInstance].B.testStatusRDMAE = 1;
        }

        /* disable the DMA for tx and rx*/
        uart_hal_configure_dma(uartInstance, 0, 0);
        /* check to make sure tx and rx DMA was enabled*/
        if (uart_hal_is_txdma_enabled(uartInstance) != 0)
        {
            s_uartTestStatus[uartInstance].B.testStatusTDMAE = 1;
        }
        if (uart_hal_is_rxdma_enabled(uartInstance) != 0)
        {
            s_uartTestStatus[uartInstance].B.testStatusRDMAE = 1;
        }

        /**********************************************/
        /* Test configure_match_address_operation*/
        /**********************************************/
        uint8_t matchAddr1, matchAddr2;
        matchAddr1 = 0xA5;
        matchAddr2 = 0x96;
        /* enable match address for register 1 and 2 and set the match address registers*/
        if (uart_hal_configure_match_address_operation(uartInstance, 1, 1, matchAddr1, matchAddr2) == kStatus_Success)
        {
#if FSL_FEATURE_UART_HAS_LOW_POWER_UART_SUPPORT
            /* for supported uart0 (low power uart) instance, check to see if bits were enabled and  */
            /* match address registers match what was programmed into them*/
            if (uartInstance == 0)
#endif
            {
                if (uart_hal_test_get_match_address_register(uartInstance, 1) != matchAddr1)
                {
                    s_uartTestStatus[uartInstance].B.testStatusMA1 = 1;
                }
                if (uart_hal_test_get_match_address_register(uartInstance, 2) != matchAddr2)
                {
                    s_uartTestStatus[uartInstance].B.testStatusMA2 = 1;
                }
                if (uart_hal_test_is_match_address_mode1_enabled(uartInstance) != 1)
                {
                    s_uartTestStatus[uartInstance].B.testStatusMAEN1 = 1;
                }
                if (uart_hal_test_is_match_address_mode2_enabled(uartInstance) != 1)
                {
                    s_uartTestStatus[uartInstance].B.testStatusMAEN2 = 1;
                }
            }
#if FSL_FEATURE_UART_HAS_LOW_POWER_UART_SUPPORT
            /* for non-uart0 (low power uart) instances, match address feature is not available, so */
            /* we should not get a success if we do, then set fail flags*/
            else
            {
                s_uartTestStatus[uartInstance].B.testStatusMA1 = 1;
                s_uartTestStatus[uartInstance].B.testStatusMA2 = 1;
                s_uartTestStatus[uartInstance].B.testStatusMAEN1 = 1;
                s_uartTestStatus[uartInstance].B.testStatusMAEN2 = 1;
            }
#endif
        }
        else
        {
/* this indicates we got some kind of failure for a supported feature, */
/* set failure flags*/
#if FSL_FEATURE_UART_HAS_LOW_POWER_UART_SUPPORT
            if (uartInstance == 0)
#endif
            {
                s_uartTestStatus[uartInstance].B.testStatusMA1 = 1;
                s_uartTestStatus[uartInstance].B.testStatusMA2 = 1;
                s_uartTestStatus[uartInstance].B.testStatusMAEN1 = 1;
                s_uartTestStatus[uartInstance].B.testStatusMAEN2 = 1;
            }
        }

        /* now disable match address for register 1 and 2 and clear the match address registers*/
        matchAddr1 = 0x00;
        matchAddr2 = 0x00;
        if (uart_hal_configure_match_address_operation(uartInstance, 0, 0, matchAddr1, matchAddr2) == kStatus_Success)
        {
#if FSL_FEATURE_UART_HAS_LOW_POWER_UART_SUPPORT
            /* for supported uart0 (low power uart) instance, check to see if bits were disabled */
            /* and match address registers match what was programmed into them*/
            if (uartInstance == 0)
#endif
            {
                if (uart_hal_test_get_match_address_register(uartInstance, 1) != matchAddr1)
                {
                    s_uartTestStatus[uartInstance].B.testStatusMA1 = 1;
                }
                if (uart_hal_test_get_match_address_register(uartInstance, 2) != matchAddr2)
                {
                    s_uartTestStatus[uartInstance].B.testStatusMA2 = 1;
                }
                if (uart_hal_test_is_match_address_mode1_enabled(uartInstance) != 0)
                {
                    s_uartTestStatus[uartInstance].B.testStatusMAEN1 = 1;
                }
                if (uart_hal_test_is_match_address_mode2_enabled(uartInstance) != 0)
                {
                    s_uartTestStatus[uartInstance].B.testStatusMAEN2 = 1;
                }
            }
#if FSL_FEATURE_UART_HAS_LOW_POWER_UART_SUPPORT
            /* for non-uart0 (low power uart) instances, match address feature is not available, */
            /* so we should  not get a success if we do, then set fail flags*/
            else
            {
                s_uartTestStatus[uartInstance].B.testStatusMA1 = 1;
                s_uartTestStatus[uartInstance].B.testStatusMA2 = 1;
                s_uartTestStatus[uartInstance].B.testStatusMAEN1 = 1;
                s_uartTestStatus[uartInstance].B.testStatusMAEN2 = 1;
            }
#endif
        }
        else
        {
/* this indicates we got some kind of failure for a supported feature, set*/
/* failure flags*/
#if FSL_FEATURE_UART_HAS_LOW_POWER_UART_SUPPORT
            if (uartInstance == 0)
#endif
            {
                s_uartTestStatus[uartInstance].B.testStatusMA1 = 1;
                s_uartTestStatus[uartInstance].B.testStatusMA2 = 1;
                s_uartTestStatus[uartInstance].B.testStatusMAEN1 = 1;
                s_uartTestStatus[uartInstance].B.testStatusMAEN2 = 1;
            }
        }

        /**********************************************/
        /* Test configure_receive_resync_disable_operation*/
        /**********************************************/
        /* configure receive resync to disable*/
        if (uart_hal_configure_receive_resync_disable_operation(uartInstance, 1) == kStatus_Success)
        {
/* for supported uart instances, check to see if bits were set to disable this mode*/
#if FSL_FEATURE_UART_HAS_RX_RESYNC_SUPPORT
            if (uartInstance == 0)
            {
                if (uart_hal_test_is_receive_resync_disabled(uartInstance) != 1)
                {
                    s_uartTestStatus[uartInstance].B.testStatusRESYNCDIS = 1;
                }
            }
            /* for non-uart instances, this feature is not available, so we should not get a success*/
            /* if we do, then set fail flags*/
            else
#endif
            {
                s_uartTestStatus[uartInstance].B.testStatusRESYNCDIS = 1;
            }
        }
        else
        {
/* this indicates we got some kind of failure for a supported feature, set failure flags*/
#if FSL_FEATURE_UART_HAS_RX_RESYNC_SUPPORT
            if (uartInstance == 0)
            {
                s_uartTestStatus[uartInstance].B.testStatusRESYNCDIS = 1;
            }
#endif
        }

        /* configure receive resync to non-disable*/
        if (uart_hal_configure_receive_resync_disable_operation(uartInstance, 0) == kStatus_Success)
        {
#if FSL_FEATURE_UART_HAS_RX_RESYNC_SUPPORT
            /* for supported uart instances, check to see if bits were set to non-disable this mode*/
            if (uartInstance == 0)
            {
                if (uart_hal_test_is_receive_resync_disabled(uartInstance) != 0)
                {
                    s_uartTestStatus[uartInstance].B.testStatusRESYNCDIS = 1;
                }
            }
            /* for non-uart instances, this feature is not available, so we should not get a success*/
            /* if we do, then set fail flags*/
            else
#endif
            {
                s_uartTestStatus[uartInstance].B.testStatusRESYNCDIS = 1;
            }
        }
        else
        {
/* this indicates we got some kind of failure for a supported feature, set failure flags*/
#if FSL_FEATURE_UART_HAS_RX_RESYNC_SUPPORT
            if (uartInstance == 0)
            {
                s_uartTestStatus[uartInstance].B.testStatusRESYNCDIS = 1;
            }
#endif
        }

        /**********************************************/
        /* Test set and get baud rate*/
        /**********************************************/
        uint32_t baudRates[] = {1200, 2400, 4800, 9600, 14400, 19200, 38400, 57600, 115200};
        uint32_t baudRateDiff;
        uint32_t baudRateDiffPercent;
        uint32_t baudRateIndex;
        uint32_t uartSrcClkIndex;
        status_t baudRateCalcStatus;

        uint32_t uartSrcClkHz[] = {8000000, 41943040, 48000000, 50000000, 60000000, 120000000};

        /* test various source clock inputs*/
        for (uartSrcClkIndex = 0; uartSrcClkIndex < (sizeof(uartSrcClkHz)) / 4; uartSrcClkIndex++)
        {
            /* test various baud rates*/
            for (baudRateIndex = 0; baudRateIndex < (sizeof(baudRates)) / 4; baudRateIndex++)
            {
                /* for lower source clock rates, especially 8MHz, it is not possible to generate*/
                /* higher baud rates. Therefore set some conservative limits for source clock*/
                /* and the baud rate*/
                if ((baudRates[baudRateIndex] > 38400) && (uartSrcClkHz[uartSrcClkIndex] < 48000000))
                {
                    break;
                }

                baudRateCalcStatus =
                    uart_hal_set_baud_rate(uartInstance, uartSrcClkHz[uartSrcClkIndex], baudRates[baudRateIndex]);
                if (baudRateCalcStatus == kStatus_Success)
                {
                    /* the set and get baud rates may not match, so first get the */
                    /* difference between the two*/
                    baudRateDiff = abs(uart_hal_test_get_baud_rate(uartInstance, uartSrcClkHz[uartSrcClkIndex]) -
                                       baudRates[baudRateIndex]);

                    /* now see if the difference is within 3%, which it should be otherwise */
                    /* function would return a baud rate calculation percent error code*/
                    baudRateDiffPercent = (baudRates[baudRateIndex] / 100) * 3;
                    if (baudRateDiff > baudRateDiffPercent)
                    {
                        /* fail if baud rate difference exceeds 3%*/
                        s_uartTestStatus[uartInstance].B.testStatusBaudRate = 1;
                    }
                }
                else
                {
                    /* fail if error code is returned*/
                    s_uartTestStatus[uartInstance].B.testStatusBaudRate = 1;
                }
            }
        }

        /**********************************************/
        /* Test bate rate modulo divisor functions*/
        /**********************************************/
        uint32_t sbrValue = 0x15A9; /* set sbrValue to a known bit pattern         */

        if (uart_hal_set_baud_rate_divisor(uartInstance, sbrValue) == kStatus_Success)
        {
            if (uart_hal_test_get_baud_rate_divisor(uartInstance) != sbrValue)
            {
                /* fail if value doesn't match what was programmed*/
                s_uartTestStatus[uartInstance].B.testStatusSBR = 1;
            }
        }
        else
        {
            /* fail if not success since sbr value is within normal parameters*/
            s_uartTestStatus[uartInstance].B.testStatusSBR = 1;
        }

        sbrValue = 0x2000; /* now set sbrValue to an unsupported value*/
        if (uart_hal_set_baud_rate_divisor(uartInstance, sbrValue) == kStatus_Success)
        {
            /* fail if success since sbr value is an invalid value*/
            s_uartTestStatus[uartInstance].B.testStatusSBR = 1;
        }

#if FSL_FEATURE_UART_HAS_BAUD_RATE_FINE_ADJUST_SUPPORT
        /**********************************************/
        /* Test bate rate fine adjust functions*/
        /**********************************************/
        uint32_t brfaValue = 0x15; /* set brfaValue to a known bit pattern  */

        if (uart_hal_set_baud_rate_fine_adjust(uartInstance, brfaValue) == kStatus_Success)
        {
            if (uart_hal_test_get_baud_rate_fine_adjust(uartInstance) != brfaValue)
            {
                /* fail if value doesn't match what was programmed*/
                s_uartTestStatus[uartInstance].B.testStatusBaudRateFineAdjust = 1;
            }
        }
        else
        {
            /* fail if not success since brfaValue is within normal parameters*/
            s_uartTestStatus[uartInstance].B.testStatusBaudRateFineAdjust = 1;
        }

        brfaValue = 0x20; /* now set brfaValue to an unsupported value*/
        if (uart_hal_set_baud_rate_fine_adjust(uartInstance, brfaValue) == kStatus_Success)
        {
            /* fail if success since brfaValue is an invalid value*/
            s_uartTestStatus[uartInstance].B.testStatusBaudRateFineAdjust = 1;
        }
#endif /* FSL_FEATURE_UART_HAS_BAUD_RATE_FINE_ADJUST_SUPPORT        */

#if FSL_FEATURE_UART_HAS_BAUD_RATE_OVER_SAMPLING_SUPPORT
        /**********************************************/
        /* Test bate rate oversampling functions*/
        /**********************************************/
        uart_configure_tx_rx_enable(uartInstance, 0);

        uint32_t osrValue = 0x15; /* set osrValue to a known bit pattern  */

        if (uart_hal_set_oversampling_ratio(uartInstance, osrValue) == kStatus_Success)
        {
            /* for supported uart instances */
            if (uartInstance == 0)
            {
                if (uart_hal_test_get_oversampling_ratio(uartInstance) != osrValue)
                {
                    /* fail if value doesn't match what was programmed*/
                    s_uartTestStatus[uartInstance].B.testStatusBaudOverSampling = 1;
                }
            }
            /* for unsupported uart instances */
            else
            {
                /* fail since this is not supported in other uart instances*/
                s_uartTestStatus[uartInstance].B.testStatusBaudOverSampling = 1;
            }
        }
        else
        {
            if (uartInstance == 0)
            {
                /* fail if not success since osrValue is within normal parameters*/
                s_uartTestStatus[uartInstance].B.testStatusBaudOverSampling = 1;
            }
        }

        osrValue = 0x20; /* now set osrValue to an unsupported value*/
        if (uartInstance == 0)
        {
            if (uart_hal_set_oversampling_ratio(uartInstance, osrValue) == kStatus_Success)
            {
                /* fail if success since osrValue is an invalid value*/
                s_uartTestStatus[uartInstance].B.testStatusBaudOverSampling = 1;
            }
        }
        uart_configure_tx_rx_enable(uartInstance, 1);
#endif /* FSL_FEATURE_UART_HAS_BAUD_RATE_OVER_SAMPLING_SUPPORT*/

#if FSL_FEATURE_UART_HAS_BOTH_EDGE_SAMPLING_SUPPORT
        /**********************************************/
        /* Test bate rate both edge sampling functions*/
        /**********************************************/
        uart_configure_tx_rx_enable(uartInstance, 0);

        /* enable both edge sampling*/
        if (uart_hal_configure_both_edge_sampling(uartInstance, 1) == kStatus_Success)
        {
            /* for supported uart instances */
            if (uartInstance == 0)
            {
                if (uart_hal_test_get_both_edge_sampling_setting(uartInstance) != 1)
                {
                    /* fail since this did not match what was programmed*/
                    s_uartTestStatus[uartInstance].B.testStatusBaudBothEdgeSampling = 1;
                }
            }
            /* for unsupported uart instances */
            else
            {
                /* fail since this is not supported in other uart instances*/
                s_uartTestStatus[uartInstance].B.testStatusBaudBothEdgeSampling = 1;
            }
        }
        else
        {
            if (uartInstance == 0)
            {
                /* fail since this is supported */
                s_uartTestStatus[uartInstance].B.testStatusBaudBothEdgeSampling = 1;
            }
        }

        /* disable both edge sampling*/
        if (uart_hal_configure_both_edge_sampling(uartInstance, 0) == kStatus_Success)
        {
            /* for supported uart instances */
            if (uartInstance == 0)
            {
                if (uart_hal_test_get_both_edge_sampling_setting(uartInstance) != 0)
                {
                    /* fail since this did not match what was programmed*/
                    s_uartTestStatus[uartInstance].B.testStatusBaudBothEdgeSampling = 1;
                }
            }
            /* for unsupported uart instances */
            else
            {
                /* fail since this is not supported in other uart instances*/
                s_uartTestStatus[uartInstance].B.testStatusBaudBothEdgeSampling = 1;
            }
        }
        else
        {
            if (uartInstance == 0)
            {
                /* fail since this is supported */
                s_uartTestStatus[uartInstance].B.testStatusBaudBothEdgeSampling = 1;
            }
        }

        uart_configure_tx_rx_enable(uartInstance, 1);
#endif
        /**********************************************/
        /* Test status flags*/
        /**********************************************/
        /* first, get the current state of all status bits*/
        uart_status_t allStatusFlag;
        uart_hal_get_all_status_flag(uartInstance, &allStatusFlag);

        g_failCount = 0;
        /* now check each status bit against the individual get status function*/
        if (uart_hal_is_line_break_detected(uartInstance) != (bool)allStatusFlag.lineBreakDetect)
        {
            g_failCount++;
        }
        if (uart_hal_is_receive_active_edge_detected(uartInstance) != (bool)allStatusFlag.receiveActiveEdgeDetect)
        {
            g_failCount++;
        }
        if (uart_hal_is_transmit_data_register_empty(uartInstance) != (bool)allStatusFlag.transmitDataRegisterEmpty)
        {
            g_failCount++;
        }
        if (uart_hal_is_transmission_complete(uartInstance) != (bool)allStatusFlag.transmissionComplete)
        {
            g_failCount++;
        }
        if (uart_hal_is_receive_data_register_full(uartInstance) != (bool)allStatusFlag.receiveDataRegisterFull)
        {
            g_failCount++;
        }
        if (uart_hal_is_idle_line_detected(uartInstance) != (bool)allStatusFlag.idleLineDetect)
        {
            g_failCount++;
        }
        if (uart_hal_is_receive_overrun_detected(uartInstance) != (bool)allStatusFlag.receiveOverrun)
        {
            g_failCount++;
        }
        if (uart_hal_is_noise_detected(uartInstance) != (bool)allStatusFlag.noiseDetect)
        {
            g_failCount++;
        }
        if (uart_hal_is_frame_error_detected(uartInstance) != (bool)allStatusFlag.frameError)
        {
            g_failCount++;
        }
        if (uart_hal_is_parity_error_detected(uartInstance) != (bool)allStatusFlag.parityError)
        {
            g_failCount++;
        }
        if (uart_hal_is_receiver_active(uartInstance) != (bool)allStatusFlag.receiverActive)
        {
            g_failCount++;
        }
#if FSL_FEATURE_UART_HAS_EXTENDED_DATA_REGISTER_FLAGS
        if (uart_hal_is_current_dataword_received_with_noise(uartInstance) != (bool)allStatusFlag.noiseInCurrentWord)
        {
            g_failCount++;
        }
        if (uart_hal_is_current_dataword_received_with_parityerror(uartInstance) !=
            (bool)allStatusFlag.parityErrorInCurrentWord)
        {
            g_failCount++;
        }
#endif
#if FSL_FEATURE_UART_HAS_FIFO
        if (uart_hal_is_tx_fifo_empty(uartInstance) != (bool)allStatusFlag.txBufferEmpty)
        {
            g_failCount++;
        }
        if (uart_hal_is_rx_fifo_empty(uartInstance) != (bool)allStatusFlag.rxBufferEmpty)
        {
            g_failCount++;
        }
        if (uart_hal_is_tx_fifo_overflow(uartInstance) != (bool)allStatusFlag.txBufferOverflow)
        {
            g_failCount++;
        }
        if (uart_hal_is_rx_fifo_underflow(uartInstance) != (bool)allStatusFlag.rxBufferUnderflow)
        {
            g_failCount++;
        }
#endif

        if (g_failCount != 0)
        {
            s_uartTestStatus[uartInstance].B.testStatusFlagsCheck = 1;
        }

        /* now clear all manually clearable status flags and check them*/
        /* this also tests the uart_hal_clear_status_flag function*/
        uart_hal_clear_all_non_autoclear_status_flags(uartInstance);

        g_failCount = 0;
        if (uart_hal_is_line_break_detected(uartInstance) != 0)
        {
            g_failCount++;
        }
        if (uart_hal_is_receive_overrun_detected(uartInstance) != 0)
        {
            g_failCount++;
        }
        if (uart_hal_is_noise_detected(uartInstance) != 0)
        {
            g_failCount++;
        }
        if (uart_hal_is_frame_error_detected(uartInstance) != 0)
        {
            g_failCount++;
        }
        if (uart_hal_is_parity_error_detected(uartInstance) != 0)
        {
            g_failCount++;
        }
        if (uart_hal_is_idle_line_detected(uartInstance) != 0)
        {
            g_failCount++;
        }
        if (uart_hal_is_receive_active_edge_detected(uartInstance) != 0)
        {
            g_failCount++;
        }
#if FSL_FEATURE_UART_HAS_FIFO
        if (uart_hal_is_tx_fifo_overflow(uartInstance) != 0)
        {
            g_failCount++;
        }
        if (uart_hal_is_rx_fifo_underflow(uartInstance) != 0)
        {
            g_failCount++;
        }
#endif

        if (g_failCount != 0)
        {
            s_uartTestStatus[uartInstance].B.testStatusFlagsClear = 1;
        }

        g_failCount = 0;
        /* finally, check non-manual cleared status flags, to make sure we get back an error*/
        if (uart_hal_clear_status_flag(uartInstance, kUartTransmitDataRegisterEmpty) == kStatus_Success)
        {
            g_failCount++;
        }
        if (uart_hal_clear_status_flag(uartInstance, kUartTransmissionComplete) == kStatus_Success)
        {
            g_failCount++;
        }
        if (uart_hal_clear_status_flag(uartInstance, kUartReceiveDataRegisterFull) == kStatus_Success)
        {
            g_failCount++;
        }
        if (uart_hal_clear_status_flag(uartInstance, kUartReceiverActive) == kStatus_Success)
        {
            g_failCount++;
        }
#if FSL_FEATURE_UART_HAS_EXTENDED_DATA_REGISTER_FLAGS
        if (uart_hal_clear_status_flag(uartInstance, kUartNoiseInCurrentWord) == kStatus_Success)
        {
            g_failCount++;
        }
        if (uart_hal_clear_status_flag(uartInstance, kUartParityErrorInCurrentWord) == kStatus_Success)
        {
            g_failCount++;
        }
#endif
#if FSL_FEATURE_UART_HAS_FIFO
        if (uart_hal_clear_status_flag(uartInstance, kUartTxBufferEmpty) == kStatus_Success)
        {
            g_failCount++;
        }
        if (uart_hal_clear_status_flag(uartInstance, kUartRxBufferEmpty) == kStatus_Success)
        {
            g_failCount++;
        }
#endif

        if (g_failCount != 0)
        {
            s_uartTestStatus[uartInstance].B.testStatusFlagsClear = 1;
        }

#if FSL_FEATURE_UART_HAS_EXTENDED_DATA_REGISTER_FLAGS
        /**********************************************/
        /* Test configuring bit10 as parity*/
        /**********************************************/
        /* enable bit-10 as parity bit*/
        /* Note, to actually do this we'll also need to enable 9-bit data (UARTx_C1[M]=1)*/
        /* and enable parity (UARTx_C1[PE]=1). However, for now we'll just check*/
        /* to make sure this bit gets set.*/
        uart_hal_configure_bit10_as_paritybit_operation(uartInstance, 1);
        if (uart_hal_is_bit10_set_as_paritybit(uartInstance) != 1)
        {
            s_uartTestStatus[uartInstance].B.testStatusBit10AsParity = 1;
        }

        /* Now disable this bit and test*/
        uart_hal_configure_bit10_as_paritybit_operation(uartInstance, 0);
        if (uart_hal_is_bit10_set_as_paritybit(uartInstance) != 0)
        {
            s_uartTestStatus[uartInstance].B.testStatusBit10AsParity = 1;
        }

        /**********************************************/
        /* Test the current word noisy and parity error flags*/
        /**********************************************/
        /* These flags should be cleared since there is no received current word */
        if (uart_hal_is_current_dataword_received_with_noise(uartInstance) != 0)
        {
            s_uartTestStatus[uartInstance].B.testStatusNoisyFlag = 1;
        }
        if (uart_hal_is_current_dataword_received_with_parityerror(uartInstance) != 0)
        {
            s_uartTestStatus[uartInstance].B.testStatusParityErrorFlag = 1;
        }
#endif
#if FSL_FEATURE_UART_HAS_MODEM_SUPPORT
        /**********************************************/
        /* Test the various RTS and CTS configuration functions*/
        /**********************************************/
        /* test enabling/disabling rx rts*/
        uart_hal_enable_receiver_rts(uartInstance);
        if (uart_hal_test_is_receiver_rts_enabled(uartInstance) != 1)
        {
            s_uartTestStatus[uartInstance].B.testStatusReceiverRtsEnable = 1;
        }

        uart_hal_disable_receiver_rts(uartInstance);
        if (uart_hal_test_is_receiver_rts_enabled(uartInstance) != 0)
        {
            s_uartTestStatus[uartInstance].B.testStatusReceiverRtsEnable = 1;
        }

        /* test enabling/disabling tx rts*/
        uart_hal_enable_transmitter_rts(uartInstance);
        if (uart_hal_test_is_transmitter_rts_enabled(uartInstance) != 1)
        {
            s_uartTestStatus[uartInstance].B.testStatusTransmitterRtsEnable = 1;
        }

        uart_hal_disable_transmitter_rts(uartInstance);
        if (uart_hal_test_is_transmitter_rts_enabled(uartInstance) != 0)
        {
            s_uartTestStatus[uartInstance].B.testStatusTransmitterRtsEnable = 1;
        }

        /* test tx rts polarity setting*/
        uart_hal_configure_transmitter_rts_polarity(uartInstance, 1);
        if (uart_hal_test_get_transmitter_rts_polarity_config(uartInstance) != 1)
        {
            s_uartTestStatus[uartInstance].B.testStatusTransmitterRtsPolarity = 1;
        }

        uart_hal_configure_transmitter_rts_polarity(uartInstance, 0);
        if (uart_hal_test_get_transmitter_rts_polarity_config(uartInstance) != 0)
        {
            s_uartTestStatus[uartInstance].B.testStatusTransmitterRtsPolarity = 1;
        }

        /* test enabling/disabling tx cts*/
        uart_hal_enable_transmitter_cts(uartInstance);
        if (uart_hal_test_is_transmitter_cts_enabled(uartInstance) != 1)
        {
            s_uartTestStatus[uartInstance].B.testStatusTransmitterCtsEnable = 1;
        }

        uart_hal_disable_transmitter_cts(uartInstance);
        if (uart_hal_test_is_transmitter_cts_enabled(uartInstance) != 0)
        {
            s_uartTestStatus[uartInstance].B.testStatusTransmitterCtsEnable = 1;
        }
#endif
#if FSL_FEATURE_UART_HAS_IR_SUPPORT
        /**********************************************/
        /* Test infrared configuration settings*/
        /**********************************************/
        /* enable infrared and test the various configurations*/
        uart_hal_configure_infrared_operation(uartInstance, 1, kUartIrThreeSixteenthsWidth);
        if (uart_hal_test_is_infrared_enabled(uartInstance) != 1)
        {
            s_uartTestStatus[uartInstance].B.testStatusInfraredEnable = 1;
        }
        if (uart_hal_test_get_infrared_pulse_config(uartInstance) != kUartIrThreeSixteenthsWidth)
        {
            s_uartTestStatus[uartInstance].B.testStatusInfraredPulse = 1;
        }

        uart_hal_configure_infrared_operation(uartInstance, 1, kUartIrOneSixteenthWidth);
        if (uart_hal_test_get_infrared_pulse_config(uartInstance) != kUartIrOneSixteenthWidth)
        {
            s_uartTestStatus[uartInstance].B.testStatusInfraredPulse = 1;
        }

        uart_hal_configure_infrared_operation(uartInstance, 1, kUartIrOneThirtysecondsWidth);
        if (uart_hal_test_get_infrared_pulse_config(uartInstance) != kUartIrOneThirtysecondsWidth)
        {
            s_uartTestStatus[uartInstance].B.testStatusInfraredPulse = 1;
        }

        uart_hal_configure_infrared_operation(uartInstance, 1, kUartIrOneFourthWidth);
        if (uart_hal_test_get_infrared_pulse_config(uartInstance) != kUartIrOneFourthWidth)
        {
            s_uartTestStatus[uartInstance].B.testStatusInfraredPulse = 1;
        }

        /* now disable infrared */
        uart_hal_configure_infrared_operation(uartInstance, 0, kUartIrThreeSixteenthsWidth);
        if (uart_hal_test_is_infrared_enabled(uartInstance) != 0)
        {
            s_uartTestStatus[uartInstance].B.testStatusInfraredEnable = 1;
        }

#endif
#if FSL_FEATURE_UART_HAS_FIFO
        /**********************************************/
        /* Test the various tx/rx FIFO configuration settings*/
        /**********************************************/
        /* disable the transmitter/receiver*/
        uart_configure_tx_rx_enable(uartInstance, 0);

        /**********************************************/
        /* Test the various tx/rx FIFO enable/disable functions*/
        /**********************************************/
        /* Test the tx fifo enable, transmitter should be disabled when enabling this*/
        if (uart_hal_enable_tx_fifo(uartInstance) != kStatus_Success)
        {
            s_uartTestStatus[uartInstance].B.testStatusTxFifoEnable = 1;
        }
        if (uart_hal_test_is_tx_fifo_enabled(uartInstance) != 1)
        {
            s_uartTestStatus[uartInstance].B.testStatusTxFifoEnable = 1;
        }

        /* Test the tx fifo disable, transmitter should be disabled when disabling this*/
        if (uart_hal_disable_tx_fifo(uartInstance) != kStatus_Success)
        {
            s_uartTestStatus[uartInstance].B.testStatusTxFifoEnable = 1;
        }
        if (uart_hal_test_is_tx_fifo_enabled(uartInstance) != 0)
        {
            s_uartTestStatus[uartInstance].B.testStatusTxFifoEnable = 1;
        }

        /* Test the rx fifo enable, receiver should be disabled when enabling this*/
        if (uart_hal_enable_rx_fifo(uartInstance) != kStatus_Success)
        {
            s_uartTestStatus[uartInstance].B.testStatusRxFifoEnable = 1;
        }
        if (uart_hal_test_is_rx_fifo_enabled(uartInstance) != 1)
        {
            s_uartTestStatus[uartInstance].B.testStatusRxFifoEnable = 1;
        }

        /* Test the rx fifo disable, receiver should be disabled when disabling this*/
        if (uart_hal_disable_rx_fifo(uartInstance) != kStatus_Success)
        {
            s_uartTestStatus[uartInstance].B.testStatusRxFifoEnable = 1;
        }
        if (uart_hal_test_is_rx_fifo_enabled(uartInstance) != 0)
        {
            s_uartTestStatus[uartInstance].B.testStatusRxFifoEnable = 1;
        }

        /* Test the tx fifo interrupt enable*/
        uart_hal_enable_tx_fifo_overflow_interrupt(uartInstance);
        if (uart_hal_is_tx_fifo_overflow_interrupt_enabled(uartInstance) != 1)
        {
            s_uartTestStatus[uartInstance].B.testStatusTxFifoEnable = 1;
        }

        /* Test the tx fifo interrupt disable*/
        uart_hal_disable_tx_fifo_overflow_interrupt(uartInstance);
        if (uart_hal_is_tx_fifo_overflow_interrupt_enabled(uartInstance) != 0)
        {
            s_uartTestStatus[uartInstance].B.testStatusTxFifoEnable = 1;
        }

        /* Test the rx fifo interrupt enable*/
        uart_hal_enable_rx_fifo_underflow_interrupt(uartInstance);
        if (uart_hal_is_rx_fifo_underflow_interrupt_enabled(uartInstance) != 1)
        {
            s_uartTestStatus[uartInstance].B.testStatusRxFifoEnable = 1;
        }

        /* Test the rx fifo interrupt disable*/
        uart_hal_disable_rx_fifo_underflow_interrupt(uartInstance);
        if (uart_hal_is_rx_fifo_underflow_interrupt_enabled(uartInstance) != 0)
        {
            s_uartTestStatus[uartInstance].B.testStatusRxFifoEnable = 1;
        }

        /**********************************************/
        /* Test the tx FIFO Empty, flush, and count*/
        /**********************************************/
        /* get the tx and rx fifo size*/
        txFifoSize[uartInstance] = uart_hal_get_tx_fifo_size(uartInstance);
        rxFifoSize[uartInstance] = uart_hal_get_rx_fifo_size(uartInstance);

        if (txFifoSize[uartInstance] != rxFifoSize[uartInstance])
        {
            s_uartTestStatus[uartInstance].B.testStatusTxFifoSize = 1;
            s_uartTestStatus[uartInstance].B.testStatusRxFifoSize = 1;
        }

        /* now calculate the number of data words per given fifo size*/
        /* just use tx fifo size since tx and rx fifo sizes would normally be the same*/
        uint32_t q;
        uint32_t dataWords;

        dataWords = (txFifoSize[uartInstance] == 0 ? 1 : 0x1 << (txFifoSize[uartInstance] + 1));

        /* Need to enable the fifos before filling them*/
        /* tx fifo enable, transmitter should be disabled when enabling this*/
        if (uart_hal_enable_tx_fifo(uartInstance) != kStatus_Success)
        {
            s_uartTestStatus[uartInstance].B.testStatusTxFifoEnable = 1;
        }
        /* rx fifo enable, receiver should be disabled when enabling this*/
        if (uart_hal_enable_rx_fifo(uartInstance) != kStatus_Success)
        {
            s_uartTestStatus[uartInstance].B.testStatusRxFifoEnable = 1;
        }

        /* fill the tx fifo with data */
        for (q = 0; q < dataWords; q++)
        {
            uart_hal_putchar(uartInstance, 0x41);
        }

        /* now check the tx fifo empty functions to ensure the fifos has data*/
        if (uart_hal_is_tx_fifo_empty(uartInstance) != 0)
        {
            s_uartTestStatus[uartInstance].B.testStatusTxFifo = 1;
        }

        /* now test the tx count to make sure it matches the number of words in the fifo*/
        if (uart_hal_get_tx_dataword_count_in_fifo(uartInstance) != dataWords)
        {
            s_uartTestStatus[uartInstance].B.testStatusTxFifo = 1;
        }

        /* flush the tx fifo, make sure no error code received*/
        if (uart_hal_flush_tx_fifo(uartInstance) != kStatus_Success)
        {
            s_uartTestStatus[uartInstance].B.testStatusTxFifo = 1;
        }

        /* now check the tx fifo empty function to ensure the fifo is empty*/
        if (uart_hal_is_tx_fifo_empty(uartInstance) != 1)
        {
            s_uartTestStatus[uartInstance].B.testStatusTxFifo = 1;
        }

        /**********************************************/
        /* Test the rx FIFO Empty, flush, and count*/
        /**********************************************/
        /* enable loopback mode*/
        uart_hal_configure_loopback_mode(uartInstance, 1);

        /* configure rx fifo watermark here before enabling the receiver*/
        /* dataWords was defined previously*/
        /* Also, make sure when setting the watermark, the receiver is disabled*/
        if (uart_hal_set_rx_fifo_watermark(uartInstance, dataWords) != kStatus_Success)
        {
            s_uartTestStatus[uartInstance].B.testStatusRxFifo = 1;
        }

        /* fill the tx fifo with data */
        for (q = 0; q < dataWords; q++)
        {
            uart_hal_putchar(uartInstance, 0x41);
        }

        /* enable tx/rx to start the transfer*/
        uart_configure_tx_rx_enable(uartInstance, 1);

        /* wait till tx FIFO is empty*/
        while (uart_hal_is_tx_fifo_empty(uartInstance) == 0)
        {
        }

        /* now, wait until the receiver fifo is filled with the dataWords amount*/
        while (uart_hal_is_receive_data_register_full(uartInstance) == 0)
        {
        }

        /* now check the rx fifo empty function to ensure the fifo has data*/
        if (uart_hal_is_rx_fifo_empty(uartInstance) != 0)
        {
            s_uartTestStatus[uartInstance].B.testStatusRxFifo = 1;
        }

        /* now test the rx count to make sure it matches the number of words in the fifo*/
        if (uart_hal_get_rx_dataword_count_in_fifo(uartInstance) != dataWords)
        {
            s_uartTestStatus[uartInstance].B.testStatusRxFifo = 1;
        }

        /* The receiver is still enabled, hence when*/
        /* flushing the rx fifo, error code should be received*/
        /* if not, then this is a failure*/
        if (uart_hal_flush_rx_fifo(uartInstance) == kStatus_Success)
        {
            s_uartTestStatus[uartInstance].B.testStatusRxFifo = 1;
        }

        /* now disable tx/rx */
        uart_configure_tx_rx_enable(uartInstance, 0);

        /*try flushing the rx fifo now that receiver has been disabled*/
        if (uart_hal_flush_rx_fifo(uartInstance) != kStatus_Success)
        {
            s_uartTestStatus[uartInstance].B.testStatusRxFifo = 1;
        }

        /* now check the rx fifo empty function to ensure the fifo is empty*/
        if (uart_hal_is_rx_fifo_empty(uartInstance) != 1)
        {
            s_uartTestStatus[uartInstance].B.testStatusRxFifo = 1;
        }

        /**********************************************/
        /* Test the tx/rx FIFO over/under flow*/
        /**********************************************/
        /* Test the tx overflow */

        /* fill the tx fifo with data */
        for (q = 0; q < dataWords; q++)
        {
            uart_hal_putchar(uartInstance, 0x41);
        }
        /* now put one more data word into the fifo to force an overflow*/
        uart_hal_putchar(uartInstance, 0x41);

        if (uart_hal_is_tx_fifo_overflow(uartInstance) != 1)
        {
            s_uartTestStatus[uartInstance].B.testStatusTxFifo = 1;
        }
        else
        {
            /* now clear the tx fifo overflow bit*/
            uart_hal_clear_tx_fifo_overflow(uartInstance);
            if (uart_hal_is_tx_fifo_overflow(uartInstance) != 0)
            {
                s_uartTestStatus[uartInstance].B.testStatusTxFifo = 1;
            }
        }

        /* Test the rx underflow */
        /* Make sure the debugger memory window is not open on the UART memory space*/
        /* otherwise inadvertant reads may occur fromt he data port not allowing rx underflow to */
        /* clear since no data in the rx fifo, do a read and it should trigger an underflow*/
        uint8_t readData;
        uart_hal_getchar(uartInstance, &readData);

        if (uart_hal_is_rx_fifo_underflow(uartInstance) != 1)
        {
            s_uartTestStatus[uartInstance].B.testStatusRxFifo = 1;
        }
        else
        {
            /* now clear the rx fifo underflow bit*/
            uart_hal_clear_rx_fifo_underflow(uartInstance);
            if (uart_hal_is_rx_fifo_underflow(uartInstance) != 0)
            {
                s_uartTestStatus[uartInstance].B.testStatusRxFifo = 1;
            }
        }

        /**********************************************/
        /* Test the tx/rx FIFO watermarks*/
        /**********************************************/
        /* note watermark value should be less than fifo size, but*/
        /* for testing purposes, set to large value*/
        uint8_t watermark = 0xA5;
        if (uart_hal_set_tx_fifo_watermark(uartInstance, watermark) != kStatus_Success)
        {
            s_uartTestStatus[uartInstance].B.testStatusTxFifo = 1;
        }
        if (uart_hal_get_tx_fifo_watermark(uartInstance) != watermark)
        {
            s_uartTestStatus[uartInstance].B.testStatusTxFifo = 1;
        }

        if (uart_hal_set_rx_fifo_watermark(uartInstance, watermark) != kStatus_Success)
        {
            s_uartTestStatus[uartInstance].B.testStatusRxFifo = 1;
        }
        if (uart_hal_get_rx_fifo_watermark(uartInstance) != watermark)
        {
            s_uartTestStatus[uartInstance].B.testStatusRxFifo = 1;
        }

        /**********************************************/
        /* End of tx/rx FIFO testing*/
        /**********************************************/
        uart_configure_tx_rx_enable(uartInstance, 1);

#endif /* FSL_FEATURE_UART_HAS_FIFO*/

        /**********************************************/
        /* Test send break*/
        /**********************************************/
        /* queue break char to send*/
        uart_hal_queue_break_char_to_send(uartInstance, 1);
        if (uart_hal_test_is_send_break_set(uartInstance) != 1)
        {
            s_uartTestStatus[uartInstance].B.testStatusSendBreak = 1;
        }

        /* clear queue break char to send*/
        uart_hal_queue_break_char_to_send(uartInstance, 0);
        if (uart_hal_test_is_send_break_set(uartInstance) != 0)
        {
            s_uartTestStatus[uartInstance].B.testStatusSendBreak = 1;
        }
    }

    /* Error reporting*/
    for (uartInstance = 0; uartInstance < uartInstanceCount; uartInstance++)
    {
        if (s_uartTestStatus[uartInstance].all)
        {
            /* report failure*/
            while (1)
            {
            }
        }
    }
}

/* create a function that enables/disables tx/rx with one function call*/
static void uart_configure_tx_rx_enable(uint32_t uartInstance, bool enable)
{
    if (enable == 1)
    {
        uart_hal_enable_transmitter(uartInstance);
        uart_hal_enable_receiver(uartInstance);
    }
    else
    {
        uart_hal_disable_transmitter(uartInstance);
        uart_hal_disable_receiver(uartInstance);
    }
}

/* Implement if the fsl_debug_uart is not included in the project */
#if __ICCARM__

size_t __write(int handle, const unsigned char *buf, size_t size)
{
    while (size--)
    {
        uart_hal_putchar(0, *buf++);
    }

    return size;
}

#endif /* __ICCARM__*/

/*******************************************************************************
 * EOF
 ******************************************************************************/

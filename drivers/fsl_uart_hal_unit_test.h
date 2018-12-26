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
#ifndef __FSL_UART_HAL_UNIT_TEST_H__
#define __FSL_UART_HAL_UNIT_TEST_H__

#include "uart/hal/fsl_uart_hal.h"
#include "uart/hal/fsl_uart_features.h"
#include <stdint.h>
#include "bootloader_common.h"

/*!
 * @addtogroup uart_hal_unit_test
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*! @brief Structure for all UART status flags */
typedef union _uart_test_status
{
    uint64_t all;
    struct _uart_test_status_bits
    {
        unsigned testStatusSBNS : 1;                 /*!< Stop Bit Number Select*/
        unsigned testStatusRXEDGIE : 1;              /*!< RX Input Active Edge Interrupt Enable (for RXEDGIF)*/
        unsigned testStatusLBKDIE : 1;               /*!< LIN Break Detect Interrupt Enable (for LBKDIF)*/
        unsigned testStatusParityConfig : 1;         /*!< parity config and associated bits PE and PT*/
        unsigned testStatusIdleLineDetectConfig : 1; /*!< Idle Line detect config and associated */
                                                     /*!  bits ILT and RWUID*/
        unsigned testStatusWAKE : 1;                 /*!< Receiver Wakeup Method Select*/
        unsigned testStatusBitCount : 1;             /*!< Bit count per char, tests M and M10 (uart0 only)*/
        unsigned testStatusLoopbackMode : 1;         /*!< Loopback Mode, associated bits RSRC and LOOPS*/
        unsigned testStatusWaitMode : 1;             /*!< UART operation in wait mode, associated bits */
                                                     /*!  DOZEEN (uart0), UARTSWAI (uart1/2) */
        unsigned testStatusSendBreak : 1;            /*!< Send Break, SBK*/
        unsigned testStatusRxWakeControl : 1;        /*!< Receiver Wakeup Control, RWU*/
        unsigned testStatusRxEnable : 1;             /*!< Receiver Enable, RE*/
        unsigned testStatusTxEnable : 1;             /*!< Transmitter Enable, TE*/
        unsigned testStatusILIE : 1;                 /*!< Idle Line Interrupt Enable for IDLE*/
        unsigned testStatusRIE : 1;                  /*!< Receiver Interrupt Enable for RDRF*/
        unsigned testStatusTCIE : 1;                 /*!< Transmission Complete Interrupt Enable for TC*/
        unsigned testStatusTIE : 1;                  /*!< Transmit Interrupt Enable for TDRE*/
        unsigned testStatusLBKDE : 1;                /*!< LIN Break Detection Enable*/
        unsigned testStatusBRK13 : 1;                /*!< Break Character Generation Length*/
        unsigned testStatusRXINV : 1;                /*!< Receive Data Inversion*/
        unsigned testStatusMSBF : 1;                 /*!< MSB First*/
        unsigned testStatusPEIE : 1;                 /*!< Parity Error Interrupt Enable*/
        unsigned testStatusFEIE : 1;                 /*!< Framing Error Interrupt Enable*/
        unsigned testStatusNEIE : 1;                 /*!< Noise Error Interrupt Enable*/
        unsigned testStatusORIE : 1;                 /*!< Overrun Interrupt Enable*/
        unsigned testStatusTXINV : 1;                /*!< Transmit Data Inversion*/
        unsigned testStatusConfigAllInterrupts : 1;  /*!< status for configuring all interrupts*/
        unsigned testStatusSinglewireMode : 1;       /*!< status for singlewire mode config, */
                                                     /*!  associated bits RSRC and LOOPS*/
        unsigned testStatusTXDIR : 1;                /*!< UART _TX Pin Direction in Single-Wire Mode*/
        unsigned testStatusMA1 : 1;                  /*!< Match Address1*/
        unsigned testStatusMA2 : 1;                  /*!< Match Address2*/
        unsigned testStatusMAEN2 : 1;                /*!< Match Address Mode Enable 2*/
        unsigned testStatusMAEN1 : 1;                /*!< Match Address Mode Enable 1*/
        unsigned testStatusRESYNCDIS : 1;            /*!< Resynchronization Disable*/
        unsigned testStatusRDMAE : 1;                /*!< Receiver Full DMA Enable*/
        unsigned testStatusTDMAE : 1;                /*!< Transmitter DMA Enable*/
        unsigned testStatusBaudRate : 1;             /*!< Baud Rate test status*/
        unsigned testStatusSBR : 1;                  /*!< Baud Rate Divisor test status*/
        unsigned testStatusFlagsCheck : 1;           /*!< Status Flag Check test status*/
        unsigned testStatusFlagsClear : 1;           /*!< Status Flag Clear test status*/
#if FSL_FEATURE_UART_HAS_EXTENDED_DATA_REGISTER_FLAGS
        unsigned testStatusBit10AsParity : 1;   /*!< Status flag for bit10 as parity*/
        unsigned testStatusNoisyFlag : 1;       /*!< Status flag for current word Noisy flag*/
        unsigned testStatusParityErrorFlag : 1; /*!< Status flag for current word Parity error flag*/
#endif
#if FSL_FEATURE_UART_HAS_MODEM_SUPPORT
        unsigned testStatusReceiverRtsEnable : 1;      /*!< Status flag for rx rts enable*/
        unsigned testStatusTransmitterRtsEnable : 1;   /*!< Status flag for tx rts enable*/
        unsigned testStatusTransmitterRtsPolarity : 1; /*!< Status flag for tx rts polarity*/
        unsigned testStatusTransmitterCtsEnable : 1;   /*!< Status flag for tx cts enable*/
#endif
#if FSL_FEATURE_UART_HAS_IR_SUPPORT
        unsigned testStatusInfraredEnable : 1; /*!< Status flag for infrared enable*/
        unsigned testStatusInfraredPulse : 1;  /*!< Status flag for infrared pulse width */
#endif
#if FSL_FEATURE_UART_HAS_FIFO
        unsigned testStatusTxFifoEnable : 1; /*!< Status flag for TX FIFO enable/disable tests*/
        unsigned testStatusRxFifoEnable : 1; /*!< Status flag for RX FIFO enable/disable tests*/
        unsigned testStatusTxFifoSize : 1;   /*!< Status flag for TX FIFO tests*/
        unsigned testStatusRxFifoSize : 1;   /*!< Status flag for RX FIFO tests*/
        unsigned testStatusTxFifo : 1;       /*!< Status flag for TX FIFO tests*/
        unsigned testStatusRxFifo : 1;       /*!< Status flag for RX FIFO tests*/
#endif
#if FSL_FEATURE_UART_HAS_BAUD_RATE_FINE_ADJUST_SUPPORT
        unsigned testStatusBaudRateFineAdjust : 1; /*!< Baud rate fine adjust test status*/
#endif
#if FSL_FEATURE_UART_HAS_BAUD_RATE_OVER_SAMPLING_SUPPORT
        unsigned testStatusBaudOverSampling : 1; /*!< Baud rate oversampling test status*/
#endif
#if FSL_FEATURE_UART_HAS_BOTH_EDGE_SAMPLING_SUPPORT
        unsigned testStatusBaudBothEdgeSampling : 1; /*!< Baud rate both edge sampling test status*/
#endif
    } B;
} uart_test_status_t;

/*******************************************************************************
 * Function Prototypes
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @name UART HAL Unit test
 * @{
 */

void init_hardware(void);

/*!
 * @brief Get the programmed UART baud rate
 *
 * @param   uartInstance    UART instance number.
 * @param   sourceClockInHz      UART source input clock in Hz.
 * @return  The programmed baud rate in bps
 */
uint32_t uart_hal_test_get_baud_rate(uint32_t uartInstance, uint32_t sourceClockInHz);

/*!
 * @brief Get the UART baud rate modulo divisor
 *
 * @param   uartInstance    UART instance number.
 * @return  The value of the UART baud rate modulo divisor
 */
uint32_t uart_hal_test_get_baud_rate_divisor(uint32_t uartInstance);

/*!
 * @brief Get the UART baud rate fine adjust (Note: Feature available on select
 *        UART instances used in conjunction with baud rate programming)
 *
 * @param   uartInstance    UART instance number.
 * @return  The value of the UART baud rate fine adjust
 */
uint8_t uart_hal_test_get_baud_rate_fine_adjust(uint32_t uartInstance);

/*!
 * @brief Get the UART baud rate over sampling ratio (Note: Feature available on select
 *        UART instances used in conjunction with baud rate programming)
 *
 * @param   uartInstance    UART instance number.
 * @return  The value of the UART baud rate oversampling ratio
 */
uint8_t uart_hal_test_get_oversampling_ratio(uint32_t uartInstance);

/*!
 * @brief Get the UART Both Edge Sampling setting (Note: Feature available on select
 *        UART instances used in conjunction with baud rate programming)
 *
 * @param   uartInstance    UART instance number.
 * @return  The value of the UART Both Edge Sampling setting
 */
bool uart_hal_test_get_both_edge_sampling_setting(uint32_t uartInstance);

/*!
 * @brief Get the number of bits per char configured in UART controller
 *
 * @param   uartInstance    UART instance number.
 * @return  Number of bits per char (kUart8BitsPerChar, kUart9BitsPerChar,
 *          or kUart10BitsPerChar, depending on the uart instance)
 */
uart_bit_count_per_char_t uart_hal_test_get_bit_count_per_char_config(uint32_t uartInstance);

/*!
 * @brief Get the parity mode configured in UART controller
 *
 * @param   uartInstance    UART instance number.
 * @return   Either kUartParityDisabled, kUartParityEven, or kUartParityOdd
 */
uart_parity_mode_t uart_hal_test_get_parity_mode_config(uint32_t uartInstance);

/*!
 * @brief Get the number of stop bits configured in UART controller
 *
 * @param   uartInstance    UART instance number.
 * @return   Either kUartOneStopBit or kUartTwoStopBit
 */
uart_stop_bit_count_t uart_hal_test_get_stop_bit_count(uint32_t uartInstance);

/*!
 * @brief Get configuration of the transmit and receive inversion control in UART controller
 *
 * @param   uartInstance    UART instance number.
 * @param   rxInvert     Return configuration: Enable (1) or disable (0) receive inversion
 * @param   txInvert     Return configuration: Enable (1) or disable (0) transmit inversion
 */
void uart_hal_test_get_tx_rx_inversion_config(uint32_t uartInstance, uint32_t *rxInvert, uint32_t *txInvert);

/*!
 * @brief Get configuration of UART loopback operation (enabled/disabled for loopback operation)
 *
 * @param   uartInstance    UART instance number.
 * @return  UART loopback mode - disabled/singlewire (0) or enabled/loopback operation (1)
 */
bool uart_hal_test_is_loopback_operation_config(uint32_t uartInstance);

/*!
 * @brief Get configuration of UART singlewire operation (enabled/disabled for
 *        singlewire operation)
 *
 * @param   uartInstance    UART instance number.
 * @return  UART singlewire mode - disabled/loopback (0) or enabled/singlewire operation (1)
 */
bool uart_hal_test_is_singlewire_operation_config(uint32_t uartInstance);

/*!
 * @brief Get configuration of UART transmit direction
 *  This only applies when the UART is configured for single-wire operation
 *
 * @param   uartInstance    UART instance number.
 * @return  UART single-wire transmit direction - kUartSinglewireTxdirIn or
 *          kUartSinglewireTxdirOut
 */
uart_singlewire_txdir_t uart_hal_test_get_txdir_config(uint32_t uartInstance);

/*!
 * @brief  Get UART idle line detect operation configuration (idle line bit-count start
 *         and wake up affect on IDLE status bit)
 *
 * @param   uartInstance    UART instance number.
 * @param   getConfig        UART configuration data for idle line detect operation
 */
void uart_hal_test_get_idle_line_detect_config(uint32_t uartInstance, uart_idle_line_config_t *getConfig);

/*!
 * @brief  Get UART break character transmit length configuration
 *
 * @param   uartInstance    UART instance number.
 * @return  UART break character (transmit) length setting: kUartBreakChar10BitMinimum or
 *          kUartBreakChar13BitMinimum
 */
uart_break_char_length_t uart_hal_test_get_break_char_transmit_length(uint32_t uartInstance);

/*!
 * @brief  Get UART break character detect length configuration
 *
 * @param   uartInstance    UART instance number.
 * @return  UART break character (detect) length setting: kUartBreakChar10BitMinimum or
 *          kUartBreakChar13BitMinimum
 */
uart_break_char_length_t uart_hal_test_get_break_char_detect_length(uint32_t uartInstance);

/*!
 * @brief  Get UART transmit send break character configuration
 *
 * @param   uartInstance    UART instance number.
 * @return  UART send break char setting - disabled (normal mode, default: 0) or
 *          enabled (queue break char: 1)
 */
bool uart_hal_test_is_send_break_set(uint32_t uartInstance);

/*!
 * @brief  Get UART match address mode2 enable/disable config (Note: Feature available on
 *         select UART instances)
 *
 * @param   uartInstance    UART instance number.
 * @return  The configuration of enable/disable for address mode config (will always return 0
 *          for non-supported uarts)
 */
bool uart_hal_test_is_match_address_mode2_enabled(uint32_t uartInstance);

/*!
 * @brief  Get UART match address mode1 enable/disable config (Note: Feature available on
 *         select UART instances)
 *
 * @param   uartInstance    UART instance number.
 * @return  The configuration of enable/disable for address mode config (will always return 0
 *          for non-supported uarts)
 */
bool uart_hal_test_is_match_address_mode1_enabled(uint32_t uartInstance);

/*!
 * @brief  Get UART match address register value (Note: Feature available on
 *         select UART instances)
 *
 * @param   uartInstance    UART instance number.
 * @param   maRegister    UART match address register number (1 or 2).
 * @return  The value of the desired match address register, will also return 0 if
 *          unsupported uart or register number
 */
uint8_t uart_hal_test_get_match_address_register(uint32_t uartInstance, uint32_t maRegister);

/*!
 * @brief  Get UART send msb first config (enabled/disabled (Note: Feature available on
 *         select UART instances)
 *
 * @param   uartInstance    UART instance number.
 * @return  State of the send msb first enable/disable configuration (for un-supported uarts,
 *          will return 0)
 */
bool uart_hal_test_is_send_msb_first_operation_enabled(uint32_t uartInstance);

/*!
 * @brief  Get the UART disable resync of received data configuration
 *         (Note: Feature available on select UART instances)
 *
 * @param   uartInstance    UART instance number.
 * @return  State of the resynchronization of the received data disable bit
 *          (disabled: 1, non-disabled: 0)
 */
bool uart_hal_test_is_receive_resync_disabled(uint32_t uartInstance);

/*!
 * @brief  Get the configuration of the receiver request-to-send enable
 *
 * @param   uartInstance    UART instance number.
 * @return  The state of the receiver request-to-send enable bit
 */
bool uart_hal_test_is_receiver_rts_enabled(uint32_t uartInstance);

/*!
 * @brief  Get the configuration of the transmitter request-to-send enable
 *
 * @param   uartInstance    UART instance number.
 * @return  The state of the transmitter request-to-send enable bit
 */
bool uart_hal_test_is_transmitter_rts_enabled(uint32_t uartInstance);

/*!
 * @brief  Get the configuration of the transmitter rts polarity: 0=active low, 1=active high
 *
 * @param   uartInstance    UART instance number.
 * @return  The state of the transmitter transmitter rts polarity: 0=active low, 1=active high
 */
bool uart_hal_test_get_transmitter_rts_polarity_config(uint32_t uartInstance);

/*!
 * @brief  Get the configuration of the transmitter clear-to-send enable
 *
 * @param   uartInstance    UART instance number.
 * @return  The state of the transmitter clear-to-send enable bit
 */
bool uart_hal_test_is_transmitter_cts_enabled(uint32_t uartInstance);

/*!
 * @brief  Get the configuration of the UART infrared enable
 *
 * @param   uartInstance    UART instance number.
 * @return  The state of the UART infrared enable (1) or disable (0)
 */
bool uart_hal_test_is_infrared_enabled(uint32_t uartInstance);

/*!
 * @brief  Get the configuration of the UART infrared narrow pulse width
 *
 * @param   uartInstance    UART instance number.
 * @return  The transmit narrow pulse width of type uart_ir_tx_pulsewidth_t
 */
uart_ir_tx_pulsewidth_t uart_hal_test_get_infrared_pulse_config(uint32_t uartInstance);

/*!
 * @brief  Get the configuration of the UART transmit FIFO enable
 *
 * @param   uartInstance    UART instance number.
 * @return  The transmit FIFO enable state, 1=enable; 0=disable
 */
bool uart_hal_test_is_tx_fifo_enabled(uint32_t uartInstance);

/*!
 * @brief  Get the configuration of the UART receive FIFO enable
 *
 * @param   uartInstance    UART instance number.
 * @return  The receive FIFO enable state, 1=enable; 0=disable
 */
bool uart_hal_test_is_rx_fifo_enabled(uint32_t uartInstance);
/*@}  */

/*! @}*/

#endif /* __FSL_UART_HAL_UNIT_TEST_H__*/
       /*******************************************************************************
        * EOF
        ******************************************************************************/

/*
 * The Clear BSD License
 * Copyright (c) 2014, Freescale Semiconductor, Inc.
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
#if !defined(__FSL_LPSPI_HAL_H__)
#define __FSL_LPSPI_HAL_H__

#include "bootloader_common.h"
#include "lpspi/fsl_lpspi_types.h"
#include "fsl_device_registers.h"
#include <stdint.h>
#include <stdbool.h>

//! @addtogroup lpspi_hal
//! @{

//! @file

////////////////////////////////////////////////////////////////////////////////
// Definitions
////////////////////////////////////////////////////////////////////////////////

/*! @brief LPSPI status flags. */
typedef enum _lpspi_status_flag
{
    kLPSPITransmitData = LPSPI_SR_TDF_SHIFT,
    kLPSPIReceiveData = LPSPI_SR_RDF_SHIFT,
    kLPSPIWordComplete = LPSPI_SR_WCF_SHIFT,
    kLPSPIFrameComplete = LPSPI_SR_FCF_SHIFT,
    kLPSPITransferComplete = LPSPI_SR_TCF_SHIFT,
    kLPSPITransmitError = LPSPI_SR_TEF_SHIFT,
    kLPSPIReceiveError = LPSPI_SR_REF_SHIFT,
    kLPSPIModuleBusy = LPSPI_SR_MBF_SHIFT
} lpspi_status_flag_t;

//! @brief LPSPI hardware configuration settings.
//!
//! Use an instance of this struct with lpspi_hal_init(). This allows you to configure the
//! most common settings of the LPSPI peripheral with a single function call.
//!
//! The @c kbitsPerSec member is handled specially. If this value is set to 0, then the baud is
//! not set by lpspi_hal_init(), and must be set with a separate call to either lpspi_hal_set_baud()
//! or lpspi_hal_set_baud_divisors(). This can be useful if you know the divisors in advance and
//! don't want to spend the time to compute them for the provided rate in kilobits/sec.
typedef struct LpspiConfig
{
    bool isEnabled;                          //!< Set to true to enable the LPSPI peripheral.
    uint32_t kbitsPerSec;                    //!< @brief Baud rate in kilobits per second.
    lpspi_master_slave_mode_t masterOrSlave; //!< Whether to put the peripheral in master or slave mode.
    lpspi_clock_polarity_t polarity;         //!< Clock polarity setting.
    lpspi_clock_phase_t phase;               //!< Clock phase setting.
    lpspi_shift_direction_t shiftDirection;  //!< Direction in which data is shifted out.
    lpspi_ss_output_mode_t ssOutputMode;     //!< Output mode for the slave select signal.
    lpspi_pin_mode_t pinMode;                //!< Pin mode with bidirectional option.
    bool enableReceiveAndFaultInterrupt;     //!< Enable for the receive and fault interrupt.
    bool enableTransmitInterrupt;            //!< Enable for the transmit interrupt.
    bool enableMatchInterrupt;               //!< Enable for the match interrupt.
} lpspi_config_t;

////////////////////////////////////////////////////////////////////////////////
// API
////////////////////////////////////////////////////////////////////////////////

#if defined(__cplusplus)
extern "C" {
#endif

//! @name Configuration
//@{

/*!
 * @brief Restore LPSPI to reset configuration.
 */
void lpspi_hal_reset(LPSPI_Type *baseAddr);

/*!
 * @brief Enable the LPSPI peripheral.
 */
static inline void lpspi_hal_enable(LPSPI_Type *baseAddr)
{
    // HW_LPSPI_CR_SET(baseAddr, BM_LPSPI_CR_MEN);

    baseAddr->CR |= LPSPI_CR_DBGEN_MASK;
    baseAddr->CR |= LPSPI_CR_MEN_MASK;
}

/*!
 * @brief Disable the LPSPI peripheral.
 */
static inline void lpspi_hal_disable(LPSPI_Type *baseAddr)
{
    baseAddr->CR &= ~LPSPI_CR_MEN_MASK;
}

/*!
 * @brief Configure LPSPI for master or slave.
 */
static inline void lpspi_hal_set_master_slave(LPSPI_Type *baseAddr, lpspi_master_slave_mode_t mode)
{
    baseAddr->CFGR1 = ((~LPSPI_CFGR1_MASTER_MASK) & baseAddr->CFGR1) | LPSPI_CFGR1_MASTER(mode);
}

/*!
 * @brief Set the polarity, phase, shift direction, and frame size.
 */
void lpspi_hal_set_data_format(LPSPI_Type *baseAddr,
                               lpspi_clock_polarity_t polarity,
                               lpspi_clock_phase_t phase,
                               lpspi_chip_selection_t selection,
                               lpspi_shift_direction_t direction,
                               uint32_t frameSize);

//@}

//! @name Low power
//@{

//@}

//! @name Interrupts
//@{

/*!
 * @brief Enable the receive buffer full interrupt.
 */
static inline void lpspi_hal_enable_receive_interrupt(LPSPI_Type *baseAddr)
{
    baseAddr->IER |= LPSPI_IER_RDIE_MASK;
}

/*!
 * @brief Disable the receive buffer full interrupt.
 */
static inline void lpspi_hal_disable_receive_interrupt(LPSPI_Type *baseAddr)
{
    baseAddr->IER &= ~LPSPI_IER_RDIE_MASK;
}

/*!
 * @brief Enable the transmit buffer empty interrupt.
 */
static inline void lpspi_hal_enable_transmit_interrupt(LPSPI_Type *baseAddr)
{
    baseAddr->IER |= LPSPI_IER_TDIE_MASK;
}

/*!
 * @brief Disable the transmit buffer empty interrupt.
 */
static inline void lpspi_hal_disable_transmit_interrupt(LPSPI_Type *baseAddr)
{
    baseAddr->IER &= ~LPSPI_IER_TDIE_MASK;
}

//@}

//! @name Status
//@{

/*!
 * @brief Check if the read buffer is full.
 */
static inline bool lpspi_hal_is_read_buffer_full(LPSPI_Type *baseAddr)
{
    return (baseAddr->SR & LPSPI_SR_RDF_MASK) >> LPSPI_SR_RDF_SHIFT;
}

/*!
 * @brief Check if the transmit buffer is empty.
 */
static inline bool lpspi_hal_is_transmit_buffer_empty(LPSPI_Type *baseAddr)
{
    return (baseAddr->SR & LPSPI_SR_TDF_MASK) >> LPSPI_SR_TDF_SHIFT;
}

/*!
 * @brief Check if a frame transfer is complete.
 */
static inline bool lpspi_hal_is_frame_transfer_complete(LPSPI_Type *baseAddr)
{
    return (baseAddr->SR & LPSPI_SR_FCF_MASK) >> LPSPI_SR_FCF_SHIFT;
}

/*!
 * @brief Check if a word transfer is complete.
 */
static inline bool lpspi_hal_is_word_transfer_complete(LPSPI_Type *baseAddr)
{
    return (baseAddr->SR & LPSPI_SR_WCF_MASK) >> LPSPI_SR_WCF_SHIFT;
}

//@}

//! @name Data transfer
//@{

/*!
 * @brief Read a byte from the data buffer.
 */
static inline uint8_t lpspi_hal_read_byte(LPSPI_Type *baseAddr)
{
    return (uint8_t)baseAddr->RDR;
}

/*!
 * @brief Write a byte into the data buffer.
 */
static inline void lpspi_hal_write_byte(LPSPI_Type *baseAddr, uint8_t data)
{
    baseAddr->TDR = data;
}

//@}

//! @name Match byte
//@{

//@}

#if defined(__cplusplus)
}
#endif

//! @}

#endif // __FSL_LPSPI_HAL_H__
////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////

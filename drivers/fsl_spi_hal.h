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
#if !defined(__FSL_SPI_HAL_H__)
#define __FSL_SPI_HAL_H__

#include "bootloader_common.h"
#include "fsl_device_registers.h"

#include "spi/fsl_spi_types.h"
#include <stdint.h>
#include <stdbool.h>

//! @addtogroup spi_hal
//! @{

//! @file

////////////////////////////////////////////////////////////////////////////////
// Definitions
////////////////////////////////////////////////////////////////////////////////

//! @brief SPI hardware configuration settings.
//!
//! Use an instance of this struct with spi_hal_init(). This allows you to configure the
//! most common settings of the SPI peripheral with a single function call.
//!
//! The @c kbitsPerSec member is handled specially. If this value is set to 0, then the baud is
//! not set by spi_hal_init(), and must be set with a separate call to either spi_hal_set_baud()
//! or spi_hal_set_baud_divisors(). This can be useful if you know the divisors in advance and
//! don't want to spend the time to compute them for the provided rate in kilobits/sec.
typedef struct SpiConfig
{
    bool isEnabled;                        //!< Set to true to enable the SPI peripheral.
    uint32_t kbitsPerSec;                  //!< @brief Baud rate in kilobits per second.
    spi_master_slave_mode_t masterOrSlave; //!< Whether to put the peripheral in master or slave mode.
    spi_clock_polarity_t polarity;         //!< Clock polarity setting.
    spi_clock_phase_t phase;               //!< Clock phase setting.
    spi_shift_direction_t shiftDirection;  //!< Direction in which data is shifted out.
    spi_ss_output_mode_t ssOutputMode;     //!< Output mode for the slave select signal.
    spi_pin_mode_t pinMode;                //!< Pin mode with bidirectional option.
    bool enableReceiveAndFaultInterrupt;   //!< Enable for the receive and fault interrupt.
    bool enableTransmitInterrupt;          //!< Enable for the transmit interrupt.
    bool enableMatchInterrupt;             //!< Enable for the match interrupt.
} spi_config_t;

////////////////////////////////////////////////////////////////////////////////
// API
////////////////////////////////////////////////////////////////////////////////

#if defined(__cplusplus)
extern "C" {
#endif

//! @name Configuration
//@{

/*!
 * @brief Configure the SPI peripheral.
 */
void spi_hal_init(SPI_Type *baseAddr, const spi_config_t *config);

/*!
 * @brief Restore SPI to reset configuration.
 */
void spi_hal_reset(SPI_Type *baseAddr);

/*!
 * @brief Enable the SPI peripheral.
 */
static inline void spi_hal_enable(SPI_Type *baseAddr)
{
    SPI_SET_C1(baseAddr, SPI_C1_SPE_MASK);
}

/*!
 * @brief Disable the SPI peripheral.
 */
static inline void spi_hal_disable(SPI_Type *baseAddr)
{
    SPI_CLR_C1(baseAddr, SPI_C1_SPE_MASK);
}

/*!
 * @brief Set the SPI baud rate in kilobits per second.
 */
void spi_hal_set_baud(SPI_Type *baseAddr, uint32_t kbitsPerSec, uint32_t busClock);

/*!
 * @brief Configure the baud rate divisors manually.
 */
static inline void spi_hal_set_baud_divisors(SPI_Type *baseAddr, uint32_t prescaleDivisor, uint32_t rateDivisor)
{
    SPI_WR_BR(baseAddr, SPI_BR_SPR(rateDivisor) | SPI_BR_SPPR(prescaleDivisor));
}

/*!
 * @brief Configure SPI for master or slave.
 */
static inline void spi_hal_set_master_slave(SPI_Type *baseAddr, spi_master_slave_mode_t mode)
{
    SPI_BWR_C1_MSTR(baseAddr, (uint32_t)mode);
}

/*!
 * @brief Set how the slave select output operates.
 */
void spi_hal_set_slave_select_output_mode(SPI_Type *baseAddr, spi_ss_output_mode_t mode);

/*!
 * @brief Set the polarity, phase, and shift direction.
 */
void spi_hal_set_data_format(SPI_Type *baseAddr,
                             spi_clock_polarity_t polarity,
                             spi_clock_phase_t phase,
                             spi_shift_direction_t direction);

/*!
 * @brief Set the SPI pin mode.
 */
void spi_hal_set_pin_mode(SPI_Type *baseAddr, spi_pin_mode_t mode);

//@}

#if FSL_FEATURE_SPI_DMA
//! @name DMA
//@{

/*!
 * @brief Configure transmit and receive DMA requests.
 */
void spi_hal_configure_dma(SPI_Type *baseAddr, bool enableTransmit, bool enableReceive);

//@}
#endif // FSL_FEATURE_SPI_DMA

//! @name Low power
//@{

/*!
 * @brief Enable or disable the SPI clock to stop when the CPU enters wait mode.
 */
static inline void spi_hal_configure_stop_in_wait_mode(SPI_Type *baseAddr, bool enable)
{
    SPI_BWR_C2_SPISWAI(baseAddr, (enable == true));
}

//@}

//! @name Interrupts
//@{

/*!
 * @brief Enable the receive buffer full and mode fault interrupt.
 */
static inline void spi_hal_enable_receive_and_fault_interrupt(SPI_Type *baseAddr)
{
    SPI_SET_C1(baseAddr, SPI_C1_SPIE_MASK);
}

/*!
 * @brief Disable the receive buffer full and mode fault interrupt.
 */
static inline void spi_hal_disable_receive_and_fault_interrupt(SPI_Type *baseAddr)
{
    SPI_CLR_C1(baseAddr, SPI_C1_SPIE_MASK);
}

/*!
 * @brief Enable the transmit buffer empty interrupt.
 */
static inline void spi_hal_enable_transmit_interrupt(SPI_Type *baseAddr)
{
    SPI_SET_C1(baseAddr, SPI_C1_SPTIE_MASK);
}

/*!
 * @brief Disable the transmit buffer empty interrupt.
 */
static inline void spi_hal_disable_transmit_interrupt(SPI_Type *baseAddr)
{
    SPI_CLR_C1(baseAddr, SPI_C1_SPTIE_MASK);
}

/*!
 * @brief Enable the match interrupt.
 */
static inline void spi_hal_enable_match_interrupt(SPI_Type *baseAddr)
{
    SPI_SET_C2(baseAddr, SPI_C2_SPMIE_MASK);
}

/*!
 * @brief Disable the match interrupt.
 */
static inline void spi_hal_disable_match_interrupt(SPI_Type *baseAddr)
{
    SPI_CLR_C2(baseAddr, SPI_C2_SPMIE_MASK);
}

//@}

//! @name Status
//@{

/*!
 * @brief Check if the read buffer is full.
 *
 * The read buffer full flag is only cleared by reading it when it is set, then reading the
 * data register by calling spi_hal_read_data(). This example code demonstrates how to check
 * the flag, read data, and clear the flag.
 * @code
 *      // Check read buffer flag.
 *      if (spi_hal_is_read_buffer_full(0))
 *      {
 *          // Read the data in the buffer, which also clears the flag.
 *          byte = spi_hal_read_data(0);
 *      }
 * @endcode
 */
static inline bool spi_hal_is_read_buffer_full(SPI_Type *baseAddr)
{
    return SPI_RD_S_SPRF(baseAddr);
}

/*!
 * @brief Check if the transmit buffer is empty.
 *
 * To clear the transmit buffer empty flag, you must first read the flag when it is set. Then
 * write a new data value into the transmit buffer with a call to spi_hal_write_data(). The
 * following code shows how to do this.
 * @code
 *      // Check if transmit buffer is empty.
 *      if (spi_hal_is_transmit_buffer_empty(0))
 *      {
 *          // Buffer has room, so write the next data value.
 *          spi_hal_write_data(0, byte);
 *      }
 * @endcode
 */
static inline bool spi_hal_is_transmit_buffer_empty(SPI_Type *baseAddr)
{
    return SPI_RD_S_SPTEF(baseAddr);
}

/*!
 * @brief Check if there has been a mode fault.
 */
static inline bool spi_hal_is_mode_fault(SPI_Type *baseAddr)
{
    return SPI_RD_S_MODF(baseAddr);
}

/*!
 * @brief Clear the mode fault flag.
 */
void spi_hal_clear_mode_fault(SPI_Type *baseAddr);

/*!
 * @brief Check if the data received matches the previously set match value.
 */
static inline bool spi_hal_is_match(SPI_Type *baseAddr)
{
    return (bool)SPI_RD_S_SPMF(baseAddr);
}

/*!
 * @brief Clear the match flag.
 */
void spi_hal_clear_match(SPI_Type *baseAddr);

//@}

//! @name Data transfer
//@{

/*!
 * @brief Read a byte from the data buffer.
 */
static inline uint8_t spi_hal_read_data(SPI_Type *baseAddr)
{
#if FSL_FEATURE_SPI_16BIT_TRANSFERS
    return SPI_RD_DL(baseAddr);
#else  // FSL_FEATURE_SPI_16BIT_TRANSFERS
    return SPI_RD_D(baseAddr);
#endif // FSL_FEATURE_SPI_16BIT_TRANSFERS
}

/*!
 * @brief Write a byte into the data buffer.
 */
static inline void spi_hal_write_data(SPI_Type *baseAddr, uint8_t data)
{
#if FSL_FEATURE_SPI_16BIT_TRANSFERS
    SPI_WR_DL(baseAddr, data);
#else  // FSL_FEATURE_SPI_16BIT_TRANSFERS
    SPI_WR_D(baseAddr, data);
#endif // FSL_FEATURE_SPI_16BIT_TRANSFERS
}

//@}

//! @name Match byte
//@{

/*!
 * @brief Set the value which will trigger the match interrupt.
 */
static inline void spi_hal_set_match_value(SPI_Type *baseAddr, uint8_t matchByte)
{
#if FSL_FEATURE_SPI_16BIT_TRANSFERS
    SPI_WR_ML(baseAddr, matchByte);
#else  // FSL_FEATURE_SPI_16BIT_TRANSFERS
    SPI_WR_M(baseAddr, matchByte);
#endif // FSL_FEATURE_SPI_16BIT_TRANSFERS
}

//@}

#if defined(__cplusplus)
}
#endif

//! @}

#endif // __FSL_SPI_HAL_H__
////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////

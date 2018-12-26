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
#if !defined(__FSL_SPI_TYPES_H__)
#define __FSL_SPI_TYPES_H__

#include "bootloader_common.h"
#include "fsl_device_registers.h"
#include <stdint.h>
#include <stdbool.h>

//! @addtogroup spi_types
//! @{

////////////////////////////////////////////////////////////////////////////////
// Definitions
////////////////////////////////////////////////////////////////////////////////

//! @brief Error codes for the SPI driver.
enum _spi_errors
{
    kStatus_SPI_SlaveTxUnderrun = MAKE_STATUS(kStatusGroup_SPIDriver, 0), //!< SPI Slave TX Underrun error.
    kStatus_SPI_SlaveRxOverrun = MAKE_STATUS(kStatusGroup_SPIDriver, 1),  //!< SPI Slave RX Overrun error.
    kStatus_SPI_Timeout = MAKE_STATUS(kStatusGroup_SPIDriver, 2),         //!< SPI tranfser timed out.
    kStatus_SPI_Busy = MAKE_STATUS(kStatusGroup_SPIDriver, 3), //!< SPI instance is already busy performing a transfer.
    kStatus_SPI_NoTransferInProgress =
        MAKE_STATUS(kStatusGroup_SPIDriver, 4) //!< Attempt to abort a transfer when no transfer was in progress.
};

//! @brief SPI master or slave configuration.
typedef enum _spi_master_slave_mode
{
    kSpiMaster = 1, //!< SPI peripheral operates in master mode.
    kSpiSlave = 0   //!< SPI peripheral operates in slave mode.
} spi_master_slave_mode_t;

//! @brief SPI clock polarity configuration.
typedef enum _spi_clock_polarity
{
    kSpiClockPolarity_ActiveHigh = 0, //!< Active-high SPI clock (idles low).
    kSpiClockPolarity_ActiveLow = 1   //!< Active-low SPI clock (idles high).
} spi_clock_polarity_t;

//! @brief SPI clock phase configuration.
typedef enum _spi_clock_phase
{
    kSpiClockPhase_FirstEdge = 0, //!< First edge on SPSCK occurs at the middle of the first cycle of a data transfer.
    kSpiClockPhase_SecondEdge = 1 //!< First edge on SPSCK occurs at the start of the first cycle of a data transfer.
} spi_clock_phase_t;

//! @brief SPI data shifter direction options.
typedef enum _spi_shift_direction
{
    kSpiMsbFirst = 0, //!< Data transfers start with most significant bit.
    kSpiLsbFirst = 1  //!< Data transfers start with least significant bit.
} spi_shift_direction_t;

//! @brief SPI slave select output mode options.
typedef enum _spi_ss_output_mode
{
    kSpiSlaveSelect_AsGpio = 0,         //!< Slave select pin configured as GPIO.
    kSpiSlaveSelect_FaultInput = 2,     //!< Slave select pin configured for fault detection.
    kSpiSlaveSelect_AutomaticOutput = 3 //!< Slave select pin configured for automatic SPI output.
} spi_ss_output_mode_t;

//! @brief SPI pin mode options.
typedef enum _spi_pin_mode
{
    kSpiPinMode_Normal = 0, //!< Pins operate in normal, single-direction mode.
    kSpiPinMode_Input = 1,  //!< Bidirectional mode. Master: MOSI pin is input; Slave: MISO pin is input
    kSpiPinMode_Output = 3  //!< Bidirectional mode. Master: MOSI pin is output; Slave: MISO pin is output
} spi_pin_mode_t;

//! @}

#endif // __FSL_SPI_TYPES_H__
////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////

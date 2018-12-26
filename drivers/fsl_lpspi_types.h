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
#if !defined(__FSL_LPSPI_TYPES_H__)
#define __FSL_LPSPI_TYPES_H__

#include "bootloader_common.h"
#include "fsl_device_registers.h"
#include <stdint.h>
#include <stdbool.h>

//! @addtogroup lpspi_types
//! @{

////////////////////////////////////////////////////////////////////////////////
// Definitions
////////////////////////////////////////////////////////////////////////////////

//! @brief Error codes for the SPI driver.
enum _lpspi_errors
{
    kStatus_LPSPI_SlaveTxUnderrun = MAKE_STATUS(kStatusGroup_SPIDriver, 0), //!< LPSPI Slave TX Underrun error.
    kStatus_LPSPI_SlaveRxOverrun = MAKE_STATUS(kStatusGroup_SPIDriver, 1),  //!< LPSPI Slave RX Overrun error.
    kStatus_LPSPI_Timeout = MAKE_STATUS(kStatusGroup_SPIDriver, 2),         //!< LPSPI tranfser timed out.
    kStatus_LPSPI_Busy =
        MAKE_STATUS(kStatusGroup_SPIDriver, 3), //!< LPSPI instance is already busy performing a transfer.
    kStatus_LPSPI_NoTransferInProgress =
        MAKE_STATUS(kStatusGroup_SPIDriver, 4) //!< Attempt to abort a transfer when no transfer was in progress.
};

//! @brief LPSPI master or slave configuration.
typedef enum _lpspi_master_slave_mode
{
    kLpspiMaster = 1, //!< SPI peripheral operates in master mode.
    kLpspiSlave = 0   //!< SPI peripheral operates in slave mode.
} lpspi_master_slave_mode_t;

//! @brief LPSPI clock polarity configuration.
typedef enum _lpspi_clock_polarity
{
    kLpspiClockPolarity_ActiveHigh = 0, //!< Active-high SPI clock (idles low).
    kLpspiClockPolarity_ActiveLow = 1   //!< Active-low SPI clock (idles high).
} lpspi_clock_polarity_t;

//! @brief LPSPI clock phase configuration.
typedef enum _lpspi_clock_phase
{
    kLpspiClockPhase_FirstEdge = 0, //!< First edge on SPSCK occurs at the middle of the first cycle of a data transfer.
    kLpspiClockPhase_SecondEdge = 1 //!< First edge on SPSCK occurs at the start of the first cycle of a data transfer.
} lpspi_clock_phase_t;

//! @brief LPSPI chip select configuration.
typedef enum _lpspi_chip_selection
{
    kLpspiChipSelection_PCS0 = 0, //!< Transfer using PCS0.
    kLpspiChipSelection_PCS1 = 1, //!< Transfer using PCS1.
    kLpspiChipSelection_PCS2 = 2, //!< Transfer using PCS2.
    kLpspiChipSelection_PCS3 = 3  //!< Transfer using PCS3.
} lpspi_chip_selection_t;

//! @brief LPSPI data shifter direction options.
typedef enum _lpspi_shift_direction
{
    kLpspiMsbFirst = 0, //!< Data transfers start with most significant bit.
    kLpspiLsbFirst = 1  //!< Data transfers start with least significant bit.
} lpspi_shift_direction_t;

//! @brief LPSPI slave select output mode options.
typedef enum _lpspi_ss_output_mode
{
    kLpspiSlaveSelect_AsGpio = 0,         //!< Slave select pin configured as GPIO.
    kLpspiSlaveSelect_FaultInput = 2,     //!< Slave select pin configured for fault detection.
    kLpspiSlaveSelect_AutomaticOutput = 3 //!< Slave select pin configured for automatic SPI output.
} lpspi_ss_output_mode_t;

//! @brief LPSPI pin mode options.
typedef enum _lpspi_pin_mode
{
    kLpspiPinMode_Normal = 0, //!< Pins operate in normal, single-direction mode.
    kLpspiPinMode_Input = 1,  //!< Bidirectional mode. Master: MOSI pin is input; Slave: MISO pin is input
    kLpspiPinMode_Output = 3  //!< Bidirectional mode. Master: MOSI pin is output; Slave: MISO pin is output
} lpspi_pin_mode_t;

//! @}

#endif // __FSL_LPSPI_TYPES_H__
////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////

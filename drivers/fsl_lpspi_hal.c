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

#include "bootloader/bootloader.h"
#include "fsl_lpspi_hal.h"
#include "fsl_device_registers.h"

////////////////////////////////////////////////////////////////////////////////
// Definitions
////////////////////////////////////////////////////////////////////////////////

//! @brief Bit offsets for bits encoded in enum values.
/*
enum _lpspi_pin_bit_encodings
{
    kSpiSsoeBit = 0U,    //!< SSOE is bit 0 of #spi_ss_output_mode_t.
    kSpiModfenBit = 1U,  //!< MODFEN is bit 1 of #spi_ss_output_mode_t.
    kSpiSpc0Bit = 0U,    //!< SPC0 is bit 0 of #spi_pin_mode_t.
    kSpiBidiroeBit = 1U  //!< BIDIROE is bit 1 of #spi_pin_mode_t.
};
*/
////////////////////////////////////////////////////////////////////////////////
// Variables
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
// Code
////////////////////////////////////////////////////////////////////////////////

// See fsl_lpspi_hal.h for documentation of this function.
void lpspi_hal_reset(LPSPI_Type *baseAddr)
{
    // Soft Reset can reset all registers, except the Control Register, So
    //  we need to clear CR manually.
    baseAddr->CR = ((~LPSPI_CR_RST_MASK) & baseAddr->CR) | LPSPI_CR_RST(1);
    baseAddr->CR = 0;
}

// See fsl_lpspi_hal.h for documentation of this function.
void lpspi_hal_set_data_format(LPSPI_Type *baseAddr,
                               lpspi_clock_polarity_t polarity,
                               lpspi_clock_phase_t phase,
                               lpspi_chip_selection_t selection,
                               lpspi_shift_direction_t direction,
                               uint32_t frameSize)
{
    // From RM : "Writes to TCR will push the the data into the transmit FIFO
    //  in the order they are written", so TCR register is actually Write Only.
    // We must configure a command word in TCR once, can not separate the same command
    //  into multiple configurations for TCR.
    uint32_t tcr = 0;
    tcr |= LPSPI_TCR_CPOL(polarity);
    tcr |= LPSPI_TCR_CPHA(phase);
    tcr |= LPSPI_TCR_PCS(selection);
    tcr |= LPSPI_TCR_LSBF(direction);
    tcr |= LPSPI_TCR_FRAMESZ(frameSize);
    baseAddr->TCR = tcr;
}

////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////

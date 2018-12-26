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
#include "fsl_spi_hal.h"
#include "fsl_device_registers.h"

////////////////////////////////////////////////////////////////////////////////
// Definitions
////////////////////////////////////////////////////////////////////////////////

//! @brief Bit offsets for bits encoded in enum values.
enum _spi_pin_bit_encodings
{
    kSpiSsoeBit = 0U,   //!< SSOE is bit 0 of #spi_ss_output_mode_t.
    kSpiModfenBit = 1U, //!< MODFEN is bit 1 of #spi_ss_output_mode_t.
    kSpiSpc0Bit = 0U,   //!< SPC0 is bit 0 of #spi_pin_mode_t.
    kSpiBidiroeBit = 1U //!< BIDIROE is bit 1 of #spi_pin_mode_t.
};

////////////////////////////////////////////////////////////////////////////////
// Variables
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
// Code
////////////////////////////////////////////////////////////////////////////////

// See fsl_spi_hal.h for documentation of this function.
void spi_hal_init(SPI_Type *baseAddr, const spi_config_t *config)
{
    // Enable or disable the module.
    SPI_BWR_C1_SPE(baseAddr, config->isEnabled == true);

    // Set master or slave moe.
    spi_hal_set_master_slave(baseAddr, config->masterOrSlave);

    // Set data format.
    spi_hal_set_data_format(baseAddr, config->polarity, config->phase, config->shiftDirection);

    // Set output and pin modes.
    spi_hal_set_slave_select_output_mode(baseAddr, config->ssOutputMode);
    spi_hal_set_pin_mode(baseAddr, config->pinMode);

    // Enable requested interrupts.
    SPI_BWR_C1_SPIE(baseAddr, config->enableReceiveAndFaultInterrupt == true);
    SPI_BWR_C1_SPTIE(baseAddr, config->enableTransmitInterrupt == true);
    SPI_BWR_C2_SPMIE(baseAddr, config->enableMatchInterrupt == true);
}

// See fsl_spi_hal.h for documentation of this function.
void spi_hal_reset(SPI_Type *baseAddr)
{
    // Restore those control and configuration registers which are used to
    // be operated
    SPI_WR_C1(baseAddr, SPI_C1_CPHA_MASK);
    SPI_WR_C2(baseAddr, 0);
    SPI_WR_BR(baseAddr, 0);

#if FSL_FEATURE_SPI_16BIT_TRANSFERS
    SPI_WR_ML(baseAddr, 0);
#else  // FSL_FEATURE_SPI_16BIT_TRANSFERS
    SPI_WR_M(baseAddr, 0);
#endif // FSL_FEATURE_SPI_16BIT_TRANSFERS
}

// See fsl_spi_hal.h for documentation of this function.
void spi_hal_set_baud(SPI_Type *baseAddr, uint32_t kbitsPerSec, uint32_t busClock)
{
    uint32_t hz = kbitsPerSec * 1000U;
    uint32_t leastError = 0xffffffffU;
    uint8_t refSpr = 0U;
    uint8_t refPrescaler = 0U;

    uint8_t prescaler = 1U;
    uint8_t divisor = 2U;

    for (prescaler = 1U; prescaler <= 8U; prescaler++)
    {
        divisor = 2U;

        uint8_t spr = 0U;
        for (spr = 0U; spr <= 8U; spr++)
        {
            uint32_t ref = busClock / (prescaler * divisor);
            uint32_t error = (ref > hz) ? ref - hz : hz - ref;
            if (error < leastError)
            {
                refSpr = spr;
                refPrescaler = prescaler - 1U;
                leastError = error;
            }
            divisor *= 2U;
        }
    }

    spi_hal_set_baud_divisors(baseAddr, refPrescaler, refSpr);
}

// See fsl_spi_hal.h for documentation of this function.
void spi_hal_set_slave_select_output_mode(SPI_Type *baseAddr, spi_ss_output_mode_t mode)
{
    // The mode enum values encode the SSOE and MODFEN bit values.
    // Bit 0: SSOE
    // Bit 1: MODFEN
    SPI_BWR_C1_SSOE(baseAddr, ((uint32_t)mode & (1U << kSpiSsoeBit)) >> kSpiSsoeBit);
    SPI_BWR_C2_MODFEN(baseAddr, ((uint32_t)mode & (1U << kSpiModfenBit)) >> kSpiModfenBit);
}

// See fsl_spi_hal.h for documentation of this function.
void spi_hal_set_data_format(SPI_Type *baseAddr,
                             spi_clock_polarity_t polarity,
                             spi_clock_phase_t phase,
                             spi_shift_direction_t direction)
{
    SPI_BWR_C1_CPOL(baseAddr, (uint32_t)polarity);
    SPI_BWR_C1_CPHA(baseAddr, (uint32_t)phase);
    SPI_BWR_C1_LSBFE(baseAddr, (uint32_t)direction);
}

// See fsl_spi_hal.h for documentation of this function.
void spi_hal_set_pin_mode(SPI_Type *baseAddr, spi_pin_mode_t mode)
{
    // The mode enum values encode the SPC0 and BIDIROE bit values.
    // Bit 0: SPC0
    // Bit 1: BIDIROE
    SPI_BWR_C2_SPC0(baseAddr, ((uint32_t)mode & (1U << kSpiSpc0Bit)) >> kSpiSpc0Bit);
    SPI_BWR_C2_BIDIROE(baseAddr, ((uint32_t)mode & (1U << kSpiBidiroeBit)) >> kSpiBidiroeBit);
}

#if FSL_FEATURE_SPI_DMA
// See fsl_spi_hal.h for documentation of this function.
void spi_hal_configure_dma(SPI_Type *baseAddr, bool enableTransmit, bool enableReceive)
{
    SPI_BWR_C2_TXDMAE(baseAddr, (enableTransmit == true));
    SPI_BWR_C2_RXDMAE(baseAddr, (enableReceive == true));
}
#endif // FSL_FEATURE_SPI_DMA

// See fsl_spi_hal.h for documentation of this function.
void spi_hal_clear_mode_fault(SPI_Type *baseAddr)
{
    // Must make sure we read from the status register first.
    if (spi_hal_is_mode_fault(baseAddr))
    {
        // Then we have to write to C1.
        SPI_WR_C1(baseAddr, SPI_RD_C1(baseAddr));
    }
}

// See fsl_spi_hal.h for documentation of this function.
void spi_hal_clear_match(SPI_Type *baseAddr)
{
    // Check that the match flag is set before writing 1 to clear it. This read
    // is required in order to clear the flag.
    if (spi_hal_is_match(baseAddr))
    {
        // We have to hack this to write to the register because it is incorrectly
        // defined as a read-only register, even though the SPI_S.SPMF bitfield documentation
        // states you must write a 1 to the bitfield to clear it.
        *(volatile uint8_t *)SPI_S_REG(baseAddr) = SPI_S_SPMF_MASK;
    }
}

////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////

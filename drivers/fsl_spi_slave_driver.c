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

#include "fsl_device_registers.h"
#include "spi/fsl_spi_slave_driver.h"
#include "spi/hal/fsl_spi_hal.h"
#include "bootloader_common.h"
#include "utilities/fsl_assert.h"

#if BL_CONFIG_SPI

////////////////////////////////////////////////////////////////////////////////
// Definitions
////////////////////////////////////////////////////////////////////////////////

//! SPI slave constants
enum _spi_slave_constants
{
    kEmptyChar = 0, //!< Empty character
};

////////////////////////////////////////////////////////////////////////////////
// Prototypes
////////////////////////////////////////////////////////////////////////////////

void spi_slave_irq_handler(unsigned instance);

////////////////////////////////////////////////////////////////////////////////
// Variables
////////////////////////////////////////////////////////////////////////////////

const uint32_t g_spiBaseAddr[] = SPI_BASE_ADDRS;

//! Place to store application configuration and callbacks for each of the SPI modules.
static spi_slave_callbacks_t s_spiSlaveInstanceCallbacks[SPI_INSTANCE_COUNT];

////////////////////////////////////////////////////////////////////////////////
// Code
////////////////////////////////////////////////////////////////////////////////

#if !defined(BL_SPI_SIZE_OPTIMIZE) || USE_ONLY_SPI(0)
//! @brief Implementation of SPI0 handler named in startup code.
//!
//! Passes instance to generic SPI IRQ handler.
void SPI0_IRQHandler()
{
    spi_slave_irq_handler(SPI0_IDX);
}
#endif // !defined(BL_SPI_SIZE_OPTIMIZE) || USE_ONLY_SPI(0)

#if (!defined(BL_SPI_SIZE_OPTIMIZE) || USE_ONLY_SPI(1)) && (SPI_INSTANCE_COUNT > 1)
//! @brief Implementation of SPI1 handler named in startup code.
//!
//! Passes instance to generic SPI IRQ handler.
void SPI1_IRQHandler()
{
    spi_slave_irq_handler(SPI1_IDX);
}
#endif // (!defined(BL_SPI_SIZE_OPTIMIZE) || USE_ONLY_SPI(1)) && (SPI_INSTANCE_COUNT > 1)

//! @brief SPI Slave Generic IRQ handler.
//!
//! @param instance Instance number of the SPI module.
void spi_slave_irq_handler(unsigned instance)
{
#if USE_ONLY_SPI(0)
    instance = 0;
#elif USE_ONLY_SPI(1)
    instance = 1;
#endif // USE_ONLY_SPI(0)
    uint32_t baseAddr = g_spiBaseAddr[instance];

    spi_slave_callbacks_t *callbacks = &s_spiSlaveInstanceCallbacks[instance];

    if (spi_hal_is_read_buffer_full((SPI_Type *)baseAddr))
    {
        // SPI receive interrupt
        uint8_t rd = spi_hal_read_data((SPI_Type *)baseAddr);
        callbacks->dataSink(rd, instance);
    }

    if (spi_hal_is_transmit_buffer_empty((SPI_Type *)baseAddr))
    {
        // SPI transimit interrupt
        uint8_t source_byte = kEmptyChar;
        callbacks->dataSource(&source_byte, instance);

        // Write the data to data register
        spi_hal_write_data((SPI_Type *)baseAddr, source_byte);
    }
}

// See spi_slave.h for documentation of this function.
void spi_slave_set_data_sink_func(uint32_t instance, void (*data_sink)(uint8_t, uint16_t))
{
    s_spiSlaveInstanceCallbacks[instance].dataSink = data_sink;
}

// See spi_slave.h for documentation of this function.
void spi_slave_init(uint32_t instance, const spi_slave_config_t *config)
{
#if USE_ONLY_SPI(0)
    instance = 0;
#elif USE_ONLY_SPI(1)
    instance = 1;
#endif // USE_ONLY_SPI(0)

    assert(config);
    assert(instance < SPI_INSTANCE_COUNT);

    uint32_t baseAddr = g_spiBaseAddr[instance];

    // Save the application info.
    s_spiSlaveInstanceCallbacks[instance] = config->callbacks;

    // Enable clock for SPI
    SIM_SET_SCGC4(SIM,
#if USE_ONLY_SPI(0) || (SPI_INSTANCE_COUNT == 1)
                  SIM_SCGC4_SPI0_MASK
#elif USE_ONLY_SPI(1)
                  SIM_SCGC4_SPI1_MASK
#elif(SPI_INSTANCE_COUNT > 1)
                  // SPI0 and SPI1
                  instance == SPI0_IDX ? SIM_SCGC4_SPI0_MASK : SIM_SCGC4_SPI1_MASK
#endif // USE_ONLY_SPI(0) || (SPI_INSTANCE_COUNT == 1)
                  );

    // Reset control register 1, slave mode
    spi_hal_reset((SPI_Type *)baseAddr);

    // Set master or slave moe.
    spi_hal_set_master_slave((SPI_Type *)baseAddr, kSpiSlave);

    // Set data format.
    spi_hal_set_data_format((SPI_Type *)baseAddr, config->polarity, config->phase, config->direction);

    spi_hal_enable_receive_and_fault_interrupt((SPI_Type *)baseAddr);
    spi_hal_enable_transmit_interrupt((SPI_Type *)baseAddr);

// Enable SPI interrupt
#if USE_ONLY_SPI(0) || (SPI_INSTANCE_COUNT == 1)
    NVIC_EnableIRQ(SPI0_IRQn);
#elif USE_ONLY_SPI(1)
    NVIC_EnableIRQ(SPI1_IRQn);
#else
    // SPI0 and SPI1
    NVIC_EnableIRQ(instance == SPI0_IDX ? SPI0_IRQn : SPI1_IRQn);
#endif // USE_ONLY_SPI

    // SPI system enable
    spi_hal_enable((SPI_Type *)baseAddr);

    // Fill in the data buffer so a master will receive a known first byte (0).
    spi_hal_write_data((SPI_Type *)baseAddr, 0);
}

// See spi_slave.h for documentation of this function.
void spi_slave_shutdown(uint32_t instance)
{
    assert(instance < SPI_INSTANCE_COUNT);

// Make sure the instance being de-initialized is actually clocked so that
// registers writes won't cause an abort
#if USE_ONLY_SPI(0) || (SPI_INSTANCE_COUNT == 1)
    if (SIM_RD_SCGC4(SIM) & SIM_SCGC4_SPI0_MASK)
    {
        NVIC_DisableIRQ(SPI0_IRQn);
        spi_hal_reset(SPI0);
        SIM_CLR_SCGC4(SIM, SIM_SCGC4_SPI0_MASK);
    }
#elif USE_ONLY_SPI(1)
    if (SIM_RD_SCGC4(SIM) & SIM_SCGC4_SPI1_MASK)
    {
        NVIC_DisableIRQ(SPI1_IRQn);
        spi_hal_reset(SPI1);
        SIM_CLR_SCGC4(SIM, SIM_SCGC4_SPI1_MASK);
    }
#else
    uint32_t baseAddr = g_spiBaseAddr[instance];

    // SPI0 and SPI1
    if (((instance == 0) && (SIM_RD_SCGC4(SIM) & SIM_SCGC4_SPI0_MASK)) ||
        ((instance == 1) && (SIM_RD_SCGC4(SIM) & SIM_SCGC4_SPI1_MASK)))
    {
        // Disable SPI interrupt
        IRQn_Type irqNumber = instance == SPI0_IDX ? SPI0_IRQn : SPI1_IRQn;
        NVIC_DisableIRQ(irqNumber);

        // Reset SPI registers.
        spi_hal_reset((SPI_Type *)baseAddr);

        // Disable clock for SPI
        SIM_CLR_SCGC4(SIM, instance == SPI0_IDX ? SIM_SCGC4_SPI0_MASK : SIM_SCGC4_SPI1_MASK);
    }
#endif // USE_ONLY_SPI(0) || (SPI_INSTANCE_COUNT == 1)
}

#endif // BL_CONFIG_SPI

////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////

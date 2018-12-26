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

#include "bootloader_config.h"
#include "bootloader/bl_peripheral_interface.h"
#include "fsl_device_registers.h"
#include "lpspi/fsl_lpspi_slave_driver.h"
#include "lpspi/hal/fsl_lpspi_hal.h"
#include "bootloader_common.h"
#include <assert.h>

#if BL_CONFIG_LPSPI

////////////////////////////////////////////////////////////////////////////////
// Definitions
////////////////////////////////////////////////////////////////////////////////

//! LPSPI slave constants
enum _lpspi_slave_constants
{
    kEmptyChar = 0, //!< Empty character
    kFrameSize = 8  //!< The actul size of the transfer word in number of bits
};

////////////////////////////////////////////////////////////////////////////////
// Prototypes
////////////////////////////////////////////////////////////////////////////////

static void lpspi_slave_irq_handler(unsigned instance);
extern void LPSPI_SetSystemIRQ(uint32_t instance, PeripheralSystemIRQSetting set);

////////////////////////////////////////////////////////////////////////////////
// Variables
////////////////////////////////////////////////////////////////////////////////

const uint32_t g_lpspiBaseAddr[] = LPSPI_BASE_ADDRS;

//! Place to store application configuration and callbacks for each of the LPSPI modules.
static lpspi_slave_callbacks_t s_lpspiSlaveInstanceCallbacks[FSL_FEATURE_SOC_LPSPI_COUNT];

////////////////////////////////////////////////////////////////////////////////
// Code
////////////////////////////////////////////////////////////////////////////////

#if !defined(BL_LPSPI_SIZE_OPTIMIZE) || USE_ONLY_SPI(0)
//! @brief Implementation of LPSPI0 handler named in startup code.
//!
//! Passes instance to generic LPSPI IRQ handler.
/*
void LPSPI0_IRQHandler()
{
    lpspi_slave_irq_handler(LPSPI0_IDX);
}
*/
#endif // !defined(BL_LPSPI_SIZE_OPTIMIZE) || USE_ONLY_SPI(0)

#if (!defined(BL_LPSPI_SIZE_OPTIMIZE) || USE_ONLY_SPI(1)) && (LPSPI_INSTANCE_COUNT > 1)
//! @brief Implementation of LPSPI1 handler named in startup code.
//!
//! Passes instance to generic LPSPI IRQ handler.
/*
void LPSPI1_IRQHandler()
{
    lpspi_slave_irq_handler(LPSPI1_IDX);
}
*/
#endif // (!defined(BL_LPSPI_SIZE_OPTIMIZE) || USE_ONLY_SPI(1)) && (LPSPI_INSTANCE_COUNT > 1)

#if (!defined(BL_LPSPI_SIZE_OPTIMIZE) || USE_ONLY_SPI(2)) && (FSL_FEATURE_SOC_LPSPI_COUNT > 2)
//! @brief Implementation of LPSPI2 handler named in startup code.
//!
//! Passes instance to generic LPSPI IRQ handler.
void LPSPI2_IRQHandler()
{
    lpspi_slave_irq_handler(2);
}
#endif // (!defined(BL_LPSPI_SIZE_OPTIMIZE) || USE_ONLY_SPI(2)) && (LPSPI_INSTANCE_COUNT > 2)

//! @brief LPSPI Slave Generic IRQ handler.
//!
//! @param instance Instance number of the LPSPI module.
void lpspi_slave_irq_handler(unsigned instance)
{
#if USE_ONLY_SPI(0)
    instance = 0;
#elif USE_ONLY_SPI(1)
    instance = 1;
#elif USE_ONLY_SPI(2)
    instance = 2;
#endif // USE_ONLY_SPI(0)
    LPSPI_Type *baseAddr = (LPSPI_Type *)g_lpspiBaseAddr[instance];

    lpspi_slave_callbacks_t *callbacks = &s_lpspiSlaveInstanceCallbacks[instance];

    if (lpspi_hal_is_read_buffer_full(baseAddr))
    {
        // LPSPI receive interrupt
        uint8_t rd = lpspi_hal_read_byte(baseAddr);
        callbacks->dataSink(rd, instance);
    }

    if (lpspi_hal_is_transmit_buffer_empty(baseAddr))
    {
        // LPSPI transimit interrupt
        uint8_t source_byte = kEmptyChar;
        callbacks->dataSource(&source_byte, instance);

        // Write the data to data register
        lpspi_hal_write_byte(baseAddr, source_byte);
    }
}

// See lpspi_slave.h for documentation of this function.
void lpspi_slave_set_data_sink_func(uint32_t instance, void (*data_sink)(uint8_t, uint16_t))
{
    s_lpspiSlaveInstanceCallbacks[instance].dataSink = data_sink;
}

// See lpspi_slave.h for documentation of this function.
void lpspi_slave_init(uint32_t instance, const lpspi_slave_config_t *config)
{
#if USE_ONLY_SPI(0)
    instance = 0;
#elif USE_ONLY_SPI(1)
    instance = 1;
#elif USE_ONLY_SPI(2)
    instance = 2;
#endif // USE_ONLY_SPI(0)

    assert(config);
    assert(instance < LPSPI_INSTANCE_COUNT);

    LPSPI_Type *baseAddr = (LPSPI_Type *)g_lpspiBaseAddr[instance];

    // Save the application info.
    s_lpspiSlaveInstanceCallbacks[instance] = config->callbacks;

    // Reset control register 1, slave mode
    lpspi_hal_reset(baseAddr);

    // Set master or slave moe.
    lpspi_hal_set_master_slave(baseAddr, kLpspiSlave);

    lpspi_hal_set_data_format(baseAddr, config->polarity, config->phase, config->selection, config->direction,
                              kFrameSize - 1);

    lpspi_hal_enable_receive_interrupt(baseAddr);
    lpspi_hal_enable_transmit_interrupt(baseAddr);

    // Enable LPSPI interrupt
    LPSPI_SetSystemIRQ(instance, kPeripheralEnableIRQ);

    // LPSPI system enable
    lpspi_hal_enable(baseAddr);
}

// See lpspi_slave.h for documentation of this function.
void lpspi_slave_deinit(uint32_t instance)
{
    assert(instance < LPSPI_INSTANCE_COUNT);

    // Make sure the instance being de-initialized is actually clocked so that
    // registers writes won't cause an abort
    uint32_t baseAddr = g_lpspiBaseAddr[instance];

    // Disable LPSPI interrupt
    LPSPI_SetSystemIRQ(instance, kPeripheralDisableIRQ);

    // Reset control register 1.
    lpspi_hal_reset((LPSPI_Type *)baseAddr);
}

#endif // #if BL_CONFIG_LPSPI

////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////

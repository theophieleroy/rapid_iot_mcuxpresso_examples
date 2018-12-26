/*
 * The Clear BSD License
 * Copyright (c) 2013-2015, Freescale Semiconductor, Inc.
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

#include "bootloader/bl_peripheral_interface.h"
#include "bootloader/bl_irq_common.h"
#include "fsl_device_registers.h"
#include "i2c/slave/fsl_i2c_slave_driver.h"
#include "bootloader_common.h"
#include "utilities/fsl_assert.h"

#if BL_CONFIG_I2C
////////////////////////////////////////////////////////////////////////////////
// Definitions
////////////////////////////////////////////////////////////////////////////////

#define I2C_EMPTY_CHAR (0x00) //!< Empty character.

enum
{
    //! Max cycles supported to set glitch filter
    kI2CGlitchFilterMaxCycles = 31,
};

////////////////////////////////////////////////////////////////////////////////
// Variables
////////////////////////////////////////////////////////////////////////////////

//! Place to store application callbacks for each of the I2C modules.
static i2c_slave_info_t s_applicationInfo[2] = {{0}};

const static uint32_t g_i2cBaseAddr[] = I2C_BASE_ADDRS;

////////////////////////////////////////////////////////////////////////////////
// Private Prototypes
////////////////////////////////////////////////////////////////////////////////

void I2C0_IRQHandler(void);
void I2C1_IRQHandler(void);
static void i2c_slave_irqhandler(int instance);
////////////////////////////////////////////////////////////////////////////////
// Code
////////////////////////////////////////////////////////////////////////////////

#if !defined(BL_FEATURE_I2C_OPTIMIZE) || USE_ONLY_I2C(0)
//! @brief Implementation of I2C0 handler named in startup code.
//!
//! Passes instance to generic I2C IRQ handler.
void I2C0_IRQHandler(void)
{
    i2c_slave_irqhandler(I2C0_IDX);
}
#endif // !defined(BL_FEATURE_I2C_OPTIMIZE) || USE_ONLY_I2C(0)

#if (!defined(BL_FEATURE_I2C_OPTIMIZE) || USE_ONLY_I2C(1)) && (I2C_INSTANCE_COUNT > 1)
//! @brief Implementation of I2C1 handler named in startup code.
//!
//! Passes instance to generic I2C IRQ handler.
void I2C1_IRQHandler(void)
{
    i2c_slave_irqhandler(I2C1_IDX);
}
#endif // (!defined(BL_FEATURE_I2C_OPTIMIZE) || USE_ONLY_I2C(1)) && (I2C_INSTANCE_COUNT > 1)

//! @brief I2C Slave Generic IRQ handler.
//!
//! Implements the flow chart at the end of the I2C chapter in the Kinetis
//! KL25 Sub-Family Reference Manual. It uses callbacks to get/put data
//! from/to the application as well as alert the application of an error condition.
//!
//! @param instance Instance number of the I2C module.
static void i2c_slave_irqhandler(int instance)
{
#if USE_ONLY_I2C(0)
    instance = 0;
#elif USE_ONLY_I2C(1)
    instance = 1;
#endif // USE_ONLY_I2C(0)

    assert(instance < I2C_INSTANCE_COUNT);

    uint32_t baseAddr = g_i2cBaseAddr[instance];

    i2c_slave_info_t *appInfo = &s_applicationInfo[instance];
    uint8_t status;

#if defined(I2C_SMB)

    status = I2C_RD_SMB((I2C_Type *)baseAddr);

    if (status & I2C_SMB_SLTF_MASK)
    {
        // Clear SCL Low Timeout flag
        I2C_SET_SMB((I2C_Type *)baseAddr, I2C_SMB_SLTF_MASK);
        // Clear interrupt flag
        I2C_SET_S((I2C_Type *)baseAddr, I2C_S_IICIF_MASK);

        appInfo->on_error(instance);

        return;
    }
#endif

    status = I2C_RD_S((I2C_Type *)baseAddr);
    bool doTransmit = false;

    // Clear interrupt flag(s)
    I2C_SET_S((I2C_Type *)baseAddr, (I2C_S_ARBL_MASK | I2C_S_IICIF_MASK) & status);

    if ((status & I2C_S_ARBL_MASK) && (!(status & I2C_S_IAAS_MASK))) // ArbitrationLost and not AddressedAsSlave
    {
        // I2C_S_ARBL is already cleared by ClearInterruptFlags()
        //         error = kStatus_I2C_AribtrationLost;
    }
    else if (status & I2C_S_IAAS_MASK) // AddressedAsSlave
    {
        if (status & I2C_S_SRW_MASK) // Master read from Slave. Slave transmit.
        {
            // Switch to TX mode
            I2C_SET_C1((I2C_Type *)baseAddr, I2C_C1_TX_MASK);

            doTransmit = true;
        }
        else // Master write to Slave. Slave receive.
        {
            // Switch to RX mode.
            I2C_CLR_C1((I2C_Type *)baseAddr, I2C_C1_TX_MASK);

            // Dummy read character.
            char dummy = I2C_RD_D((I2C_Type *)baseAddr);
        }
    }
    else // not AddressedAsSlave
    {
        if (I2C_RD_C1_TX((I2C_Type *)baseAddr)) // Transmit.
        {
            if (status & I2C_S_RXAK_MASK) // No ACK from receiver.
            {
                // Switch to RX mode.
                I2C_CLR_C1((I2C_Type *)baseAddr, I2C_C1_TX_MASK);

                // Dummy read character.
                char dummy = I2C_RD_D((I2C_Type *)baseAddr);
            }
            else // ACK from receiver.
            {
                // DO TRANSMIT
                doTransmit = true;
            }
        }
        else // Receive.
        {
            // Get byte from data register.
            uint8_t sink_byte = I2C_RD_D((I2C_Type *)baseAddr);

            appInfo->data_sink(sink_byte, instance);
        }
    }

    if (doTransmit)
    {
        uint8_t source_byte = I2C_EMPTY_CHAR;

        appInfo->data_source(&source_byte);

        // Store char to transmit register.
        I2C_WR_D((I2C_Type *)baseAddr, source_byte);
    }
}

// See fsl_i2c_slave_driver.h for documentation of this function.
void i2c_slave_reset(int instance)
{
    uint32_t baseAddr = g_i2cBaseAddr[instance];

    // Restore those control and configuration registers which are used to
    // be operated
    I2C_WR_C1((I2C_Type *)baseAddr, 0);
    I2C_WR_A1((I2C_Type *)baseAddr, 0);
    I2C_WR_FLT((I2C_Type *)baseAddr, 0);

#if defined(I2C_SMB)
    // Clear SCL Low Timeout Flag
    I2C_WR_SMB((I2C_Type *)baseAddr, I2C_SMB_SLTF_MASK);
    // Disable SCL Low Timout check, configure with 0.
    I2C_WR_SLTH((I2C_Type *)baseAddr, 0);
    I2C_WR_SLTL((I2C_Type *)baseAddr, 0);
#endif // I2C_SMB
}

// See i2c_slave.h for documentation of this function.
void i2c_slave_init(int instance, i2c_slave_info_t *appInfo)
{
#if USE_ONLY_I2C(0)
    instance = 0;
#elif USE_ONLY_I2C(1)
    instance = 1;
#endif // USE_ONLY_I2C(0)

    assert(appInfo);
    assert(instance < I2C_INSTANCE_COUNT);

    uint32_t baseAddr = g_i2cBaseAddr[instance];

    // Save the application info.
    s_applicationInfo[instance] = *appInfo;

// Enable clock for I2C.
#if USE_ONLY_I2C(0) || (I2C_INSTANCE_COUNT == 1)
    SIM_SET_SCGC4(SIM, SIM_SCGC4_I2C0_MASK);
#elif USE_ONLY_I2C(1)
    SIM_SET_SCGC4(SIM, SIM_SCGC4_I2C1_MASK);
#else
    SIM_SET_SCGC4(SIM, instance == I2C0_IDX ? SIM_SCGC4_I2C0_MASK : SIM_SCGC4_I2C1_MASK);
#endif // USE_ONLY_I2C(0) || (I2C_INSTANCE_COUNT == 1)

    // Clear control register.
    I2C_WR_C1((I2C_Type *)baseAddr, 0);

    // Clear bus status interrupt flags.
    I2C_WR_FLT((I2C_Type *)baseAddr, I2C_FLT_STOPF_MASK);

    // Clear interrupt flag.
    I2C_WR_S((I2C_Type *)baseAddr, I2C_S_IICIF_MASK);

    // Enable I2C IRQ.
    i2c_set_system_IRQ_gate(instance, kPeripheralEnableIRQ);

    // Set Slave Address.
    I2C_WR_A1_AD((I2C_Type *)baseAddr, appInfo->slaveAddress);
    // No extended functions.
    I2C_WR_C2((I2C_Type *)baseAddr, 0);
    // No glitch filter.
    I2C_WR_FLT((I2C_Type *)baseAddr, 0);
#if defined(I2C_SMB)
    // Select the source of Timeout Counter, Clear SCL Low Timeout Flag
    I2C_WR_SMB((I2C_Type *)baseAddr, I2C_SMB_TCKSEL_MASK | I2C_SMB_SLTF_MASK);
    // Enable SCL Low Timout check, configure with max value.
    I2C_WR_SLTH((I2C_Type *)baseAddr, I2C_SLTH_SSLT_MASK);
    I2C_WR_SLTL((I2C_Type *)baseAddr, I2C_SLTL_SSLT_MASK);
#endif
    // Set baud rate.
    I2C_WR_F((I2C_Type *)baseAddr, 0);
    // Enable I2C device.
    I2C_SET_C1((I2C_Type *)baseAddr, I2C_C1_IICEN_MASK);
    // Enable interrupt.
    I2C_SET_C1((I2C_Type *)baseAddr, I2C_C1_IICIE_MASK);
}

// See i2c_slave.h for documentation of this function.
void i2c_slave_deinit(int instance)
{
    assert(instance < I2C_INSTANCE_COUNT);

// Make sure the instance being de-initialized is actually clocked so that
// registers writes won't cause an abort
#if USE_ONLY_I2C(0) || (I2C_INSTANCE_COUNT == 1)
    if (SIM_RD_SCGC4(SIM) & SIM_SCGC4_I2C0_MASK)
    {
        NVIC_DisableIRQ(I2C0_IRQn);
#if defined(I2C)
        I2C_WR_C1(I2C, 0);
        I2C_BWR_A1_AD(I2C, 0);
        I2C_WR_FLT(I2C, 0);
#else
        // Set the i2c slave address as 0x0
        I2C_WR_C1(I2C0, 0);
        I2C_BWR_A1_AD(I2C0, 0);
        I2C_WR_FLT(I2C0, 0);
#endif // defined (I2C)

#if defined(I2C_SMB)
#if defined(I2C)
        // Clear SCL Low Timeout Flag
        I2C_WR_SMB(I2C, I2C_SMB_SLTF_MASK);
        // Disable SCL Low Timout check, configure with 0.
        I2C_WR_SLTH(I2C, 0);
        I2C_WR_SLTL(I2C, 0);
#else
        // Clear SCL Low Timeout Flag
        I2C_WR_SMB(I2C0, I2C_SMB_SLTF_MASK);
        // Disable SCL Low Timout check, configure with 0.
        I2C_WR_SLTH(I2C0, 0);
        I2C_WR_SLTL(I2C0, 0);
#endif // defined (I2C)
#endif // I2C_SMB
        SIM_CLR_SCGC4(SIM, SIM_SCGC4_I2C0_MASK);
    }
#elif USE_ONLY_I2C(1)
    if (SIM_RD_SCGC4(SIM) & SIM_SCGC4_I2C1_MASK)
    {
        NVIC_DisableIRQ(I2C1_IRQn);
        I2C_WR_C1(I2C1, 0);
        // Set the i2c slave address as 0x0
        I2C_WR_A1_AD(I2C1, 0);
        I2C_WR_FLT(I2C1, 0);
#if defined(I2C_SMB)
        // Clear SCL Low Timeout Flag
        I2C_WR_SMB(I2C1, I2C_SMB_SLTF_MASK);
        // Disable SCL Low Timout check, configure with 0.
        I2C_WR_SLTH(I2C1, 0);
        I2C_WR_SLTL(I2C1, 0);
#endif // I2C_SMB
        SIM_CLR_SCGC4(SIM, SIM_SCGC4_I2C1_MASK);
    }
#else
    //    uint32_t baseAddr = g_i2cBaseAddr[instance];

    if (((instance == 0) && (SIM_RD_SCGC4(SIM) & SIM_SCGC4_I2C0_MASK)) ||
        ((instance == 1) && (SIM_RD_SCGC4(SIM) & SIM_SCGC4_I2C1_MASK)))
    {
        // Disable I2C interrupt
        // Enable I2C IRQ.
        i2c_set_system_IRQ_gate(instance, kPeripheralDisableIRQ);

        // Reset I2C registers
        i2c_slave_reset(instance);

        // Disable clock for I2C.
        SIM_CLR_SCGC4(SIM, instance == I2C0_IDX ? SIM_SCGC4_I2C0_MASK : SIM_SCGC4_I2C1_MASK);
    }
#endif // USE_ONLY_I2C(0) || (I2C_INSTANCE_COUNT == 1)
}

// See i2c_slave.h for documentation of this function.
void i2c_set_glitch_filter_width(int instance, uint32_t busClock_Hz, uint32_t glitchWidth_ns)
{
#if USE_ONLY_I2C(0)
    instance = 0;
#elif USE_ONLY_I2C(1)
    instance = 1;
#endif // USE_ONLY_I2C(0)

    uint32_t baseAddr = g_i2cBaseAddr[instance];
    uint32_t busCycle_ns = 1000000 / (busClock_Hz / 1000);

    // Search for the cycle count just below the desired glitch width.
    uint32_t cycles = 0;
    while (((cycles + 1) * busCycle_ns) < glitchWidth_ns)
    {
        ++cycles;
    }

    // If we end up with zero cycles, then set the filter to a single cycle unless the
    // bus clock is greater than 10x the desired glitch width.
    if ((cycles == 0) && (busCycle_ns <= (glitchWidth_ns * 10)))
    {
        cycles = 1;
    }
    // If the cycles is greater the max cycles supported to set glitch filter,
    // then cycles should be equal to max cycles
    else if (cycles > kI2CGlitchFilterMaxCycles)
    {
        cycles = kI2CGlitchFilterMaxCycles;
    }

    I2C_BWR_FLT_FLT((I2C_Type *)baseAddr, cycles);
}

// See i2c_slave.h for documentation of this function.
void i2c_slave_set_data_sink_func(uint32_t instance, void (*data_sink)(uint8_t, uint32_t))
{
    s_applicationInfo[instance].data_sink = data_sink;
}
#endif // BL_CONFIG_I2C
////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////

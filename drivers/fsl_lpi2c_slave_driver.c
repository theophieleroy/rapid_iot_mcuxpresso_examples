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
#include "lpi2c/slave/fsl_lpi2c_slave_driver.h"
#include "lpi2c/hal/fsl_lpi2c_hal.h"
#include "bootloader_common.h"
#include "microseconds/microseconds.h"
#include <assert.h>

#if BL_CONFIG_LPI2C

////////////////////////////////////////////////////////////////////////////////
// Definitions
////////////////////////////////////////////////////////////////////////////////

#define I2C_EMPTY_CHAR (0x00) //!< Empty character.

enum
{
    //! Width of glitches to filter in nanoseconds.
    kLPI2CGlitchFilterWidth_ns = 50,
    // Max cycles supported to set glitch filter
    kLPI2CGlitchFilterMaxCycles = 15,
    //! Part of SDA data valid delay time.
    kLPI2CDataValidDelayCycles = 1,
    //! Part of SCL clock hold time.
    kLPI2CClockHoldCycles = 2,
    //! Timeout before shut down module
    kLPI2CShutdownTimeout_ms = 100
};

////////////////////////////////////////////////////////////////////////////////
// Variables
////////////////////////////////////////////////////////////////////////////////

//! Place to store application callbacks for each of the LPI2C modules.
static lpi2c_slave_info_t s_applicationInfo[3] = {{0}};

const static uint32_t g_lpi2cBaseAddr[] = LPI2C_BASE_ADDRS;

////////////////////////////////////////////////////////////////////////////////
// Private Prototypes
////////////////////////////////////////////////////////////////////////////////

void LPI2C0_IRQHandler(void);
void LPI2C1_IRQHandler(void);
void LPI2C2_IRQHandler(void);
static void lpi2c_slave_irqhandler(int instance);
extern void lpi2c_set_system_IRQ_gate(uint32_t instance, PeripheralSystemIRQSetting set);

////////////////////////////////////////////////////////////////////////////////
// Code
////////////////////////////////////////////////////////////////////////////////

#if !defined(BL_I2C_SIZE_OPTIMIZE) || USE_ONLY_I2C(0)
//! @brief Implementation of LPI2C0 handler named in startup code.
//!
//! Passes instance to generic LPI2C IRQ handler.
void LPI2C0_IRQHandler(void)
{
    lpi2c_slave_irqhandler(LPI2C0_IDX);
}
#endif // !defined(BL_I2C_SIZE_OPTIMIZE) || USE_ONLY_I2C(0)

#if (!defined(BL_I2C_SIZE_OPTIMIZE) || USE_ONLY_I2C(1)) && (LPI2C_INSTANCE_COUNT > 1)
//! @brief Implementation of LPI2C1 handler named in startup code.
//!
//! Passes instance to generic LPI2C IRQ handler.
void LPI2C1_IRQHandler(void)
{
    lpi2c_slave_irqhandler(LPI2C1_IDX);
}
#endif // (!defined(BL_I2C_SIZE_OPTIMIZE) || USE_ONLY_I2C(1)) && (LPI2C_INSTANCE_COUNT > 1)

#if (!defined(BL_I2C_SIZE_OPTIMIZE) || USE_ONLY_I2C(2)) && (LPI2C_INSTANCE_COUNT > 2)
//! @brief Implementation of LPI2C2 handler named in startup code.
//!
//! Passes instance to generic LPI2C IRQ handler.
void LPI2C2_IRQHandler(void)
{
    lpi2c_slave_irqhandler(LPI2C1_IDX);
}
#endif // (!defined(BL_I2C_SIZE_OPTIMIZE) || USE_ONLY_I2C(2)) && (LPI2C_INSTANCE_COUNT > 2)

//! @brief LPI2C Slave Generic IRQ handler.
//!
//! Implements the flow chart at the end of the LPI2C chapter in the Kinetis
//! KL25 Sub-Family Reference Manual. It uses callbacks to get/put data
//! from/to the application as well as alert the application of an error condition.
//!
//! @param instance Instance number of the LPI2C module.
static void lpi2c_slave_irqhandler(int instance)
{
#if USE_ONLY_I2C(0)
    instance = 0;
#elif USE_ONLY_I2C(1)
    instance = 1;
#elif USE_ONLY_I2C(2)
    instance = 2;
#endif // USE_ONLY_I2C(0)

    assert(instance < LPI2C_INSTANCE_COUNT);

    LPI2C_Type *baseAddr = (LPI2C_Type *)g_lpi2cBaseAddr[instance];

    lpi2c_slave_info_t *appInfo = &s_applicationInfo[instance];

    if (lpi2c_hal_get_slave_status_flag(baseAddr, kLPI2CSlaveAddressValid))
    {
        lpi2c_hal_get_slave_address_status(baseAddr);
        appInfo->isReadyForShutdown = false;
    }

    if (lpi2c_hal_get_slave_status_flag(baseAddr, kLPI2CSlaveReceiveData))
    {
        // Get byte from slave receive register.
        uint8_t sink_byte = lpi2c_hal_read_slave_byte(baseAddr);

        appInfo->data_sink(sink_byte, instance);
    }

    if (lpi2c_hal_get_slave_status_flag(baseAddr, kLPI2CSlaveTransmitData))
    {
        uint8_t source_byte = I2C_EMPTY_CHAR;

        appInfo->data_source(&source_byte);

        // Store char to slave transmit register.
        lpi2c_hal_write_slave_byte(baseAddr, source_byte);
    }

    if (lpi2c_hal_get_slave_status_flag(baseAddr, kLPI2CSlaveStopDetect))
    {
        lpi2c_hal_clear_slave_status_flag(baseAddr, kLPI2CSlaveStopDetect);
        appInfo->isReadyForShutdown = true;
    }
}

// See lpi2c_slave.h for documentation of this function.
void lpi2c_slave_init(int instance, lpi2c_slave_info_t *appInfo)
{
#if USE_ONLY_I2C(0)
    instance = 0;
#elif USE_ONLY_I2C(1)
    instance = 1;
#elif USE_ONLY_I2C(2)
    instance = 2;
#endif // USE_ONLY_I2C(0)

    assert(appInfo);
    assert(instance < LPI2C_INSTANCE_COUNT);

    LPI2C_Type *baseAddr = (LPI2C_Type *)g_lpi2cBaseAddr[instance];

    // Save the application info.
    s_applicationInfo[instance] = *appInfo;

    // Clear slave control registers.
    lpi2c_hal_reset_slave(baseAddr);

    // Set Slave Address.
    lpi2c_hal_set_slave_7bit_address0(baseAddr, appInfo->slaveAddress);

    // Set Slave glitch filter for both SDA and SCL input
    lpi2c_set_glitch_filter_width(instance, get_bus_clock(), kLPI2CGlitchFilterWidth_ns);

    // Set the SDA data valid delay time for the LPI2C slave equal to FILTSCL+DATAVD+3 cycles
    // Note: SDA data valid delay function is active only when feeding a non-zero value here
    lpi2c_hal_set_slave_data_valid_delay(baseAddr, kLPI2CDataValidDelayCycles);

    // Transmit Data Flag will assert whenever the transmit data register is empty.
    lpi2c_hal_config_slave_transmit_data_flag(baseAddr, kLPI2CSlaveTransmitFlagAssertWheneverEmpty);

    // Enable SCL clock stretching
    lpi2c_hal_enbale_slave_tx_data_scl_stall(baseAddr);
    lpi2c_hal_enbale_slave_rx_scl_stall(baseAddr);
    lpi2c_hal_set_slave_clock_hold_time(baseAddr, kLPI2CClockHoldCycles);

    // Enable slave interrupts.
    lpi2c_hal_enable_slave_address_valid_interrupt(baseAddr);
    lpi2c_hal_enable_slave_transmit_data_interrupt(baseAddr);
    lpi2c_hal_enable_slave_receive_data_interrupt(baseAddr);
    lpi2c_hal_enable_slave_stop_detect_interrupt(baseAddr);

    // Enable LPI2C interrupt
    lpi2c_set_system_IRQ_gate(instance, kPeripheralEnableIRQ);

    // Enable slave LPI2C device.
    lpi2c_hal_enable_slave(baseAddr);
}

// See lpi2c_slave.h for documentation of this function.
void lpi2c_slave_deinit(int instance)
{
    assert(instance < LPI2C_INSTANCE_COUNT);

    lpi2c_slave_info_t *appInfo = &s_applicationInfo[instance];

    // LPI2C can only be shut down after current transfer has been completed(we has detected the STOP bit)
    // ,and we need to add a timeout check here in case there is an incomplete transmission
    uint64_t lastTicks = microseconds_get_ticks();
    const uint64_t timeoutTicks = microseconds_convert_to_ticks(1000) * kLPI2CShutdownTimeout_ms;
    while (!appInfo->isReadyForShutdown)
    {
        uint64_t elapsedTicks = microseconds_get_ticks() - lastTicks;
        if (elapsedTicks >= timeoutTicks)
        {
            break;
        }
    }

    // Make sure the instance being de-initialized is actually clocked so that
    // registers writes won't cause an abort
    uint32_t baseAddr = g_lpi2cBaseAddr[instance];

    // Disable LPI2C interrupt
    lpi2c_set_system_IRQ_gate(instance, kPeripheralDisableIRQ);

    // Clear slave control registers.
    lpi2c_hal_reset_slave((LPI2C_Type *)baseAddr);
}

// See lpi2c_slave.h for documentation of this function.
void lpi2c_set_glitch_filter_width(int instance, uint32_t busClock_Hz, uint32_t glitchWidth_ns)
{
#if USE_ONLY_I2C(0)
    instance = 0;
#elif USE_ONLY_I2C(1)
    instance = 1;
#elif USE_ONLY_I2C(2)
    instance = 2;
#endif // USE_ONLY_I2C(0)

    uint32_t baseAddr = g_lpi2cBaseAddr[instance];
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
    else if (cycles > kLPI2CGlitchFilterMaxCycles)
    {
        cycles = kLPI2CGlitchFilterMaxCycles;
    }

    lpi2c_hal_set_slave_sda_glitch_filter((LPI2C_Type *)baseAddr, cycles);
    lpi2c_hal_set_slave_scl_glitch_filter((LPI2C_Type *)baseAddr, cycles);

    lpi2c_hal_enable_slave_digital_filtering((LPI2C_Type *)baseAddr);
}

// See lpi2c_slave.h for documentation of this function.
void lpi2c_slave_set_data_sink_func(uint32_t instance, void (*data_sink)(uint8_t, uint32_t))
{
    s_applicationInfo[instance].data_sink = data_sink;
}

#endif // BL_CONFIG_LPI2C
////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////

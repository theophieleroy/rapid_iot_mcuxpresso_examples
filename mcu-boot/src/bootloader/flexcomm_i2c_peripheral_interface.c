/*
 * The Clear BSD License
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
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

#include <assert.h>
#include "bootloader/bl_context.h"
#include "bootloader_common.h"
#include "bootloader/bl_peripheral_interface.h"
#include "packet/command_packet.h"
#include "flexcomm/fsl_flexcomm.h"
#include "flexcomm/fsl_i2c.h"
#include "fsl_device_registers.h"
#include "peripherals_pinmux.h"
#include "packet/serial_packet.h"

//! @addtogroup flexcomm_i2c_peripheral
//! @{

#if BL_CONFIG_FLEXCOMM_I2C

////////////////////////////////////////////////////////////////////////////////
// Definitions
////////////////////////////////////////////////////////////////////////////////

#define I2C_EMPTY_CHAR (0x00) //!< Empty character.

// Allow the default to be overridden via the bootloader_config.h file.
#if !defined(BL_DEFAULT_I2C_SLAVE_ADDRESS)
//! @brief Default FLEXCOMM I2C slave address in 7-bit format.
#define BL_DEFAULT_I2C_SLAVE_ADDRESS (0x10)
#endif // BL_DEFAULT_I2C_SLAVE_ADDRESS

//! @brief Synchronization state between FLEXCOMM I2C ISR and read/write functions.
typedef struct _flexcomm_i2c_transfer_info
{
    const uint8_t *writeData;                                //!< The applications data to write
    volatile uint32_t bytesToTransfer;                       //!< The total number of bytes to be transmitted
    void (*data_source)(uint8_t *source_byte);               // !< Callback used to get byte to transmit.
    void (*data_sink)(uint8_t sink_byte, uint32_t instance); // !< Callback used to put received byte.
} flexcomm_i2c_transfer_info_t;

////////////////////////////////////////////////////////////////////////////////
// Private Prototypes
////////////////////////////////////////////////////////////////////////////////

static bool flexcomm_i2c_poll_for_activity(const peripheral_descriptor_t *self);
static status_t flexcomm_i2c_full_init(const peripheral_descriptor_t *self, serial_byte_receive_func_t function);
static void flexcomm_i2c_full_shutdown(const peripheral_descriptor_t *self);
static status_t flexcomm_i2c_write(const peripheral_descriptor_t *self, const uint8_t *buffer, uint32_t byteCount);

static void flexcomm_i2c_data_source(uint8_t *source_byte);
static void flexcomm_i2c_initial_data_sink(uint8_t sink_byte, uint32_t instance);
static void flexcomm_i2c_data_sink(uint8_t sink_byte, uint32_t instance);

static void flexcomm_i2c_SlaveIRQHandler(uint32_t instance);

extern void FLEXCOMM_I2C_SetSystemIRQ(uint32_t instance, PeripheralSystemIRQSetting set);

////////////////////////////////////////////////////////////////////////////////
// Variables
////////////////////////////////////////////////////////////////////////////////

const peripheral_control_interface_t g_flexcommI2cControlInterface = {
    .pollForActivity = flexcomm_i2c_poll_for_activity, .init = flexcomm_i2c_full_init, .shutdown = flexcomm_i2c_full_shutdown, .pump = 0
};

const peripheral_byte_inteface_t g_flexcommI2cByteInterface = {.init = NULL, .write = flexcomm_i2c_write };

//! @brief Global state for the FLEXCOMM I2C peripheral interface.
static flexcomm_i2c_transfer_info_t s_flexcommI2cInfo = {
    .writeData = 0, .bytesToTransfer = 0, .data_source = flexcomm_i2c_data_source, .data_sink = flexcomm_i2c_initial_data_sink
};

static bool s_flexcommI2cActivity[FSL_FEATURE_SOC_I2C_COUNT] = { false };
static bool s_flexcommI2cIntialized[FSL_FEATURE_SOC_I2C_COUNT] = { false };

const static uint32_t g_flexcommI2cBaseAddr[] = I2C_BASE_ADDRS;

static serial_byte_receive_func_t s_flexcomm_i2c_app_data_sink_callback;

////////////////////////////////////////////////////////////////////////////////
// Code
////////////////////////////////////////////////////////////////////////////////

#if defined(BL_CONFIG_FLEXCOMM_I2C_2)
/*
 * I2C2 IRQ Handler
 *
 */
void I2C2_IRQHandler(void)
{
    flexcomm_i2c_SlaveIRQHandler(2);
}
#endif // !defined(BL_CONFIG_FLEXCOMM_I2C_2)

//! @brief FLEXCOMM I2C Slave Generic IRQ handler.
//!
//! Implements the flow chart at the end of the FLEXCOMM I2C chapter in the Kinetis
//! KL25 Sub-Family Reference Manual. It uses callbacks to get/put data
//! from/to the application as well as alert the application of an error condition.
//!
//! @param instance Instance number of the FLEXCOMM I2C module.
static void flexcomm_i2c_SlaveIRQHandler(uint32_t instance)
{
    assert(instance < FSL_FEATURE_SOC_I2C_COUNT);

    I2C_Type *baseAddr = (I2C_Type *)g_flexcommI2cBaseAddr[instance];

    uint32_t status = I2C_GetStatusFlags(baseAddr);

    /* SLVPENDING flag is cleared by writing I2C_SLVCTL_SLVCONTINUE_MASK to SLVCTL register */
    if (status & I2C_STAT_SLVPENDING_MASK)
    {
        if (((status & I2C_STAT_SLVSTATE_MASK) >> I2C_STAT_SLVSTATE_SHIFT) == I2C_STAT_SLVST_ADDR)
        {
            // continue transaction
            baseAddr->SLVCTL = I2C_SLVCTL_SLVCONTINUE_MASK;
        }
        else if (((status & I2C_STAT_SLVSTATE_MASK) >> I2C_STAT_SLVSTATE_SHIFT) == I2C_STAT_SLVST_RX)
        {
            // Get byte from slave receive register.
            uint8_t sink_byte = (uint8_t)baseAddr->SLVDAT;
            s_flexcommI2cInfo.data_sink(sink_byte, instance);

            // continue transaction
            baseAddr->SLVCTL = I2C_SLVCTL_SLVCONTINUE_MASK;

        }
        else if (((status & I2C_STAT_SLVSTATE_MASK) >> I2C_STAT_SLVSTATE_SHIFT) == I2C_STAT_SLVST_TX)
        {
            uint8_t source_byte = I2C_EMPTY_CHAR;
            s_flexcommI2cInfo.data_source(&source_byte);
            // Store char to slave transmit register.
            baseAddr->SLVDAT = source_byte;

            // continue transaction
            baseAddr->SLVCTL = I2C_SLVCTL_SLVCONTINUE_MASK;
        }
        else
        {
            baseAddr->SLVCTL = I2C_SLVCTL_SLVNACK_MASK;
        }
    }
}

bool flexcomm_i2c_poll_for_activity(const peripheral_descriptor_t *self)
{
    return s_flexcommI2cActivity[self->instance];
}

void flexcomm_i2c_data_source(uint8_t *source_byte)
{
    assert(source_byte);

    if (s_flexcommI2cInfo.bytesToTransfer)
    {
        *source_byte = *s_flexcommI2cInfo.writeData++;
        s_flexcommI2cInfo.bytesToTransfer--;
    }
    else
    {
        *source_byte = 0;
    }
}

void flexcomm_i2c_initial_data_sink(uint8_t sink_byte, uint32_t instance)
{
    if (sink_byte == kFramingPacketStartByte)
    {
        s_flexcommI2cActivity[instance] = true;
        s_flexcommI2cInfo.data_sink = flexcomm_i2c_data_sink;
        s_flexcomm_i2c_app_data_sink_callback(sink_byte);
    }
}

void flexcomm_i2c_data_sink(uint8_t sink_byte, uint32_t instance)
{
    s_flexcomm_i2c_app_data_sink_callback(sink_byte);
}

status_t flexcomm_i2c_full_init(const peripheral_descriptor_t *self, serial_byte_receive_func_t function)
{
    s_flexcomm_i2c_app_data_sink_callback = function;

    uint32_t baseAddr = g_flexcommI2cBaseAddr[self->instance];

    // Read the address from the configuration field. If it is not set, i.e. 0xff,
    // use the default address.
    uint8_t slaveAddress = g_bootloaderContext.propertyInterface->store->configurationData.i2cSlaveAddress;
    if (slaveAddress == 0xff)
    {
        slaveAddress = BL_DEFAULT_I2C_SLAVE_ADDRESS;
    }

    // Init the driver.
    i2c_slave_config_t config;

    I2C_SlaveGetDefaultConfig(&config);
    config.address0.address = slaveAddress;

    // Configure selected pin as flexcomm i2c peripheral interface
#if defined(BL_FEATURE_6PINS_PERIPHERAL) && BL_FEATURE_6PINS_PERIPHERAL
    self->pinmuxConfig(self->instance, kPinmuxType_PollForActivity);
#else
    self->pinmuxConfig(self->instance, kPinmuxType_Peripheral);
#endif // BL_FEATURE_6PINS_PERIPHERAL

    // Ungate the FLEXCOMM I2C clock.
    I2C_SlaveInit((I2C_Type *)baseAddr, &config, get_flexcomm_clock(self->instance));
    I2C_SlaveEnable((I2C_Type *)baseAddr, true);
    I2C_EnableInterrupts((I2C_Type *)baseAddr, I2C_INTENCLR_SLVPENDINGCLR_MASK);

    // Enable FLEXCOMM I2C interrupt
    FLEXCOMM_I2C_SetSystemIRQ(self->instance, kPeripheralEnableIRQ);

    s_flexcommI2cInfo.data_sink = flexcomm_i2c_initial_data_sink;

    s_flexcommI2cIntialized[self->instance] = true;

    return kStatus_Success;
}

void flexcomm_i2c_full_shutdown(const peripheral_descriptor_t *self)
{
    if (s_flexcommI2cIntialized[self->instance])
    {
        uint32_t baseAddr = g_flexcommI2cBaseAddr[self->instance];

        // Ungate the FLEXCOMM I2C clock.
        I2C_SlaveDeinit((I2C_Type *)baseAddr);
#if BL_FEATURE_6PINS_PERIPHERAL
        // When the active peripheral is not I2C, we should only restore
        //   those pin which we used to poll for activity.
        if (g_bootloaderContext.activePeripheral == NULL)
        {
            self->pinmuxConfig(self->instance, kPinmuxType_RestoreForActivity);
        }
        // When the active peripheral is I2C, we should restore all
        //  the I2C peripheral pin.
        else
#endif
        {
            // Restore selected pin to default state to reduce IDD.
            self->pinmuxConfig(self->instance, kPinmuxType_Default);
        }

        s_flexcommI2cIntialized[self->instance] = false;
    }
}

status_t flexcomm_i2c_write(const peripheral_descriptor_t *self, const uint8_t *buffer, uint32_t byteCount)
{
    s_flexcommI2cInfo.writeData = buffer;
    s_flexcommI2cInfo.bytesToTransfer = byteCount;

    while (s_flexcommI2cInfo.bytesToTransfer)
        ;

    return kStatus_Success;
}

//! @}

#endif // BL_CONFIG_FLEXCOMM_I2C

////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////

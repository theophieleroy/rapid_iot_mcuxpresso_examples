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

#include "utilities/fsl_assert.h"
#include "bootloader/bl_context.h"
#include "bootloader_common.h"
#include "bootloader/bl_peripheral_interface.h"
#include "packet/command_packet.h"
#include "flexcomm/fsl_flexcomm.h"
#include "flexcomm/fsl_spi.h"
#include "fsl_device_registers.h"
#include "peripherals_pinmux.h"
#include "packet/serial_packet.h"

#if BL_CONFIG_FLEXCOMM_SPI

//! @addtogroup flexcomm_spi_peripheral
//! @{

////////////////////////////////////////////////////////////////////////////////
// Definitions
////////////////////////////////////////////////////////////////////////////////

#define SPI_EMPTY_CHAR (0x00) //!< Empty character.

//! @brief Synchronization state between SPI ISR and read/write functions.
typedef struct _flexcomm_spi_transfer_info
{
    const uint8_t *writeData;                                //!< The applications data to write
    volatile uint32_t bytesToTransfer;                       //!< The total number of bytes to be transmitted
    void (*data_source)(uint8_t *source_byte);               // !< Callback used to get byte to transmit.
    void (*data_sink)(uint8_t sink_byte, uint16_t instance); // !< Callback used to put received byte.
} flexcomm_spi_transfer_info_t;

////////////////////////////////////////////////////////////////////////////////
// Prototypes
////////////////////////////////////////////////////////////////////////////////

static bool flexcomm_spi_poll_for_activity(const peripheral_descriptor_t *self);
static status_t flexcomm_spi_full_init(const peripheral_descriptor_t *self, serial_byte_receive_func_t function);
static void flexcomm_spi_full_shutdown(const peripheral_descriptor_t *self);

static void flexcomm_spi_data_source(uint8_t *source_byte);

static void flexcomm_spi_initial_data_sink(uint8_t sink_byte, uint16_t instance);
static void flexcomm_spi_data_sink(uint8_t sink_byte, uint16_t instance);

static status_t flexcomm_spi_write(const peripheral_descriptor_t *self, const uint8_t *buffer, uint32_t byteCount);

static void flexcomm_spi_slave_irq_handler(uint32_t instance);

extern void FLEXCOMM_SPI_SetSystemIRQ(uint32_t instance, PeripheralSystemIRQSetting set);

////////////////////////////////////////////////////////////////////////////////
// Variables
////////////////////////////////////////////////////////////////////////////////

const peripheral_control_interface_t g_flexcommSpiControlInterface = {
    .pollForActivity = flexcomm_spi_poll_for_activity, .init = flexcomm_spi_full_init, .shutdown = flexcomm_spi_full_shutdown, .pump = 0
};

const peripheral_byte_inteface_t g_flexcommSpiByteInterface = {.init = NULL, .write = flexcomm_spi_write };

static flexcomm_spi_transfer_info_t s_flexcommSpiInfo = {.writeData = 0, .bytesToTransfer = 0, .data_source = flexcomm_spi_data_source };

//! @brief Flag for detecting device activity
static bool s_flexcommSpiActivity[FSL_FEATURE_SOC_SPI_COUNT] = { false };
static bool s_flexcommSpiIntialized[FSL_FEATURE_SOC_SPI_COUNT] = { false };

const static uint32_t g_flexcommSpiBaseAddr[] = SPI_BASE_ADDRS;

static serial_byte_receive_func_t s_flexcomm_spi_app_data_sink_callback;

////////////////////////////////////////////////////////////////////////////////
// Code
////////////////////////////////////////////////////////////////////////////////

#if defined(BL_CONFIG_FLEXCOMM_SPI_3)
/*
 * SPI3 IRQ Handler
 *
 */
void SPI3_IRQHandler()
{
    flexcomm_spi_slave_irq_handler(3);
}
#endif // defined(BL_CONFIG_FLEXCOMM_SPI_3)

#if defined(BL_CONFIG_FLEXCOMM_SPI_9)
/*
 * SPI9 IRQ Handler
 *
 */
void SPI9_IRQHandler()
{
    flexcomm_spi_slave_irq_handler(9);
}
#endif // defined(BL_CONFIG_FLEXCOMM_SPI_9)

//! @brief FLEXCOMM SPI Slave Generic IRQ handler.
//!
//! @param instance Instance number of the SPI module.
static void flexcomm_spi_slave_irq_handler(uint32_t instance)
{
    assert(instance < FSL_FEATURE_SOC_SPI_COUNT);

    SPI_Type *baseAddr = (SPI_Type *)g_flexcommSpiBaseAddr[instance];

    uint32_t status = SPI_GetStatusFlags(baseAddr);

    if (status & kSPI_RxNotEmptyFlag)
    {
        // Get byte from receive data register.
        uint8_t sink_byte = (uint8_t)SPI_ReadData(baseAddr);
        s_flexcommSpiInfo.data_sink(sink_byte, instance);
    }

    if (status & kSPI_TxNotFullFlag)
    {
        uint8_t source_byte = SPI_EMPTY_CHAR;
        s_flexcommSpiInfo.data_source(&source_byte);

        SPI_WriteData(baseAddr, source_byte, 0);
    }
}

bool flexcomm_spi_poll_for_activity(const peripheral_descriptor_t *self)
{
    return s_flexcommSpiActivity[self->instance];
}

void flexcomm_spi_data_source(uint8_t *source_byte)
{
    assert(source_byte);

    if (s_flexcommSpiInfo.bytesToTransfer)
    {
        *source_byte = *s_flexcommSpiInfo.writeData++;
        s_flexcommSpiInfo.bytesToTransfer--;
    }
    else
    {
        *source_byte = 0;
    }
}

void flexcomm_spi_initial_data_sink(uint8_t sink_byte, uint16_t instance)
{
    if (sink_byte == kFramingPacketStartByte)
    {
        s_flexcommSpiActivity[instance] = true;
        s_flexcommSpiInfo.data_sink = flexcomm_spi_data_sink;
        s_flexcomm_spi_app_data_sink_callback(sink_byte);
    }
}

void flexcomm_spi_data_sink(uint8_t sink_byte, uint16_t instance)
{
    s_flexcomm_spi_app_data_sink_callback(sink_byte);
}

status_t flexcomm_spi_full_init(const peripheral_descriptor_t *self, serial_byte_receive_func_t function)
{
    s_flexcomm_spi_app_data_sink_callback = function;
    uint32_t baseAddr = g_flexcommSpiBaseAddr[self->instance];

    s_flexcommSpiInfo.data_sink = flexcomm_spi_initial_data_sink;
    // Init the driver.
    spi_slave_config_t config;
    SPI_SlaveGetDefaultConfig(&config);

    config.phase = kSPI_ClockPhaseSecondEdge;
    config.polarity = kSPI_ClockPolarityActiveLow;

    // Configure selected pin as flexcommSpi peripheral interface
#if defined(BL_FEATURE_6PINS_PERIPHERAL) && BL_FEATURE_6PINS_PERIPHERAL
    self->pinmuxConfig(self->instance, kPinmuxType_PollForActivity);
#else
    self->pinmuxConfig(self->instance, kPinmuxType_Peripheral);
#endif // BL_FEATURE_6PINS_PERIPHERAL

    // Ungate the FLEXCOMM SPI clock.
    SPI_SlaveInit((SPI_Type *)baseAddr, &config);

    // Enable FLEXCOMM SPI interrupt
    FLEXCOMM_SPI_SetSystemIRQ(self->instance, kPeripheralEnableIRQ);
    SPI_EnableInterrupts((SPI_Type *)baseAddr, kSPI_RxLvlIrq | kSPI_TxLvlIrq);

    s_flexcommSpiIntialized[self->instance] = true;

    return kStatus_Success;
}

#if BL_FEATURE_6PINS_PERIPHERAL
#if (defined(__GNUC__))
/* #pragma GCC push_options */
/* #pragma GCC optimize("O0") */
void __attribute__((optimize("O0"))) flexcomm_spi_full_shutdown(const peripheral_descriptor_t *self)
#else
#if (defined(__ICCARM__))
#pragma optimize = none
#endif
#if (defined(__CC_ARM))
#pragma push
#pragma O0
#endif
void flexcomm_spi_full_shutdown(const peripheral_descriptor_t *self)
#endif
#else
void flexcomm_spi_full_shutdown(const peripheral_descriptor_t *self)
#endif
{
    if (s_flexcommSpiIntialized[self->instance])
    {
        uint32_t baseAddr = g_flexcommSpiBaseAddr[self->instance];

        SPI_Deinit((SPI_Type *)baseAddr);
#if BL_FEATURE_6PINS_PERIPHERAL
        // When the active peripheral is not SPI, we should only restore
        //   those pin which we used to poll for activity.
        if (g_bootloaderContext.activePeripheral == NULL)
        {
            self->pinmuxConfig(self->instance, kPinmuxType_RestoreForActivity);
        }
        // When the active peripheral is SPI, we should restore all
        //  the SPI peripheral pin.
        else
#endif
        {
            // Restore selected pin to default state to reduce IDD.
            self->pinmuxConfig(self->instance, kPinmuxType_Default);
        }

        s_flexcommSpiIntialized[self->instance] = false;
    }
}
#if BL_FEATURE_6PINS_PERIPHERAL
#if (defined(__CC_ARM))
#pragma pop
#endif
#if (defined(__GNUC__))
/* #pragma GCC pop_options */
#endif
#endif

status_t flexcomm_spi_write(const peripheral_descriptor_t *self, const uint8_t *buffer, uint32_t byteCount)
{
    s_flexcommSpiInfo.writeData = buffer;
    s_flexcommSpiInfo.bytesToTransfer = byteCount;

    while (s_flexcommSpiInfo.bytesToTransfer)
        ;

    return kStatus_Success;
}

//! @}

#endif // BL_CONFIG_FLEXCOMM_SPI

////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////

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

#include "lpspi/fsl_lpspi_master_driver.h"
#include "lpspi/hal/fsl_lpspi_hal.h"
#include "fsl_lpspi_shared_irqs.h"
#include "utilities/fsl_rtos_abstraction.h"
#include <stdlib.h>
#include <string.h>

////////////////////////////////////////////////////////////////////////////////
// Definitions
////////////////////////////////////////////////////////////////////////////////

//! @brief Runtime state of the LPSPI master driver.
//!
//! This struct holds data that are used by the LPSPI master peripheral driver to
//! communicate between the transfer function and the interrupt handler. The
//! interrupt handler also uses this information to keep track of its progress.
typedef struct LPSPIMasterState
{
    bool isTransferInProgress;          //!< True if there is an active transfer.
    bool isTransferAsync;               //!< Whether the transfer is asynchronous.
    const uint8_t *restrict sendBuffer; //!< The buffer being sent.
    uint8_t *restrict receiveBuffer;    //!< The buffer into which received bytes are placed.
    size_t remainingSendByteCount;      //!< Number of bytes remaining to send.
    size_t remainingReceiveByteCount;   //!< Number of bytes remaining to receive.
    size_t transferredByteCount;        //!< Number of bytes transferred so far.
    sync_object_t irqSync;              //!< Used to wait for ISR to complete its business.
} lpspi_master_state_t;

////////////////////////////////////////////////////////////////////////////////
// Variables
////////////////////////////////////////////////////////////////////////////////

const uint32_t g_lpspiBaseAddr[] = LPSPI_BASE_ADDRS;

//! @brief Contains global state information for the LPSPI driver.
static lpspi_master_state_t s_lpspiMasterState[LPSPI_INSTANCE_COUNT];

////////////////////////////////////////////////////////////////////////////////
// Prototypes
////////////////////////////////////////////////////////////////////////////////

static status_t lpspi_master_start_transfer(uint32_t instance,
                                            const lpspi_device_t *restrict device,
                                            bool isAsync,
                                            const uint8_t *restrict sendBuffer,
                                            uint8_t *restrict receiveBuffer,
                                            size_t transferByteCount,
                                            uint32_t timeout);
static void lpspi_master_complete_transfer(uint32_t instance);
void lpspi_master_irq_handler(uint32_t instance);

////////////////////////////////////////////////////////////////////////////////
// Code
////////////////////////////////////////////////////////////////////////////////

// See fsl_lpspi_driver.h for documentation of this function.
void lpspi_master_init(uint32_t instance)
{
    uint32_t baseAddr = g_lpspiBaseAddr[instance];

    // Enable clock for LPSPI
    SIM_SET_SCGC4(SIM_BASE, instance == HW_LPSPI0 ? SIM_SCGC4_LPSPI0_MASK : SIM_SCGC4_LPSPI1_MASK);

    // Reset the LPSPI module and switch it to master mode.
    lpspi_hal_reset(baseAddr);

    // Init LPSPI module.
    lpspi_config_t moduleConfig = {.isEnabled = false,
                                   .kbitsPerSec = 0,
                                   .masterOrSlave = kLpspiMaster,
                                   .polarity = kLpspiClockPolarity_ActiveLow,
                                   .phase = kLpspiClockPhase_SecondEdge,
                                   .shiftDirection = kLpspiMsbFirst,
                                   .ssOutputMode = kLpspiSlaveSelect_AutomaticOutput,
                                   .pinMode = kLpspiPinMode_Normal,
                                   .enableReceiveAndFaultInterrupt = false,
                                   .enableTransmitInterrupt = false,
                                   .enableMatchInterrupt = false};

    lpspi_hal_init(baseAddr, &moduleConfig);

    // Clear the state for this instance.
    memset(&s_lpspiMasterState[instance], 0, sizeof(lpspi_master_state_t));

    // Init the interrupt sync object.
    sync_init(&s_lpspiMasterState[instance].irqSync, false);

    // Configure shared IRQ.
    lpspi_set_shared_irq_is_master(instance, true);

    // Enable LPSPI interrupt.
    NVIC_EnableIRQ(instance == HW_LPSPI0 ? LPSPI0_IRQn : LPSPI1_IRQn);

    // LPSPI system enable
    lpspi_hal_enable(baseAddr);
}

// See fsl_lpspi_driver.h for documentation of this function.
void lpspi_master_shutdown(uint32_t instance)
{
    uint32_t baseAddr = g_lpspiBaseAddr[instance];

    // Restore the module to defaults then power it down.
    lpspi_hal_reset(baseAddr);
    lpspi_hal_disable(baseAddr);

    // Disable LPSPI interrupt.
    NVIC_DisableIRQ(instance == HW_LPSPI0 ? LPSPI0_IRQn : LPSPI1_IRQn);

    // Gate the clock for LPSPI.
    SIM_CLR_SCGC4(SIM_BASE, instance == HW_LPSPI0 ? SIM_SCGC4_LPSPI0_MASK : SIM_SCGC4_LPSPI1_MASK);
}

// See fsl_lpspi_driver.h for documentation of this function.
void lpspi_master_configure_bus(uint32_t instance, const lpspi_device_t *device)
{
    uint32_t baseAddr = g_lpspiBaseAddr[instance];

    // Configure the bus to access the provided device.
    lpspi_hal_set_baud(baseAddr, device->busFrequencyKHz);
    lpspi_hal_set_data_format(baseAddr, device->polarity, device->phase, device->direction);
}

// See fsl_lpspi_driver.h for documentation of this function.
status_t lpspi_master_transfer(uint32_t instance,
                               const lpspi_device_t *restrict device,
                               const uint8_t *restrict sendBuffer,
                               uint8_t *restrict receiveBuffer,
                               size_t transferByteCount,
                               uint32_t timeout)
{
    // Start the transfer.
    return lpspi_master_start_transfer(instance, device, false, sendBuffer, receiveBuffer, transferByteCount, timeout);
}

// See fsl_lpspi_driver.h for documentation of this function.
status_t lpspi_master_transfer_async(uint32_t instance,
                                     const lpspi_device_t *restrict device,
                                     const uint8_t *restrict sendBuffer,
                                     uint8_t *restrict receiveBuffer,
                                     size_t transferByteCount)
{
    // Start the transfer.
    return lpspi_master_start_transfer(instance, device, true, sendBuffer, receiveBuffer, transferByteCount, 0);
}

// See fsl_lpspi_driver.h for documentation of this function.
status_t lpspi_master_get_tranfer_status(uint32_t instance, uint32_t *bytesTransferred)
{
    lpspi_master_state_t *state = &s_lpspiMasterState[instance];

    // Fill in the bytes transferred.
    if (bytesTransferred)
    {
        *bytesTransferred = state->transferredByteCount;
    }

    return (state->isTransferInProgress ? kStatus_LPSPI_Busy : kStatus_Success);
}

// See fsl_lpspi_driver.h for documentation of this function.
status_t lpspi_master_abort_transfer(uint32_t instance)
{
    lpspi_master_state_t *state = &s_lpspiMasterState[instance];

    // Check if a transfer is running.
    if (!state->isTransferInProgress)
    {
        return kStatus_LPSPI_NoTransferInProgress;
    }

    // Stop the running transfer.
    lpspi_master_complete_transfer(instance);

    return kStatus_Success;
}

//! @brief Initiate a transfer.
static status_t lpspi_master_start_transfer(uint32_t instance,
                                            const lpspi_device_t *restrict device,
                                            bool isAsync,
                                            const uint8_t *restrict sendBuffer,
                                            uint8_t *restrict receiveBuffer,
                                            size_t transferByteCount,
                                            uint32_t timeout)
{
    lpspi_master_state_t *state = &s_lpspiMasterState[instance];
    uint32_t baseAddr = g_lpspiBaseAddr[instance];

    // If the byte count is zero, then return immediately.
    if (transferByteCount == 0)
    {
        return kStatus_Success;
    }

    // Check that we're not busy.
    if (state->isTransferInProgress)
    {
        return kStatus_LPSPI_Busy;
    }

    // Configure bus for this device. If NULL is passed, we assume the caller has
    // preconfigured the bus using lpspi_master_configure_bus().
    if (device)
    {
        lpspi_master_configure_bus(instance, device);
    }

    // Save information about the transfer for use by the ISR.
    state->isTransferInProgress = true;
    state->isTransferAsync = isAsync;
    state->sendBuffer = (const uint8_t *)sendBuffer;
    state->receiveBuffer = (uint8_t *)receiveBuffer;
    state->remainingSendByteCount = transferByteCount;
    state->remainingReceiveByteCount = transferByteCount;
    state->transferredByteCount = 0;

    // Enable the LPSPI module for the duration of this transfer.
    lpspi_hal_enable(baseAddr);

    // Clear the read buffer if there's anything in it.
    if (spi_hal_is_read_buffer_full(baseAddr))
    {
        uint8_t unused = lpspi_hal_read_data(baseAddr);
        unused = unused; // Keep compiler happy.
    }

    // Wait until the transmit buffer is empty (which it should already be).
    while (!spi_hal_is_transmit_buffer_empty(baseAddr))
    {
        // TODO: Add timeout check.
    }

    // Start the transfer by writing the first byte. If a send buffer was provided, the byte
    // comes from there. Otherwise we just send a zero byte.
    uint8_t byteToSend = 0;
    if (state->sendBuffer)
    {
        byteToSend = *(state->sendBuffer);
        ++state->sendBuffer;
        --state->remainingSendByteCount;
    }
    lpspi_hal_write_data(baseAddr, byteToSend);

    // Enable interrupts.
    lpspi_hal_enable_receive_and_fault_interrupt(baseAddr);
    lpspi_hal_enable_transmit_interrupt(baseAddr);

    // If this is a synchronous transfer, wait until the transfer is complete.
    status_t error = kStatus_Success;
    if (!isAsync)
    {
        if (!sync_wait(&state->irqSync, timeout))
        {
            // Abort the transfer so it doesn't continue unexpectedly.
            lpspi_master_abort_transfer(instance);

            // Return an error.
            error = kStatus_LPSPI_Timeout;
        }
    }

    return error;
}

//! @brief Finish up a transfer.
//!
//! Cleans up after a transfer is complete. Interrupts are disabled, and the LPSPI module
//! is disabled.
static void lpspi_master_complete_transfer(uint32_t instance)
{
    lpspi_master_state_t *state = &s_lpspiMasterState[instance];
    uint32_t baseAddr = g_lpspiBaseAddr[instance];

    // The transfer is complete.
    state->isTransferInProgress = false;

    // Disable interrupts.
    lpspi_hal_disable_receive_and_fault_interrupt(baseAddr);
    lpspi_hal_disable_transmit_interrupt(baseAddr);

    // Transfer is complete, so disable the module.
    lpspi_hal_disable(baseAddr);
}

//! @brief Interrupt handler for LPSPI master mode.
//!
//! This handler uses the buffers stored in the #s_lpspiMasterState structs to transfer data.
void lpspi_master_irq_handler(uint32_t instance)
{
    lpspi_master_state_t *state = &s_lpspiMasterState[instance];
    uint32_t baseAddr = g_lpspiBaseAddr[instance];

    // Exit the ISR if no transfer is happening for this instance.
    if (!state->isTransferInProgress)
    {
        return;
    }

    // Check write buffer. We always have to send a byte in order to keep the transfer
    // moving. So if the caller didn't provide a send buffer, we just send a zero byte.
    uint8_t byteToSend = 0;
    if (state->remainingSendByteCount)
    {
        if (lpspi_hal_is_transmit_buffer_empty(baseAddr))
        {
            if (state->sendBuffer)
            {
                byteToSend = *(state->sendBuffer);
                ++state->sendBuffer;
            }
            --state->remainingSendByteCount;
            ++state->transferredByteCount;

            lpspi_hal_write_data(baseAddr, byteToSend);
        }
    }

    // Check read buffer.
    uint8_t byteReceived;
    if (state->remainingReceiveByteCount)
    {
        if (lpspi_hal_is_read_buffer_full(baseAddr))
        {
            byteReceived = lpspi_hal_read_data(baseAddr);

            if (state->receiveBuffer)
            {
                *state->receiveBuffer = byteReceived;
                ++state->receiveBuffer;
            }
            --state->remainingReceiveByteCount;
        }
    }

    // Check if we're done with this transfer.
    if ((state->remainingSendByteCount == 0) && (state->remainingReceiveByteCount == 0))
    {
        // Complete the transfer. This disables the interrupts, so we don't wind up in
        // the ISR again.
        lpspi_master_complete_transfer(instance);

        // Signal the synchronous completion object if the transfer wasn't async.
        if (!state->isTransferAsync)
        {
            sync_signal(&state->irqSync);
        }
    }
}

////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////

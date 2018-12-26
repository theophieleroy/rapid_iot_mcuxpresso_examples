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
#ifndef __LPI2C_SLAVE_H__
#define __LPI2C_SLAVE_H__

#include "bootloader_common.h"

////////////////////////////////////////////////////////////////////////////////
// Definitions
////////////////////////////////////////////////////////////////////////////////

//! @brief LPI2C slave driver status codes.
enum _lpi2c_errors
{
    kStatus_LPI2C_SlaveTxUnderrun = MAKE_STATUS(kStatusGroup_I2CDriver, 0), //!< LPI2C Slave TX Underrun error.
    kStatus_LPI2C_SlaveRxOverrun = MAKE_STATUS(kStatusGroup_I2CDriver, 1),  //!< LPI2C Slave RX Overrun error.
    kStatus_LPI2C_AribtrationLost = MAKE_STATUS(kStatusGroup_I2CDriver, 2), //!< LPI2C Arbitration Lost error.
};

//! @brief Definition of application implemented callback functions used by the
//!     LPI2C slave driver.
typedef struct _lpi2c_slave_info
{
    void (*data_source)(uint8_t *source_byte);               // !< Callback used to get byte to transmit.
    void (*data_sink)(uint8_t sink_byte, uint32_t instance); // !< Callback used to put received byte.
    uint8_t slaveAddress;    // !< 7-bit slave address to use. The address must be right-aligned in the byte.
    bool isReadyForShutdown; // !< flag used to make sure that LPI2C can only be shut down after current transfer has
                             // been completed
} lpi2c_slave_info_t;

////////////////////////////////////////////////////////////////////////////////
// API
////////////////////////////////////////////////////////////////////////////////

#if defined(__cplusplus)
extern "C" {
#endif

//! @name LPI2C Slave
//@{

//! @brief Initializes the LPI2C module.
//!
//! Saves the application callback info, turns on the clock to the module,
//! Enables the device and enables interrupts. Set the LPI2C to slave mode.
//! IOMux is expected to already be handled in init_hardware().
//!
//! @param instance Instance number of the LPI2C module.
//! @param appInfo Pointer of application information
void lpi2c_slave_init(int instance, lpi2c_slave_info_t *appInfo);

//! @brief Deinitializes the device.
//!
//! Clears the control register and turns off the clock to the module.
//!
//! @param instance Instance number of the LPI2C module.
void lpi2c_slave_deinit(int instance);

//! @brief Configure LPI2C to filter glitches below a certain width.
//!
//! The glitch filter width will be set to as close to just below the requested width
//! as is possible given the bus clock frequency. If a single bus clock cycle
//! is longer than the requested width, the filter will be set to a single cycle.
//! However, if the bus clock cycle is more than 10 times the requested width, the
//! glitch filter will be disabled.
void lpi2c_set_glitch_filter_width(int instance, uint32_t busClock_Hz, uint32_t glitchWidth_ns);

/*!
 * @brief Set the data sink function pointer.
 *
 * Set the data sink function pointer
 *
 * @param instance Instance number of the LPI2C module.
 * @param data_sink Pointer to data_sink function.
 */
void lpi2c_slave_set_data_sink_func(uint32_t instance, void (*data_sink)(uint8_t, uint32_t));

//@}

#if defined(__cplusplus)
}
#endif

#endif // ifndef __LPI2C_SLAVE_H__
////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////

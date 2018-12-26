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

#if !defined(__FSL_SPI_SLAVE_DRIVER_H__)
#define __FSL_SPI_SLAVE_DRIVER_H__

#include "bootloader_common.h"
#include "spi/fsl_spi_types.h"

//! @addtogroup spi_slave_driver
//! @{

////////////////////////////////////////////////////////////////////////////////
// Definitions
////////////////////////////////////////////////////////////////////////////////

//! @brief The set of callbacks used for SPI slave mode.
typedef struct SPISlaveCallbacks
{
    //! Callback used to get byte to transmit.
    void (*dataSource)(uint8_t *sourceByte, uint16_t instance);

    //! Callback used to put received byte.
    void (*dataSink)(uint8_t sinkByte, uint16_t instance);
} spi_slave_callbacks_t;

//! @brief Definition of application implemented configuration and callback
//! functions used by the SPI slave driver.
typedef struct SPISlaveConfig
{
    spi_slave_callbacks_t callbacks; //!< Application callbacks.
    spi_clock_phase_t phase;         //!< Clock phase setting.
    spi_clock_polarity_t polarity;   //!< Clock polarity setting.
    spi_shift_direction_t direction; //!< Either LSB or MSB first.
} spi_slave_config_t;

////////////////////////////////////////////////////////////////////////////////
// API
////////////////////////////////////////////////////////////////////////////////

#if defined(__cplusplus)
extern "C" {
#endif

//! @name SPI Slave
//@{

/*!
 * @brief Initializes the SPI module.
 *
 * Saves the application callback info, turns on the clock to the module,
 * Enables the device and enables interrupts. Set the SPI to slave mode.
 *
 * @param instance Instance number of the SPI module.
 * @param config Pointer to slave mode configuration.
 */
void spi_slave_init(uint32_t instance, const spi_slave_config_t *config);

/*!
 * @brief Set the data sink function pointer.
 *
 * Set the data sink function pointer
 *
 * @param instance Instance number of the SPI module.
 * @param data_sink Pointer to data_sink function.
 */
void spi_slave_set_data_sink_func(uint32_t instance, void (*data_sink)(uint8_t, uint16_t));

/*!
 * @brief Deinitializes the device.
 *
 * Clears the control register and turns off the clock to the module.
 *
 * @param instance Instance number of the SPI module.
 */
void spi_slave_shutdown(uint32_t instance);

//@}

#if defined(__cplusplus)
}
#endif

//! @}

#endif // __FSL_SPI_SLAVE_DRIVER_H__
////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////

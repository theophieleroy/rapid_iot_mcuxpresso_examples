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
#if !defined(__FSL_DSPI_SLAVE_DRIVER_H__)
#define __FSL_DSPI_SLAVE_DRIVER_H__

#include "bootloader_common.h"
#include "dspi/fsl_dspi_types.h"
#include "dspi/hal/fsl_dspi_hal.h"

/*!
 * @addtogroup dspi_slave_driver
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*!
 * @brief The set of callbacks used for the DSPI slave mode.
 *
 * The user will need to create the function implementations.
 */
typedef struct DSPISlaveCallbacks
{
    /*! Callback used to get word to transmit */
    void (*dataSource)(uint8_t *sourceWord, uint32_t instance);

    /*! Callback used to put received word */
    void (*dataSink)(uint8_t sinkWord, uint32_t instance);

    /*! Callback used to report a DSPI error, such as an under-run or over-run error */
    void (*onError)(status_t error, uint32_t instance);
} dspi_slave_callbacks_t;

/*!
 * @brief Runtime state of the DSPI slave driver.
 *
 * This struct holds data that are used by the DSPI slave peripheral driver to
 * communicate between the transfer function and the interrupt handler. The user just
 * needs to pass in the memory for this structure, the driver will take care of filling out
 * the members.
 */
typedef struct DSPISlaveState
{
    uint32_t instance;                /*!< DSPI module instance number */
    dspi_slave_callbacks_t callbacks; /*!< Application/user callbacks. */
    uint32_t bitsPerFrame;            /*!< Desired number of bits per frame */
} dspi_slave_state_t;

/*!
 *  @brief User configuration structure and callback functions for the DSPI slave driver.
 */
typedef struct DSPISlaveUserConfig
{
    dspi_slave_callbacks_t callbacks;     /*!< Application/user callbacks. */
    dspi_data_format_config_t dataConfig; /*!< Data format configuration structure */
} dspi_slave_user_config_t;

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @name Init and shutdown
 * @{
 */

/*!
 * @brief Initialize a DSPI instance for slave mode operation.
 *
 * This function saves the callbacks to the run-time state structure for later use in the
 * interrupt handler. It also ungates the clock to the DSPI module, initializes the DSPI
 * for slave mode, enables the module and corresponding interrupts. Once initialized, the
 * DSPI module is configured in slave mode and ready to receive data from a SPI master. The
 * following is an example of how to set up the dspi_slave_state_t and the dspi_slave_user_config_t
 * parameters and how to call the dspi_slave_init function by passing in these parameters:
 * @code
    dspi_slave_state_t dspiSlaveState; <- the user simply allocates memory for this struct
    dspi_slave_user_config_t slaveUserConfig;
    slaveUserConfig.callbacks.dataSink = data_sink; <- set to user implementation of function
    slaveUserConfig.callbacks.dataSource = data_source; <- set to user implementation of function
    slaveUserConfig.callbacks.onError = on_error; <- set to user implementation of function
    slaveUserConfig.dataConfig.bitsPerFrame = 16;
    slaveUserConfig.dataConfig.clkPhase = kDspiClockPhase_FirstEdge;
    slaveUserConfig.dataConfig.clkPolarity = kDspiClockPolarity_ActiveHigh;
    dspi_slave_init(slaveInstance, &slaveUserConfig, &dspiSlaveState);
 * @endcode
 *
 * @param instance The instance number of the DSPI peripheral.
 * @param dspiState The pointer to the DSPI slave driver state structure.
 * @param user The configuration structure of type dspi_slave_user_config_t, including
 *  the callbacks.
 *
 * @return An error code or kStatus_Success.
 */
status_t dspi_slave_init(uint32_t instance, const dspi_slave_user_config_t *slaveConfig, dspi_slave_state_t *dspiState);

/*!
 * @brief Shutdown a DSPI instance.
 * Resets the DSPI peripheral, disables the interrupt to the core, and gates its clock.
 *
 * @param dspiState The pointer to the DSPI slave driver state structure.
 */
void dspi_slave_shutdown(dspi_slave_state_t *dspiState);

/* @} */

#if defined(__cplusplus)
}
#endif

/*! @} */

#endif /* __FSL_DSPI_SLAVE_DRIVER_H__ */
       /*******************************************************************************
        * EOF
        ******************************************************************************/

/*
 * The Clear BSD License
 * Copyright (c) 2013 - 2014, Freescale Semiconductor, Inc.
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
#ifndef __FSL_I2C_MASTER_DRIVER_H__
#define __FSL_I2C_MASTER_DRIVER_H__

#include <stdlib.h>
#include <stdbool.h>
#include "i2c/master/hal/fsl_i2c_hal.h"
#include "utilities/fsl_rtos_abstraction.h"

/*!
 * @addtogroup i2c_master
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*! @brief Information necessary to communicate with an I2C slave device.*/
typedef struct I2CDevice
{
    uint16_t address;       /*!< Slave's 7-bit or 10-bit address. If 10-bit address,
                                 the first 6 bits must be 011110 in binary.*/
    uint32_t baudRate_kbps; /*!< The baud rate in kbps to use with this slave device.*/
} i2c_device_t;

/*!
 * @brief Internal driver state information.
 *
 * @note The contents of this structure are internal to the driver and should not be
 *      modified by users. Also, contents of the structure are subject to change in
 *      future releases.
 */
typedef struct I2CMasterState
{
    uint8_t *rxBuff;
    volatile uint32_t rxSize;
    uint8_t *txBuff;
    volatile uint32_t txSize;
    volatile i2c_status_t status;
    volatile bool i2cIdle;
    uint32_t lastBaudRate_kbps;
    semaphore_t irqSync;
} i2c_master_state_t;

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @name I2C Master
 * @{
 */

/*!
 * @brief Initialize the I2C master mode driver.
 *
 * @param instance The I2C peripheral instance number.
 * @param master   The pointer to the I2C master driver state structure.
 */
void I2C_DRV_MasterInit(uint32_t instance, i2c_master_state_t *master);

/*!
 * @brief Shut down the driver.
 *
 * @param instance The I2C peripheral instance number.
 */
void I2C_DRV_MasterDeinit(uint32_t instance);

/*!
 * @brief Configure the I2C bus to access a device.
 *
 * @param instance The I2C peripheral instance number.
 * @param device   The pointer to the I2C device information struct.
 */
void I2C_DRV_MasterSetBaud(uint32_t instance, const i2c_device_t *device);

/*!
 * @brief Perform a blocking send transaction on the I2C bus.
 *
 * @param instance    The I2C peripheral instance number.
 * @param device      The pointer to the I2C device information struct.
 * @param cmdBuff     The pointer to the commands to be transfered.
 * @param cmdSize     The length in bytes of the commands to be transfered.
 * @param txBuff      The pointer to the data to be transfered.
 * @param txSize      The length in bytes of the data to be transfered.
 * @param timeout_ms  The timeout in milliseconds to wait for the transfer to finish
 */
i2c_status_t I2C_DRV_MasterSendDataBlocking(uint32_t instance,
                                            const i2c_device_t *device,
                                            uint8_t *cmdBuff,
                                            uint32_t cmdSize,
                                            uint8_t *txBuff,
                                            uint32_t txSize,
                                            uint32_t timeout_ms);

/*!
 * @brief Perform a blocking receive transaction on the I2C bus.
 *
 * @param instance    The I2C peripheral instance number.
 * @param device      The pointer to the I2C device information struct.
 * @param cmdBuff     The pointer to the commands to be transfered.
 * @param cmdSize     The length in bytes of the commands to be transfered.
 * @param rxBuff      The pointer to the data to be transfered.
 * @param rxSize      The length in bytes of the data to be transfered.
 * @param timeout_ms  The timeout in milliseconds to wait for the transfer to finish
 */
i2c_status_t I2C_DRV_MasterReceiveDataBlocking(uint32_t instance,
                                               const i2c_device_t *device,
                                               uint8_t *cmdBuff,
                                               uint32_t cmdSize,
                                               uint8_t *rxBuff,
                                               uint32_t rxSize,
                                               uint32_t timeout_ms);

/* @} */

#if defined(__cplusplus)
}
#endif

/*! @}*/

#endif /* __FSL_I2C_MASTER_DRIVER_H__*/
       /*******************************************************************************
        * EOF
        ******************************************************************************/

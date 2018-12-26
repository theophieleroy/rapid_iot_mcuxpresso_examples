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
#if !defined(__FSL_LPSPI_DRIVER_H__)
#define __FSL_LPSPI_DRIVER_H__

#include "bootloader_common.h"
#include "spi/fsl_lpspi_types.h"
#include <stdint.h>
#include <stdlib.h>

//! @addtogroup lpspi_master_driver
//! @{

////////////////////////////////////////////////////////////////////////////////
// Definitions
////////////////////////////////////////////////////////////////////////////////

enum _lpspi_timeouts
{
    //! Wait forever for a transfer to complete.
    kLpspiWaitForever = 0xffffffffU
};

//! @brief Information about a device on the LPSPI bus.
typedef struct LPSPIDevice
{
    uint32_t busFrequencyKHz;
    lpspi_clock_polarity_t polarity;
    lpspi_clock_phase_t phase;
    lpspi_shift_direction_t direction;
} lpspi_device_t;

////////////////////////////////////////////////////////////////////////////////
// API
////////////////////////////////////////////////////////////////////////////////

#if defined(__cplusplus)
extern "C" {
#endif

//! @name Init and shutdown
//@{

/*!
 * @brief Initialize a LPSPI instance for master mode operation.
 *
 * @param instance The instance number of the LPSPI peripheral.
 */
void lpspi_master_init(uint32_t instance);

/*!
 * @brief Shutdown a LPSPI instance.
 *
 * Resets the LPSPI peripheral and gates its clock.
 *
 * @param instance The instance number of the LPSPI peripheral.
 */
void lpspi_master_shutdown(uint32_t instance);

//@}

//! @name Bus configuration
//@{

/*!
 * @brief Configure the LPSPI port to access a device on the bus.
 *
 * @param instance The instance number of the LPSPI peripheral.
 * @param device Pointer to the device information struct. This struct contains the settings
 *      for how the LPSPI bus will be configured.
 */
void lpspi_master_configure_bus(uint32_t instance, const lpspi_device_t *device);

//@}

//! @name Blocking transfers
//@{

/*!
 * @brief Perform a blocking LPSPI master mode transfer.
 *
 * This function simultaneously sends and receives data on the LPSPI bus, as LPSPI is naturally
 * a full-duplex bus.
 *
 * @param instance The instance number of the LPSPI peripheral.
 * @param device Pointer to the device information struct. This struct contains the settings
 *      for how the LPSPI bus will be configured for this transfer. You may pass NULL for this
 *      parameter, in which case the current bus configuration is used unmodified.
 * @param sendBuffer Buffer of data to send. You may pass NULL for this parameter, in which case
 *      bytes with a value of 0 (zero) will be sent.
 * @param receiveBuffer Buffer where received bytes are stored. If you pass NULL for this parameter,
 *      the received bytes are ignored.
 * @param transferByteCount The number of bytes to send and receive.
 * @param timeout A timeout for the transfer in microseconds. If the transfer takes longer than
 *      this amount of time, the transfer will be aborted and a #kStatus_LPSPI_Timeout error will be
 *      returned.
 *
 * @retval #kStatus_Success The transfer was succesful.
 * @retval #kStatus_LPSPI_Busy Cannot perform another transfer because one is already in progress.
 * @retval #kStatus_LPSPI_Timeout The transfer timed out and was aborted.
 */
status_t lpspi_master_transfer(uint32_t instance,
                               const lpspi_device_t *restrict device,
                               const uint8_t *restrict sendBuffer,
                               uint8_t *restrict receiveBuffer,
                               size_t transferByteCount,
                               uint32_t timeout);

//@}

//! @name Non-blocking transfers
//@{

/*!
 * @brief Perform an non-blocking LPSPI master mode transfer.
 *
 * @param instance The instance number of the LPSPI peripheral.
 * @param device Pointer to the device information struct. This struct contains the settings
 *      for how the LPSPI bus will be configured for this transfer. You may pass NULL for this
 *      parameter, in which case the current bus configuration is used unmodified.
 * @param sendBuffer Buffer of data to send. You may pass NULL for this parameter, in which case
 *      bytes with a value of 0 (zero) will be sent.
 * @param receiveBuffer Buffer where received bytes are stored. If you pass NULL for this parameter,
 *      the received bytes are ignored.
 * @param transferByteCount The number of bytes to send and receive.
 *
 * @retval #kStatus_Success The transfer was succesful.
 * @retval #kStatus_LPSPI_Busy Cannot perform another transfer because one is already in progress.
 * @retval #kStatus_LPSPI_Timeout The transfer timed out and was aborted.
 */
status_t lpspi_master_transfer_async(uint32_t instance,
                                     const lpspi_device_t *restrict device,
                                     const uint8_t *restrict sendBuffer,
                                     uint8_t *restrict receiveBuffer,
                                     size_t transferByteCount);

/*!
 * @brief Returns whether the previous transfer has finished yet.
 *
 * @param instance The instance number of the LPSPI peripheral.
 * @param bytesTransferred Pointer to value that will be filled in with the number of bytes that
 *      have been sent in the active transfer
 *
 * @retval kStatus_Success The transfer has completed successfully.
 * @retval kStatus_LPSPI_Busy The transfer is still in progress. @a bytesTransferred will be filled
 *      with the number of bytes that have been transferred so far.
 */
status_t lpspi_master_get_tranfer_status(uint32_t instance, uint32_t *bytesTransferred);

/*!
 * @brief Terminates an asynchronous transfer early.
 *
 * @param instance The instance number of the LPSPI peripheral.
 *
 * @retval #kStatus_Success The transfer was succesful.
 */
status_t lpspi_master_abort_transfer(uint32_t instance);

//@}

#if defined(__cplusplus)
}
#endif

//! @}

#endif // __FSL_LPSPI_DRIVER_H__
////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////

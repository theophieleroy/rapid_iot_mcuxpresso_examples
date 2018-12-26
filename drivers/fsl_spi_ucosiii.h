/*
 * The Clear BSD License
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
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
#ifndef __FSL_SPI_UCOSIII_H__
#define __FSL_SPI_UCOSIII_H__

#include <os.h>
#include "fsl_spi.h"

/*!
 * @addtogroup spi_ucosiii_driver SPI uCOS III driver
 * @{
 */

/**********************************************************************************************************************
 * Definitions
 *********************************************************************************************************************/
/*! @brief SPI uCOS III driver version */
#define FSL_SPI_UCOSIII_DRIVER_VERSION (MAKE_VERSION(2, 0, 0)) /*!< Version 2.0.0. */

/*! @brief SPI RTOS handle */
typedef struct _spi_rtos_handle
{
    SPI_Type *base;                 /*!< SPI base address */
    spi_master_handle_t drv_handle; /*!< Handle of the underlying driver, treated as opaque by the RTOS layer */
    status_t async_status;
    OS_SEM mutex; /*!< Mutex to lock the handle during a trasfer */
#define RTOS_SPI_COMPLETE 0x1
    OS_FLAG_GRP event; /*!< Semaphore to notify and unblock task when transfer ends */
} spi_rtos_handle_t;

/**********************************************************************************************************************
 * API
 *********************************************************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @name SPI RTOS Operation
 * @{
 */

/*!
 * @brief Initializes SPI.
 *
 * This function initializes the SPI module and related RTOS context.
 *
 * @param handle The RTOS SPI handle, the pointer to an allocated space for RTOS context.
 * @param base The pointer base address of the SPI instance to initialize.
 * @param masterConfig Configuration structure to set-up SPI in master mode.
 * @param srcClock_Hz Frequency of input clock of the SPI module.
 * @return status of the operation.
 */
status_t SPI_RTOS_Init(spi_rtos_handle_t *handle,
                       SPI_Type *base,
                       const spi_master_config_t *masterConfig,
                       uint32_t srcClock_Hz);

/*!
 * @brief Deinitializes the SPI.
 *
 * This function deinitializes the SPI module and related RTOS context.
 *
 * @param handle The RTOS SPI handle.
 */
status_t SPI_RTOS_Deinit(spi_rtos_handle_t *handle);

/*!
 * @brief Performs SPI transfer.
 *
 * This function performs an SPI transfer according to data given in the transfer structure.
 *
 * @param handle The RTOS SPI handle.
 * @param transfer Structure specifying the transfer parameters.
 * @return status of the operation.
 */
status_t SPI_RTOS_Transfer(spi_rtos_handle_t *handle, spi_transfer_t *transfer);

/*!
 * @}
 */

#if defined(__cplusplus)
}
#endif

/*!
 * @}
 */

#endif /* __FSL_SPI_UCOSIII_H__ */

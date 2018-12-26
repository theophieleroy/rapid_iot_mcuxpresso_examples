/*
 * The Clear BSD License
 * Copyright (c) 2017, NXP Semiconductors, Inc.
 * All rights reserved.
 *
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided
 * that the following conditions are met:
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

#ifndef _FSL_NOR_Flash_H_
#define _FSL_NOR_Flash_H_

#include "fsl_common.h"
#include "fsl_spi_adapter.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @brief Initializes the Flash peripheral.
 *
 * @param handle Flash operation handle.
 * @param config Pointer to the user-defined configuration structure.
 * @return Returns @ref flash_ok if initialize success, otherwise returns
 * error code.
 */
flash_err_t NOR_Flash_Init(flash_handle_t *handle, flash_config_t *config);

/*!
 * @brief Reads the specified number of bytes from NOR_Flash memory.
 *
 * @param handle Flash operation handle.
 * @param address NOR_Flash address to start reading from.
 * @param data Pointer to a memory location where the data read out from
 * NOR_Flash will be stored.
 * @param dataSize Number of bytes to be read.
 * @return Returns @ref flash_ok if read success, otherwise returns
 * error code.
 */
flash_err_t NOR_Flash_ReadData(flash_handle_t *handle, uint32_t address, uint8_t *data, uint32_t dataSize);

/*!
 * @brief Writes the specified number of bytes to NOR_Flash memory.
 *
 * @param handle Flash operation handle.
 * @param address NOR_Flash address to start writing to.
 * @param data Pointer to a memory location where the data data to be
 * written to NOR_Flash is stored.
 * @param dataSize Number of bytes to be written.
 * @return Returns @ref flash_ok if write success, otherwise returns
 * error code.
 */
flash_err_t NOR_Flash_WriteData(flash_handle_t *handle, uint32_t address, uint8_t *data, uint32_t dataSize);

/*!
 * @brief Get the NOR_Flash busy status.
 *
 * @param handle Flash operation handle.
 * @return Returns @ref flash_busy if flash busy, otherwise returns
 * flash_ok. If error happens, return error code.
 */
flash_err_t NOR_Flash_isBusy(flash_handle_t *handle);

/*!
 * @brief Erase a block of block size memory
 *
 * @param handle Flash operation handle.
 * @param address NOR_Flash address to start erase.
 * @param blockSize Erase block size.
 * @return Returns @ref flash_ok if erase success, otherwise return error code.
 */
flash_err_t NOR_Flash_EraseBlock(flash_handle_t *handle, uint32_t address, uint32_t blockSize);

/*!
 * @brief Erase the entire NOR_Flash memory.
 *
 * @param handle Flash operation handle.
 * @return Returns @ref flash_ok if erase success, otherwise return error code.
 */
flash_err_t NOR_Flash_ChipErase(flash_handle_t *handle);

#if defined(__cplusplus)
}
#endif

#endif

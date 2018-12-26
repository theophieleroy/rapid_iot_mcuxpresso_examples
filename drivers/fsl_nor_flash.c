/*
 * The Clear BSD License
 * Copyright (c) 2017, NXP Semiconductors, Inc.
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

#include "fsl_nor_flash.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
* Prototypes
******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/

static flash_err_t NOR_Flash_WriteEnable(flash_handle_t *handle)
{
    flash_transfer_t xfer;

    xfer.opCode = kNorFlash_WriteEnable;
    xfer.data = NULL;
    xfer.dataSize = 0;

    return handle->executeCommand(handle->resource, &xfer);
}

flash_err_t NOR_Flash_Init(flash_handle_t *handle, flash_config_t *config)
{
    assert(handle);

    handle->totalSize = config->totalSize;
    handle->pageSize = config->pageSize;
    handle->sectorSize = config->sectorSize;
    handle->needWriteEnable = config->needWriteEnable;
    handle->statusValueSize = config->statusValueSize;
    handle->statusBusyMask = config->statusBusyMask;
    handle->statusBusyValue = config->statusBusyValue;

    return flash_ok;
}

flash_err_t NOR_Flash_isBusy(flash_handle_t *handle)
{
    flash_transfer_t xfer;
    uint32_t statusValue;
    flash_err_t result = flash_ok;

    xfer.opCode = kNorFlash_ReadStatus;
    xfer.data = (uint8_t *)&statusValue;
    xfer.dataSize = handle->statusValueSize;

    result = handle->executeCommand(handle->resource, &xfer);
    if (result != flash_ok)
    {
        return result;
    }

    if (statusValue & handle->statusBusyMask)
    {
        if (handle->statusBusyValue)
        {
            return flash_busy;
        }
    }
    else
    {
        if (!(handle->statusBusyValue))
        {
            return flash_busy;
        }
    }

    return flash_ok;
}

flash_err_t NOR_Flash_ChipErase(flash_handle_t *handle)
{
    flash_transfer_t xfer;
    flash_err_t result = flash_ok;

    while (NOR_Flash_isBusy(handle) != flash_ok)
        ;

    if (handle->needWriteEnable)
    {
        result = NOR_Flash_WriteEnable(handle);
        if (result != flash_ok)
        {
            return result;
        }
    }

    xfer.opCode = kNorFlash_EraseChip;
    xfer.data = NULL;
    xfer.dataSize = 0;

    result = handle->executeCommand(handle->resource, &xfer);
    if (result != flash_ok)
    {
        return result;
    }

    while (NOR_Flash_isBusy(handle) != flash_ok)
        ;

    return result;
}

flash_err_t NOR_Flash_EraseBlock(flash_handle_t *handle, uint32_t address, uint32_t blockSize)
{
    flash_transfer_t xfer;
    flash_err_t result = flash_ok;

    assert(!(address % blockSize));

    while (NOR_Flash_isBusy(handle) != flash_ok)
        ;

    if (handle->needWriteEnable)
    {
        result = NOR_Flash_WriteEnable(handle);
        if (result != flash_ok)
        {
            return result;
        }
    }

    xfer.opCode = kNorFlash_EraseSector;
    xfer.address = address;
    xfer.data = NULL;
    xfer.dataSize = blockSize;

    result = handle->executeCommand(handle->resource, &xfer);
    if (result != flash_ok)
    {
        return result;
    }

    while (NOR_Flash_isBusy(handle) != flash_ok)
        ;

    return result;
}

flash_err_t NOR_Flash_ReadData(flash_handle_t *handle, uint32_t address, uint8_t *data, uint32_t dataSize)
{
    flash_transfer_t xfer;

    while (NOR_Flash_isBusy(handle) != flash_ok)
        ;

    xfer.opCode = kNorFlash_Read;
    xfer.address = address;
    xfer.data = data;
    xfer.dataSize = dataSize;

    return handle->executeCommand(handle->resource, &xfer);
}

flash_err_t NOR_Flash_WriteData(flash_handle_t *handle, uint32_t address, uint8_t *data, uint32_t dataSize)
{
    flash_transfer_t xfer;
    flash_err_t result = flash_ok;

    while (NOR_Flash_isBusy(handle) != flash_ok)
        ;

    if (handle->needWriteEnable)
    {
        result = NOR_Flash_WriteEnable(handle);
        if (result != flash_ok)
        {
            return result;
        }
    }

    xfer.opCode = kNorFlash_Write;
    xfer.address = address;
    xfer.data = data;
    xfer.dataSize = dataSize;

    return handle->executeCommand(handle->resource, &xfer);
}

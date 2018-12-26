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

#ifndef _FSL_Flash_H_
#define _FSL_Flash_H_

#include "fsl_common.h"
#include "fsl_spi_adapter.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define NOR_FLASH_MAX_COMMAND_SZIE 4

/*! @brief Flash error code structure. */
typedef enum
{
    flash_ok,
    flash_too_big,
    flash_not_aligned,
    flash_busy,
    flash_error
} flash_err_t;

/*! @brief Flash operation code structure. */
typedef enum _flash_op_code
{
    kNorFlash_WriteEnable = 0x0, /*!< Operation code: WREN. */
    kNorFlash_Write = 0x1,       /*!< Operation code: Page Program. */
    kNorFlash_Read = 0x2,        /*!< Operation code: Read Data. */
    kNorFlash_ReadStatus = 0x3,  /*!< Operation code: Read Status Register. */
    kNorFlash_EraseSector = 0x4, /*!< Operation code: Erase sector. */
    kNorFlash_EraseChip = 0x5,   /*!< Operation code: Erase whole chip. */
} flash_op_code_t;

/*! @brief Flash config structure. */
typedef struct _flash_config
{
    uint32_t totalSize;      /*!< Total flash size. */
    uint32_t pageSize;       /*!< Page size. */
    uint32_t sectorSize;     /*!< Sector size. */
    bool needWriteEnable;    /*!< Need do write enable before write/erase operation. */
    uint8_t statusValueSize; /*!< Status value size in bytes. */
    uint32_t statusBusyMask; /*!< Status busy mask. */
    uint8_t statusBusyValue; /*!< Status busy value. */
} flash_config_t;

/*! @brief Flash transfer structure. */
typedef struct _flash_transfer
{
    flash_op_code_t opCode; /*!< Operation code. */
    uint8_t address;        /*!< Program/write address. */
    uint8_t *data;          /*!< Data buffer pointer for program buffer or store buffer. */
    uint32_t dataSize;      /*!< Data buffer size. */
} flash_transfer_t;

/*! @brief Flash handle. */
typedef struct _flash_handle
{
    flash_err_t (*executeCommand)(void *resource, flash_transfer_t *xfer); /*!< Execute command function. */
    void *resource;          /*!< Pointer for resource, resource mean the SPI adapter resource. */
    uint32_t totalSize;      /*!< Flash total size. */
    uint32_t pageSize;       /*!< Flash page size. */
    uint32_t sectorSize;     /*!< Flash sector size. */
    bool needWriteEnable;    /*!< Need do write enable before write/erase operation. */
    uint8_t statusValueSize; /*!< Status value size in bytes. */
    uint32_t statusBusyMask; /*!< Status busy mask. */
    uint8_t statusBusyValue; /*!< Status busy value. */
} flash_handle_t;

#endif

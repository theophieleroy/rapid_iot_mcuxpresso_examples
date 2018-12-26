/*
 * The Clear BSD License
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
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
#if !defined(__FLEXSPINOR_MEMORY_H__)
#define __FLEXSPINOR_MEMORY_H__

#include "memory/memory.h"
#include "flexspi_nor/flexspi_nor_flash.h"

#if BL_FEATURE_FLEXSPI_NOR_MODULE
////////////////////////////////////////////////////////////////////////////////
// Definitions
////////////////////////////////////////////////////////////////////////////////
enum
{
    //! @brief Max page size used to create a page buffer
    kFlexSpiNorMemory_MaxPageSize = 1024U,
    //! @brief Max total size of NOR FLASH supported by FlexSPI
    kFlexSpiNorMemory_MaxSize = 256U * 1024 * 1024,
    //! @brief Max Size of SPI NOR alias memory
    kFlexSpiNorAliasAreaMaxSize = 0x04000000U, // 64MB

    kFlexSpiNorProgramConfigBlock_Tag = 0x0F,
};

#define MAGIC_NUMBER_FLEXSPI_NOR_PROGRAM_CFG \
    (((uint32_t)kFlexSpiNorProgramConfigBlock_Tag << 28) | kFlexSpiNorProgramConfigBlock_Tag)

typedef enum _flexspi_nor_property
{
    kFlexspiNorProperty_InitStatus = 0,
    kFlexspiNorProperty_StartAddress = 1,           //!< Tag used to retrieve start address
    kFlexspiNorProperty_TotalFlashSizeInKBytes = 2, //!< Tag used to retrieve total flash size in terms of KByte
    kFlexspiNorProperty_PageSize = 3,               //!< Tag used to retrieve page size in terms of byte
    kFlexspiNorProperty_SectorSize = 4,             //!< Tag used to retrieve sector size in term of byte
    kFlexspiNorProperty_BlockSize = 5,              //!< Tag used to retrieve block size in terms of byte

    kFlexspiNorProperty_TotalFlashSize = 0x10, //!< Tag used to retrieve total flash size in terms of byte
} flexspi_nor_property_t;

////////////////////////////////////////////////////////////////////////////////
// Prototypes
////////////////////////////////////////////////////////////////////////////////
#if defined(__cplusplus)
extern "C" {
#endif // __cplusplus

//! @name FlexSPI NOR memory
//@{

//! @brief Initialize SPI NOR memory
status_t flexspi_nor_mem_init(void);

//! @brief Read SPI NOR memory.
status_t flexspi_nor_mem_read(uint32_t address, uint32_t length, uint8_t *buffer);

//! @brief Write SPI NOR memory.
status_t flexspi_nor_mem_write(uint32_t address, uint32_t length, const uint8_t *buffer);

//! @brief Fill SPI NOR memory with a word pattern.
status_t flexspi_nor_mem_fill(uint32_t address, uint32_t length, uint32_t pattern);

//! @brief Erase SPI NOR memory
status_t flexspi_nor_mem_erase(uint32_t address, uint32_t length);

//! @brief flush cached data to SPI NOR memory
status_t flexspi_nor_mem_flush(void);

#if BL_FEATURE_FLEXSPI_ALIAS_AREA
//! @brief Initialize SPI NOR alias
status_t flexspi_nor_alias_init(void);

//! @brief Write SPI NOR alias.
status_t flexspi_nor_alias_write(uint32_t address, uint32_t length, const uint8_t *buffer);

//! @brief Fill SPI NOR alias with a word pattern.
status_t flexspi_nor_alias_fill(uint32_t address, uint32_t length, uint32_t pattern);

//! @brief  Erase SPI NOR alias
status_t flexspi_nor_alias_erase(uint32_t address, uint32_t length);
#endif // #if BL_FEATURE_FLEXSPI_ALIAS_AREA

//@}

#if defined(__cplusplus)
}
#endif // __cplusplus

#endif // BL_FEATURE_FLEXSPI_NOR_MODULE
#endif // __FLEXSPINOR_MEMORY_INTERFACE_H__
////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////

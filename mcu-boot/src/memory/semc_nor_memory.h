/*
 * The Clear BSD License
 * Copyright 2016-2017 NXP
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

#ifndef __SEMC_NOR_MEMORY_H__
#define __SEMC_NOR_MEMORY_H__

#include <stdint.h>
#include <stdbool.h>
#include "fsl_common.h"
#include "semc/fsl_semc.h"
#include "semc_nor/semc_nor_flash.h"

////////////////////////////////////////////////////////////////////////////////
// Declarations
////////////////////////////////////////////////////////////////////////////////

#define SEMC_NOR_MAX_SIZE    (16U*1024*1024)

typedef enum _semc_nor_property
{
    kSemcNorProperty_InitStatus = 0,
    kSemcNorProperty_StartAddress = 1,           //!< Tag used to retrieve start address
    kSemcNorProperty_TotalFlashSizeInKBytes = 2, //!< Tag used to retrieve total flash size in terms of KByte
    kSemcNorProperty_PageSize = 3,               //!< Tag used to retrieve page size in terms of byte
    kSemcNorProperty_BlockSize = 5,              //!< Tag used to retrieve block size in terms of byte

    kSemcNorProperty_TotalFlashSize = 0x10, //!< Tag used to retrieve total flash size in terms of byte
} semc_nor_property_t;

////////////////////////////////////////////////////////////////////////////////
// API
////////////////////////////////////////////////////////////////////////////////

#if defined(__cplusplus)
extern "C" {
#endif

//! @name SEMC NOR memory
//@{

//! @brief Initialize SEMC NOR memory
status_t semc_nor_mem_init(void);

//! @brief Read SEMC NOR memory.
status_t semc_nor_mem_read(uint32_t address, uint32_t length, uint8_t *buffer);

//! @brief Write SEMC NOR memory.
status_t semc_nor_mem_write(uint32_t address, uint32_t length, const uint8_t *buffer);

//! @brief Fill SEMC NOR memory with a word pattern.
status_t semc_nor_mem_fill(uint32_t address, uint32_t length, uint32_t pattern);

//! @brief Erase SEMC NOR memory
status_t semc_nor_mem_erase(uint32_t address, uint32_t length);

status_t semc_nor_get_default_config_block(semc_nor_config_t *config);

//@}

#if defined(__cplusplus)
}
#endif
#endif /* #ifndef __SEMC_NOR_MEMORY_H__ */

////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////

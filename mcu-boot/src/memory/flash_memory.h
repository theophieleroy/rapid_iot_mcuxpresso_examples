/*
 * The Clear BSD License
 * Copyright (c) 2013-2014, Freescale Semiconductor, Inc.
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
#if !defined(__FLASH_MEMORY_INTERFACE_H__)
#define __FLASH_MEMORY_INTERFACE_H__

#include "memory/memory.h"
#if !BL_FEATURE_HAS_NO_INTERNAL_FLASH
#include "fsl_flash.h"
#endif

////////////////////////////////////////////////////////////////////////////////
// Definitions
////////////////////////////////////////////////////////////////////////////////

#define BL_IS_FLASH_SECTION_PROGRAMMING_ENABLED \
    (BL_FEATURE_ENABLE_FLASH_PROGRAM_SECTION && FSL_FEATURE_FLASH_HAS_PROGRAM_SECTION_CMD)

////////////////////////////////////////////////////////////////////////////////
// Externs
////////////////////////////////////////////////////////////////////////////////

//! @brief flash memory array.
#if !BL_FEATURE_HAS_NO_INTERNAL_FLASH
#if !BL_DEVICE_IS_LPC_SERIES
extern flash_config_t g_flashState;
extern ftfx_cache_config_t g_flashcacheState;
#endif // !BL_DEVICE_IS_LPC_SERIES
#endif // #if !BL_FEATURE_HAS_NO_INTERNAL_FLASH


////////////////////////////////////////////////////////////////////////////////
// Prototypes
////////////////////////////////////////////////////////////////////////////////

#if defined(__cplusplus)
extern "C" {
#endif // __cplusplus

//! @name Flash memory
//! @note Flash read is done through the normal memory interface.
//@{

//! @brief Init selected Flash memory
status_t flash_mem_init(void);

//! @brief Read selected Flash memory.
status_t flash_mem_read(uint32_t address, uint32_t length, uint8_t *restrict buffer);

//! @brief Write selected Flash memory.
status_t flash_mem_write(uint32_t address, uint32_t length, const uint8_t *buffer);

//! @brief Fill selected Flash memory with a word pattern.
status_t flash_mem_fill(uint32_t address, uint32_t length, uint32_t pattern);

//! @brief Flush final buffer or cached data into selected FLASH memory.
status_t flash_mem_flush(void);

//! @brief Erase selected Flash memory.
status_t flash_mem_erase(uint32_t address, uint32_t length);

//@}

#if defined(__cplusplus)
}
#endif // __cplusplus

#endif // __FLASH_MEMORY_INTERFACE_H__
////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////

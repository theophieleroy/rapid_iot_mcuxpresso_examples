/*
 * The Clear BSD License
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
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

#include "bootloader_common.h"
#include "memory/memory.h"
#include "flashiap_memory.h"
#include "normal_memory.h"
#include "flashiap_wrapper/fsl_flashiap_wrapper.h"
#include "bootloader/bl_context.h"
#include "fsl_device_registers.h"
#include "utilities/fsl_rtos_abstraction.h"
#include "utilities/fsl_assert.h"
#include <string.h>

////////////////////////////////////////////////////////////////////////////////
// Definitions
////////////////////////////////////////////////////////////////////////////////

// If the bootloader is running from flash, then we need to make sure that all
// interrupts are disabled during the execution of a flash operation, so that
// no code is unexpectedly run from flash (which would cause a hard fault).
//
// If we're running from ROM or RAM, then we neither need to nor want to disable
// interrupts during flash operations.
#if !BL_TARGET_FLASH
#define flashiap_lock_release() (void)sizeof(0)
#define flashiap_lock_acquire() (void)sizeof(0)
#endif // BL_TARGET_FLASH

//! @brief FLASHIAP page program memory context
//!
//! An instance is maintained in this file, will is used to keep key information for write and flush
//! operatations.
typedef struct _flash_page_program_info
{
    uint32_t startAddress;                      //!< This address is used to record the address which is used
                                                //!< to write the whole page into flash memory
    uint32_t storedBytes;                       //!< A variable which is used to indicate if the buffer is full.
    uint8_t buffer[kFLASHIAP_PageSize];         //!< A buffer which is used to buffer a full page of data
} flash_page_program_info_t;

flash_page_program_info_t s_flash_page_program_info[kFLASHCount] = { 0 };

////////////////////////////////////////////////////////////////////////////////
// Variables
////////////////////////////////////////////////////////////////////////////////

//! @brief flashiap memory array.
flashiap_config_t g_flashiapState[] = {
    // Main Flash
    {.FlashMemoryIndex = kFlashIndex_Main},
    {0}
};

//! @brief Current flash memory index.
static volatile uint32_t s_flashiapMemoryIndex = kFlashIndex_Main;

//! @brief Interface to LPC main flash memory operations.
const memory_region_interface_t g_flashiapMemoryInterface = {
    .init = &flashiap_mem_init,
    .read = &flashiap_mem_read,
    .write = &flashiap_mem_write,
#if !BL_FEATURE_MIN_PROFILE || BL_FEATURE_FILL_MEMORY
    .fill = &flashiap_mem_fill,
#endif // !BL_FEATURE_MIN_PROFILE
    .flush = &flashiap_mem_flush,
    .erase = &flashiap_mem_erase,
};

#if BL_TARGET_FLASH
static uint32_t s_regPrimask = 0U;
#endif

////////////////////////////////////////////////////////////////////////////////
// Prototypes
////////////////////////////////////////////////////////////////////////////////

status_t flashiap_check_access_before_programming(uint32_t address, uint32_t length);

////////////////////////////////////////////////////////////////////////////////
// Code
////////////////////////////////////////////////////////////////////////////////

#if BL_TARGET_FLASH
static void flashiap_lock_acquire(void)
{
    // Ensure that the program operation cannots be interrupted.
    s_regPrimask = __get_PRIMASK();
    __disable_irq();
}

static void flashiap_lock_release(void)
{
    // Release lock after the write operation completes.
    __set_PRIMASK(s_regPrimask);
}
#endif

// See flash_memory.h for documentation on this function.
status_t flashiap_mem_init(void)
{
    // Update address range of flash
    memory_map_entry_t *map;
    map = (memory_map_entry_t *)&g_bootloaderContext.memoryMap[kIndexFlashArray];

    g_bootloaderContext.flashDriverInterface->flash_get_property(
        &g_bootloaderContext.allFlashState[s_flashiapMemoryIndex], kFLASHIAP_PropertyPflashBlockBaseAddr, &map->startAddress);
    uint32_t tmp;
    g_bootloaderContext.flashDriverInterface->flash_get_property(&g_bootloaderContext.allFlashState[s_flashiapMemoryIndex],
                                                                 kFLASHIAP_PropertyPflashTotalSize, &tmp);
    map->endAddress = map->startAddress + tmp - 1;

    return kStatus_Success;
}

// See flash_memory.h for documentation on this function.
status_t flashiap_mem_read(uint32_t address, uint32_t length, uint8_t *restrict buffer)
{
    return normal_mem_read(address, length, buffer);
}

// See flashiap_memory.h for documentation on this function.
status_t flashiap_mem_write(uint32_t address, uint32_t length, const uint8_t *buffer)
{
    // Note: the check for "length != 0" and "range not in reserved region" is done in mem_write().
    assert(length);
    assert(buffer);

    status_t status = kStatus_Success;

    while (length)
    {
        // Set start address when storing first byte into page program buffer
        if ((!s_flash_page_program_info[s_flashiapMemoryIndex].storedBytes) &&
            (!s_flash_page_program_info[s_flashiapMemoryIndex].startAddress))
        {
            // Check address alignment
            if (address & (kFLASHIAP_PageSize - 1))
            {
                return kStatus_FLASHIAP_AlignmentError;
            }
            s_flash_page_program_info[s_flashiapMemoryIndex].startAddress = address;
        }
        else
        {
            // Start page programming operation when meet discontinuous address
            if ((s_flash_page_program_info[s_flashiapMemoryIndex].startAddress + s_flash_page_program_info[s_flashiapMemoryIndex].storedBytes) != address)
            {
                // flush cached data into target memory,
                status = flashiap_mem_flush();
                if (status != kStatus_Success)
                {
                    return status;
                }
                continue;
            }
        }

        uint32_t storeBytes;
        // Check to see if page program buffer will be filled with current data packet
        if ((s_flash_page_program_info[s_flashiapMemoryIndex].storedBytes + length) <= sizeof(s_flash_page_program_info[s_flashiapMemoryIndex].buffer))
        {
            storeBytes = length;
        }
        else
        {
            storeBytes = sizeof(s_flash_page_program_info[s_flashiapMemoryIndex].buffer) - s_flash_page_program_info[s_flashiapMemoryIndex].storedBytes;
        }

        // Copy data to page program buffer
        if (buffer != &s_flash_page_program_info[s_flashiapMemoryIndex].buffer[s_flash_page_program_info[s_flashiapMemoryIndex].storedBytes])
        {
            memcpy(&s_flash_page_program_info[s_flashiapMemoryIndex].buffer[s_flash_page_program_info[s_flashiapMemoryIndex].storedBytes], buffer, storeBytes);
        }

        s_flash_page_program_info[s_flashiapMemoryIndex].storedBytes += storeBytes;
        buffer += storeBytes;
        address += storeBytes;
        length -= storeBytes;

        // Start page programming operation when page program buffer is full
        if (s_flash_page_program_info[s_flashiapMemoryIndex].storedBytes == sizeof(s_flash_page_program_info[s_flashiapMemoryIndex].buffer))
        {
            // flush cached data into target memory,
            status = flashiap_mem_flush();
            if (status != kStatus_Success)
            {
                return status;
            }
        }
    }

    return kStatus_Success;
}

// See flashiap_memory.h for documentation on this function.
status_t flashiap_mem_fill(uint32_t address, uint32_t length, uint32_t pattern)
{
    // Note: the check for "length != 0" and "range not in reserved region" is done in mem_fill().
    assert(length);

    status_t status;

    // Pre-fill page program buffer with pattern
    uint32_t *buffer = (uint32_t *)s_flash_page_program_info[s_flashiapMemoryIndex].buffer;
    uint32_t maxPatterns = sizeof(s_flash_page_program_info[s_flashiapMemoryIndex].buffer) >> 2;
    for (uint32_t i = 0; i < maxPatterns; i++)
    {
        *buffer++ = pattern;
    }

    while (length)
    {
        uint32_t bytes;

        s_flash_page_program_info[s_flashiapMemoryIndex].storedBytes = 0;

        // Check to see if remaining address range can hold whole page program buffer
        if (length < sizeof(s_flash_page_program_info[s_flashiapMemoryIndex].buffer))
        {
            bytes = length;
        }
        else
        {
            bytes = sizeof(s_flash_page_program_info[s_flashiapMemoryIndex].buffer);
        }

        // flush cached data into target memory,
        status = flashiap_mem_write(address, bytes, s_flash_page_program_info[s_flashiapMemoryIndex].buffer);
        if (status != kStatus_Success)
        {
            return status;
        }

        address += bytes;
        length -= bytes;
    }

    // flush cached data into target memory,
    status = flashiap_mem_flush();
    if (status != kStatus_Success)
    {
        return status;
    }

    return kStatus_Success;
}

// See memory.h for documentation on this function.
status_t flashiap_mem_flush(void)
{
    status_t status = kStatus_Success;

    if (s_flash_page_program_info[s_flashiapMemoryIndex].storedBytes)
    {
        uint32_t address = s_flash_page_program_info[s_flashiapMemoryIndex].startAddress;
        uint32_t length = s_flash_page_program_info[s_flashiapMemoryIndex].storedBytes;

        // Clear related states no matter following operations are executed successfully or not.
        s_flash_page_program_info[s_flashiapMemoryIndex].startAddress = 0;
        s_flash_page_program_info[s_flashiapMemoryIndex].storedBytes = 0;

        // Align length to page program unit
        uint32_t alignedLength = ALIGN_UP(length, (uint32_t)kFLASHIAP_PageSize);

        status = flashiap_check_access_before_programming(address, alignedLength);
        if (status != kStatus_Success)
        {
            return status;
        }

        // Fill unused region with oxFFs.
        assert(length <= sizeof(s_flash_page_program_info[s_flashiapMemoryIndex].buffer));
        if (length < alignedLength)
        {
            memset(&s_flash_page_program_info[s_flashiapMemoryIndex].buffer[length], 0xFF, alignedLength - length);
        }

        flashiap_lock_acquire();
        // Write data of aligned length to flash
        status = FLASHIAP_Program(&g_bootloaderContext.allFlashState[s_flashiapMemoryIndex], address,
                                  (uint32_t *)s_flash_page_program_info[s_flashiapMemoryIndex].buffer, alignedLength);
        flashiap_lock_release();

        if (status != kStatus_Success)
        {
            return status;
        }

// Verify wether the data has been programmed to flash successfully.
#if !BL_FEATURE_FLASH_VERIFY_DISABLE
        // Verify flash program
        if (g_bootloaderContext.propertyInterface->store->verifyWrites)
        {
            uint32_t failedAddress;
            uint32_t failedData;

            flashiap_lock_acquire();
            status = g_bootloaderContext.flashDriverInterface->flash_verify_program(
                &g_bootloaderContext.allFlashState[s_flashiapMemoryIndex], address, alignedLength,
                (uint32_t *)&s_flash_page_program_info[s_flashiapMemoryIndex].buffer,
                &failedAddress, &failedData);
            flashiap_lock_release();
            if (status != kStatus_Success)
            {
                debug_printf("Error: flash verify failed at address: 0x%x\r\n", failedAddress);
                return status;
            }
        }
#endif // !BL_FEATURE_FLASH_VERIFY_DISABLE

    }

    return status;
}

// See memory.h for documentation on this function.
status_t flashiap_mem_erase(uint32_t address, uint32_t length)
{
    status_t status;

    flashiap_lock_acquire();
    status = g_bootloaderContext.flashDriverInterface->flash_erase(&g_bootloaderContext.allFlashState[s_flashiapMemoryIndex], address, length,
                                                                   kFLASHIAP_ApiEraseKey);
    flashiap_lock_release();

#if !BL_FEATURE_FLASH_VERIFY_DISABLE
    if ((status == kStatus_Success) && (g_bootloaderContext.propertyInterface->store->verifyWrites))
    {
        flashiap_lock_acquire();
        status = g_bootloaderContext.flashDriverInterface->flash_verify_erase(
            &g_bootloaderContext.allFlashState[s_flashiapMemoryIndex], address, length);
        flashiap_lock_release();
        if (status != kStatus_Success)
        {
            debug_printf("Error: flash_verify_erase failed\r\n");
            return status;
        }
    }
#endif // !BL_FEATURE_FLASH_VERIFY_DISABLE

    return status;
}

status_t flash_mem_erase_all(void)
{
    status_t status = kStatus_Success;

    memory_map_entry_t *map = (memory_map_entry_t *)&g_bootloaderContext.memoryMap[kIndexFlashArray];

// Decompose the the flash erase all into two region erases.
#if BL_TARGET_FLASH
    reserved_region_t *reservedRegion =
        &g_bootloaderContext.propertyInterface->store->reservedRegions[kProperty_FlashReservedRegionIndex];
    const uint32_t eraseSize = g_bootloaderContext.propertyInterface->store->flashSectorSize[kFlashIndex_Main];

    bool isReservedRegionEmpty =
        (reservedRegion->startAddress == map->startAddress) && (reservedRegion->endAddress == map->startAddress);
    if (!isReservedRegionEmpty)
    {
        // Erase the initial unreserved region, if any.
        if (reservedRegion->startAddress > map->startAddress)
        {
            uint32_t length = ALIGN_DOWN(reservedRegion->startAddress, eraseSize);
            if (length > 0)
            {
                status = flashiap_mem_erase(map->startAddress, length);
            }
        }

        // Erase the final unreserved region, if any.
        if (status == kStatus_Success)
        {
            uint32_t start = ALIGN_UP(reservedRegion->endAddress, eraseSize);
            if (start < map->endAddress)
            {
                status = flashiap_mem_erase(start, (map->endAddress + 1) - start);
            }
        }

        return status;
    }

#endif // BL_TARGET_FLASH

    // Do full erase and verify.
    flashiap_mem_erase(map->startAddress, (map->endAddress + 1) - map->startAddress);

    return status;
}

status_t flashiap_check_access_before_programming(uint32_t address, uint32_t length)
{
    status_t status = kStatus_Success;

// Do cumulative write check
#if BL_FEATURE_FLASH_CHECK_CUMULATIVE_WRITE
    bool isCumulativeCheckNeeded = true;
    if (isCumulativeCheckNeeded)
    {
        uint32_t actualLength = ALIGN_UP(length, kFLASHIAP_PageSize);
        if (!mem_is_erased(address, actualLength))
        {
            return kStatusMemoryCumulativeWrite;
        }
    }
#endif // BL_FEATURE_FLASH_CHECK_CUMULATIVE_WRITE

    return status;
}

////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////

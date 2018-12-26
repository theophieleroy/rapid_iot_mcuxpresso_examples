/*
 * The Clear BSD License
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
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
#include "bootloader/bootloader.h"
#include "memory/memory.h"
#include "spifi_nor_memory.h"
#include "spifi_nor/spifi_nor_flash.h"

//! @brief SPIFI NOR memory feature inforamation
//!
//! An instance is maintained in this file, will is used to keep key information for write and flush
//! operatations.
typedef struct
{
    bool isConfigured;                              //!< The state which indicates whether spi block is successfully
    //! configured.
    bool isAddingToBuffer;                         //!< State used of determine whether it is the first write at
                                                   //!< Start address of one page
    uint32_t writeAddress;                         //!< This address is used to record the address which is used
                                                   //!< to write the whole page into spi memory
    uint32_t offset;                               //!< A variable which is used to indicate if the buffer is
                                                   //!< full.
    uint32_t nextStartAddress;                     //!< A variable is used to indicate if recent two writes
                                                   //!< are continuous
    uint8_t buffer[kSpifiNorMemory_MaxPageSize];    //!< A buffer which is used to buffer a full
                                                   //!< page of data
    uint8_t readback_buf[kSpifiNorMemory_MaxPageSize];    //!< A buffer which is used to read back programed page data

} spifi_nor_mem_context_t;

static bool spifi_nor_mem_is_erased(uint32_t address, uint32_t length);

//! @brief Interface to spifi memory operations
const memory_region_interface_t g_spifiMemoryInterface = {
    .init = spifi_nor_mem_init,
    .read = spifi_nor_mem_read,
    .write = spifi_nor_mem_write,
    .fill = spifi_nor_mem_fill,
    .erase = spifi_nor_mem_erase,
    .flush = spifi_nor_mem_flush
};

//! @brief Configuration block for spifi
static spifi_nor_config_t s_spifiNorConfigBlock; 

//! @brief Context of spifi operation.
static spifi_nor_mem_context_t s_spifiNorContext;

//! @brief Get the status of spifi configuration
#define is_spifi_nor_configured()   (s_spifiNorContext.isConfigured)

static bool spifi_nor_mem_is_erased(uint32_t address, uint32_t length)
{
    status_t status;
    uint32_t read_length;

    while (length)
    {
        // Make read length don't exceed the read buffer
        if (length < sizeof(s_spifiNorContext.readback_buf))
        {
            read_length = length;
            length = 0;
        }
        else
        {
            read_length = sizeof(s_spifiNorContext.readback_buf);
            length -= sizeof(s_spifiNorContext.readback_buf);
        }

        // Read data from spifi norflash
        status = spifi_nor_mem_read(address, read_length, s_spifiNorContext.readback_buf);
        if (status != kStatus_Success)
        {
            return false;
        }

        // Check if all are erased data
        if(!mem_is_erased((uint32_t)s_spifiNorContext.readback_buf, read_length))
        {
            return false;
        }

        if (length)
        {
            // Update to next address
            address += read_length;
        }
    }

    return true;
}

status_t spifi_nor_mem_init(void)
{
    memset(&s_spifiNorContext, 0x00, sizeof(s_spifiNorContext));

    /*
     * NOTE: The SPI NOR instruction set may be switched during Boot,
     *       So bootloader doesn't init it here.
     */
    return kStatus_Fail;
}

//! @brief Get Property from spifi driver
status_t spifi_nor_get_property(uint32_t whichProperty, uint32_t *value)
{
    if (value == NULL)
    {
        return kStatus_InvalidArgument;
    }

    switch (whichProperty)
    {
        case kSpifiNorProperty_InitStatus:
            *value = is_spifi_nor_configured() ? kStatus_Success : kStatusMemoryNotConfigured;
            break;

        case kSpifiNorProperty_StartAddress:
            *value = FSL_FEATURE_SPIFI_START_ADDR;
            break;

        case kSpifiNorProperty_TotalFlashSizeInKBytes:
        {
            uint32_t totalFlashSizeInBytes = s_spifiNorConfigBlock.memorySize;

            *value = totalFlashSizeInBytes / 1024U;
        }
        break;

        case kSpifiNorProperty_PageSize:
            *value = s_spifiNorConfigBlock.pageSize;
            break;

        case kSpifiNorProperty_SectorSize:
            *value = s_spifiNorConfigBlock.sectorSize;
            break;

        case kSpifiNorProperty_TotalFlashSize:
        {
            uint32_t totalFlashSize = s_spifiNorConfigBlock.memorySize;

            *value = totalFlashSize;
        }
        break;

        default: // catch inputs that are not recognized
            return kStatus_InvalidArgument;
    }

    return kStatus_Success;
}

// See memory.h for documentation on this function.
status_t spifi_nor_mem_config(uint32_t *config)
{
    status_t status = kStatus_InvalidArgument;

    bool isNorConfigOption = false;

    do
    {
        uint32_t startAddr = (uint32_t)config;
        uint32_t endAddr = startAddr + sizeof(spifi_nor_config_t) - 1;

        // Should check the config is in valid internal space.
        if ((!is_valid_application_location(startAddr)) || (!is_valid_application_location(endAddr)))
        {
            break;
        }

        // Try to check whether the 'config' variable is an option supported by spifi_nor_get_config function
        spifi_nor_config_option_t *option = (spifi_nor_config_option_t *)startAddr;
        // Try to check whether the config variable stored at the specified addres is a whole config block
        spifi_nor_config_t *norConfig = (spifi_nor_config_t *)config;

        if (option->option0.B.tag == kSpifiNorCfgOption_Tag)
        {
            status = spifi_nor_get_config(BL_CONFIG_SPIFI, &s_spifiNorConfigBlock, option);
            if (status != kStatus_Success)
            {
                break;
            }
            isNorConfigOption = true;
        }
        else if (norConfig->tag == SPIFI_CFG_BLK_TAG)
        {
            // Over-write config block.
            memcpy(&s_spifiNorConfigBlock, norConfig, sizeof(spifi_nor_config_t));
            isNorConfigOption = true;
        }
        else
        {
            break;
        }
        if (true == isNorConfigOption)
        {
            s_spifiNorContext.isConfigured = false;

            status = spifi_norflash_init(BL_CONFIG_SPIFI, &s_spifiNorConfigBlock);
            if (status != kStatus_Success)
            {
                break;
            }
            
            s_spifiNorContext.isConfigured = true;
        }

    } while (0);

    return status;
}

status_t spifi_nor_mem_read(uint32_t address, uint32_t length, uint8_t *buffer)
{
    status_t status;

    assert(length);
    assert(buffer);

    if (!is_spifi_nor_configured())
    {
        return kStatusMemoryNotConfigured;
    }

    status = spifi_norflash_read(BL_CONFIG_SPIFI, &s_spifiNorConfigBlock, address, buffer, length);
    if (status != kStatus_Success)
    {
        return kStatusMemoryReadFailed;
    }

    return status;
}

status_t spifi_nor_mem_write(uint32_t address, uint32_t length, const uint8_t *buffer)
{
    status_t status;
    uint32_t writeLength;

    assert(length);
    assert(buffer);

    if (!is_spifi_nor_configured())
    {
        return kStatusMemoryNotConfigured;
    }

    while (length)
    {
        // If buffer is invalid, it means it is a new write operation.
        if (!s_spifiNorContext.isAddingToBuffer)
        {
            // If address is page aligned, it means it is a valid start address, else return an error status
            if (address & (s_spifiNorConfigBlock.pageSize - 1))
            {
                return kStatus_SPIFINOR_WriteAlignmentError;
            }

            // Start buffering data
            s_spifiNorContext.isAddingToBuffer = true;
            s_spifiNorContext.offset = 0;
            s_spifiNorContext.writeAddress = address;
        }
        else
        {
            // In this case, it means recent two writes are not continuous, should flush last cached data into memory,
            // then switch to processsing of this write operation.
            if ((s_spifiNorContext.offset + s_spifiNorContext.writeAddress) != address)
            {
                // flush cached data into target memory,
                status = spifi_nor_mem_flush();
                if (status != kStatus_Success)
                {
                    return status;
                }
                // Start processing this write
                continue;
            }
            // Otherwise, it means recent two writes are continuous, continue to buffer data until whole page gets
            // buffered.
        }

        if (s_spifiNorContext.offset + length < s_spifiNorConfigBlock.pageSize)
        {
            writeLength = length;
        }
        else
        {
            writeLength = s_spifiNorConfigBlock.pageSize - s_spifiNorContext.offset;
        }
        // Copy data to internal buffer
        memcpy(&s_spifiNorContext.buffer[s_spifiNorContext.offset], buffer, writeLength);

        s_spifiNorContext.offset += writeLength;
        address += writeLength;
        buffer += writeLength;
        length -= writeLength;

        assert(s_spifiNorContext.offset <= s_spifiNorConfigBlock.pageSize);
        // If the buffer is full, it is time to flush cached data to target memory.
        if (s_spifiNorContext.offset == s_spifiNorConfigBlock.pageSize)
        {
            status = spifi_nor_mem_flush();
            if (status != kStatus_Success)
            {
                return status;
            }
        }
    }

    return kStatus_Success;
}

status_t spifi_nor_mem_fill(uint32_t address, uint32_t length, uint32_t pattern)
{
    uint32_t alignedLength = ALIGN_DOWN(length, 4U);
    uint32_t leftLengthInBytes = length & 3U;
    status_t status;

    if (!is_spifi_nor_configured())
    {
        return kStatusMemoryNotConfigured;
    }

    while (alignedLength)
    {
        status = spifi_nor_mem_write(address, 4U, (const uint8_t *)&pattern);
        if (status != kStatus_Success)
        {
            return status;
        }

        address += 4U;
        alignedLength -= 4U;
    }

    if (leftLengthInBytes)
    {
        status = spifi_nor_mem_write(address, leftLengthInBytes, (const uint8_t *)&pattern);
        if (status != kStatus_Success)
        {
            return status;
        }
    }

    return spifi_nor_mem_flush();
}

status_t spifi_nor_mem_flush(void)
{
    status_t status = kStatus_Success;

    if (s_spifiNorContext.isAddingToBuffer)
    {
        s_spifiNorContext.isAddingToBuffer = false;
        // Fill unused region with oxFFs.
        assert(s_spifiNorContext.offset <= s_spifiNorConfigBlock.pageSize);

        if (s_spifiNorContext.offset != s_spifiNorConfigBlock.pageSize)
        {
            memset(&s_spifiNorContext.buffer[s_spifiNorContext.offset], 0xFFU,
                   s_spifiNorConfigBlock.pageSize - s_spifiNorContext.offset);
        }

#if BL_FEATURE_FLASH_CHECK_CUMULATIVE_WRITE
        if (!spifi_nor_mem_is_erased(s_spifiNorContext.writeAddress, s_spifiNorConfigBlock.pageSize))
        {
            return kStatusMemoryCumulativeWrite;
        }
#endif // BL_FEATURE_FLASH_CHECK_CUMULATIVE_WRITE

        status = spifi_norflash_program_page(BL_CONFIG_SPIFI, &s_spifiNorConfigBlock, s_spifiNorContext.writeAddress, s_spifiNorContext.buffer);
        // Clear related states.
        s_spifiNorContext.isAddingToBuffer = false;
        s_spifiNorContext.offset = 0;
        if (status != kStatus_Success)
        {
            return status;
        }

        status = spifi_nor_mem_read(s_spifiNorContext.writeAddress, sizeof(s_spifiNorContext.readback_buf),
                                    s_spifiNorContext.readback_buf);
        if (status != kStatus_Success)
        {
            return kStatusMemoryReadFailed;
        }

        // Verify wether the data has been programmed to spi flash successfully.
        if (memcmp(s_spifiNorContext.readback_buf, s_spifiNorContext.buffer, s_spifiNorConfigBlock.pageSize))
        {
            return kStatus_SPIFINOR_CommandFailure;
        }
    }

    return status;
}

status_t spifi_nor_mem_erase(uint32_t address, uint32_t length)
{
    assert(length);

    uint32_t sectorSize = s_spifiNorConfigBlock.sectorSize;
    uint32_t alignedAddress = ALIGN_DOWN(address, sectorSize);
    uint32_t alignedLength = ALIGN_UP(address + length, sectorSize) - alignedAddress;
    status_t status;

    if (!is_spifi_nor_configured())
    {
        return kStatusMemoryNotConfigured;
    }

    while (alignedLength)
    {
        status = spifi_norflash_erase(BL_CONFIG_SPIFI, &s_spifiNorConfigBlock, alignedAddress, kSpifiNorEraseType_Sector);
        if (status != kStatus_Success)
        {
            return status;
        }
#if SPIFI_NOR_ERASE_VERIFY
        if (!spifi_nor_mem_is_erased(alignedAddress, sectorSize))
        {
            return kStatus_SPIFINOR_CommandFailure;
        }
#endif // #if SPIFI_NOR_ERASE_VERIFY
        alignedLength -= sectorSize;
        alignedAddress += sectorSize;
    }

    return kStatus_Success;
}

status_t spifi_nor_mem_erase_all(void)
{
    status_t status;

    if (!is_spifi_nor_configured())
    {
        return kStatusMemoryNotConfigured;
    }

    if (0 == s_spifiNorConfigBlock.memorySize)
    {
        return kStatus_InvalidArgument;
    }
    status = spifi_norflash_erase(BL_CONFIG_SPIFI, &s_spifiNorConfigBlock, FSL_FEATURE_SPIFI_START_ADDR, kSpifiNorEraseType_Bulk);

#if SPIFI_NOR_ERASE_VERIFY
        if (!spifi_nor_mem_is_erased(FSL_FEATURE_SPIFI_START_ADDR, s_spifiNorConfigBlock.memorySize))
        {
            return kStatus_SPIFINOR_CommandFailure;
        }
#endif // #if SPIFI_NOR_ERASE_VERIFY
    return status;
}


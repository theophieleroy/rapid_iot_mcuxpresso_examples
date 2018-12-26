/*
 * The Clear BSD License
 * Copyright (c) 2013 - 2014, Freescale Semiconductor, Inc.
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
#ifdef BOOTLOADER
#include "bootloader_common.h"
#endif
#include "crc/crc16.h"
#include "utilities/fsl_assert.h"
#include "fsl_rtos_abstraction.h"

#include "fsl_device_registers.h"

#if FSL_FEATURE_SOC_CRC_COUNT && !defined(BL_TARGET_RAM)
#if !BL_DEVICE_IS_LPC_SERIES
#include "fsl_crc.h"
#else // BL_DEVICE_IS_LPC_SERIES
#include "lpc_crc/fsl_crc.h"
#endif // !BL_DEVICE_IS_LPC_SERIES

/* Table of base addresses for crc instances. */
static CRC_Type *const g_crcBase[1] = CRC_BASE_PTRS;

void crc16_init(crc16_data_t *crc16Config)
{
    assert(crc16Config);

    crc16Config->currentCrc = 0x0000U;
}

void crc16_update(crc16_data_t *crc16Config, const uint8_t *src, uint32_t lengthInBytes)
{
    assert(crc16Config);
    assert(src);

    crc_config_t crcUserConfigPtr;

    CRC_GetDefaultConfig(&crcUserConfigPtr);

#if !BL_DEVICE_IS_LPC_SERIES
    crcUserConfigPtr.crcBits = kCrcBits16;
    crcUserConfigPtr.seed = crc16Config->currentCrc;
    crcUserConfigPtr.polynomial = 0x1021U;
    crcUserConfigPtr.complementChecksum = false;
    crcUserConfigPtr.reflectIn = false;
    crcUserConfigPtr.reflectOut = false;
#else // BL_DEVICE_IS_LPC_SERIES
    crcUserConfigPtr.seed = crc16Config->currentCrc;
    crcUserConfigPtr.polynomial = kCRC_Polynomial_CRC_CCITT;
    crcUserConfigPtr.reverseIn = false;
    crcUserConfigPtr.reverseOut = false;
    crcUserConfigPtr.complementIn = false;
    crcUserConfigPtr.complementOut = false;
#endif // !BL_DEVICE_IS_LPC_SERIES

    // Init CRC module and then run it
    //! Note: We must init CRC module here, As we may seperate one crc calculation into several times
    //! Note: It is better to use lock to ensure the integrity of current updating operation of crc calculation
    //        in case crc module is shared by multiple crc updating requests at the same time
    if (lengthInBytes)
    {
#ifdef BOOTLOADER
        lock_acquire();
#endif
        CRC_Init(g_crcBase[0], &crcUserConfigPtr);
        CRC_WriteData(g_crcBase[0], src, lengthInBytes);
        crcUserConfigPtr.seed = CRC_Get16bitResult(g_crcBase[0]);
#ifdef BOOTLOADER
        lock_release();
#endif
    }

    crc16Config->currentCrc = crcUserConfigPtr.seed;
}

void crc16_finalize(crc16_data_t *crc16Config, uint16_t *hash)
{
    assert(crc16Config);
    assert(hash);

    *hash = crc16Config->currentCrc;

    // De-init CRC module when we complete a full crc calculation
    CRC_Deinit(g_crcBase[0]);
}

void crc16_onfi_init(crc16_data_t *crc16Config)
{
    assert(crc16Config);

    crc16Config->currentCrc = 0x4F4EU;
}

void crc16_onfi_update(crc16_data_t *crc16Config, const uint8_t *src, uint32_t lengthInBytes)
{
    assert(crc16Config);
    assert(src);

    crc_config_t crcUserConfigPtr;

    CRC_GetDefaultConfig(&crcUserConfigPtr);

    crcUserConfigPtr.crcBits = kCrcBits16;
    crcUserConfigPtr.seed = crc16Config->currentCrc;
    crcUserConfigPtr.polynomial = 0x8005U;
    crcUserConfigPtr.complementChecksum = false;
    crcUserConfigPtr.reflectIn = false;
    crcUserConfigPtr.reflectOut = false;

    // Init CRC module and then run it
    //! Note: We must init CRC module here, As we may seperate one crc calculation into several times
    //! Note: It is better to use lock to ensure the integrity of current updating operation of crc calculation
    //        in case crc module is shared by multiple crc updating requests at the same time
    if (lengthInBytes)
    {
        lock_acquire();
        CRC_Init(g_crcBase[0], &crcUserConfigPtr);
        CRC_WriteData(g_crcBase[0], src, lengthInBytes);
        crcUserConfigPtr.seed = CRC_Get16bitResult(g_crcBase[0]);
        lock_release();
    }

    crc16Config->currentCrc = crcUserConfigPtr.seed;
}

#else
////////////////////////////////////////////////////////////////////////////////
// Code
////////////////////////////////////////////////////////////////////////////////
void crc16_init(crc16_data_t *crc16Config)
{
    assert(crc16Config);

    // initialize running crc and byte count
    crc16Config->currentCrc = 0;
}

void crc16_update(crc16_data_t *crc16Config, const uint8_t *src, uint32_t lengthInBytes)
{
    assert(crc16Config);
    assert(src);

    uint32_t crc = crc16Config->currentCrc;

    uint32_t j;
    for (j = 0; j < lengthInBytes; ++j)
    {
        uint32_t i;
        uint32_t byte = src[j];
        crc ^= byte << 8;
        for (i = 0; i < 8; ++i)
        {
            uint32_t temp = crc << 1;
            if (crc & 0x8000)
            {
                temp ^= 0x1021;
            }
            crc = temp;
        }
    }

    crc16Config->currentCrc = crc;
}

void crc16_finalize(crc16_data_t *crc16Config, uint16_t *hash)
{
    assert(crc16Config);
    assert(hash);

    *hash = crc16Config->currentCrc;
}

void crc16_onfi_init(crc16_data_t *crc16Config)
{
    assert(crc16Config);

    // initialize running crc and byte count
    crc16Config->currentCrc = 0x4F4EU;
}

void crc16_onfi_update(crc16_data_t *crc16Config, const uint8_t *src, uint32_t lengthInBytes)
{
    assert(crc16Config);
    assert(src);

    uint32_t crc = crc16Config->currentCrc;

    uint32_t j;
    for (j = 0; j < lengthInBytes; ++j)
    {
        uint32_t i;
        uint32_t byte = src[j];
        crc ^= byte << 8;
        for (i = 0; i < 8; ++i)
        {
            uint32_t temp = crc << 1;
            if (crc & 0x8000)
            {
                temp ^= 0x8005U;
            }
            crc = temp;
        }
    }

    crc16Config->currentCrc = crc;
}
#endif
////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////

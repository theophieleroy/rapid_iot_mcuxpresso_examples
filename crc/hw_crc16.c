/*
 * The Clear BSD License
 * Copyright (c) 2014, Freescale Semiconductor, Inc.
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
#include "crc/crc16.h"
#include "drivers/crc/fsl_crc_driver.h"
#include "utilities/fsl_rtos_abstraction.h"
#include "utilities/fsl_assert.h"

////////////////////////////////////////////////////////////////////////////////
// Code
////////////////////////////////////////////////////////////////////////////////
void crc16_init(crc16_data_t *crc16Config)
{
    assert(crc16Config);

    crc16Config->currentCrc = 0x0000U;
}

void crc16_update(crc16_data_t *crc16Config, const uint8_t *src, uint32_t lengthInBytes)
{
    assert(crc16Config);
    assert(src);

    crc_user_config_t crcUserConfigPtr;

    crcUserConfigPtr.crcWidth = kCrc16Bits;
    crcUserConfigPtr.seed = crc16Config->currentCrc;
    crcUserConfigPtr.polynomial = 0x1021U;
    crcUserConfigPtr.writeTranspose = kCrcNoTranspose;
    crcUserConfigPtr.readTranspose = kCrcNoTranspose;
    crcUserConfigPtr.complementRead = false;

    // Init CRC module and then run it
    //! Note: We must init CRC module here, As we may seperate one crc calculation into several times
    //! Note: It is better to use lock to ensure the integrity of current updating operation of crc calculation
    //        in case crc module is shared by multiple crc updating requests at the same time
    if (lengthInBytes)
    {
        lock_acquire();
        CRC_DRV_Init(CRC_INSTANCE, &crcUserConfigPtr);
        crcUserConfigPtr.seed = CRC_DRV_GetCrcBlock(CRC_INSTANCE, (uint8_t *)src, lengthInBytes);
        lock_release();
    }

    crc16Config->currentCrc = crcUserConfigPtr.seed;
}

void crc16_finalize(crc16_data_t *crc16Config, uint16_t *hash)
{
    assert(crc16Config);
    assert(hash);

    *hash = crc16Config->currentCrc;

    // De-init CRC module when we complete a full crc calculation
    CRC_DRV_Deinit(CRC_INSTANCE);
}

////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////

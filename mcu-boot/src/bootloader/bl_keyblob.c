/*
 * The Clear BSD License
 * Copyright (c) 2012-2016, Freescale Semiconductor, Inc.
 * Copyright (c) 2017 NXP
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
#include <stdint.h>
#include <stdbool.h>
#include "fsl_common.h"
#include "bootloader/bl_keyblob.h"
#include "bootloader/bootloader.h"

////////////////////////////////////////////////////////////////////////////////
// Definitions
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
// Variables
////////////////////////////////////////////////////////////////////////////////
static keyblob_context_t s_keyblob_context;

////////////////////////////////////////////////////////////////////////////////
// Code
////////////////////////////////////////////////////////////////////////////////

int32_t keyblob_update(keyblob_info_t *key_info)
{
    status_t status = kStatus_InvalidArgument;
    do
    {
        if (key_info == NULL)
        {
            break;
        }
        if (key_info->option.B.tag != kKeyBlobInfoOption_Tag)
        {
            break;
        }

        int32_t keyblob_info_type = key_info->option.B.type;
        if ((keyblob_info_type != kKeyBlobInfoType_Program) && (keyblob_info_type != kKeyBlobInfoType_Update))
        {
            break;
        }

        if (keyblob_info_type == kKeyBlobInfoType_Update)
        {
            uint32_t dek_size = key_info->option.B.dek_size;
            switch (dek_size)
            {
                case kDekSize_128bits:
                    dek_size = 128;
                    break;
                case kDekSize_192bits:
                    dek_size = 192;
                    break;
                case kDekSize_256bits:
                    dek_size = 256;
                    break;
                default:
                    break;
            }
            if (dek_size < 128)
            {
                break;
            }
            dek_size /= 8; // Calculate key bytes
            if ((!is_valid_application_location(key_info->dek_addr)) ||
                (!is_valid_application_location(key_info->dek_addr + dek_size)))
            {
                break;
            }

            uint32_t bk_size = BL_FEATURE_KEYBLOB_BK_SIZE;
            s_keyblob_context.bk_size = bk_size;

            memcpy(s_keyblob_context.dek, (uint32_t *)key_info->dek_addr, dek_size);
            s_keyblob_context.dek_size = dek_size;
            // header size = 8, mac size = 16
            s_keyblob_context.keyblob_size = kKeyBlobMaxSize;

            status = kStatus_Success;
        }
    } while (0);

    return status;
}

int32_t keyblob_get(uint8_t **keyblob_start, uint32_t *keyblob_size)
{
    status_t status = kStatus_InvalidArgument;

    do
    {
        if ((keyblob_start == NULL) || (keyblob_size == NULL))
        {
            break;
        }

        memset(&s_keyblob_context.keyblob, 0, sizeof(s_keyblob_context.keyblob));
        status = generate_key_blob(s_keyblob_context.dek, (uint8_t *)&s_keyblob_context.keyblob);
        if (status != kStatus_Success)
        {
            break;
        }

        *keyblob_start = (uint8_t *)&s_keyblob_context.keyblob[0];
        *keyblob_size = s_keyblob_context.keyblob_size;

    } while (0);

    return status;
}

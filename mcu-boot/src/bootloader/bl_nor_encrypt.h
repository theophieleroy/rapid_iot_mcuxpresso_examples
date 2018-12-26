/*
 * The Clear BSD License
 * Copyright 2017 NXP
 *
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification,
 * are permitted (subject to the limitations in the disclaimer below) provided
 *  that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this
 * list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice,
 * this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS LICENSE.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef __BL_NOR_ENCRYPT_H__
#define __BL_NOR_ENCRYPT_H__

#include "fsl_common.h"

////////////////////////////////////////////////////////////////////////////////
// Definitions
////////////////////////////////////////////////////////////////////////////////
typedef struct
{
    union
    {
        struct
        {
            uint32_t reserved : 28;
            uint32_t tag : 4; //!< Tag, must be 0x0e
        } B;
        uint32_t U;
    } option0;
    uint32_t reserved[10];
} nor_encrypt_option_t;

enum
{
    kNorEncyptOption_Tag = 0x0e, //!< Tag
};

////////////////////////////////////////////////////////////////////////////////
// Prototypes
////////////////////////////////////////////////////////////////////////////////

#if defined(__cplusplus)
extern "C" {
#endif // __cplusplus

//! @brief Determine whether there is a valid encrypt region info
bool bl_nor_encrypt_region_info_valid(void *arg);

//! @brief Return the Encrypted Region Info presence status
bool bl_nor_encrypt_has_encrypted_region(void);

//! @brief Initialize Encrypt Region based on specified argument
status_t bl_nor_encrypt_init(void *arg);

//! @brief Refresh Encrypted region info
void bl_nor_encrypt_region_refresh(uint32_t start, uint32_t bytes);

//! @brief Check if a specified region is in encrypted region
bool bl_nor_in_encrypted_region(uint32_t start, uint32_t bytes);

//! @brief Get Configuration block
status_t bl_nor_encrypt_get_config_block(uint32_t index, uint32_t *start, uint32_t *bytes);

//! @brief Encrypted data in specified region
status_t bl_nor_encrypt_data(uint32_t addr, uint32_t size, uint32_t *data_start);

#if defined(__cplusplus)
}
#endif // __cplusplus

#endif // __BL_NOR_ENCRYPT_H__

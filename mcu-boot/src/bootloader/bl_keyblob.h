/*
 * The Clear BSD License
 * Copyright (c) 2012, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 *
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

#ifndef __BL_KEYBLOB_H__
#define __BL_KEYBLOB_H__

#include "fsl_device_registers.h"

#if defined(CAAM)
#include "bootloader/bl_keyblob_caam.h"
#endif

//! @name Error codes
//@{
#if !defined(SUCCESS)
#define SUCCESS (0)
#endif

typedef struct
{
    union
    {
        struct
        {
            uint32_t image_index : 4; //!< Key blob for the image specified by image_index, valid only if type = 1
            uint32_t dek_size : 4;  //!< Decryption Key size, 0-128bits, 1-192 bits, 2-256 bits, valid only if type = 0
            uint32_t reserved : 12; //!< Reserved
            uint32_t size : 4;      //!< Keyblob Info structure size in longwords, valid only if type = 0
            uint32_t type : 4; //!< Type, 0 - Update key blob info, 1 - Generate and program Keyblob to corresponding
            //! offset for image specified by image_index
            uint32_t tag : 4; //!< Tag, fixed to 0x0F
        } B;
        uint32_t U;
    } option;
    uint32_t dek_addr;       //!< Valid if type = 0
    uint32_t keyblob_offset; //!< Valid if type = 0
} keyblob_info_t;

enum
{
    kKeyBlobInfoOption_Tag = 0x0B, //!< Key blob info tag
    kKeyBlobInfoType_Update = 0,   //!< key blob info type: Update
    kKeyBlobInfoType_Program = 1,  //!< key blob info type: Program
    kKeyBlobInfoSize = 3,          //!< KeyBlob Size in long word when type = kKeyBlobInfoType_Update

    kDekSize_128bits = 0,
    kDekSize_192bits = 1,
    kDekSize_256bits = 2,

    kBlobKeySize_128bits = 0,
    kBlobKeySize_192bits = 1,
    kBlobKeySize_256bits = 2,

    kKeyBlobHeaderSize = 8,
    kKeyBlobMacSize = 16,
    kKeyBlobBkMaxSize = 32,
    kKeyBlobDekMaxSize = 32,

    kKeyBlobMaxSize = 512,
};

typedef struct _keyblob_context
{
    uint32_t dek_size;
    uint32_t bk_size;
    uint32_t keyblob_size;
    uint32_t dek[kKeyBlobDekMaxSize / sizeof(uint32_t)];
    uint32_t keyblob[kKeyBlobMaxSize / sizeof(uint32_t)];
} keyblob_context_t;

////////////////////////////////////////////////////////////////////////////////
//! @brief Wrapper function for generating blob.
//!
//! @param[in] key_addr  Location address of 128 bit dek key.
//!
//! @return SUCCESS
//! @return ERROR_XXX
////////////////////////////////////////////////////////////////////////////////
int32_t generate_key_blob(uint32_t *key_addr, uint8_t *key_blob_addr);

int32_t keyblob_update(keyblob_info_t *key_info);

int32_t keyblob_get(uint8_t **keyblob_start, uint32_t *keyblob_size);

#endif /* __BL_KEYBLOB_H__ */

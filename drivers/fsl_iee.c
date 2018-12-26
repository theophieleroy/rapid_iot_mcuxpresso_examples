/*
 * The Clear BSD License
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright (c) 2017, NXP
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

#include "fsl_iee.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/* Component ID definition, used by tools. */
#ifndef FSL_COMPONENT_ID
#define FSL_COMPONENT_ID "platform.drivers.iee"
#endif


/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/
void IEE_Init(IEE_Type *base)
{
    base->GCFG = IEE_GCFG_RST_MASK;
}

void IEE_GetDefaultConfig(iee_config_t *config)
{
    config->bypass = kIEE_AesUseMdField;
    config->mode = kIEE_ModeNone;
    config->keySize = kIEE_AesCTR128XTS256;
    config->pageOffset = 0U;
}

void IEE_SetRegionConfig(IEE_Type *base, iee_region_t region, iee_config_t *config)
{
    IEE->REGX[region].REGATTR =
        IEE_REGATTR_BYP(config->bypass) | IEE_REGATTR_MD(config->mode) | IEE_REGATTR_KS(config->keySize);
    IEE->REGX[region].REGPO = IEE_REGPO_PGOFF(config->pageOffset);
}

status_t IEE_SetRegionKey(
    IEE_Type *base, iee_region_t region, iee_aes_key_num_t keyNum, const uint8_t *key, size_t keySize)
{
    register const uint32_t *from32 = (const uint32_t *)(uintptr_t)key;
    register volatile uint32_t *to32 = NULL;

    if ((uintptr_t)key & 0x3u)
    {
        return kStatus_InvalidArgument;
    }

    if (keyNum == kIEE_AesKey1)
    {
        to32 = &base->REGX[region].REGKEY1[0];
    }

    else if (keyNum == kIEE_AesKey2)
    {
        to32 = &base->REGX[region].REGKEY2[0];
    }
    else
    {
        return kStatus_InvalidArgument;
    }

    while (keySize >= sizeof(uint32_t))
    {
        *to32 = *from32;
        keySize -= sizeof(uint32_t);
        from32++;
        to32++;
    }

    return kStatus_Success;
}

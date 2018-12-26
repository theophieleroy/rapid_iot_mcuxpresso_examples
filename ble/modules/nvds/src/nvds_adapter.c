/*
 * The Clear BSD License
 * Copyright (c) 2016, NXP Semiconductors, N.V.
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
 * o Neither the name of Freescale Semiconductor, Inc. nor the names of its
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

#include "nvds_adapter.h"
#include "nvds.h"
#include "fsl_flash.h"
#include "system_config.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#if defined(CFG_NVDS_IN_CODE_SECTION)
extern void nvds_read(uint32_t address, uint32_t length, uint8_t *buf);
extern void nvds_null_write(uint32_t address, uint32_t length, uint8_t *buf);
extern void nvds_null_erase(uint32_t address, uint32_t length);
static uint8_t nvds_get_itf(uint8_t tag, nvds_tag_len_t *lengthPtr, uint8_t *buf);
static uint8_t nvds_del_itf(uint8_t tag);
static uint8_t nvds_lock_itf(uint8_t tag);
static uint8_t nvds_put_itf(uint8_t tag, nvds_tag_len_t length, uint8_t *buf);
#endif

/*******************************************************************************
 * Variables
 ******************************************************************************/

#if defined(CFG_NVDS_IN_CODE_SECTION)
/*! @brief Define in code section for read only */
const nvds_data_st_t nvds_space = {
    .magic_number = {'N', 'V', 'D', 'S'},
    .bd_addr = {0x70, 0x00, 0x00, 0xbe, 0x7c, 0x08},
    .dev_name = {'N', 'X', 'P', ' ', 'B', 'L', 'E', '\0'},
    .ppm = 0x64,
    .wakeup_time = 0x384,
    .xtal_loadcap = 0x08,
    .xtal32k_loadcap = 0x30,
};

const struct nvds_if nvds_api_itf = {nvds_read,    nvds_null_write, nvds_null_erase, nvds_get_itf,
                                     nvds_del_itf, nvds_lock_itf,   nvds_put_itf};

uint8_t nvds_public_key[64] = {0};
uint8_t nvds_private_key[32] = {0};
#else
extern flash_config_t g_flash_cfg;
#endif

/*******************************************************************************
 * Code
 ******************************************************************************/

uint8_t nvds_adapter_init(void)
{
    uint8_t *nvds_base;
    uint32_t nvds_size;
    uint8_t *nvds_backup_base;
    struct nvds_if *nvds_api;
    flash_config_t *flash_cfg;

#if defined(CFG_NVDS_IN_CODE_SECTION)
    nvds_base = (uint8_t *)&nvds_space;
    nvds_size = sizeof(nvds_space);
    nvds_backup_base = NULL;
    nvds_api = (struct nvds_if *)&nvds_api_itf;
    flash_cfg = NULL;
#else
    FLASH_GetDefaultConfig(&g_flash_cfg);
    FLASH_Init(&g_flash_cfg);
    nvds_base = (uint8_t *)CFG_NVDS_ADDRESS;
    nvds_size = CFG_NVDS_SIZE;
    nvds_backup_base = (uint8_t *)CFG_NVDS_BACKUP_ADDRESS;
    nvds_api = NULL;
    flash_cfg = &g_flash_cfg;
#endif

    return nvds_init(nvds_base, nvds_size, nvds_backup_base, nvds_api, flash_cfg);
}

#if defined(CFG_NVDS_IN_CODE_SECTION)
static uint8_t nvds_get_itf(uint8_t tag, nvds_tag_len_t *lengthPtr, uint8_t *buf)
{
    uint8_t status = NVDS_OK;
    uint16_t len;
    uint8_t *src;

    switch (tag)
    {
        case NVDS_TAG_BD_ADDRESS:
            src = (uint8_t *)&nvds_space.bd_addr[0];
            len = NVDS_LEN_BD_ADDRESS;
            break;
        case NVDS_TAG_DEVICE_NAME:
            src = (uint8_t *)&nvds_space.dev_name[0];
            len = sizeof(nvds_space.dev_name);
            break;
        case NVDS_TAG_LPCLK_DRIFT:
            src = (uint8_t *)&nvds_space.ppm;
            len = NVDS_LEN_LPCLK_DRIFT;
            break;
        case NVDS_TAG_OSC_WAKEUP_TIME:
            src = (uint8_t *)&nvds_space.wakeup_time;
            len = NVDS_LEN_OSC_WAKEUP_TIME;
            break;
        case NVDS_TAG_XCSEL:
            src = (uint8_t *)&nvds_space.xtal_loadcap;
            len = NVDS_LEN_XCSEL;
            break;
        case NVDS_TAG_32K_XCSEL:
            src = (uint8_t *)&nvds_space.xtal32k_loadcap;
            len = NVDS_LEN_32K_XCSEL;
            break;
        case NVDS_TAG_LE_PUBLIC_KEY_P256:
            status = NVDS_FAIL;
            for (int i = 0; i < NVDS_LEN_LE_PUBLIC_KEY_P256; i++)
            {
                if (nvds_public_key[i])
                {
                    status = NVDS_OK;
                    break;
                }
            }
            src = &nvds_public_key[0];
            len = NVDS_LEN_LE_PUBLIC_KEY_P256;
            break;
        case NVDS_TAG_LE_PRIVATE_KEY_P256:
            status = NVDS_FAIL;
            for (int i = 0; i < NVDS_LEN_LE_PRIVATE_KEY_P256; i++)
            {
                if (nvds_private_key[i])
                {
                    status = NVDS_OK;
                    break;
                }
            }
            src = &nvds_private_key[0];
            len = NVDS_LEN_LE_PRIVATE_KEY_P256;
            break;
        default:
            status = NVDS_FAIL;
            break;
    }

    if (status == NVDS_OK)
    {
        memcpy(buf, src, len);
    }

    return (status);
}

static uint8_t nvds_del_itf(uint8_t tag)
{
    return NVDS_FAIL;
}

static uint8_t nvds_lock_itf(uint8_t tag)
{
    return NVDS_FAIL;
}

static uint8_t nvds_put_itf(uint8_t tag, nvds_tag_len_t length, uint8_t *buf)
{
    uint8_t status = NVDS_OK;
    uint16_t len = length;
    uint8_t *dst;

    switch (tag)
    {
        case NVDS_TAG_LE_PUBLIC_KEY_P256:
            dst = (uint8_t *)&nvds_public_key[0];
            if (len > NVDS_LEN_LE_PUBLIC_KEY_P256)
            {
                status = NVDS_FAIL;
            }
            break;
        case NVDS_TAG_LE_PRIVATE_KEY_P256:
            dst = (uint8_t *)&nvds_private_key[0];
            if (len > NVDS_LEN_LE_PRIVATE_KEY_P256)
            {
                status = NVDS_FAIL;
            }
            break;
        default:
            status = NVDS_FAIL;
            break;
    }

    if (status == NVDS_OK)
    {
        memcpy(dst, buf, len);
    }

    return (status);
}
#endif

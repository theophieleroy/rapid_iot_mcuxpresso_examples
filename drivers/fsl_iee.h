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

#ifndef _FSL_IEE_H_
#define _FSL_IEE_H_

#include "fsl_common.h"

/*!
 * @addtogroup iee
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*! @name Driver version */
/*@{*/
/*! @brief IEE driver version. Version 2.0.0.
 *
 * Current version: 2.0.0
 *
 * Change log:
 * - Version 2.0.0
 *   - Initial version
 */
#define FSL_IEE_DRIVER_VERSION (MAKE_VERSION(2, 0, 0))
/*@}*/

typedef enum _iee_region
{
    kIEE_Region0 = 0U, /*!< IEE region 0 */
    kIEE_Region1 = 1U, /*!< IEE region 1 */
    kIEE_Region2 = 2U, /*!< IEE region 2 */
    kIEE_Region3 = 3U  /*!< IEE region 3 */
} iee_region_t;

typedef enum _iee_aes_bypass
{
    kIEE_AesUseMdField = 0U, /*!< AES encryption/decryption enabled */
    kIEE_AesBypass = 1U      /*!< AES encryption/decryption bypass */
} iee_aes_bypass_t;

typedef enum _iee_aes_mode
{
    kIEE_ModeNone = 0U,            /*!< AES NONE mode */
    kIEE_ModeAesXTS = 1U,          /*!< AES XTS mode */
    kIEE_ModeAesCTRWAddress = 2U,  /*!< CTR w address binding mode */
    kIEE_ModeAesCTRWOAddress = 3U, /*!< AES CTR w/o address binding mode */
    kIEE_ModeAesCTRkeystream = 4U  /*!< AES CTR keystream only */
} iee_aes_mode_t;

typedef enum _iee_aes_key_size
{
    kIEE_AesCTR128XTS256 = 0U, /*!< AES 128 bits (CTR), 256 bits (XTS) */
    kIEE_AesCTR256XTS512 = 1U  /*!< AES 256 bits (CTR), 512 bits (XTS) */
} iee_aes_key_size_t;

typedef enum _iee_aes_key_num
{
    kIEE_AesKey1 = 1U, /*!< AES Key 1 */
    kIEE_AesKey2 = 2U  /*!< AES Key 2 */
} iee_aes_key_num_t;

/*! @brief IEE configuration structure. */
typedef struct _iee_config
{
    iee_aes_bypass_t bypass;    /*!< AES encryption/decryption bypass */
    iee_aes_mode_t mode;        /*!< AES mode */
    iee_aes_key_size_t keySize; /*!< size of AES key */
    uint32_t pageOffset;        /*!< Offset to physical memory location from IEE start address */
} iee_config_t;

/*******************************************************************************
 * API
 ******************************************************************************/
#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @brief Resets IEE module to factory default values.
 *
 * This function performs hardware reset of IEE module. Attributes and keys of all regions are cleared.
 *
 * @param base IEER peripheral address.
 */
void IEE_Init(IEE_Type *base);

/*!
 * @brief Loads default values to the IEE configuration structure.
 *
 * Loads default values to the IEE region configuration structure. The default values are as follows.
 * @code
 *   config->bypass = kIEE_AesUseMdField;
 *   config->mode = kIEE_ModeNone;
 *   config->keySize = kIEE_AesCTR128XTS256;
 *   config->pageOffset = 0U;
 * @endcode
 *
 * @param config Configuration for the selected IEE region.
 */
void IEE_GetDefaultConfig(iee_config_t *config);

/*!
 * @brief Sets the IEE module according to the configuration structure.
 *
 * This function configures IEE region according to configuration structure.
 *
 * @param base IEE peripheral address.
 * @param region Selection of the IEE region to be configured.
 * @param config Configuration for the selected IEE region.
 */
void IEE_SetRegionConfig(IEE_Type *base, iee_region_t region, iee_config_t *config);

/*!
 * @brief Sets the IEE module key.
 *
 * This function sets specified AES key for the given region.
 *
 * @param base IEE peripheral address.
 * @param region Selection of the IEE region to be configured.
 * @param keyNum Selection of AES KEY1 or KEY2.
 * @param key AES key.
 * @param keySize Size of AES key.
 */
status_t IEE_SetRegionKey(
    IEE_Type *base, iee_region_t region, iee_aes_key_num_t keyNum, const uint8_t *key, size_t keySize);

/*!
 * @brief Computes IEE offset to be set for specifed memory location.
 *
 * This function calculates offset that must be set for IEE region to access physical memory location.
 *
 * @param addressIee Address of IEE peripheral.
 * @param addressMemory Address of physical memory location.
 */
static inline uint32_t IEE_GetOffset(uint32_t addressIee, uint32_t addressMemory)
{
    return (addressMemory - addressIee) >> 12;
}

#if defined(__cplusplus)
}
#endif

/*!
 *@}
 */

#endif /* _FSL_IEE_H_ */

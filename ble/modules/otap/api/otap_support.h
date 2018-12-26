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

#ifndef _OTA_SUPPORT_H_
#define _OTA_SUPPORT_H_
#include <stdint.h>
#include <stdbool.h>
#include "otap_interface.h"

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 * Definitions
 ******************************************************************************/

#ifndef gEnableOTAServer_d
#define gEnableOTAServer_d (0)
#endif

#ifndef gUpgradeImageOnCurrentDevice_d
#define gUpgradeImageOnCurrentDevice_d (0)
#endif

#define gOtaVersion_c (0x01)

#ifndef gOtaVerifyWrite_d
#define gOtaVerifyWrite_d (1)
#endif

#define gBootValueForTRUE_c (0x00)
#define gBootValueForFALSE_c (0xFF)

#define gBootData_ImageLength_Offset_c (0x00)
#define gBootData_ImageLength_Size_c (0x04)
#define gBootData_SectorsBitmap_Offset_c \
    gEepromAlignAddr_d(gBootData_ImageLength_Offset_c + gBootData_ImageLength_Size_c)
#define gBootData_SectorsBitmap_Size_c (32)
#define gBootData_Image_Offset_c gEepromAlignAddr_d(gBootData_SectorsBitmap_Offset_c + gBootData_SectorsBitmap_Size_c)

/* The maximum amount of MCU Flash memory */
#define gFlashParams_MaxImageLength_c (FSL_FEATURE_FLASH_PFLASH_BLOCK_SIZE * FSL_FEATURE_FLASH_PFLASH_BLOCK_COUNT)

typedef enum
{
    gOtaSucess_c = 0,
    gOtaNoImage_c,
    gOtaUpdated_c,
    gOtaError_c,
    gOtaCrcError_c,
    gOtaInvalidParam_c,
    gOtaInvalidOperation_c,
    gOtaExternalFlashError_c,
    gOtaInternalFlashError_c,
} otaResult_t;

/*******************************************************************************
 * API
 ******************************************************************************/

/*!
 * @brief  CRC compute.
 *
 * @param[in] pData Pointer to data.
 * @param[in] lenData Data length.
 * @param[in] crcValueOld Old CRC value.
 *
 * @return CRC value
 * @description
 *
 * This function compute data CRC with old CRC value.
 */
uint16_t OTA_CrcCompute(uint8_t *pData, uint32_t lenData, uint16_t crcValueOld);

/*!
 * @brief  Image information exchange.
 *
 * @param[in] pRemoteImgId Pointer to data.
 * @param[in] pRemoteImgVer Data length.
 * @param[in] pCurrentImgId Old CRC value.
 * @param[in] pcurrentImgVer Old CRC value.
 *
 * @return Whether ota client will accept the image from ota server.
 * @description
 *
 * Ota client use this function to decide whether the image from ota server is valid.
 */
bool OtapClient_ImageInformationExchange(uint8_t *pRemoteImgId,
                                         uint8_t *pRemoteImgVer,
                                         const uint8_t *pCurrentImgId,
                                         const uint8_t *pcurrentImgVer);

/*!
 * @brief  Check new image's header information.
 *
 * @param[in] imgFileHeader Pointer to new image's header section.
 *
 * @return Checked status of header information.
 * @description
 *
 * Ota client get more information about the new image from the new image's header section.
 */
otapStatus_t OtapClient_IsImageFileHeaderValid(bleOtaImageFileHeader_t *imgFileHeader);

/*!
 * @brief  Image information process.
 *
 * @param[in] pRemoteImgId Pointer to data.
 * @param[in] pRemoteImgVer Data length.
 * @param[in] pCurrentImgId Old CRC value.
 * @param[in] pcurrentImgVer Old CRC value.
 *
 * @return Null.
 * @description
 *
 * Ota server can do some more operations by using remote device's image information and its self's.
 */
void OtapServer_ImageInfoProc(uint8_t *pRemoteImgId,
                              uint8_t *pRemoteImgVer,
                              const uint8_t *pCurrentImgId,
                              const uint8_t *pcurrentImgVer);

/*!
 * @brief  Ota client received file type operation.
 *
 * @param[in] tag File type tag.
 *
 * @return Offset of the image should be stored in flash.
 * @description
 *
 * Ota client may received many types files, such as user data or an image. Ota client should store the file in right
 * position.
 */
uint32_t OtapClient_ReceivedFileType(bleOtaImageFileSubElementTagId_t tag);

/*!
 * @brief  Ota transmited image file percentage.
 *
 * @param[in] percent 0-100.
 *
 * @description
 *
 * Ota client received or server sent image file percentage.
 */
void Otap_TransmitedImagePercent(int percent);

#ifdef __cplusplus
}
#endif

#endif /* _OTA_SUPPORT_H_ */

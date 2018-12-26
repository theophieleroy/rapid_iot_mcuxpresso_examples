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

#ifndef _OTA_CLIENT_H_
#define _OTA_CLIENT_H_

/*!
 * @addtogroup otapClient_API
 * @{
 */

#include "otap_interface.h"
#include "fsl_flash.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define gBootData_SectorsBitmap_Size_c (32)

enum bootOption
{
    BOOT_IMG0,
    BOOT_IMG1,
};

struct boot_info_t
{
    uint32_t app_status; // valid: 0x55aaaa55; invalid: other values
    uint32_t app_addr;
    uint32_t app_size;
    uint32_t app_crc;
    uint32_t usr_info_addr;
    uint32_t usr_info_size;
    uint32_t info_protect;    // 1: boot info and user info cannot be read
    uint32_t conn_repeats;    // Connecting duration = conn_repeats * 20ms(@16M crystal)
    uint32_t watchdog_dis;    // Disable watchdog timer
    uint32_t watchdog_period; // Watchdog timer counter
};

typedef enum otapClientState_tag
{
    mOtapClientStateIdle_c = 0x00,
    mOtapClientStateDownloadingImage_c = 0x01,
    mOtapClientStateImageDownloadComplete_c = 0x02,
} otapClientState_t;

/*! Structure containing the OTAP Client functional data. */
typedef struct otapClientAppData_tag
{
    otapClientState_t state;
    uint8_t currentImgId[gOtap_ImageIdFieldSize_c]; /*!< Id of the currently running image on the OTAP Client */
    uint8_t
        currentImgVer[gOtap_ImageVersionFieldSize_c]; /*!< Version of the currently running image on the OTAP Client */
    uint8_t imgId[gOtap_ImageIdFieldSize_c];          /*!< Id of the image being downloaded from the OTAP Server */
    uint8_t imgVer[gOtap_ImageVersionFieldSize_c];    /*!< Version of the image being downloaded from the OTAP Server */
    uint32_t imgSize;          /*!< Size of the image file being downloaded from the OTAP Server */
    uint16_t imgComputedCrc;   /*!< Computed 16 bit CRC of the image file used in this implementation. */
    uint16_t imgReceivedCrc;   /*!< Received 16 bit CRC of the image file used in this implementation. */
    uint32_t currentPos;       /*!< Current position of the file being downloaded. */
    uint16_t chunkSize;        /*!< Current chunk size for the image file transfer. */
    uint16_t chunkSeqNum;      /*!< Current chunk sequence number for the block being transferred. */
    uint16_t totalBlockChunks; /*!< Total number of chunks for the block being transferred. */
    uint32_t totalBlockSize;   /*!< Total size of the block which was requested. may be smaller than totalBlockChunks *
                                  chunkSize. */
    const otapTransferMethod_t transferMethod; /*!< Currently used transfer method for the OTAP Image File */
    uint16_t l2capChannelOrPsm; /*!< L2CAP Channel or PSM used for the transfer of the image file: channel 0x0004 for
                                   ATT, application specific PSM for CoC. */
    bool serverWrittenCccd;     /*!< The OTAP Server has written the CCCD to receive commands from the OTAp Client. */
    otapCmdIdt_t
        lastCmdSentToOtapServer; /*!< The last command sent to the OTAP Server for which an Indication is expected. */
    uint16_t negotiatedMaxAttChunkSize;   /*!< The negotiated maximum ATT chunk size based on the negotiated ATT MTU
                                             between the OTAP Server and the OTAP Client. */
    uint16_t negotiatedMaxL2CapChunkSize; /*!< The negotiated maximum L2CAP chunk size based on the negotiated L2CAP MTU
                                             between the OTAP Server and the OTAP Client. */
    bool upgradeImageReceived;            /*!< Upgrade image has received. */
} otapClientAppData_t;

typedef struct otapClientInterface_tag
{
    void (*sendData)(uint16_t len, uint8_t *data);
    int (*usrDataProc)(uint16_t len, uint8_t *data);
    bool (*ImgInfoExchange)(uint8_t *pRemoteImgId,
                            uint8_t *pRemoteImgVer,
                            const uint8_t *pCurrentImgId,
                            const uint8_t *pcurrentImgVer);
    otapStatus_t (*IsImageFileHeaderValid)(bleOtaImageFileHeader_t *imgFileHeader);
    uint32_t (*getFlashDstAddrByImageType)(bleOtaImageFileSubElementTagId_t tag);
    uint16_t (*getMaxMtu)(void);
    void (*uploadImageComplete)(void);
} otapClientInterface_t;
/*******************************************************************************
 * Variables
 ******************************************************************************/
/*******************************************************************************
 * API
 ******************************************************************************/

#ifdef __cplusplus
extern "C" {
#endif

/*!
 * @brief OTA client register interface.
 *
 * @param[in] clientInterface       pointer to the data struct of otapClientInterface_t @see otapClientInterface_t
 *
 * @return
 * @description
 *
 * OTA client register interface.
 */
void OtapClient_InterfaceRegister(otapClientInterface_t *clientInterface);

/*!
 * @brief OTA client send a image information request to iap server.
 *
 * @description
 *
 * OTA client send a image information request to iap server.
 */
void OtapClient_ImageInfoRequest(void);

/*!
 * @brief OTA client process received data.
 *
 * @param[in] length        received data length
 * @param[in] pData         Pointer to received data
 *
 * @return
 * @description
 *
 * This function process iap client received data.
 */
void OtapClient_Proc(uint16_t length, uint8_t *pData);

/*!
 * @brief OTA client continue download from break point.
 *
 * @description
 *
 * This function continue download from break point.
 */
void OtapClient_ContinueDownloadFromBreakPoint(void);

/*!
 * @brief OTA client inform server to stop transmit image.
 *
 * @description
 *
 * This function make ota client send a stop command to server.
 */
void OtapClient_Stop(void);

/*!
 * @brief OTA client write received image to flash.
 *
 * @param[in] length        received data length
 * @param[in] data         Pointer to received data
 *
 * @description
 *
 * This function write received image to flash.
 */
status_t OtapClient_WriteImageToFlash(uint16_t len, const uint8_t *data);

/*!
 * @brief OTA client change boot information for copy new image from swap area to running area.
 *
 * @description
 *
 * This function change boot information for copy new image from swap area to running area.
 */
void OtapClient_BootInfoChange(void);

#ifdef __cplusplus
}
#endif

/*! @brief @} otapClient_API */

#endif /* _OTA_CLIENT_H_ */

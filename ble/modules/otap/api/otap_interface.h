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
#ifndef _OTAP_INTERFACE_H_
#define _OTAP_INTERFACE_H_

/*!
 * @addtogroup otapInterface_API
 * @{
 */

/************************************************************************************
*************************************************************************************
* Include
*************************************************************************************
************************************************************************************/
#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>
#include <app_config.h>

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#if defined(__GNUC__)
#define PACKED_STRUCT struct __attribute__((packed))
#define PACKED_UNION union __attribute__((packed))

#elif defined(__IAR_SYSTEMS_ICC__)
#define PACKED_STRUCT __packed struct
#define PACKED_UNION __packed union
#else
#define PACKED_STRUCT struct
#define PACKED_UNION union
#endif

#define BOOT_VALID_DATA (0x55AAAA55)
#define FLASH_SECTOR_SIZE (2048)
#define FLASH_START_ADDRESS (0x21000000)

#define FLASH_BOOT_IMG0_OFFSET (0x800)
#define FLASH_BOOT_IMG1_OFFSET (0x0)
#if defined(CFG_QN908XA)
#define FLASH_IMG0_SWAP_OFFSET (0x3F000)
#endif
#if defined(CFG_QN908XB)
#define FLASH_IMG0_SWAP_OFFSET (0x3F800)
#endif

#define FLASH_BOOT_INFO0_OFFSET (0x7E800)
#define FLASH_BOOT_INFO1_OFFSET (0x7F000)
#define VECTOR_CHECKA_SUM_OFFSET (0x1c)

#define FLASH_IMG0_ADDRESS (FLASH_START_ADDRESS + FLASH_BOOT_IMG0_OFFSET)
#define FLASH_IMG0_SWAP_ADDRESS (FLASH_START_ADDRESS + FLASH_IMG0_SWAP_OFFSET)
#define FLASH_IMG1_ADDRESS (FLASH_START_ADDRESS + FLASH_BOOT_IMG1_OFFSET)

#define APP_VALID_DATA (0x55AAAA55)
#define APP1_INFO_FLASH_PAGE_INDEX (FLASH_BOOT_INFO0_OFFSET / FLASH_SECTOR_SIZE)
#define APP2_INFO_FLASH_PAGE_INDEX (FLASH_BOOT_INFO1_OFFSET / FLASH_SECTOR_SIZE)

#define IMG0_SWAP_FLASH_PAGE_INDEX (FLASH_IMG0_SWAP_OFFSET / FLASH_SECTOR_SIZE)
#define BOOT0_IMG0_FLASH_PAGE_INDEX (FLASH_BOOT_IMG0_OFFSET / FLASH_SECTOR_SIZE)
#define BOOT1_IMG1_FLASH_PAGE_INDEX (FLASH_BOOT_IMG1_OFFSET / FLASH_SECTOR_SIZE)

#define gL2capCidAtt_c (0x0004)

#define gOtap_ServerConnectionNum (1U)

/*! Default value of the ATT_MTU */
#define gAttDefaultMtu_c (23)

/*! Maximum possible value of the ATT_MTU for this device. This is used during the MTU Exchange. */
#define gAttMaxMtu_c (247)

/*! BLE OTAP Protocol definitions. */
#define gOtap_CmdIdFieldSize_c (1U)
#define gOtap_ImageIdFieldSize_c (2U)
#define gOtap_ImageVersionFieldSize_c (8U)
#define gOtap_ChunkSeqNumberSize_c (1U)
#define gOtap_MaxChunksPerBlock_c (256U)

/*!< ATT_MTU - 3 - 1 - 1 = 23 - 3 - 1 - 1 = 20 - 2 = 18 for ATT Write Without Response */
#define gOtap_AttCommandMtuDataChunkOverhead_c \
    (3 + gOtap_CmdIdFieldSize_c +              \
     gOtap_ChunkSeqNumberSize_c) /*!< 3 for Att Opcode [1] and Attribute Handle [2] - ATT Write Without Response */
#define gOtap_l2capCmdMtuDataChunkOverhead_c \
    (2 + gOtap_CmdIdFieldSize_c + gOtap_ChunkSeqNumberSize_c) /*!< 2 for the eventual SDU Length */
#define gOtap_ImageChunkDataSizeAttDefault_c (gAttDefaultMtu_c - gOtap_AttCommandMtuDataChunkOverhead_c)
#define gOtap_ImageChunkDataSizeAttMax_c (gAttMaxMtu_c - gOtap_AttCommandMtuDataChunkOverhead_c)
#define gOtap_ImageChunkDataSizeL2capCoc_c (255U)
#define gOtap_ChunkHeaderLen_c (2)

#define gOtap_L2capLePsm_c (0x004F)

/*! BLE OTAP Image File Format Definitions */

/*! Enumerated type for the recognized image file header versions. */
typedef enum bleOtaFileHeaderVersion_tag
{
    gbleOtapHeaderVersion0100_c = 0x0100,
} bleOtaFileHeaderVersion_t;

#define gBleOtaFileHeaderIdentifier_c (0x0B1EF11E)
#define gBleOtaFileHeaderDefaultFieldControl_c (0x0000)
#define gBleOtaCompanyIdentifier_c (0x01FF)
#define gBleOtaHeaderStrLength_c (32)

#define gBleOtaImageIdCurrentRunningImage_c (0x0000)
#define gBleOtaImageIdNoImageAvailable_c (0xFFFF)
#pragma pack(1)

/*! BLE OTAP Image File Header */
typedef PACKED_STRUCT bleOtaImageFileHeader_tag
{
    uint32_t
        fileIdentifier; /*!< 0x0B1EF11E - Magic number to identify an image file as being a BLE upgrade image file. */
    uint16_t headerVersion; /*!< Version of the header. */
    uint16_t headerLength;  /*!< Header length. */
    uint16_t
        fieldControl;   /*!< Header field control - shows if any onther non-default fields are presetn in the header.*/
    uint16_t companyId; /*!< 0x01FF - FSL Company Identifier. */
    uint8_t imageId[gOtap_ImageIdFieldSize_c]; /*!< Should be unique for each image on an OTAP Server. */
    uint8_t imageVersion[gOtap_ImageVersionFieldSize_c];
    uint8_t headerString[gBleOtaHeaderStrLength_c];
    uint32_t totalImageFileSize; /*!< Total image file size including the header. */
}
bleOtaImageFileHeader_t;

/*! BLE OTAP Image File Sub-element tag id enumeration. */
typedef enum bleOtaImageFileSubElementTagId_tag
{
    /* Reserved sub-element tags */
    gBleOtaSubElemTagIdUpgradeImage_c = 0x0000, /*!< Sub-element contains the actual upgrade image. */
    /* Manufacturer specific sub-element tags */
    gBleOtaSubElemTagIdSectorBitmap_c =
        0xF000, /*!< Sub-element contains the sector bitmap specific to the FLASH memory of a the target device.
                 *   The size of this sub-element value is implementation specific. */
    gBleOtaSubElemTagIdImageFileCrc_c =
        0xF100, /*!< Sub-element contains the CRC of the image file (The CRC does not cover this sub-element)
                 *   The size of this sub-element value is implementation specific. */
    gBleOtaSubElemTagIdUserData_c = 0xF200,
} bleOtaImageFileSubElementTagId_t;

typedef PACKED_STRUCT subElementHeader_tag
{
    uint16_t tagId;   /*!< Sub-element tag identifier. */
    uint32_t dataLen; /*!< Sub-element value field length. */
}
subElementHeader_t;

/*! BLE OTAP Protocol statuses. */
typedef enum otapStatus_tag
{
    gOtapStatusSuccess_c = 0x00,              /*!< The operation was successful. */
    gOtapStatusImageDataNotExpected_c = 0x01, /*!< The OTAP Server tried to send an image data chunk to the OTAP Client
                                                 but the Client was not expecting it. */
    gOtapStatusUnexpectedTransferMethod_c = 0x02, /*!< The OTAP Server tried to send an image data chunk using a
                                                     transfer method the OTAP Client does not support/expect. */
    gOtapStatusUnexpectedCmdOnDataChannel_c =
        0x03, /*!< The OTAP Server tried to send an unexpected command (different from a data chunk) on a data Channel
                 (ATT or CoC) */
    gOtapStatusUnexpectedL2capChannelOrPsm_c =
        0x04, /*!< The selected channel or PSM is not valid for the selected transfer method (ATT or CoC). */
    gOtapStatusUnexpectedOtapPeer_c =
        0x05, /*!< A command was received from an unexpected OTAP Server or Client device. */
    gOtapStatusUnexpectedCommand_c =
        0x06, /*!< The command sent from the OTAP peer device is not expected in the current state. */
    gOtapStatusUnknownCommand_c = 0x07,            /*!< The command sent from the OTAP peer device is not known. */
    gOtapStatusInvalidCommandLength_c = 0x08,      /*!< Invalid command length. */
    gOtapStatusInvalidCommandParameter_c = 0x09,   /*!< A parameter of the command was not valid. */
    gOtapStatusFailedImageIntegrityCheck_c = 0x0A, /*!< The image integrity check has failed. */
    gOtapStatusUnexpectedSequenceNumber_c = 0x0B,  /*!< A chunk with an unexpected sequence number has been received. */
    gOtapStatusImageSizeTooLarge_c = 0x0C,         /*!< The upgrade image size is too large for the OTAP Client. */
    gOtapStatusUnexpectedDataLength_c = 0x0D,      /*!< The length of a Data Chunk was not expected. */
    gOtapStatusUnknownFileIdentifier_c = 0x0E,     /*!< The image file identifier is not recognized. */
    gOtapStatusUnknownHeaderVersion_c = 0x0F,      /*!< The image file header version is not recognized. */
    gOtapStatusUnexpectedHeaderLength_c =
        0x10, /*!< The image file header length is not expected for the current header version. */
    gOtapStatusUnexpectedHeaderFieldControl_c =
        0x11, /*!< The image file header field control is not expected for the current header version. */
    gOtapStatusUnknownCompanyId_c = 0x12,        /*!< The image file header company identifier is not recognized. */
    gOtapStatusUnexpectedImageId_c = 0x13,       /*!< The image file header image identifier is not as expected. */
    gOtapStatusUnexpectedImageVersion_c = 0x14,  /*!< The image file header image version is not as expected. */
    gOtapStatusUnexpectedImageFileSize_c = 0x15, /*!< The image file header image file size is not as expected. */
    gOtapStatusInvalidSubElementLength_c = 0x16, /*!< One of the sub-elements has an invalid length. */
    gOtapStatusImageStorageError_c = 0x17,       /*!< An image storage error has occurred. */
    gOtapStatusInvalidImageCrc_c = 0x18,         /*!< The computed CRC does not match the received CRC. */
    gOtapStatusInvalidImageFileSize_c = 0x19,    /*!< The image file size is not valid. */
    gOtapStatusInvalidL2capPsm_c = 0x1A, /*!< A block transfer request has been made via the L2CAP CoC method but the
                                            specified Psm is not known. */
    gOtapStatusNoL2capPsmConnection_c = 0x1B, /*!< A block transfer request has been made via the L2CAP CoC method but
                                                 there is no valid PSM connection. */
    gOtapNumberOfStatuses_c,
} otapStatus_t;

/*! OTAP Server storage mode type ennumeration. */
typedef enum otapServerStorageMode_tag
{
    gOtapServerStoargeNone_c = 0x00,     /*!< The OTAP Server does not have internal storage and all commands will be
                                            relayed between the OTAP Client and the PC (or other serial connected device).
                                            */
    gOtapServerStoargeInternal_c = 0x01, /*!< The OTAP Server has internal storage. It will download an image from a PC
                                            (or other serial connected device) and it will then send it to the OTAP
                                            Client. */
} otapServerStorageMode_t;

/*! OTAP Transfer Methods type defintion */
typedef enum otapTransferMethod_tag
{
    gOtapTransferMethodAtt_c = 0x00,
    gOtapTransferMethodL2capCoC_c = 0x01,
} otapTransferMethod_t;

/*! OTAP Protocol Command Id ennumeration */
typedef enum otapCmdIdt_tag
{
    gOtapCmdIdNoCommand_c = 0x00, /*!< No command. */
    gOtapCmdIdNewImageNotification_c =
        0x01, /*!< OTAP Server -> OTAP Client - A new image is available on the OTAP Server */
    gOtapCmdIdNewImageInfoRequest_c =
        0x02, /*!< OTAP Client -> OTAP Server - The OTAP Client requests image information from the OTAP Server */
    gOtapCmdIdNewImageInfoResponse_c =
        0x03, /*!< OTAP Server -> OTAP Client - The OTAP Server sends requested image information to the OTAP Client */
    gOtapCmdIdImageBlockRequest_c =
        0x04, /*!< OTAP Client -> OTAP Server - The OTAP Client requests an image block from the OTAP Server */
    gOtapCmdIdImageChunk_c =
        0x05, /*!< OTAP Server -> OTAP Client - The OTAP Server sends an image chunk to the OTAP Client */
    gOtapCmdIdImageTransferComplete_c =
        0x06, /*!< OTAP Client -> OTAP Server - The OTAP Client notifies the OTAP Server that an image transfer was
                 completed*/
    gOtapCmdIdErrorNotification_c = 0x07, /*!< Bidirectional - An error has occurred */
    gOtapCmdIdStopImageTransfer_c =
        0x08, /*!< OTAP Client -> OTAP Server - The OTAP Client request the OTAP Server to stop an image transfer. */
    gOtapCmdIdImageChunkRsp_c = 0x09,
} otapCmdIdt_t;

/*! OTAP New Image Notification Command */
typedef PACKED_STRUCT otapCmdNewImgNotification_tag
{
    uint8_t imageId[gOtap_ImageIdFieldSize_c];
    uint8_t imageVersion[gOtap_ImageVersionFieldSize_c];
    uint32_t imageFileSize;
}
otapCmdNewImgNotification_t;

/*! OTAP New Image Info Request Command */
typedef PACKED_STRUCT otapCmdNewImgInfoReq_tag
{
    uint8_t currentImageId[gOtap_ImageIdFieldSize_c];
    uint8_t currentImageVersion[gOtap_ImageVersionFieldSize_c];
}
otapCmdNewImgInfoReq_t;

/*! OTAP New Image Info Response Command */
typedef PACKED_STRUCT otapCmdNewImgInfoRes_tag
{
    uint8_t imageId[gOtap_ImageIdFieldSize_c];
    uint8_t imageVersion[gOtap_ImageVersionFieldSize_c];
    uint32_t imageFileSize;
}
otapCmdNewImgInfoRes_t;

/*! OTAP Image Block Request Command */
typedef PACKED_STRUCT otapCmdImgBlockReq_tag
{
    uint8_t imageId[gOtap_ImageIdFieldSize_c];
    uint32_t startPosition;
    uint32_t blockSize;
    uint16_t chunkSize;
    uint8_t transferMethod; /*!< otapTransferMethod_t - ATT or L2CAP Credit Oriented Channels (CoC) */
    uint16_t l2capChannelOrPsm;
}
otapCmdImgBlockReq_t;

/*! OTAP Image Chunk Command - for ATT transfer method only. */
typedef PACKED_STRUCT otapCmdImgChunkAtt_tag
{
    uint8_t seqNumber; /*!< The sequence number of the sent chunk. Max 256 chunks per block. */
    uint8_t data[gOtap_ImageChunkDataSizeAttMax_c];
}
otapCmdImgChunkAtt_t;

/*! OTAP Image Chunk Command - for CoC transfer method only. */
typedef PACKED_STRUCT otapCmdImgChunkCoc_tag
{
    uint8_t seqNumber; /*!< The sequence number of the sent chunk. Max 256 chunks per block. */
    uint8_t data[gOtap_ImageChunkDataSizeL2capCoc_c];
}
otapCmdImgChunkCoc_t;

/*! OTAP Image Transfer Complete Command */
typedef PACKED_STRUCT otapCmdImgTransferComplete_tag
{
    uint8_t imageId[gOtap_ImageIdFieldSize_c];
    uint8_t status; /*!< otapStatus_t */
}
otapCmdImgTransferComplete_t;

/*! OTAP Error Notification Command */
typedef PACKED_STRUCT otapErrNotification_tag
{
    uint8_t cmdId;     /*!< otapCmdIdt_t - The command which caused the error. */
    uint8_t errStatus; /*!< otapStatus_t */
}
otapErrNotification_t;

/*! OTAP Stop Image Transfer Command */
typedef PACKED_STRUCT otapCmdStopImgTransfer_tag
{
    uint8_t imageId[gOtap_ImageIdFieldSize_c];
}
otapCmdStopImgTransfer_t;

/*! General OTAP Command type struncture.
 *  The image chunk commands are not included here. Their length is variable. */
typedef PACKED_STRUCT otapCommand_tag
{
    uint8_t cmdId;
    PACKED_UNION
    {
        /* Pairing Methods */
        otapCmdNewImgNotification_t newImgNotif;
        otapCmdNewImgInfoReq_t newImgInfoReq;
        otapCmdNewImgInfoRes_t newImgInfoRes;
        otapCmdImgBlockReq_t imgBlockReq;
        otapCmdImgTransferComplete_t imgTransComplete;
        otapErrNotification_t errNotif;
        otapCmdStopImgTransfer_t stopImgTransf;
    }
    cmd;
}
otapCommand_t;
#pragma pack()

/*! Structure type holding basic information about the images available on an OTAP Server. */
typedef struct imgInfo_tag
{
    uint8_t imgId[gOtap_ImageIdFieldSize_c];       /*!< Image id. */
    uint8_t imgVer[gOtap_ImageVersionFieldSize_c]; /*!< Image version. */
    uint32_t imgSize;                              /*!< Image size. */
} imgInfo_t;

/*! OTAP Protocol Command Id to Command Length table.
 *  The length includes the Command Id and the Command Payload. */
static const uint8_t cmdIdToCmdLengthTable[] = {
        [gOtapCmdIdNoCommand_c] = 0,
        [gOtapCmdIdNewImageNotification_c] = sizeof(otapCmdNewImgNotification_t) + gOtap_CmdIdFieldSize_c,
        [gOtapCmdIdNewImageInfoRequest_c] = sizeof(otapCmdNewImgInfoReq_t) + gOtap_CmdIdFieldSize_c,
        [gOtapCmdIdNewImageInfoResponse_c] = sizeof(otapCmdNewImgInfoRes_t) + gOtap_CmdIdFieldSize_c,
        [gOtapCmdIdImageBlockRequest_c] = sizeof(otapCmdImgBlockReq_t) + gOtap_CmdIdFieldSize_c,
        [gOtapCmdIdImageChunk_c] =
            sizeof(otapCmdImgChunkAtt_t) + gOtap_CmdIdFieldSize_c, /*!< For ATT transfer method only, maximum length. */
        [gOtapCmdIdImageTransferComplete_c] = sizeof(otapCmdImgTransferComplete_t) + gOtap_CmdIdFieldSize_c,
        [gOtapCmdIdErrorNotification_c] = sizeof(otapErrNotification_t) + gOtap_CmdIdFieldSize_c,
        [gOtapCmdIdStopImageTransfer_c] = sizeof(otapCmdStopImgTransfer_t) + gOtap_CmdIdFieldSize_c,
};

#define gOtapCmdImageChunkCocMaxLength_c (sizeof(otapCmdImgChunkCoc_t) + gOtap_CmdIdFieldSize_c)
//#define gOtapCmdImageChunkAttMaxLength_c   (gOtap_ImageChunkDataSizeAttMax_c + gOtap_ChunkSeqNumberSize_c +
// gOtap_CmdIdFieldSize_c)

/*******************************************************************************
 * Variables
 ******************************************************************************/
/*******************************************************************************
 * API
 ******************************************************************************/
#ifdef __cplusplus
extern "C" {
#endif

#ifdef __cplusplus
}
#endif
/*! @brief @} otapClient_API */
#endif /* _OTAP_INTERFACE_H_ */

/* _OTAP_INTERFACE_H_ */

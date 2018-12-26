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

#include <string.h>
#include <stddef.h>
#include "otap_client.h"
#include "otap_server.h"
#include "otap_interface.h"
#include "otap_support.h"

/*******************************************************************************
 * Variables
 ******************************************************************************/
extern flash_config_t g_flash_cfg;
static uint32_t writeFlashOffset = 0;
static uint32_t startAddrForOtap = FLASH_IMG0_SWAP_OFFSET;

/*! OTAP Client data structure.
 *  Contains current image information and state informations
 *  regarding the image download procedure. */
static otapClientAppData_t s_otapClientData = {
    .state = mOtapClientStateIdle_c,
    .currentImgId = {0x00, 0x00},
    .currentImgVer = {0x01, 0x00, 0x00, 0x41, 0x11, 0x11, 0x11, 0x01},
    .imgId = {0x00, 0x00},
    .imgVer = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    .imgSize = 0,
    .imgComputedCrc = 0,
    .imgReceivedCrc = 0,
    .currentPos = 0,
    .chunkSize = 0,
    .chunkSeqNum = 0,
    .totalBlockChunks = 0,
    .totalBlockSize = 0,
    .transferMethod = gOtapTransferMethodAtt_c,
    .l2capChannelOrPsm = gL2capCidAtt_c,
    .lastCmdSentToOtapServer = gOtapCmdIdNoCommand_c,
    .negotiatedMaxAttChunkSize = gAttDefaultMtu_c - gOtap_AttCommandMtuDataChunkOverhead_c,
    .negotiatedMaxL2CapChunkSize = gOtap_ImageChunkDataSizeL2capCoc_c,
    .upgradeImageReceived = false,
};

static otapClientInterface_t s_otapClientInterface;

/*******************************************************************************
 * Code
 ******************************************************************************/

static void OtapClient_CancelImage(void)
{
    writeFlashOffset = 0;
}

static void OtapClient_ResetBlockTransferParameters(void)
{
    s_otapClientData.chunkSize = 0;
    s_otapClientData.chunkSeqNum = 0;
    s_otapClientData.totalBlockChunks = 0;
    s_otapClientData.totalBlockSize = 0;
}
static void OtapClient_ContinueImageDownload(void)
{
    otapCommand_t otapCommand;
    uint32_t bytesToDownload;
    uint32_t maxBlockSize;

    /* If the last received chunk sequence number is equal to the total block
     * chunks or they are both zero then ask for a new block from the server. */
    if (s_otapClientData.chunkSeqNum != s_otapClientData.totalBlockChunks)
        return;

    /* Ask for another block only if the image transfer was not completed. */
    if (s_otapClientData.currentPos >= s_otapClientData.imgSize)
        return;

    bytesToDownload = s_otapClientData.imgSize - s_otapClientData.currentPos;

    if (s_otapClientData.transferMethod == gOtapTransferMethodAtt_c)
    {
        maxBlockSize = s_otapClientData.negotiatedMaxAttChunkSize * gOtap_MaxChunksPerBlock_c;
        s_otapClientData.l2capChannelOrPsm = gL2capCidAtt_c;
        s_otapClientData.chunkSize = s_otapClientData.negotiatedMaxAttChunkSize;
    }
    else if (s_otapClientData.transferMethod == gOtapTransferMethodL2capCoC_c)
    {
        if (s_otapClientData.l2capChannelOrPsm == gOtap_L2capLePsm_c)
        {
            maxBlockSize = s_otapClientData.negotiatedMaxL2CapChunkSize * gOtap_MaxChunksPerBlock_c;
            s_otapClientData.chunkSize = s_otapClientData.negotiatedMaxL2CapChunkSize;
        }
        else
        {
            /* If the L2CAP CoC is not valid then some kind of error or missconfiguration has
             * occurred. Send a proper error notification to the peer and
             * reset the download state machine to Idle. */
            s_otapClientData.state = mOtapClientStateIdle_c;

            otapCommand.cmdId = gOtapCmdIdErrorNotification_c;
            otapCommand.cmd.errNotif.cmdId = gOtapCmdIdNoCommand_c;
            otapCommand.cmd.errNotif.errStatus = gOtapStatusUnexpectedL2capChannelOrPsm_c;

            QPRINTF("gOtapCmdIdErrorNotification_c\r\n");
            s_otapClientInterface.sendData(cmdIdToCmdLengthTable[gOtapCmdIdErrorNotification_c],
                                           (void *)(&otapCommand));

            return;
        }
    }
    else
    {
        /* If the transfer method is not recognized then this image has been missconfigured
         * or a critical error has occurred. Send a proper error notification to the peer and
         * reset the download state machien to Idle. */
        s_otapClientData.state = mOtapClientStateIdle_c;

        otapCommand.cmdId = gOtapCmdIdErrorNotification_c;
        otapCommand.cmd.errNotif.cmdId = gOtapCmdIdNoCommand_c;
        otapCommand.cmd.errNotif.errStatus = gOtapStatusUnexpectedTransferMethod_c;

        QPRINTF("gOtapCmdIdErrorNotification_c\r\n");
        s_otapClientInterface.sendData(cmdIdToCmdLengthTable[gOtapCmdIdErrorNotification_c], (void *)(&otapCommand));
        return;
    }

    if (bytesToDownload >= maxBlockSize)
    {
        /* If there are more bytes to download than the maximum block size then
         * ask a full block from the server on the selected tansfer method and set up
         * the client to recieve the chunks.*/
        s_otapClientData.chunkSeqNum = 0;
        s_otapClientData.totalBlockChunks = gOtap_MaxChunksPerBlock_c;
        s_otapClientData.totalBlockSize = maxBlockSize;
    }
    else
    {
        /* If there are fewer bytes to download than the maximum block size then compute the
        *  number of chunks expected and set the expected block size to the number of
        *  bytes to download. */
        s_otapClientData.chunkSeqNum = 0;
        /* Compute number of full chunks. Integer division. */
        s_otapClientData.totalBlockChunks = bytesToDownload / s_otapClientData.chunkSize;
        /* Add an extra chunk if the chunk size is not a divisor of the number of bytes to download. */
        s_otapClientData.totalBlockChunks += (bytesToDownload % s_otapClientData.chunkSize) ? 1 : 0;
        s_otapClientData.totalBlockSize = bytesToDownload;
    }

    /* Send the Block request Command with the determined parameters. */
    otapCommand.cmdId = gOtapCmdIdImageBlockRequest_c;

    memcpy(otapCommand.cmd.imgBlockReq.imageId, s_otapClientData.imgId, gOtap_ImageIdFieldSize_c);
    otapCommand.cmd.imgBlockReq.startPosition = s_otapClientData.currentPos;
    otapCommand.cmd.imgBlockReq.blockSize = s_otapClientData.totalBlockSize;
    otapCommand.cmd.imgBlockReq.chunkSize = s_otapClientData.chunkSize;
    otapCommand.cmd.imgBlockReq.transferMethod = s_otapClientData.transferMethod;
    otapCommand.cmd.imgBlockReq.l2capChannelOrPsm = s_otapClientData.l2capChannelOrPsm;

    s_otapClientInterface.sendData(cmdIdToCmdLengthTable[gOtapCmdIdImageBlockRequest_c], (void *)(&otapCommand));
}
static void OtapClient_HandleErrorNotification(uint16_t length, uint8_t *pValue)
{
    otapCommand_t otapCommand;
    otapStatus_t otapStatus = gOtapStatusSuccess_c;
    otapCommand_t *pRemoteCmd = (otapCommand_t *)pValue;

    /* Check the command length and parameters. */
    if (length == cmdIdToCmdLengthTable[gOtapCmdIdErrorNotification_c])
    {
        /*! Handle remote error statuses here. */
        if (pRemoteCmd->cmd.errNotif.errStatus < gOtapNumberOfStatuses_c)
        {
            /* Handle all errors in the same way, disconnect to restart the download process. Not done yet*/
        }
        else
        {
            otapStatus = gOtapStatusInvalidCommandParameter_c;
        }
    }
    else
    {
        otapStatus = gOtapStatusInvalidCommandLength_c;
    }

    if (otapStatus != gOtapStatusSuccess_c)
    {
        otapCommand.cmdId = gOtapCmdIdErrorNotification_c;
        otapCommand.cmd.errNotif.cmdId = gOtapCmdIdNewImageInfoResponse_c;
        otapCommand.cmd.errNotif.errStatus = otapStatus;

        QPRINTF("gOtapCmdIdErrorNotification_c\r\n");
        s_otapClientInterface.sendData(cmdIdToCmdLengthTable[gOtapCmdIdErrorNotification_c], (void *)(&otapCommand));
    }
}

static otapStatus_t OtapClient_DataChunkLengthCheck(uint16_t length, uint8_t *pData)
{
    otapStatus_t otapStatus = gOtapStatusInvalidCommandLength_c;
    otapCmdImgChunkCoc_t *pDataChunk =
        (otapCmdImgChunkCoc_t *)(&((otapCommand_t *)pData)->cmd); // use the CoC Data Chunk type but observe the length
    uint16_t dataLen = length - gOtap_CmdIdFieldSize_c - gOtap_ChunkSeqNumberSize_c; // len

    /* Check if the command length is as expected. */
    if (length <= (gOtap_CmdIdFieldSize_c + gOtap_ChunkSeqNumberSize_c))
        return otapStatus;
    if (!(((s_otapClientData.transferMethod == gOtapTransferMethodAtt_c) &&
           (length <= s_otapClientData.negotiatedMaxAttChunkSize + gOtap_ChunkHeaderLen_c)) ||
          ((s_otapClientData.transferMethod == gOtapTransferMethodL2capCoC_c) &&
           (length <= gOtapCmdImageChunkCocMaxLength_c))))
        return otapStatus;
    /* Check if the chunk (sequence number) is as expected */
    if (!((pDataChunk->seqNumber == s_otapClientData.chunkSeqNum) &&
          (pDataChunk->seqNumber < s_otapClientData.totalBlockChunks)))
    {
        otapStatus = gOtapStatusUnexpectedSequenceNumber_c;
        return otapStatus;
    }
    /*  Check if the data length is as expected. */
    if (((dataLen == s_otapClientData.chunkSize) &&
         ((pDataChunk->seqNumber < (s_otapClientData.totalBlockChunks - 1)) ||
          (s_otapClientData.totalBlockSize % s_otapClientData.chunkSize == 0))) ||
        ((dataLen < s_otapClientData.chunkSize) && (pDataChunk->seqNumber == (s_otapClientData.totalBlockChunks - 1)) &&
         (dataLen == s_otapClientData.totalBlockSize % s_otapClientData.chunkSize)))
    {
        otapStatus = gOtapStatusSuccess_c;
    }
    else
    {
        otapStatus = gOtapStatusUnexpectedDataLength_c;
    }

    return otapStatus;
}

#if defined(CFG_QN908XA)
static void OtapClient_BootInfoSet(enum bootOption bootSection, bool valid)
{
    struct boot_info_t boot_info;
    uint32_t info_offset;
    uint32_t app_addr;

    if (bootSection == BOOT_IMG1)
    {
        info_offset = FLASH_BOOT_INFO1_OFFSET;
        app_addr = FLASH_BOOT_IMG1_OFFSET;
    }
    else
    {
        info_offset = FLASH_BOOT_INFO0_OFFSET;
        app_addr = FLASH_BOOT_IMG0_OFFSET;
    }

    memcpy(&boot_info, (struct boot_info_t *)(FLASH_START_ADDRESS + info_offset), sizeof(struct boot_info_t));
    if (valid == true)
        boot_info.app_status = BOOT_VALID_DATA;
    else
        boot_info.app_status = 0;

    boot_info.app_addr = FLASH_START_ADDRESS + app_addr;
    boot_info.usr_info_addr = 0x2107D800;
    boot_info.usr_info_size = 2048;
    boot_info.info_protect = 0;
    boot_info.conn_repeats = 16;
    boot_info.watchdog_dis = 0;
    boot_info.watchdog_period = 0x9896800;

    FLASH_PageErase(&g_flash_cfg, info_offset / FLASH_SECTOR_SIZE);
    FLASH_Program(&g_flash_cfg, FLASH_START_ADDRESS + info_offset, (uint8_t *)&boot_info,
                  sizeof(struct boot_info_t));
}

static void OtapClient_BootImageSet(enum bootOption boot)
{
    if (boot == BOOT_IMG1)
    {
        OtapClient_BootInfoSet(BOOT_IMG1, true);
        OtapClient_BootInfoSet(BOOT_IMG0, false);
    }
    else if (boot == BOOT_IMG0)
    {
        OtapClient_BootInfoSet(BOOT_IMG0, true);
        OtapClient_BootInfoSet(BOOT_IMG1, false);
    }
}
#endif

void OtapClient_BootInfoChange(void)
{
#if defined(CFG_QN908XA)
    OtapClient_BootImageSet(BOOT_IMG1);
#endif
}

static otapStatus_t OtapClient_CalculateWetherNeedNewBlock(uint16_t length, uint8_t *pData)
{
    otapStatus_t otapStatus = gOtapStatusSuccess_c;
    otapCommand_t otapCommand;
    /* If the chunk has been successfully processed increase the expected sequence number. */
    s_otapClientData.chunkSeqNum += 1;

    /* Check if the block and/or image transfer is complete */
    if (s_otapClientData.chunkSeqNum < s_otapClientData.totalBlockChunks)
        return otapStatus;

    /* If the image transfer is complete check the image CRC then
     * commit the image and set the bootloader flags. */
    if (s_otapClientData.currentPos >= s_otapClientData.imgSize)
    {
        if (s_otapClientData.imgComputedCrc != s_otapClientData.imgReceivedCrc)
        {
            otapStatus = gOtapStatusInvalidImageCrc_c;
            s_otapClientData.currentPos = 0;
            OtapClient_CancelImage();
        }
        else
        {
            /* The new image was successfully committed, set the bootloader new image flags,
             * set the image transfer state as downloaded and send an image transfer complete
             * message to the peer. */
            s_otapClientData.state = mOtapClientStateImageDownloadComplete_c;

            otapCommand.cmdId = gOtapCmdIdImageTransferComplete_c;
            memcpy((uint8_t *)otapCommand.cmd.imgTransComplete.imageId, s_otapClientData.imgId,
                   sizeof(otapCommand.cmd.imgTransComplete.imageId));
            otapCommand.cmd.imgTransComplete.status = gOtapStatusSuccess_c;

            QPRINTF("\r\ngOtapCmdIdImageTransferComplete_c\r\n");
            s_otapClientInterface.sendData(cmdIdToCmdLengthTable[gOtapCmdIdImageTransferComplete_c],
                                           (void *)(&otapCommand));

            if (s_otapClientData.upgradeImageReceived == true)
            {
                uint16_t crc = OTA_CrcCompute(((uint8_t*)(FLASH_IMG0_SWAP_OFFSET +  FLASH_START_ADDRESS)),
                                            *((uint32_t*)(FLASH_IMG0_SWAP_OFFSET +  FLASH_START_ADDRESS)), 0); 
                OtapClient_WriteImageToFlash(2, (uint8_t *)&crc);
                s_otapClientInterface.uploadImageComplete();
            }
        }
    }
    else
    {
        /* If just the current block is complete ask for another block. */
        OtapClient_ContinueImageDownload();
    }

    return otapStatus;
}

static otaResult_t OtapClient_StartImage(uint32_t length)
{
    if (length <= (FLASH_IMG0_SWAP_OFFSET - 4))
        return gOtaSucess_c;
    else
        return gOtaInvalidParam_c;
}

static void OtapClient_HandleDataChunk(uint16_t length, uint8_t *pData)
{
    otapCommand_t otapCommand;

    otapStatus_t otapStatus = gOtapStatusInvalidCommandLength_c;

    otapCmdImgChunkCoc_t *pDataChunk =
        (otapCmdImgChunkCoc_t *)(&((otapCommand_t *)pData)->cmd); // use the CoC Data Chunk type but observe the length
    uint16_t dataLen = length - gOtap_CmdIdFieldSize_c - gOtap_ChunkSeqNumberSize_c; // len

    /* Variables for the local image file parsing state machine. */
    static uint32_t currentImgElemRcvdLen =
        0; /*!< Contains the number of received bytes for th current image element (header or othe sub element).
                     *   This is needed because the */
    static bleOtaImageFileHeader_t imgFileHeader; /*!< Saved image file header. */
    static uint32_t elementEnd = 0;               /*!< Current image file element expected end. */
    static subElementHeader_t subElemHdr;

    otapStatus = OtapClient_DataChunkLengthCheck(length, pData);

    do
    {
        /*! If all checks were successful then parse the current data chunk, else send an error notification. */
        if (otapStatus != gOtapStatusSuccess_c)
            break;

        pData = (uint8_t *)(&pDataChunk->data);

        /* If the Current position is 0 then reset the received length for the current image element
         * and the current image CRC to the initialization value which is 0.
         * The current position should be 0 only at the start of the image file transfer. */
        if (s_otapClientData.currentPos == 0)
        {
            currentImgElemRcvdLen = 0;
            s_otapClientData.imgComputedCrc = 0;
        }

        /* Parse all the bytes in the data payload. */
        while (dataLen)
        {
            /* Wait for the header to arrive and check it's contents
             * then handle the elements of the image. */
            if (s_otapClientData.currentPos < sizeof(bleOtaImageFileHeader_t))
            {
                if ((s_otapClientData.currentPos + dataLen) >= sizeof(bleOtaImageFileHeader_t))
                {
                    uint16_t residualHeaderLen = sizeof(bleOtaImageFileHeader_t) - s_otapClientData.currentPos;

                    /* There is enough information in the data payload to complete the header. */
                    memcpy((uint8_t *)(&imgFileHeader) + s_otapClientData.currentPos, pData, residualHeaderLen);
                    s_otapClientData.currentPos += residualHeaderLen;
                    pData += residualHeaderLen;
                    dataLen -= residualHeaderLen;

                    /* Check header contents, and if it is not valid return and error and reset the image download
                     * position. */
                    otapStatus = s_otapClientInterface.IsImageFileHeaderValid(&imgFileHeader);
                    if (otapStatus != gOtapStatusSuccess_c)
                    {
                        s_otapClientData.currentPos = 0;
                        break;
                    }

                    /* If the header is valid then update the CRC over the header part of the image. */
                    s_otapClientData.imgComputedCrc = OTA_CrcCompute(
                        (uint8_t *)(&imgFileHeader), sizeof(bleOtaImageFileHeader_t), s_otapClientData.imgComputedCrc);

                    currentImgElemRcvdLen = 0;

                    /* If the remaining data length is not 0 then the loop will continue with the parsing of the next
                     * element. */
                }
                else
                {
                    /* Not enough data to complete the header.
                     * Copy all the data into the temporary header and
                     * increment the current image position. */
                    memcpy((uint8_t *)(&imgFileHeader) + s_otapClientData.currentPos, pData, dataLen);
                    s_otapClientData.currentPos += dataLen;
                    dataLen = 0;
                }
            }
            else
            {
                /* The parsing has reached the sub-elements portion of the image.
                 * Wait for each sub-element tag to arrive or parse it if it is known. */
                if (currentImgElemRcvdLen < sizeof(subElementHeader_t))
                {
                    if ((currentImgElemRcvdLen + dataLen) >= sizeof(subElementHeader_t))
                    {
                        uint16_t residualSubElemHdrLen = sizeof(subElementHeader_t) - currentImgElemRcvdLen;

                        /* There is enough information in the data payload to complete the sub-element header. */
                        memcpy((uint8_t *)(&subElemHdr) + currentImgElemRcvdLen, pData, residualSubElemHdrLen);
                        s_otapClientData.currentPos += residualSubElemHdrLen;
                        currentImgElemRcvdLen += residualSubElemHdrLen;
                        pData += residualSubElemHdrLen;
                        dataLen -= residualSubElemHdrLen;

                        /* Update the CRC over the sub-element header only if it is not the CRC Sub-Element header. */
                        if (subElemHdr.tagId != gBleOtaSubElemTagIdImageFileCrc_c)
                        {
                            s_otapClientData.imgComputedCrc = OTA_CrcCompute(
                                (uint8_t *)(&subElemHdr), sizeof(subElementHeader_t), s_otapClientData.imgComputedCrc);
                        }

                        elementEnd = s_otapClientData.currentPos + subElemHdr.dataLen;

                        /* If the remaining data length is not 0 then the loop will
                        continue with the parsing of the sub-element. */
                    }
                    else
                    {
                        /* Not enough data to complete the sub-element header.
                         * Copy all the data into the temporary sub-element header
                         * and increment the current image position. */
                        memcpy((uint8_t *)(&subElemHdr) + currentImgElemRcvdLen, pData, dataLen);
                        s_otapClientData.currentPos += dataLen;
                        currentImgElemRcvdLen += dataLen;
                        dataLen = 0;
                    }
                }
                else
                {
                    uint32_t elementChunkLength = 0;

                    /* Make sure we do not pass the current element boundary. */
                    if ((s_otapClientData.currentPos + dataLen) >= elementEnd)
                    {
                        elementChunkLength = elementEnd - s_otapClientData.currentPos;
                    }
                    else
                    {
                        elementChunkLength = dataLen;
                    }

                    /* Handle sub-element payload. */
                    switch (subElemHdr.tagId)
                    {
                        case gBleOtaSubElemTagIdUpgradeImage_c:
                            /* Immediately after receiving the header check if the image sub-element length is valid
                                           * by trying to start the image upgrade procedure. */
                            if (false == s_otapClientData.upgradeImageReceived)
                            {
                                startAddrForOtap =
                                    s_otapClientInterface.getFlashDstAddrByImageType(gBleOtaSubElemTagIdUpgradeImage_c);
                                s_otapClientData.upgradeImageReceived = true;
                                OtapClient_WriteImageToFlash(4, (uint8_t *)&subElemHdr.dataLen);
                                QPRINTF("get image\r\n");
                            }
                            if ((currentImgElemRcvdLen == sizeof(subElementHeader_t)) &&
                                (gOtaSucess_c != OtapClient_StartImage(subElemHdr.dataLen)))
                            {
                                /* The sub-element length is invalid, set an error status and reset
                                 * the image file download process. */
                                otapStatus = gOtapStatusImageSizeTooLarge_c;
                                s_otapClientData.currentPos = 0;
                                break;
                            }

                            /* Upgrade Image Tag - compute the CRC and try to push the chunk to the storage. */
                            s_otapClientData.imgComputedCrc =
                                OTA_CrcCompute(pData, elementChunkLength, s_otapClientData.imgComputedCrc);
                            if (gOtaSucess_c != s_otapClientInterface.usrDataProc(elementChunkLength, pData))
                            {
                                otapStatus = gOtapStatusImageStorageError_c;
                                s_otapClientData.currentPos = 0;
                                OtapClient_CancelImage();
                                break;
                            }
                            break;

                        case gBleOtaSubElemTagIdUserData_c:
                            startAddrForOtap =
                                s_otapClientInterface.getFlashDstAddrByImageType(gBleOtaSubElemTagIdUserData_c);
                            if ((currentImgElemRcvdLen == sizeof(subElementHeader_t)) &&
                                (gOtaSucess_c != OtapClient_StartImage(subElemHdr.dataLen)))
                            {
                                /* The sub-element length is invalid, set an error status and reset
                                 * the image file download process. */
                                otapStatus = gOtapStatusImageSizeTooLarge_c;
                                s_otapClientData.currentPos = 0;
                                break;
                            }

                            /* Upgrade Image Tag - compute the CRC and try to push the chunk to the storage. */
                            s_otapClientData.imgComputedCrc =
                                OTA_CrcCompute(pData, elementChunkLength, s_otapClientData.imgComputedCrc);
                            if (gOtaSucess_c != s_otapClientInterface.usrDataProc(elementChunkLength, pData))
                            {
                                otapStatus = gOtapStatusImageStorageError_c;
                                s_otapClientData.currentPos = 0;
                                OtapClient_CancelImage();
                                break;
                            }

                            break;

                        case gBleOtaSubElemTagIdImageFileCrc_c:
                            /* Immediately after receiving the header check if the sub-element length is valid. */
                            if ((currentImgElemRcvdLen == sizeof(subElementHeader_t)) &&
                                (subElemHdr.dataLen != sizeof(s_otapClientData.imgReceivedCrc)))
                            {
                                /* The sub-element length is invalid, set an error status and reset
                                 * the image file download process. */
                                otapStatus = gOtapStatusInvalidSubElementLength_c;
                                s_otapClientData.currentPos = 0;
                                OtapClient_CancelImage();
                                break;
                            }

                            /* CRC Tag - Just copy the received CRC to the buffer. */
                            memcpy((uint8_t *)(&s_otapClientData.imgReceivedCrc) +
                                       (currentImgElemRcvdLen - sizeof(subElementHeader_t)),
                                   pData, elementChunkLength);
                            break;

                        default:
                            /* Unknown sub-element type, just compute the CRC over it. */
                            s_otapClientData.imgComputedCrc =
                                OTA_CrcCompute(pData, elementChunkLength, s_otapClientData.imgComputedCrc);
                            break;
                    };

                    if (otapStatus != gOtapStatusSuccess_c)
                    {
                        /* If an error has occurred then break the loop. */
                        break;
                    }

                    s_otapClientData.currentPos += elementChunkLength;
                    currentImgElemRcvdLen += elementChunkLength;
                    pData += elementChunkLength;
                    dataLen -= elementChunkLength;

                    int otaPercent = s_otapClientData.currentPos * 100 / s_otapClientData.imgSize;
                    Otap_TransmitedImagePercent(otaPercent);

                    /* If this element has been completely received then reset the current element
                     * received length to trigger the reception of the next sub-element. */
                    if (s_otapClientData.currentPos >= elementEnd)
                    {
                        currentImgElemRcvdLen = 0;
                    }
                }
            }
        } /* while (dataLen) */

    } while (0);

    if (otapStatus == gOtapStatusSuccess_c)
    {
        otapStatus = OtapClient_CalculateWetherNeedNewBlock(length, pData);
    }

    if (otapStatus != gOtapStatusSuccess_c)
    {
        otapCommand.cmdId = gOtapCmdIdErrorNotification_c;
        otapCommand.cmd.errNotif.cmdId = gOtapCmdIdImageChunk_c;
        otapCommand.cmd.errNotif.errStatus = otapStatus;

        QPRINTF("gOtapCmdIdErrorNotification_c %02x\r\n", otapStatus);
        s_otapClientInterface.sendData(cmdIdToCmdLengthTable[gOtapCmdIdErrorNotification_c], (void *)(&otapCommand));
    }
}

static void OtapClient_IdleProc(uint16_t length, uint8_t *pData)
{
    otapCommand_t *pRemoteCmd = (otapCommand_t *)pData;
    otapCommand_t otapCommand;
    /*! Handle all OTAP Server to Client Commands Here. */
    switch (pRemoteCmd->cmdId)
    {
        case gOtapCmdIdNewImageNotification_c:
            OtapClient_ImageInfoRequest();
            s_otapClientData.lastCmdSentToOtapServer = gOtapCmdIdNewImageInfoRequest_c;
            break;

        case gOtapCmdIdNewImageInfoResponse_c:
            QPRINTF("gOtapCmdIdNewImageInfoResponse_c\r\n");
            /*! comment this for phone app not do version compare */
            /*! if((s_otapClientData.lastCmdSentToOtapServer == gOtapCmdIdNewImageInfoRequest_c)&&
                 (s_otapClientInterface.ImgInfoExchange(pRemoteCmd->cmd.newImgInfoRes.imageId,
               pRemoteCmd->cmd.newImgInfoRes.imageVersion,
                                                 s_otapClientData.currentImgId, s_otapClientData.currentImgVer)))*/
            {
                s_otapClientData.lastCmdSentToOtapServer = gOtapCmdIdNoCommand_c;

                memcpy(s_otapClientData.imgId, pRemoteCmd->cmd.newImgInfoRes.imageId, gOtap_ImageIdFieldSize_c);
                memcpy(s_otapClientData.imgVer, pRemoteCmd->cmd.newImgInfoRes.imageVersion,
                       gOtap_ImageVersionFieldSize_c);
                s_otapClientData.imgSize = pRemoteCmd->cmd.newImgInfoRes.imageFileSize;
                s_otapClientData.currentPos = 0;
                OtapClient_ResetBlockTransferParameters();

                /* Change the Client state to Downloading and trigger the download. */
                s_otapClientData.state = mOtapClientStateDownloadingImage_c;
                OtapClient_ContinueImageDownload();
            }
            /*! comment this for phone app not do version compare */
            /*!
            else
                QPRINTF("OTA client received old version image\r\n");
            */
            break;
        case gOtapCmdIdErrorNotification_c:
            OtapClient_HandleErrorNotification(length, pData);
            break;

        default:
            otapCommand.cmdId = gOtapCmdIdErrorNotification_c;
            otapCommand.cmd.errNotif.cmdId = pData[0];
            otapCommand.cmd.errNotif.errStatus = gOtapStatusUnexpectedCommand_c;

            QPRINTF("gOtapCmdIdErrorNotification_c\r\n");

            s_otapClientInterface.sendData(cmdIdToCmdLengthTable[gOtapCmdIdErrorNotification_c],
                                           (void *)(&otapCommand));
            break;
    };
}

static void OtapClient_Download(uint16_t length, uint8_t *pData)
{
    otapCommand_t *pRemoteCmd = (otapCommand_t *)pData;
    /*! Handle all OTAP Server to Client Commands Here. */
    switch (pRemoteCmd->cmdId)
    {
        case gOtapCmdIdImageChunk_c:
        {
            OtapClient_HandleDataChunk(length, pData);
            break;
        }
        case gOtapCmdIdNewImageNotification_c:
            OtapClient_ImageInfoRequest();
            s_otapClientData.lastCmdSentToOtapServer = gOtapCmdIdNewImageInfoRequest_c;
            break;

        case gOtapCmdIdNewImageInfoResponse_c:
        {
            if (s_otapClientData.lastCmdSentToOtapServer == gOtapCmdIdNewImageInfoRequest_c)
            {
                s_otapClientData.lastCmdSentToOtapServer = gOtapCmdIdNoCommand_c;
                /*! Check if the image is the one currently being downloaded and if it is continue the download,
                *  else if the image is newer than the current one being downloaded then restart the whole download
                * process. */
                if ((memcmp(s_otapClientData.imgId, pRemoteCmd->cmd.newImgNotif.imageId, gOtap_ImageIdFieldSize_c) ==
                     0) &&
                    (memcmp(s_otapClientData.imgVer, pRemoteCmd->cmd.newImgNotif.imageVersion,
                            gOtap_ImageVersionFieldSize_c) == 0))
                {
                    OtapClient_ResetBlockTransferParameters();
                    OtapClient_ContinueImageDownload();
                }
                else if (s_otapClientInterface.ImgInfoExchange(
                             pRemoteCmd->cmd.newImgNotif.imageId, pRemoteCmd->cmd.newImgNotif.imageVersion,
                             s_otapClientData.currentImgId, s_otapClientData.currentImgVer))
                {
                    /*! A newer image than the one being donloaded is available, restart the download with the new
                     * image. */
                    memcpy(s_otapClientData.imgId, pRemoteCmd->cmd.newImgNotif.imageId, gOtap_ImageIdFieldSize_c);
                    memcpy(s_otapClientData.imgVer, pRemoteCmd->cmd.newImgNotif.imageVersion,
                           gOtap_ImageVersionFieldSize_c);
                    s_otapClientData.imgSize = pRemoteCmd->cmd.newImgNotif.imageFileSize;
                    s_otapClientData.currentPos = 0;
                    OtapClient_ResetBlockTransferParameters();

                    OtapClient_ContinueImageDownload();
                }
            }
            break;
        }
    }
}

void OtapClient_InterfaceRegister(otapClientInterface_t *clientInterface)
{
    s_otapClientInterface.sendData = clientInterface->sendData;
    s_otapClientInterface.usrDataProc = clientInterface->usrDataProc;
    s_otapClientInterface.ImgInfoExchange = clientInterface->ImgInfoExchange;
    s_otapClientInterface.IsImageFileHeaderValid = clientInterface->IsImageFileHeaderValid;
    s_otapClientInterface.getFlashDstAddrByImageType = clientInterface->getFlashDstAddrByImageType;
    s_otapClientInterface.getMaxMtu = clientInterface->getMaxMtu;
    s_otapClientInterface.uploadImageComplete = clientInterface->uploadImageComplete;

    s_otapClientData.negotiatedMaxAttChunkSize =
        s_otapClientInterface.getMaxMtu() - gOtap_AttCommandMtuDataChunkOverhead_c;
}

void OtapClient_ImageInfoRequest(void)
{
    otapCommand_t otapCommand;

    otapCommand.cmdId = gOtapCmdIdNewImageInfoRequest_c;

    otapCommand.cmd.newImgInfoReq.currentImageId[0] = s_otapClientData.currentImgId[0];
    otapCommand.cmd.newImgInfoReq.currentImageId[1] = s_otapClientData.currentImgId[1];

    otapCommand.cmd.newImgInfoReq.currentImageVersion[0] = s_otapClientData.currentImgVer[0];
    otapCommand.cmd.newImgInfoReq.currentImageVersion[1] = s_otapClientData.currentImgVer[1];
    otapCommand.cmd.newImgInfoReq.currentImageVersion[2] = s_otapClientData.currentImgVer[2];
    otapCommand.cmd.newImgInfoReq.currentImageVersion[3] = s_otapClientData.currentImgVer[3];
    otapCommand.cmd.newImgInfoReq.currentImageVersion[4] = s_otapClientData.currentImgVer[4];
    otapCommand.cmd.newImgInfoReq.currentImageVersion[5] = s_otapClientData.currentImgVer[5];
    otapCommand.cmd.newImgInfoReq.currentImageVersion[6] = s_otapClientData.currentImgVer[6];
    otapCommand.cmd.newImgInfoReq.currentImageVersion[7] = s_otapClientData.currentImgVer[7];

    s_otapClientInterface.sendData(cmdIdToCmdLengthTable[gOtapCmdIdNewImageInfoRequest_c], (uint8_t *)&otapCommand);
}

void OtapClient_Stop(void)
{
    otapCommand_t otapCommand;

    otapCommand.cmdId = gOtapCmdIdStopImageTransfer_c;

    otapCommand.cmd.stopImgTransf.imageId[0] = s_otapClientData.imgId[0];
    otapCommand.cmd.stopImgTransf.imageId[1] = s_otapClientData.imgId[1];

    s_otapClientInterface.sendData(cmdIdToCmdLengthTable[gOtapCmdIdStopImageTransfer_c], (uint8_t *)&otapCommand);
}

void OtapClient_ContinueDownloadFromBreakPoint(void)
{
    if (s_otapClientData.currentPos != 0)
    {
        s_otapClientData.state = mOtapClientStateDownloadingImage_c;
        OtapClient_ImageInfoRequest();
        s_otapClientData.lastCmdSentToOtapServer = gOtapCmdIdNewImageInfoRequest_c;
    }
}

void OtapClient_Proc(uint16_t length, uint8_t *pData)
{
    switch (s_otapClientData.state)
    {
        case mOtapClientStateIdle_c:
        {
            OtapClient_IdleProc(length, pData);
            break;
        }

        case mOtapClientStateDownloadingImage_c:
        {
            OtapClient_Download(length, pData);
            break;
        }

        default:
            /*! Ignore. */
            break;
    }
}

status_t OtapClient_WriteImageToFlash(uint16_t len, const uint8_t *data)
{
    uint32_t addr = 0;
    uint32_t start_block;
    uint32_t stop_block;
    status_t flashStatus;

    addr = startAddrForOtap + writeFlashOffset;
    start_block = addr / FLASH_SECTOR_SIZE;
    stop_block = (addr + len) / FLASH_SECTOR_SIZE;

    if (writeFlashOffset == 0)
    {
        flashStatus = FLASH_PageErase(&g_flash_cfg, start_block);
    }

    for (uint32_t i = 0; i < (stop_block - start_block); i++)
    {
        flashStatus = FLASH_PageErase(&g_flash_cfg, start_block + i + 1);
    }
    flashStatus = FLASH_Program(&g_flash_cfg, addr + FLASH_START_ADDRESS, (uint32_t *)data, len);
    writeFlashOffset += len;

    return flashStatus;
}

bool OtapClient_DownloadImageCompleted(void)
{
    if (s_otapClientData.state == mOtapClientStateImageDownloadComplete_c)
        return true;
    else
        return false;
}

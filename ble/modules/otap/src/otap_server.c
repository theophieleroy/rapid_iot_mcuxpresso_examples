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
#include "otap_server.h"
#include "otap_interface.h"
#include "otap_support.h"

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*! OTAP Server data structure.
 *  Contains functional information, available images information and state information
 *  regarding the image download procedure to the OTAp Client. */
static otapServerAppData_t s_otapServerData[gOtap_ServerConnectionNum];

otapServerInterface_t s_otapServerInterface[gOtap_ServerConnectionNum];

const otapServerAppData_t s_otapServerInitData = {
    .images =
        {
            {
                .imgId = {0xFF, 0xFF}, .imgVer = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, .imgSize = 0,
            },
        },
    .transferMethod = gOtapTransferMethodAtt_c,
    .l2capChannelOrPsm = gL2capCidAtt_c,
    .sentInitialImgNotification = true,
    .lastCmdSentToOtapClient = gOtapCmdIdNoCommand_c,
    .negotiatedMaxAttChunkSize = gAttDefaultMtu_c - gOtap_AttCommandMtuDataChunkOverhead_c,
    .negotiatedMaxL2CapChunkSize = gOtap_ImageChunkDataSizeL2capCoc_c,
    .l2capPsmConnected = false,
    .l2capPsmChannelId = 0,
    .chunkStartPos = 0,
    .blockLen = 0,
    .chunkLen = 0,
    .chunkSeqNum = 0,
    .chunkSendEnable = 0,
};

/*******************************************************************************
 * Code
 ******************************************************************************/

static void OtapServer_SendImageInfo(uint16_t conidx, otapCmdIdt_t cmd)
{
    otapCommand_t otapCommand;

    otapCommand.cmdId = cmd;

    otapCommand.cmd.newImgNotif.imageId[0] = s_otapServerData[conidx].images[0].imgId[0];
    otapCommand.cmd.newImgNotif.imageId[1] = s_otapServerData[conidx].images[0].imgId[1];

    otapCommand.cmd.newImgNotif.imageVersion[0] = s_otapServerData[conidx].images[0].imgVer[0];
    otapCommand.cmd.newImgNotif.imageVersion[1] = s_otapServerData[conidx].images[0].imgVer[1];
    otapCommand.cmd.newImgNotif.imageVersion[2] = s_otapServerData[conidx].images[0].imgVer[2];
    otapCommand.cmd.newImgNotif.imageVersion[3] = s_otapServerData[conidx].images[0].imgVer[3];
    otapCommand.cmd.newImgNotif.imageVersion[4] = s_otapServerData[conidx].images[0].imgVer[4];
    otapCommand.cmd.newImgNotif.imageVersion[5] = s_otapServerData[conidx].images[0].imgVer[5];
    otapCommand.cmd.newImgNotif.imageVersion[6] = s_otapServerData[conidx].images[0].imgVer[6];
    otapCommand.cmd.newImgNotif.imageVersion[7] = s_otapServerData[conidx].images[0].imgVer[7];
    otapCommand.cmd.newImgNotif.imageFileSize = s_otapServerData[conidx].images[0].imgSize;

    s_otapServerInterface[conidx].sendData(cmdIdToCmdLengthTable[gOtapCmdIdNewImageNotification_c],
                                           (uint8_t *)&otapCommand);
    s_otapServerData[conidx].sentInitialImgNotification = true;
}
static void OtapServer_ImageInfoResponse(uint16_t conidx)
{
    OtapServer_SendImageInfo(conidx, gOtapCmdIdNewImageInfoResponse_c);
}

static void OtapServer_InterfaceRegister(uint16_t conidx, otapServerInterface_t *otapServerInterface)
{
    s_otapServerInterface[conidx].sendData = otapServerInterface->sendData;
    s_otapServerInterface[conidx].imageInfoProc = otapServerInterface->imageInfoProc;
    s_otapServerInterface[conidx].getMaxMtu = otapServerInterface->getMaxMtu;
}

void OtapServer_LoadImage(uint16_t conidx, uint32_t *file)
{
    bleOtaImageFileHeader_t *otaHeader;
    otaHeader = (bleOtaImageFileHeader_t *)file;

    memcpy(&s_otapServerData[conidx].images, ((uint8_t *)file) + offsetof(bleOtaImageFileHeader_t, imageId), 10);

    s_otapServerData[conidx].images[0].imgSize = otaHeader->totalImageFileSize;
    s_otapServerData[conidx].pOtaFile = file;
}

void OtapServer_init(uint16_t conidx, otapServerInterface_t *otapServerInterface)
{
    memcpy(&s_otapServerData[conidx], &s_otapServerInitData, sizeof(s_otapServerInitData));
    OtapServer_InterfaceRegister(conidx, otapServerInterface);

    s_otapServerData[conidx].negotiatedMaxAttChunkSize =
        s_otapServerInterface[conidx].getMaxMtu(conidx) - gOtap_AttCommandMtuDataChunkOverhead_c;
}

void OtapServer_SendNewImageNotify(uint16_t conidx)
{
    OtapServer_SendImageInfo(conidx, gOtapCmdIdNewImageNotification_c);
}

void OtapServer_Proc(uint16_t conidx, uint16_t length, uint8_t *pData)
{
    otapCommand_t *pRemoteCmd = (otapCommand_t *)pData;
    uint8_t cmdBuffer[s_otapServerData[conidx].negotiatedMaxAttChunkSize + gOtap_ChunkHeaderLen_c];
    uint16_t sendLen = 0;
    int32_t imgLeftSize = 0;


    /*! Handle all OTAP Server to Client Commands Here. */
    switch (pRemoteCmd->cmdId)
    {
        case gOtapCmdIdNewImageInfoRequest_c:
        {
            s_otapServerInterface[conidx].imageInfoProc(
                pRemoteCmd->cmd.newImgInfoReq.currentImageId, pRemoteCmd->cmd.newImgInfoReq.currentImageVersion,
                s_otapServerData[conidx].images[0].imgId, s_otapServerData[conidx].images[0].imgVer);
            OtapServer_ImageInfoResponse(conidx);
        }
        break;

        case gOtapCmdIdImageBlockRequest_c:
            imgLeftSize = s_otapServerData[conidx].images[0].imgSize - pRemoteCmd->cmd.imgBlockReq.startPosition;
            if (imgLeftSize > 0)
            {
                s_otapServerData[conidx].chunkSendEnable = 1;
                s_otapServerData[conidx].chunkStartPos = pRemoteCmd->cmd.imgBlockReq.startPosition;
                s_otapServerData[conidx].blockLen = pRemoteCmd->cmd.imgBlockReq.blockSize;
                s_otapServerData[conidx].chunkLen = pRemoteCmd->cmd.imgBlockReq.chunkSize;
                s_otapServerData[conidx].l2capPsmChannelId = pRemoteCmd->cmd.imgBlockReq.l2capChannelOrPsm;
                s_otapServerData[conidx].chunkSeqNum = 0;

                /*! build image chunk packet */
                cmdBuffer[0] = gOtapCmdIdImageChunk_c;
                cmdBuffer[1] = s_otapServerData[conidx].chunkSeqNum;
                sendLen =
                    imgLeftSize < s_otapServerData[conidx].chunkLen ? imgLeftSize : s_otapServerData[conidx].chunkLen;

                memcpy((uint8_t *)(cmdBuffer + 2),
                       ((uint8_t *)s_otapServerData[conidx].pOtaFile) + s_otapServerData[conidx].chunkStartPos +
                           s_otapServerData[conidx].chunkLen * s_otapServerData[conidx].chunkSeqNum,
                       sendLen);
                s_otapServerData[conidx].chunkSeqNum++;
                s_otapServerInterface[conidx].sendData(sendLen + gOtap_ChunkHeaderLen_c, cmdBuffer);
            }

            break;

        case gOtapCmdIdImageChunkRsp_c:
            imgLeftSize = s_otapServerData[conidx].images[0].imgSize -
                  (s_otapServerData[conidx].chunkStartPos +
                   s_otapServerData[conidx].chunkLen * s_otapServerData[conidx].chunkSeqNum);

            if (s_otapServerData[conidx].chunkSendEnable && (imgLeftSize > 0) &&
                (s_otapServerData[conidx].chunkLen * s_otapServerData[conidx].chunkSeqNum <
                 s_otapServerData[conidx].blockLen))
            {
                int otaPercent = 100 - imgLeftSize * 100 / s_otapServerData[conidx].images[0].imgSize;
                Otap_TransmitedImagePercent(otaPercent);
                cmdBuffer[0] = gOtapCmdIdImageChunk_c;
                cmdBuffer[1] = s_otapServerData[conidx].chunkSeqNum;
                sendLen =
                    imgLeftSize < s_otapServerData[conidx].chunkLen ? imgLeftSize : s_otapServerData[conidx].chunkLen;
                memcpy((uint8_t *)(cmdBuffer + gOtap_ChunkHeaderLen_c),
                       ((uint8_t *)s_otapServerData[conidx].pOtaFile) + s_otapServerData[conidx].chunkStartPos +
                           s_otapServerData[conidx].chunkLen * s_otapServerData[conidx].chunkSeqNum,
                       sendLen);
                s_otapServerData[conidx].chunkSeqNum++;
                s_otapServerInterface[conidx].sendData(sendLen + gOtap_ChunkHeaderLen_c, cmdBuffer);
            }
            break;

        case gOtapCmdIdErrorNotification_c:
        {
            s_otapServerData[conidx].chunkSendEnable = 0;
            QPRINTF("OTA server error cmd 0x%02x, code 0x%02x\r\n", pData[1], pData[2]);
            break;
        }

        case gOtapCmdIdImageTransferComplete_c:
            QPRINTF("\r\nOTA server completed\r\n");
            break;

        case gOtapCmdIdStopImageTransfer_c:
            s_otapServerData[conidx].chunkSendEnable = 0;
            QPRINTF("\r\nOTA server received client request stop\r\n");
            break;
    }
}

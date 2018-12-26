/*! *********************************************************************************
* \addtogroup BLE OTAP Client ATT
* @{
********************************************************************************** */
/*!
* Copyright (c) 2015, Freescale Semiconductor, Inc.
* Copyright 2016-2017 NXP
*
* \file
*
* This file is the source file for the BLE OTAP Client ATT application
*
* Redistribution and use in source and binary forms, with or without modification,
* are permitted provided that the following conditions are met:
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

/************************************************************************************
*************************************************************************************
* Include
*************************************************************************************
************************************************************************************/
/* Framework / Drivers */
#include "FunctionLib.h"
#include "Panic.h"
#include "l2ca_types.h"
#include "shell_gattdb.h"
#include "shell_gap.h"
#include "otap_interface.h"
#include "att_errors.h"
#include "ble_otap_client.h"

#include "otap_interface.h"


#include "flash_ica_driver.h"
#include "spi_flash_driver.h"
#include "app_program_ext.h"
#include "shell.h"
#include "TimersManager.h"
#include "fsl_os_abstraction.h"

#include "FsciInterface.h"

#include "ui_manager.h"
extern osaSemaphoreId_t gOtaSem;
/************************************************************************************
*************************************************************************************
* Extern functions
*************************************************************************************
************************************************************************************/
extern void ResetMCU(void);


/************************************************************************************
*************************************************************************************
* Private macros
*************************************************************************************
************************************************************************************/
#define gBootData_SectorsBitmap_Size_c     (32)
/************************************************************************************
*************************************************************************************
* Private type definitions
*************************************************************************************
************************************************************************************/
typedef enum otapClientState_tag
{
    mOtapClientStateIdle_c                  = 0x00,
    mOtapClientStateDownloadingImage_c      = 0x01,
    mOtapClientStateImageDownloadComplete_c = 0x02,
} otapClientState_t;

/*! Structure containing the OTAP Client functional data. */
typedef struct otapClientAppData_tag
{
    otapClientState_t           state;
    const uint8_t               currentImgId[gOtap_ImageIdFieldSize_c];         /*!< Id of the currently running image on the OTAP Client */
    const uint8_t               currentImgVer[gOtap_ImageVersionFieldSize_c];   /*!< Version of the currently running image on the OTAP Client */
    deviceId_t                  peerOtapServer;                                 /*!< Device id of the OTAP Server a new image is being downloaded from. */
    uint8_t                     imgId[gOtap_ImageIdFieldSize_c];                /*!< Id of the image being downloaded from the OTAP Server */
    uint8_t                     imgVer[gOtap_ImageVersionFieldSize_c];          /*!< Version of the image being downloaded from the OTAP Server */
    uint32_t                    imgSize;                                        /*!< Size of the image file being downloaded from the OTAP Server */
    uint16_t                    imgComputedCrc;                                 /*!< Computed 16 bit CRC of the image file used in this implementation. */
    uint16_t                    imgReceivedCrc;                                 /*!< Received 16 bit CRC of the image file used in this implementation. */
    uint8_t                     imgSectorBitmap[gBootData_SectorsBitmap_Size_c];/*!< Flash sector bitmap for the recieved image for the current implementation. */
    uint32_t                    currentPos;                                     /*!< Current position of the file being downloaded. */
    uint16_t                    chunkSize;                                      /*!< Current chunk size for the image file transfer. */
    uint16_t                    chunkSeqNum;                                    /*!< Current chunk sequence number for the block being transferred. */
    uint16_t                    totalBlockChunks;                               /*!< Total number of chunks for the block being transferred. */
    uint32_t                    totalBlockSize;                                 /*!< Total size of the block which was requested. may be smaller than totalBlockChunks * chunkSize. */
    const otapTransferMethod_t  transferMethod;                                 /*!< Currently used transfer method for the OTAP Image File */
    uint16_t                    l2capChannelOrPsm;                              /*!< L2CAP Channel or PSM used for the transfer of the image file: channel 0x0004 for ATT, application specific PSM for CoC. */
    bool_t                      serverWrittenCccd;                              /*!< The OTAP Server has written the CCCD to receive commands from the OTAp Client. */
    otapCmdIdt_t                lastCmdSentToOtapServer;                        /*!< The last command sent to the OTAP Server for which an Indication is expected. */
    uint16_t                    negotiatedMaxAttChunkSize;                      /*!< The negotiated maximum ATT chunk size based on the negotiated ATT MTU between the OTAP Server and the OTAP Client. */
    uint16_t                    negotiatedMaxL2CapChunkSize;                    /*!< The negotiated maximum L2CAP chunk size based on the negotiated L2CAP MTU between the OTAP Server and the OTAP Client. */
} otapClientAppData_t;

/************************************************************************************
*************************************************************************************
* Private memory declarations
*************************************************************************************
************************************************************************************/
static uint8_t fica_image_type;
static uint32_t fica_image_size;

//static deviceId_t  mPeerdeviceId = gInvalidDeviceId_c;
//static otapClientConfig_t otapServiceConfig;
//static uint16_t otapWriteNotifHandles[2];

//extern osaMutexId_t gExtMemMutex;

//extern uint16_t hServiceOTA;
extern uint16_t hValueControlPointOTA;
extern uint16_t hValueOTACCCD;
extern uint16_t hValueDataOTA;
extern uint16_t ota_demo;

/*! OTAP Client data structure.
 *  Contains current image information and state informations
 *  regarding the image download procedure. */
static otapClientAppData_t     otapClientData =
{
    .state = mOtapClientStateIdle_c,
    .currentImgId = {0x00, 0x00},     // Current Running Image Id - should be 0x0000
    .currentImgVer = {0x01, 0x00, 0x00,    // Build Version
                      0x41,                // Stack Version
                      0x11, 0x11, 0x11,    // Hardware Id
                      0x01                 // Manufacturer Id
                     },               // Current Image Version
    .peerOtapServer = gInvalidDeviceId_c,
    .imgId = {0x00, 0x00},
    .imgVer = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    .imgSize = 0,
    .imgComputedCrc = 0,
    .imgReceivedCrc = 0,
    .imgSectorBitmap = {0x00},
    .currentPos = 0,
    .chunkSize = 0,
    .chunkSeqNum = 0,
    .totalBlockChunks = 0,
    .totalBlockSize = 0,
    .transferMethod = gOtapTransferMethodAtt_c,   // The default transfer method is ATT
    .l2capChannelOrPsm = gL2capCidAtt_c,   // The default L2CAP channel is the ATT Channel
    .lastCmdSentToOtapServer = gOtapCmdIdNoCommand_c,
    .negotiatedMaxAttChunkSize = gAttDefaultMtu_c - gOtap_AttCommandMtuDataChunkOverhead_c,
    .negotiatedMaxL2CapChunkSize = gOtap_ImageChunkDataSizeL2capCoc_c,
};


/************************************************************************************
*************************************************************************************
* Private functions prototypes
*************************************************************************************
************************************************************************************/

/* Gatt and Att callbacks */
//static void BleApp_ConnectionCallback (deviceId_t peerdeviceId, gapConnectionEvent_t* pConnectionEvent);

/* OTAP Client functions */
/* Commands received from the OTAP Server */
static void OtapClient_HandleDataChunk (deviceId_t deviceId, uint16_t length, uint8_t* pData);
static void OtapClient_HandleNewImageNotification (void *param);
static void OtapClient_HandleNewImageInfoResponse (void *param);
static void OtapClient_HandleErrorNotification (void *param);
/* Confirmations of commands sent to the OTAP Server */
static void OtapClient_HandleNewImageInfoRequestConfirmation (deviceId_t deviceId);
static void OtapClient_HandleImageBlockRequestConfirmation (deviceId_t deviceId);
static void OtapClient_HandleImageTransferCompleteConfirmation (deviceId_t deviceId);
static void OtapClient_HandleErrorNotificationConfirmation (deviceId_t deviceId);
static void OtapClient_HandleStopImageTransferConfirmation (deviceId_t deviceId);
/* Otap Client operations */
static void OtapClient_ContinueImageDownload (deviceId_t deviceId);
static void OtapClient_ResetBlockTransferParameters (deviceId_t deviceId);
static bool_t OtapClient_IsRemoteImageNewer (uint8_t* pRemoteImgId, uint8_t* pRemoteImgVer);
static otapStatus_t OtapClient_IsImageFileHeaderValid (bleOtaImageFileHeader_t* imgFileHeader);

/************************************************************************************
*************************************************************************************
* Public functions
*************************************************************************************
************************************************************************************/
void BleApp_AttMtuChanged (deviceId_t deviceId, uint16_t negotiatedMtu)
{
//    shell_write("\n\rBleApp_AttMtuChanged\n\r");
    otapClientData.negotiatedMaxAttChunkSize = negotiatedMtu - gOtap_AttCommandMtuDataChunkOverhead_c;
}

void BleApp_CccdWritten_lazy()
{
    if (otapClientData.serverWrittenCccd)
        BleApp_CccdWritten (OtapCS_GetSubscribed(), hValueOTACCCD);
}

void BleApp_CccdWritten (deviceId_t deviceId, uint16_t handle)
{
    shell_write("\n\rBleApp_CccdWritten");

    otapCommand_t otapCommand;

    if (handle == hValueOTACCCD)
        otapClientData.serverWrittenCccd = TRUE;

    /*! Check if the OTAP control point CCCD was written. */
    if (handle == hValueOTACCCD && deviceId == OtapCS_GetSubscribed())
    {
        otapClientData.serverWrittenCccd = TRUE;
        // Stop the sensor reading
        ota_demo = 1;
        switch (otapClientData.state)
        {
        case mOtapClientStateDownloadingImage_c:
        case mOtapClientStateIdle_c:
            /*! If the state is Idle try to send a New Image Info Request Command to the OTAP Server. */
            otapCommand.cmdId = gOtapCmdIdNewImageInfoRequest_c;
            FLib_MemCpy (otapCommand.cmd.newImgInfoReq.currentImageId,
                         (uint8_t*)otapClientData.currentImgId,
                         gOtap_ImageIdFieldSize_c);
            FLib_MemCpy (otapCommand.cmd.newImgInfoReq.currentImageVersion,
                         (uint8_t*)otapClientData.currentImgVer,
                         gOtap_ImageVersionFieldSize_c);

            OtapCS_SendCommandToOtapServer (hValueControlPointOTA,
                                            (void*)(&otapCommand),
                                            cmdIdToCmdLengthTable[gOtapCmdIdNewImageInfoRequest_c]);
            otapClientData.lastCmdSentToOtapServer = gOtapCmdIdNewImageInfoRequest_c;

            break;

        case mOtapClientStateImageDownloadComplete_c:
            /*! Simply ignore the situation if the image download is complete. */
            break;

        default:
            /*! Ignore. */
            break;
        }
    }
}

static tmrTimerID_t     mDelayTimerID = gTmrInvalidTimerID_c;
typedef struct NewImageNotification_tag {
    uint8_t DeviceId;  // The device ID of the connected peer
    uint16_t ValueLength;  // Length of data to be notified
    uint8_t *Value;  // Data to be notified
} NewImageNotification_t;

void BleApp_AttributeWritten(deviceId_t  deviceId,
                                    uint16_t    handle,
                                    uint16_t    length,
                                    uint8_t*    pValue)
{
    otapCommand_t otapCommand;

#if DEBUG_BLE_OTA
    shell_write("\n\rBleApp_AttributeWritten");
#endif

    if (mDelayTimerID == gTmrInvalidTimerID_c)
    {
        mDelayTimerID = TMR_AllocateTimer();
    }

    /* Only the OTAP Control Point attribute is expected to be written using the
     * ATT Write Command. */
    if (handle == hValueControlPointOTA)
    {
        /*! Handle all OTAP Server to Client Commands Here. */
        NewImageNotification_t *newimagenotif = MEM_BufferAlloc(sizeof(NewImageNotification_t));
        if (!newimagenotif)
        {
            shell_write("\r\n-->  !!! Insufficient memory !!! ");
            return;
        }
        else
        {
            newimagenotif->DeviceId = deviceId;
            newimagenotif->Value = MEM_BufferAlloc(length);
            if (!newimagenotif->Value)
            {
                shell_write("\r\n-->  !!! Insufficient memory !!! ");
                return;
            }
            FLib_MemCpy(newimagenotif->Value, pValue, length);
            newimagenotif->ValueLength = length;
        }

        switch(((otapCommand_t*)pValue)->cmdId)
        {
        case gOtapCmdIdNewImageNotification_c:
            ShellGattDb_SendAttributeWrittenStatusRequest (deviceId,
                                                           hValueControlPointOTA,
                                                           gAttErrCodeNoError_c);

            TMR_StartSingleShotTimer(mDelayTimerID, 250, OtapClient_HandleNewImageNotification, newimagenotif);
            break;
        case gOtapCmdIdNewImageInfoResponse_c:
            ShellGattDb_SendAttributeWrittenStatusRequest (deviceId,
                                                           hValueControlPointOTA,
                                                           gAttErrCodeNoError_c);
            TMR_StartSingleShotTimer(mDelayTimerID, 25, OtapClient_HandleNewImageInfoResponse, newimagenotif);
            break;
        case gOtapCmdIdErrorNotification_c:
            ShellGattDb_SendAttributeWrittenStatusRequest (deviceId,
                                                           hValueControlPointOTA,
                                                           gAttErrCodeNoError_c);
            TMR_StartSingleShotTimer(mDelayTimerID, 25, OtapClient_HandleErrorNotification, newimagenotif);
            break;

        default:
#if DEBUG_BLE_OTA
            shell_write(" -> default -> gOtapCmdIdErrorNotification_c");
#endif
            otapCommand.cmdId = gOtapCmdIdErrorNotification_c;
            otapCommand.cmd.errNotif.cmdId = pValue[0];
            otapCommand.cmd.errNotif.errStatus = gOtapStatusUnexpectedCommand_c;

            OtapCS_SendCommandToOtapServer (hValueControlPointOTA,
                                                        (void*)(&otapCommand),
                                                        cmdIdToCmdLengthTable[gOtapCmdIdErrorNotification_c]);
            otapClientData.lastCmdSentToOtapServer = gOtapCmdIdErrorNotification_c;
            break;
        };
    }
    else
    {
        /*! A GATT Server is trying to GATT Write an unknown attribute value.
         *  This should not happen. Disconnect the link. */
        GAPDisconnectRequest_t req;
        req.DeviceId = deviceId;  // the connected peer to disconnect from
        GAPDisconnectRequest(&req, BLE_FSCI_IF);
    }
}

void BleApp_AttributeWrittenWithoutResponse (deviceId_t deviceId,
                                                    uint16_t handle,
                                                    uint16_t length,
                                                    uint8_t* pValue)
{
    otapCommand_t otapCommand;
    otapStatus_t otapStatus = gOtapStatusSuccess_c;

    /* Only the OTAP Data attribute is expected to be written using the
     * ATT Write Without Response Command. */
    if (handle == hValueDataOTA)
    {
        if (otapClientData.state == mOtapClientStateDownloadingImage_c)
        {
            if (otapClientData.transferMethod == gOtapTransferMethodAtt_c)
            {
                if (((otapCommand_t*)pValue)->cmdId == gOtapCmdIdImageChunk_c)
                {
                    OtapClient_HandleDataChunk (deviceId,
                                                length,
                                                pValue);
                }
                else
                {
                    /* If the OTAP Client received an unexpected command on the data channel send an error to the OTAP Server. */
                    otapStatus = gOtapStatusUnexpectedCmdOnDataChannel_c;
                    shell_write("\r\ngOtapStatusUnexpectedCmdOnDataChannel_c");
                }
            }
            else
            {
                /* If the OTAP Client is not expecting image file chunks via ATT send an error to the OTAP Server. */
                otapStatus = gOtapStatusUnexpectedTransferMethod_c;
                shell_write("\r\ngOtapStatusUnexpectedTransferMethod_c");
            }
        }
        else
        {
            /* If the OTAP Client is not expecting image file chunks send an error to the OTAP Server. */
            otapStatus = gOtapStatusImageDataNotExpected_c;
            shell_write("\r\ngOtapStatusImageDataNotExpected_c");
        }

        if (otapStatus != gOtapStatusSuccess_c)
        {
            otapCommand.cmdId = gOtapCmdIdErrorNotification_c;
            otapCommand.cmd.errNotif.cmdId = pValue[0];
            otapCommand.cmd.errNotif.errStatus = otapStatus;

            OtapCS_SendCommandToOtapServer (hValueControlPointOTA,
                                            (void*)(&otapCommand),
                                            cmdIdToCmdLengthTable[gOtapCmdIdErrorNotification_c]);
            otapClientData.lastCmdSentToOtapServer = gOtapCmdIdErrorNotification_c;
        }
    }
}

void BleApp_HandleValueConfirmation (deviceId_t deviceId)
{
    otapCommand_t otapCommand;

    /*! Check for which command sent to the OTAP Server the confirmation has been received. */
    switch (otapClientData.lastCmdSentToOtapServer)
    {
    case gOtapCmdIdNewImageInfoRequest_c:
//        shell_write("lastCmdSentToOtapServer:gOtapCmdIdNewImageInfoRequest_c");
        OtapClient_HandleNewImageInfoRequestConfirmation (deviceId);
        break;

    case gOtapCmdIdImageBlockRequest_c:
//        shell_write("lastCmdSentToOtapServer:gOtapCmdIdImageBlockRequest_c");
        OtapClient_HandleImageBlockRequestConfirmation (deviceId);
        break;

    case gOtapCmdIdImageTransferComplete_c:
//        shell_write("lastCmdSentToOtapServer:gOtapCmdIdImageTransferComplete_c");
        OtapClient_HandleImageTransferCompleteConfirmation (deviceId);
        break;

    case gOtapCmdIdErrorNotification_c:
//        shell_write("lastCmdSentToOtapServer:gOtapCmdIdErrorNotification_c");
        OtapClient_HandleErrorNotificationConfirmation (deviceId);
        break;

    case gOtapCmdIdStopImageTransfer_c:
//        shell_write("lastCmdSentToOtapServer:gOtapCmdIdStopImageTransfer_c");
        OtapClient_HandleStopImageTransferConfirmation (deviceId);
        break;

    default:
        otapCommand.cmdId = gOtapCmdIdErrorNotification_c;
        otapCommand.cmd.errNotif.cmdId = gOtapCmdIdNoCommand_c;
        otapCommand.cmd.errNotif.errStatus = gOtapStatusUnexpectedCommand_c;

        OtapCS_SendCommandToOtapServer (hValueControlPointOTA,
                                                    (void*)(&otapCommand),
                                                    cmdIdToCmdLengthTable[gOtapCmdIdErrorNotification_c]);
        otapClientData.lastCmdSentToOtapServer = gOtapCmdIdErrorNotification_c;
        break;
    };
}
/************************************************************************************
*************************************************************************************
* Private functions
*************************************************************************************
************************************************************************************/
static void OtapClient_HandleDataChunk (deviceId_t deviceId, uint16_t length, uint8_t* pData)
{
    otapCommand_t otapCommand;
    otapStatus_t otapStatus = gOtapStatusSuccess_c;

    otapCmdImgChunkCoc_t* pDataChunk = (otapCmdImgChunkCoc_t*)(&((otapCommand_t*)pData)->cmd); //use the CoC Data Chunk type but observe the length
    uint16_t dataLen = length - gOtap_CmdIdFieldSize_c - gOtap_ChunkSeqNumberSize_c; // len

    /* Variables for the local image file parsing state machine. */
    static uint32_t currentImgElemRcvdLen = 0;      /*!< Contains the number of received bytes for the current image element (header or the sub element) */
    static bleOtaImageFileHeader_t imgFileHeader;   /*!< Saved image file header. */
    static uint32_t elementEnd = 0;                 /*!< Current image file element expected end. */
    static subElementHeader_t subElemHdr;
    static int update_percentage = 0;

    if (deviceId == otapClientData.peerOtapServer)
    {
        /* Check if the command length is as expected. */
        if ((length > (gOtap_CmdIdFieldSize_c + gOtap_ChunkSeqNumberSize_c)) &&
            (((otapClientData.transferMethod == gOtapTransferMethodAtt_c) && (length <= gOtapCmdImageChunkAttMaxLength_c)) ||
             ((otapClientData.transferMethod == gOtapTransferMethodL2capCoC_c) && (length <= gOtapCmdImageChunkCocMaxLength_c))
            )
           )
        {
            /* Check if the chunk (sequence number) is as expected */
            if (((pDataChunk->seqNumber == otapClientData.chunkSeqNum) &&
                (pDataChunk->seqNumber < otapClientData.totalBlockChunks)))
            {
                /*  Check if the data length is as expected. */
                if (((dataLen == otapClientData.chunkSize) && ((pDataChunk->seqNumber < (otapClientData.totalBlockChunks - 1)) || (otapClientData.totalBlockSize % otapClientData.chunkSize == 0))) ||
                    ((dataLen < otapClientData.chunkSize) && (pDataChunk->seqNumber == (otapClientData.totalBlockChunks - 1)) && (dataLen == otapClientData.totalBlockSize % otapClientData.chunkSize))
                   )
                {
                    /* Do more checks here if necessary. */
                }
                else
                {
                    otapStatus = gOtapStatusUnexpectedDataLength_c;
                    shell_write("\r\ngOtapStatusUnexpectedDataLength_c");
                }
            }
            else
            {
                otapStatus = gOtapStatusUnexpectedSequenceNumber_c;
//                shell_write("\r\ngOtapStatusUnexpectedSequenceNumber_c");
            }
        }
        else
        {
            otapStatus = gOtapStatusInvalidCommandLength_c;
            shell_write("\r\ngOtapStatusInvalidCommandLength_c");
        }
    }
    else
    {
        otapStatus = gOtapStatusUnexpectedOtapPeer_c;
        shell_write("\r\ngOtapStatusUnexpectedOtapPeer_c");
    }

    /*! If all checks were successful then parse the current data chunk, else send an error notification. */
    if (otapStatus == gOtapStatusSuccess_c)
    {
        pData = (uint8_t*)(&pDataChunk->data);

        /* If the Current position is 0 then reset the received length for the current image element
         * and the current image CRC to the initialization value which is 0.
         * The current position should be 0 only at the start of the image file transfer. */
        if (otapClientData.currentPos == 0)
        {
            currentImgElemRcvdLen = 0;
            otapClientData.imgComputedCrc = 0;
        }

        /* Parse all the bytes in the data payload. */
        while (dataLen)
        {
            /* Wait for the header to arrive and check it's contents
             * then handle the elements of the image. */
            if (otapClientData.currentPos < sizeof(bleOtaImageFileHeader_t))
            {
                if ((otapClientData.currentPos + dataLen) >= sizeof(bleOtaImageFileHeader_t))
                {
                    uint16_t residualHeaderLen = sizeof(bleOtaImageFileHeader_t) - otapClientData.currentPos;

                    /* There is enough information in the data payload to complete the header. */
                    FLib_MemCpy ((uint8_t*)(&imgFileHeader) + otapClientData.currentPos, pData, residualHeaderLen);
                    otapClientData.currentPos += residualHeaderLen;
                    pData += residualHeaderLen;
                    dataLen -= residualHeaderLen;

                    /* Check header contents, and if it is not valid return and error and reset the image download position. */
                    otapStatus = OtapClient_IsImageFileHeaderValid (&imgFileHeader);
                    if (otapStatus != gOtapStatusSuccess_c)
                    {
                        otapClientData.currentPos = 0;
                        break;
                    }

                    /* If the header is valid then update the CRC over the header part of the image. */
                    otapClientData.imgComputedCrc = FICA_compute_chunk_CRC ((uint8_t*)(&imgFileHeader),
                                                                            sizeof(bleOtaImageFileHeader_t),
                                                                            otapClientData.imgComputedCrc);

                    currentImgElemRcvdLen = 0;

                    /* If the remaining data length is not 0 then the loop will continue with the parsing of the next element. */
                }
                else
                {
                    /* Not enough data to complete the header.
                     * Copy all the data into the temporary header and
                     * increment the current image position. */
                    FLib_MemCpy((uint8_t*)(&imgFileHeader) + otapClientData.currentPos, pData, dataLen);
                    otapClientData.currentPos += dataLen;
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
                        FLib_MemCpy ((uint8_t*)(&subElemHdr) + currentImgElemRcvdLen, pData, residualSubElemHdrLen);
                        otapClientData.currentPos += residualSubElemHdrLen;
                        currentImgElemRcvdLen += residualSubElemHdrLen;
                        pData += residualSubElemHdrLen;
                        dataLen -= residualSubElemHdrLen;

                        /* Update the CRC over the sub-element header only if it is not the CRC Sub-Element header. */
                        if (subElemHdr.tagId != gBleOtaSubElemTagIdImageFileCrc_c)
                        {
                            otapClientData.imgComputedCrc = FICA_compute_chunk_CRC ((uint8_t*)(&subElemHdr),
                                                                                    sizeof(subElementHeader_t),
                                                                                    otapClientData.imgComputedCrc);
                        }

                        elementEnd = otapClientData.currentPos + subElemHdr.dataLen;

                        /* If the remaining data length is not 0 then the loop will
                        continue with the parsing of the sub-element. */
                    }
                    else
                    {
                        /* Not enough data to complete the sub-element header.
                         * Copy all the data into the temporary sub-element header
                         * and increment the current image position. */
                        FLib_MemCpy ((uint8_t*)(&subElemHdr) + currentImgElemRcvdLen, pData, dataLen);
                        otapClientData.currentPos += dataLen;
                        currentImgElemRcvdLen += dataLen;
                        dataLen = 0;
                    }
                }
                else
                {
                    uint32_t    elementChunkLength = 0;

                    /* Make sure we do not pass the current element boundary. */
                    if ((otapClientData.currentPos + dataLen) >= elementEnd)
                    {
                        elementChunkLength = elementEnd - otapClientData.currentPos;
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
                        if (currentImgElemRcvdLen == sizeof(subElementHeader_t))
                        {
                            fica_image_size = subElemHdr.dataLen;
#if DEBUG_BLE_OTA
                            char temp[100] = {0};
                            char temp2[100] = {0};
                            memcpy(temp, "Initiate BLE OTA process -> Update image size is ", sizeof("Initiate BLE OTA process -> Update image size is ") - 1);
                            itoa(subElemHdr.dataLen, temp2, 10);
                            memcpy(temp + strlen(temp), temp2, strlen(temp2));
                            shell_write("\r\n");
                            shell_write(temp);
#endif
                            /* Set event to lock UI during OTA process */
                            UiManager_SetEvent(kUiLock);

                            if (SPI_FLASH_NO_ERROR != app_program_ext_init(fica_image_type))
                            {
                                /* The sub-element length is invalid, set an error status and reset
                                 * the image file download process. */
                                otapStatus = gOtapStatusImageSizeTooLarge_c;
                                otapClientData.currentPos = 0;

                                break;
                            }
                        }

                        /* Upgrade Image Tag - compute the CRC and try to push the chunk to the storage. */
                        otapClientData.imgComputedCrc = FICA_compute_chunk_CRC (pData,
                                                                                elementChunkLength,
                                                                                otapClientData.imgComputedCrc);

                        if (SPI_FLASH_NO_ERROR != app_program_ext_cont(pData, elementChunkLength))
                        {
                            otapStatus = gOtapStatusImageStorageError_c;
                            otapClientData.currentPos = 0;
                            shell_write("\r\napp_program_ext_cont gOtapStatusImageStorageError_c !!!");

                            break;
                        }
                        break;

                    case gBleOtaSubElemTagIdSectorBitmap_c:
                        /* Immediately after receiving the header check if the sub-element length is valid. */
                        if (currentImgElemRcvdLen == sizeof(subElementHeader_t))
                        {
                            if (subElemHdr.dataLen != sizeof(otapClientData.imgSectorBitmap))
                            {
                                /* The sub-element length is invalid, set an error status and reset
                                 * the image file download process. */
                                otapStatus = gOtapStatusInvalidSubElementLength_c;
                                otapClientData.currentPos = 0;
                                shell_write("\r\ngBleOtaSubElemTagIdSectorBitmap_c gOtapStatusInvalidSubElementLength_c !!!");

                                break;
                            }
                        }

                        /* Sector Bitmap Tag - Compute the CRC and copy the received bitmap to the buffer. */
                        otapClientData.imgComputedCrc = FICA_compute_chunk_CRC (pData,
                                                                                elementChunkLength,
                                                                                otapClientData.imgComputedCrc);

                        FLib_MemCpy ((uint8_t*)otapClientData.imgSectorBitmap + (currentImgElemRcvdLen - sizeof(subElementHeader_t)),
                                     pData,
                                     elementChunkLength);
                        break;

                    case gBleOtaSubElemTagIdImageFileCrc_c:
                        /* Immediately after receiving the header check if the sub-element length is valid. */
                        if (currentImgElemRcvdLen == sizeof(subElementHeader_t))
                        {
                            if (subElemHdr.dataLen != sizeof(otapClientData.imgReceivedCrc))
                            {
                                /* The sub-element length is invalid, set an error status and reset
                                 * the image file download process. */
                                otapStatus = gOtapStatusInvalidSubElementLength_c;
                                otapClientData.currentPos = 0;
                                shell_write("\r\ngBleOtaSubElemTagIdImageFileCrc_c gOtapStatusInvalidSubElementLength_c !!!");
                                break;
                            }
                        }

                        /* CRC Tag - Just copy the received CRC to the buffer. */
                        FLib_MemCpy ((uint8_t*)(&otapClientData.imgReceivedCrc) + (currentImgElemRcvdLen - sizeof(subElementHeader_t)),
                                     pData,
                                     elementChunkLength);
                        break;

                    default:
                        /* Unknown sub-element type, just compute the CRC over it. */
                        otapClientData.imgComputedCrc = FICA_compute_chunk_CRC (pData,
                                                                                elementChunkLength,
                                                                                otapClientData.imgComputedCrc);
                        break;
                    };

                    if (otapStatus != gOtapStatusSuccess_c)
                    {
                        /* If an error has occurred then break the loop. */
                        break;
                    }

                    otapClientData.currentPos += elementChunkLength;
                    currentImgElemRcvdLen += elementChunkLength;
                    pData += elementChunkLength;
                    dataLen -= elementChunkLength;

#if DEBUG_BLE_OTA
                    //otapClientData.currentPos >= otapClientData.imgSize
                    if((otapClientData.currentPos - otapClientData.chunkSize) == 0)
                        shell_write("\r\n");
                    int current_percentage = otapClientData.currentPos * 100 / otapClientData.imgSize;
                    if (current_percentage != update_percentage)
                    {
                        shell_printf("%d%% ", current_percentage);
                        update_percentage = current_percentage;

                        if(!(update_percentage % 10))
                            shell_write("\r\n");
                    }
#endif

                    /* If this element has been completely received then reset the current element
                     * received length to trigger the reception of the next sub-element. */
                    if (otapClientData.currentPos >= elementEnd)
                    {
                        currentImgElemRcvdLen = 0;
                    }
                }
            }
        } /* while (dataLen) */
    }

    if (otapStatus == gOtapStatusSuccess_c)
    {
        /* If the chunk has been successfully processed increase the expected sequence number. */
        otapClientData.chunkSeqNum += 1;

        /* Check if the block and/or image transfer is complete */
        if (otapClientData.chunkSeqNum >= otapClientData.totalBlockChunks)
        {
            /* If the image transfer is complete check the image CRC then
             * commit the image and set the bootloader flags. */
            if (otapClientData.currentPos >= otapClientData.imgSize)
            {
                uint32_t flash_image_crc = 0;
                if (otapClientData.imgComputedCrc != otapClientData.imgReceivedCrc)
                {
                    shell_write("\r\nCRC mismatch after image transmission !!!\r\n");
                    otapStatus = gOtapStatusInvalidImageCrc_c;
                    otapClientData.currentPos = 0;
                }
                else if (IMG_EXT_NO_ERROR != app_program_ext_flush())
                {
                    otapStatus = gOtapStatusImageStorageError_c;
                    otapClientData.currentPos = 0;
                }
                /* Calculate flash image CRC */
                else if (IMG_EXT_NO_ERROR !=
                                     app_program_ext_get_crc (fica_image_type,
                                                              &flash_image_crc))
                {
                    otapStatus = gOtapStatusInvalidImageCrc_c;
                    otapClientData.currentPos = 0;
                }
                /* Write image CRC on flash */
                else if (IMG_EXT_NO_ERROR !=
                                    app_program_ext_program_crc(fica_image_type,
                                                                flash_image_crc))
                {
                    otapStatus = gOtapStatusImageStorageError_c;
                    otapClientData.currentPos = 0;
                }
                else
                {
                    /* The new image was successfully committed, set the bootloader new image flags,
                     * set the image transfer state as downloaded and send an image transfer complete
                     * message to the peer. */
                    otapClientData.state = mOtapClientStateImageDownloadComplete_c;

                    otapCommand.cmdId = gOtapCmdIdImageTransferComplete_c;
                    FLib_MemCpy((uint8_t*)otapCommand.cmd.imgTransComplete.imageId, otapClientData.imgId, sizeof(otapCommand.cmd.imgTransComplete.imageId));
                    otapCommand.cmd.imgTransComplete.status = gOtapStatusSuccess_c;

                    OtapCS_SendCommandToOtapServer (hValueControlPointOTA,
                                                    (void*)(&otapCommand),
                                                    cmdIdToCmdLengthTable[gOtapCmdIdImageTransferComplete_c]);

                    otapClientData.lastCmdSentToOtapServer = gOtapCmdIdErrorNotification_c;
                    shell_write("\r\nOTA process completed!");

                    /* Signal UI manager it can unlock */
                    OSA_SemaphorePost(gOtaSem);
                }
            }
            else
            {
                /* If just the current block is complete ask for another block. */
                OtapClient_ContinueImageDownload (deviceId);
            }
        }
    }

    if (otapStatus != gOtapStatusSuccess_c)
    {
        otapCommand.cmdId = gOtapCmdIdErrorNotification_c;
        otapCommand.cmd.errNotif.cmdId = gOtapCmdIdImageChunk_c;
        otapCommand.cmd.errNotif.errStatus = otapStatus;

        OtapCS_SendCommandToOtapServer (hValueControlPointOTA,
                                        (void*)(&otapCommand),
                                        cmdIdToCmdLengthTable[gOtapCmdIdErrorNotification_c]);
        otapClientData.lastCmdSentToOtapServer = gOtapCmdIdErrorNotification_c;
    }
}

static void OtapClient_HandleNewImageNotification (void *param)
{
    NewImageNotification_t *newimage = param;
    deviceId_t deviceId = newimage->DeviceId;
    uint16_t length = newimage->ValueLength;
    uint8_t* pValue = newimage->Value;

    otapCommand_t otapCommand;
    otapStatus_t otapStatus = gOtapStatusSuccess_c;
    otapCommand_t*  pRemoteCmd = (otapCommand_t*)pValue;

#if DEBUG_BLE_OTA
    shell_write("\r\ngOtapCmdIdNewImageNotification_c");
#endif

    /* Check the command length and parameters. */
    if (length != cmdIdToCmdLengthTable[gOtapCmdIdNewImageNotification_c])
    {
        otapStatus = gOtapStatusInvalidCommandLength_c;
        shell_write("-> gOtapStatusInvalidCommandLength_c");
    }
    else if (pRemoteCmd->cmd.newImgNotif.imageFileSize <= (sizeof(bleOtaImageFileHeader_t) + sizeof(subElementHeader_t)))
    {
        otapStatus = gOtapStatusInvalidImageFileSize_c;
        shell_write("-> gOtapStatusInvalidImageFileSize_c");
    }
    else
    {
        switch (otapClientData.state)
        {
        case mOtapClientStateIdle_c:
            if (OtapClient_IsRemoteImageNewer(pRemoteCmd->cmd.newImgNotif.imageId, pRemoteCmd->cmd.newImgNotif.imageVersion))
            {
                /* If a response for a New Image Info Request is expected from the OTAP Server simply ignore the
                 * New Image Notification. */
                if (otapClientData.lastCmdSentToOtapServer != gOtapCmdIdNewImageInfoRequest_c)
                {
                    /* Set up the Client to receive the image file. */
                    otapClientData.peerOtapServer = deviceId;
                    FLib_MemCpy(otapClientData.imgId, pRemoteCmd->cmd.newImgNotif.imageId, gOtap_ImageIdFieldSize_c);
                    FLib_MemCpy(otapClientData.imgVer, pRemoteCmd->cmd.newImgNotif.imageVersion, gOtap_ImageVersionFieldSize_c);
                    otapClientData.imgSize = pRemoteCmd->cmd.newImgNotif.imageFileSize;
                    otapClientData.currentPos = 0;
                    OtapClient_ResetBlockTransferParameters (deviceId);

                    /* Change the Client state to Downloading and trigger the download. */
                    otapClientData.state = mOtapClientStateDownloadingImage_c;
                    OtapClient_ContinueImageDownload (deviceId);
                }
                else
                {
                    shell_write("-> ignore the notification");
                }
            }
            break;

        case mOtapClientStateDownloadingImage_c:
            /*! Check if the image is the one currently being downloaded and if it is continue the download,
             *  else if the image is newer than the current one being downloaded then restart the whole download process. */
            if ((FLib_MemCmp(otapClientData.imgId, pRemoteCmd->cmd.newImgNotif.imageId, gOtap_ImageIdFieldSize_c)) &&
                (FLib_MemCmp(otapClientData.imgVer, pRemoteCmd->cmd.newImgNotif.imageVersion, gOtap_ImageVersionFieldSize_c))
               )
            {
                OtapClient_ResetBlockTransferParameters (deviceId);
                OtapClient_ContinueImageDownload (deviceId);
            }
            else if (OtapClient_IsRemoteImageNewer(pRemoteCmd->cmd.newImgNotif.imageId, pRemoteCmd->cmd.newImgNotif.imageVersion))
            {
                /*! A newer image than the one being downloaded is available, restart the download with the new image. */
                otapClientData.peerOtapServer = deviceId;
                FLib_MemCpy(otapClientData.imgId, pRemoteCmd->cmd.newImgNotif.imageId, gOtap_ImageIdFieldSize_c);
                FLib_MemCpy(otapClientData.imgVer, pRemoteCmd->cmd.newImgNotif.imageVersion, gOtap_ImageVersionFieldSize_c);
                otapClientData.imgSize = pRemoteCmd->cmd.newImgNotif.imageFileSize;
                otapClientData.currentPos = 0;
                OtapClient_ResetBlockTransferParameters (deviceId);

                OtapClient_ContinueImageDownload (deviceId);
            }
            break;
        case mOtapClientStateImageDownloadComplete_c:
            /* Simply ignore the message if an image is being downloaded or
             * an image download is complete. */
            break;

        default:
            /* Some kind of internal error has occurred. Reset the
             * client state to Idle and act as if the state was Idle. */
            otapClientData.state = mOtapClientStateIdle_c;
            if (OtapClient_IsRemoteImageNewer(pRemoteCmd->cmd.newImgNotif.imageId, pRemoteCmd->cmd.newImgNotif.imageVersion))
            {
                /* If a response for a New Image Info Request is expected from the OTAp Server simply ignore the
                 * New Image Notification. */
                if (otapClientData.lastCmdSentToOtapServer != gOtapCmdIdNewImageInfoRequest_c)
                {
                    /* Set up the Client to receive the image file. */
                    otapClientData.peerOtapServer = deviceId;
                    FLib_MemCpy(otapClientData.imgId, pRemoteCmd->cmd.newImgNotif.imageId, gOtap_ImageIdFieldSize_c);
                    FLib_MemCpy(otapClientData.imgVer, pRemoteCmd->cmd.newImgNotif.imageVersion, gOtap_ImageVersionFieldSize_c);
                    otapClientData.imgSize = pRemoteCmd->cmd.newImgNotif.imageFileSize;
                    otapClientData.currentPos = 0;

                    shell_write("-> internal error ...");

                    OtapClient_ResetBlockTransferParameters (deviceId);

                    /* Change the Client state to Downloading and trigger the download. */
                    otapClientData.state = mOtapClientStateDownloadingImage_c;
                    OtapClient_ContinueImageDownload (deviceId);
                }
            }
            break;
        };
    }

    if (otapStatus != gOtapStatusSuccess_c)
    {
        otapCommand.cmdId = gOtapCmdIdErrorNotification_c;
        otapCommand.cmd.errNotif.cmdId = pValue[0];
        otapCommand.cmd.errNotif.errStatus = otapStatus;

        OtapCS_SendCommandToOtapServer (hValueControlPointOTA,
                                        (void*)(&otapCommand),
                                        cmdIdToCmdLengthTable[gOtapCmdIdErrorNotification_c]);
        otapClientData.lastCmdSentToOtapServer = gOtapCmdIdErrorNotification_c;
    }

    MEM_BufferFree(newimage->Value);
    MEM_BufferFree(newimage);
}

static void OtapClient_HandleNewImageInfoResponse (void *param)
{
    NewImageNotification_t *newimage = param;
    deviceId_t deviceId = newimage->DeviceId;
    uint16_t length = newimage->ValueLength;
    uint8_t* pValue = newimage->Value;

    otapCommand_t otapCommand;
    otapStatus_t otapStatus = gOtapStatusSuccess_c;
    otapCommand_t*  pRemoteCmd = (otapCommand_t*)pValue;

#if DEBUG_BLE_OTA
            shell_write("\r\ngOtapCmdIdNewImageInfoResponse_c");
#endif

    /* Check the command length and parameters. */
    if (length != cmdIdToCmdLengthTable[gOtapCmdIdNewImageInfoResponse_c])
    {
        otapStatus = gOtapStatusInvalidCommandLength_c;
        shell_write("-> gOtapStatusInvalidCommandLength_c");
    }
    else if (pRemoteCmd->cmd.newImgInfoRes.imageFileSize <= (sizeof(bleOtaImageFileHeader_t) + sizeof(subElementHeader_t)))
    {
        otapStatus = gOtapStatusInvalidImageFileSize_c;
        shell_write("-> Status:gOtapStatusInvalidImageFileSize_c");
    }
    else
    {
        switch (otapClientData.state)
        {
        case mOtapClientStateIdle_c:
            if (OtapClient_IsRemoteImageNewer(pRemoteCmd->cmd.newImgInfoRes.imageId, pRemoteCmd->cmd.newImgInfoRes.imageVersion))
            {
                /* Set up the Client to receive the image file. */
                otapClientData.peerOtapServer = deviceId;
                FLib_MemCpy(otapClientData.imgId, pRemoteCmd->cmd.newImgInfoRes.imageId, gOtap_ImageIdFieldSize_c);
                FLib_MemCpy(otapClientData.imgVer, pRemoteCmd->cmd.newImgInfoRes.imageVersion, gOtap_ImageVersionFieldSize_c);
                otapClientData.imgSize = pRemoteCmd->cmd.newImgInfoRes.imageFileSize;
                otapClientData.currentPos = 0;
                OtapClient_ResetBlockTransferParameters (deviceId);

                /* Change the Client state to Downloading and trigger the download. */
                otapClientData.state = mOtapClientStateDownloadingImage_c;
                OtapClient_ContinueImageDownload (deviceId);
            }
            break;

        case mOtapClientStateDownloadingImage_c:
            /*! Check if the image is the one currently being downloaded and if it is continue the download,
             *  else if the image is newer than the current one being downloaded then restart the whole download process. */
            if ((FLib_MemCmp(otapClientData.imgId, pRemoteCmd->cmd.newImgNotif.imageId, gOtap_ImageIdFieldSize_c)) &&
                (FLib_MemCmp(otapClientData.imgVer, pRemoteCmd->cmd.newImgNotif.imageVersion, gOtap_ImageVersionFieldSize_c))
               )
            {
                OtapClient_ResetBlockTransferParameters (deviceId);
                OtapClient_ContinueImageDownload (deviceId);
            }
            else if (OtapClient_IsRemoteImageNewer(pRemoteCmd->cmd.newImgNotif.imageId, pRemoteCmd->cmd.newImgNotif.imageVersion))
            {
                /*! A newer image than the one being downloaded is available, restart the download with the new image. */
                otapClientData.peerOtapServer = deviceId;
                FLib_MemCpy(otapClientData.imgId, pRemoteCmd->cmd.newImgNotif.imageId, gOtap_ImageIdFieldSize_c);
                FLib_MemCpy(otapClientData.imgVer, pRemoteCmd->cmd.newImgNotif.imageVersion, gOtap_ImageVersionFieldSize_c);
                otapClientData.imgSize = pRemoteCmd->cmd.newImgNotif.imageFileSize;
                otapClientData.currentPos = 0;
                OtapClient_ResetBlockTransferParameters (deviceId);

                OtapClient_ContinueImageDownload (deviceId);
            }
            break;

        case mOtapClientStateImageDownloadComplete_c:
            /* Simply ignore the message if an image is being downloaded or
             * an image download is complete. */
            break;

        default:
            /* Some kind of internal error has occurred. Reset the
             * client state to Idle and act as if the state was Idle. */
            otapClientData.state = mOtapClientStateIdle_c;
            if (OtapClient_IsRemoteImageNewer(pRemoteCmd->cmd.newImgInfoRes.imageId, pRemoteCmd->cmd.newImgInfoRes.imageVersion))
            {
                /* Set up the Client to receive the image file. */
                otapClientData.peerOtapServer = deviceId;
                FLib_MemCpy(otapClientData.imgId, pRemoteCmd->cmd.newImgInfoRes.imageId, gOtap_ImageIdFieldSize_c);
                FLib_MemCpy(otapClientData.imgVer, pRemoteCmd->cmd.newImgInfoRes.imageVersion, gOtap_ImageVersionFieldSize_c);
                otapClientData.imgSize = pRemoteCmd->cmd.newImgInfoRes.imageFileSize;
                otapClientData.currentPos = 0;
                shell_write("-> internal error ...");
                OtapClient_ResetBlockTransferParameters (deviceId);

                /* Change the Client state to Downloading and trigger the download. */
                otapClientData.state = mOtapClientStateDownloadingImage_c;
                OtapClient_ContinueImageDownload (deviceId);
            }
            /* If the remote image is not newer than the current image simply ignore the New Image Info Response */
            break;
        };
    }

    if (otapStatus != gOtapStatusSuccess_c)
    {
        otapCommand.cmdId = gOtapCmdIdErrorNotification_c;
        otapCommand.cmd.errNotif.cmdId = gOtapCmdIdNewImageInfoResponse_c;
        otapCommand.cmd.errNotif.errStatus = otapStatus;

        OtapCS_SendCommandToOtapServer (hValueControlPointOTA,
                                        (void*)(&otapCommand),
                                        cmdIdToCmdLengthTable[gOtapCmdIdErrorNotification_c]);

        otapClientData.lastCmdSentToOtapServer = gOtapCmdIdErrorNotification_c;
    }

    MEM_BufferFree(newimage->Value);
    MEM_BufferFree(newimage);
}

static void OtapClient_HandleErrorNotification (void *param)
{
    NewImageNotification_t *newimage = param;
    deviceId_t deviceId = newimage->DeviceId;
    uint16_t length = newimage->ValueLength;
    uint8_t* pValue = newimage->Value;

    otapCommand_t otapCommand;
    otapStatus_t otapStatus = gOtapStatusSuccess_c;
    otapCommand_t*  pRemoteCmd = (otapCommand_t*)pValue;

#if DEBUG_BLE_OTA
            shell_write(" -> gOtapCmdIdErrorNotification_c");
#endif

    /* Check the command length and parameters. */
    if (length == cmdIdToCmdLengthTable[gOtapCmdIdErrorNotification_c])
    {
        /*! Handle remote error statuses here. */
        if (pRemoteCmd->cmd.errNotif.errStatus < gOtapNumberOfStatuses_c)
        {
            /* Handle all errors in the same way, disconnect to restart the download process. */
            GAPDisconnectRequest_t req;
            req.DeviceId = deviceId;
            GAPDisconnectRequest(&req, BLE_FSCI_IF);
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

        OtapCS_SendCommandToOtapServer (hValueControlPointOTA,
                                        (void*)(&otapCommand),
                                        cmdIdToCmdLengthTable[gOtapCmdIdErrorNotification_c]);
        otapClientData.lastCmdSentToOtapServer = gOtapCmdIdErrorNotification_c;
    }

    MEM_BufferFree(newimage->Value);
    MEM_BufferFree(newimage);
}

static void OtapClient_HandleNewImageInfoRequestConfirmation (deviceId_t deviceId)
{
    /* Clear the last command sent to the OTAP Server for which a Confirmation is expected. */
    otapClientData.lastCmdSentToOtapServer = gOtapCmdIdNoCommand_c;

    /* Nothing more to do here. If the New Image Info Request Command has reached
     * the OTAP Server then the OTAP Client expects a New Image Info Response */
}

static void OtapClient_HandleImageBlockRequestConfirmation (deviceId_t deviceId)
{
    /* Clear the last command sent to the OTAP Server for which a Confirmation is expected. */
    otapClientData.lastCmdSentToOtapServer = gOtapCmdIdNoCommand_c;

    /* Nothing more to do here. If the Image Block Request Command has reached
     * the OTAP Server then the OTAP Client expects the requested image chunks
     * or an Error Notification. */
}

static void OtapClient_HandleImageTransferCompleteConfirmation (deviceId_t deviceId)
{
    otapCommand_t otapCommand;

    /* Clear the last command sent to the OTAP Server for which a Confirmation is expected. */
    otapClientData.lastCmdSentToOtapServer = gOtapCmdIdNoCommand_c;

    /* If the image transfer was not successful then the image download state should be Idle.
     * If it is, try to trigger a new download.
     * If the Image Transfer Complete Command has reached the OTAP Server and the transfer was succesful
     * then the OTAP Client will just wait for the restart and the
     * bootloader to flash the new image. */
    if (otapClientData.state == mOtapClientStateIdle_c)
    {
        otapCommand.cmdId = gOtapCmdIdNewImageInfoRequest_c;
        FLib_MemCpy (otapCommand.cmd.newImgInfoReq.currentImageId,
                     (uint8_t*)otapClientData.currentImgId,
                     gOtap_ImageIdFieldSize_c);
        FLib_MemCpy (otapCommand.cmd.newImgInfoReq.currentImageVersion,
                     (uint8_t*)otapClientData.currentImgVer,
                     gOtap_ImageVersionFieldSize_c);

        OtapCS_SendCommandToOtapServer (hValueControlPointOTA,
                                        (void*)(&otapCommand),
                                        cmdIdToCmdLengthTable[gOtapCmdIdNewImageInfoRequest_c]);
        otapClientData.lastCmdSentToOtapServer = gOtapCmdIdNewImageInfoRequest_c;
    }
    else if (otapClientData.state == mOtapClientStateImageDownloadComplete_c)
    {
        /* If the image transfer is complete trigger the bootloader and reset the device. */
        GAPDisconnectRequest_t req;
        req.DeviceId = deviceId;
        shell_write("\r\n OtapClient_HandleImageTransferCompleteConfirmation !! \r\n");
        GAPDisconnectRequest(&req, BLE_FSCI_IF);
//        ResetMCU ();
    }
}

static void OtapClient_HandleErrorNotificationConfirmation (deviceId_t deviceId)
{
    /* Clear the last command sent to the OTAP Server for which a Confirmation is expected. */
    otapClientData.lastCmdSentToOtapServer = gOtapCmdIdNoCommand_c;

    /* Reset block download parameters to safe values. */
    OtapClient_ResetBlockTransferParameters (deviceId);

    /* If an error has occured try to continue the image download from a "safe" point. */
    OtapClient_ContinueImageDownload (deviceId);
}

static void OtapClient_HandleStopImageTransferConfirmation (deviceId_t deviceId)
{
    /* Clear the last command sent to the OTAP Server for which a Confirmation is expected. */
    otapClientData.lastCmdSentToOtapServer = gOtapCmdIdNoCommand_c;

    /* Reset block download parameters to safe values. */
    OtapClient_ResetBlockTransferParameters (deviceId);

    /* If an error has occured try to continue the image download from a "safe" point. */
    OtapClient_ContinueImageDownload (deviceId);
}

void OtapClient_HandleConnectionEvent (deviceId_t deviceId)
{
#if gAppUseBonding_d
    bool_t isBonded = FALSE;

    if ((gBleSuccess_c == Gap_CheckIfBonded (mPeerdeviceId, &isBonded)) &&
        (TRUE == isBonded))
    {
        otapClientData.serverWrittenCccd = TRUE;
        ota_demo = 1;
    }
#endif



    GAPDisconnectRequest_t req;
    switch (otapClientData.state)
    {
    case mOtapClientStateIdle_c:
//        otapClientData.serverWrittenCccd = TRUE;

        /*! If the OTAP Server has written the CCCD to receive commands fromt he OTAp Client then send a
         *  new image info request. */
        if (otapClientData.serverWrittenCccd == TRUE)
        {
            otapCommand_t otapCommand;

            otapCommand.cmdId = gOtapCmdIdNewImageInfoRequest_c;
            FLib_MemCpy (otapCommand.cmd.newImgInfoReq.currentImageId,
                         (uint8_t*)otapClientData.currentImgId,
                         gOtap_ImageIdFieldSize_c);
            FLib_MemCpy (otapCommand.cmd.newImgInfoReq.currentImageVersion,
                         (uint8_t*)otapClientData.currentImgVer,
                         gOtap_ImageVersionFieldSize_c);

            OtapCS_SendCommandToOtapServer (hValueControlPointOTA,
                                            (void*)(&otapCommand),
                                            cmdIdToCmdLengthTable[gOtapCmdIdNewImageInfoRequest_c]);

            otapClientData.lastCmdSentToOtapServer = gOtapCmdIdNewImageInfoRequest_c;
            otapClientData.serverWrittenCccd = TRUE;
        }
        break;
    case  mOtapClientStateDownloadingImage_c:
        /*! If the state is Downloading try to continue the download from where it was left off.
         *  Check if the appropriate server is connected first. */
        if (otapClientData.peerOtapServer == deviceId)
        {
            /* Reset block download parameters to safe values. */
            OtapClient_ResetBlockTransferParameters (deviceId);

            if (otapClientData.serverWrittenCccd == TRUE)
            {
                OtapClient_ContinueImageDownload (deviceId);
            }
        }
        break;
    case mOtapClientStateImageDownloadComplete_c:
        req.DeviceId = deviceId;
        GAPDisconnectRequest(&req, BLE_FSCI_IF);
        ResetMCU ();
        break;
    default:
        /* Some kind of internal error has occurred. Reset the
         * client state to Idle and act as if the state was Idle. */
        otapClientData.state = mOtapClientStateIdle_c;
        break;
    };
}

void OtapClient_HandleDisconnectionEvent (deviceId_t deviceId)
{
    /* Check if the peer OTAP server was disconnected and if so reset block download
     * parameters to safe values. */
    if (otapClientData.peerOtapServer == deviceId)
    {
        OtapClient_ResetBlockTransferParameters (deviceId);
    }

    otapClientData.serverWrittenCccd = FALSE;
    ota_demo = 0;

    OtapCS_Unsubscribe();
}

static void OtapClient_ContinueImageDownload (deviceId_t deviceId)
{
    otapCommand_t   otapCommand;
    uint32_t        bytesToDownload;
    uint32_t        maxBlockSize;
    GAPDisconnectRequest_t req;
    /* If the Server did not write the CCCD and the image is being downloaded exit immediately.
     * No commands can be exchanged before the CCCD is written. */
    if ((otapClientData.serverWrittenCccd == FALSE) &&
        (otapClientData.state == mOtapClientStateDownloadingImage_c))
    {
        shell_write("\r\nOtapClient_ContinueImageDownload checks failed ...");
        return;
    }

    switch (otapClientData.state)
    {
    case mOtapClientStateIdle_c:
        /* If the state is Idle do nothing. No need to continue the transfer of an image. */
        break;
    case mOtapClientStateDownloadingImage_c:
        /* If the last received chunk sequence number is equal to the total block
         * chunks or they are both zero then ask for a new block from the server. */
        if (otapClientData.chunkSeqNum == otapClientData.totalBlockChunks)
        {
            /* Ask for another block only if the image transfer was not completed. */
            if (otapClientData.currentPos < otapClientData.imgSize)
            {
                bytesToDownload = otapClientData.imgSize - otapClientData.currentPos;

                if (otapClientData.transferMethod == gOtapTransferMethodAtt_c)
                {
                    maxBlockSize = otapClientData.negotiatedMaxAttChunkSize * gOtap_MaxChunksPerBlock_c;
                    otapClientData.l2capChannelOrPsm = gL2capCidAtt_c;
                    otapClientData.chunkSize = otapClientData.negotiatedMaxAttChunkSize;
                }
                else if (otapClientData.transferMethod == gOtapTransferMethodL2capCoC_c)
                {
                    if (otapClientData.l2capChannelOrPsm == gOtap_L2capLePsm_c)
                    {
                        maxBlockSize = otapClientData.negotiatedMaxL2CapChunkSize * gOtap_MaxChunksPerBlock_c;
                        otapClientData.chunkSize = otapClientData.negotiatedMaxL2CapChunkSize;
                    }
                    else
                    {
                        /* If the L2CAP CoC is not valid then some kind of error or missconfiguration has
                         * occurred. Send a proper error notification to the peer and
                         * reset the download state machine to Idle. */
                        otapClientData.state = mOtapClientStateIdle_c;

                        otapCommand.cmdId = gOtapCmdIdErrorNotification_c;
                        otapCommand.cmd.errNotif.cmdId = gOtapCmdIdNoCommand_c;
                        otapCommand.cmd.errNotif.errStatus = gOtapStatusUnexpectedL2capChannelOrPsm_c;

                        OtapCS_SendCommandToOtapServer (hValueControlPointOTA,
                                                        (void*)(&otapCommand),
                                                        cmdIdToCmdLengthTable[gOtapCmdIdErrorNotification_c]);
                        otapClientData.lastCmdSentToOtapServer = gOtapCmdIdErrorNotification_c;

                        return;
                    }
                }
                else
                {
                    /* If the transfer method is not recognized then this image has been missconfigured
                     * or a critical error has occurred. Send a proper error notification to the peer and
                     * reset the download state machien to Idle. */
                    otapClientData.state = mOtapClientStateIdle_c;

                    otapCommand.cmdId = gOtapCmdIdErrorNotification_c;
                    otapCommand.cmd.errNotif.cmdId = gOtapCmdIdNoCommand_c;
                    otapCommand.cmd.errNotif.errStatus = gOtapStatusUnexpectedTransferMethod_c;

                    OtapCS_SendCommandToOtapServer (hValueControlPointOTA,
                                                    (void*)(&otapCommand),
                                                    cmdIdToCmdLengthTable[gOtapCmdIdErrorNotification_c]);
                    otapClientData.lastCmdSentToOtapServer = gOtapCmdIdErrorNotification_c;

                    return;
                }

                if (bytesToDownload >= maxBlockSize)
                {
                    /* If there are more bytes to download than the maximum block size then
                     * ask a full block from the server on the selected tansfer method and set up
                     * the client to recieve the chunks.*/
                    otapClientData.chunkSeqNum = 0;
                    otapClientData.totalBlockChunks = gOtap_MaxChunksPerBlock_c;
                    otapClientData.totalBlockSize = maxBlockSize;
                }
                else
                {
                    /* If there are fewer bytes to download than the maximum block size then compute the
                    *  number of chunks expected and set the expected block size to the number of
                    *  bytes to download. */
                    otapClientData.chunkSeqNum = 0;
                    /* Compute number of full chunks. Integer division. */
                    otapClientData.totalBlockChunks = bytesToDownload / otapClientData.chunkSize;
                    /* Add an extra chunk if the chunk size is not a divisor of the number of bytes to download. */
                    otapClientData.totalBlockChunks += (bytesToDownload % otapClientData.chunkSize) ? 1 : 0;
                    otapClientData.totalBlockSize = bytesToDownload;
                }

                /* Send the Block request Command with the determined parameters. */
                otapCommand.cmdId = gOtapCmdIdImageBlockRequest_c;

                FLib_MemCpy(otapCommand.cmd.imgBlockReq.imageId, otapClientData.imgId, gOtap_ImageIdFieldSize_c);
                otapCommand.cmd.imgBlockReq.startPosition = otapClientData.currentPos;
                otapCommand.cmd.imgBlockReq.blockSize = otapClientData.totalBlockSize;
                otapCommand.cmd.imgBlockReq.chunkSize = otapClientData.chunkSize;
                otapCommand.cmd.imgBlockReq.transferMethod = otapClientData.transferMethod;
                otapCommand.cmd.imgBlockReq.l2capChannelOrPsm = otapClientData.l2capChannelOrPsm;

                OtapCS_SendCommandToOtapServer (hValueControlPointOTA,
                                                (void*)(&otapCommand),
                                                cmdIdToCmdLengthTable[gOtapCmdIdImageBlockRequest_c]);
                otapClientData.lastCmdSentToOtapServer = gOtapCmdIdImageBlockRequest_c;
            }
        }
        break;
    case mOtapClientStateImageDownloadComplete_c:
        req.DeviceId = deviceId;
        GAPDisconnectRequest(&req, BLE_FSCI_IF);

        /*Reset the proper MCU here*/
        if (fica_image_type == FICA_IMG_TYPE_K41Z_USER)
        {
            FSCI_transmitPayload(0xCE, 0xD2, "reset", 5, 0);
        }
        else
        {
            ResetMCU ();
        }
        break;
    default:
        /* This code should never be reached in normal running conditions.
        Do nothing here, no need to continue the transfer of an image. */
        break;
    };
}

static void OtapClient_ResetBlockTransferParameters (deviceId_t deviceId)
{
    otapClientData.chunkSize = 0;
    otapClientData.chunkSeqNum = 0;
    otapClientData.totalBlockChunks = 0;
    otapClientData.totalBlockSize = 0;
#if DEBUG_BLE_OTA
//    shell_write("\r\nOtapClient_ResetBlockTransferParameters");
#endif
}



static bool_t OtapClient_IsRemoteImageNewer (uint8_t* pRemoteImgId, uint8_t* pRemoteImgVer)
{
    uint32_t    remoteBuildVer;
    uint32_t    localeBuildVer;
    /* Ignore the Image Id for the moment. */
    /* Check the Manufacturer Id */
    if (pRemoteImgVer[7] != otapClientData.currentImgVer[7])
    {
        return FALSE;
    }

    /* Check Hardware Id */
    if (!FLib_MemCmp((void*)(&(pRemoteImgVer[4])), (void*)(&(otapClientData.currentImgVer[4])), 3))
    {
        return FALSE;
    }

    /* Check Stack Version */
    if (pRemoteImgVer[3] < otapClientData.currentImgVer[3])
    {
        return FALSE;
    }

    /* Check Build Version */
    remoteBuildVer = (uint32_t)pRemoteImgVer[0] + ((uint32_t)(pRemoteImgVer[1]) << 8) + ((uint32_t)(pRemoteImgVer[2]) << 16);
    localeBuildVer = (uint32_t)otapClientData.currentImgVer[0] + ((uint32_t)(otapClientData.currentImgVer[1]) << 8) + ((uint32_t)(otapClientData.currentImgVer[2]) << 16);
    if (remoteBuildVer <= localeBuildVer)
    {
        return FALSE;
    }

    return TRUE;
}

static otapStatus_t OtapClient_IsImageFileHeaderValid (bleOtaImageFileHeader_t* imgFileHeader)
{
    shell_write("\r\nValidating OTA header ");
    if (imgFileHeader->fileIdentifier != gBleOtaFileHeaderIdentifier_c)
    {
        return gOtapStatusUnknownFileIdentifier_c;
    }

    if (imgFileHeader->headerVersion != gbleOtapHeaderVersion0100_c)
    {
        return gOtapStatusUnknownHeaderVersion_c;
    }

    if (imgFileHeader->headerLength != sizeof(bleOtaImageFileHeader_t))
    {
        return gOtapStatusUnexpectedHeaderLength_c;
    }

    if (imgFileHeader->fieldControl != gBleOtaFileHeaderDefaultFieldControl_c)
    {
        return gOtapStatusUnexpectedHeaderFieldControl_c;
    }

    if (imgFileHeader->companyId != gBleOtaCompanyIdentifier_c)
    {
        return gOtapStatusUnknownCompanyId_c;
    }

    if (FALSE == FLib_MemCmp (imgFileHeader->imageId, otapClientData.imgId, sizeof(imgFileHeader->imageId)))
    {
        return gOtapStatusUnexpectedImageId_c;
    }

    if (FALSE == FLib_MemCmp (imgFileHeader->imageVersion, otapClientData.imgVer, sizeof(imgFileHeader->imageVersion)))
    {
        return gOtapStatusUnexpectedImageVersion_c;
    }

    if (imgFileHeader->totalImageFileSize != otapClientData.imgSize)
    {
        return gOtapStatusUnexpectedImageFileSize_c;
    }

    if (FLib_MemCmp((uint8_t *)imgFileHeader->headerString, gBleOtaHeaderStringK41, sizeof(gBleOtaHeaderStringK41)))
//    if (1)
    {
        shell_write("-> Detected KW41Z update image");
        fica_image_type = FICA_IMG_TYPE_K41Z_USER;
    }
    else
    if (FLib_MemCmp((uint8_t *)imgFileHeader->headerString, gBleOtaHeaderStringK64, sizeof(gBleOtaHeaderStringK64)))
//    if(1)
    {
        shell_write("-> Detected K64 update image");
        fica_image_type = FICA_IMG_TYPE_K64F_USER;
    }
    else
    {
        shell_write("-> Unexpected image ID, stopping OTA process!");
        return gOtapStatusUnexpectedImageId_c;
    }

    return gOtapStatusSuccess_c;
}

/*! *********************************************************************************
* @}
********************************************************************************** */

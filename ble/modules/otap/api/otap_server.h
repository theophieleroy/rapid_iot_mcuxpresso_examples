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

#ifndef _OTA_SERVER_H_
#define _OTA_SERVER_H_

/*!
 * @addtogroup otapServer_API
 * @{
 */

#include "otap_interface.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*! Structure containing the OTAP Server functional data. */
typedef struct otapServerAppData_tag
{
    imgInfo_t                       images[1];                      /*!< Array of images available on this OTAP Server. Only 1 image is supported at thsi time. */
    otapTransferMethod_t            transferMethod;                 /*!< OTAP Image File ransfer method requested by the OTAP Client. */
    uint16_t                        l2capChannelOrPsm;              /*!< L2CAP Channel or PSM used for the transfer of the image file: channel 0x0004 for ATT, application specific PSM for CoC. */
    bool                            sentInitialImgNotification;     /*!< Boolean flag which is set if an Image Notification is sent ot the OTAP Client on the first connection. */
    otapCmdIdt_t                    lastCmdSentToOtapClient;        /*!< The last command sent to the OTAP Client for which a Write Response is expected. */
    uint16_t                        negotiatedMaxAttChunkSize;      /*!< The negotiated maximum ATT chunk size based on the negotiated ATT MTU between the OTAP Server and the OTAP Client. */
    uint16_t                        negotiatedMaxL2CapChunkSize;    /*!< The negotiated maximum L2CAP chunk size based on the negotiated L2CAP MTU between the OTAP Server and the OTAP Client. */
    bool                            l2capPsmConnected;              /*!< Flag which is set to true if an L2CAP PSM connection is currently established with a peer device. */
    uint16_t                        l2capPsmChannelId;              /*!< Channel Id for an L2CAP PSM connection currently established with a peer device. */
    uint32_t*                       pOtaFile;
    uint32_t                        chunkStartPos;
    uint32_t                        blockLen;
    uint32_t                        chunkLen;
    uint16_t                        chunkSeqNum;
    uint8_t                         chunkSendEnable;
} otapServerAppData_t;

typedef struct otapServerInterface_tag
{
    void (*sendData)(uint16_t len, uint8_t *data);
    void (*imageInfoProc)(uint8_t* pRemoteImgId, uint8_t* pRemoteImgVer, 
                          const uint8_t* pCurrentImgId, const uint8_t* pcurrentImgVer);
    uint16_t (*getMaxMtu)(uint16_t conidx);
}otapServerInterface_t;

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
 * @brief OTA server load image file.
 *
 * @param[in] conidx    connection idex
 * @param[in] file          the start memory contains image to be update to client
 *
 * @return
 * @description
 *
 * OTA server load image file.
 */
 void OtapServer_LoadImage(uint16_t conidx, uint32_t *file);


/*!
 * @brief OTA server send new image information notity to client.
 *
 * @param[in] conidx    connection idex
*
 * @description
 *
 * OTA server send new image information notity to client.
 */
void OtapServer_SendNewImageNotify(uint16_t conidx);

/*!
 * @brief OTA server process received data.
 *
 * @param[in] conidx        connection index
 * @param[in] length        received data length
 * @param[in] pData         Pointer to received data
 *
 * @return
 * @description
 *
 * This function process iap server received data.
 */

void OtapServer_Proc(uint16_t conidx, uint16_t length, uint8_t* pData);
/*!
 * @brief Otap server initiation.
 *
 * @param[in] conidx        connection index
 * @param[in] otapServerInterface       otap interface
 *
 * @return
 * @description
 *
 * This function initiate otap server enviroment and do interface registration on connection index.
 */
void OtapServer_init(uint16_t conidx, otapServerInterface_t *otapServerInterface);
#ifdef __cplusplus
}
#endif

/*! @brief @} otapServer_API */
#endif /* _OTA_SERVER_H_ */


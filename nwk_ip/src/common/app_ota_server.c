/*
 * Copyright (c) 2014 - 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
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
 * o Neither the name of the copyright holder nor the names of its
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

/*!=================================================================================================
\file        app_ota_server.c
\brief      This is a public source file for the OTA server module
==================================================================================================*/

/*==================================================================================================
Include Files
==================================================================================================*/
/* Framework */
#include "Panic.h"
#include "Keyboard.h"
#include "Flash_Adapter.h"
#include "TimersManager.h"

/* Application */
#include "ip_sockets.h"
#include "session.h"
#include "app_stack_config.h"
#include "app_ota.h"
#include "OtaSupport.h"
#include "coap.h"
#include "thread_utils.h"

/*==================================================================================================
Private macros
==================================================================================================*/
#ifndef OTA_SERVER_DEFAULT_PORT
    #define OTA_SERVER_DEFAULT_PORT (61630)
#endif

/* The new firmware image is placed in server internal or external flash */
#ifndef gOtaServer_EnableStandaloneMode_c
    #define gOtaServer_EnableStandaloneMode_c            TRUE
#endif

/* Thci interface used for ota server */
#ifndef gOtaServer_ThciInteface_c
    #define gOtaServer_ThciInteface_c                    0x00
#endif

#ifndef gOtaServer_ThciMaxRetryNo_c
    #define gOtaServer_ThciMaxRetryNo_c                  10
#endif

#define gOtaServer_MinDelayForEndRequestMs_c             20000
#define gOtaServer_MaxDelayForEndRequestMs_c             40000
#define gOtaServer_TempAbortListSize_c                   0x06
#define gOtaServer_InvalidClientId_c                     THR_ALL_FFs16
#define gOtaServer_MaxOtaImages_c                        0x01
#define gOtaServer_MaxSimultaneousClients_c              0x0A

#define gOtaServer_CloseThciSessionTimeoutMs_c           1000    /* 1 seconds */
#define gOtaServer_DelayForNextRequestMs_c               60000   /* 60 seconds */
#define gOtaServer_ClientSessionExpirationMs_c           30000   /* 30 seconds */


#if gOtaServer_EnableStandaloneMode_c
    #define gOtaServer_DefaultOperationMode_c       gOtaServerOpMode_Standalone_c
#else
    #define gOtaServer_DefaultOperationMode_c       gOtaServerOpMode_Dongle_c
#endif

#define gOtaFileVersionPolicies_Upgrade_c       (1<<0)
#define gOtaFileVersionPolicies_Reinstall_c     (1<<1)
#define gOtaFileVersionPolicies_Downgrade_c     (1<<2)

#define gOtaFileVersionDefaultPolicies_c         gOtaFileVersionPolicies_Upgrade_c | \
                                                 gOtaFileVersionPolicies_Reinstall_c | \
                                                 gOtaFileVersionPolicies_Downgrade_c

#define gOtaServer_ThciBlockRequestRetryNo_c      5
#define gOtaServer_ThciBlockRequestTimeoutMs_c    1000
/*==================================================================================================
Private type definitions
==================================================================================================*/
typedef struct otaServerSetup_tag
{
    uint8_t           coapInstanceId;
    otaServerOpMode_t opMode;
    tmrTimerID_t      otaServerTmr;
    bool_t            isActive;
    uint8_t           fileVersionPolicy;
    int32_t           mOtaUdpSrvSockFd;
    uint16_t          downloadPort;
} otaServerSetup_t;

typedef struct otaServerImageList_tag
{
    uint16_t manufCode;
    uint16_t imageType;
    uint32_t fileSize;
    uint32_t imageAddr;
    uint32_t fileVersion;
    bool_t isValidEntry;
} otaServerImageList_t;

typedef struct otaServerTempClientData_tag
{
    uint16_t          clientId;
    uint8_t           currentOffset[4];
    coapSession_t     *pSession;
    ipAddr_t          ipAddr;
    uint16_t          port;
} otaServerTempClientData_t;

typedef struct otaServerTempAbortList_tag
{
    uint16_t clientId;
} otaServerTempAbortList_t;

typedef struct otaClientInfo_tag
{
    ipAddr_t  remoteAddr;
    ipAddr_t  sourceAddr;
    uint16_t  port;
    uint64_t  timeStamp;
    uint32_t  dataLen;
    uint8_t   pData[1];
} otaClientInfo_t;

typedef struct otaClientSessionInfo_tag
{
    ipAddr_t  remoteAddr;
    uint64_t  timeStamp;
} otaClientSessionInfo_t;

/*==================================================================================================
Private function prototypes
==================================================================================================*/
/* OTA Server Coap callback */
static void OtaServer_CoapCb(coapSessionStatus_t sessionStatus, void *pData,
                             coapSession_t *pSession, uint32_t dataLen);

/* OTA Server Udp Socket Callback*/
static void OtaClient_UdpServerService(void *pInData);

/* OTA Server Coap command handlers */
static void OtaServer_ClientProcess(void *param);
static void OtaServer_CmdProcess(void *param);
static otaStatus_t OtaServer_ProcessNextClient(void);
static otaStatus_t OtaServer_CmdCheck(void *pData, uint32_t dataLen);
static otaStatus_t OtaServer_QueryImageReqHandler(otaClientInfo_t *pOtaClientInfo);
static otaStatus_t OtaServer_BlockReqHandler(otaClientInfo_t *pOtaClientInfo);
static otaStatus_t OtaServer_UpgradeEndReqHandler(otaClientInfo_t *pOtaClientInfo);
static otaStatus_t OtaServer_ServerDiscoveryHandler(otaClientInfo_t *pOtaClientInfo);

/* OTA Server Coap commands */
static otaStatus_t OtaServer_CoapSendImageNotify(thciOtaServer_ImageNotify_t *pThciImageNotify, ipAddr_t *pDestAddr);
static otaStatus_t OtaServer_CoapSendRsp(otaClientInfo_t *pOtaClientInfo, uint8_t *pData, uint32_t pDataLen);
static void OtaServer_CoapSendQueryImageRsp(void *pParam);
static otaStatus_t OtaServer_CoapSendRspWaitAbortData(otaClientInfo_t *pOtaClientInfo, uint8_t status,
        uint32_t delayInMs);

/* OTA Server utility functions */
static void OtaServer_CloseThciSessionTimeout(void *pParam);
static void OtaServer_CloseThciSessionTimeoutCb(void *pParam);
static otaStatus_t OtaServer_CheckThciQueue(otaClientInfo_t *pOtaClientInfo);
static bool_t OtaServer_IsClientValid(uint16_t clientId);
static bool_t OtaServer_AddClientToAbortList(uint16_t clientId);
static bool_t OtaServer_RemoveClientFromAbortList(uint16_t clientId);
static otaStatus_t OtaServer_CheckClientSessionTable(otaClientInfo_t *pOtaClientInfo);
static otaStatus_t OtaServer_HandleBlockSocket(bool_t onOff);

/* OTA Server dongle functions: */
static otaStatus_t OtaServerDongle_QueryImageReqHandler(otaClientInfo_t *pOtaClientInfo);
static otaStatus_t OtaServerDongle_BlockReqHandler(otaClientInfo_t *pOtaClientInfo);
static otaStatus_t OtaServerDongle_ServerDiscoveryHandler(otaClientInfo_t *pOtaClientInfo);
static void OtaServerDongle_SendBlockRsp(void *pParam);

#if gOtaServer_EnableStandaloneMode_c
    /* OTA Server standalone functions: */
    static void OtaServer_InitStandaloneOpMode(void);
    static uint8_t OtaServerStandalone_ValidateImage(uint16_t manufCode, uint16_t imageType, uint32_t fileVersion, bool_t serialProtocol);
    static uint8_t OtaServerStandalone_KeepImageInfo(uint16_t manufCode, uint16_t imageType, uint32_t fileVersion, uint32_t fileSize);
    static otaStatus_t OtaServerStandalone_QueryImageReqHandler(otaClientInfo_t *pOtaClientInfo);
    static otaStatus_t OtaServerStandalone_BlockReqHandler(otaClientInfo_t *pOtaClientInfo);
    static otaStatus_t OtaServerStandalone_ServerDiscoveryHandler(otaClientInfo_t *pOtaClientInfo);
    static void OtaServer_BlockRequestRetryCb(void *pParam);
#endif

/* OTA Server Serial Protocol callbacks */
static otaResult_t ThciOtaServer_ImageNotifyCnf(uint8_t *pBuffer, uint16_t len);
static otaResult_t ThciOtaServer_SetOperationMode(uint8_t *pBuffer, uint16_t len);
static otaResult_t ThciOtaServer_QueryImageRspCnf(uint8_t *pBuffer, uint16_t len);
static otaResult_t ThciOtaServer_BlockReceivedCnf(uint8_t *pBuffer, uint16_t len);
static otaResult_t ThciOtaServer_CancelImgCnf(uint8_t *pBuffer, uint16_t len);
static otaResult_t ThciOtaServer_AbortProcessCnf(uint8_t *pBuffer, uint16_t len);
static otaResult_t ThciOtaServer_SetFileVersPoliciesCnf(uint8_t *pBuffer, uint16_t len);
static otaResult_t ThciOtaServer_ClientInfoCnf(uint8_t *pBuffer, uint16_t len);

/*==================================================================================================
Private global variables declarations
==================================================================================================*/
/* Pointer to task message queue */
static taskMsgQueue_t *mpOtaServerMsgQueue = NULL;

/* Ota server setup parameters */
static otaServerSetup_t mOtaServerSetup = {.mOtaUdpSrvSockFd = -1,
                                           .otaServerTmr = gTmrInvalidTimerID_c,
                                           .downloadPort = OTA_SERVER_DEFAULT_PORT
                                          };

#if gOtaServer_EnableStandaloneMode_c
    /* Ota server standalone informations: */
    static otaServerImageList_t mOtaServerImageList[gOtaServer_MaxOtaImages_c];
    static uint32_t mOtaServerTempImageOffset = 0;
    static uint32_t mOtaServerTempImageIdx = gOtaServer_MaxOtaImages_c;
#endif

/* Ota server temporary client info data */
static otaClientInfo_t *mpOtaServerTempClientInfo = NULL;

/* Client requests queue - used in dongle mode */
static msgQueue_t mOtaServerThciClientInfoQueue;

/* Ota server temporary abort list */
static otaServerTempAbortList_t mOtaSeverTempAbortList[gOtaServer_TempAbortListSize_c];

/* Ota server serial protocol (callbacks) */
static otaServer_AppCB_t mThciOtaServerCb =
{
    ThciOtaServer_ImageNotifyCnf,
    ThciOtaServer_SetOperationMode,
    ThciOtaServer_QueryImageRspCnf,
    ThciOtaServer_BlockReceivedCnf,
    ThciOtaServer_CancelImgCnf,
    ThciOtaServer_AbortProcessCnf,
    ThciOtaServer_SetFileVersPoliciesCnf,
    ThciOtaServer_ClientInfoCnf
};

static otaClientSessionInfo_t mOtaClientSessionInfoTable[gOtaServer_MaxSimultaneousClients_c];
#if gOtaServer_EnableStandaloneMode_c
    static uint8_t mOtaServer_ThciBlockRequestRetryNo = gOtaServer_ThciBlockRequestRetryNo_c;
#endif

/*==================================================================================================
Public global variables declarations
==================================================================================================*/

/*==================================================================================================
Public functions
==================================================================================================*/
/*!*************************************************************************************************
\fn     otaStatus_t OtaServerInit(taskMsgQueue_t *pMsgQueue)
\brief  Initialize OTA server application.

\param  [in]    pMsgQueue      Pointer to task message queue

\return         otaStatus_t    Status of the operation
 ***************************************************************************************************/
otaStatus_t OtaServerInit
(
    taskMsgQueue_t *pMsgQueue
)
{
    otaStatus_t otaStatus = gOtaStatus_Success_c;

    if (gTmrInvalidTimerID_c == mOtaServerSetup.otaServerTmr)
    {
        mOtaServerSetup.otaServerTmr = TMR_AllocateTimer();

        if (gTmrInvalidTimerID_c == mOtaServerSetup.otaServerTmr)
        {
            panic(0, (uint32_t)OtaServerInit, 0, 0);
            return gOtaStatus_NoTimers_c;
        }
    }

    if (!mpOtaServerMsgQueue)
    {
        coapRegCbParams_t cbParams[] =  {{OtaServer_CoapCb, (coapUriPath_t *) &gOTA_CLIENT_URI_PATH}};
        coapStartUnsecParams_t coapParams = {COAP_DEFAULT_PORT, AF_INET6};

        /* Register Services in COAP */
        mOtaServerSetup.coapInstanceId = COAP_CreateInstance(NULL, &coapParams, gIpIfSlp0_c,
                                         (coapRegCbParams_t *)cbParams, NumberOfElements(cbParams));
        mOtaServerSetup.opMode = gOtaServer_DefaultOperationMode_c;
        mOtaServerSetup.isActive = FALSE;
        FLib_MemSet(&mOtaSeverTempAbortList, THR_ALL_FFs8,
                    sizeof(otaServerTempAbortList_t) * gOtaServer_TempAbortListSize_c);
        FLib_MemSet(&mOtaClientSessionInfoTable, 0x00,
                    sizeof(otaClientSessionInfo_t) * gOtaServer_MaxSimultaneousClients_c);
        mOtaServerSetup.fileVersionPolicy = gOtaFileVersionDefaultPolicies_c;

        /* Init ota server protocol callbacks */
        (void)OTA_RegisterToFsci(gOtaServer_ThciInteface_c, &mThciOtaServerCb);
        mpOtaServerMsgQueue = pMsgQueue;
        ListInit(&mOtaServerThciClientInfoQueue, gOtaServer_MaxSimultaneousClients_c);

#if gOtaServer_EnableStandaloneMode_c

        if (mOtaServerSetup.opMode == gOtaServerOpMode_Standalone_c)
        {
            OtaServer_InitStandaloneOpMode();

            if (TRUE == mOtaServerSetup.isActive)
            {
                OtaServer_HandleBlockSocket(TRUE);
            }
        }

#endif

#if OTA_USE_NWK_DATA
        thrLocalServiceSet_t otaServiceSet = {.thrSenterpriseNumber = THREAD_ENTERPRISE_NUMBER_ARRAY,
                                              .thrSserviceDataLen   = 9,
                                              .thrSserviceData      = "otaserver",
                                              .thrSserver16Addr     = {0xFF, 0xFF},
                                              .thrSserverDataLen    = 0,
                                              .thrSserverData       = {0},
                                              .thrSserviceId        = THREAD_OTA_SERVICE_TYPE_ID,
                                              .thrSstable           = TRUE
                                             };
        (void)THR_ServiceAttrAddEntry(0, &otaServiceSet);
#endif

    }
    else
    {
        otaStatus = gOtaStatus_AlreadyStarted_c;
    }

    return otaStatus;
}

/*!*************************************************************************************************
\fn     otaStatus_t OtaClient_StartServerImageNotify(void *param)
\brief  This function is used to send a Server multicast image notify.

\param  [in]   param          Not used

\return        otaStatus_t    Status of the operation
 ***************************************************************************************************/
otaStatus_t OtaClient_StartServerImageNotify
(
    void *param
)
{
#if gOtaServer_EnableStandaloneMode_c
    otaStatus_t otaStatus = gOtaStatus_Failed_c;

    if ((mOtaServerSetup.opMode == gOtaServerOpMode_Standalone_c) &&
            (mOtaServerSetup.isActive == TRUE))
    {
        thciOtaServer_ImageNotify_t imageNotify;

        FLib_MemCpy(&imageNotify.fileVersion, &mOtaServerImageList[0].fileVersion, sizeof(uint32_t));
        FLib_MemCpy(&imageNotify.imageType, &mOtaServerImageList[0].imageType, sizeof(uint16_t));
        FLib_MemCpy(&imageNotify.manufacturerCode, &mOtaServerImageList[0].manufCode, sizeof(uint16_t));

        /* Send back an image notify command */
        otaStatus = OtaServer_CoapSendImageNotify(&imageNotify, &in6addr_realmlocal_allthreadnodes);
    }

    return otaStatus;
#else
    return gOtaStatus_EmptyEntry_c;
#endif
}

/*==================================================================================================
Private functions
==================================================================================================*/
/*!*************************************************************************************************
\private
\fn     static void OtaServer_CoapCb(bool_t sessionStatus, void *pData, coapSession_t *pSession, uint32_t dataLen)
\brief  This function is the callback function for CoAP message.

\param  [in]    sessionStatus   Status for CoAP session
\param  [in]    pData           Pointer to CoAP message payload
\param  [in]    pSession        Pointer to CoAP session
\param  [in]    dataLen         Length of CoAP payload
***************************************************************************************************/
static void OtaServer_CoapCb
(
    coapSessionStatus_t sessionStatus,
    void *pData,
    coapSession_t *pSession,
    uint32_t dataLen
)
{
    if (gCoapSuccess_c == sessionStatus)
    {
        if (gOtaStatus_Success_c == OtaServer_CmdCheck(pData, dataLen))
        {
            otaClientInfo_t *pOtaClientInfo = MEM_BufferAlloc(sizeof(otaClientInfo_t) + dataLen);

            if (NULL != pOtaClientInfo)
            {
                ipAddr_t nullAddr = {0};

                /* Save client info params */
                IP_AddrCopy(&pOtaClientInfo->remoteAddr, &pSession->remoteAddr);

                if (FLib_MemCmp(&pSession->sourceAddr, &nullAddr, sizeof(pOtaClientInfo->sourceAddr)))
                {
                    IP_AddrCopy(&pOtaClientInfo->sourceAddr, &in6addr_realmlocal_allthreadnodes);
                }

                FLib_MemCpy(&pOtaClientInfo->pData, pData, dataLen);
                pOtaClientInfo->dataLen = dataLen;
                pOtaClientInfo->timeStamp = TMR_GetTimestamp();

                /* Send nwk msg to process client request */
                if (FALSE == NWKU_SendMsg(OtaServer_ClientProcess, pOtaClientInfo, mpOtaServerMsgQueue))
                {
                    MEM_BufferFree(pOtaClientInfo);
                }
            }
        }
    }
}

/*!*************************************************************************************************
\private
\fn     static void OtaClient_UdpServerService(void *pInData)
\brief  This function is the callback function for Ota server socket.

\param  [in]    pInData     Pointer to the received packet
***************************************************************************************************/
static void OtaClient_UdpServerService
(
    void *pInData
)
{
    sessionPacket_t *pSessionPacket = (sessionPacket_t *)pInData;
    uint8_t otaCommand = *(uint8_t *)pSessionPacket->pData;

    if (gOtaCmd_BlockReq_c == otaCommand)
    {
        otaClientInfo_t *pOtaClientInfo = MEM_BufferAlloc(sizeof(otaClientInfo_t) + pSessionPacket->dataLen);

        if (NULL != pOtaClientInfo)
        {
            /* Save client info params */
            IP_AddrCopy(&pOtaClientInfo->remoteAddr, &pSessionPacket->remAddr.ss_addr);
            IP_AddrCopy(&pOtaClientInfo->sourceAddr, &pSessionPacket->localAddr.ss_addr);
            FLib_MemCpy(&pOtaClientInfo->pData, pSessionPacket->pData, pSessionPacket->dataLen);
            pOtaClientInfo->port = ((sockaddrIn6_t *)&pSessionPacket->remAddr)->sin6_port;
            pOtaClientInfo->dataLen = pSessionPacket->dataLen;
            pOtaClientInfo->timeStamp = TMR_GetTimestamp();
            OtaServer_ClientProcess((void *)pOtaClientInfo);
        }
    }

    MEM_BufferFree(pSessionPacket->pData);
    MEM_BufferFree(pSessionPacket);
}

/*!*************************************************************************************************
\private
\fn     static void OtaServer_ClientProcess(void *param)
\brief  This function is used to process ota client commands.

\param  [in]    param    Pointer to pOtaClientInfo structure
***************************************************************************************************/
static void OtaServer_ClientProcess
(
    void *param
)
{
    otaClientInfo_t *pOtaClientInfo = (otaClientInfo_t *)param;

    if (FALSE == mOtaServerSetup.isActive)
    {
        /* Server is not active -  send back a Rsp command with status no image available */
        OtaServer_CoapSendRspWaitAbortData(pOtaClientInfo, gOtaFileStatus_NoImageAvailable_c, 0);
    }
    else
    {
        if ((gOtaStatus_Success_c == OtaServer_CheckClientSessionTable(pOtaClientInfo)) &&
                (gOtaStatus_Success_c == OtaServer_HandleBlockSocket(TRUE)))
        {
            if (mOtaServerSetup.opMode == gOtaServerOpMode_Dongle_c)
            {
                if ((NULL == mpOtaServerTempClientInfo) && (0 == mOtaServerThciClientInfoQueue.size))
                {
                    /* Process client */
                    OtaServer_CmdProcess(pOtaClientInfo);
                }
                else
                {
                    if (gOtaStatus_Success_c == OtaServer_CheckThciQueue(pOtaClientInfo))
                    {
                        /* Add Client to queue */
                        if (gListOk_c != MSG_Queue(&mOtaServerThciClientInfoQueue, pOtaClientInfo))
                        {
                            MEM_BufferFree(pOtaClientInfo);
                        }
                    }
                    else
                    {
                        MEM_BufferFree(pOtaClientInfo);
                    }
                }
            }

#if gOtaServer_EnableStandaloneMode_c
            else if (mOtaServerSetup.opMode == gOtaServerOpMode_Standalone_c)
            {
                OtaServer_CmdProcess(pOtaClientInfo);
            }

#endif
            else
            {
                /* Wrong configuration */
                MEM_BufferFree(pOtaClientInfo);
            }
        }
        else
        {
            OtaServer_CoapSendRspWaitAbortData(pOtaClientInfo,
                                               gOtaFileStatus_ServerBusy_c,
                                               gOtaServer_DelayForNextRequestMs_c);
        }
    }
}

/*!*************************************************************************************************
\private
\fn     static otaStatus_t OtaServer_CheckClientSessionTable(otaClientInfo_t *pOtaClientInfo)
\brief  This function is used to check if the server can process this new client request.

\param  [in]    pOtaClientInfo    Pointer to client info

\return         otaStatus_t       Status of the operation
***************************************************************************************************/
static otaStatus_t OtaServer_CheckClientSessionTable
(
    otaClientInfo_t *pOtaClientInfo
)
{
    otaStatus_t status = gOtaStatus_Failed_c;
    uint8_t idxInfoTable;
    uint8_t firstExpiredEntry = gOtaServer_MaxSimultaneousClients_c;

    for (idxInfoTable = 0; idxInfoTable < gOtaServer_MaxSimultaneousClients_c; idxInfoTable++)
    {
        if (IP_IsAddrEqual(&pOtaClientInfo->remoteAddr, &mOtaClientSessionInfoTable[idxInfoTable].remoteAddr))
        {
            mOtaClientSessionInfoTable[idxInfoTable].timeStamp = pOtaClientInfo->timeStamp;
            return gOtaStatus_Success_c;
        }

        if (((mOtaClientSessionInfoTable[idxInfoTable].timeStamp == 0) ||
                (mOtaClientSessionInfoTable[idxInfoTable].timeStamp + (gOtaServer_ClientSessionExpirationMs_c * 1000) < pOtaClientInfo->timeStamp)) &&
                (firstExpiredEntry == gOtaServer_MaxSimultaneousClients_c))
        {
            firstExpiredEntry = idxInfoTable;
        }
    }

    if (firstExpiredEntry < gOtaServer_MaxSimultaneousClients_c)
    {
        IP_AddrCopy(&mOtaClientSessionInfoTable[firstExpiredEntry].remoteAddr, &pOtaClientInfo->remoteAddr);
        mOtaClientSessionInfoTable[firstExpiredEntry].timeStamp = pOtaClientInfo->timeStamp;
        status = gOtaStatus_Success_c;
    }

    return status;
}

/*!*************************************************************************************************
\private
\fn     static otaStatus_t OtaServer_CmdCheck(void *pData, uint32_t dataLen)
\brief  This function is used to check if ota client command is valid.

\param  [in]    pData          OTA cmd data
\param  [in]    dataLen        OTA cmd length

\return         otaStatus_t    Status of the operation
***************************************************************************************************/
static otaStatus_t OtaServer_CmdCheck
(
    void *pData,
    uint32_t dataLen
)
{
    otaStatus_t status = gOtaStatus_Failed_c;
    uint8_t otaCommand = *(uint8_t *)pData;

    switch (otaCommand)
    {
        case gOtaCmd_QueryImageReq_c:
            if (dataLen == sizeof(otaCmd_QueryImageReq_t))
            {
                status = gOtaStatus_Success_c;
            }

            break;

        case gOtaCmd_BlockReq_c:
            if (dataLen == (sizeof(otaCmd_BlockReq_t)))
            {
                status = gOtaStatus_Success_c;
            }

            break;

        case gOtaCmd_UpgradeEndReq_c:
            if (dataLen == sizeof(otaCmd_UpgradeEndReq_t))
            {
                status = gOtaStatus_Success_c;
            }

            break;

        case gOtaCmd_ServerDiscovery_c:
            if (dataLen == sizeof(otaCmd_ServerDiscovery_t))
            {
                status = gOtaStatus_Success_c;
            }

            break;

        default:
            break;
    }

    return status;
}

/*!*************************************************************************************************
\private
\fn     static void OtaServer_CmdProcess(void *param)
\brief  This function is used to process ota client commands.

\param  [in]    param    Pointer to pOtaClientInfo structure
***************************************************************************************************/
static void OtaServer_CmdProcess
(
    void *param
)
{
    otaClientInfo_t *pOtaClientInfo = (otaClientInfo_t *)param;
    uint8_t otaCommand = *pOtaClientInfo->pData;

    switch (otaCommand)
    {
        case gOtaCmd_QueryImageReq_c:
            (void)OtaServer_QueryImageReqHandler(pOtaClientInfo);
            break;

        case gOtaCmd_BlockReq_c:
            (void)OtaServer_BlockReqHandler(pOtaClientInfo);
            break;

        case gOtaCmd_UpgradeEndReq_c:
            (void)OtaServer_UpgradeEndReqHandler(pOtaClientInfo);
            break;

        case gOtaCmd_ServerDiscovery_c:
            (void)OtaServer_ServerDiscoveryHandler(pOtaClientInfo);
            break;

        default:
            MEM_BufferFree(pOtaClientInfo);
            break;
    }
}

/*!*************************************************************************************************
\private
\fn     static otaStatus_t OtaServer_QueryImageReqHandler(otaClientInfo_t *pOtaClientInfo)
\brief  This function is used to process a QueryImageReq command.

\param  [in]    pOtaClientInfo    Pointer to client info

\return         otaStatus_t       Status of the operation
***************************************************************************************************/
static otaStatus_t OtaServer_QueryImageReqHandler
(
    otaClientInfo_t *pOtaClientInfo
)
{
    otaStatus_t status = gOtaStatus_Failed_c;

    if (mOtaServerSetup.opMode == gOtaServerOpMode_Dongle_c)
    {
        status = OtaServerDongle_QueryImageReqHandler(pOtaClientInfo);
    }

#if gOtaServer_EnableStandaloneMode_c
    else
    {
        status = OtaServerStandalone_QueryImageReqHandler(pOtaClientInfo);
    }

#endif

    return status;
}

/*!*************************************************************************************************
\private
\fn     static otaStatus_t OtaServer_BlockReqHandler(otaClientInfo_t *pOtaClientInfo)
\brief  This function is used to process a BlockRequest command.

\param  [in]    pOtaClientInfo    Pointer to client info

\return         otaStatus_t       Status of the operation
***************************************************************************************************/
static otaStatus_t OtaServer_BlockReqHandler
(
    otaClientInfo_t *pOtaClientInfo
)
{
    otaStatus_t status = gOtaStatus_Failed_c;
    uint16_t clientId = (pOtaClientInfo->remoteAddr.addr8[14]  << 8) + pOtaClientInfo->remoteAddr.addr8[15];

    if (OtaServer_IsClientValid(clientId))
    {
        if (mOtaServerSetup.opMode == gOtaServerOpMode_Dongle_c)
        {
            status = OtaServerDongle_BlockReqHandler(pOtaClientInfo);
        }

#if gOtaServer_EnableStandaloneMode_c
        else
        {
            status = OtaServerStandalone_BlockReqHandler(pOtaClientInfo);
        }

#endif
    }
    else
    {
        /* Client is in the abort list-  send back a Block Rsp command with Abort status */
        if (OtaServer_CoapSendRspWaitAbortData(pOtaClientInfo, gOtaFileStatus_Abort_c, 0) == gOtaStatus_Success_c)
        {
            OtaServer_RemoveClientFromAbortList(clientId);
            status = gOtaStatus_Success_c;
        }
    }

    return status;
}

/*!*************************************************************************************************
\private
\fn     static otaStatus_t OtaServer_UpgradeEndReqHandler(otaClientInfo_t *pOtaClientInfo)
\brief  This function is used to process an UpdateEndRequest command.

\param  [in]    pOtaClientInfo    Pointer to client info

\return         otaStatus_t       Status of the operation
***************************************************************************************************/
static otaStatus_t OtaServer_UpgradeEndReqHandler
(
    otaClientInfo_t *pOtaClientInfo
)
{
    otaStatus_t status = gOtaStatus_Failed_c;
    otaCmd_UpgradeEndRsp_t upgradeRsp = {gOtaCmd_UpgradeEndRsp_c, 0x00};
    uint32_t timeInMs = NWKU_GetTimestampMs();

    FLib_MemCpy(&upgradeRsp.data.success.currentTime, &timeInMs, sizeof(uint32_t));
    timeInMs += NWKU_GetRandomNoFromInterval(gOtaServer_MinDelayForEndRequestMs_c, gOtaServer_MaxDelayForEndRequestMs_c);
    FLib_MemCpy(&upgradeRsp.data.success.upgradeTime, &timeInMs, sizeof(uint32_t));

    /* For dongle configuration */
    if (mOtaServerSetup.opMode == gOtaServerOpMode_Dongle_c)
    {
        if (FLib_MemCmp(mpOtaServerTempClientInfo->remoteAddr.addr8, pOtaClientInfo->remoteAddr.addr8, sizeof(ipAddr_t)))
        {
            OtaServer_CloseThciSessionTimeout(NULL);
        }
    }

    if (OtaServer_CoapSendRsp(pOtaClientInfo, (uint8_t *)&upgradeRsp,
                              sizeof(otaCmd_UpgradeEndRsp_t)) == gOtaStatus_Success_c)
    {
        uint16_t clientId;

        FLib_MemCpyReverseOrder(&clientId, &pOtaClientInfo->remoteAddr.addr8[14], sizeof(uint16_t));
        OTA_ClientInfoCnf((uint8_t *)&pOtaClientInfo->remoteAddr, THR_ALL_FFs32, clientId);
        status = gOtaStatus_Success_c;
    }

    return status;
}

/*!*************************************************************************************************
\private
\fn     static otaStatus_t OtaServer_ServerDiscoveryHandler(otaClientInfo_t *pOtaClientInfo)
\brief  This function is used to process a ServerDiscovery command.

\param  [in]    pOtaClientInfo    Pointer to client info

\return         otaStatus_t       Status of the operation
***************************************************************************************************/
static otaStatus_t OtaServer_ServerDiscoveryHandler
(
    otaClientInfo_t *pOtaClientInfo
)
{
    otaStatus_t status = gOtaStatus_Success_c;

    if (mOtaServerSetup.opMode == gOtaServerOpMode_Dongle_c)
    {
        status = OtaServerDongle_ServerDiscoveryHandler(pOtaClientInfo);
    }

#if gOtaServer_EnableStandaloneMode_c
    else
    {
        status = OtaServerStandalone_ServerDiscoveryHandler(pOtaClientInfo);
    }

#endif

    return status;
}

/*!*************************************************************************************************
\private
\fn     static otaStatus_t OtaServer_CoapSendImageNotify(thciOtaServer_ImageNotify_t *pThciImageNotify,
                                                         ipAddr_t *pDestAddr)
\brief  This function is used to send multicast a coap Image notify command to all thread nodes

\param  [in]   pThciImageNotify    Pointer to image notify message
\param  [in]   pDestAddr           Pointer to IPv6 address

\return        otaStatus_t         Status of the operation
 ***************************************************************************************************/
static otaStatus_t OtaServer_CoapSendImageNotify
(
    thciOtaServer_ImageNotify_t *pThciImageNotify,
    ipAddr_t *pDestAddr
)
{
    otaStatus_t status = gOtaStatus_Failed_c;
    coapSession_t *pSession = COAP_OpenSession(mOtaServerSetup.coapInstanceId);

    if (pSession)
    {
        otaServerCmd_ImageNotify_t imageNotify;
        uint16_t fragmentSize = gOtaMaxBlockDataSize_c;

        pSession->ipIfId = gIpIfSlp0_c;
        pSession->pCallback = NULL;
        FLib_MemCpy(&pSession->remoteAddr, pDestAddr, sizeof(ipAddr_t));
        pSession->msgType = gCoapNonConfirmable_c;
        pSession->code = gCoapPOST_c;
        (void)THR_GetIP6Addr(0, OTA_ADDR_TYPE, &pSession->sourceAddr, NULL);
        /* Complete command */
        imageNotify.commandId = gOtaCmd_ImageNotify_c;
        FLib_MemCpy(&imageNotify.manufacturerCode, &pThciImageNotify->manufacturerCode,
                    sizeof(thciOtaServer_ImageNotify_t) - 2);
        FLib_MemCpy(&imageNotify.serverDownloadPort[0], (void *)&mOtaServerSetup.downloadPort, sizeof(uint16_t));
        FLib_MemCpy(&imageNotify.fragmentSize[0], &fragmentSize, sizeof(imageNotify.fragmentSize));

        COAP_AddOptionToList(pSession, COAP_URI_PATH_OPTION, (uint8_t *)OTA_SERVER_URI_PATH, SizeOfString(OTA_SERVER_URI_PATH));

        /* Send command */
        if (gNwkStatusSuccess_c == COAP_Send(pSession, gCoapMsgTypeNonPost_c, &imageNotify, sizeof(otaServerCmd_ImageNotify_t)))
        {
            status = gOtaStatus_Success_c;
        }
    }

    return status;
}

/*!*************************************************************************************************
\private
\fn     static otaStatus_t OtaServer_CoapSendRsp(otaClientInfo_t *pOtaClientInfo, uint8_t *pData, uint32_t dataLen)
\brief  This function is used to send a coap response to a OTA client node.

\param  [in]   pOtaClientInfo    Pointer to client info
\param  [in]   pData             Pointer to data
\param  [in]   dataLen           Payload length

\return        otaStatus_t       Status of the operation
***************************************************************************************************/
static otaStatus_t OtaServer_CoapSendRsp
(
    otaClientInfo_t *pOtaClientInfo,
    uint8_t *pData,
    uint32_t dataLen
)
{
    otaStatus_t status = gOtaStatus_Failed_c;
    coapSession_t *pCoapSession = COAP_OpenSession(mOtaServerSetup.coapInstanceId);

    if (pCoapSession)
    {
        FLib_MemCpy(&pCoapSession->remoteAddr, &pOtaClientInfo->remoteAddr, sizeof(pCoapSession->remoteAddr));
        pCoapSession->ipIfId = gIpIfSlp0_c;
        pCoapSession->pCallback = NULL;
        /* complete coap message */
        /* add coap options */
        pCoapSession->msgType = gCoapNonConfirmable_c;
        pCoapSession->code = gCoapPOST_c;
        pCoapSession->autoClose = TRUE;
        (void)THR_GetIP6Addr(0, OTA_ADDR_TYPE, &pCoapSession->sourceAddr, NULL);
        COAP_AddOptionToList(pCoapSession, COAP_URI_PATH_OPTION, (uint8_t *)OTA_SERVER_URI_PATH, SizeOfString(OTA_SERVER_URI_PATH));

        /* send command */
        if (gNwkStatusSuccess_c == COAP_Send(pCoapSession, gCoapMsgTypeNonPost_c, pData, dataLen))
        {
            status = gOtaStatus_Success_c;
        }
    }

    MEM_BufferFree(pOtaClientInfo);

    /* For dongle configuration */
    OtaServer_ProcessNextClient();

    return status;
}

/*!*************************************************************************************************
\private
\fn     static otaStatus_t OtaServer_SocketSendRsp(otaClientInfo_t *pOtaClientInfo, uint8_t *pData, uint32_t dataLen)
\brief  This function is used to send a socket response to a OTA client node.

\param  [in]   pOtaClientInfo    Pointer to client info
\param  [in]   pData             Pointer to data
\param  [in]   dataLen           Payload length

\return        otaStatus_t       Status of the operation
***************************************************************************************************/
static otaStatus_t OtaServer_SocketSendRsp
(
    otaClientInfo_t *pOtaClientInfo,
    uint8_t *pData,
    uint32_t dataLen
)
{
    sockaddrStorage_t portAddr;

    /* Set remote address and local port */
    FLib_MemSet(&portAddr, 0, sizeof(sockaddrStorage_t));
    ((sockaddrIn6_t *)&portAddr)->sin6_family = AF_INET6;
    ((sockaddrIn6_t *)&portAddr)->sin6_port = pOtaClientInfo->port;
    IP_AddrCopy(&((sockaddrIn6_t *)&portAddr)->sin6_addr, &pOtaClientInfo->remoteAddr);
    ((sockaddrIn6_t *)&portAddr)->sin6_flowinfo = BSDS_SET_TX_SEC_FLAGS(1, 5);
    ((sockaddrIn6_t *)&portAddr)->sin6_scope_id = gIpIfSlp0_c;
    (void)sendmsg(mOtaServerSetup.mOtaUdpSrvSockFd, &pOtaClientInfo->sourceAddr, pData, dataLen, MSG_SEND_WITH_MEMBUFF,
                  &portAddr, sizeof(portAddr));
    MEM_BufferFree(pOtaClientInfo);

    /* For dongle configuration */
    OtaServer_ProcessNextClient();

    return gOtaStatus_Success_c;
}

/*!*************************************************************************************************
\private
\fn     static void OtaServer_CoapSendQueryImageRsp(void *pParam)
\brief  This function is used to send a Query Image Rsp command to the OTA client node.

\param  [in]   pParam    Pointer to data
 ***************************************************************************************************/
static void OtaServer_CoapSendQueryImageRsp
(
    void *pParam
)
{
    otaClientInfo_t *pTempClient = mpOtaServerTempClientInfo;

    mpOtaServerTempClientInfo = NULL;

    if (pTempClient)
    {
        (void)OtaServer_CoapSendRsp(pTempClient, (uint8_t *)pParam, sizeof(otaCmd_QueryImageRsp_t));
    }

    MEM_BufferFree(pParam);
}

/*!*************************************************************************************************
\private
\fn     static otaStatus_t OtaServer_CoapSendRspWaitAbortData(otaClientInfo_t *pOtaClientInfo, uint8_t status,
                                                              uint32_t delayInMs)
\brief  This function is used to send a Query Image Rsp command to the OTA client node using
        a status != Success

\param  [in]   pOtaClientInfo    Pointer to client info
\param  [in]   status            Query image response status != Success
\param  [in]   delayInMs         Delay in milliseconds

\return        otaStatus_t       Status of the operation
 ***************************************************************************************************/
static otaStatus_t OtaServer_CoapSendRspWaitAbortData
(
    otaClientInfo_t *pOtaClientInfo,
    uint8_t status,
    uint32_t delayInMs
)
{
    otaStatus_t result = gOtaStatus_Failed_c;

    if (status != gOtaFileStatus_Success_c)
    {
        /* All busy / abort responses have the same structure as queryRsp*/
        otaCmd_QueryImageRsp_t queryRsp = {gOtaCmd_QueryImageRsp_c, gOtaFileStatus_ServerBusy_c, {0x00}};
        uint8_t len = 2 + sizeof(otaCmd_QueryImageRspWait_t);
        uint8_t otaCommand = *pOtaClientInfo->pData;
        uint32_t timeInMs = NWKU_GetTimestampMs();

        switch (otaCommand)
        {
            case gOtaCmd_BlockReq_c:
                queryRsp.commandId = gOtaCmd_BlockRsp_c;
                break;

            case gOtaCmd_QueryImageReq_c:
                queryRsp.commandId = gOtaCmd_QueryImageRsp_c;
                break;

            case gOtaCmd_UpgradeEndReq_c:
                queryRsp.commandId = gOtaCmd_UpgradeEndRsp_c;
                break;

            default:
                MEM_BufferFree(pOtaClientInfo);
                return gOtaStatus_InvalidParam_c;
        }

        queryRsp.status = status;
        FLib_MemCpy(&queryRsp.data.wait.currentTime, &timeInMs, sizeof(uint32_t));
        timeInMs = timeInMs + delayInMs;
        FLib_MemCpy(&queryRsp.data.wait.requestTime, &timeInMs, sizeof(uint32_t));

        if (gOtaStatus_Success_c == OtaServer_CoapSendRsp(pOtaClientInfo, (uint8_t *)&queryRsp, len))
        {
            result = gOtaStatus_Success_c;
        }
    }
    else
    {
        MEM_BufferFree(pOtaClientInfo);
    }

    return result;
}

/*!*************************************************************************************************
\private
\fn     static void OtaServerDongle_SendBlockRsp(void *pParam)
\brief  This function is used to send a Block Rsp command to the OTA client node.
        Only called in dongle mode after receiving the requested block from the PC tool.

\param  [in]   pParam    Pointer to data
 ***************************************************************************************************/
static void OtaServerDongle_SendBlockRsp
(
    void *pParam
)
{
    otaCmd_BlockRsp_t *pBlockRsp = (otaCmd_BlockRsp_t *)pParam;
    otaClientInfo_t *pTempClient = mpOtaServerTempClientInfo;

    mpOtaServerTempClientInfo = NULL;

    if (pTempClient)
    {
        (void)OtaServer_SocketSendRsp(pTempClient, (uint8_t *)pBlockRsp,
                                      sizeof(otaCmd_BlockRsp_t) - 1 + pBlockRsp->data.success.dataSize);
    }

    MEM_BufferFree(pBlockRsp);
}

/*!*************************************************************************************************
\private
\fn  static void OtaServer_CloseThciSessionTimeoutCb(void *pParam)
\brief  This function is used to send a msg to close temporary coap session.

\param [in]   pParam    Pointer to data
 ***************************************************************************************************/
static void OtaServer_CloseThciSessionTimeoutCb
(
    void *pParam
)
{
    if (FALSE == NWKU_SendMsg(OtaServer_CloseThciSessionTimeout, NULL, mpOtaServerMsgQueue))
    {
        TMR_StartSingleShotTimer(mOtaServerSetup.otaServerTmr, gOtaServer_CloseThciSessionTimeoutMs_c,
                                 OtaServer_CloseThciSessionTimeoutCb, NULL);
    }
}

/*!*************************************************************************************************
\private
\fn     static void OtaServer_CloseThciSessionTimeout(void *pParam)
\brief  This function is used to close temporary coap session.

\param  [in]   pParam    Pointer to data
 ***************************************************************************************************/
static void OtaServer_CloseThciSessionTimeout
(
    void *pParam
)
{
    if (mpOtaServerTempClientInfo)
    {
        MEM_BufferFree(mpOtaServerTempClientInfo);
        mpOtaServerTempClientInfo = NULL;
    }

    OtaServer_ProcessNextClient();
}

/*!*************************************************************************************************
\private
\fn     static bool_t OtaServer_IsClientValid(uint16_t clientId)
\brief  This function is used to validate client ID.

\param  [in]   clientId    Client ID

\return        bool_t    TRUE - client valid, FALSE - client is in the abort list
 ***************************************************************************************************/
static bool_t OtaServer_IsClientValid
(
    uint16_t clientId
)
{
    bool_t result = TRUE;

    for (uint8_t i = 0; i < gOtaServer_TempAbortListSize_c; i++)
    {
        if (mOtaSeverTempAbortList[i].clientId == clientId)
        {
            result = FALSE;
            break;
        }
    }

    return result;
}

/*!*************************************************************************************************
\private
\fn     static bool_t OtaServer_AddClientToAbortList(uint16_t clientId)
\brief  This function is used to add a client ID to abort list.

\param  [in]   clientId    Client ID

\return        bool_t      TRUE - client added, FALSE - otherwise
 ***************************************************************************************************/
static bool_t OtaServer_AddClientToAbortList
(
    uint16_t clientId
)
{
    bool_t result = FALSE;

    for (uint8_t i = 0; i < gOtaServer_TempAbortListSize_c; i++)
    {
        if (mOtaSeverTempAbortList[i].clientId == gOtaServer_InvalidClientId_c)
        {
            mOtaSeverTempAbortList[i].clientId = clientId;
            result = TRUE;
            break;
        }
    }

    return result;
}

/*!*************************************************************************************************
\private
\fn     static bool_t OtaServer_RemoveClientFromAbortList(uint16_t clientId)
\brief  This function is used to remove a client ID from abort list.

\param [in]   clientId    Client ID

\return       bool_t      TRUE - client added, FALSE - otherwise
 ***************************************************************************************************/
static bool_t OtaServer_RemoveClientFromAbortList
(
    uint16_t clientId
)
{
    bool_t result = FALSE;

    for (uint8_t i = 0; i < gOtaServer_TempAbortListSize_c; i++)
    {
        if (mOtaSeverTempAbortList[i].clientId == clientId)
        {
            mOtaSeverTempAbortList[i].clientId = gOtaServer_InvalidClientId_c;
            result = TRUE;
            break;
        }
    }

    return result;
}

/*************************************************************************************************
*
*  OTA Server Dongle functions
*
**************************************************************************************************/

/*!*************************************************************************************************
\private
\fn     static otaStatus_t OtaServerDongle_QueryImageReqHandler(otaClientInfo_t *pOtaClientInfo)
\brief  This function is used to process a QueryImageReq command.

\param  [in]    pOtaClientInfo   Pointer to client info

\return        otaStatus_t       Status of the operation
***************************************************************************************************/
static otaStatus_t OtaServerDongle_QueryImageReqHandler
(
    otaClientInfo_t *pOtaClientInfo
)
{
    otaStatus_t status = gOtaStatus_Success_c;
    otaCmd_QueryImageReq_t  *pQueryImgReq = (otaCmd_QueryImageReq_t *)pOtaClientInfo->pData;
    uint16_t clientId = (pOtaClientInfo->remoteAddr.addr8[14] << 8) + pOtaClientInfo->remoteAddr.addr8[15];

    /* Should not end up here. Earlier in the call stack there already should be a check for the value of mpOtaServerTempClientInfo to be NULL. Just a safety check */
    if (mpOtaServerTempClientInfo != NULL)
    {
        MEM_BufferFree(pOtaClientInfo);
        status = gOtaStatus_Failed_c;
    }
    else
    {
        mpOtaServerTempClientInfo = pOtaClientInfo; /* Store the data for the current client that is served. */

        TMR_StartSingleShotTimer(mOtaServerSetup.otaServerTmr, gOtaServer_CloseThciSessionTimeoutMs_c,
                                 OtaServer_CloseThciSessionTimeoutCb, NULL);

        /* wait the host application to confirm (see ThciOtaServer_QueryImageRspCnf) */
        OTA_QueryImageReq(clientId, (uint16_t)(pQueryImgReq->manufacturerCode[0] + (pQueryImgReq->manufacturerCode[1] << 8)),
                          (uint16_t)(pQueryImgReq->imageType[0] + (pQueryImgReq->imageType[1] << 8)),
                          (uint32_t)(pQueryImgReq->fileVersion[0] + (pQueryImgReq->fileVersion[1] << 8) +
                                     (pQueryImgReq->fileVersion[2] << 16) + (pQueryImgReq->fileVersion[3] << 24)));
    }

    return status;
}

/*!*************************************************************************************************
\private
\fn     static otaStatus_t OtaServerDongle_BlockReqHandler(otaClientInfo_t *pOtaClientInfo)
\brief  This function is used to process a BlockRequest command.

\param  [in]    pOtaClientInfo    Pointer to client info

\return         otaStatus_t       Status of the operation
***************************************************************************************************/
static otaStatus_t OtaServerDongle_BlockReqHandler
(
    otaClientInfo_t *pOtaClientInfo
)
{
    otaStatus_t status = gOtaStatus_Failed_c;
    otaCmd_BlockReq_t *pBlockReq = (otaCmd_BlockReq_t *)pOtaClientInfo->pData;
    uint16_t clientId = (pOtaClientInfo->remoteAddr.addr8[14] << 8) + pOtaClientInfo->remoteAddr.addr8[15];
    uint32_t offset = 0;

    FLib_MemCpy(&offset, pBlockReq->fileOffset, sizeof(offset));

    /* Should not end up here. Earlier in the call stack there already should be a check for the value of mpOtaServerTempClientInfo to be NULL. Just a safety check */
    if (mpOtaServerTempClientInfo != NULL)
    {
        MEM_BufferFree(pOtaClientInfo);
        status = gOtaStatus_Failed_c;
    }
    else
    {
        mpOtaServerTempClientInfo = pOtaClientInfo; /* Store the data for the current client that is served. */
        TMR_StartSingleShotTimer(mOtaServerSetup.otaServerTmr, gOtaServer_CloseThciSessionTimeoutMs_c,
                                 OtaServer_CloseThciSessionTimeoutCb, NULL);

        /* send info once at 100 packets */
        if (((offset / pBlockReq->maxDataSize) % 100) == 0 || (offset == 0))
        {
            OTA_ClientInfoCnf((uint8_t *)&pOtaClientInfo->remoteAddr, offset, clientId);
        }

        /* wait the host application to reply (see ThciOtaServer_BlockReceivedCnf) */
        OTA_ImageChunkReq(offset, pBlockReq->maxDataSize, clientId);
    }

    return status;
}

/*!*************************************************************************************************
\private
\fn     static otaStatus_t OtaServerDongle_ServerDiscoveryHandler(otaClientInfo_t *pOtaClientInfo)
\brief  This function is used to process a ServerDiscovery command.

\param  [in]    pOtaClientInfo    Pointer to client info

\return         otaStatus_t       Status of the operation
***************************************************************************************************/
static otaStatus_t OtaServerDongle_ServerDiscoveryHandler
(
    otaClientInfo_t *pOtaClientInfo
)
{
    otaStatus_t status = gOtaStatus_Success_c;

    /* Should not end up here. Earlier in the call stack there already should be a check for the value of mpOtaServerTempClientInfo to be NULL. Just a safety check */
    if (mpOtaServerTempClientInfo != NULL)
    {
        MEM_BufferFree(pOtaClientInfo);
        status = gOtaStatus_Failed_c;
    }
    else
    {
        /* check if it is a packet in progress */
        otaCmd_ServerDiscovery_t *pCmdData = (otaCmd_ServerDiscovery_t *)pOtaClientInfo->pData;

        mpOtaServerTempClientInfo = pOtaClientInfo; /* Store the data for the current client that is served. */
        TMR_StartSingleShotTimer(mOtaServerSetup.otaServerTmr, gOtaServer_CloseThciSessionTimeoutMs_c,
                                 OtaServer_CloseThciSessionTimeoutCb, NULL);
        OTA_QueryImageReq(gOtaServer_InvalidClientId_c,
                          (uint16_t)(pCmdData->manufacturerCode[0] + (pCmdData->manufacturerCode[1] << 8)),
                          (uint16_t)(pCmdData->imageType[0] + (pCmdData->imageType[1] << 8)), THR_ALL_FFs32);
    }

    return status;
}

#if gOtaServer_EnableStandaloneMode_c
/*************************************************************************************************
*
*  OTA Server Standalone functions
*
**************************************************************************************************/
/*!*************************************************************************************************
\private
\fn     static void OtaServer_InitStandaloneOpMode(void)
\brief  Initialize ota server standalone operation mode.
***************************************************************************************************/
static void OtaServer_InitStandaloneOpMode
(
    void
)
{
    bool_t imageAvailable = FALSE;
    uint8_t index = 0;
    uint8_t *pData;
    uint32_t currentAddr = 0;
    otaFileHeader_t otaHeader;

#if TWR_KW24D512 || TWR_KW22D512 || TWR_KW21D256 || TWR_KW21D512
    /* de-Init the keyboard module: for the specified platforms the
        keyboard and external EEPROM are on the same Bus */
    KBD_Deinit();
#endif
    /* init external memory */
    OTA_InitExternalMemory();

    /* process OTA header information */
    pData = (uint8_t *)&otaHeader;

    while ((currentAddr < gFlashParams_MaxImageLength_c) && (index < gOtaServer_MaxOtaImages_c))
    {
        uint32_t fileIdentifier;

        FLib_MemCpy(&fileIdentifier, otaHeader.fileIdentifier, sizeof(fileIdentifier));
        /* Read OTA Header */
        (void)OTA_ReadExternalMemory(pData, sizeof(otaFileHeader_t), currentAddr);

        if (fileIdentifier == gOtaFileIdentifierNo_c)
        {
            uint16_t manufCode, imageType;
            uint32_t fileVersion, fileSize;

            FLib_MemCpy(&fileVersion, &otaHeader.fileVersion, sizeof(uint32_t));
            FLib_MemCpy(&imageType, &otaHeader.imageType, sizeof(uint16_t));
            FLib_MemCpy(&manufCode, &otaHeader.manufacturerCode, sizeof(uint16_t));
            FLib_MemCpy(&fileSize, &otaHeader.totalImageSize, sizeof(uint32_t));
            index = OtaServerStandalone_KeepImageInfo(manufCode, imageType, fileVersion, fileSize);

            if (index < gOtaServer_MaxOtaImages_c)
            {
                uint32_t totalImageSize;

                FLib_MemCpy(&totalImageSize, (void *)otaHeader.totalImageSize, sizeof(uint32_t));
                currentAddr += totalImageSize;
                mOtaServerImageList[index].isValidEntry = TRUE;
                imageAvailable = TRUE;
            }
        }
        else
        {
            /* ignore other data */
            currentAddr = gFlashParams_MaxImageLength_c;
        }
    }

    mOtaServerSetup.isActive = imageAvailable;
}

/*!*************************************************************************************************
\private
\fn     static uint8_t OtaServerStandalone_ValidateImage(uint16_t manufCode, uint16_t imageType,
                                                         uint32_t fileVersion, bool_t serialProtocol)
\brief  Validate image by checking internal table.

\param  [in]    manufCode         Manufacturer code
        [in]    imageType         Image type
        [in]    fileVersion       File version
        [in]    serialProtocol    Image validation is required for the ota Server serial protocol

\return         uint8_t           Image index if success, otherwise returns gOtaServer_MaxOtaImages_c
***************************************************************************************************/
static uint8_t OtaServerStandalone_ValidateImage
(
    uint16_t manufCode,
    uint16_t imageType,
    uint32_t fileVersion,
    bool_t serialProtocol
)
{
    uint8_t result = gOtaServer_MaxOtaImages_c;

    /* validate internal list */
    for (uint8_t i = 0; i < gOtaServer_MaxOtaImages_c; i++)
    {
        if ((manufCode == mOtaServerImageList[i].manufCode) &&
                (imageType == mOtaServerImageList[i].imageType) &&
                (mOtaServerImageList[i].isValidEntry))
        {
            if (((fileVersion ==  mOtaServerImageList[i].fileVersion) && (mOtaServerSetup.fileVersionPolicy & gOtaFileVersionPolicies_Reinstall_c)) ||
                    ((fileVersion <  mOtaServerImageList[i].fileVersion) && (mOtaServerSetup.fileVersionPolicy & gOtaFileVersionPolicies_Upgrade_c)) ||
                    ((fileVersion >  mOtaServerImageList[i].fileVersion) && (mOtaServerSetup.fileVersionPolicy & gOtaFileVersionPolicies_Downgrade_c)) ||
                    ((serialProtocol) && (fileVersion ==  mOtaServerImageList[i].fileVersion)) || (fileVersion == THR_ALL_FFs32))
            {
                result = i;
                break;
            }
        }
    }

    return result;
}

/*!*************************************************************************************************
\private
\fn     static otaStatus_t OtaServerStandalone_QueryImageReqHandler(otaClientInfo_t *pOtaClientInfo)
\brief  This function is used to process a QueryImageReq command.

\param  [in]    pOtaClientInfo    Pointer to client info

\return         otaStatus_t       Status of the operation
***************************************************************************************************/
static otaStatus_t OtaServerStandalone_QueryImageReqHandler
(
    otaClientInfo_t *pOtaClientInfo
)
{
    otaStatus_t status = gOtaStatus_Failed_c;
    uint8_t index = 0;
    otaCmd_QueryImageReq_t *pQueryImgReq = (otaCmd_QueryImageReq_t *)pOtaClientInfo->pData;

    index = OtaServerStandalone_ValidateImage((uint16_t)(pQueryImgReq->manufacturerCode[0] + (pQueryImgReq->manufacturerCode[1] << 8)),
            (uint16_t)(pQueryImgReq->imageType[0] + (pQueryImgReq->imageType[1] << 8)),
            (uint32_t)(pQueryImgReq->fileVersion[0] + (pQueryImgReq->fileVersion[1] << 8) +
                       (pQueryImgReq->fileVersion[2] << 16) + (pQueryImgReq->fileVersion[3] << 24)), FALSE);

    if (index < gOtaServer_MaxOtaImages_c)
    {
        otaCmd_QueryImageRsp_t queryImgRsp = {gOtaCmd_QueryImageRsp_c, gOtaFileStatus_Success_c, {0x00}};
        /* image size */
        uint8_t len = sizeof(otaCmd_QueryImageRsp_t) - sizeof(queryImgRsp.data);

        FLib_MemCpy(&queryImgRsp.data.success.manufacturerCode, &mOtaServerImageList[index].manufCode, sizeof(uint16_t));
        FLib_MemCpy(&queryImgRsp.data.success.fileVersion, &mOtaServerImageList[index].fileVersion, sizeof(uint32_t));
        FLib_MemCpy(&queryImgRsp.data.success.imageType, &mOtaServerImageList[index].imageType, sizeof(uint16_t));
        FLib_MemCpy(&queryImgRsp.data.success.fileSize, &mOtaServerImageList[index].fileSize, sizeof(uint32_t));
        FLib_MemCpy(&queryImgRsp.data.success.serverDownloadPort, &mOtaServerSetup.downloadPort, sizeof(uint16_t));

        len += sizeof(otaCmd_QueryImageRspSuccess_t);
        (void)OtaServer_CoapSendRsp(pOtaClientInfo, (uint8_t *)&queryImgRsp, len);
        status = gOtaStatus_Success_c;
    }
    else
    {
        /* packet in progress -  send back a Query Image Rsp command with status no image available */
        OtaServer_CoapSendRspWaitAbortData(pOtaClientInfo, gOtaFileStatus_NoImageAvailable_c, 0);
        status = gOtaStatus_Success_c;
    }

    return status;
}

/*!*************************************************************************************************
\private
\fn     static otaStatus_t OtaServerStandalone_BlockReqHandler(otaClientInfo_t *pOtaClientInfo)
\brief  This function is used to process a BlockRequest command.

\param  [in]    pOtaClientInfo    Pointer to client info

\return         otaStatus_t       Status of the operation
***************************************************************************************************/
static otaStatus_t OtaServerStandalone_BlockReqHandler
(
    otaClientInfo_t *pOtaClientInfo
)
{
    otaStatus_t status = gOtaStatus_Failed_c;
    uint8_t index = gOtaServer_MaxOtaImages_c;
    otaCmd_BlockReq_t *pBlockReq = (otaCmd_BlockReq_t *)pOtaClientInfo->pData;

    index = OtaServerStandalone_ValidateImage((uint16_t)(pBlockReq->manufacturerCode[0] + (pBlockReq->manufacturerCode[1] << 8)),
            (uint16_t)(pBlockReq->imageType[0] + (pBlockReq->imageType[1] << 8)),
            (uint32_t)(pBlockReq->fileVersion[0] + (pBlockReq->fileVersion[1] << 8) +
                       (pBlockReq->fileVersion[2] << 16) + (pBlockReq->fileVersion[3] << 24)), FALSE);

    if (index < gOtaServer_MaxOtaImages_c)
    {
        otaCmd_BlockRsp_t *pBlockRsp = NULL;
        uint32_t respLength = 0;
        uint32_t len =  pBlockReq->maxDataSize;
        uint32_t imageOffset = (uint32_t)(pBlockReq->fileOffset[0] + (pBlockReq->fileOffset[1] << 8) +
                                          (pBlockReq->fileOffset[2] << 16) + (pBlockReq->fileOffset[3] << 24));

        if ((mOtaServerImageList[index].fileSize - imageOffset) < len)
        {
            len = mOtaServerImageList[index].fileSize - imageOffset;
        }

        respLength = 2 + sizeof(otaCmd_BlockRspSuccess_t) - 1 + len;
        pBlockRsp = MEM_BufferAlloc(respLength);

        if (pBlockRsp)
        {
            uint32_t addr = imageOffset + mOtaServerImageList[index].imageAddr;

            status = gOtaStatus_Success_c;
            pBlockRsp->commandId = gOtaCmd_BlockRsp_c;
            pBlockRsp->status = gOtaFileStatus_Success_c;
            FLib_MemCpy(pBlockRsp->data.success.fileVersion, pBlockReq->fileVersion, sizeof(uint32_t));
            FLib_MemCpy(pBlockRsp->data.success.fileOffset, &imageOffset, sizeof(uint32_t));
            pBlockRsp->data.success.dataSize = len;

            /* send info once at 100 packets */
            if (((imageOffset / len) % 100 == 0) || (imageOffset == 0))
            {
                uint16_t clientId = (pOtaClientInfo->remoteAddr.addr8[14] << 8) + pOtaClientInfo->remoteAddr.addr8[15];

                OTA_ClientInfoCnf((uint8_t *)&pOtaClientInfo->remoteAddr, imageOffset, clientId);
            }

            if (OTA_ReadExternalMemory(&pBlockRsp->data.success.pData[0], len, addr) != gOtaSucess_c)
            {
                (void)OtaServer_CoapSendRspWaitAbortData(pOtaClientInfo, gOtaFileStatus_Abort_c, 0);
            }
            else
            {
                OtaServer_SocketSendRsp(pOtaClientInfo, (uint8_t *)pBlockRsp, respLength);
            }

            MEM_BufferFree(pBlockRsp);
        }
    }
    else
    {
        /* image is not available, abort current session */
        status = gOtaStatus_Success_c;
        (void)OtaServer_CoapSendRspWaitAbortData(pOtaClientInfo, gOtaFileStatus_Abort_c, 0);
    }

    return status;
}

/*!*************************************************************************************************
\private
\fn     static otaStatus_t OtaServerStandalone_ServerDiscoveryHandler(otaClientInfo_t *pOtaClientInfo)
\brief  This function is used to process a ServerDiscovery command.

\param  [in]    pOtaClientInfo    Pointer to OTA client info

\return         otaStatus_t       Status of the operation
***************************************************************************************************/
static otaStatus_t OtaServerStandalone_ServerDiscoveryHandler
(
    otaClientInfo_t *pOtaClientInfo
)
{
    otaStatus_t status = gOtaStatus_Success_c;
    uint8_t index = gOtaServer_MaxOtaImages_c;
    otaCmd_ServerDiscovery_t *pCmdData = (otaCmd_ServerDiscovery_t *)pOtaClientInfo->pData;

    index = OtaServerStandalone_ValidateImage((uint16_t)(pCmdData->manufacturerCode[0] + (pCmdData->manufacturerCode[1] << 8)),
            (uint16_t)(pCmdData->imageType[0] + (pCmdData->imageType[1] << 8)),
            THR_ALL_FFs32, FALSE);

    if (index < gOtaServer_MaxOtaImages_c)
    {
        thciOtaServer_ImageNotify_t imageNotify;
        otaFileSubElement_t imageTag;

        if (OTA_ReadExternalMemory((uint8_t *)&imageTag, sizeof(imageTag), mOtaServerImageList[index].imageAddr + sizeof(otaFileHeader_t)) == gOtaSucess_c)
        {
            FLib_MemSet(&imageNotify, 0, sizeof(thciOtaServer_ImageNotify_t));
            FLib_MemCpy(&imageNotify.fileVersion, &mOtaServerImageList[index].fileVersion, sizeof(uint32_t));
            FLib_MemCpy(&imageNotify.imageType, &mOtaServerImageList[index].imageType, sizeof(uint16_t));
            FLib_MemCpy(&imageNotify.manufacturerCode, &mOtaServerImageList[index].manufCode, sizeof(uint16_t));
            FLib_MemCpy(&imageNotify.imageSize, &imageTag.length, sizeof(imageNotify.imageSize));
            FLib_MemCpy(&imageNotify.fileSize, &mOtaServerImageList[index].fileSize, sizeof(uint32_t));
            /* send back a unicast image notify command */
            (void)OtaServer_CoapSendImageNotify(&imageNotify, &pOtaClientInfo->remoteAddr);
        }
    }

    MEM_BufferFree(pOtaClientInfo);

    return status;
}

/*!*************************************************************************************************
\private
\fn     static bool_t OtaServerStandalone_FileSizeValidation(uint32_t newImageSize)
\brief  This function is used to validate the space required for storing the new image.

\param  [in]    newFileSize    Required file size

\return         bool_t          TRUE if file size is valid, FALSE otherwise
***************************************************************************************************/
static bool_t OtaServerStandalone_FileSizeValidation
(
    uint32_t newFileSize
)
{
    bool_t result = TRUE;
    uint32_t usedMemory = 0;

    for (uint8_t i = 0; i < gOtaServer_MaxOtaImages_c; i++)
    {
        if (mOtaServerImageList[i].isValidEntry)
        {
            usedMemory += mOtaServerImageList[i].fileSize;
        }
    }

    if (usedMemory + newFileSize > gFlashParams_MaxImageLength_c)
    {
        result = FALSE;
    }

    return result;
}

/*!*************************************************************************************************
\private
\fn     static uint8_t OtaServerStandalone_KeepImageInfo(uint16_t manufCode, uint16_t imageType,
                                                        uint32_t fileVersion, uint32_t fileSize)
\brief  This function is used to store image information in local table.

\param  [in]    manufCode      Manufacturer code
        [in]    imageType      Image type
        [in]    fileVersion    File version
        [in]    imageSize      Image size

\return         uint8_t        Image index if success, otherwise returns gOtaServer_MaxOtaImages_c
***************************************************************************************************/
static uint8_t OtaServerStandalone_KeepImageInfo
(
    uint16_t manufCode,
    uint16_t imageType,
    uint32_t fileVersion,
    uint32_t fileSize
)
{
    uint8_t result = gOtaServer_MaxOtaImages_c;
    uint32_t imageAddrOffset = 0;

    for (uint8_t i = 0; i < gOtaServer_MaxOtaImages_c; i++)
    {
        if (mOtaServerImageList[i].isValidEntry)
        {
            /* entry is valid, update image offset */
            imageAddrOffset += mOtaServerImageList[i].fileSize;
        }
        else
        {
            /* entry is free */
            mOtaServerImageList[i].fileVersion = fileVersion;
            mOtaServerImageList[i].imageAddr = imageAddrOffset;
            mOtaServerImageList[i].imageType = imageType;
            mOtaServerImageList[i].manufCode = manufCode;
            mOtaServerImageList[i].fileSize = fileSize;
            mOtaServerImageList[i].isValidEntry = FALSE; /* True when the download is completed */
            mOtaServerTempImageOffset = 0;
            result = i;
            break;
        }
    }

    return result;
}

/*!*************************************************************************************************
\private
\fn     static void OtaServerStandalone_ThciQueryImageReq(void *pParam)
\brief  Callback used to request any image, using the ota server thci protocol.

\param  [in]    pParam    Not used
***************************************************************************************************/
static void OtaServerStandalone_ThciQueryImageReq
(
    void *pParam
)
{
    /* wait the host application to confirm (see ThciOtaServer_QueryImageRspCnf) */
    OTA_QueryImageReq(THR_ALL_FFs16, THR_ALL_FFs16, THR_ALL_FFs16, THR_ALL_FFs32);

    (void) pParam;
}

/*!*************************************************************************************************
\private
\fn     static uint8_t OtaServerStandalone_ThciBlockReq(void *pParam)
\brief  Callback used to request a block of memory, using the ota server thci protocol.

\param  [in]    pParam    Not used
***************************************************************************************************/
static void OtaServerStandalone_ThciBlockReq
(
    void *pParam
)
{
    /* wait the host application to confirm (see ThciOtaServer_BlockReceivedCnf) */
    OTA_ImageChunkReq(mOtaServerTempImageOffset, gOtaMaxBlockDataSize_c, THR_ALL_FFs16);

    (void) pParam;
}
#endif /* gOtaServer_EnableStandaloneMode_c */

/*************************************************************************************************
*
*  OTA Server Serial Protocol callbacks
*
**************************************************************************************************/

/*!*************************************************************************************************
\private
\fn     static otaResult_t ThciOtaServer_ImageNotifyCnf(uint8_t *pBuffer, uint16_t len)
\brief  Process Image Notification command received from an external application.

\param  [in]    pBuffer        Pointer to data
\param  [in]    len            Data length

\return         otaResult_t    Result of the operation
***************************************************************************************************/
static otaResult_t ThciOtaServer_ImageNotifyCnf
(
    uint8_t *pBuffer,
    uint16_t len
)
{
    otaResult_t result = gOtaError_c;
    thciOtaServer_ImageNotify_t imageNotify;

    FLib_MemCpy((void *)&imageNotify, pBuffer, sizeof(imageNotify));

    if (mOtaServerSetup.opMode == gOtaServerOpMode_Dongle_c)
    {
        mOtaServerSetup.isActive = TRUE;
        /* Inform clients that a new image is available */
        result = (OtaServer_CoapSendImageNotify(&imageNotify, &in6addr_realmlocal_allthreadnodes) == gOtaStatus_Success_c) ? gOtaSucess_c : gOtaError_c;
    }

#if gOtaServer_EnableStandaloneMode_c
    else if (mOtaServerSetup.opMode == gOtaServerOpMode_Standalone_c)
    {
        uint16_t manufacturerCode, imageType;
        uint32_t fileVersion;

        result = gOtaSucess_c;
        FLib_MemCpy(&manufacturerCode, imageNotify.manufacturerCode, sizeof(uint16_t));
        FLib_MemCpy(&imageType, imageNotify.imageType, sizeof(uint16_t));
        FLib_MemCpy(&fileVersion, imageNotify.fileVersion, sizeof(uint32_t));

        /* Validate image params. If the image is already in the memory, ignore the command */
        if (OtaServerStandalone_ValidateImage(manufacturerCode, imageType,
                                              fileVersion, TRUE) == gOtaServer_MaxOtaImages_c)
        {
            /* start downloading the image */
            (void)NWKU_SendMsg(OtaServerStandalone_ThciQueryImageReq, NULL, mpOtaServerMsgQueue);
        }
    }

#endif

    return result;
}

/*!*************************************************************************************************
\private
\fn     static otaResult_t ThciOtaServer_SetOperationMode(uint8_t *pBuffer, uint16_t len)
\brief  Process set operation mode command received from an external application.

\param  [in]    pBuffer        Pointer to data
\param  [in]    len            Data length

\return         otaResult_t    Result of the operation
***************************************************************************************************/
static otaResult_t ThciOtaServer_SetOperationMode
(
    uint8_t *pBuffer,
    uint16_t len
)
{
    otaResult_t result = gOtaError_c;
    otaServerOpMode_t operationMode = (otaServerOpMode_t)pBuffer[0];

#if gOtaServer_EnableStandaloneMode_c

    if ((operationMode == gOtaServerOpMode_Standalone_c) ||
            (operationMode == gOtaServerOpMode_Dongle_c))
    {
        result = gOtaSucess_c;
        mOtaServerSetup.opMode = operationMode;

        if (mOtaServerSetup.opMode == gOtaServerOpMode_Standalone_c)
        {
            OtaServer_InitStandaloneOpMode();

            /* clear current image entries */
            for (uint8_t i = 0; i < gOtaServer_MaxOtaImages_c; i++)
            {
                mOtaServerImageList[i].isValidEntry = FALSE;
            }

            /* start downloading the image */
            (void)NWKU_SendMsg(OtaServerStandalone_ThciQueryImageReq, NULL, mpOtaServerMsgQueue);
        }
    }

#else

    if (operationMode == gOtaServerOpMode_Dongle_c)
    {
        mOtaServerSetup.opMode = (otaServerOpMode_t)operationMode;
        result = gOtaSucess_c;
    }

#endif

    /* avoid compiler warning */
    (void)len;

    return result;
}

/*!*************************************************************************************************
\private
\fn     static otaResult_t ThciOtaServer_QueryImageRspCnf(uint8_t *pBuffer, uint16_t len)
\brief  Process QueryImageRsp command received from an external application.

\param  [in]    pBuffer        Pointer to data
\param  [in]    len            Data length

\return         otaResult_t    Result of the operation
***************************************************************************************************/
static otaResult_t ThciOtaServer_QueryImageRspCnf
(
    uint8_t *pBuffer,
    uint16_t len
)
{
    bool_t status = FALSE;
    thciOtaServer_QueryImage_t *pThciQueryImg = (thciOtaServer_QueryImage_t *)pBuffer;
    uint16_t queryImgDeviceId = 0;

    /* Validate parameters */
    if ((len == 0) || (pBuffer == NULL))
    {
        return gOtaError_c;
    }

    /* validate operation mode */
    if (mOtaServerSetup.opMode == gOtaServerOpMode_Dongle_c)
    {
        otaCmd_QueryImageRsp_t *pQueryImgRsp;

        FLib_MemCpy(&queryImgDeviceId, pThciQueryImg->deviceId, sizeof(queryImgDeviceId));
        pQueryImgRsp = MEM_BufferAlloc(sizeof(otaCmd_QueryImageRsp_t));

        if (pQueryImgRsp)
        {
            pQueryImgRsp->commandId = gOtaCmd_QueryImageRsp_c;
            pQueryImgRsp->status = pThciQueryImg->imageStatus;
            /* copy manufacturer code, image type, file version and image size */
            FLib_MemCpy(&pQueryImgRsp->data.success.manufacturerCode[0], &pThciQueryImg->manufacturerCode,
                        sizeof(otaCmd_QueryImageRsp_t) - sizeof(pQueryImgRsp->commandId) - sizeof(pQueryImgRsp->status));
            FLib_MemCpy(&pQueryImgRsp->data.success.serverDownloadPort[0], &mOtaServerSetup.downloadPort, sizeof(uint16_t));

            status = NWKU_SendMsg(OtaServer_CoapSendQueryImageRsp, pQueryImgRsp, mpOtaServerMsgQueue);

            if (status == FALSE)
            {
                MEM_BufferFree(pQueryImgRsp);
            }
        }
        else
        {
            MEM_BufferFree(mpOtaServerTempClientInfo);
            mpOtaServerTempClientInfo = NULL;
            OtaServer_ProcessNextClient();
        }
    }

#if gOtaServer_EnableStandaloneMode_c
    else if (mOtaServerSetup.opMode == gOtaServerOpMode_Standalone_c)
    {
        if (pThciQueryImg->imageStatus == gOtaSucess_c)
        {
            /* validate memory */
            uint32_t totalImgSize;

            FLib_MemCpy(&totalImgSize, pThciQueryImg->totalImgSize, sizeof(uint32_t));

            if (OtaServerStandalone_FileSizeValidation(totalImgSize))
            {
                /* update server informations */
                uint16_t manufCode, imageType;
                uint32_t fileVersion, fileSize;

                FLib_MemCpy(&fileVersion, pThciQueryImg->fileVersion, sizeof(uint32_t));
                FLib_MemCpy(&imageType, pThciQueryImg->imageType, sizeof(uint16_t));
                FLib_MemCpy(&manufCode, pThciQueryImg->manufacturerCode, sizeof(uint16_t));
                FLib_MemCpy(&fileSize, pThciQueryImg->totalImgSize, sizeof(uint32_t));
                mOtaServerTempImageIdx = OtaServerStandalone_KeepImageInfo(manufCode, imageType, fileVersion, fileSize);

                if (mOtaServerTempImageIdx < gOtaServer_MaxOtaImages_c)
                {
                    /* send a first block Request */
                    status = NWKU_SendMsg(OtaServerStandalone_ThciBlockReq, NULL, mpOtaServerMsgQueue);
                    TMR_StartSingleShotTimer(mOtaServerSetup.otaServerTmr, gOtaServer_ThciBlockRequestTimeoutMs_c,
                                             OtaServer_BlockRequestRetryCb, NULL);
                }
            }
        }
    }

#endif

    return (status == TRUE) ? gOtaSucess_c : gOtaError_c;
}

/*!*************************************************************************************************
\private
\fn     static otaResult_t ThciOtaServer_BlockReceivedCnf(uint8_t *pBuffer, uint16_t len)
\brief  Process BlockReceived command received from an external application.

\param  [in]    pBuffer        Pointer to data
\param  [in]    len            Data length

\return         otaResult_t    Result of the operation
***************************************************************************************************/
static otaResult_t ThciOtaServer_BlockReceivedCnf
(
    uint8_t *pBuffer,
    uint16_t len
)
{
    bool_t status = FALSE;

    /* Validate parameters */
    if ((len == 0) || (pBuffer == NULL))
    {
        return gOtaError_c;
    }

    /* validate operation mode */
    if (mOtaServerSetup.opMode == gOtaServerOpMode_Dongle_c)
    {
        otaCmd_BlockRsp_t *pBlockRsp = NULL;

        if (NULL !=  mpOtaServerTempClientInfo)
        {
            pBlockRsp = MEM_BufferAlloc(2 + sizeof(otaCmd_BlockRspSuccess_t) - 1 + len);

            if (pBlockRsp)
            {
                pBlockRsp->commandId = gOtaCmd_BlockRsp_c;
                pBlockRsp->status = gOtaFileStatus_Success_c;
                otaCmd_BlockReq_t *pOtaCmd_BlockReq = (otaCmd_BlockReq_t *)mpOtaServerTempClientInfo->pData;

                FLib_MemCpy(pBlockRsp->data.success.fileVersion, pOtaCmd_BlockReq->fileVersion, sizeof(uint32_t));
                FLib_MemCpy(pBlockRsp->data.success.fileOffset, pOtaCmd_BlockReq->fileOffset, sizeof(uint32_t));
                pBlockRsp->data.success.dataSize = len;
                FLib_MemCpy(pBlockRsp->data.success.pData, pBuffer, len);

                status = NWKU_SendMsg(OtaServerDongle_SendBlockRsp, pBlockRsp, mpOtaServerMsgQueue);

                if (status == FALSE)
                {
                    MEM_BufferFree(pBlockRsp);
                }
                else
                {
                    TMR_StopTimer(mOtaServerSetup.otaServerTmr);
                }
            }
        }
    }

#if gOtaServer_EnableStandaloneMode_c
    else if (mOtaServerSetup.opMode == gOtaServerOpMode_Standalone_c)
    {
        TMR_StopTimer(mOtaServerSetup.otaServerTmr);
        mOtaServer_ThciBlockRequestRetryNo = gOtaServer_ThciBlockRequestRetryNo_c;

        /* Store image block to external memory */
        if (OTA_WriteExternalMemory(pBuffer, len, mOtaServerImageList[mOtaServerTempImageIdx].imageAddr +
                                    mOtaServerTempImageOffset) == gOtaSucess_c)
        {
            status = TRUE;
            mOtaServerTempImageOffset += len;

            if (mOtaServerTempImageOffset < mOtaServerImageList[mOtaServerTempImageIdx].fileSize)
            {
                /* request the next block*/
                status = NWKU_SendMsg(OtaServerStandalone_ThciBlockReq, NULL, mpOtaServerMsgQueue);
                TMR_StartSingleShotTimer(mOtaServerSetup.otaServerTmr, gOtaServer_ThciBlockRequestTimeoutMs_c,
                                         OtaServer_BlockRequestRetryCb, NULL);
            }
            else
            {
                thciOtaServer_ImageNotify_t imageNotify;
                otaFileSubElement_t imageTag;

                /* transfer completed */
                mOtaServerImageList[mOtaServerTempImageIdx].isValidEntry = TRUE;
                mOtaServerSetup.isActive = TRUE;

                if (OTA_ReadExternalMemory((uint8_t *)&imageTag, sizeof(imageTag), mOtaServerImageList[mOtaServerTempImageIdx].imageAddr + sizeof(otaFileHeader_t)) == gOtaSucess_c)
                {
                    /* Inform clients that a new image is available */
                    FLib_MemSet(&imageNotify, 0, sizeof(thciOtaServer_ImageNotify_t));
                    FLib_MemCpy(&imageNotify.fileVersion, &mOtaServerImageList[mOtaServerTempImageIdx].fileVersion, sizeof(uint32_t));
                    FLib_MemCpy(&imageNotify.imageType, &mOtaServerImageList[mOtaServerTempImageIdx].imageType, sizeof(uint16_t));
                    FLib_MemCpy(&imageNotify.manufacturerCode, &mOtaServerImageList[mOtaServerTempImageIdx].manufCode, sizeof(uint16_t));
                    FLib_MemCpy(&imageNotify.imageSize, &imageTag.length, sizeof(imageNotify.imageSize));
                    FLib_MemCpy(&imageNotify.fileSize, &mOtaServerImageList[mOtaServerTempImageIdx].fileSize, sizeof(uint32_t));

                    status = (OtaServer_CoapSendImageNotify(&imageNotify, &in6addr_realmlocal_allthreadnodes) == gOtaStatus_Success_c) ? TRUE : FALSE;
                }
            }
        }
    }

#endif

    return (status == TRUE) ? gOtaSucess_c : gOtaError_c;
}

/*!*************************************************************************************************
\private
\fn     static otaResult_t ThciOtaServer_CancelImgCnf(uint8_t *pBuffer, uint16_t len)
\brief  Process CancelImage command received from an external application.

\param  [in]    pBuffer        Pointer to data
\param  [in]    len            Data length

\return         otaResult_t    Result of the operation
***************************************************************************************************/
static otaResult_t ThciOtaServer_CancelImgCnf
(
    uint8_t *pBuffer,
    uint16_t len
)
{
    /* avoid compiler warning */
    (void)len;
    (void)pBuffer;

    mOtaServerSetup.isActive = FALSE;

    if (mOtaServerSetup.opMode == gOtaServerOpMode_Dongle_c)
    {
        if (mpOtaServerTempClientInfo)
        {
            MEM_BufferFree(mpOtaServerTempClientInfo);
            mpOtaServerTempClientInfo = NULL;
        }

        while (MSG_Pending(&mOtaServerThciClientInfoQueue))
        {
            void *pOtaClientInfo = MSG_DeQueue(&mOtaServerThciClientInfoQueue);
            MEM_BufferFree(pOtaClientInfo);
        }
    }

    return gOtaSucess_c;
}

/*!*************************************************************************************************
\private
\fn     static otaResult_t ThciOtaServer_AbortProcessCnf(uint8_t *pBuffer, uint16_t len)
\brief  Process AbortProcess command received from an external application.

\param  [in]    pBuffer        Pointer to data
\param  [in]    len            Data length

\return         otaResult_t    Result of the operation
***************************************************************************************************/
static otaResult_t ThciOtaServer_AbortProcessCnf
(
    uint8_t *pBuffer,
    uint16_t len
)
{
    otaResult_t status = gOtaError_c;
    uint16_t clientId;

    FLib_MemCpy(&clientId, pBuffer, sizeof(clientId));

    if (OtaServer_AddClientToAbortList(clientId))
    {
        status = gOtaSucess_c;
    }

    /* avoid compiler warning */
    (void)len;

    return status;
}

/*!*************************************************************************************************
\private
\fn     static otaResult_t ThciOtaServer_SetFileVersPoliciesCnf(uint8_t *pBuffer, uint16_t len)
\brief  Process setFileVersPolicies command received from an external application.

\param  [in]    pBuffer        Pointer to data
\param  [in]    len            Data length

\return         otaResult_t    Result of the operation
***************************************************************************************************/
static otaResult_t ThciOtaServer_SetFileVersPoliciesCnf
(
    uint8_t *pBuffer,
    uint16_t len
)
{
    mOtaServerSetup.fileVersionPolicy = pBuffer[0];
    /* avoid compiler warning */
    (void)len;

    return gOtaSucess_c;
}

/*!*************************************************************************************************
\private
\fn     static otaResult_t ThciOtaServer_ClientInfoCnf(uint8_t *pBuffer, uint16_t len)
\brief  Process getClientAddress command received from an external application.

\param  [in]    pBuffer        Pointer to data
\param  [in]    len            Data length

\return         otaResult_t    Result of the operation
***************************************************************************************************/
static otaResult_t ThciOtaServer_ClientInfoCnf
(
    uint8_t *pBuffer,
    uint16_t len
)
{
    uint16_t clientId = 0;

    if (len != 0)
    {
        FLib_MemCpy(&clientId, pBuffer, sizeof(uint16_t));
    }

    if (mpOtaServerTempClientInfo != NULL)
    {
        /* when sent on confirm, do not send information about progress */
        OTA_ClientInfoCnf((uint8_t *)&mpOtaServerTempClientInfo->remoteAddr, 0, clientId);
    }

    return gOtaSucess_c;
}

/*!*************************************************************************************************
\private
\fn     static otaStatus_t OtaServer_CheckThciQueue(otaClientInfo_t *pOtaClientInfo)
\brief  This function is used to check if we do not have already a request from this client in thci queue.

\param  [in]    pOtaClientInfo    Pointer to client info

\return         otaStatus_t       Status of the operation
***************************************************************************************************/
static otaStatus_t OtaServer_CheckThciQueue
(
    otaClientInfo_t *pOtaClientInfo
)
{
    otaStatus_t otaStatus = gOtaStatus_Success_c;

    if ((mOtaServerSetup.opMode == gOtaServerOpMode_Dongle_c))
    {
        otaClientInfo_t *pRequestClient = (otaClientInfo_t *)ListGetHeadMsg(&mOtaServerThciClientInfoQueue);

        while (pRequestClient)
        {
            if (IP_IsAddrEqual(&pRequestClient->remoteAddr, &pOtaClientInfo->remoteAddr))
            {
                /* we have a request from this client to the PC server that has not expired */
                otaStatus = gOtaStatus_Failed_c;
                break;
            }

            pRequestClient = (otaClientInfo_t *)ListGetNextMsg((listElementHandle_t)pRequestClient);
        }

        if ((NULL != mpOtaServerTempClientInfo) && (NULL != pRequestClient))
        {
            if (IP_IsAddrEqual(&mpOtaServerTempClientInfo->remoteAddr, &pOtaClientInfo->remoteAddr))
            {
                /* we have a request from this client to the PC server that has not expired */
                otaStatus = gOtaStatus_Failed_c;
            }
        }
    }

    return otaStatus;
}

/*!*************************************************************************************************
\private
\fn     static otaStatus_t OtaServer_ProcessNextClient(void)
\brief  This function is used to process next client command.

\return   otaStatus_t       Status of the operation
***************************************************************************************************/
static otaStatus_t OtaServer_ProcessNextClient
(
    void
)
{
    otaClientInfo_t *pOtaClientInfo = NULL;

    if ((mOtaServerSetup.opMode == gOtaServerOpMode_Dongle_c) &&
            (NULL == mpOtaServerTempClientInfo))
    {
        TMR_StopTimer(mOtaServerSetup.otaServerTmr);
        pOtaClientInfo = MSG_DeQueue(&mOtaServerThciClientInfoQueue);

        if (pOtaClientInfo)
        {
            if (FALSE == NWKU_SendMsg(OtaServer_CmdProcess, pOtaClientInfo, mpOtaServerMsgQueue))
            {
                MEM_BufferFree(pOtaClientInfo);

                /* Clean queue */
                while (MSG_Pending(&mOtaServerThciClientInfoQueue))
                {
                    pOtaClientInfo = MSG_DeQueue(&mOtaServerThciClientInfoQueue);
                    MEM_BufferFree(pOtaClientInfo);
                }
            }
        }
    }

    return gOtaStatus_Success_c;
}

/*!*************************************************************************************************
\private
\fn     static otaStatus_t OtaServer_HandleBlockSocket(bool_t onOff)
\brief  This function is used to handle block sockets.

\param  [in]   onOff          If TRUE, create and bind and if FALSE, shutdown socket

\return        otaStatus_t    Status of the operation
 ***************************************************************************************************/
static otaStatus_t OtaServer_HandleBlockSocket
(
    bool_t onOff
)
{
    otaStatus_t status = gOtaStatus_Success_c;
    sockaddrStorage_t portAddr;

    if (TRUE == onOff)
    {
        if (mOtaServerSetup.mOtaUdpSrvSockFd == -1)
        {
            /* Set Ota Server local address and local port */
            FLib_MemSet(&portAddr, 0, sizeof(sockaddrStorage_t));
            ((sockaddrIn6_t *)&portAddr)->sin6_family = AF_INET6;
            ((sockaddrIn6_t *)&portAddr)->sin6_port = OTA_SERVER_DEFAULT_PORT;
            //(void)THR_GetIP6Addr(0, OTA_ADDR_TYPE, &((sockaddrIn6_t *)&portAddr)->sin6_addr, NULL);
            IP_AddrCopy(&((sockaddrIn6_t *)&portAddr)->sin6_addr, &in6addr_any);

            /* Open Ota Server UDP socket */
            mOtaServerSetup.mOtaUdpSrvSockFd = socket(AF_INET6, SOCK_DGRAM, IPPROTO_UDP);

            if (mOtaServerSetup.mOtaUdpSrvSockFd != -1)
            {
                bind(mOtaServerSetup.mOtaUdpSrvSockFd, &portAddr, sizeof(sockaddrStorage_t));
                Session_RegisterCb(mOtaServerSetup.mOtaUdpSrvSockFd, OtaClient_UdpServerService, mpOtaServerMsgQueue);
            }
            else
            {
                panic(0, (uint32_t)OtaServer_HandleBlockSocket, 0, 0);
                status = gOtaStatus_Failed_c;
            }
        }
        else
        {
            status = gOtaStatus_Success_c;
        }
    }
    else
    {
        if (mOtaServerSetup.mOtaUdpSrvSockFd != -1)
        {
            Session_UnRegisterCb(mOtaServerSetup.mOtaUdpSrvSockFd);
            shutdown(mOtaServerSetup.mOtaUdpSrvSockFd, 0);
            mOtaServerSetup.mOtaUdpSrvSockFd = -1;
        }
    }

    return status;
}

#if 0
/*!*************************************************************************************************
\private
\fn     static otaStatus_t OtaServer_SendImgNtf(void *pParam)
\brief  This function is used for the retransmission of Image Notification commands.

\param  [in]    pParam          Pointer to the thciOtaServer_ImageNotify_t structure containing
                                the Image Notification information used for the OTA command.

\return         otaStatus_t     Status of the operation
***************************************************************************************************/
static otaStatus_t OtaServer_SendImgNtf
(
    void *pParam
)
{
    otaStatus_t status = gOtaStatus_Failed_c;

    if (pParam)
    {
        thciOtaServer_ImageNotify_t *pImageNotify = pParam;
        status = OtaServer_CoapSendImageNotify(pImageNotify, &in6addr_realmlocal_allthreadnodes);
    }

    return status;
}
#endif

#if gOtaServer_EnableStandaloneMode_c
/*!*************************************************************************************************
\private
\fn     static void OtaServer_BlockRequestRetryCb(void *pParam)
\brief  This function is used to send a msg to retry a block request in standalone OTA.

\param  [in]   pParam    Pointer to data
 ***************************************************************************************************/
static void OtaServer_BlockRequestRetryCb
(
    void *pParam
)
{
    if (mOtaServer_ThciBlockRequestRetryNo != 0)
    {
        mOtaServer_ThciBlockRequestRetryNo--;
        (void)NWKU_SendMsg(OtaServerStandalone_ThciBlockReq, NULL, mpOtaServerMsgQueue);
        TMR_StartSingleShotTimer(mOtaServerSetup.otaServerTmr, gOtaServer_ThciBlockRequestTimeoutMs_c,
                                 OtaServer_BlockRequestRetryCb, NULL);
    }
    else
    {
        ThciOtaServer_CancelImgCnf(NULL, 0);
    }
}
#endif
/*==================================================================================================
Private debug functions
==================================================================================================*/

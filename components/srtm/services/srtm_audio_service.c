/*
 * The Clear BSD License
 * Copyright (c) 2017-2018, NXP
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

#include <assert.h>
#include <string.h>

#include "fsl_common.h"

#include "srtm_heap.h"
#include "srtm_list.h"
#include "srtm_dispatcher.h"
#include "srtm_service.h"
#include "srtm_service_struct.h"
#include "srtm_audio_service.h"
#include "srtm_message.h"
#include "srtm_message_struct.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/* Protocol definition */
#define SRTM_AUDIO_CATEGORY (0x3U)

#define SRTM_AUDIO_VERSION (0x0102U)

#define SRTM_AUDIO_RETURN_CODE_SUCEESS (0x0U)
#define SRTM_AUDIO_RETURN_CODE_FAIL (0x1U)
#define SRTM_AUDIO_RETURN_CODE_UNSUPPORTED (0x2U)

/* Audio Service Request Command definition */
#define SRTM_AUDIO_CMD_TX_OPEN (0x0U)
#define SRTM_AUDIO_CMD_TX_START (0x1U)
#define SRTM_AUDIO_CMD_TX_PAUSE (0x2U)
#define SRTM_AUDIO_CMD_TX_RESTART (0x3U)
#define SRTM_AUDIO_CMD_TX_STOP (0x4U)
#define SRTM_AUDIO_CMD_TX_CLOSE (0x5U)
#define SRTM_AUDIO_CMD_TX_SET_PARAM (0x6U)
#define SRTM_AUDIO_CMD_TX_SET_BUF (0x7U)
#define SRTM_AUDIO_CMD_TX_SUSPEND (0x8U)
#define SRTM_AUDIO_CMD_TX_RESUME (0x9U)
#define SRTM_AUDIO_CMD_RX_OPEN (0xAU)
#define SRTM_AUDIO_CMD_RX_START (0xBU)
#define SRTM_AUDIO_CMD_RX_PAUSE (0xCU)
#define SRTM_AUDIO_CMD_RX_RESTART (0xDU)
#define SRTM_AUDIO_CMD_RX_STOP (0xEU)
#define SRTM_AUDIO_CMD_RX_CLOSE (0xFU)
#define SRTM_AUDIO_CMD_RX_SET_PARAM (0x10U)
#define SRTM_AUDIO_CMD_RX_SET_BUF (0x11U)
#define SRTM_AUDIO_CMD_RX_SUSPEND (0x12U)
#define SRTM_AUDIO_CMD_RX_RESUME (0x13U)
#define SRTM_AUDIO_CMD_SET_CODEC_REG (0x14U)
#define SRTM_AUDIO_CMD_GET_CODEC_REG (0x15U)
#define SRTM_AUDIO_CMD_TX_GET_BUF_OFFSET (0x16U)
#define SRTM_AUDIO_CMD_RX_GET_BUF_OFFSET (0x17U)

/* Audio Service Notification Command definition */
#define SRTM_AUDIO_NTF_TX_PERIOD_DONE (0x0U)
#define SRTM_AUDIO_NTF_RX_PERIOD_DONE (0x1U)

/* Audio Service Sample Format definition */
#define SRTM_AUDIO_SAMPLE_FORMAT_S16_LE (0x0U)
#define SRTM_AUDIO_SAMPLE_FORMAT_S24_LE (0x1U)

/* Audio Service Channel identifier definition */
#define SRTM_AUDIO_CHANNEL_LEFT (0x0U)
#define SRTM_AUDIO_CHANNEL_RIGHT (0x1U)
#define SRTM_AUDIO_CHANNEL_STEREO (0x2U)

/* Service handle */
typedef struct _srtm_audio_service
{
    struct _srtm_service service;
    srtm_sai_adapter_t sai;
    srtm_codec_adapter_t codec;
    /* Currently assume just one peer core, to support multiple peer cores, channel need to be a list */
    srtm_channel_t channel;
    srtm_list_t freeProcs;
} * srtm_audio_service_t;

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/
static void SRTM_AudioService_RecycleMessage(srtm_message_t msg, void *param)
{
    uint32_t primask;
    srtm_audio_service_t handle = (srtm_audio_service_t)param;

    /* Put message back to freeProcs */
    primask = DisableGlobalIRQ();
    SRTM_List_AddTail(&handle->freeProcs, &msg->node);
    EnableGlobalIRQ(primask);
}

/* CALLED IN SRTM DISPATCHER TASK */
static void SRTM_AudioService_HandlePeriodDone(srtm_dispatcher_t dispatcher,
                                               srtm_audio_dir_t dir,
                                               srtm_audio_service_t handle,
                                               uint32_t index)
{
    srtm_notification_t notif;
    struct _srtm_audio_payload *payload;

    SRTM_DEBUG_MESSAGE(SRTM_DEBUG_VERBOSE_INFO, "%s: %s\r\n", __func__, dir == SRTM_AudioDirTx ? "Tx" : "Rx");

    /* First allocate a notification and send to peer core */
    notif =
        SRTM_Notification_Create(handle->channel, SRTM_AUDIO_CATEGORY, SRTM_AUDIO_VERSION,
                                 dir == SRTM_AudioDirTx ? SRTM_AUDIO_NTF_TX_PERIOD_DONE : SRTM_AUDIO_NTF_RX_PERIOD_DONE,
                                 sizeof(struct _srtm_audio_payload));
    if (!notif)
    {
        SRTM_DEBUG_MESSAGE(SRTM_DEBUG_VERBOSE_ERROR, "%s: alloc %s notification failed.\r\n", __func__,
                           dir == SRTM_AudioDirTx ? "Tx" : "Rx");
        return;
    }

    payload = (struct _srtm_audio_payload *)SRTM_CommMessage_GetPayload(notif);
    payload->index = (uint8_t)(index >> 24U);
    payload->periodIdx = index & 0xFFFFFFU;

    SRTM_Dispatcher_DeliverNotification(handle->service.dispatcher, notif);
}

static void SRTM_AudioService_HandleTxPeriodDone(srtm_dispatcher_t dispatcher, void *param1, void *param2)
{
    SRTM_AudioService_HandlePeriodDone(dispatcher, SRTM_AudioDirTx, (srtm_audio_service_t)param1, (uint32_t)param2);
}

static void SRTM_AudioService_HandleRxPeriodDone(srtm_dispatcher_t dispatcher, void *param1, void *param2)
{
    SRTM_AudioService_HandlePeriodDone(dispatcher, SRTM_AudioDirRx, (srtm_audio_service_t)param1, (uint32_t)param2);
}

/* CALLED IN AUDIO DRIVER ISR */
static srtm_status_t SRTM_AudioService_PeriodDone(srtm_service_t service,
                                                  srtm_audio_dir_t dir,
                                                  uint8_t index,
                                                  uint32_t periodIdx)
{
    srtm_audio_service_t handle = (srtm_audio_service_t)service;
    uint32_t primask;
    srtm_list_t *list;
    srtm_procedure_t proc = NULL;

    primask = DisableGlobalIRQ();
    if (!SRTM_List_IsEmpty(&handle->freeProcs))
    {
        list = handle->freeProcs.next;
        SRTM_List_Remove(list);
        proc = SRTM_LIST_OBJ(srtm_procedure_t, node, list);
    }
    EnableGlobalIRQ(primask);

    assert(proc); /* For debugging the shortage of procedure messages */

    if (!proc)
    {
        return SRTM_Status_OutOfMemory;
    }
    else
    {
        proc->procMsg.cb =
            dir == SRTM_AudioDirTx ? SRTM_AudioService_HandleTxPeriodDone : SRTM_AudioService_HandleRxPeriodDone;
        proc->procMsg.param1 = service;
        proc->procMsg.param2 = (void *)((((uint32_t)index) << 24U) | (periodIdx & 0xFFFFFFU));
    }

    return SRTM_Dispatcher_PostProc(service->dispatcher, proc);
}

static uint16_t SRTM_AudioService_GetRespLen(uint8_t command)
{
    return sizeof(struct _srtm_audio_payload);
}

/* Both request and notify are called from SRTM dispatcher context */
static srtm_status_t SRTM_AudioService_Request(srtm_service_t service, srtm_request_t request)
{
    srtm_status_t status;
    srtm_audio_service_t handle = (srtm_audio_service_t)service;
    srtm_sai_adapter_t sai = handle->sai;
    srtm_codec_adapter_t codec = handle->codec;
    srtm_channel_t channel;
    uint8_t command;
    uint32_t payloadLen;
    srtm_response_t response;
    struct _srtm_audio_payload *audioReq;
    uint8_t *audioRespBuf;
    struct _srtm_audio_payload *audioResp;

    assert(sai);
    assert(service->dispatcher);

    SRTM_DEBUG_MESSAGE(SRTM_DEBUG_VERBOSE_INFO, "%s\r\n", __func__);

    channel = SRTM_CommMessage_GetChannel(request);
    command = SRTM_CommMessage_GetCommand(request);
    audioReq = (struct _srtm_audio_payload *)SRTM_CommMessage_GetPayload(request);
    payloadLen = SRTM_CommMessage_GetPayloadLen(request);

    response = SRTM_Response_Create(channel, SRTM_AUDIO_CATEGORY, SRTM_AUDIO_VERSION, command,
                                    SRTM_AudioService_GetRespLen(command));
    if (!response)
    {
        return SRTM_Status_OutOfMemory;
    }
    else
    {
        /* Remember the channel for future notification */
        handle->channel = channel;
    }

    audioRespBuf = (uint8_t *)SRTM_CommMessage_GetPayload(response);
    audioResp = (struct _srtm_audio_payload *)audioRespBuf;

    status = SRTM_Service_CheckVersion(service, request, SRTM_AUDIO_VERSION);
    if (status != SRTM_Status_Success || !audioReq || payloadLen != sizeof(struct _srtm_audio_payload))
    {
        SRTM_DEBUG_MESSAGE(SRTM_DEBUG_VERBOSE_WARN, "%s: format error!\r\n", __func__);
        audioRespBuf[0] = audioReq ? audioReq->index : 0;
        audioRespBuf[1] = SRTM_AUDIO_RETURN_CODE_UNSUPPORTED;
    }
    else
    {
        audioRespBuf[0] = audioReq->index;
        switch (command)
        {
            case SRTM_AUDIO_CMD_TX_OPEN:
                assert(sai->open);
                status = sai->open(sai, SRTM_AudioDirTx, audioReq->index);
                audioRespBuf[1] =
                    status == SRTM_Status_Success ? SRTM_AUDIO_RETURN_CODE_SUCEESS : SRTM_AUDIO_RETURN_CODE_FAIL;
                break;
            case SRTM_AUDIO_CMD_TX_START:
                assert(sai->start);
                status = sai->start(sai, SRTM_AudioDirTx, audioReq->index);
                audioRespBuf[1] =
                    status == SRTM_Status_Success ? SRTM_AUDIO_RETURN_CODE_SUCEESS : SRTM_AUDIO_RETURN_CODE_FAIL;
                break;
            case SRTM_AUDIO_CMD_TX_PAUSE:
                assert(sai->pause);
                status = sai->pause(sai, SRTM_AudioDirTx, audioReq->index);
                audioRespBuf[1] =
                    status == SRTM_Status_Success ? SRTM_AUDIO_RETURN_CODE_SUCEESS : SRTM_AUDIO_RETURN_CODE_FAIL;
                break;
            case SRTM_AUDIO_CMD_TX_RESTART:
                assert(sai->restart);
                status = sai->restart(sai, SRTM_AudioDirTx, audioReq->index);
                audioRespBuf[1] =
                    status == SRTM_Status_Success ? SRTM_AUDIO_RETURN_CODE_SUCEESS : SRTM_AUDIO_RETURN_CODE_FAIL;
                break;
            case SRTM_AUDIO_CMD_TX_STOP:
                assert(sai->stop);
                status = sai->stop(sai, SRTM_AudioDirTx, audioReq->index);
                audioRespBuf[1] =
                    status == SRTM_Status_Success ? SRTM_AUDIO_RETURN_CODE_SUCEESS : SRTM_AUDIO_RETURN_CODE_FAIL;
                break;
            case SRTM_AUDIO_CMD_TX_CLOSE:
                assert(sai->close);
                status = sai->close(sai, SRTM_AudioDirTx, audioReq->index);
                audioRespBuf[1] =
                    status == SRTM_Status_Success ? SRTM_AUDIO_RETURN_CODE_SUCEESS : SRTM_AUDIO_RETURN_CODE_FAIL;
                break;
            case SRTM_AUDIO_CMD_TX_SET_PARAM:
                assert(sai->setParam);
                assert(codec->setParam);
                status = sai->setParam(sai, SRTM_AudioDirTx, audioReq->index, audioReq->format, audioReq->channels,
                                       audioReq->srate);
                status = status == SRTM_Status_Success ?
                             codec->setParam(codec, audioReq->index, audioReq->format, audioReq->srate) :
                             status;
                audioRespBuf[1] =
                    status == SRTM_Status_Success ? SRTM_AUDIO_RETURN_CODE_SUCEESS : SRTM_AUDIO_RETURN_CODE_FAIL;
                break;
            case SRTM_AUDIO_CMD_TX_SET_BUF:
                assert(sai->setBuf);
                status = sai->setBuf(sai, SRTM_AudioDirTx, audioReq->index, (uint8_t *)audioReq->bufAddr,
                                     audioReq->bufSize, audioReq->periodSize, audioReq->periodIdx);
                audioRespBuf[1] =
                    status == SRTM_Status_Success ? SRTM_AUDIO_RETURN_CODE_SUCEESS : SRTM_AUDIO_RETURN_CODE_FAIL;
                break;
            case SRTM_AUDIO_CMD_TX_SUSPEND:
                assert(sai->suspend);
                status = sai->suspend(sai, SRTM_AudioDirTx, audioReq->index);
                audioRespBuf[1] =
                    status == SRTM_Status_Success ? SRTM_AUDIO_RETURN_CODE_SUCEESS : SRTM_AUDIO_RETURN_CODE_FAIL;
                break;
            case SRTM_AUDIO_CMD_TX_RESUME:
                assert(sai->resume);
                status = sai->resume(sai, SRTM_AudioDirTx, audioReq->index);
                audioRespBuf[1] =
                    status == SRTM_Status_Success ? SRTM_AUDIO_RETURN_CODE_SUCEESS : SRTM_AUDIO_RETURN_CODE_FAIL;
                break;
            case SRTM_AUDIO_CMD_RX_OPEN:
                assert(sai->open);
                status = sai->open(sai, SRTM_AudioDirRx, audioReq->index);
                audioRespBuf[1] =
                    status == SRTM_Status_Success ? SRTM_AUDIO_RETURN_CODE_SUCEESS : SRTM_AUDIO_RETURN_CODE_FAIL;
                break;
            case SRTM_AUDIO_CMD_RX_START:
                assert(sai->start);
                status = sai->start(sai, SRTM_AudioDirRx, audioReq->index);
                audioRespBuf[1] =
                    status == SRTM_Status_Success ? SRTM_AUDIO_RETURN_CODE_SUCEESS : SRTM_AUDIO_RETURN_CODE_FAIL;
                break;
            case SRTM_AUDIO_CMD_RX_PAUSE:
                assert(sai->pause);
                status = sai->pause(sai, SRTM_AudioDirRx, audioReq->index);
                audioRespBuf[1] =
                    status == SRTM_Status_Success ? SRTM_AUDIO_RETURN_CODE_SUCEESS : SRTM_AUDIO_RETURN_CODE_FAIL;
                break;
            case SRTM_AUDIO_CMD_RX_RESTART:
                assert(sai->restart);
                status = sai->restart(sai, SRTM_AudioDirRx, audioReq->index);
                audioRespBuf[1] =
                    status == SRTM_Status_Success ? SRTM_AUDIO_RETURN_CODE_SUCEESS : SRTM_AUDIO_RETURN_CODE_FAIL;
                break;
            case SRTM_AUDIO_CMD_RX_STOP:
                assert(sai->stop);
                status = sai->stop(sai, SRTM_AudioDirRx, audioReq->index);
                audioRespBuf[1] =
                    status == SRTM_Status_Success ? SRTM_AUDIO_RETURN_CODE_SUCEESS : SRTM_AUDIO_RETURN_CODE_FAIL;
                break;
            case SRTM_AUDIO_CMD_RX_CLOSE:
                assert(sai->close);
                status = sai->close(sai, SRTM_AudioDirRx, audioReq->index);
                audioRespBuf[1] =
                    status == SRTM_Status_Success ? SRTM_AUDIO_RETURN_CODE_SUCEESS : SRTM_AUDIO_RETURN_CODE_FAIL;
                break;
            case SRTM_AUDIO_CMD_RX_SET_PARAM:
                assert(sai->setParam);
                assert(codec->setParam);
                status = sai->setParam(sai, SRTM_AudioDirRx, audioReq->index, audioReq->format, audioReq->channels,
                                       audioReq->srate);
                status = status == SRTM_Status_Success ?
                             codec->setParam(codec, audioReq->index, audioReq->format, audioReq->srate) :
                             status;
                audioRespBuf[1] =
                    status == SRTM_Status_Success ? SRTM_AUDIO_RETURN_CODE_SUCEESS : SRTM_AUDIO_RETURN_CODE_FAIL;
                break;
            case SRTM_AUDIO_CMD_RX_SET_BUF:
                assert(sai->setBuf);
                status = sai->setBuf(sai, SRTM_AudioDirRx, audioReq->index, (uint8_t *)audioReq->bufAddr,
                                     audioReq->bufSize, audioReq->periodSize, audioReq->periodIdx);
                audioRespBuf[1] =
                    status == SRTM_Status_Success ? SRTM_AUDIO_RETURN_CODE_SUCEESS : SRTM_AUDIO_RETURN_CODE_FAIL;
                break;
            case SRTM_AUDIO_CMD_RX_SUSPEND:
                assert(sai->suspend);
                status = sai->suspend(sai, SRTM_AudioDirRx, audioReq->index);
                audioRespBuf[1] =
                    status == SRTM_Status_Success ? SRTM_AUDIO_RETURN_CODE_SUCEESS : SRTM_AUDIO_RETURN_CODE_FAIL;
                break;
            case SRTM_AUDIO_CMD_RX_RESUME:
                assert(sai->resume);
                status = sai->resume(sai, SRTM_AudioDirRx, audioReq->index);
                audioRespBuf[1] =
                    status == SRTM_Status_Success ? SRTM_AUDIO_RETURN_CODE_SUCEESS : SRTM_AUDIO_RETURN_CODE_FAIL;
                break;
            case SRTM_AUDIO_CMD_SET_CODEC_REG:
                assert(codec->setReg);
                status = codec->setReg(codec, audioReq->reg, audioReq->regVal);
                audioRespBuf[1] =
                    status == SRTM_Status_Success ? SRTM_AUDIO_RETURN_CODE_SUCEESS : SRTM_AUDIO_RETURN_CODE_FAIL;
                break;
            case SRTM_AUDIO_CMD_GET_CODEC_REG:
                assert(codec->getReg);
                audioResp->reg = audioReq->reg;
                status = codec->getReg(codec, audioReq->reg, &audioResp->regVal);
                audioResp->retCode =
                    status == SRTM_Status_Success ? SRTM_AUDIO_RETURN_CODE_SUCEESS : SRTM_AUDIO_RETURN_CODE_FAIL;
                break;
            case SRTM_AUDIO_CMD_TX_GET_BUF_OFFSET:
                assert(sai->getBufOffset);
                status = sai->getBufOffset(sai, SRTM_AudioDirTx, audioReq->index, &audioResp->bufOffset);
                audioResp->retCode =
                    status == SRTM_Status_Success ? SRTM_AUDIO_RETURN_CODE_SUCEESS : SRTM_AUDIO_RETURN_CODE_FAIL;
                break;
            case SRTM_AUDIO_CMD_RX_GET_BUF_OFFSET:
                assert(sai->getBufOffset);
                status = sai->getBufOffset(sai, SRTM_AudioDirRx, audioReq->index, &audioResp->bufOffset);
                audioResp->retCode =
                    status == SRTM_Status_Success ? SRTM_AUDIO_RETURN_CODE_SUCEESS : SRTM_AUDIO_RETURN_CODE_FAIL;
                break;
            default:
                SRTM_DEBUG_MESSAGE(SRTM_DEBUG_VERBOSE_WARN, "%s: command %d unsupported!\r\n", __func__, command);
                audioRespBuf[1] = SRTM_AUDIO_RETURN_CODE_UNSUPPORTED;
                break;
        }
    }

    return SRTM_Dispatcher_DeliverResponse(service->dispatcher, response);
}

static srtm_status_t SRTM_AudioService_Notify(srtm_service_t service, srtm_notification_t notif)
{
    srtm_status_t status;
    srtm_audio_service_t handle = (srtm_audio_service_t)service;
    srtm_sai_adapter_t sai = handle->sai;
    uint8_t command;
    struct _srtm_audio_payload *payload;
    uint32_t payloadLen;

    assert(sai);
    assert(service->dispatcher);

    SRTM_DEBUG_MESSAGE(SRTM_DEBUG_VERBOSE_INFO, "%s\r\n", __func__);

    command = SRTM_CommMessage_GetCommand(notif);
    payload = (struct _srtm_audio_payload *)SRTM_CommMessage_GetPayload(notif);
    payloadLen = SRTM_CommMessage_GetPayloadLen(notif);

    status = SRTM_Service_CheckVersion(service, notif, SRTM_AUDIO_VERSION);
    if (status != SRTM_Status_Success || !payload || payloadLen != sizeof(struct _srtm_audio_payload))
    {
        SRTM_DEBUG_MESSAGE(SRTM_DEBUG_VERBOSE_WARN, "%s: format error!\r\n", __func__);
        status = SRTM_Status_ServiceNotFound;
    }
    else
    {
        switch (command)
        {
            case SRTM_AUDIO_NTF_TX_PERIOD_DONE:
                if (sai->periodReady)
                {
                    status = sai->periodReady(sai, SRTM_AudioDirTx, payload->index, payload->periodIdx);
                }
                break;
            case SRTM_AUDIO_NTF_RX_PERIOD_DONE:
                if (sai->periodReady)
                {
                    status = sai->periodReady(sai, SRTM_AudioDirRx, payload->index, payload->periodIdx);
                }
                break;
            default:
                SRTM_DEBUG_MESSAGE(SRTM_DEBUG_VERBOSE_WARN, "%s: command %d unsupported!\r\n", __func__, command);
                break;
        }
    }

    return status;
}

srtm_service_t SRTM_AudioService_Create(srtm_sai_adapter_t sai, srtm_codec_adapter_t codec)
{
    srtm_audio_service_t handle;
    srtm_procedure_t proc;
    uint32_t i;

    SRTM_DEBUG_MESSAGE(SRTM_DEBUG_VERBOSE_INFO, "%s\r\n", __func__);

    handle = (srtm_audio_service_t)SRTM_Heap_Malloc(sizeof(struct _srtm_audio_service));
    assert(handle);

    sai->service = &handle->service;
    sai->periodDone = SRTM_AudioService_PeriodDone;
    handle->sai = sai;

    handle->codec = codec;

    SRTM_List_Init(&handle->service.node);
    handle->service.dispatcher = NULL;
    handle->service.category = SRTM_AUDIO_CATEGORY;
    handle->service.destroy = SRTM_AudioService_Destroy;
    handle->service.request = SRTM_AudioService_Request;
    handle->service.notify = SRTM_AudioService_Notify;

    handle->channel = NULL;

    /* Create procedure messages list to be used in ISR */
    SRTM_List_Init(&handle->freeProcs);
    for (i = 0; i < SRTM_AUDIO_SERVICE_CONFIG_PROC_NUMBER; i++)
    {
        proc = SRTM_Procedure_Create(NULL, NULL, NULL);
        assert(proc);
        SRTM_Message_SetFreeFunc(proc, SRTM_AudioService_RecycleMessage, handle);
        SRTM_List_AddTail(&handle->freeProcs, &proc->node);
    }

    return &handle->service;
}

void SRTM_AudioService_Destroy(srtm_service_t service)
{
    srtm_list_t *list;
    srtm_procedure_t proc;
    srtm_audio_service_t handle = (srtm_audio_service_t)service;

    assert(service);

    SRTM_DEBUG_MESSAGE(SRTM_DEBUG_VERBOSE_INFO, "%s\r\n", __func__);

    /* Service must be unregistered from dispatcher before destroy */
    assert(SRTM_List_IsEmpty(&service->node));

    while (!SRTM_List_IsEmpty(&handle->freeProcs))
    {
        list = handle->freeProcs.next;
        SRTM_List_Remove(list);
        proc = SRTM_LIST_OBJ(srtm_procedure_t, node, list);
        SRTM_Message_SetFreeFunc(proc, NULL, NULL);
        SRTM_Message_Destroy(proc);
    }

    SRTM_Heap_Free(handle);
}

void SRTM_AudioService_Reset(srtm_service_t service, srtm_peercore_t core)
{
    srtm_audio_service_t handle = (srtm_audio_service_t)service;

    assert(service);

    SRTM_DEBUG_MESSAGE(SRTM_DEBUG_VERBOSE_INFO, "%s\r\n", __func__);

    /* Currently assume just one peer core, need to stop all audio activity. */
    handle->sai->stop(handle->sai, SRTM_AudioDirRx, 0);
    handle->sai->stop(handle->sai, SRTM_AudioDirTx, 0);
    handle->sai->close(handle->sai, SRTM_AudioDirRx, 0);
    handle->sai->close(handle->sai, SRTM_AudioDirTx, 0);

    handle->channel = NULL;
}

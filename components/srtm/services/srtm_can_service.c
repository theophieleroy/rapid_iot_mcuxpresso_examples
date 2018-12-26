/*
 * The Clear BSD License
 * Copyright (c) 2018, NXP
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
#include "srtm_can_service.h"

#include "srtm_heap.h"
#include "srtm_list.h"
#include "srtm_dispatcher.h"
#include "srtm_service.h"
#include "srtm_service_struct.h"
#include "srtm_channel.h"
#include "srtm_channel_struct.h"
#include "srtm_peercore.h"
#include "srtm_peercore_struct.h"
#include "srtm_message.h"
#include "srtm_message_struct.h"
#include "srtm_mutex.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/* Protocol definition */
#define SRTM_CAN_CATEGORY (0x8U)

#define SRTM_CAN_VERSION (0x0100U)

#define SRTM_CAN_RETURN_CODE_SUCEESS (0x0U)
#define SRTM_CAN_RETURN_CODE_FAIL (0x1U)
#define SRTM_CAN_RETURN_CODE_UNSUPPORTED (0x2U)

/* Can Service Request Command definition */
#define SRTM_CAN_CMD_REGISTER (0x0U)
#define SRTM_CAN_CMD_UNREGISTER (0x1U)
#define SRTM_CAN_CMD_SEND_EVENT (0x2U)

/* Can Service Notification Command definition */

/* Can Service Index definition */
#define SRTM_CAN_INDEX_ANY (0xFFFFFFFFU)

/* Can event register */
typedef struct _strm_can_client
{
    srtm_list_t node;
    srtm_channel_t channel;
    uint32_t index;
    uint32_t userParam;
} * srtm_can_client_t;

/* Service handle */
typedef struct _srtm_can_service
{
    struct _srtm_service service;
    srtm_can_adapter_t can;
    srtm_list_t clients;
    bool listLocked; /* During sending CAN event, the client list will be locked and
                        any register/unregister operation will fail. */
    srtm_mutex_t mutex;
} * srtm_can_service_t;

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/
static srtm_status_t SRTM_CanService_Register(srtm_service_t service,
                                              uint32_t index,
                                              srtm_channel_t channel,
                                              uint32_t userParam)
{
    srtm_can_service_t handle = (srtm_can_service_t)service;
    srtm_can_adapter_t can = handle->can;
    srtm_can_client_t client;
    srtm_list_t *list;
    srtm_status_t status = SRTM_Status_Success;

    SRTM_Mutex_Lock(handle->mutex);

    if (handle->listLocked)
    {
        status = SRTM_Status_ListAddFailed;
    }
    else
    {
        /* First find out if client already registered */
        for (list = handle->clients.next; list != &handle->clients; list = list->next)
        {
            client = SRTM_LIST_OBJ(srtm_can_client_t, node, list);
            if (client->channel == channel && client->index == index && client->userParam == userParam)
            {
                break;
            }
        }

        if (list == &handle->clients)
        {
            /* Not found */
            client = (srtm_can_client_t)SRTM_Heap_Malloc(sizeof(struct _strm_can_client));
            if (!client)
            {
                status = SRTM_Status_OutOfMemory;
            }
            else
            {
                /* Notify application that there's a new client registered */
                if (can->registerEvent)
                {
                    status = can->registerEvent(can, index, userParam);
                }

                if (status != SRTM_Status_Success)
                {
                    SRTM_Heap_Free(client);
                }
                else
                {
                    client->index = index;
                    client->channel = channel;
                    client->userParam = userParam;
                    SRTM_List_AddTail(&handle->clients, &client->node);
                }
            }
        }
        else
        {
            SRTM_DEBUG_MESSAGE(SRTM_DEBUG_VERBOSE_WARN, "%s: already registered\r\n", __func__);
            status = SRTM_Status_Error;
        }
    }

    SRTM_Mutex_Unlock(handle->mutex);

    return status;
}

static srtm_status_t SRTM_CanService_Unregister(srtm_service_t service,
                                                uint32_t index,
                                                srtm_channel_t channel,
                                                uint32_t userParam)
{
    srtm_can_service_t handle = (srtm_can_service_t)service;
    srtm_can_adapter_t can = handle->can;
    srtm_can_client_t client;
    srtm_list_t *list;
    srtm_status_t status = SRTM_Status_Success;

    SRTM_Mutex_Lock(handle->mutex);

    if (handle->listLocked)
    {
        status = SRTM_Status_ListRemoveFailed;
    }
    else
    {
        /* First find out if client already registered */
        for (list = handle->clients.next; list != &handle->clients; list = list->next)
        {
            client = SRTM_LIST_OBJ(srtm_can_client_t, node, list);
            if (client->channel == channel && client->index == index && client->userParam == userParam)
            {
                break;
            }
        }

        if (list == &handle->clients)
        {
            /* Not found */
            status = SRTM_Status_Error;
        }
        else
        {
            /* Notify application that there's a new client registered */
            if (can->unregisterEvent)
            {
                status = can->unregisterEvent(can, index, userParam);
            }

            if (status == SRTM_Status_Success)
            {
                SRTM_List_Remove(&client->node);
                SRTM_Heap_Free(client);
            }
        }
    }

    SRTM_Mutex_Unlock(handle->mutex);

    return status;
}

/* Both request and notify are called from SRTM dispatcher context */
static srtm_status_t SRTM_CanService_Request(srtm_service_t service, srtm_request_t request)
{
    srtm_status_t status;
    srtm_channel_t channel;
    uint8_t command;
    uint32_t payloadLen;
    srtm_response_t response;
    struct _srtm_can_payload *canReq;
    struct _srtm_can_payload *canResp;

    assert(service->dispatcher);

    SRTM_DEBUG_MESSAGE(SRTM_DEBUG_VERBOSE_INFO, "%s\r\n", __func__);

    channel = SRTM_CommMessage_GetChannel(request);
    command = SRTM_CommMessage_GetCommand(request);
    canReq = (struct _srtm_can_payload *)SRTM_CommMessage_GetPayload(request);
    payloadLen = SRTM_CommMessage_GetPayloadLen(request);

    response =
        SRTM_Response_Create(channel, SRTM_CAN_CATEGORY, SRTM_CAN_VERSION, command, sizeof(struct _srtm_can_payload));
    if (!response)
    {
        return SRTM_Status_OutOfMemory;
    }

    canResp = (struct _srtm_can_payload *)SRTM_CommMessage_GetPayload(response);

    status = SRTM_Service_CheckVersion(service, request, SRTM_CAN_VERSION);
    if (status != SRTM_Status_Success || !canReq || payloadLen != sizeof(struct _srtm_can_payload))
    {
        SRTM_DEBUG_MESSAGE(SRTM_DEBUG_VERBOSE_WARN, "%s: format error!\r\n", __func__);
        canResp->index = canReq ? canReq->index : 0;
        canResp->userParam = canReq ? canReq->userParam : 0;
        canResp->retCode = SRTM_CAN_RETURN_CODE_UNSUPPORTED;
    }
    else
    {
        canResp->index = canReq->index;
        canResp->userParam = canReq->userParam;
        switch (command)
        {
            case SRTM_CAN_CMD_REGISTER:
                status = SRTM_CanService_Register(service, canReq->index, channel, canReq->userParam);
                canResp->retCode =
                    status == SRTM_Status_Success ? SRTM_CAN_RETURN_CODE_SUCEESS : SRTM_CAN_RETURN_CODE_FAIL;
                break;
            case SRTM_CAN_CMD_UNREGISTER:
                status = SRTM_CanService_Unregister(service, canReq->index, channel, canReq->userParam);
                canResp->retCode =
                    status == SRTM_Status_Success ? SRTM_CAN_RETURN_CODE_SUCEESS : SRTM_CAN_RETURN_CODE_FAIL;
                break;
            default:
                SRTM_DEBUG_MESSAGE(SRTM_DEBUG_VERBOSE_WARN, "%s: command %d unsupported!\r\n", __func__, command);
                canResp->retCode = SRTM_CAN_RETURN_CODE_UNSUPPORTED;
                break;
        }
    }

    return SRTM_Dispatcher_DeliverResponse(service->dispatcher, response);
}

static srtm_status_t SRTM_CanService_Notify(srtm_service_t service, srtm_notification_t notif)
{
    SRTM_DEBUG_MESSAGE(SRTM_DEBUG_VERBOSE_WARN, "%s: command %d unsupported!\r\n", __func__,
                       SRTM_CommMessage_GetCommand(notif));

    return SRTM_Status_ServiceNotFound;
}

static void SRTM_CanService_Cleanup(srtm_can_service_t handle, srtm_peercore_t core)
{
    srtm_can_client_t client;
    srtm_list_t *list, *next;

    SRTM_Mutex_Lock(handle->mutex);
    /* If cleanup is called when the CAN event sending is ongoing, then there must be application issue. */
    assert(!handle->listLocked);
    for (list = handle->clients.next; list != &handle->clients; list = next)
    {
        next = list->next;
        client = SRTM_LIST_OBJ(srtm_can_client_t, node, list);
        assert(client->channel);
        if (client->channel->core == core || !core)
        {
            SRTM_List_Remove(list);
            SRTM_Heap_Free(client);
        }
    }
    SRTM_Mutex_Unlock(handle->mutex);
}

srtm_service_t SRTM_CanService_Create(srtm_can_adapter_t can)
{
    srtm_can_service_t handle;

    SRTM_DEBUG_MESSAGE(SRTM_DEBUG_VERBOSE_INFO, "%s\r\n", __func__);

    handle = (srtm_can_service_t)SRTM_Heap_Malloc(sizeof(struct _srtm_can_service));
    assert(handle);

    can->service = &handle->service;
    handle->can = can;
    handle->listLocked = false;
    handle->mutex = SRTM_Mutex_Create();
    assert(handle->mutex);
    SRTM_List_Init(&handle->clients);

    SRTM_List_Init(&handle->service.node);
    handle->service.dispatcher = NULL;
    handle->service.category = SRTM_CAN_CATEGORY;
    handle->service.destroy = SRTM_CanService_Destroy;
    handle->service.request = SRTM_CanService_Request;
    handle->service.notify = SRTM_CanService_Notify;

    return &handle->service;
}

void SRTM_CanService_Destroy(srtm_service_t service)
{
    srtm_can_service_t handle = (srtm_can_service_t)service;

    assert(service);

    SRTM_DEBUG_MESSAGE(SRTM_DEBUG_VERBOSE_INFO, "%s\r\n", __func__);

    /* Service must be unregistered from dispatcher before destroy */
    assert(SRTM_List_IsEmpty(&service->node));

    SRTM_CanService_Cleanup(handle, NULL);

    SRTM_Mutex_Destroy(handle->mutex);

    SRTM_Heap_Free(handle);
}

void SRTM_CanService_Reset(srtm_service_t service, srtm_peercore_t core)
{
    srtm_can_service_t handle = (srtm_can_service_t)service;

    assert(service);

    SRTM_DEBUG_MESSAGE(SRTM_DEBUG_VERBOSE_INFO, "%s\r\n", __func__);

    SRTM_CanService_Cleanup(handle, core);
}

srtm_status_t SRTM_CanService_SendEvent(srtm_service_t service,
                                        uint32_t index,
                                        srtm_can_event_t event,
                                        uint32_t timeout)
{
    srtm_can_service_t handle = (srtm_can_service_t)service;
    srtm_can_client_t client;
    srtm_list_t *list;
    srtm_request_t req;
    srtm_response_t resp;
    struct _srtm_can_payload *canReq;
    struct _srtm_can_payload *canResp;
    srtm_status_t status;
    uint32_t payloadLen;
    uint32_t failureNum = 0;
    uint32_t successNum = 0;

    assert(service);
    assert(timeout > 0);

    SRTM_DEBUG_MESSAGE(SRTM_DEBUG_VERBOSE_INFO, "%s: index %d, event %d\r\n", __func__, index, event);

    req = SRTM_Request_Create(NULL, SRTM_CAN_CATEGORY, SRTM_CAN_VERSION, SRTM_CAN_CMD_SEND_EVENT,
                              sizeof(struct _srtm_can_payload));
    if (!req)
    {
        return SRTM_Status_OutOfMemory;
    }

    canReq = (struct _srtm_can_payload *)SRTM_CommMessage_GetPayload(req);
    canReq->event = (uint32_t)event;

    SRTM_Mutex_Lock(handle->mutex);
    handle->listLocked = true;
    SRTM_Mutex_Unlock(handle->mutex);

    for (list = handle->clients.next; list != &handle->clients; list = list->next)
    {
        client = SRTM_LIST_OBJ(srtm_can_client_t, node, list);
        if (client->index == SRTM_CAN_INDEX_ANY || client->index == index)
        {
            /* Matched client */
            req->channel = client->channel;
            canReq->index = client->index;
            canReq->userParam = client->userParam;
            resp = NULL;
            status = SRTM_Dispatcher_Request(client->channel->core->dispatcher, req, &resp, timeout);
            if (status == SRTM_Status_Success)
            {
                assert(resp);
                canResp = (struct _srtm_can_payload *)SRTM_CommMessage_GetPayload(resp);
                payloadLen = SRTM_CommMessage_GetPayloadLen(resp);
                assert(payloadLen >= sizeof(struct _srtm_can_payload));
                status = canResp->retCode == SRTM_CAN_RETURN_CODE_SUCEESS ? SRTM_Status_Success : SRTM_Status_Error;
                SRTM_Response_Destroy(resp);
            }

            if (status != SRTM_Status_Success)
            {
                failureNum++;
            }
            else
            {
                successNum++;
            }
        }
    }

    SRTM_Mutex_Lock(handle->mutex);
    handle->listLocked = false;
    SRTM_Mutex_Unlock(handle->mutex);

    SRTM_Request_Destroy(req);

    return (failureNum == 0 && successNum > 0) ? SRTM_Status_Success : SRTM_Status_Error;
}

/*!
* \file
* This is a source file for event_manager.c
*
* Copyright (c) 2014 - 2015, Freescale Semiconductor, Inc.
* Copyright 2016-2017 NXP
*
* NXP Confidential Proprietary
*
* No part of this document must be reproduced in any form - including copied, transcribed, printed
* or by any electronic means - without specific written permission from NXP.
*
*/

/*==================================================================================================
Include Files
==================================================================================================*/
#include "event_manager.h"
#include "network_utils.h"
#include "FunctionLib.h"
#include "nvm_adapter.h"
#include "thread_app_callbacks.h"
#include "stack_config.h"

/*==================================================================================================
Private macros
==================================================================================================*/
/*! Event code mask */
#define EVM_EVENT_SET_MASK      0xFFFF0000

/*==================================================================================================
Private type definitions
==================================================================================================*/

/*==================================================================================================
Private prototypes
==================================================================================================*/

static void EVM_SendEvent(eventManagerEntry_t *pEvent, uint32_t code, void *pEventData, uint16_t dataSize,
                          instanceId_t instanceId, bool_t sendEvMonitor);

/*==================================================================================================
Private global variables declarations
==================================================================================================*/


/*==================================================================================================
Public global variables declarations
==================================================================================================*/
/* Pointer to event monitoring function */
pfFunction_t pfEventMonitoringFunction = NULL;
/*==================================================================================================
Public functions
==================================================================================================*/
/*!*************************************************************************************************
\fn    void EVM_EventNotify(uint32_t code, void *pEventData, uint16_t dataSize, instanceId_t instanceId)
\brief Notifies the event manager of a an event that occurred.

\param [in]   code          event code
\param [in]   pEventData    pointer to the event data. NOTE: Should be pointer to static data not
                            dynamically allocated memory. The Event manager module will NOT free it.
\param [in]   dataSize      the size in bytes of the event data
\param [in]   instanceId    instance Id of the module that is signaling the event
                            Possible values: - instance Id value (if the module has an instance Id)
                                             - gEvmAnyInstanceId_c (if the module does not have an instance Id)
***************************************************************************************************/
void EVM_EventNotify
(
    uint32_t code,
    void *pEventData,
    uint16_t dataSize,
    instanceId_t instanceId
)
{
    eventManagerEntry_t *pEntry = (eventManagerEntry_t *)&aStaticEventsTable[0];
    bool_t sendEventMonitor = TRUE;


    /* Search for notifications in the static table */
    while (pEntry < &aStaticEventsTable[aStaticEventsTableSize])
    {
        if (((code == pEntry->code)                                 /* Entry for a single event */
                || ((code & EVM_EVENT_SET_MASK) == pEntry->code))   /* Entry for the whole event set */
                && ((instanceId == pEntry->instanceId)              /* Match instance Id */
                    || (gEvmAnyInstanceId_c == pEntry->instanceId)
                    || (gEvmAnyInstanceId_c == instanceId)))
        {
#if DEBUG
            /* Message Queue MUST be present for Asynchronous Notifications */
            if (((TRUE == pEntry->bNotifyAsync)
                    && (NULL != pEntry->ppMsgQueue)
                    && (NULL != *(pEntry->ppMsgQueue)))
                    || (FALSE == pEntry->bNotifyAsync))
#endif
            {
                if ((gEvmAnyInstanceId_c == instanceId) &&
                        (gEvmAnyInstanceId_c != pEntry->instanceId))
                {
                    instanceId = pEntry->instanceId;
                }

                EVM_SendEvent(pEntry, code, pEventData, dataSize, instanceId, sendEventMonitor);
                sendEventMonitor = FALSE;

                if (pEventData)
                {
                    /*
                        Pass the buffer only to the first callback to avoid the
                        same buffer being used in multiple places.
                    */
                    break;
                }
            }
        }

        pEntry++;
    }
}
/*==================================================================================================
Private functions
==================================================================================================*/
/*!*************************************************************************************************
\private
\fn    void EVM_SendEvent(eventManagerEntry_t *pEvent, uint32_t code, void *pEventData, uint16_t dataSize,
                          instanceId_t instanceId, bool_t sendEventMonitor)
\brief Send the event to the registered callback, asynchronous t0 a task queue or synchronous.

\param [in]   pEvent              pointer to event manager entry structure
\param [in]   code                event code
\param [in]   pEventData          pointer to the event data
\param [in]   dataSize            the size in bytes of the event data
\param [in]   instanceId          instance Id of the module that is signaling the event
\param [in]   sendEventMonitor    if TURE send the event to the event monitor callback
***************************************************************************************************/
static void EVM_SendEvent
(
    eventManagerEntry_t *pEvent,
    uint32_t code,
    void *pEventData,
    uint16_t dataSize,
    instanceId_t instanceId,
    bool_t sendEventMonitor
)
{
    evmParams_t *pEvt = NULL;

    /* Copy the event data to be sent to the message handling function */
    pEvt = NWKU_MEM_BufferAlloc(sizeof(evmParams_t));

    if (pEvt)
    {
        pEvt->code = code;
        pEvt->instanceId = instanceId;
        pEvt->buffSize = dataSize;
        pEvt->pBuff = pEventData;

        if (pfEventMonitoringFunction && sendEventMonitor)
        {
            pfEventMonitoringFunction(pEvt);
        }

        if (pEvent->bNotifyAsync)
        {
            if (FALSE == NWKU_SendMsg(pEvent->pfFunction, pEvt, *(pEvent->ppMsgQueue)))
            {
                MEM_BufferFree(pEvt);
                APP_CriticalExitCb((uint32_t)EVM_SendEvent, FALSE);
            }
        }
        else
        {
            pEvent->pfFunction(pEvt);
        }
    }
    else
    {
        APP_CriticalExitCb((uint32_t)EVM_SendEvent, FALSE);
    }
}
/*==================================================================================================
Private debug functions
==================================================================================================*/

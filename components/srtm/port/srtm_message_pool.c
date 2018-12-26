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

#include "srtm_message_pool.h"
#include "srtm_heap.h"
#include "srtm_list.h"
#include "fsl_common.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/* Simple algorithm:
 * Here we suppose most SRTM messages data are small.
 * By default we set each message buffer to 96 (0x60) bytes (including struct _srtm_message
 * which occupies 52 bytes). So we have 44 bytes for the SRTM message data (10bytes header +
 * 34 bytes payload which is sufficient for all current SRTM category).
 */
/* Total buffer size for messages in the pool. */
#ifndef SRTM_MESSAGE_POOL_SIZE
#define SRTM_MESSAGE_POOL_SIZE (0x1000)
#endif

/* Each message buffer size */
#ifndef SRTM_MESSAGE_BUF_SIZE
#define SRTM_MESSAGE_BUF_SIZE (0x60)
#endif

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
/* Define a structure to hold SRTM_MESSAGE_BUF_SIZE buffer. */
typedef struct
{
    srtm_list_t node;
    uint8_t buf[SRTM_MESSAGE_BUF_SIZE - sizeof(srtm_list_t)];
} srtm_message_buf_t;

/*******************************************************************************
 * Variables
 ******************************************************************************/
static srtm_message_buf_t srtmMsgs[SRTM_MESSAGE_POOL_SIZE / sizeof(srtm_message_buf_t)];
static srtm_list_t srtmMsgList;
#ifdef SRTM_DEBUG_MESSAGE_FUNC
/* Used for probe current and minimum free message buffer count in debugger. */
static uint32_t freeMsgCount;
static uint32_t minFreeMsgCount;
#endif

/*******************************************************************************
 * Code
 ******************************************************************************/
void *SRTM_MessagePool_Alloc(uint32_t size)
{
    uint32_t i;
    void *buf;
    uint32_t primask;

    if (!srtmMsgList.next)
    {
        primask = DisableGlobalIRQ();
        if (!srtmMsgList.next)
        {
            /* Message list not initialized, initialize now */
            SRTM_List_Init(&srtmMsgList);
            for (i = 0; i < sizeof(srtmMsgs) / sizeof(srtm_message_buf_t); i++)
            {
                SRTM_List_AddTail(&srtmMsgList, &srtmMsgs[i].node);
            }
#ifdef SRTM_DEBUG_MESSAGE_FUNC
            freeMsgCount = sizeof(srtmMsgs) / sizeof(srtm_message_buf_t);
            minFreeMsgCount = freeMsgCount;
#endif
        }
        EnableGlobalIRQ(primask);
    }

    if (size > sizeof(srtm_message_buf_t))
    {
        SRTM_DEBUG_MESSAGE(SRTM_DEBUG_VERBOSE_INFO,
                           "Message size larger than SRTM_MESSAGE_BUF_SIZE %d, allocated in heap.\r\n",
                           SRTM_MESSAGE_BUF_SIZE);
        buf = SRTM_Heap_Malloc(size);
    }
    else
    {
        primask = DisableGlobalIRQ();
        if (SRTM_List_IsEmpty(&srtmMsgList))
        {
            EnableGlobalIRQ(primask);
            SRTM_DEBUG_MESSAGE(SRTM_DEBUG_VERBOSE_WARN, "Message pool (size %d) used up, allocated in heap.\r\n",
                               SRTM_MESSAGE_POOL_SIZE);
            buf = SRTM_Heap_Malloc(size);
        }
        else
        {
            buf = (void *)srtmMsgList.next;
            SRTM_List_Remove(srtmMsgList.next);
#ifdef SRTM_DEBUG_MESSAGE_FUNC
            freeMsgCount--;
            if (freeMsgCount < minFreeMsgCount)
            {
                minFreeMsgCount = freeMsgCount;
            }
#endif
            EnableGlobalIRQ(primask);
        }
    }

    return buf;
}

void SRTM_MessagePool_Free(void *buf)
{
    srtm_message_buf_t *msgBuf;
    uint32_t primask;

    if (buf >= &srtmMsgs[0] && buf < &srtmMsgs[sizeof(srtmMsgs) / sizeof(srtm_message_buf_t)])
    {
        /* buffer locates in message pool */
        assert(((uint32_t)buf - (uint32_t)&srtmMsgs[0]) % sizeof(srtm_message_buf_t) == 0);
        msgBuf = (srtm_message_buf_t *)buf;
        primask = DisableGlobalIRQ();
        SRTM_List_AddTail(&srtmMsgList, &msgBuf->node);
#ifdef SRTM_DEBUG_MESSAGE_FUNC
        freeMsgCount++;
#endif
        EnableGlobalIRQ(primask);
    }
    else
    {
        SRTM_Heap_Free(buf);
    }
}

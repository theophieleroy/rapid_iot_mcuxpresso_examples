/*
 * The Clear BSD License
 * Copyright (c) 2017, NXP
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

#ifndef __SRTM_MESSAGE_STRUCT_H__
#define __SRTM_MESSAGE_STRUCT_H__

#include "srtm_defs.h"
#include "srtm_dispatcher.h"
#include "srtm_sem.h"
#include "srtm_list.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/**
* @brief SRTM message type fields
*/
typedef enum _srtm_message_type
{
    SRTM_MessageTypeRequest = 0x00U,   /*!< Request message */
    SRTM_MessageTypeResponse,          /*!< Response message for certain Request */
    SRTM_MessageTypeNotification,      /*!< Notification message that doesn't require response */
    SRTM_MessageTypeCommLast,          /*!< Last value of communication message */

    SRTM_MessageTypeProcedure = 0x40,  /*!< Local procedure */
    SRTM_MessageTypeRawData = 0x41,    /*!< Raw data message */
} srtm_message_type_t;

/**
* @brief SRTM message direction fields
*/
typedef enum _srtm_message_direct
{
    SRTM_MessageDirectNone = 0x00U, /*!< Local procedure message has no direction */
    SRTM_MessageDirectRx,           /*!< Received message */
    SRTM_MessageDirectTx,           /*!< Transfer message */
} srtm_message_direct_t;

SRTM_ANON_DEC_BEGIN

/**
* @brief SRTM message structure
*/
typedef struct _srtm_message
{
    srtm_list_t node;                          /*!< SRTM message list node to link to a list */
    srtm_message_type_t type;                  /*!< SRTM message type */
    srtm_message_direct_t direct;              /*!< SRTM message direction */
    void *data;                                /*!< SRTM raw data, including header and payload for CommMessage */
    uint32_t dataLen;                          /*!< SRTM raw data bytes */
    srtm_channel_t channel;                    /*!< SRTM communication channel */
    srtm_status_t error;                       /*!< SRTM message error status */
    uint8_t priority;                          /*!< SRTM message priority */
    srtm_message_free_t free;                  /*!< SRTM user defined message free function */
    void *freeParam;                           /*!< SRTM user defined message free param */

    union
    {
        struct
        {
            srtm_message_proc_cb_t cb;         /*!< SRTM procedure message callabck function */
            void *param1;                      /*!< SRTM procedure message callabck parameter */
            void *param2;                      /*!< SRTM procedure message callabck parameter */
            srtm_sem_t sig;                    /*!< SRTM procedure message signal to wait for procedure finish */
        } procMsg;
        struct
        {
            bool isSyncReq;                    /*!< Flag to indicate synchronized request */
            union
            {
                struct
                {
                    srtm_sem_t sig;            /*!< SRTM request message signal to wait for response */
                    srtm_response_t resp;      /*!< SRTM request message response */
                } sync;                        /*!< SRTM request message from SRTM_Request() */
                struct
                {
                    srtm_dispatcher_resp_cb_t cb; /*!< SRTM request message response callabck */
                    void *param;                  /*!< SRTM request message response callabck parameter */
                } async;                          /*!< SRTM request message from SRTM_Deliver_Request() */
            };
        } reqMsg;
        struct
        {
            uint32_t dummy;                    /*!< SRTM response message unused field */
        } rspMsg;
        struct
        {
            uint32_t dummy;                    /*!< SRTM notification message unused field */
        } ntfMsg;
    };
} srtm_message;

/**
* @brief SRTM communication packet head
*/
typedef SRTM_PACKED_BEGIN struct _srtm_packet_head
{
    union {
        struct
        {
            uint8_t category;
            uint8_t majorVersion;
            uint8_t minorVersion;
            uint8_t type;
            uint8_t command;
            uint8_t priority;
            uint8_t reserved[4U];
        };
        uint8_t header[10U];
    };
} SRTM_PACKED_END srtm_packet_head_t;

SRTM_ANON_DEC_END

/*******************************************************************************
 * API
 ******************************************************************************/
#ifdef __cplusplus
extern "C" {
#endif

/*!
 * @brief Create SRTM base message.
 *
 * @param len the data length in bytes.
 * @return SRTM base message handle, or NULL on failure.
 */
srtm_message_t SRTM_Message_Create(uint32_t len);

/*!
 * @brief Duplicate a SRTM message.
 *
 * @param msg The message to duplicate.
 * @return SRTM message handle, or NULL on failure.
 */
srtm_message_t SRTM_Message_Duplicate(srtm_message_t msg);

/*!
 * @brief Destroy SRTM base message.
 *
 * @param message SRTM base message handle.
 */
void SRTM_Message_Destroy(srtm_message_t message);

#ifdef __cplusplus
}
#endif

#endif /* __SRTM_MESSAGE_STRUCT_H__ */

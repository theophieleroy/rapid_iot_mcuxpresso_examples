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

#ifndef __SRTM_DISPATCHER_STRUCT_H__
#define __SRTM_DISPATCHER_STRUCT_H__

#include "srtm_defs.h"
#include "srtm_list.h"
#include "srtm_sem.h"
#include "srtm_mutex.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/**
* @brief SRTM dispatcher message number to hold the received data.
*
* The received message is freed for reuse once it gets processed by the
* dispatcher task. In a busy traffic, 8 received messages might be used
* up and latter calling SRTM_Dispatcher_PostRecvData() from channel will
* return failed. In this case, user need to augment the number. 
*/
#ifndef SRTM_DISPATCHER_CONFIG_RX_MSG_NUMBER
#define SRTM_DISPATCHER_CONFIG_RX_MSG_NUMBER          (8U)
#endif

/**
* @brief SRTM dispatcher message maximum length in bytes.
*
* The maximum buffer length for SRTM_Dispatcher_PostRecvData() call. User
* need to augment the number when there's long message in application
* protocol.
*/
#ifndef SRTM_DISPATCHER_CONFIG_RX_MSG_MAX_LEN
#define SRTM_DISPATCHER_CONFIG_RX_MSG_MAX_LEN         (256U)
#endif

/**
* @brief SRTM dispatcher struct
*/
struct _srtm_dispatcher
{
    srtm_list_t cores;       /*!< SRTM peer core list head */
    srtm_list_t services;    /*!< SRTM service list head */

    srtm_mutex_t mutex;      /*!< Mutex for multi-task protection */

    srtm_list_t freeRxMsgs;  /*!< Free Rx messages list to hold the callback Rx data */
    srtm_list_t messageQ;    /*!< Message queue to hold the messages to process */
    srtm_list_t waitingReqs; /*!< Message queue to hold the request waiting for the response */

    volatile bool stopReq;   /*!< SRTM dispatcher stop request flag */
    bool started;            /*!< SRTM dispatcher started flag */
    srtm_sem_t startSig;     /*!< SRTM dispatcher start signal */
    srtm_sem_t stopSig;      /*!< SRTM dispatcher stop signal */
    srtm_sem_t queueSig;     /*!< SRTM dispatcher messageQ signal */
};

/*******************************************************************************
 * API
 ******************************************************************************/
#ifdef __cplusplus
extern "C" {
#endif

/*!
 * @brief Post received data from channel to dispatcher.
 *
 * @param dispatcher SRTM dispatcher handle.
 * @param channel The channel where Rx data is got from.
 * @param buf The buffer address of Rx data.
 * @param len The buffer len of Rx data in bytes.
 * @return SRTM_Status_Success on success and others on failure.
 */
srtm_status_t SRTM_Dispatcher_PostRecvData(srtm_dispatcher_t disp, srtm_channel_t channel,
                                          void *buf, uint32_t len);

/*!
 * @brief The main message processor of dispatcher.
 *
 * The function MUST be called in dispatcher running thread context.
 *
 * @param dispatcher SRTM dispatcher handle.
 * @param msg The message to be handled.
 * @return SRTM_Status_Success on success and others on failure.
 */
srtm_status_t SRTM_Dispatcher_ProcessMessage(srtm_dispatcher_t disp, srtm_message_t msg);

/*!
 * @brief Deliver all messages in the list sequentially and clean up the list.
 *
 * This function is used to combine some messages into one delivery to make sure the
 * messages can be handled in sequence without other messages interleaved. The messages
 * can be different type, e.g. combining procedure message and response message is allowed.
 *
 * @param disp SRTM dispatcher handle.
 * @param msgs Message list to send to peer core.
 * @return SRTM_Status_Success on success and others on failure.
 */
srtm_status_t SRTM_Dispatcher_DeliverMessages(srtm_dispatcher_t disp, srtm_list_t *msgs);

#ifdef __cplusplus
}
#endif
/*! @} */

#endif /* __SRTM_DISPATCHER_STRUCT_H__ */

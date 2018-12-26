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
#ifndef APP_YELLS_H_
#define APP_YELLS_H_

/*!
 * @addtogroup APP_YELLS_API
 * @{
 */

#if BLE_YELL_SERVER
#include "yells_task.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/* QBlue Private Profile Server environment variable */
struct app_yells_env_tag
{
    /* Profile role state: enabled/disabled */
    /* uint8_t enabled; */
    /* notification configured */
    uint8_t ntf_cfg[BLE_CONNECTION_MAX];
    /* Connection handle */
    uint16_t conhdl;
};

/*******************************************************************************
 * Variables
 ******************************************************************************/
/*! @brief yells environment variable */
extern struct app_yells_env_tag g_AppYellsEnv;
/*! @brief Table of message handlers */
extern const struct ke_state_handler g_AppYellsTableHandler;

/*******************************************************************************
 * API
 ******************************************************************************/
/*
 * Handles the error indication nessage from the YELLS.
 */
int APP_YellsErrorIndHandler(ke_msg_id_t const msgid,
                             struct yells_error_ind *param,
                             ke_task_id_t const dest_id,
                             ke_task_id_t const src_id);

/*!
 * @brief Handles the send data response message from the YELLS.
 *
 * @param[in] msgid     YELLS_SEND_DATA_RSP
 * @param[in] param     Pointer to the struct yells_send_data_rsp
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_YELLS
 *
 * @return If the message was consumed or not.
 * @description
 * This handler is used to report to the application a confirmation, or error status of a notification
 * request being sent by application.
 */
int APP_YellsSendDataRspHandler(ke_msg_id_t const msgid,
                                struct yells_send_data_rsp *param,
                                ke_task_id_t const dest_id,
                                ke_task_id_t const src_id);

/*!
 * @brief Handles the ind/ntf indication message from the YELLS.
 *
 * @param[in] msgid     YELLS_CFG_INDNTF_IND
 * @param[in] param     Pointer to the struct yells_cfg_indntf_ind
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_YELLS
 *
 * @return If the message was consumed or not.
 * @description
 * This handler is used to inform application that peer device has changed notification
 * configuration.
 */
int APP_YellsCfgIndntfIndHandler(ke_msg_id_t const msgid,
                                 struct yells_cfg_indntf_ind *param,
                                 ke_task_id_t const dest_id,
                                 ke_task_id_t const src_id);

/*!
 * @brief Handles the data ind message from the YELLS.
 *
 * @param[in] msgid     YELLS_GET_DATA_IND
 * @param[in] param     Pointer to the struct yells_get_data_ind
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_YELLS
 *
 * @return If the message was consumed or not.
 * @description
 * This handler is used to handle the data sent form peer device
 */
int APP_YellsGetDataIndHandler(ke_msg_id_t const msgid,
                               struct yells_get_data_ind *param,
                               ke_task_id_t const dest_id,
                               ke_task_id_t const src_id);

/*******************************************************************************
 * API
 ******************************************************************************/
void APP_YellsAddProfileTask(void);

/*!
 * @brief Send a notification containing raw data - at connection.
 *
 * @param[in] conhdl Connection handle
 * @param[in] length Length of data to be sent
 * @param[in] data Pointer to data to be sent
 *
 * @response YELLS_SEND_DATA_RSP
 * @description
 * This function is used by the application to send a raw data.
 */
void APP_YellsSendDataReq(uint16_t conhdl, uint8_t length, uint8_t *data);

/*
 * Handles the reception of APP_YELLS_PERIOD_NTF_TIMER message
 */
int app_yells_period_ntf_timer_handler(ke_msg_id_t const msgid,
                                       void const *param,
                                       ke_task_id_t const dest_id,
                                       ke_task_id_t const src_id);

/*!
 * @brief Handles the reception of APP_YELLS_THROUGHPUT_TIMER message
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance.
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 */
int APP_YellsThroughputTimerHandler(ke_msg_id_t const msgid,
                                    void const *param,
                                    ke_task_id_t const dest_id,
                                    ke_task_id_t const src_id);

#endif
/*! @brief @} APP_YELLS_API */

#endif /* APP_YELLS_H_ */

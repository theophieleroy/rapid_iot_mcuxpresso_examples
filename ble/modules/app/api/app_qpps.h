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
#ifndef _APP_QPPS_H_
#define _APP_QPPS_H_

/*!
 * @addtogroup APP_QPPS_API
 * @{
 */

#if BLE_QPP_SERVER
#include "qpps.h"
#include "qpps_task.h"

/*******************************************************************************
 * Variables
 ******************************************************************************/
/*! @brief Table of message handlers */
extern const struct ke_state_handler g_AppQppsTableHandler;

/*******************************************************************************
 * API
 ******************************************************************************/

/*!
 * @brief Create QPPS database.
 */
void APP_QppsAddProfileTask(void);

/*!
 * @brief Send a notification containing raw data - at connection.
 *
 * @param[in] conhdl Connection handle
 * @param[in] length Length of data to be sent
 * @param[in] data Pointer to data to be sent
 *
 * @response QPPS_SEND_DATA_RSP
 * @description
 * This function is used by the application to send a raw data.
 *
 */
int APP_QppsSendDataReq(uint16_t conhdl, uint16_t length, uint8_t *data);

/*!
 * @brief Handles the send data response message from the QPPS.
 *
 * @param[in] msgid     QPPS_SEND_DATA_RSP
 * @param[in] param     Pointer to the struct qpps_send_data_rsp
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_QPPS
 *
 * @return If the message was consumed or not.
 * @description
 * This handler is used to report to the application a response, or error status of a send data
 * request being sent by application.
 */
int APP_QppsSendDataRspHandler(ke_msg_id_t const msgid,
                               struct qpps_send_data_rsp *param,
                               ke_task_id_t const dest_id,
                               ke_task_id_t const src_id);

/*!
 * @brief Handles the ind/ntf indication message from the QPPS.
 *
 * @param[in] msgid     QPPS_CFG_INDNTF_IND
 * @param[in] param     Pointer to the struct qpps_cfg_indntf_ind
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_QPPS
 *
 * @return If the message was consumed or not.
 * @description
 * This handler is used to inform application that peer device has changed notification
 * configuration.
 */
int APP_QppsCfgIndntfIndHandler(ke_msg_id_t const msgid,
                                struct qpps_cfg_indntf_ind *param,
                                ke_task_id_t const dest_id,
                                ke_task_id_t const src_id);

/*!
 * @brief Handles the data indication message from the QPPS.
 *
 * @param[in] msgid     QPPS_RECV_DATA_IND
 * @param[in] param     Pointer to the struct qpps_recv_data_ind
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_QPPS
 *
 * @return If the message was consumed or not.
 * @description
 * This handler is used to handle the data sent form peer device
 */
int APP_QppsRecvDataIndHandler(ke_msg_id_t const msgid,
                               struct qpps_recv_data_ind *param,
                               ke_task_id_t const dest_id,
                               ke_task_id_t const src_id);

#endif
/*! @brief @} APP_QPPS_API */

#endif /* APP_QPPS_H_ */

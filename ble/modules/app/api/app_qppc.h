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
#ifndef _APP_QPPC_H_
#define _APP_QPPC_H_

/*!
 * @addtogroup APP_QPPC_API
 * @{
 */

#if BLE_QPP_CLIENT
#include "qppc_task.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*! @brief Qppc Service Client environment variable */
struct app_qppc_env_tag
{
    uint16_t conhdl; /*!<  Connection handle */
};

/*******************************************************************************
 * Variables
 ******************************************************************************/
/*! @brief Table of message handlers */
extern const struct ke_state_handler g_AppQppcTableHandler;

/*******************************************************************************
 * API
 ******************************************************************************/

/*!
 * @brief Add a QBlue Private Profile Client instance.
 */
void APP_QppcAddProfileTask(void);

/*!
 * @brief Start the profile - at connection.
 * @param[in] conhdl        Connection handle for which the profile client role is enabled.
 * @response  QPPC_ENABLE_RSP
 * @description
 *  This API is used for enabling the Client role of the QPP. Profile will discovery all
 *  the attributes related to QPPS.
 */
void APP_QppcEnableReq(uint16_t conhdl);

/*!
 * @brief Generic message to read a QPPS characteristic descriptor values.
 * @param[in] conhdl        Connection handle for which the profile QPP client role is enabled
 * @param[in] char_code     Code for which characteristic to read:
 *  - QPPC_QPPS_RX_CHAR_VALUE_USER_DESP
 *  - QPPC_QPPS_TX_VALUE_CLI_CFG
 * @response  QPPC_RD_CHAR_RSP
 * @description
 * This API shall be used to read the value of a descriptor in the peer device database.
 */
void APP_QppcRdCharReq(uint16_t conhdl, uint8_t char_code);

/*!
 * @brief Generic message for configuring the characteristics that can be handled.
 * @param[in] conhdl        Connection handle
 * @param[in] cfg_val       Stop/notify value to configure into the peer characteristic.
 *  - PRF_CLI_STOP_NTFIND
 *  - PRF_CLI_START_NTF
 * @response  QPPC_SNED_DATA_RSP
 * @description
 *  This API shall be used to enable or disable the notifications from QPPS.
 */
void APP_QppcCfgIndntfReq(uint16_t conhdl, uint16_t cfg_val);

/*!
 * @brief Send data to server.
 * @param[in] conhdl        Connection handle
 * @param[in] len           Length of data to be sent
 * @param[in] val           Pointer to data to be sent
 * @response  QPPC_SEND_DATA_RSP
 * @description
 * This function is used by the application to send a raw data to server.
 */
void APP_QppcSendDataReq(uint16_t conhdl, uint16_t len, uint8_t *val);

/*!
 * @brief Handles the enable confirmation from the QPPC.
 *
 * @param[in] msgid     QPPC_ENABLE_RSP
 * @param[in] param     Pointer to struct qppc_enable_rsp
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_QPPC
 * @return If the message was consumed or not.
 * @description
 * This API is used by the Client to either send the discovery results of QPPS and
 * confirm enabling of the Client role, or to simply confirm enabling of Client role.
 */
int APP_QppcEnableRspHandler(ke_msg_id_t const msgid,
                             struct qppc_enable_rsp *param,
                             ke_task_id_t const dest_id,
                             ke_task_id_t const src_id);

/*!
 * @brief Handles the generic message for read responses for APP.
 *
 * @param[in] msgid     QPPC_RD_CHAR_RSP
 * @param[in] param     Pointer to struct qppc_rd_char_rsp
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_QPPC
 * @return If the message was consumed or not.
 * @description
 * This API is used by the Client role to inform the Application of a received read response. This message in qppc
 * contains
 * QPPS' version.
 */
int APP_QppcRdCharRspHandler(ke_msg_id_t const msgid,
                             struct qppc_rd_char_rsp *param,
                             ke_task_id_t const dest_id,
                             ke_task_id_t const src_id);

/*!
 * @brief Handles the generic message for write characteristic response status to APP.
 *
 * @param[in] msgid     QPPC_SEND_DATA_RSP
 * @param[in] param     Pointer to struct qppc_send_data_rsp
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_QPPC
 * @return If the message was consumed or not.
 * @description
 * This API is used by the Client role to inform the Application of a received send data response.
 */
int APP_QppcSendDataRspHandler(ke_msg_id_t const msgid,
                               struct qppc_send_data_rsp *param,
                               ke_task_id_t const dest_id,
                               ke_task_id_t const src_id);
/*
 * Handles the generic message for config indicate/notify responses for APP.
 */

int APP_QppcCfgIndntfRspHandler(ke_msg_id_t const msgid,
                                struct qppc_cfg_indntf_rsp *param,
                                ke_task_id_t const dest_id,
                                ke_task_id_t const src_id);

/*!
 * @brief Handles the value send to APP.
 *
 * @param[in] msgid     QPPC_RECV_DATA_IND
 * @param[in] param     Pointer to struct qppc_recv_data_ind
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_QPPC
 * @return If the message was consumed or not.
 * @description
 * This API is used by the Client role to inform the Application of a received value by
 * notification. The application will do what it needs to do with the received value.
 */
int APP_QppcRecvDataIndHandler(ke_msg_id_t const msgid,
                               struct qppc_recv_data_ind *param,
                               ke_task_id_t const dest_id,
                               ke_task_id_t const src_id);

#endif /*BLE_QPP_CLIENT */

/*! @brief @} APP_QPPC_API */

#endif /* _APP_QPPC_H_ */

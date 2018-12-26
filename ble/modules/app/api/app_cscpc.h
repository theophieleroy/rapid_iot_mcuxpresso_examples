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
#ifndef _APP_CSCPC_H_
#define _APP_CSCPC_H_

/*!
 * @addtogroup APP_CSCPC_API
 * @{
 */

#if (BLE_CSC_COLLECTOR)

#include "cscpc.h"
#include "cscpc_task.h"

/*******************************************************************************
 * Variables
 ******************************************************************************/
/*! @brief Table of message handlers */
extern const struct ke_state_handler g_AppCscpcTableHandler;

/*******************************************************************************
 * API
 ******************************************************************************/
/*!
 * @brief Add Cycling Speed and Cadence Profile Collector.
 *
 */
void APP_CscpcAddProfileTask(void);

/*!
 * @brief Start the profile - at connection.
 * @param[in] conhdl        Connection handle
 * @response  CSCPC_ENABLE_RSP
 * @description
 * This message is used for enabling the Collector role of the CPP.
 *
 */
void APP_CscpcEnableReq(uint16_t conhdl);

/*!
 * @brief Handles the response of CSCPC_ENABLE_REQ.
 *
 * @param[in] msgid     CSCPC_ENABLE_RSP
 * @param[in] param     cscpc_enable_rsp
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_CSCPC_IDX
 * @return If the message was consumed or not.
 * @description
 * This message informs the application about the status of the enabling procedure.
 *
 */
int APP_CscpcEnableRspHandler(ke_msg_id_t const msgid,
                              struct cscpc_enable_rsp *param,
                              ke_task_id_t const dest_id,
                              ke_task_id_t const src_id);

/*!
 * @brief read the value of peer device's attribute.
 * @param[in] conhdl                Connection handle
 * @param[in] operation             Operation code set by the profile task
 * @param[in] read_code             Read code
 * @response  CSCPC_VALUE_IND and CSCPC_CMP_EVT
 * @description
 * The message shall be used to read the value of an attribute in the peer device database.
 *
 */
void APP_CscpcReadCmd(uint16_t conhdl, uint8_t operation, uint8_t read_code);

/*!
 * @brief configure notification/indication of peer device database.
 * @param[in] conhdl                Connection handle
 * @param[in] operation             Operation code
 * @param[in] desc_code             Descriptor code
 * @param[in] ntfind_cfg            NTF/IND configuration
 * @response   CSCPC_CMP_EVT
 * @description
 * This message is used to configure sending of notification/indication in the peer device database.
 *
 */
void APP_CscpcCfgNtfindCmd(uint16_t conhdl, uint8_t operation, uint8_t desc_code, uint16_t ntfind_cfg);

/*!
 * @brief write the value of CP controle point.
 * @param[in] conhdl                Connection handle
 * @param[in] operation             Operation code
 * @param[in] ctnl_pt               Control point request
 * @response   CSCPC_CMP_EVT
 * @description
 * This message allows writing the value of the SC Control Point characteristic.
 *
 */
void APP_CscpcCtnlPtCfgReq(uint16_t conhdl, uint8_t operation, struct cscp_sc_ctnl_pt_req sc_ctnl_pt);

/*!
 * @brief indication from SC control Point.
 *
 * @param[in] msgid     CSCPC_CTNL_PT_RSP
 * @param[in] param     cscpc_ctnl_pt_rsp
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_CSCPC_IDX
 * @return If the message was consumed or not.
 * @description
 * Response from the peer device containing the parameters of the control point.
 *
 */
int APP_CscpcCtnlPtRspHandler(ke_msg_id_t const msgid,
                              struct cscpc_ctnl_pt_rsp *param,
                              ke_task_id_t const dest_id,
                              ke_task_id_t const src_id);

/*!
 * @brief Handles the response of CSCPC_READ_CMD.
 *
 * @param[in] msgid     CSCPC_VALUE_IND
 * @param[in] param     cscpc_value_ind
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_CSCPC_IDX
 * @return If the message was consumed or not.
 * @description
 * This API message is sent to the application when a new value is received from the peer device within a
 * read response, or a notification.
 *
 */
int APP_CscpcValueIndHandler(ke_msg_id_t const msgid,
                             struct cscpc_value_ind *param,
                             ke_task_id_t const dest_id,
                             ke_task_id_t const src_id);

/**
 ****************************************************************************************
 * @brief Handles the response of command completed.
 *
 * @param[in] msgid     CSCPC_CMP_EVT
 * @param[in] param     cscpc_cmp_evt
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_CSCPC_IDX
 * @return If the message was consumed or not.
 * @description
 * The API message is used by the CSCPC task to inform the sender of a command that the procedure is over
 * and contains the status of the procedure.
 *
 ****************************************************************************************
 */
int APP_CscpcCmpEvtHandler(ke_msg_id_t const msgid,
                           struct cscpc_cmp_evt *param,
                           ke_task_id_t const dest_id,
                           ke_task_id_t const src_id);
#endif

/*! @brief @} APP_CSCPC_API */

#endif /* _APP_CSCPC_H_ */

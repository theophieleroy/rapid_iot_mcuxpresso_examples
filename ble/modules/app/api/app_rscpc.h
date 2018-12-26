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
#ifndef _APP_RSCPC_H_
#define _APP_RSCPC_H_

/*!
 * @addtogroup APP_RSCPC_API
 * @{
 */

#if (BLE_RSC_COLLECTOR)

#include "rscpc.h"
#include "rscpc_task.h"

/*******************************************************************************
 * Variables
 ******************************************************************************/
/*! @brief Table of message handlers */
extern const struct ke_state_handler g_AppRscpcTableHandler;

/*******************************************************************************
 * API
 ******************************************************************************/
/*!
 * @brief Add Running Speed and Cadence Profile Collector.
 *
 */
void APP_RscpcAddProfileTask(void);

/*!
 * @brief Start the profile - at connection.
 * @param[in] conhdl        Connection handle
 * @response  RSCPC_ENABLE_RSP
 * @description
 * This message is used for enabling the Collector role of the RSCP.
 *
 */
void APP_RscpcEnableReq(uint16_t conhdl);

/*!
 * @brief Handles the response of RSCPC_ENABLE_REQ.
 *
 * @param[in] msgid     RSCPC_ENABLE_RSP
 * @param[in] param     rscpc_enable_rsp
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_RSCPC_IDX
 * @return If the message was consumed or not.
 * @description
 * This message informs the application about the status of the enabling procedure.
 *
 */
int APP_RscpcEnableRspHandler(ke_msg_id_t const msgid,
                              struct rscpc_enable_rsp *param,
                              ke_task_id_t const dest_id,
                              ke_task_id_t const src_id);

/*!
 * @brief read the value of peer device's attribute.
 * @param[in] conhdl                Connection handle
 * @param[in] operation             Operation code set by the profile task
 * @param[in] read_code             Read code
 * @response  RSCPC_VALUE_IND and RSCPC_CMP_EVT
 * @description
 * The message shall be used to read the value of an attribute in the peer device database.
 *
 */
void APP_RscpcReadCmd(uint16_t conhdl, uint8_t operation, uint8_t read_code);

/*!
 * @brief configure notification/indication of peer device database.
 * @param[in] conhdl                Connection handle
 * @param[in] operation             Operation code
 * @param[in] desc_code             Descriptor code
 * @param[in] ntfind_cfg            NTF/IND configuration
 * @response   RSCPC_CMP_EVT
 * @description
 * This message is used to configure sending of notification/indication in the peer device database.
 *
 */
void APP_RscpcCfgNtfindCmd(uint16_t conhdl, uint8_t operation, uint8_t desc_code, uint16_t ntfind_cfg);

/*!
 * @brief write the value of control point.
 * @param[in] conhdl                Connection handle
 * @param[in] operation             Operation code
 * @param[in] ctnl_pt               Control point request
 * @response   RSCPC_CMP_EVT
 * @description
 * This message allows writing the value of the SC Control Point characteristic.
 *
 */
void APP_RscpcCtnlPtCfgReq(uint16_t conhdl, uint8_t operation, struct rscp_sc_ctnl_pt_req sc_ctnl_pt);

/*!
 * @brief indication from SC control Point.
 *
 * @param[in] msgid     RSCPC_CTNL_PT_RSP
 * @param[in] param     rscpc_ctnl_pt_rsp
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_RSCPC_IDX
 * @return If the message was consumed or not.
 * @description
 * Response from the peer device containing the parameters of the control point.
 *
 */
int APP_RscpcCtnlPtRspHandler(ke_msg_id_t const msgid,
                              struct rscpc_ctnl_pt_cfg_rsp *param,
                              ke_task_id_t const dest_id,
                              ke_task_id_t const src_id);

/*!
 * @brief Handles the response of CSCPC_READ_CMD.
 *
 * @param[in] msgid     RSCPC_VALUE_IND
 * @param[in] param     rscpc_value_ind
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_RSCPC_IDX
 * @return If the message was consumed or not.
 * @description
 * This API message is sent to the application when a new value is received from the peer device within a
 * read response, an indication, or a notification.
 *
 */
int APP_RscpcValueIndHandler(ke_msg_id_t const msgid,
                             struct rscpc_value_ind *param,
                             ke_task_id_t const dest_id,
                             ke_task_id_t const src_id);

/**
 ****************************************************************************************
 * @brief Handles the response of command completed.
 *
 * @param[in] msgid     RSCPC_CMP_EVT
 * @param[in] param     rscpc_cmp_evt
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_RSCPC_IDX
 * @return If the message was consumed or not.
 * @description
 * The API message is used by the RSCPC task to inform the sender of a command that the procedure is over
 * and contains the status of the procedure.
 *
 ****************************************************************************************
 */
int APP_RscpcCmpEvtHandler(ke_msg_id_t const msgid,
                           struct rscpc_cmp_evt *param,
                           ke_task_id_t const dest_id,
                           ke_task_id_t const src_id);
#endif

/*! @brief @} APP_RSCPC_API */

#endif /* _APP_RSCPC_H_ */

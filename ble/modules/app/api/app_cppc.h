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
#ifndef _APP_CPPC_H_
#define _APP_CPPC_H_

/*!
 * @addtogroup APP_CPPC_API
 * @{
 */

#if BLE_CP_COLLECTOR

#include "cppc.h"
#include "cppc_task.h"

/*******************************************************************************
 * Variables
 ******************************************************************************/
/*! @brief Table of message handlers */
extern const struct ke_state_handler g_AppCppcTableHandler;

/*******************************************************************************
 * API
 ******************************************************************************/
/*!
 * @brief Add Cycling Power Profile Collector.
 *
 */
void APP_CppcAddProfileTask(void);

/*!
 * @brief Start the profile at connection.
 * @param[in] conhdl        Connection handle
 * @response  CPPC_ENABLE_RSP
 * @description
 * This message is used for enabling the Collector role of the CPP.
 */
void APP_CppcEnableReq(uint16_t conhdl);

/*!
 * @brief Handles the response of CPPC_ENABLE_REQ.
 * @param[in] msgid     CPPC_ENABLE_RSP
 * @param[in] param     cppc_enable_rsp
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_CPPC_IDX
 * @return If the message was consumed or not.
 * @description
 * This message informs the application about the status of the enabling procedure.
 */
int APP_CppcEnableRspHandler(ke_msg_id_t const msgid,
                             struct cppc_enable_rsp *param,
                             ke_task_id_t const dest_id,
                             ke_task_id_t const src_id);

/*!
 * @brief Read the value of peer device's attribute.
 * @param[in] conhdl                Connection handle
 * @param[in] operation             Operation code set by the profile task
 *         CPPC_READ_OP_CODE
 * @param[in] read_code             Read code
 *         CPPC_RD_CP_FEAT
 *         CPPC_RD_SENSOR_LOC
 *         CPPC_RD_WR_CP_MEAS_CL_CFG
 *         CPPC_RD_WR_CP_MEAS_SV_CFG
 *         CPPC_RD_WR_VECTOR_CFG
 *         CPPC_RD_WR_CTNL_PT_CFG
 * @response  CPPC_VALUE_IND and CPPC_CMP_EVT
 * @description
 * The message shall be used to read the value of an attribute in the peer device database.
 */
void APP_CppcReadCmd(uint16_t conhdl, uint8_t operation, uint8_t read_code);

/*!
 * @brief configure notification/indication of peer device database.
 * @param[in] conhdl                Connection handle
 * @param[in] operation             Operation code
 *             CPPC_CFG_NTF_IND_OP_CODE
 * @param[in] desc_code             Descriptor code
 *             CPPC_RD_WR_CP_MEAS_CL_CFG
 *             CPPC_RD_WR_CP_MEAS_SV_CFG
 *             CPPC_RD_WR_VECTOR_CFG
 *             CPPC_RD_WR_CTNL_PT_CFG
 * @param[in] ntfind_cfg            NTF/IND configuration
 * @response   CPPC_CMP_EVT
 * @description
 * This message is used to configure sending of notification/indication in the peer device database.
 */
void APP_CppcCfgNtfindCmd(uint16_t conhdl, uint8_t operation, uint8_t desc_code, uint16_t ntfind_cfg);

/*!
 * @brief write the value of CP controle point.
 * @param[in] conhdl                Connection handle
 * @param[in] operation             Operation code
 *             CPPC_CTNL_PT_CFG_WR_OP_CODE
 * @param[in] ctnl_pt               Control point request
 * @response   CPPC_CMP_EVT
 * @description
 * This message allows writing the value of the CP Control Point characteristic.
 */
void APP_CppcCtnlPtCfgReq(uint16_t conhdl, uint8_t operation, struct cpp_ctnl_pt_req ctnl_pt);

/*!
 * @brief indication from CP control Point.
 * @param[in] msgid     CPPC_CTNL_PT_RSP
 * @param[in] param     cppc_ctnl_pt_rsp
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_CPPC_IDX
 * @return If the message was consumed or not.
 * @description
 * This message is sent to the application when a new value is received from the CP control Point indication.
 */
int APP_CppcCtnlPtRspHandler(ke_msg_id_t const msgid,
                             struct cppc_ctnl_pt_rsp *param,
                             ke_task_id_t const dest_id,
                             ke_task_id_t const src_id);

/*!
 * @brief Handles the response of CPPC_READ_CMD.
 * @param[in] msgid     CPPC_VALUE_IND
 * @param[in] param     cppc_value_ind
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_CPPC_IDX
 * @return If the message was consumed or not.
 * @description
 * This message is sent to the application when a new value is received from the peer device within a read
 * response or a notification.
 */

int APP_CppcValueIndHandler(ke_msg_id_t const msgid,
                            struct cppc_value_ind *param,
                            ke_task_id_t const dest_id,
                            ke_task_id_t const src_id);

/*!
 * @brief Handles the response of command completed.
 * @param[in] msgid     CPPC_CMP_EVT
 * @param[in] param     cppc_cmp_evt
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_CPPC_IDX
 * @return If the message was consumed or not.
 * @description
 * The message is used by the CPPC task to inform the sender of a command that the procedure is over and
 * contains the status of the procedure.
 */
int APP_CppcCmpEvtHandler(ke_msg_id_t const msgid,
                          struct cppc_cmp_evt *param,
                          ke_task_id_t const dest_id,
                          ke_task_id_t const src_id);

/*!
 * @brief handle timeout of CPP.
 * @param[in] msgid     CPPC_TIMEOUT_TIMER_IND
 * @param[in] param     Input param
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_CPPC_IDX
 * @return If the message was consumed or not.
 * @description
 * The message is used to notify the application timeout occur.
 */
int APP_CppcTimeoutTimerHandler(ke_msg_id_t const msgid,
                                void const *param,
                                ke_task_id_t const dest_id,
                                ke_task_id_t const src_id);

#endif

/*! @brief @} APP_CPPC_API */

#endif /* _APP_CPPC_H_ */

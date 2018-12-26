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

#ifndef _APP_LANC_H_
#define _APP_LANC_H_

/*!
 * @addtogroup APP_LANC_API
 * @{
 */

#if BLE_LN_COLLECTOR
#include "lanc.h"
#include "lanc_task.h"
#include "prf_utils.h"

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*! @brief Table of message handlers  */
extern const struct ke_state_handler g_AppLancTableHandler;

/*******************************************************************************
  * API
 ******************************************************************************/
/*!
 * @brief Add Location and Navigation Profile Collector..
 */
void APP_LancAddProfileTask(void);

/*!
 * @brief Start the profile at connection.
 * @param[in] conhdl        Connection handle
 * @response  LANC_ENABLE_RSP
 * @description
 * This API message is used for enabling the Collector role of the LANP.
 */
void APP_LancEnableReq(uint16_t conhdl);

/*!
 * @brief Handles the response of LANC_ENABLE_REQ.
 *
 * @param[in] msgid     LANC_ENABLE_RSP
 * @param[in] param     lanc_enable_rsp
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_LANC_IDX
 * @return If the message was consumed or not.
 * @description
 * This message informs the application about the status of the operation.
 */

int APP_LancEnableRspHandler(ke_msg_id_t const msgid,
                             struct lanc_enable_rsp *param,
                             ke_task_id_t const dest_id,
                             ke_task_id_t const src_id);

/*!
 * @brief Read the value of characteristic.
 * @param[in] operation        Operation code
 *       LANC_READ_OP_CODE
 * @param[in] read_code        Read code
 *       LANC_RD_LN_FEAT
 *       LANC_RD_POS_Q
 *       LANC_RD_WR_LOC_SPEED_CL_CFG
 *       LANC_RD_WR_LN_CTNL_PT_CFG
 *       LANC_RD_WR_NAVIGATION_CFG
 * @response  LANC_VALUE_IND and LANC_CMP_EVT
 * @description
 * The API message shall be used to read the value of an attribute in the peer device database.
 */
void APP_LancReadCmd(uint16_t conhdl, uint8_t operation, uint8_t read_code);

/*!
 * @brief Receive reponse or notification.
 *
 * @param[in] msgid     LANC_VALUE_IND
 * @param[in] param     lanc_value_ind
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_LANC_IDX
 * @return If the message was consumed or not.
 * @description
 * This message is sent to the application when a new value is received from the peer device
 * within a read response or a notification.
 *
 */
int APP_LancValueIndHandler(ke_msg_id_t const msgid,
                            struct lanc_value_ind *param,
                            ke_task_id_t const dest_id,
                            ke_task_id_t const src_id);

/*!
 * @brief Send configure command.
 * @param[in] operation        Operation code
 *       LANC_CFG_NTF_IND_OP_CODE
 * @param[in] desc_code        Descriptor code
 *       LANC_RD_WR_LOC_SPEED_CL_CFG
 *       LANC_RD_WR_LN_CTNL_PT_CFG
 *       LANC_RD_WR_NAVIGATION_CFG
 * @param[in] ntfind_cfg       NTF/IND configuration
 * @response  LANC_CMP_EVT
 * @description
 * This API message is used to configure sending of notification/indication in the peer device database.
 *
 */
void APP_LancCfgNtfindCmd(uint16_t conhdl, uint8_t operation, uint8_t desc_code, uint16_t ntfind_cfg);

/*!
 * @brief Write CP characteristic value.
 * @param[in] operation        Operation code
 *       LANC_LN_CTNL_PT_CFG_WR_OP_CODE
 * @param[in] ctnl_pt          Control point request
 * @response  LANC_CMP_EVT
 * @description
 * This message allows writing the value of the LN Control Point characteristic.
 */
void APP_LancLnCtnlPtCfgReq(uint16_t conhdl, uint8_t operation, struct lan_ln_ctnl_pt_req *ln_ctnl_pt);

/*!
 * @brief Receive control Point indication.
 *
 * @param[in] msgid     LANC_LN_CTNL_PT_RSP
 * @param[in] param     lanc_ln_ctnl_pt_rsp
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_LANC_IDX
 * @return If the message was consumed or not.
 * @description
 * This message is sent to the application when a new value is received from the LN
 * control Point indication.
 */

int APP_LancLnCtnlPtRspHandler(ke_msg_id_t const msgid,
                               struct lanc_ln_ctnl_pt_rsp *param,
                               ke_task_id_t const dest_id,
                               ke_task_id_t const src_id);

/*!
 * @brief Receive response of command.
 *
 * @param[in] msgid     LANC_CMP_EVT
 * @param[in] param     lanc_cmp_evt
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_LANC_IDX
 * @return If the message was consumed or not.
 * @description
 * The message is used by the LANC task to inform the sender of a command that the
 * procedure is over and contains the status of the procedure.
 *
 */

int APP_LancCmpEvtHandler(ke_msg_id_t const msgid,
                          struct lanc_cmp_evt *param,
                          ke_task_id_t const dest_id,
                          ke_task_id_t const src_id);

#endif
/*! @brief @} APP_LANC_API */
#endif // _APP_LANC_H_

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
#ifndef _APP_TIPC_H_
#define _APP_TIPC_H_

/*!
 * @addtogroup APP_TIPC_API
 * @{
 */

#if BLE_TIP_CLIENT
#include "tipc.h"
#include "tipc_task.h"

/*******************************************************************************
 * Variables
 ******************************************************************************/
/*! @brief Table of message handlers */
extern const struct ke_state_handler g_AppTipcTableHandler;

/*******************************************************************************
 * API
 ******************************************************************************/
/*!
 * @brief Add Time Profile Client.
 */
void APP_TipcAddProfileTask(void);

/*!
 * @brief Start the profile at connection.
 * @param[in] conhdl        Connection handle
 * @response  TIPC_ENABLE_RSP
 * @description
 * This API message is used for enabling the Client role of the Time profile.
 */
void APP_TipcEnableReq(uint16_t conhdl);

/*!
 * @brief Handles the response of TIPC_ENABLE_REQ.
 * @param[in] msgid     TIPC_ENABLE_RSP
 * @param[in] param     tipc_enable_rsp
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_TIPC_IDX
 * @return If the message was consumed or not.
 * @description
 * This API message is used by the Client to either send the discovery results of CTS, NDCS or RTUS and confirm enabling
 * of the Client role (status = PRF_ERR_OK), or to simply confirm enabling of Client role if it is a normal connection and
 * the attribute details are already known (status = PRF_ERR_OK), or to inform the application that the discovery process
 * has been stopped because of a missing attribute (status = PRF_ERR_STOP_DISC_CHAR_MISSING)..
 */
int APP_TipcEnableRspHandler(ke_msg_id_t const msgid,
                             struct tipc_enable_rsp *param,
                             ke_task_id_t const dest_id,
                             ke_task_id_t const src_id);


/*!
 * @brief Read the characteristic value.
 * @param[in] conhdl        Connection handle
 * @param[in] char_code     Code for which characteristic to read.
 *              TIPC_RD_CTS_CURR_TIME
 *              TIPC_RD_CTS_LOCAL_TIME_INFO
 *              TIPC_RD_CTS_REF_TIME_INFO
 *              TIPC_RD_CTS_CURR_TIME_CLI_CFG
 *              TIPC_RD_NDCS_TIME_WITH_DST
 *              TIPC_RD_RTUS_TIME_UPD_STATE
 * @response  TIPC_RD_CHAR_RSP
 * @description
 * This API message is used by the application to request sending of a GATT_READ_CHAR_REQ with the parameters
 * deduced from the char_code.
 */
void APP_TipcRdCharReq(uint16_t conhdl,uint8_t  char_code);


/*!
 * @brief Handles the response of TIPC_RD_CHAR_REQ.
 * @param[in] msgid     TIPC_RD_CHAR_RSP
 * @param[in] param     tipc_rd_char_rsp
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_TIPC_IDX
 * @return If the message was consumed or not.
 * @description
 * This API message is used by the Client role to inform the application that the read operation was performed
 * successfully. Note that the parameter will depend on the op_code..
 */
int APP_TipcRdCharRspHandler(ke_msg_id_t const msgid,
                             struct tipc_rd_char_rsp *param,
                             ke_task_id_t const dest_id,
                             ke_task_id_t const src_id);

/*!
 * @brief Peer device configurate notification.
 * @param[in] conhdl        Connection handle
 * @param[in] cfg_val       Configuration value
 * @response  TIPC_CT_NTF_CFG_RSP
 * @description
 * This API message is used by the application to send a GATT_WRITE_CHAR_REQ to the Current Time Client
 * Configuration Characteristic Descriptor with the parameter cfg_val.
 */
void APP_TipcCtNtfCfgReq(uint16_t conhdl,uint16_t cfg_val);


/*!
 * @brief Handles the response of TIPC_CT_NTF_CFG_REQ.
 * @param[in] msgid     TIPC_CT_NTF_CFG_RSP
 * @param[in] param     tipc_ct_ntf_cfg_rsp
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_TIPC_IDX
 * @return If the message was consumed or not.
 * @description
 * This API message informs the app about the status of the operation.
 */
int APP_TipcCtNtfCfgRspHandler(ke_msg_id_t const msgid,
                             struct tipc_ct_ntf_cfg_rsp *param,
                             ke_task_id_t const dest_id,
                             ke_task_id_t const src_id);


/*!
 * @brief Peer device write new value.
 * @param[in] conhdl        Connection handle
 * @param[in] value         Time Update Control Point value to write
 * @response  TIPC_WR_TIME_UPD_CTNL_PT_RSP
 * @description
 * This API message is used by the application to send a GATT_WRITE_CHAR_REQ to the Time Update
 * Control Point Characteristic.
 */
void APP_TipcWrTimeUpdCtnlPtReq(uint16_t conhdl,uint8_t value);


/*!
 * @brief Handles the response of TIPC_WR_TIME_UPD_CTNL_PT_REQ.
 * @param[in] msgid     TIPC_WR_TIME_UPD_CTNL_PT_RSP
 * @param[in] param     tipc_wr_time_upd_ctnl_pt_rsp
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_TIPC_IDX
 * @return If the message was consumed or not.
 * @description
 * This API message is used by the Client role to inform the Application of a received write response.
 */
int APP_TipcWrTimeUpdCtnlPtRspHandler(ke_msg_id_t const msgid,
                                      struct tipc_wr_time_upd_ctnl_pt_rsp *param,
                                      ke_task_id_t const dest_id,
                                      ke_task_id_t const src_id);

/*!
 * @brief Receive notification from peer device.
 * @param[in] msgid     TIPC_CT_IND
 * @param[in] param     tipc_ct_ind
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_TIPC_IDX
 * @return If the message was consumed or not.
 * @description
 * This API message is used by the Client role to inform the Application of a notified current time value.
 */
int APP_TipcCtIndHandler(ke_msg_id_t const msgid,
                         struct tipc_ct_ind *param,
                         ke_task_id_t const dest_id,
                         ke_task_id_t const src_id);


#endif
/*! @brief @} APP_TIPC_API */
#endif /* _APP_TIPC_H_ */


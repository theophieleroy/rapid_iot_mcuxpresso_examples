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
#ifndef _APP_BLPC_H_
#define _APP_BLPC_H_

/*!
 * @addtogroup APP_BLPC_API
 * @{
 */

#if BLE_BP_COLLECTOR
#include "blpc.h"
#include "blpc_task.h"

/*******************************************************************************
 * Variables
 ******************************************************************************/
/*! @brief Table of message handlers */
extern const struct ke_state_handler g_AppBlpcTableHandler;

/*******************************************************************************
 * API
 ******************************************************************************/
/*!
 * @brief Add Blood Pressure Profile Client.
 */
void APP_BlpcAddProfileTask(void);

/*!
 * @brief Start the profile at connection.
 * @param[in] conhdl        Connection handle
 * @response  BLPC_ENABLE_RSP
 * @description
 * This API message is used for enabling the Client role of the Blood Pressure profile.
 */
void APP_BlpcEnableReq(uint16_t conhdl);

/*!
 * @brief Handles the response of BLPC_ENABLE_REQ.
 * @param[in] msgid     BLPC_ENABLE_RSP
 * @param[in] param     blpc_enable_rsp
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_BLPC_IDX
 * @return If the message was consumed or not.
 * @description
 * This API message is used by the Collector in order to either send the discovery results of BPS on the
 * Blood Pressure or to simply confirm enabling of Collector role if it is a normal connection and the attribute details are
 * already known.
 */
int APP_BlpcEnableRspHandler(ke_msg_id_t const msgid,
                             struct blpc_enable_rsp *param,
                             ke_task_id_t const dest_id,
                             ke_task_id_t const src_id);


/*!
 * @brief Read the characteristic value.
 * @param[in] conhdl        Connection handle
 * @param[in] char_code     Code for which characteristic to read.
 * @response  BLPC_RD_CHAR_RSP
 * @description
 * This API message is used by the application to request sending of a GATT_READ_CHAR_REQ with the parameters
 * deduced from the char_code.
 */
void APP_BlpcRdCharReq(uint16_t conhdl,uint8_t  char_code);


/*!
 * @brief Handles the response of BLPC_RD_CHAR_REQ.
 * @param[in] msgid     BLPC_RD_CHAR_RSP
 * @param[in] param     blpc_rd_char_rsp
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_BLPC_IDX
 * @return If the message was consumed or not.
 * @description
 * This API message is used by the Collector role to inform the Application of a received read response. The
 * status and the data from the read response are passed directly to Application, which must interpret them based on
 * the request it made.
 */
int APP_BlpcRdCharRspHandler(ke_msg_id_t const msgid,
                             struct blpc_rd_char_rsp *param,
                             ke_task_id_t const dest_id,
                             ke_task_id_t const src_id);

/*!
 * @brief Enable or disable the notification or indication
 * @param[in] conhdl        Connection handle
 * @param[in] char_code     Code for which characteristic to configure in ntf/ind
 * @param[in] cfg_val       Configuration value
 * @response  BLPC_WR_CHAR_RSP
 * @description
 * This API message is used by the application to send a GATT_WRITE_CHAR_REQ with the parameters
 * deduced from the char_code and cfg_val. 
 */
void APP_BlpcCfgIndNtfReq(uint16_t conhdl, uint8_t char_code, uint16_t cfg_val);


/*!
 * @brief Handles the response of BLPC_CFG_INDNTF_REQ.
 * @param[in] msgid     BLPC_CFG_INDNTF_RSP
 * @param[in] param     tipc_ct_ntf_cfg_rsp
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_BLPC_IDX
 * @return If the message was consumed or not.
 * @description
 * This API message is used by the Collector role to inform the Application of a received write response. The
 * status and the data from the write response are passed directly to Application, which must interpret them based on
 * the request it made.
 */
int APP_BlpcCfgIndNtfRspHandler(ke_msg_id_t const msgid,
                             struct tipc_ct_ntf_cfg_rsp *param,
                             ke_task_id_t const dest_id,
                             ke_task_id_t const src_id);


/*!
 * @brief This API message is used by the Collector role to inform the Application of a received blood pressure value.
 * @param[in] msgid     BLPC_BP_MEAS_IND
 * @param[in] param     blpc_bp_meas_ind
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_BLPC_IDX
 * @return If the message was consumed or not.
 * @description
 * This API message is used by the Collector role to inform the Application of a received blood pressure
 * value, either by notification flag_interm_cp = intermediate) or indication (flag_interm_cp = stable). The application
 * will do what it needs to do with the received measurement.
 */
void APP_BlpcBpMeasIndHandler(ke_msg_id_t const msgid,
                             struct blpc_bp_meas_ind *param,
                             ke_task_id_t const dest_id,
                             ke_task_id_t const src_id);


#endif
/*! @brief @} APP_BLPC_API */
#endif /* _APP_BLPC_H_ */


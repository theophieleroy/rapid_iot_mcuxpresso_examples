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
#ifndef _APP_HRPC_H_
#define _APP_HRPC_H_

/*!
 * @addtogroup APP_HRPC_API
 * @{
 */

#if BLE_HR_COLLECTOR
#include "hrpc.h"
#include "hrpc_task.h"

/*******************************************************************************
 * Variables
 ******************************************************************************/
/*! @brief Table of message handlers */
extern const struct ke_state_handler g_AppHrpcTableHandler;

/*******************************************************************************
 * API
 ******************************************************************************/
/*!
 * @brief Add Heart Rate Profile Collector.
 */
void APP_HrpcAddProfileTask(void);

/*!
 * @brief Start the profile at connection.
 * @param[in] conhdl        Connection handle
 * @param[in] hrs           Existing handle values HRS
 * @response  HRPC_ENABLE_RSP
 * @description
 * This API message is used for enabling the Collector role of the Heart Rate profile.
 */
void APP_HrpcEnableReq(uint16_t conhdl,struct hrs_content *hrs);

/*!
 * @brief Handles the response of HRPC_ENABLE_REQ.
 * @param[in] msgid     HRPC_ENABLE_RSP
 * @param[in] param     hrpc_enable_rsp
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_HRPC_IDX
 * @return If the message was consumed or not.
 * @description
 * This API message is used by the Collector to either send the discovery results of HRS on the Heart Rate
 * or to simply confirm enabling of Collector role if it is a normal connection and the attribute details 
 * are already known.
 */
int APP_HrpcEnableRspHandler(ke_msg_id_t const msgid,
                             struct hrpc_enable_rsp *param,
                             ke_task_id_t const dest_id,
                             ke_task_id_t const src_id);



/*!
 * @brief Read value from peer device.
 * @param[in] conhdl        Connection handle
 * @param[in] char_code     Code for which characteristic to read
 * @response  HRPC_RD_CHAR_RSP
 * @description
 * This API message is used by the application to send a GATT_READ_CHAR_REQ with the parameters
 * deduced from the char_code.
 */
void APP_HrpcRdCharReq(uint16_t conhdl,uint8_t char_code);


/*!
 * @brief Handles the response of HRPC_RD_CHAR_REQ.
 * @param[in] msgid     HRPC_RD_CHAR_RSP
 * @param[in] param     hrpc_rd_char_rsp
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_HRPC_IDX
 * @return If the message was consumed or not.
 * @description
 * This API message is used by the Collector role to inform the Application of a received read response.
 */
int APP_HrpcRdCharRspHandler(ke_msg_id_t const msgid,
                             struct hrpc_rd_char_rsp *param,
                             ke_task_id_t const dest_id,
                             ke_task_id_t const src_id);


/*!
 * @brief Write control point attribute.
 * @param[in] conhdl        Connection handle
 * @param[in] val           Reset
 * @response  HRPC_WR_CNTL_POINT_RSP
 * @description
 * This API message is used by the application to write control point attribute in order to reset Energy
 * Expanded value.
 */
void APP_HrpcWrCntlPointReq(uint16_t conhdl,uint8_t val);


/*!
 * @brief Handles the response of HRPC_WR_CNTL_POINT_REQ.
 * @param[in] msgid     HRPC_WR_CNTL_POINT_RSP
 * @param[in] param     hrpc_wr_cntl_point_rsp
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_HRPC_IDX
 * @return If the message was consumed or not.
 * @description
 * This API message informs the application about the status of the operation performed by the control point.
 */
int APP_HrpcWrCntlPointRspHandler(ke_msg_id_t const msgid,
                                  struct hrpc_wr_cntl_point_rsp *param,
                                  ke_task_id_t const dest_id,
                                  ke_task_id_t const src_id);


/*!
 * @brief Write value to peer device.
 * @param[in] conhdl        Connection handle
 * @param[in] cfg_val       Configuration value
 * @response  HRPC_CFG_INDNTF_RSP
 * @description
 * This API message is used by the application to send a GATT_WRITE_CHAR_REQ to the peer device in
 * order to write the provided value in the measurement characteristic.
 */
void APP_HrpcCfgIndntfReq(uint16_t conhdl,uint8_t cfg_val);


/*!
 * @brief Handles the response of HRPC_CFG_INDNTF_REQ.
 * @param[in] msgid     HRPC_CFG_INDNTF_RSP
 * @param[in] param     hrpc_cfg_indntf_rsp
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_HRPC_IDX
 * @return If the message was consumed or not.
 * @description
 * This API message is used by the Collector role to inform the Application of a received write response.
 */
int APP_HrpcCfgIndntfRspHandler(ke_msg_id_t const msgid,
                                struct hrpc_cfg_indntf_rsp *param,
                                ke_task_id_t const dest_id,
                                ke_task_id_t const src_id);


/*!
 * @brief Receive notification from peer device.
 * @param[in] msgid     HRPC_HR_MEAS_IND
 * @param[in] param     hrpc_hr_meas_ind
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_HRPC_IDX
 * @return If the message was consumed or not.
 * @description
 * This API message is used by the Collector role to inform the Application of a received Heart Rate value by
 * notification.
 */
int APP_HrpcHrMeasIndHandler(ke_msg_id_t const msgid,
                             struct hrpc_hr_meas_ind *param,
                             ke_task_id_t const dest_id,
                             ke_task_id_t const src_id);


#endif
/*! @brief @} APP_HRPC_API */
#endif /* _APP_HRPC_H_ */


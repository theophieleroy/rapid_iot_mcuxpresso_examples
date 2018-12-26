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
#ifndef _APP_HTPC_H_
#define _APP_HTPC_H_

/*!
 * @addtogroup APP_HTPC_API
 * @{
 */

#if BLE_HT_COLLECTOR
#include "htpc.h"
#include "htpc_task.h"

/*******************************************************************************
 * Variables
 ******************************************************************************/
/*! @brief Table of message handlers */
extern const struct ke_state_handler g_AppHtpcTableHandler;

/*******************************************************************************
 * API
 ******************************************************************************/
/*!
 * @brief Add Health Thermometer Profile Collector.
 */
void APP_HtpcAddProfileTask(void);

/*!
 * @brief Start the profile at connection.
 * @param[in] conhdl        Connection handle
 * @response  HTPC_ENABLE_RSP
 * @description
 * This API message is used for enabling the Collector role of the Health Thermometer profile.
 */
void APP_HtpcEnableReq(uint16_t conhdl);

/*!
 * @brief Handles the response of HTPC_ENABLE_REQ.
 * @param[in] msgid     HTPC_ENABLE_RSP
 * @param[in] param     htpc_enable_rsp
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_HTPC_IDX
 * @return If the message was consumed or not.
 * @description
 * This API message is used by the Collector to either send the discovery results of HTS on
 * the thermometer and confirm enabling of the Collector role, or to simply confirm enabling
 * of Collector role.
 */
int APP_HtpcEnableRspHandler(ke_msg_id_t const msgid,
                             struct htpc_enable_rsp *param,
                             ke_task_id_t const dest_id,
                             ke_task_id_t const src_id);

/*!
 * @brief Configurate peer device for notification or indication.
 * @param[in] conhdl        Connection handle
 * @param[in] cfg_val       Configuration value
 *        PRF_CLI_STOP_NTFIND = 0x0000,
 *        PRF_CLI_START_NTF,
 *        PRF_CLI_START_IND,
 * @param[in] char_code     Configuration value
 *        HTPC_CHAR_HTS_TEMP_MEAS,
 *        HTPC_CHAR_HTS_INTM_TEMP,
 *        HTPC_CHAR_HTS_MEAS_INTV,
 * @response  HTPC_HEALTH_TEMP_NTF_CFG_RSP
 * @description
 * This API message is used by the application to send a GATT_WRITE_CHAR_REQ with the parameters
 * deduced from the char_code and cfg_val.
 */
void APP_HtpcHealthTempNtfCfgReq(uint16_t conhdl, uint16_t cfg_val, uint8_t char_code);

/*!
 * @brief Handles the response of HTPC_HEALTH_TEMP_NTF_CFG_REQ.
 * @param[in] msgid     HTPC_HEALTH_TEMP_NTF_CFG_RSP
 * @param[in] param     htpc_health_temp_ntf_cfg_rsp
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_HTPC_IDX
 * @return If the message was consumed or not.
 * @description
 * This API message is used by the Collector role to inform the Application of a received write response.
 */
int APP_HtpcHealthTempNtfCfgRspHandler(ke_msg_id_t const msgid,
                                       struct htpc_health_temp_ntf_cfg_rsp *param,
                                       ke_task_id_t const dest_id,
                                       ke_task_id_t const src_id);

/*!
 * @brief Write new interval value to peer device.
 * @param[in] conhdl        Connection handle
 * @param[in] intv          Interval value
 * @response  HTPC_WR_MEAS_INTV_RSP
 * @description
 * This API message is used by the application to send a GATT_WRITE_CHAR_REQ to the HTS Measurement
 * Interval Char. In the Thermometer, with the new interval value in intv.
 */
void APP_HtpcWrMeasIntvReq(uint16_t conhdl, uint16_t intv);

/*!
 * @brief Handles the response of HTPC_WR_MEAS_INTV_REQ.
 * @param[in] msgid     HTPC_WR_MEAS_INTV_RSP
 * @param[in] param     htpc_wr_meas_intv_rsp
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_HTPC_IDX
 * @return If the message was consumed or not.
 * @description
 * This API message is used by the Collector role to inform the Application of a received write response.
 */
int APP_HtpcWrMeasIntvRspHandler(ke_msg_id_t const msgid,
                                 struct htpc_wr_meas_intv_rsp *param,
                                 ke_task_id_t const dest_id,
                                 ke_task_id_t const src_id);

/*!
 * @brief  Read the value of character.
 * @param[in] conhdl        Connection handle
 * @param[in] char_code     Characteristic value code
 *        HTPC_RD_TEMP_TYPE           = 0,
 *        HTPC_RD_MEAS_INTV,
 *        HTPC_RD_TEMP_MEAS_CLI_CFG,
 *        HTPC_RD_INTM_TEMP_CLI_CFG,
 *        HTPC_RD_MEAS_INTV_CLI_CFG,
 *        HTPC_RD_MEAS_INTV_VAL_RGE,
 * @response  HTPC_RD_CHAR_RSP
 * @description
 * This API message is used by the application to send a GATT_READ_CHAR_REQ with the parameters
 * deduced from the char_code.
 */
void APP_HtpcRdCharReq(uint16_t conhdl, uint8_t char_code);

/*!
 * @brief Handles the response of HTPC_RD_CHAR_REQ.
 * @param[in] msgid     HTPC_RD_CHAR_RSP
 * @param[in] param     htpc_rd_char_rsp
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_HTPC_IDX
 * @return If the message was consumed or not.
 * @description
 * This API message is used by the Collector role to inform the application of a received read response.
 */
int APP_HtpcRdCharRspHandler(ke_msg_id_t const msgid,
                             struct htpc_rd_char_rsp *param,
                             ke_task_id_t const dest_id,
                             ke_task_id_t const src_id);

/*!
 * @brief handle the received notification or indication of temperature value.
 * @param[in] msgid     HTPC_TEMP_IND
 * @param[in] param     htpc_temp_ind
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_HTPC_IDX
 * @return If the message was consumed or not.
 * @description
 * This API message is used by the Collector role to inform the Application of a received temperature
 * value,either by notification or indication .
 */
int APP_HtpcTempIndHandler(ke_msg_id_t const msgid,
                           struct htpc_temp_ind *param,
                           ke_task_id_t const dest_id,
                           ke_task_id_t const src_id);

/*!
 * @brief handle the received notification of Measurement Interval.
 * @param[in] msgid     HTPC_MEAS_INTV_IND
 * @param[in] param     htpc_meas_intv_ind
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_HTPC_IDX
 * @return If the message was consumed or not.
 * @description
 * This API message is used by the Collector role to inform the Application of a received Measurement
 * Interval Char.
 */
int APP_HtpcMeasIntvIndHandler(ke_msg_id_t const msgid,
                               struct htpc_meas_intv_ind *param,
                               ke_task_id_t const dest_id,
                               ke_task_id_t const src_id);

#endif
/*! @brief @} APP_HTPC_API */
#endif /* _APP_HTPC_H_ */

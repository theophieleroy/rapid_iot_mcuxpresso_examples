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
#ifndef _APP_HOGPRH_H_
#define _APP_HOGPRH_H_

/*!
 * @addtogroup APP_HOGPRH_API
 * @{
 */

#if BLE_HID_REPORT_HOST

#include "hogprh.h"
#include "hogprh_task.h"

/*******************************************************************************
 * Variables
 ******************************************************************************/
/*! @brief Table of message handlers */
extern const struct ke_state_handler g_AppHogprhTableHandler;

/*******************************************************************************
 * API
 ******************************************************************************/
/*!
 * @brief Add HID Over GATT Profile Report Host.
 */
void APP_HogprhAddProfileTask(void);

/*!
 * @brief Start the profile at connection.
 * @param[in] conhdl        Connection handle
 * @param[in] type          Connection type
 * @param[in] hids_nb       Number of instances of the HID Service that have been found 
 * @param[in] hids          Information about HID Services that have been found 
 * @response  HOGPRH_ENABLE_RSP
 * @description
 * This API message is used for enabling the Report Host role of the HOGP.
 */
void APP_HogprhEnableReq(uint16_t conhdl,uint8_t type,uint8_t hids_nb,struct hogprh_content hids[HOGPRH_NB_HIDS_INST_MAX]);

/*!
 * @brief Handles the response of HOGPRH_ENABLE_REQ.
 * @param[in] msgid     HOGPRH_ENABLE_RSP
 * @param[in] param     hogprh_enable_rsp
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_HOGPRH_IDX
 * @return If the message was consumed or not.
 * @description
 * This API message is used by the Report Host to either send the discovery results of HIDS on the HID
 * device and confirm enabling of the Report Host role, or to simply confirm enabling of Report Host role if it is a normal
 * connection and the attribute details are already known.
 */
int APP_HogprhEnableRspHandler(ke_msg_id_t const msgid,
                               struct hogprh_enable_rsp *param,
                               ke_task_id_t const dest_id,
                               ke_task_id_t const src_id);



/*!
 * @brief Read the value of a characteristic.
 * @param[in] conhdl        Connection handle 
 * @param[in] info          Characteristic info
 *           HOGPRH_PROTO_MODE
 *           HOGPRH_REPORT_MAP
 *           HOGPRH_REPORT_MAP_EXT_REP_REF
 *           HOGPRH_HID_INFO
 *           HOGPRH_HID_CTNL_PT
 *           HOGPRH_REPORT
 *           HOGPRH_REPORT_REF
 *           HOGPRH_REPORT_NTF_CFG
 * @param[in] hid_idx       HID Service Instance
 * @param[in] report_idx    HID Report Index
 *           HOGPRH_REPORT
 *           HOGPRH_REPORT_REF
 *           HOGPRH_REPORT_NTF_CFG
 * @response  
 * @description
 * This API message shall be used to read the value of a characteristic or a descriptor in the HID Device
 * database.
 */
void APP_HogprhReadInfoReq(uint16_t conhdl, uint8_t info, uint8_t hid_idx, uint8_t report_idx);

/*!
 * @brief Handles the response of HOGPRH_READ_INFO_REQ.
 * @param[in] msgid     HOGPRH_READ_INFO_RSP
 * @param[in] param     hogprh_read_info_rsp
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_HOGPRH_IDX
 * @return If the message was consumed or not.
 * @description
 * This response contains value of requested data or an error code if nothing has been found.
 */
int APP_HogprhReadInfoRspHandler(ke_msg_id_t const msgid,
                                 struct hogprh_read_info_rsp *param,
                                 ke_task_id_t const dest_id,
                                 ke_task_id_t const src_id);




/*!
 * @brief Write the value to peer device.
 * @param[in] conhdl        Connection handle 
 * @param[in] info          Characteristic info
 *           HOGPRH_PROTO_MODE
 *           HOGPRH_REPORT_MAP
 *           HOGPRH_REPORT_MAP_EXT_REP_REF
 *           HOGPRH_HID_INFO
 *           HOGPRH_HID_CTNL_PT
 *           HOGPRH_REPORT
 *           HOGPRH_REPORT_REF
 *           HOGPRH_REPORT_NTF_CFG
 * @param[in] hid_idx       HID Service Instance
 * @param[in] report_idx    HID Report Index
 *           HOGPRH_REPORT
 *           HOGPRH_REPORT_NTF_CFG
 * @param[in] wr_cmd        Write type ( Write without Response True or Write Request)
 * @param[in] wr_cmd        Information data 
 * @response  
 * @description
 * This API message shall be used to:
 *   - Enable or disable the notifications for a Report Characteristic instance.
 *   - Write the HID Control Point Characteristic value in the peer device database.
 *   - Set the protocol mode of a HID Service instance to the Report Procotol Mode.
 *   - Write the value of a Report Characteristic in the peer device database
 */
void APP_HogprhWriteReq(uint16_t conhdl, uint8_t info, uint8_t hid_idx, uint8_t report_idx, bool  wr_cmd, union hogprh_data *data);


/*!
 * @brief Handles the response of HOGPRH_WRITE_REQ.
 * @param[in] msgid     HOGPRH_WRITE_RSP
 * @param[in] param     hogprh_write_rsp
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_HOGPRH_IDX
 * @return If the message was consumed or not.
 * @description
 * The API message is used to inform the application about the write request status.
 */
int APP_HogprhWriteRspHandler(ke_msg_id_t const msgid,
                             struct hogprh_write_rsp *param,
                             ke_task_id_t const dest_id,
                             ke_task_id_t const src_id);


/*!
 * @brief Receive report from peer device.
 * @param[in] msgid     HOGPRH_REPORT_IND
 * @param[in] param     hogprh_report_ind
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_HOGPRH_IDX
 * @return If the message was consumed or not.
 * @description
 * The API message is used to inform the application about a modification of report on peer database
 */
int APP_HogprhReportIndHandler(ke_msg_id_t const msgid,
                             struct hogprh_report_ind *param,
                             ke_task_id_t const dest_id,
                             ke_task_id_t const src_id);



#endif
/*! @brief @} APP_HOGPRH_API */
#endif /* _APP_HOGPRH_H_ */


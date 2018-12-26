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
#ifndef _APP_HOGPBH_H_
#define _APP_HOGPBH_H_

/*!
 * @addtogroup APP_HOGPBH_API
 * @{
 */

#if BLE_HID_BOOT_HOST
#include "hogpbh.h"
#include "hogpbh_task.h"

/*******************************************************************************
 * Variables
 ******************************************************************************/
/*! @brief Table of message handlers */
extern const struct ke_state_handler g_AppHogpbhTableHandler;

/*******************************************************************************
 * API
 ******************************************************************************/
/*!
 * @brief Add HID Over GATT Profile Boot Host.
 */
void APP_HogpbhAddProfileTask(void);

/*!
 * @brief Start the profile at connection.
 * @param[in] conhdl        Connection handle
 * @param[in] hids_nb       Number of instances of the HID Service that have been found 
 * @param[in] hids          Information about HID Services that have been found 
 * @response  HOGPBH_ENABLE_RSP
 * @description
 * This API message is used for enabling the Boot Host role of the HOGP. This Application message contains
 * BLE connection handle, the connection type and the previously saved discovered HIDS details on peer.
 */
 void APP_HogpbhEnableReq(uint16_t conhdl,uint8_t type, uint8_t hids_nb, struct hogpbh_content hids[HOGPBH_NB_HIDS_INST_MAX]);

/*!
 * @brief Handles the response of HOGPBH_ENABLE_REQ.
 * @param[in] msgid     HOGPBH_ENABLE_RSP
 * @param[in] param     hogpbh_enable_rsp
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_HOGPBH_IDX
 * @return If the message was consumed or not.
 * @description
 * This API message is used by the Boot Host to either send the discovery results of HIDS on the HID device
 * and confirm enabling of the Boot Host role, or to simply confirm enabling of Boot Host role if it is a 
 * normal connection and the attribute details are already known.
 */
int APP_HogpbhEnableRspHandler(ke_msg_id_t const msgid,
                               struct hogpbh_enable_rsp *param,
                               ke_task_id_t const dest_id,
                               ke_task_id_t const src_id);




/*!
 * @brief Read value of characteristic.
 * @param[in] conhdl        Connection handle
 * @param[in] info          Characteristic info
 *               HOGPBH_PROTO_MODE
 *               HOGPBH_BOOT_KB_IN_REPORT
 *               HOGPBH_BOOT_KB_OUT_REPORT
 *               HOGPBH_BOOT_MOUSE_IN_REPORT
 *               HOGPBH_BOOT_KB_IN_NTF_CFG
 *               HOGPBH_BOOT_MOUSE_IN_NTF_CFG
 * @param[in] hid_idx       HID Service Instance
 * @response  HOGPBH_READ_INFO_RSP
 * @description
 * This API message shall be used to read the value of a characteristic or a descriptor in the HID 
 * Device database.
 */
void APP_HogpbhReadInfoReq(uint16_t conhdl,uint8_t info,uint8_t hid_idx);


/*!
 * @brief Handles the response of HOGPBH_READ_INFO_REQ.
 * @param[in] msgid     HOGPBH_READ_INFO_RSP
 * @param[in] param     hogpbh_read_info_rsp
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_HOGPBH_IDX
 * @return If the message was consumed or not.
 * @description
 * The API message is used to inform the application about the read Client Characteristic Configuration
 * Descriptor value.
 */
int APP_HogpbhReadInfoRspHandler(ke_msg_id_t const msgid,
                                 struct hogpbh_read_info_rsp *param,
                                 ke_task_id_t const dest_id,
                                 ke_task_id_t const src_id);


/*!
 * @brief Enable or disable the notifications.
 * @param[in] conhdl        Connection handle
 * @param[in] info          Characteristic info
 * @param[in] hid_idx       HID Service Instance
 * @param[in] wr_cmd        Write type
 * @param[in] data          Information data
 * @response  HOGPBH_WRITE_RSP
 * @description
 * This API message shall be used to:
 *    - Enable or disable the notifications for the Boot Keyboard Input Characteristic
 *    - Enable or disable the notifications for the Boot Mouse Input Characteristic.
 *    - Write the value of one of the Boot Report Characteristics in the peer device database.
 */
void APP_HogpbhWriteReq(uint16_t conhdl,uint8_t info,uint8_t hid_idx,bool wr_cmd,union hogpbh_data *data);


/*!
 * @brief Handles the response of HOGPBH_WRITE_REQ.
 * @param[in] msgid     HOGPBH_WRITE_RSP
 * @param[in] param     hogpbh_write_rsp
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_HOGPBH_IDX
 * @return If the message was consumed or not.
 * @description
 * The API message is used to inform the application about status of the write request.
 */
int APP_HogpbhWriteRspHandler(ke_msg_id_t const msgid,
                              struct hogpbh_write_rsp *param,
                              ke_task_id_t const dest_id,
                              ke_task_id_t const src_id);


/*!
 * @brief Receive report from peer device .
 * @param[in] msgid     HOGPBH_BOOT_REPORT_IND
 * @param[in] param     hogpbh_boot_report_ind
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_HOGPBH_IDX
 * @return If the message was consumed or not.
 * @description
 * The API message is used to inform the application about a new value has been received within a read
 * response message or a notification for one of the Boot Report Characteristics present in the peer device database.
 */

int APP_HogpbhBootReportIndHandler(ke_msg_id_t const msgid,
                                   struct hogpbh_boot_report_ind *param,
                                   ke_task_id_t const dest_id,
                                   ke_task_id_t const src_id);


#endif
/*! @brief @} APP_HOGPBH_API */
#endif /* _APP_HOGPBH_H_ */


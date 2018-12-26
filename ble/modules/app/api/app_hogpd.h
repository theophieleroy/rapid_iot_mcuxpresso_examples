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

#ifndef APP_HOGPD_H_
#define APP_HOGPD_H_

/*!
 * @addtogroup APP_HOGPD_API
 * @{
 */

#if (BLE_HID_DEVICE)
#include "hogpd.h"
#include "hogpd_task.h"
#include "prf_utils.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define MOUSE_INSTANCE (0)
#define KEYBOARD_INSTANCE (1)
#define BOOT_REPORTS_INSTANCE (0)
#define MOUSE_REPORT_DATA_LEN (3)
#define MOUSE_LEFT_PRESS (1)
#define MOUSE_MIDDLE_PRESS (1 << 1)
#define MOUSE_RIGHT_PRESS (1 << 2)
#define MOUSE_NOT_PRESS (0)
#define KEYBOARD_REPORT_DATA_LEN (8)
#define MOUSE_REPORT_NUMBER (1)
#define MOUSE_INPUT_REPORT_ID (1)
#define BINARY_CODED_DECIMAL (0x150U)
#define COUNTRY_CODE (0x56U)
#define HID_NOTIFY_INTERVAL (100)
#define KEYBOARD_OUTPUT_CAPS_LOCK (0x02)

struct app_hogpd_env_tag
{
    uint16_t hid_noti_intv;  /*!< HID Device Interval value */
    uint8_t current_idx;     /*!< Current Connect Device index */
    uint8_t protocol_mode;   /*!< Protocol Mode */
    uint8_t pt_suspend_flag; /*!< Control Point Suspend flag */
};

/*******************************************************************************
 * Variables
 ******************************************************************************/
/*! @brief Table of message handler */
extern const struct ke_state_handler g_AppHogpdTableHandler;

/*! @brief Application hogpd environment */
extern struct app_hogpd_env_tag g_AppHogpdEnv;

/*******************************************************************************
 * API
 ******************************************************************************/
/*!
 * @brief Add HID Over GATT Profile device.
 */
void APP_HogpdAddProfileTask(void);

/*!
 * @brief Start the profile at connection.
 * @param[in] conhdl                           Connection handle
 * @param[in] ntf_cfg[HOGPD_NB_HIDS_INST_MAX]  Saved notification configurations
 *          HOGPD_OP_NO
 *          HOGPD_OP_REPORT_READ
 *          HOGPD_OP_REPORT_WRITE
 *          HOGPD_OP_PROT_UPDATE
 * @response  HOGPD_ENABLE_RSP
 * @description
 * This API message can be used after the connection with a peer device has been established
 * in order to restore BOND data of a known device.
 */
void APP_HogpdEnableReq(uint16_t conhdl, uint8_t ntf_cfg[HOGPD_NB_HIDS_INST_MAX]);

/*!
* @brief Response of HOGPD_ENABLE_REQ.
* @param[in] msgid     HOGPD_ENABLE_RSP
* @param[in] param     hogpd_enable_rsp
* @param[in] dest_id   TASK_APP
* @param[in] src_id    TASK_HOGPD
* @return If the message was consumed or not.
* @description
* Inform application if restoring bond data for peer device succeed or not.
*/
int APP_HogpdEnableRspHandler(ke_msg_id_t const msgid,
                              struct hogpd_enable_rsp *param,
                              ke_task_id_t const dest_id,
                              ke_task_id_t const src_id);

/*!
* @brief Enable or disable notifications configuration.
* @param[in] msgid     HOGPD_NTF_CFG_IND
* @param[in] param     hogpd_ntf_cfg_ind
* @param[in] dest_id   TASK_APP
* @param[in] src_id    TASK_HOGPD
* @return If the message was consumed or not.
* @description
* This API message is sent to the application each time the host enables or disables
* sending of notifications for a characteristic.
*/
int APP_HogpdNtfCfgIndHandler(ke_msg_id_t const msgid,
                              struct hogpd_ntf_cfg_ind *param,
                              ke_task_id_t const dest_id,
                              ke_task_id_t const src_id);

/*!
* @brief Peer device write protocol mode value.
* @param[in] msgid     HOGPD_PROTO_MODE_REQ_IND
* @param[in] param     hogpd_proto_mode_req_ind
* @param[in] dest_id   TASK_APP
* @param[in] src_id    TASK_HOGPD
* @return If the message was consumed or not.
* @description
* This API message is used by the HID Device role to inform the application that a Protocol Mode
* characteristic value has been written by a peer device.
*/
int APP_HogpdProtoModeReqIndHandler(ke_msg_id_t const msgid,
                                    struct hogpd_proto_mode_req_ind *param,
                                    ke_task_id_t const dest_id,
                                    ke_task_id_t const src_id);

/*!
 * @brief Confirm the protocol mode.
 * @param[in] conhdl         Connection handle
 * @param[in] status         Status code
 * @param[in] Hids_idx       HID Service instance.
 * @param[in] proto_mode     New Protocol Mode Characteristic value
 *         HOGP_BOOT_PROTOCOL_MODE
 *         HOGP_REPORT_PROTOCOL_MODE
 * @description
 * This API message is used to confirm or not modification of the protocol mode.
 */
void APP_HogpdProtoModeCfm(uint16_t conhdl, uint8_t status, uint8_t hid_idx, uint8_t proto_mode);

/*!
* @brief Receive CP request.
* @param[in] msgid     HOGPD_CTNL_PT_IND
* @param[in] param     hogpd_ctnl_pt_ind
* @param[in] dest_id   TASK_APP
* @param[in] src_id    TASK_HOGPD
* @return If the message was consumed or not.
* @description
* This API message is used by the HID Device role to inform the application that a HID Control Point
* Characteristic value has been written by a peer device.
*/
int APP_HogpdCtnlPtIndHandler(ke_msg_id_t const msgid,
                              struct hogpd_ctnl_pt_ind *param,
                              ke_task_id_t const dest_id,
                              ke_task_id_t const src_id);

/*!
* @brief Reponse of HOGPD_REPORT_UPD_REQ.
* @param[in] msgid     HOGPD_REPORT_UPD_RSP
* @param[in] param     hogpd_report_upd_rsp
* @param[in] dest_id   TASK_APP
* @param[in] src_id    TASK_HOGPD
* @return If the message was consumed or not.
* @description
* This message is sent to the application after reception of the HOGPD_REPORT_UPD_REQ to inform
* it if a notification has been sent to the Host or if an error has been raised.
*/
int APP_HogpdReportUpdRspHandler(ke_msg_id_t const msgid,
                                 struct hogpd_report_upd_rsp *param,
                                 ke_task_id_t const dest_id,
                                 ke_task_id_t const src_id);

/*!
* @brief Receive report from peer device.
* @param[in] msgid     HOGPD_REPORT_REQ_IND
* @param[in] param     hogpd_report_req_ind
* @param[in] dest_id   TASK_APP
* @param[in] src_id    TASK_HOGPD
* @return If the message was consumed or not.
* @description
* This message is sent to the application after the peer Host has written the value of one of
* the Report Characteristics or if peer device request information of a report value.
*/
int APP_HogpdReportReqIndHandler(ke_msg_id_t const msgid,
                                 struct hogpd_report_req_ind *param,
                                 ke_task_id_t const dest_id,
                                 ke_task_id_t const src_id);

/*!
 * @brief Response HOGPD_REPORT_REQ_IND.
 * @param[in] conhdl         Connection handle
 * @param[in] operation      Operation requested
 *            HOGPD_OP_NO              0x01
 *            HOGPD_OP_REPORT_READ     0x02
 *            HOGPD_OP_REPORT_WRITE    0x03
 *            HOGPD_OP_PROT_UPDATE     0x04
 * @param[in] status         Status code
 * @param[in] report         Report Info
 * @description
 * This message is sent by the application to confirm read or modification of a report.
 */
void APP_HogpdReportCfm(uint16_t conhdl, uint8_t status, uint8_t operation, struct hogpd_report_info *report);

#endif
/*! @brief @} APP_HOGPD_API */
#endif

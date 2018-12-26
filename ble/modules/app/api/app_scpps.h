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

#ifndef APP_SCPPS_H_
#define APP_SCPPS_H_

/*!
 * @addtogroup APP_SCPPS_API
 * @{
 */

#if (BLE_SP_SERVER)
#include "scpps.h"
#include "scpps_task.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

struct app_scpps_env_tag
{
    uint16_t le_scan_intv[BLE_CONNECTION_MAX];   /*!< Scan interval configuration */
    uint16_t le_scan_window[BLE_CONNECTION_MAX]; /*!< Scan window configuration  */
};

/*******************************************************************************
 * Variables
 ******************************************************************************/
/*! @brief Table of message handler */
extern const struct ke_state_handler g_AppScppsTableHandler;

/*! @brief Application scpps environment */
extern struct app_scpps_env_tag g_AppScppsEnv;

/*******************************************************************************
 * API
 ******************************************************************************/
/*!
 * @brief Add Scan Parameters Profile.
 */
void APP_ScppsAddProfileTask(void);

/*!
 * @brief Start the profile at connection.
 * @param[in] conhdl        Connection handle
 * @param[in] ntf_cfg       Notification configuration
 * @response  SCPPS_ENABLE_RSP
 * @description
 * This API message can be used after the connection to restore bond data (peer notification configuration)
 * of a known device.
 */
void APP_ScppsEnableReq(uint16_t conhdl, uint8_t ntf_ind_cfg);

/*!
* @brief Response of SCPPS_ENABLE_REQ.
* @param[in] msgid     SCPPS_ENABLE_RSP
* @param[in] param     scpps_enable_rsp
* @param[in] dest_id   TASK_APP
* @param[in] src_id    TASK_SCPPS
* @return If the message was consumed or not.
* @description
* Inform application if restoring bond data for peer device succeed or not.
*/
int APP_ScppsEnableRspHandler(ke_msg_id_t const msgid,
                              struct scpps_enable_rsp *param,
                              ke_task_id_t const dest_id,
                              ke_task_id_t const src_id);

/*!
* @brief Peer device write Scan Interval Window value.
* @param[in] msgid     SCPPS_SCAN_INTV_WD_IND
* @param[in] param     scpps_scan_intv_wd_ind
* @param[in] dest_id   TASK_APP
* @param[in] src_id    TASK_SCPPS
* @return If the message was consumed or not.
* @description
* This API message informs the application that the Scan Interval Window Characteristic value has been
* written by the peer device.
*/
int APP_ScppsScanIntvWdIndHandler(ke_msg_id_t const msgid,
                                  struct scpps_scan_intv_wd_ind *param,
                                  ke_task_id_t const dest_id,
                                  ke_task_id_t const src_id);

/*!
* @brief Peer devie enable or disable notification configuration.
* @param[in] msgid     SCPPS_SCAN_REFRESH_NTF_CFG_IND
* @param[in] param     scpps_scan_refresh_ntf_cfg_ind
* @param[in] dest_id   TASK_APP
* @param[in] src_id    TASK_SCPPS
* @return If the message was consumed or not.
* @description
* This API message is sent to the application to inform it that the peer device has enabled or disabled
* sending of notifications for the Scan Refresh Characteristic.
*/
int APP_ScppsScanRefreshNtfCfgIndHandler(ke_msg_id_t const msgid,
                                         struct scpps_scan_refresh_ntf_cfg_ind *param,
                                         ke_task_id_t const dest_id,
                                         ke_task_id_t const src_id);

/*!
 * @brief Send refresh scan parameters request to peer device.
 * @param[in] conhdl        Connection handle
 * @response  SCPPS_SCAN_REFRESH_SEND_RSP
 * @description
 * This API message is used notify the Client that the Server writes the latest intended scan parameters to
 * the Scan Interval Window Characteristic.
 */
void APP_ScppsScanRefreshSendReq(uint16_t conhdl);

/*!
* @brief Response of SCPPS_SCAN_REFRESH_SEND_REQ.
* @param[in] msgid     SCPPS_SCAN_REFRESH_SEND_RSP
* @param[in] param     scpps_scan_refresh_send_rsp
* @param[in] dest_id   TASK_APP
* @param[in] src_id    TASK_SCPPS
* @return If the message was consumed or not.
* @description
* This API message is used to inform the application if the Scan Refresh Characteristic value has been
* notified or not.
*/
int APP_ScppsScanRefreshSendRspHandler(ke_msg_id_t const msgid,
                                       struct scpps_scan_refresh_send_rsp *param,
                                       ke_task_id_t const dest_id,
                                       ke_task_id_t const src_id);

#endif
/*! @brief @} APP_SCPPS_API */
#endif

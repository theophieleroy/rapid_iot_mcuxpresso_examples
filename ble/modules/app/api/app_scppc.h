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
#ifndef _APP_SCPPC_H_
#define _APP_SCPPC_H_

/*!
 * @addtogroup APP_SCPPC_API
 * @{
 */

#if BLE_SP_CLIENT
#include "scppc.h"
#include "scppc_task.h"

/*******************************************************************************
 * Variables
 ******************************************************************************/
/*! @brief Parameters of the @ref SCPPC_SCAN_REFRESH_NTF_CFG_RD_REQ message*/
struct scppc_scan_refresh_ntf_cfg_rd_req
{
    uint8_t dummy; /*!< dummy */
};

/*! @brief Table of message handlers */
extern const struct ke_state_handler g_AppScppcTableHandler;

/*******************************************************************************
 * API
 ******************************************************************************/
/*!
 * @brief Add Scan Parameters Profile Client.
 */
void APP_ScppcAddProfileTask(void);

/*!
 * @brief Start the profile at connection.
 * @param[in] conhdl        Connection handle
 * @response  SCPPC_ENABLE_RSP
 * @description
 * This API message is used for enabling the Client role of the SCPP.
 */
void APP_ScppcEnableReq(uint16_t conhdl);

/*!
 * @brief Handles the response of SCPPC_ENABLE_REQ.
 * @param[in] msgid     SCPPC_ENABLE_RSP
 * @param[in] param     scppc_enable_rsp
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_SCPPC_IDX
 * @return If the message was consumed or not.
 * @description
 * This API message is used by the Client role task to either send the discovery results of SCPS on the peer
 * device and confirm enabling of the Client role, or to simply confirm enabling of Client role if it is a
 * normal connection and the attribute details are already known.
 */
int APP_ScppcEnableRspHandler(ke_msg_id_t const msgid,
                              struct scppc_enable_rsp *param,
                              ke_task_id_t const dest_id,
                              ke_task_id_t const src_id);

/*!
 * @brief Client write scan parameter .
 * @param[in] conhdl           Connection handle
 * @param[in] le_scan_intv     Scan interval configuration
 * @param[in] le_scan_window   Scan window configuration
 * @response  SCPPC_SCAN_INTV_WD_WR_RSP
 * @description
 * This API message shall be used to inform the Scan Server that the Scan Client has changed its intended
 * scanning behavior.
 */
void APP_ScppcScanIntvWdWrReq(uint16_t conhdl, uint16_t le_scan_intv, uint16_t le_scan_window);

/*!
 * @brief Handles the response of SCPPC_SCAN_INTV_WD_WR_REQ.
 * @param[in] msgid     SCPPC_SCAN_INTV_WD_WR_RSP
 * @param[in] param     scppc_scan_intv_wd_wr_rsp
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_SCPPC_IDX
 * @return If the message was consumed or not.
 * @description
 * This API message is sent to the application when a write response has been received from the peer device
 * after sending of a write request.
 */
int APP_ScppcScanIntvWdWrRspHandler(ke_msg_id_t const msgid,
                                    struct scppc_scan_intv_wd_wr_rsp *param,
                                    ke_task_id_t const dest_id,
                                    ke_task_id_t const src_id);

/*!
 * @brief Read notification configuration .
 * @param[in] conhdl           Connection handle
 * @response  SCPPC_SCAN_REFRESH_NTF_CFG_RD_RSP
 * @description
 * This API message shall be used to read the value of the Scan Refresh Characteristic Client Characteristic
 * Configuration Descriptor.
 */
void APP_ScppcScanRefreshNtfCfgRdReq(uint16_t conhdl);

/*!
 * @brief Handles the response of SCPPC_SCAN_REFRESH_NTF_CFG_RD_REQ.
 * @param[in] msgid     SCPPC_SCAN_REFRESH_NTF_CFG_RD_RSP
 * @param[in] param     scppc_scan_refresh_ntf_cfg_rd_rsp
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_SCPPC_IDX
 * @return If the message was consumed or not.
 * @description
 * This API message is sent to the application to inform it about the read Client Characteristic Configuration
 * Descriptor value for the Scan Refresh Characteristic.
 */
int APP_ScppcScanRefreshNtfCfgRdRspHandler(ke_msg_id_t const msgid,
                                           struct scppc_scan_refresh_ntf_cfg_rd_rsp *param,
                                           ke_task_id_t const dest_id,
                                           ke_task_id_t const src_id);

/*!
 * @brief  Enable or disable notifications.
 * @param[in] conhdl           Connection handle
 * @param[in] ntf_cfg          Configuration value to write
 *         PRF_CLI_STOP_NTFIND
 *         PRF_CLI_START_NTF
 * @response  SCPPC_SCAN_REFRESH_NTF_CFG_RSP
 * @description
 * This API message shall be used to either enable or disable notifications for the Scan Refresh Characteristic.
 */
void APP_ScppcScanRefreshNtfCfgReq(uint16_t conhdl, uint16_t ntf_cfg);

/*!
 * @brief Handles the response of SCPPC_SCAN_REFRESH_NTF_CFG_REQ.
 * @param[in] msgid     SCPPC_SCAN_REFRESH_NTF_CFG_RSP
 * @param[in] param     scppc_scan_refresh_ntf_cfg_rsp
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_SCPPC_IDX
 * @return If the message was consumed or not.
 * @description
 * This API message is sent to the application when a write response has been received from the peer device
 * after sending of a write request.
 */
int APP_ScppcScanRefreshNtfCfgRspHandler(ke_msg_id_t const msgid,
                                         struct scppc_scan_refresh_ntf_cfg_rsp *param,
                                         ke_task_id_t const dest_id,
                                         ke_task_id_t const src_id);

/*!
 * @brief Server request a refresh of scan paramters value .
 * @param[in] msgid     SCPPC_SCAN_REFRESH_IND
 * @param[in] param     param
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_SCPPC_IDX
 * @return If the message was consumed or not.
 * @description
 * This API message is sent to the application when peer device request a refresh of scan parameters values .
 */
int APP_ScppcScanRefreshIndHandler(ke_msg_id_t const msgid,
                                   void *param,
                                   ke_task_id_t const dest_id,
                                   ke_task_id_t const src_id);

#endif
/*! @brief @} APP_SCPPC_API */
#endif /* _APP_SCPPC_H_ */

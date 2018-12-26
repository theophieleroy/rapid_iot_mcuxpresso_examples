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
#ifndef _APP_PROXM_H_
#define _APP_PROXM_H_

/*!
 * @addtogroup APP_PROXM_API
 * @{
 */

#if BLE_PROX_MONITOR
#include "proxm.h"
#include "proxm_task.h"

/*******************************************************************************
 * Variables
 ******************************************************************************/
/*! @brief Table of message handlers */
extern const struct ke_state_handler g_AppProxmTableHandler;

/*******************************************************************************
 * API
 ******************************************************************************/
/*!
 * @brief Add proximity monitor profile.
 */
void APP_ProxmAddProfileTask(void);

/*!
 * @brief Start the profile at connection.
 * @param[in] conhdl        Connection handle
 * @response  PROXM_ENABLE_RSP
 * @description
 * This API message is used for enabling the monitor role of the proximity profile.
 * Profile will discovery all the attributes related to proximity reporter.
 */
void APP_ProxmEnableReq(uint16_t conhdl);

/*!
 * @brief Handles the enable confirmation from the PROXM.
 * @param[in] msgid     PROXM_ENABLE_RSP
 * @param[in] param     proxm_enable_rsp
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_PROXM_IDX
 * @return If the message was consumed or not.
 * @description
 * This API message is used to receive the discovery results of LLS, IAS and TPS on
 * Reporter and confirm enabling of the monitor role.
 */
int APP_ProxmEnableRspHandler(ke_msg_id_t const msgid,
                              struct proxm_enable_rsp *param,
                              ke_task_id_t const dest_id,
                              ke_task_id_t const src_id);

/*!
 * @brief Read alert or tx power value from server.
 * @param[in] conhdl        Connection handle
 * @param[in] svc_code      Code for the service:
 *        PROXM_RD_LL_ALERT_LVL = 0x00,
 *        PROXM_RD_TX_POWER_LVL,
 * @response  PROXM_RD_RSP
 * @description
 * This API message is used for reading the alert level in LLS Alert Level Characteristic
 * or the TX Power level value of peer device service.
 */
void APP_ProxmRdReq(uint16_t conhdl, uint8_t svc_code);

/*!
 * @brief Handles the read character value confirmation from the proxm.
 * @param[in] msgid     PROXM_RD_RSP
 * @param[in] param     proxm_rd_rsp
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_PROXM_IDX
 * @return If the message was consumed or not.
 * @description
 * Response of the Read request of Link Loss alert value or TX Power value..
 */
int APP_ProxmRdRspHandler(ke_msg_id_t const msgid,
                          struct proxm_rd_rsp *param,
                          ke_task_id_t const dest_id,
                          ke_task_id_t const src_id);

/*!
 * @brief Write alert level to server.
 * @param[in] conhdl        Connection handle
 * @param[in] svc_code      Code for the service:
 *        PROXM_SET_LK_LOSS_ALERT
 *        PROXM_SET_IMMDT_ALERT
 * @param[in] lvl     The alert level:
 *        PROXM_ALERT_NONE  = 0x00,
 *        ROXM_ALERT_MILD,
 *        PROXM_ALERT_HIGH,
 * @response  PROXM_WR_ALERT_LVL_RSP
 * @description
 * This API message is used by the application to write either a LLS alert level or an IAS alert level.
 */
void APP_ProxmWrAlertLvlReq(uint16_t conhdl, uint8_t svc_code, uint8_t lvl);

/*!
 * @brief Handles the write response.
 * @param[in] msgid     PROXM_WR_ALERT_LVL_RSP
 * @param[in] param     proxm_wr_alert_lvl_rsp
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_PROXM_IDX
 * @return If the message was consumed or not.
 * @description
 * Response of the write request of Link Loss or immediate alert value.
 */
int APP_ProxmWrAlertLvlRspHandler(ke_msg_id_t const msgid,
                                  struct proxm_wr_alert_lvl_rsp *param,
                                  ke_task_id_t const dest_id,
                                  ke_task_id_t const src_id);

#endif

#endif /* _APP_PROXM_H_ */

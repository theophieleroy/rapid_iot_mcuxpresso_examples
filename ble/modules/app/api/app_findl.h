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
#ifndef _APP_FINDL_H_
#define _APP_FINDL_H_

/*!
 * @addtogroup APP_FINDL_API
 * @{
 */

#if BLE_FINDME_LOCATOR
#include "findl_task.h"

/*******************************************************************************
 * Variables
 ******************************************************************************/
/*! @brief Table of message handlers */
extern const struct ke_state_handler g_AppFindlTableHandler;

/*******************************************************************************
 * API
 ******************************************************************************/
/*!
 * @brief Add Find Me Locator Profile .
 */
void APP_FindlAddProfileTask(void);

/*!
 * @brief Start the profile at connection.
 * @param[in] conhdl        Connection handle
 * @response  FINDL_ENABLE_RSP
 * @description
 *  This API message is used for enabling the Locator role of the Find Me profile.
 */
void APP_FindlEnableReq(uint16_t conhdl);

/*!
 * @brief Handles the response of FINDL_ENABLE_REQ.
 *
 * @param[in] msgid     FINDL_ENABLE_RSP
 * @param[in] param     findl_enable_rsp
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_FINDL_IDX
 * @return If the message was consumed or not.
 * @description
 * This API message is used by the Locator to either send the discovery results of IAS on Target
 * and confirm enabling of the Locator role, or to simply confirm enabling of Locator role if it
 * is a normal connection and the IAS details are already known.
 */
int APP_FindlEnableRspHandler(ke_msg_id_t const msgid,
                              struct findl_enable_rsp *param,
                              ke_task_id_t const dest_id,
                              ke_task_id_t const src_id);

/*!
 * @brief Set a alert level on the peer target device.
 * @param[in] conhdl        Connection handle
 * @param[in] alert_lvl     The alert level:
 *        FINDL_ALERT_NONE,
 *        FINDL_ALERT_MILD,
 *        FINDL_ALERT_HIGH,
 * @response  FINDL_SET_ALERT_RSP
 * @description
 * This API message is used by the application to trigger/stop and alert on the peer Target device.
 */
void APP_FindlSetAlertReq(uint16_t conhdl, uint8_t alert_lvl);

/*!
 * @brief Handles the response of FINDL_SET_ALERT_REQ.
 *
 * @param[in] msgid     FINDL_SET_ALERT_RSP
 * @param[in] param     findl_enable_rsp
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_FINDL_IDX
 * @return If the message was consumed or not.
 * @description
 * Inform Application that alert has been written on peer device.
 */
int APP_FindlSetAlertRspHandler(ke_msg_id_t const msgid,
                                struct findl_set_alert_rsp *param,
                                ke_task_id_t const dest_id,
                                ke_task_id_t const src_id);

#endif
/*! @brief @} APP_FINDL_API */

#endif /* _APP_FINDL_H_ */

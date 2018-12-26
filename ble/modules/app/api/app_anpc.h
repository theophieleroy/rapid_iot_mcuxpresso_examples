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
#ifndef _APP_ANPC_H_
#define _APP_ANPC_H_

/*!
 * @addtogroup APP_ANPC_API
 * @{
 */

#if BLE_AN_CLIENT
#include "anpc.h"
#include "anpc_task.h"

/*******************************************************************************
 * Variables
 ******************************************************************************/
/*! @brief Table of message handlers */
extern const struct ke_state_handler g_AppAnpcTableHandler;

/*******************************************************************************
 * API
 ******************************************************************************/

/*!
 * @brief Add a Alert Notification Profile Client instance.
 */
void APP_AnpcAddProfileTask(void);

/*!
 * @brief Start the profile - at connection.
 * @param[in] conhdl        Connection handle for which the profile client role is enabled.
 * @response  ANPC_ENABLE_RSP
 * @description
 * This API is used for enabling the Client role of the ANP. Profile will discovery all
 * the attributes related to ANS.
 */
void APP_AnpcEnableReq(uint16_t conhdl);

/*!
 * @brief The API message shall be used to read the value of an attribute in the peer device database.
 * @param[in] conhdl        Connection handle for which the profile PASP client role is enabled
 * @param[in] read_code     Code for which characteristic to read:
 *  - ANPC_RD_SUP_NEW_ALERT_CAT
 *  - ANPC_RD_SUP_UNREAD_ALERT_CAT
 *  - ANPC_RD_WR_NEW_ALERT_CFG
 *  - ANPC_RD_WR_UNREAD_ALERT_STATUS_CFG
 * @response  ANPC_VALUE_IND and ANPC_CMP_EVT
 * @description
 * TThe API message shall be used to read the value of an attribute in the peer device database.
 */
void APP_AnpcReadCmd(uint16_t conhdl, uint8_t read_code);

/*!
 * @brief This API message shall be used by the application to write the value of one of the writable attribute in the
 * peer device database.
 * @param[in] conhdl        Connection handle
 * @param[in] write_cmd     Write_code and the value to write
 * @response  ANPC_CMP_EVT
 * @description
 * This API message shall be used by the application to write the value of one of the writable attribute in the
 * peer device database.
 */
void APP_AnpcWriteCmd(uint16_t conhdl, struct anpc_write_cmd write_cmd);

#endif /*BLE_AN_CLIENT */

/*! @brief @} APP_ANPC_API */

#endif /* _APP_ANPC_H_ */

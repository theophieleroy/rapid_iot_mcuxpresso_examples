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

#ifndef _APP_DISC_H_
#define _APP_DISC_H_

/*!
 * @addtogroup APP_DISC_API
 * @{
 */

#if BLE_DIS_CLIENT
#include "disc_task.h"

/*******************************************************************************
 * Variables
 ******************************************************************************/
/*! @brief Table of message handlers */
extern const struct ke_state_handler g_AppDiscTableHandler;

/*******************************************************************************
 * API
 ******************************************************************************/

/*!
 * @brief add disc profile task to kernel.
 */
void APP_DiscAddProfileTask(void);

/*!
 * @brief Start the profile at connection.
 * @param[in] conhdl        Connection handle for which the profile Locator role is enabled.
 * @response  DISC_ENABLE_RSP
 * @description
 * This API message is used for enabling the Client role of the Device Information Service. The Application
 * sends it with connection handle related to the peer device connected with the connection type and the
 * previously saved discovered DIS details on peer.
 */
void APP_DiscEnableReq(uint16_t conhdl);

/*!
 * @brief Read DIS value.
 *
 * @param[in] conhdl    Connection handle
 * @param[in] char_code Characteristic code, possible values are:
 * - DIS_MANUFACTURER_NAME_CHAR
 * - DIS_MODEL_NB_STR_CHAR
 * - DIS_SERIAL_NB_STR_CHAR
 * - DIS_HARD_REV_STR_CHAR
 * - DIS_FIRM_REV_STR_CHAR
 * - DIS_SW_REV_STR_CHAR
 * - DIS_SYSTEM_ID_CHAR
 * - DIS_IEEE_CHAR
 * - DIS_PNP_ID_CHAR
 * @response  DISC_RD_CHAR_RSP or DISC_ERROR_IND
 * @description
 * This API message is used by the application to send a GATT_READ_CHAR_REQ with the parameters deduced from
 * the char_code. Upon reception of this message, DISC checks whether the parameters are correct, then if the
 * handle for the characteristic is valid (not 0x0000) and the request is sent to GATT.
 */
void APP_DiscRdCharReq(uint16_t conhdl, uint8_t char_code);

/*!
 * @brief Handles the enable confirmation from the DISC.
 *
 * @param[in] msgid     DISC_ENABLE_RSP
 * @param[in] param     Pointer to struct disc_enable_rsp
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_DISC
 * @return If the message was consumed or not.
 * @description
 * This API message is used by the Client to either send the discovery results of DIS on Server or confirm enabling
 * of the Client role, or to simply confirm enabling of Client role if it is a normal connection and the DIS details are
 * already known. An error may have also occurred and is signaled.
 */
int APP_DiscEnableRspHandler(ke_msg_id_t const msgid,
                             struct disc_enable_rsp *param,
                             ke_task_id_t const dest_id,
                             ke_task_id_t const src_id);

/*!
 * @brief Handles the read character response message from the DISC.
 *
 * @param[in] msgid     DISC_ENABLE_RSP
 * @param[in] param     Pointer to struct disc_rd_char_rsp
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_DISC
 * @return If the message was consumed or not.
 * @description
 * This API message is used by the Client role to inform the Application of a received read response. The status and
 * the data from the read response are passed directly to Application, which must interpret them based on the
 * request it made.
 */
int APP_DiscRdCharRspHandler(ke_msg_id_t const msgid,
                             struct disc_rd_char_rsp *param,
                             ke_task_id_t const dest_id,
                             ke_task_id_t const src_id);

#endif /*BLE_DIS_CLIENT */
/*! @brief @} APP_DISC_API */

#endif /* _APP_DISC_H_ */

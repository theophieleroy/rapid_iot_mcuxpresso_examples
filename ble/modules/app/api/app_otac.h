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
#ifndef _APP_OTAC_H_
#define _APP_OTAC_H_

/*!
 * @addtogroup APP_OTAC_API OTA Profile Server
 * @{
 */

#if BLE_OTA_CLIENT
#include "otac.h"
#include "otac_task.h"


/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/
/*! @brief Table of message handlers */
extern const struct ke_state_handler g_AppOtacTableHandler;

/*******************************************************************************
 * API
 ******************************************************************************/

/*!
 * @brief Add a Ota Profile Client instance.
 */
void APP_OtacAddProfileTask(void);
/*!
 * @brief Handles the send data response message from the OTAC.
 *
 * @param[in] msgid     OTAC_SEND_DATA_RSP
 * @param[in] param     Pointer to the struct otac_send_data_rsp
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_OTAC
 *
 * @return If the message was consumed or not.
 * @description
 * This handler is used to report to the application a response, or error status of a send data
 * request being sent by application.
 */
int APP_OtacSendDataRspHandler(ke_msg_id_t const msgid,
                               struct otac_send_data_rsp *param,
                               ke_task_id_t const dest_id,
                               ke_task_id_t const src_id);

/*!
 * @brief Handles the ind/ntf indication message from the OTAC.
 *
 * @param[in] msgid     OTAC_CFG_INDNTF_IND
 * @param[in] param     Pointer to the struct otac_cfg_indntf_ind
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_OTAC
 *
 * @return If the message was consumed or not.
 * @description
 * This handler is used to inform application that peer device has changed notification
 * configuration.
 *
 */
int APP_OtacCfgIndntfIndHandler(ke_msg_id_t const msgid,
                                struct otac_cfg_indntf_ind *param,
                                ke_task_id_t const dest_id,
                                ke_task_id_t const src_id);

/*!
 * @brief Handles the data ind message from the OTAC.
 *
 * @param[in] msgid     OTAC_RECV_DATA_IND
 * @param[in] param     Pointer to the struct otac_recv_data_ind
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_OTAC
 *
 * @return If the message was consumed or not.
 * @description
 * This handler is used to handle the data sent form peer device
 */
int APP_OtacRecvDataIndHandler(ke_msg_id_t const msgid,
                               struct otac_recv_data_ind *param,
                               ke_task_id_t const dest_id,
                               ke_task_id_t const src_id);
#endif /* BLE_OTA_CLIENT */
/*! @brief @} APP_OTAC_API */

#endif /* APP_OTAC_H_ */

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

#ifndef _APP_FINDT_H_
#define _APP_FINDT_H_

/*!
 * @addtogroup APP_FINDT_API
 * @{
 */

#if (BLE_FINDME_TARGET)
#include "findt.h"
#include "findt_task.h"

/*******************************************************************************
 * Variables
 ******************************************************************************/
/*! @brief Table of message handler */
extern const struct ke_state_handler g_AppFindtTableHandler;

/*******************************************************************************
 * API
 ******************************************************************************/
/*!
 * @brief Add find me target profile.
 */
void APP_FindtAddProfileTask(void);

/*!
* @brief Receive a alert from peer device.
* @param[in] msgid     FINDT_ALERT_IND
* @param[in] param     findt_alert_ind
* @param[in] dest_id   TASK_APP
* @param[in] src_id    TASK_FINDL_IDX
* @return If the message was consumed or not.
* @description
* This API message is used by the Target role to inform the Application of a valid alert level written by the
* peer in the IAS Alert Level Characteristic.
*
*/
int APP_FindtAlertIndHandler(ke_msg_id_t const msgid,
                             struct findt_alert_ind *param,
                             ke_task_id_t const dest_id,
                             ke_task_id_t const src_id);

#endif
/*! @brief @} APP_FINDT_API */
#endif

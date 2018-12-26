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

#ifndef _APP_PROXR_H_
#define _APP_PROXR_H_

/*!
 * @addtogroup APP_PROXR_API
 * @{
 */

#if (BLE_PROX_REPORTER)
#include "proxr.h"
#include "proxr_task.h"

/*******************************************************************************
 * Variables
 ******************************************************************************/
/*! @brief Table of message handler */
extern const struct ke_state_handler g_AppProxrTableHandler;

/*******************************************************************************
 * API
 ******************************************************************************/
/*!
 * @brief Add proximity profile.
 */
void APP_ProxrAddProfileTask(void);

/*!
 * @brief This API message is used by the Reporter role to request the Application to start the alert on the device
 * considering the indicated alert level.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance .
 * @param[in] src_id    ID of the sending task instance.
 * @return If the message was consumed or not.
 *
 * @description
 * This API message is used by the Reporter role to request the Application to start the alert on the device
 * considering the indicated alert level. The message may be created and sent on two conditions:
 * - The IAS alert level characteristic has been written to a valid value, in which case alert_lvl will be set to the
 * IAS
 * alert level value.
 * - A disconnection has been received, in which case alert_lvl will be set to the LLS alert level value.
 * The connection handle and application task ID stored in the Target environment are used for the creation of the
 * kernel
 * message.
 * The Application actions following reception of this indication are strictly implementation specific (it may try to
 * reconnect to the peer and stop alert upon that, or timeout the alert after a certain time, please see the
 * specification)
 *
 */
int APP_ProxrAlertIndHandler(ke_msg_id_t const msgid,
                             struct proxr_alert_ind *param,
                             ke_task_id_t const dest_id,
                             ke_task_id_t const src_id);

#endif
/*! @brief @} APP_PROXR_API */
#endif /* _APP_PROXR_H_ */

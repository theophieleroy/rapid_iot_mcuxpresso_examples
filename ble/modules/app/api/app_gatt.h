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

#ifndef _APP_GATT_H_
#define _APP_GATT_H_

/*!
 * @addtogroup APP_GATT_API
 * @{
 */

#include "gattc_task.h"
#include "gattm_task.h"

/*******************************************************************************
 * Variables
 ******************************************************************************/
/*! @brief Table of message handlers */
extern const struct ke_state_handler g_AppGattTableHandler;

/*******************************************************************************
 * API
 ******************************************************************************/
/*!
 * @brief Start the MTU exchange procedure.
 *
 * @param[in] conhdl    Connection handle.
 * @response
 * - GATTC_MTU_CHANGED_IND: triggered when MTU has been negotiated
 * - GATTC_CMP_EVT: when command is proceed
 * @description
 *
 *  The MTU sent by the device will be the MTU set during the configuration of the device.
 */
void APP_GattcExcMtuCmd(uint16_t conhdl);

/*!
 * @brief Handles GATTC exchange MTU  indication messae from the GATTC.
 *
 * @param[in] msgid     GATTC_MTU_CHANGED_IND
 * @param[in] param     Pointer to struct gattc_mtu_changed_ind
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_GATTC
 *
 * @return If the message was consumed or not.
 * @description
 *
 * This handler is used to inform the application that service .
 */
static int APP_GattcMtuChangedIndHandler(ke_msg_id_t const msgid,
                                         struct gattc_mtu_changed_ind const *param,
                                         ke_task_id_t const dest_id,
                                         ke_task_id_t const src_id);

/*!
 * @brief Send a Service Change Characteristic Indication .
 * @param[in] conhdl    Connection handle
 * @param[in] start_hdl  Start handle of the affected part of the database
 * @param[in] end_hdl  End handle of the affected part of the database
 * @response
 * @description
 * This message is used by the application to require sending of a Service Change Characteristic Indication.
 * The Service Changed characteristic is used to indicate to connected devices that services in the database
 * have changed (i.e added, removed, modified). It shall be used to indicate to a bonded device that the database
 * content has been modified between the last disconnection and the reconnection.
 */
void APP_GattcSendSvcChangedCmd(uint16_t conhdl, uint16_t start_hdl, uint16_t end_hdl);

/*!
 * @brief Handles GATTC service changed indication messae from the GATTC.
 *
 * @param[in] msgid     GATTC_SVC_CHANGED_CFG_IND
 * @param[in] param     Pointer to struct gattc_svc_changed_cfg
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_GATTC
 *
 * @return If the message was consumed or not.
 * @description
 * This message is sent to the application each time the Client Characteristic Configuration descriptor value is
 * written by the peer device.
 */
static int APP_GattcSvcChangedCfgIndHandler(ke_msg_id_t const msgid,
                                            struct gattc_svc_changed_cfg const *param,
                                            ke_task_id_t const dest_id,
                                            ke_task_id_t const src_id);

/*!
 * @brief Handles GATTC complete event from the GATTC.
 *
 * @param[in] msgid     GATTC_CMP_EVT
 * @param[in] param     Pointer to struct gattc_cmp_evt
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_GATTC
 *
 * @return If the message was consumed or not.
 * @description
 *
 * This handler is used to inform the application that a GATT procedure finish.
 */
static int APP_GattcCmpEvtHandler(ke_msg_id_t const msgid,
                                  struct gattc_cmp_evt const *param,
                                  ke_task_id_t const dest_id,
                                  ke_task_id_t const src_id);

/*! @brief @} APP_GATT_API */

#endif /* _APP_GATT_H_ */

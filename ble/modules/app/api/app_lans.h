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

#ifndef APP_LANS_H_
#define APP_LANS_H_

/*!
 * @addtogroup APP_LANS_API
 * @{
 */

#if (BLE_LN_SENSOR)

#include "lans.h"
#include "lans_task.h"
#include "prf_utils.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

struct app_lans_env_tag
{
    uint16_t noti_intv; /*!< Notification interval value */
};

/*******************************************************************************
 * Variables
 ******************************************************************************/
/*! @brief Table of message handler */
extern const struct ke_state_handler g_AppLansTableHandler;
/*! @brief Application lans environment */
extern struct app_lans_env_tag g_AppLansEnv;

/*******************************************************************************
 * API
 ******************************************************************************/
/*!
 * @brief Add Location and Navigation Profile Sensor.
 */
void APP_LansAddProfileTask(void);

/*!
 * @brief Start the profile at connection.
 * @param[in] conhdl               Connection handle
 * @param[in] prfl_ntf_ind_cfg     Characteristic configuration descriptor bit field value for a bonded device
 *       Bit 0: Location and speed characteristic client configuration
 *       Bit 1: LN Control point characteristic indication configuration
 *       Bit 2: Navigation characteristic client configuration
 * @response  LANS_ENABLE_RSP
 * @description
 * This message shall be used after the connection with a peer in order to restore the LNP Sensor bond data
 * Application shall provide the Connection index in order to activate the profile.
 */
void APP_LansEnableReq(uint16_t conhdl, uint16_t prfl_ntf_ind_cfg);

/*!
* @brief Response of LANS_ENABLE_REQ.
* @param[in] msgid     LANS_ENABLE_RSP
* @param[in] param     lans_enable_rsp
* @param[in] dest_id   TASK_APP
* @param[in] src_id    TASK_LANS
* @return If the message was consumed or not.
* @description
* This message informs the application about the status of the operation.
*/
int APP_LansEnableRspHandler(ke_msg_id_t const msgid,
                             struct lans_enable_rsp *param,
                             ke_task_id_t const dest_id,
                             ke_task_id_t const src_id);

/*!
 * @brief Send location and speed notification .
 * @param[in] conhdl               Connection handle
 * @param[in] parameters           Structure containing location and speed notification fields
 * @response  LANS_NTF_LOC_SPEED_RSP
 * @description
 * This message shall be used by the application to send a Location and Speed Measurement notification to
 * every connected device.
 */
void APP_LansNtfLocSpeedReq(uint16_t conhdl, struct lanp_loc_speed *parameters);

/*!
* @brief Response of LANS_NTF_LOC_SPEED_REQ.
* @param[in] msgid     LANS_NTF_LOC_SPEED_RSP
* @param[in] param     lans_ntf_loc_speed_rsp
* @param[in] dest_id   TASK_APP
* @param[in] src_id    TASK_LANS
* @return If the message was consumed or not.
* @description
* This message informs the application about the status of the operation.
*/

int APP_LansNtfLocSpeedRspHandler(ke_msg_id_t const msgid,
                                  struct lans_ntf_loc_speed_rsp *param,
                                  ke_task_id_t const dest_id,
                                  ke_task_id_t const src_id);

/*!
 * @brief Send a Navigation notification.
 * @param[in] conhdl               Connection handle
 * @param[in] parameters           Structure containing navigation notification fields
 * @response  LANS_NTF_NAVIGATION_RSP
 * @description
 * This API message shall be used by the application to send a Navigation notification to every
 * connected device.
 */
void APP_LansNtfNavigationReq(uint16_t conhdl, struct lanp_navigation *parameters);

/*!
* @brief Response of LANS_NTF_NAVIGATION_REQ.
* @param[in] msgid     LANS_NTF_NAVIGATION_RSP
* @param[in] param     lans_ntf_navigation_rsp
* @param[in] dest_id   TASK_APP
* @param[in] src_id    TASK_LANS
* @return If the message was consumed or not.
* @description
* This message informs the application about the status of the operation.
*/
int APP_LansNtfNavigationRspHandler(ke_msg_id_t const msgid,
                                    struct lans_ntf_navigation_rsp *param,
                                    ke_task_id_t const dest_id,
                                    ke_task_id_t const src_id);

/*!
 * @brief Send position quality values.
 * @param[in] conhdl               Connection handle
 * @param[in] parameters           Structure containing position quality fields
 * @response  LANS_UPD_POS_Q_RSP
 * @description
 * This API message shall be used by the application to modify the position quality values.
 */
void APP_LansUpdPosQReq(uint16_t conhdl, struct lanp_posq *parameters);

/*!
* @brief Response of LANS_UPD_POS_Q_REQ.
* @param[in] msgid     LANS_UPD_POS_Q_RSP
* @param[in] param     lans_upd_pos_q_rsp
* @param[in] dest_id   TASK_APP
* @param[in] src_id    TASK_LANS
* @return If the message was consumed or not.
* @description
* This message informs the application about the status of the operation.
*/
int APP_LansUpdPosQRspHandler(ke_msg_id_t const msgid,
                              struct lans_upd_pos_q_rsp *param,
                              ke_task_id_t const dest_id,
                              ke_task_id_t const src_id);

/*!
* @brief Receive CP request from peer device.
* @param[in] msgid     LANS_LN_CTNL_PT_REQ_IND
* @param[in] param     lans_ln_ctnl_pt_req_ind
* @param[in] dest_id   TASK_APP
* @param[in] src_id    TASK_LANS
* @return If the message was consumed or not.
* @description
* The message is sent to the application when the LN Control Point characteristic is written
* by the peer device.
*/
int APP_LansLnCtnlPtReqIndHandler(ke_msg_id_t const msgid,
                                  struct lans_ln_ctnl_pt_req_ind *param,
                                  ke_task_id_t const dest_id,
                                  ke_task_id_t const src_id);

/*!
 * @brief  Confirm LANS_LN_CTNL_PT_REQ_IND message.
 * @param[in] conhdl               Connection handle
 * @param[in] parameters           Structure control confirmation message
 * @description
 * This message is sent by the application as a response to the LANS_LN_CTNL_PT_REQ_IND.
 * It contains the value requested by the profile.
 */
void APP_LansLnCtnlPtCfm(struct lans_ln_ctnl_pt_cfm *parameters);

/*!
* @brief Receive configuration from peer device.
* @param[in] msgid     LANS_CFG_NTFIND_IND
* @param[in] param     lans_cfg_ntfind_ind
* @param[in] dest_id   TASK_APP
* @param[in] src_id    TASK_LANS
* @return If the message was consumed or not.
* @description
* This message is sent to the application each time a peer device successfully writes the Client
* Characteristic Configuration descriptor of the Location and speed, Navigation or the LN Control
* Point characteristics.
*/
int APP_LansCfgNtfindIndHandler(ke_msg_id_t const msgid,
                                struct lans_cfg_ntfind_ind *param,
                                ke_task_id_t const dest_id,
                                ke_task_id_t const src_id);

/*!
* @brief Command completed.
* @param[in] msgid     LANS_CMP_EVT
* @param[in] param     lans_cmp_evt
* @param[in] dest_id   TASK_APP
* @param[in] src_id    TASK_LANS
* @return If the message was consumed or not.
* @description
* The message is used by the LANS task to inform the sender of a command that the procedure is
* over and contains the status of the procedure.
*/
int APP_LansCmpEvtHandler(ke_msg_id_t const msgid,
                          struct lans_cmp_evt *param,
                          ke_task_id_t const dest_id,
                          ke_task_id_t const src_id);

#endif
/*! @brief @} APP_LANS_API */
#endif

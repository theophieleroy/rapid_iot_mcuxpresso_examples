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

#ifndef APP_TIPS_H_
#define APP_TIPS_H_

/*!
 * @addtogroup APP_TIPS_API
 * @{
 */

#if (BLE_TIP_SERVER)
#include "tips.h"
#include "tips_task.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define TIMER_UPDATE_INTV_VALUE                      (2) 
#define TIPS_CTS_SEC_MINU_MAX                        (59)
#define TIPS_CTS_HOUR_MAX                            (23)



struct app_tips_env_tag
{
    uint16_t con_handle;               /*!< Connection hadle */
    uint16_t current_time_intv;        /*!< Time Interval value */
	uint8_t  time_update_state;        /*!< Time Update state*/
    
};

/*******************************************************************************
 * Variables
 ******************************************************************************/
/*! @brief Table of message handler */
extern const struct ke_state_handler g_AppTipsTableHandler;

/*! @brief Application htpt environment */
extern struct app_tips_env_tag g_AppTipsEnv;

/*******************************************************************************
 * API
 ******************************************************************************/
/*!
 * @brief Add Time Profile Server.
 */
void APP_TipsAddProfileTask(void);

/*!
 * @brief Start the profile at connection.
 * @param[in] conhdl                 Connection handle
 * @param[in] current_time_ntf_en    Value stored for Current Time Notification Client Configuration Char
 * @response  TIPS_ENABLE_RSP
 * @description
 *This API message is used for enabling the Time Server role of the Time profile. Before sending this message,
 * a BLE connection shall exist with peer device. 
 */
void APP_TipsEnableReq(uint16_t conhdl, uint16_t current_time_ntf_en);

/*!
* @brief Response of TIPS_ENABLE_REQ.
* @param[in] msgid     TIPS_ENABLE_RSP
* @param[in] param     tips_enable_rsp
* @param[in] dest_id   TASK_APP
* @param[in] src_id    TASK_TIPS
* @return If the message was consumed or not.
* @description
* This API message is used by the Time Server role to inform the application of the enable status operation.
*/
int APP_TipsEnableRspHandler(ke_msg_id_t const msgid,
                             struct tips_enable_rsp *param,
                             ke_task_id_t const dest_id,
                             ke_task_id_t const src_id);



/*!
 * @brief Update current time value.
 * @param[in] conhdl            Connection handle
 * @param[in] current_time      Current Time characteristic value
 * @param[in] enable_ntf_send   Define if a notification of new current time value will be send.
 * @response  TIPS_UPD_CURR_TIME_RSP
 * @description
 * This API message is used by the application for requesting an update of the Current Time characteristic value.
 */
void APP_TipsUpdCurrTimeReq(uint16_t conhdl, struct tip_curr_time *current_time,uint8_t enable_ntf_send);


/*!
* @brief Response of TIPS_UPD_CURR_TIME_REQ.
* @param[in] msgid     TIPS_ UPD_CURR_TIME_RSP
* @param[in] param     tips_upd_curr_time_rsp
* @param[in] dest_id   TASK_APP
* @param[in] src_id    TASK_TIPS
* @return If the message was consumed or not.
* @description
* This API message is used by the Time Server role to inform the application of the enable status operation.
*/
int APP_TipsUpdCurrTimeRspHandler(ke_msg_id_t const msgid,
                                  struct tips_upd_curr_time_rsp *param,
                                  ke_task_id_t const dest_id,
                                  ke_task_id_t const src_id);



/*!
* @brief Peer device send read request.
* @param[in] msgid     TIPS_RD_REQ_IND
* @param[in] param     tips_rd_req_ind
* @param[in] dest_id   TASK_APP
* @param[in] src_id    TASK_TIPS
* @return If the message was consumed or not.
* @description
* This API message informs the application that a peer device wants to read one, the application must
* answer to this request using the TIPS_RD_REQ_CFM message followed by the correct parameters.
*/
int APP_TipsRdReqIndHandler(ke_msg_id_t const msgid,
                            struct tips_rd_req_ind *param,
                            ke_task_id_t const dest_id,
                            ke_task_id_t const src_id);



/*!
 * @brief Confirm  TIPS_RD_REQ_IND.
 * @param[in] conhdl         Connection handle
 * @param[in] cfm            Comfirmed value 
 * @description
 * This API message is used to send the requested data after the read command was correctly received by
 * the device.
 */
void APP_TipsRdCfm(uint16_t conhdl, struct tips_rd_cfm *cfm);



/*!
* @brief Peer device modify client configuration.
* @param[in] msgid     TIPS_CURRENT_TIME_CCC_IND
* @param[in] param     tips_current_time_ccc_ind
* @param[in] dest_id   TASK_APP
* @param[in] src_id    TASK_TIPS
* @return If the message was consumed or not.
* @description
* This API message is used to inform the application about a modification of the Current Time Client Configuration
* characteristic value.
*/
int APP_TipsCurrentTimeCccIndHandler(ke_msg_id_t const msgid,
                                     struct tips_current_time_ccc_ind *param,
                                     ke_task_id_t const dest_id,
                                     ke_task_id_t const src_id);

/*!
* @brief Receive modification of control point.
* @param[in] msgid     TIPS_TIME_UPD_CTNL_PT_IND
* @param[in] param     tips_time_upd_ctnl_pt_ind
* @param[in] dest_id   TASK_APP
* @param[in] src_id    TASK_TIPS
* @return If the message was consumed or not.
* @description
* This API message is used to inform the application about a modification of the Time Update Control Point
* Characteristic value.
*/
int APP_TipsTimeUpdCtnlPtIndHandler(ke_msg_id_t const msgid,
                                    struct tips_time_upd_ctnl_pt_ind *param,
                                    ke_task_id_t const dest_id,
                                    ke_task_id_t const src_id);


#endif
/*! @brief @} APP_TIPS_API */
#endif

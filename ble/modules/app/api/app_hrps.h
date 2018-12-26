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

#ifndef APP_HRPS_H_
#define APP_HRPS_H_

/*!
 * @addtogroup APP_HRPS_API
 * @{
 */

#if (BLE_HR_SENSOR)

#include "hrps.h"
#include "hrps_task.h"
#include "prf_utils.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define HRPS_INIT_ENERGY_EXP_VAL (0)
#define HRPS_INIT_MEAS_INTVL (1)
#define HRPS_ENERGY_EXP_VAL (10)
#define HRPS_HEART_RATE_INCREMENT (5)
#define HRPS_HEART_RATE_MAX (250)
#define HRPS_HEART_RATE_MIN (50)

struct app_hrps_env_tag
{
    uint16_t meas_intv;      /*!< Heart Rate Profile Sensor measurements interval */
    uint16_t energy_exp_val; /*!< Heart Rate Profile Sensor energy expended value */
};

/*******************************************************************************
 * Variables
 ******************************************************************************/
/*! @brief Table of message handler */
extern const struct ke_state_handler g_AppHrpsTableHandler;
/*! @brief Application hrps environment */
extern struct app_hrps_env_tag g_AppHrpsEnv;

/*******************************************************************************
 * API
 ******************************************************************************/
/*!
 * @brief Add Heart Rate Profile Sensor.
 */
void APP_HrpsAddProfileTask(void);

/*!
 * @brief Start the profile at connection.
 * @param[in] conhdl            Connection handle
 * @param[in] hr_meas_ntf       Heart Rate Notification configuration
 * @response  HRPS_ENABLE_RSP
 * @description
 * This API message is used for restoring the Heart Rate Sensor bond data for a given connection handle.
 * Before sending this message, a BLE connection shall exist with peer device.
 */
void APP_HrpsEnableReq(uint16_t conhdl, uint16_t hr_meas_ntf);

/*!
* @brief Response of HRPS_ENABLE_REQ.
* @param[in] msgid     HRPS_ENABLE_RSP
* @param[in] param     hrps_enable_rsp
* @param[in] dest_id   TASK_APP
* @param[in] src_id    TASK_HRPS
* @return If the message was consumed or not.
* @description
* This API message is used by the Heart Rate sensor role to inform the Application of a correct enable.
*
*/

int APP_HrpsEnableRspHandler(ke_msg_id_t const msgid,
                             struct hrps_enable_rsp *param,
                             ke_task_id_t const dest_id,
                             ke_task_id_t const src_id);

/*!
 * @brief Send measurement to each device.
 * @param[in] conhdl            Connection handle
 * @param[in] meas_val          Heart Rate Measurement Structure
 * @response  HRPS_MEAS_SEND_RSP
 * @description
 * This message is used by the application (which handles the Heart Rate device driver and measurements)
 * to send a Heart Rate measurement through the Heart Rate sensor role to every connected device.
 */
void APP_HrpsMeasSendReq(uint16_t conhdl, struct hrs_hr_meas *meas_val);

/*!
* @brief Response of HRPS_MEAS_SEND_REQ.
* @param[in] msgid     HRPS_MEAS_SEND_RSP
* @param[in] param     hrps_meas_send_rsp
* @param[in] dest_id   TASK_APP
* @param[in] src_id    TASK_HRPS
* @return If the message was consumed or not.
* @description
* This message is used by HRPS to send to the application, a confirmation, or error status of a notification.
*/

int APP_HrpsMeasSendRspHandler(ke_msg_id_t const msgid,
                               struct hrps_meas_send_rsp *param,
                               ke_task_id_t const dest_id,
                               ke_task_id_t const src_id);

/*!
* @brief Receive configuration from peer device.
* @param[in] msgid     HRPS_CFG_INDNTF_IND
* @param[in] param     hrps_cfg_indntf_ind
* @param[in] dest_id   TASK_APP
* @param[in] src_id    TASK_HRPS
* @return If the message was consumed or not.
* @description
* This message is used by HRPS to inform application that peer device has changed notification
* configuration.
*/

int APP_HrpsCfgIndntfIndHandler(ke_msg_id_t const msgid,
                                struct hrps_cfg_indntf_ind *param,
                                ke_task_id_t const dest_id,
                                ke_task_id_t const src_id);

/*!
* @brief Receive information energy expanded reset .
* @param[in] msgid     HRPS_ENERGY_EXP_RESET_IND
* @param[in] param     hrps_energy_exp_reset_ind
* @param[in] dest_id   TASK_APP
* @param[in] src_id    TASK_HRPS
* @return If the message was consumed or not.
* @description
* This message is used by HRPS to inform application that Energy Expanded value shall be reset.
*/
int APP_HrpsEnergyExpResetIndHandler(ke_msg_id_t const msgid,
                                     struct hrps_energy_exp_reset_ind *param,
                                     ke_task_id_t const dest_id,
                                     ke_task_id_t const src_id);

#endif
/*! @brief @} APP_HRPS_API */
#endif

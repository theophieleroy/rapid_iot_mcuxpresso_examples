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

#ifndef APP_HTPT_H_
#define APP_HTPT_H_

/*!
 * @addtogroup APP_HTPT_API
 * @{
 */

#if (BLE_HT_THERMOM)
#include "htpt.h"
#include "htpt_task.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define MEAS_INTL_DEFAULT_VALUE (3)
#define MEAS_INTL_RANGE_MAX (10)
#define MEAS_INTL_RANGE_MIN (1)
#define MAX_INTERM_TEMP_TIMES (3)
#define MIN_TEMP_VALUE (98)
#define MAX_TEMP_VALUE (104)

struct app_htpt_env_tag
{
    uint16_t meas_intv;      /*!< Measurement Interval value */
    uint16_t temp_meas_intv; /*!< Temporary Measurement Interval value */
};

/*******************************************************************************
 * Variables
 ******************************************************************************/
/*! @brief Table of message handler */
extern const struct ke_state_handler g_AppHtptTableHandler;

/*! @brief Application htpt environment */
extern struct app_htpt_env_tag g_AppHtptEnv;

/*******************************************************************************
 * API
 ******************************************************************************/
/*!
 * @brief Add Health Thermometer profile Thermometer.
 */
void APP_HtptAddProfileTask(void);

/*!
 * @brief Start the profile at connection.
 * @param[in] conhdl        Connection handle
 * @param[in] ntf_ind_cfg   Notification configuration
 *              HTPT_CFG_STABLE_MEAS_IND
 *              HTPT_CFG_INTERM_MEAS_NTF
 *              HTPT_CFG_MEAS_INTV_IND
 * @response  HTPT_ENABLE_RSP
 * @description
 * This API message can be used after the connection with a peer device has been established
 * in order to restore known device bond data.
 */
void APP_HtptEnableReq(uint16_t conhdl, uint8_t ntf_ind_cfg);

/*!
* @brief Response of HTPT_ENABLE_REQ.
* @param[in] msgid     HTPT_ENABLE_RSP
* @param[in] param     htpt_enable_rsp
* @param[in] dest_id   TASK_APP
* @param[in] src_id    TASK_HTPT
* @return If the message was consumed or not.
* @description
* Inform application if restoring bond data for peer device succeed or not.
*
 */
int APP_HtptEnableRspHandler(ke_msg_id_t const msgid,
                             struct htpt_enable_rsp *param,
                             ke_task_id_t const dest_id,
                             ke_task_id_t const src_id);

/*!
 * @brief Indicate/notify a temperature measurement.
 * @param[in] conhdl            Connection handle
 * @param[in] temp_meas         Temperature Measurement value
 * @param[in] stable_meas       Indicate if the temperature measurement is stable or not
 *     0: will be sent using the Temperature Measurement characteristic
 *     1: will be sent using the Intermediate Temperature characteristic
 * @response  HTPT_TEMP_SEND_RSP
 * @description
 * This message is used by the application (which handles the temperature device driver and
 * measurements) to send a temperature measurement through the Thermometer role.
 */
void APP_HtptTempSendReq(uint16_t conhdl, struct htp_temp_meas *temp_meas, bool stable_meas);

/*!
* @brief Response of HTPT_TEMP_SEND_REQ.
* @param[in] msgid     HTPT_TEMP_SEND_RSP
* @param[in] param     htpt_temp_send_rsp
* @param[in] dest_id   TASK_APP
* @param[in] src_id    TASK_HTPT
* @return If the message was consumed or not.
* @description
* This message is used by HTPT to inform that sent temperature measurement indication or notification is
* finished request.
*
 */
int APP_HtptTempSendRspHandler(ke_msg_id_t const msgid,
                               struct htpt_temp_send_rsp *param,
                               ke_task_id_t const dest_id,
                               ke_task_id_t const src_id);

/*!
 * @brief Indicate a Measurement Interval.
 * @param[in] conhdl            Connection handle
 * @param[in] meas_intv         Measurement Interval value
 * @response  HTPT_MEAS_INTV_UPD_RSP
 * @description
 * This message is used by the application to order the HTPT profile to generate an indication (if enabled)
 * of the Measurement Interval Char on all connected peer device if indication configuration has been set.
 */
void APP_HtptMeasIntvUpdReq(uint16_t conhdl, uint16_t meas_intv);

/*!
* @brief Response of HTPT_MEAS_INTV_UPD_REQ.
* @param[in] msgid     HTPT_MEAS_INTV_UPD_RSP
* @param[in] param     htpt_meas_intv_upd_rsp
* @param[in] dest_id   TASK_APP
* @param[in] src_id    TASK_HTPT
* @return If the message was consumed or not.
* @description
* This message is used by HTPT to inform that measurement interval has been updated and peer devices
* have been informed.
*
 */
int APP_HtptMeasIntvUpdRspHandler(ke_msg_id_t const msgid,
                                  struct htpt_meas_intv_upd_rsp *param,
                                  ke_task_id_t const dest_id,
                                  ke_task_id_t const src_id);

/*!
* @brief Indicate application to update the measurement interval value.
* @param[in] msgid     HTPT_MEAS_INTV_CHG_REQ_IND
* @param[in] param     htpt_meas_intv_chg_req_ind
* @param[in] dest_id   TASK_APP
* @param[in] src_id    TASK_HTPT
* @return If the message was consumed or not.
* @description
* This message is used by the HTPT to inform the application that a peer device requests to update the
* measurement interval value.
*
 */
int APP_HtptMeasIntvChgReqIndHandler(ke_msg_id_t const msgid,
                                     struct htpt_meas_intv_chg_req_ind *param,
                                     ke_task_id_t const dest_id,
                                     ke_task_id_t const src_id);

/*!
* @brief Receove updated notification and indication configuration.
* @param[in] msgid     HTPT_CFG_INDNTF_IND
* @param[in] param     HTPT_CFG_INDNTF_IND
* @param[in] dest_id   TASK_APP
* @param[in] src_id    TASK_HTPT
* @return If the message was consumed or not.
* @description
* This message is used to inform the Application that a peer device updated notification and indication
* configuration.
*
 */
int APP_HtptCfgIndntfIndHandler(ke_msg_id_t const msgid,
                                struct htpt_cfg_indntf_ind *param,
                                ke_task_id_t const dest_id,
                                ke_task_id_t const src_id);

#endif
/*! @brief @} APP_HTPT_API */
#endif

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
#ifndef _APP_GLPC_H_
#define _APP_GLPC_H_

/*!
 * @addtogroup APP_GLPC_API
 * @{
 */

#if (BLE_GL_COLLECTOR)
#include "glpc.h"
#include "glpc_task.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
struct app_glpc_read_req
{
    uint8_t dummy; /*!< just construct read glucose sensor features message */
};

/*******************************************************************************
 * Variables
 ******************************************************************************/
/*! @brief Table of message handlers */
extern const struct ke_state_handler g_AppGlpcTableHandler;

/*******************************************************************************
 * API
 ******************************************************************************/
/*!
 * @brief Add Glucose Profile Collector .
 */
void APP_GlpcAddProfileTask(void);

/*!
 * @brief Start the profile at connection.
 * @param[in] conhdl        Connection handle
 * @response  GLPC_ENABLE_RSP
 * @description
 *  This API message is used for enabling the Collector role of the Glucose profile.
 */
void APP_GlpcEnableReq(uint16_t conhdl);

/*!
 * @brief Handles the enable confirmation from the GLPS.
 * @param[in] msgid     GLPC_ENABLE_RSP
 * @param[in] param     glpc_enable_rsp
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_GLPC
 * @return If the message was consumed or not.
 * @description
 * This API message is used by the Collector to either send the discovery results of GLS on the Glucose sensor
 * or confirm enabling of the Collector role, or to simply confirm enabling of Collector role if it is a normal
 * connection and the attribute details are already known.
 */
int APP_GlpcEnableRspHandler(ke_msg_id_t const msgid,
                             struct glpc_enable_rsp *param,
                             ke_task_id_t const dest_id,
                             ke_task_id_t const src_id);

/*!
 * @brief Register Glucose measurement  notifications.
 * @param[in] conhdl        Connection handle
 * @param[in] ctx_en        Enable or discable ctx notifications
 * @response  GLPC_REGISTER_RSP
 * @description
 * This API message is used by the application to register to Glucose sensor notifications and indications.
 */
void APP_GlpcRegisterReq(uint16_t conhdl, bool ctx_en);

/*!
 * @brief Handles response of GLPC_REGISTER_REQ .
 * @param[in] msgid     GLPC_REGISTER_RSP
 * @param[in] param     glpc_register_rsp
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_GLPC
 * @return If the message was consumed or not.
 * @description
 * This API message is used by the Collector role to inform the Application about Glucose sensor event
 * registration status.
 */

int APP_GlpcRegisterRspHandler(ke_msg_id_t const msgid,
                               struct glpc_register_rsp *param,
                               ke_task_id_t const dest_id,
                               ke_task_id_t const src_id);

/*!
 * @brief  Read peer GLPS features.
 * @param[in] conhdl        Connection handle
 * @response  GLPC_READ_FEATURES_RSP
 * @description
 * This API message is used by the application read peer Glucose sensor features.
 */
void APP_GlpcReadFeaturesReq(uint16_t conhdl);

/*!
 * @brief Handles response of feature .
 * @param[in] msgid     GLPC_READ_FEATURES_RSP
 * @param[in] param     glpc_read_features_rsp
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_GLPC
 * @return If the message was consumed or not.
 * @description
 * This API message is used by the Collector role to inform the Application of received peer Glucose sensor
 * features.
 */
int APP_GlpcReadFeatureRspHandler(ke_msg_id_t const msgid,
                                  struct glpc_read_features_rsp *param,
                                  ke_task_id_t const dest_id,
                                  ke_task_id_t const src_id);

/*!
 * @brief Request to report stored record.
 * @param[in] conhdl        Connection handle
 * @description
 * This action could be report glucose measurements.
 */
void APP_GlpcReportStoredRecord(uint16_t conhdl);

/*!
 * @brief Request to delete stored record.
 * @param[in] conhdl        Connection handle
 * @description
 * This action could be delete measurements.
 */
void APP_GlpcDelStoredRecord(uint16_t conhdl);

/*!
 * @brief Request to report number of stored record.
 * @param[in] conhdl        Connection handle
 * @description
 * This action could be report the number of glucose measurements.
 */
void APP_GlpcReportNumStoredRecord(uint16_t conhdl);

/*!
 * @brief Request to abort an operation.
 * @param[in] conhdl        Connection handle
 * @description
 * This action could be abort an on-going operation.
 */
void APP_GlpcAbortOperation(uint16_t conhdl);

/*!
 * @brief Handles response of GLPC_RACP_REQ .
 * @param[in] msgid     GLPC_RACP_RSP
 * @param[in] param     glpc_racp_rsp
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_GLPC
 * @return If the message was consumed or not.
 * @description
 * This API message is used by the Collector role to inform the Application of a status of Record Access
 * Control Point Action.
 */
int APP_GlpcRacpRspHandler(ke_msg_id_t const msgid,
                           struct glpc_racp_rsp *param,
                           ke_task_id_t const dest_id,
                           ke_task_id_t const src_id);

/*!
 * @brief Handles notificatios of measurement from peer device .
 * @param[in] msgid     GLPC_MEAS_IND
 * @param[in] param     glpc_meas_ind
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_GLPC
 * @return If the message was consumed or not.
 * @description
 * This API message is used by the Collector role to inform the Application of a received Glucose
 * measurement value.
 */

int APP_GlpcMeasIndHandler(ke_msg_id_t const msgid,
                           struct glpc_meas_ind *param,
                           ke_task_id_t const dest_id,
                           ke_task_id_t const src_id);

/*!
 * @brief Handles notificatios of measurement context from peer device.
 * @param[in] msgid     GLPC_MEAS_CTX_IND
 * @param[in] param     glpc_meas_ctx_ind
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_GLPC
 * @return If the message was consumed or not.
 * @description
 * This API message is used by the Collector role to inform the Application of a received Glucose
 * measurement context value.
 */
int APP_GlpcMeasCtxIndHandler(ke_msg_id_t const msgid,
                              struct glpc_meas_ctx_ind *param,
                              ke_task_id_t const dest_id,
                              ke_task_id_t const src_id);

/*!
 * @brief Handles racp timeout information from peer device.
 * @param[in] msgid     GLPC_RACP_REQ_TIMEOUT
 * @param[in] param     Timeout information
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_GLPC
 * @return If the message was consumed or not.
 * @description
 * This API message is used by the Collector role to inform the Application of a received timeout
 * information.
 */
int APP_GlpcRacpReq_timeout_handler(ke_msg_id_t const msgid,
                                    void *param,
                                    ke_task_id_t const dest_id,
                                    ke_task_id_t const src_id);

#endif

#endif /* _APP_GLPC_H_ */

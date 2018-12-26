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

#ifndef APP_CPPS_H_
#define APP_CPPS_H_

/*!
 * @addtogroup APP_CPPS_API
 * @{
 */

#if (BLE_CP_SENSOR)
#include "cpps.h"
#include "cpps_task.h"
#include "prf_utils.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define CPPS_INIT_WHEEL_REVOL (0)
#define CPPS_INIT_MEAS_INTVL (2)
#define CPPS_TORQUE_INCREMENT (320)
#define CPPS_WHEEL_EVENT_TIME_INCREMENT (2048)
#define CPPS_CRANK_EVENT_TIME_INCREMENT (1024)
#define CPPS_PEDAL_POWER_BALANCE (20)
#define CPPS_INSTANTANEOUS_POWER (200)

enum app_broadcast_state
{

    APP_BROADCAST_IDLE = 0x00, /*!< idle state */
    APP_BROADCAST_READY,       /*!< ready to get broadcast data */
    APP_BROADCAST_START,       /*!< start broadcating */
};

struct app_cpps_env_tag
{
    uint16_t meas_intv;      /*!< Cycling Power measurement interval vlaue */
    uint8_t braodcast_state; /*!< Cycling Power measurement broadcast state */
};

/*******************************************************************************
 * Variables
 ******************************************************************************/
/*! @brief Table of message handler */
extern const struct ke_state_handler g_AppCppsTableHandler;
/*! @brief Application cpps environment */
extern struct app_cpps_env_tag g_AppCppsEnv;

/*******************************************************************************
 * API
 ******************************************************************************/
/*!
 * @brief Add Cycling Power Profile Sensor.
 */
void APP_CppsAddProfileTask(void);

/*!
 * @brief Start the profile at connection.
 * @param[in] conhdl                    Connection Handle
 * @param[in] prfl_ntf_ind_cfg          Characteristic Configuration Descriptor bit field value for a bonded device:
 *        Bit 0: Measurement Characteristic client configuration
 *        Bit 1: Measurement Characteristic server configuration
 *        Bit 2: Vector Characteristic notification configuration
 *        Bit 3: Control Point Characteristic indication configuration
 * @response  CCPS_ENABLE_RSP
 * @description
 * This message shall be used after the connection with a peer in order to restore the CPP Sensor bond data.
 */
void APP_CppsEnableReq(uint16_t conhdl, uint16_t prfl_ntf_ind_cfg);

/*!
 * @brief Response of CPPS_ENABLE_REQ.
 *
 * @param[in] msgid     CPPS_ENABLE_RSP
 * @param[in] param     cpps_enable_rsp
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_CPPS
 * @return If the message was consumed or not.
 *
 * @description
 * This message corresponds to the response of setting bond data operation.
 */
int APP_CppsEnableRspHandler(ke_msg_id_t const msgid,
                             struct cpps_enable_rsp *param,
                             ke_task_id_t const dest_id,
                             ke_task_id_t const src_id);

/*!
 * @brief  Request advertising data.
 * @param[in] conhdl            Connection handle
 * @param[in] parameters        Structure containing measurement notification fields
 * @response  CCPS_ENABLE_RSP
 * @description
 * Packs provided data in a single array including the advertising header required for the Cycling Power
 * Service.
 */
void APP_CppsGetAdvDataReq(uint16_t conhdl, struct cpp_cp_meas *parameters);

/*!
* @brief Response of CPPS_GET_ADV_DATA_REQ.
* @param[in] msgid     CPPS_GET_ADV_DATA_RSP
* @param[in] param     cpps_get_adv_data_rsp
* @param[in] dest_id   TASK_APP
* @param[in] src_id    TASK_CPPS
* @return If the message was consumed or not.
* @description
* Returns packed data to be broadcasted by the application.
*
 */
int APP_CppsGetAdvDataRspHandler(ke_msg_id_t const msgid,
                                 struct cpps_get_adv_data_rsp *param,
                                 ke_task_id_t const dest_id,
                                 ke_task_id_t const src_id);

/*!
 * @brief Send measurement notification to peer device.
 * @param[in] conhdl            Connection handle
 * @param[in] parameters        Structure containing measurement notification fields
 * @response  CPPS_NTF_CP_MEAS_RSP
 * @description
 * This message shall be used by the application to send a CP Measurement notification to every connected
 * device.
 */
void APP_CppsNtfCpMeasReq(uint16_t conhdl, struct cpp_cp_meas *parameters);

/*!
* @brief Response of CPPS_NTF_CP_MEAS_REQ.
* @param[in] msgid     CPPS_NTF_CP_MEAS_RSP
* @param[in] param     cpps_get_adv_data_rsp
* @param[in] dest_id   TASK_APP
* @param[in] src_id    TASK_CPPS
* @description
* Message to inform the application about the status of the measurement notification.
*
*/
int APP_CppsNtfCpMeasRspHandler(ke_msg_id_t const msgid,
                                struct cpps_ntf_cp_meas_rsp *param,
                                ke_task_id_t const dest_id,
                                ke_task_id_t const src_id);

/*!
 * @brief Send vector notification to peer device.
 * @param[in] conhdl            Connection handle
 * @param[in] parameters        Structure containing vector notification fields
 * @response  CPPS_NTF_CP_VECTOR_RSP
 * @description
 * This message shall be used by the application to send a CP Vector notification to every connected device.
 */
void APP_CppsNtfCpVectorReq(uint16_t conhdl, struct cpp_cp_vector *parameters);

/*!
* @brief Response of CPPS_NTF_CP_VECTOR_REQ.
*
* @param[in] msgid     CPPS_NTF_CP_VECTOR_RSP
* @param[in] param     cpps_ntf_cp_vector_rsp
* @param[in] dest_id   TASK_APP
* @param[in] src_id    TASK_CPPS
* @return If the message was consumed or not.
*
* @description
* Message corresponding to the response of the vector notification.
*
*/
int APP_CppsNtfCpVectorRspHandler(ke_msg_id_t const msgid,
                                  struct cpps_ntf_cp_vector_rsp *param,
                                  ke_task_id_t const dest_id,
                                  ke_task_id_t const src_id);

/*!
* @brief Receive written request of control point characteristic .
* @param[in] msgid     CPPS_CTNL_PT_REQ_IND
* @param[in] param     cpps_ctnl_pt_req_ind
* @param[in] dest_id   TASK_APP
* @param[in] src_id    TASK_CPPS
* @return If the message was consumed or not.
* @description
* The message is sent to the application when the CP Control Point characteristic is written by the peer
* device.
*
*/
int APP_CppsCtnlPtReqIndHandler(ke_msg_id_t const msgid,
                                struct cpps_ctnl_pt_req_ind *param,
                                ke_task_id_t const dest_id,
                                ke_task_id_t const src_id);

/*!
 * @brief Send confirmation of CPPS_CTNL_PT_REQ_IND to peer device.
 * @param[in] conhdl        Connection handle
 * @param[in] cfm           Parameters of the confirmation
 * @description
 * This message is sent by the application in response to the CPPS_CTNL_PT_REQ_IND message. It contains
 * the value requested by the profile.
 */
void APP_CppsCtnlPtCfm(struct cpps_ctnl_pt_cfm *cfm);

/*!
* @brief Receive configuration requirement of measurement or control point.
* @param[in] msgid     CPPS_CFG_NTFIND_IND
* @param[in] param     cpps_cfg_ntfind_ind
* @param[in] dest_id   TASK_APP
* @param[in] src_id    TASK_CPPS
* @return If the message was consumed or not.
* @description
* This message is sent to the application each time a peer device has successfully written to the Client
* Characteristic Configuration descriptor of the CP Measurement (client and server) or the CP Control Point
* characteristics.
*
 */
int APP_CppsCfgNtfindIndHandler(ke_msg_id_t const msgid,
                                struct cpps_cfg_ntfind_ind *param,
                                ke_task_id_t const dest_id,
                                ke_task_id_t const src_id);

/*!
* @brief receive the configuration requirement of vector.
* @param[in] msgid     CPPS_VECTOR_CFG_REQ_IND
* @param[in] param     cpps_vector_cfg_req_ind
* @param[in] dest_id   TASK_APP
* @param[in] src_id    TASK_CPPS
* @return If the message was consumed or not.
* @description
* This message is sent to the application each time a peer device has successfully written to the Client
* Characteristic Configuration descriptor of the Vector characteristic.
*
*/
int APP_CppsVectorCfgReqIndHandler(ke_msg_id_t const msgid,
                                   struct cpps_vector_cfg_req_ind *param,
                                   ke_task_id_t const dest_id,
                                   ke_task_id_t const src_id);

/*!
* @brief Receive the event of command completed.
* @param[in] msgid     CPPS_CMP_EVT
* @param[in] param     cpps_cmp_evt
* @param[in] dest_id   TASK_APP
* @param[in] src_id    TASK_CPPS
* @return If the message was consumed or not.
* @description
* The message is used by the CPPS task to inform the sender of a command that the procedure is over and
* contains the status of the procedure.
*
*/
int APP_CppsCmpEvtHandler(ke_msg_id_t const msgid,
                          struct cpps_cmp_evt *param,
                          ke_task_id_t const dest_id,
                          ke_task_id_t const src_id);

#endif
/*! @brief @} APP_CPPS_API */
#endif

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

#ifndef _APP_BASC_H_
#define _APP_BASC_H_

/*!
 * @addtogroup APP_BASC_API
 * @{
 */

#if BLE_BATT_CLIENT
#include "basc_task.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/*! @brief Battery Service Client environment variable */
struct app_basc_env_tag
{
    uint8_t bas_nb[BLE_CONNECTION_MAX]; /*!< Number of BAS instances found */
};

/*******************************************************************************
 * Variables
 ******************************************************************************/
/*! @brief Table of message handlers */
extern const struct ke_state_handler g_AppBascTableHandler;

/*******************************************************************************
 * API
 ******************************************************************************/
/*!
 * @brief Add battery profile.
 */
void APP_BascAddProfileTask(void);

/*!
 * @brief Start the profile - at connection.
 * @param[in] conhdl        Connection handle for which the profile Locator role is enabled.
 * @response  BASC_ENABLE_RSP
 * @description
 * This API message is used for enabling the Client role of the BAS. This Application message contains
 * connection type and the previously saved discovered BAS details on peer.
 */
void APP_BascEnableReq(uint16_t conhdl);

/*!
 * @brief read the value of a characteristic or a descriptor in the peer device database
 * @param[in] conhdl        Connection handle for which the profile Locator role is enabled.
 * @param[in] info            Characteristic info (@see enum basc_info)
 * - BASC_BATT_LVL_VAL,
 * - BASC_NTF_CFG,
 * - BASC_BATT_LVL_PRES_FORMAT,
 * @param[in] bas_nb        Battery Service instance.
 * @response  BASC_READ_INFO_RSP
 * @description
 * This API message shall be used to read the value of a characteristic or a descriptor in the peer device
 * database
 */
void APP_BascReadInfoReq(uint16_t conhdl, uint8_t info, uint8_t bas_nb);

/*!
 * @brief Config battery level character descriptor in the peer device database
 * @param[in] conhdl        Connection handle for which the profile Locator role is enabled.
 * @param[in] ntf_cfg       Possible values for setting client configuration characteristics (@see enum prf_cli_conf)
 * - PRF_CLI_STOP_NTFIND,
 * - PRF_CLI_START_NTF,
 * @param[in] bas_nb        Number of battery Service instance.
 * @response  BASC_BATT_LEVEL_NTF_CFG_RSP
 * @description
 * This API message shall be used to set the notification configuration for specific battery service instance.
 */
void APP_BascBattLevelNtfCfgReq(uint16_t conhdl, uint16_t ntf_cfg, uint8_t bas_nb);

/*!
 * @brief Handles the enable confirmation from the BASC.
 *
 * @param[in] msgid     BASC_ENABLE_RSP
 * @param[in] param     Pointer to struct basc_enable_rsp
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_BASC
 * @return If the message was consumed or not.
 * @description
 * This API is used by the Client to either send the discovery results of BASSS and
 * confirm enabling of the Client role, or to simply confirm enabling of Client
 * role if it is a normal connection and the attribute details are already known.
 */
int APP_BascEnableRspHandler(ke_msg_id_t const msgid,
                             struct basc_enable_rsp *param,
                             ke_task_id_t const dest_id,
                             ke_task_id_t const src_id);

/*!
 * @brief Handles the basc read info rsponse from the BASC.
 *
 * @param[in] msgid     BASC_READ_INFO_RSP
 * @param[in] param     Pointer to struct basc_read_info_rsp
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_BASC
 * @return If the message was consumed or not.
 * @description
 * Handles the basc read info rsponse from the BASC.
 */
int APP_BascReadInfoRspHandler(ke_msg_id_t const msgid,
                               struct basc_read_info_rsp *param,
                               ke_task_id_t const dest_id,
                               ke_task_id_t const src_id);

/*!
 * @brief Handles the basc batt level indication from the BASC.
 *
 * @param[in] msgid     BASC_BATT_LEVEL_IND
 * @param[in] param     Pointer to struct basc_batt_level_ind
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_BASC
 * @return If the message was consumed or not.
 * @description
 * Handles the basc batt level indication from the BASC.
 */
int APP_BascBattLevelIndHandler(ke_msg_id_t const msgid,
                                struct basc_batt_level_ind *param,
                                ke_task_id_t const dest_id,
                                ke_task_id_t const src_id);

/*!
 * @brief Handles the basc batt level notify config response from the BASC.
 *
 * @param[in] msgid     BASC_BATT_LEVEL_NTF_CFG_RSP
 * @param[in] param     Pointer to struct basc_batt_level_ntf_cfg_rsp
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_BASC
 * @return If the message was consumed or not.
 * @description
 * Handles the basc batt level notify config response from the BASC.
 */
int APP_BascBattLevelNtfCfgRsp_handler(ke_msg_id_t const msgid,
                                       struct basc_batt_level_ntf_cfg_rsp *param,
                                       ke_task_id_t const dest_id,
                                       ke_task_id_t const src_id);

#endif /*BLE_BATT_CLIENT */

/*! @brief @} APP_BASC_API */

#endif /* _APP_BASC_H_ */

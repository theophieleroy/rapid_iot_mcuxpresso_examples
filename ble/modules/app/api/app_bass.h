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

#ifndef _APP_BASS_H_
#define _APP_BASS_H_

/*!
 * @addtogroup APP_BASS_API
 * @{
 */

#if (BLE_BATT_SERVER)
#include "bass.h"
#include "bass_task.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/*! @brief Battery Application Module Environment Structure */
struct app_bass_env_tag
{
    uint8_t ntf_cfg[BLE_CONNECTION_MAX];           /*!< notification configured  */
    uint8_t batt_lvl[BASS_NB_BAS_INSTANCES_MAX];   /*!<  current battery level */
    uint16_t start_hdl[BASS_NB_BAS_INSTANCES_MAX]; /*!< Service start handle  */
};

/*! @brief Battery data should be stored in bonded device */
struct app_bass_bond_data
{
    uint8_t ntf_cfg;                                 /*!<   Notification Configuration */
    uint8_t old_batt_lvl[BASS_NB_BAS_INSTANCES_MAX]; /*!<  Old Battery Level used to decide if notification should be
                                                        triggered */
};

/*******************************************************************************
 * Variables
 ******************************************************************************/
/*! @brief Application bass environment */
extern struct app_bass_env_tag g_AppBassEnv;
/*! @brief Table of message handlers */
extern const struct ke_state_handler g_AppBassTableHandler;

/*******************************************************************************
 * API
 ******************************************************************************/
/*!
 * @brief Add battery service profile.
 */
void APP_BassAddProfileTask(void);

/*!
 * @brief This API message can be used after the connection with a peer device has been established in order to
 *           restore known device bond data.
 * @response  BASS_ENABLE_RSP
 * @description
 * The ntf_cfg parameter is a bit field containing notification configuration for each battery service instances.
 *
 */
void APP_BassEnableReq(uint16_t conhdl, uint8_t ntf_cfg, uint8_t *old_batt_lvl);

/*!
 * @brief Handles the reception of APP_MSG_BASS_BATT_LVL_CHK_TIMER message.
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance.
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 */
int APP_BassBattLvlChkTimerHandler(ke_msg_id_t const msgid,
                                   void const *param,
                                   ke_task_id_t const dest_id,
                                   ke_task_id_t const src_id);

/*!
 * @brief Update battery level.
 * @param[in] bas_instance    Battery Service Instance.
 * @param[in] batt_lvl           Battery Level Characteristic value.
 * @response  BASS_BATT_LEVEL_UPD_RSP
 * @description
 * This API message is sent by the application to update the Battery Level Characteristic value for one of the
 * BAS instance. This value will be stored in the database so that it can be read by the peer device.
 */
void APP_BassBattLevelUpdReq(uint8_t bas_instance, uint8_t batt_lvl);

/*!
 * @brief Handles the notification configuration message from the BASS.
 *
 * @param[in] msgid     BASS_BATT_LEVEL_NTF_CFG_IND
 * @param[in] param     Pointer to struct bass_batt_level_ntf_cfg_ind
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_BASS
 * @return If the message was consumed or not.
 * @description
 * This API message is sent to the application when the notification configuration has been modified for one
 * of the Battery Level Characteristics.
 */
int APP_BassBattLevelNtfCfgIndHandler(ke_msg_id_t const msgid,
                                      struct bass_batt_level_ntf_cfg_ind const *param,
                                      ke_task_id_t const dest_id,
                                      ke_task_id_t const src_id);

/*!
 * @brief Handles the batt level update response message.
 *
 * @param[in] msgid     BASS_BATT_LEVEL_UPD_RSP
 * @param[in] param     Pointer to struct bass_batt_level_upd_rsp
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_BASS
 * @return If the message was consumed or not.
 * @description
 * This API message is sent to the application to inform it if a notification has been sent to the peer device.
 */
int APP_BassBattLevelUpdRspHandler(ke_msg_id_t const msgid,
                                   struct bass_batt_level_upd_rsp const *param,
                                   ke_task_id_t const dest_id,
                                   ke_task_id_t const src_id);

/*!
 * @brief Handles the bass enable response message.
 *
 * @param[in] msgid     BASS_ENABLE_RSP
 * @param[in] param     Pointer to struct bass_enable_rsp
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_BASS
 * @return If the message was consumed or not.
 * @description
 * This API message is sent to the application to inform it if restoring bond data for peer device succeed or not.
 */
int APP_BassEnableRspHandler(ke_msg_id_t const msgid,
                             struct bass_enable_rsp const *param,
                             ke_task_id_t const dest_id,
                             ke_task_id_t const src_id);

/*!
 * @brief Handles the bass battery level check timer message.
 *
 * @param[in] msgid     APP_MSG_BASS_BATT_LVL_CHK_TIMER
 * @param[in] param     NULL
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_BASS
 * @return If the message was consumed or not.
 * @description
 * This API message is sent to the application when the bass battery level check timer
 * expired. The batteries level should be checked in this handler.
 */
int APP_BassBattLvlChkTimerHandler(ke_msg_id_t const msgid,
                                   void const *param,
                                   ke_task_id_t const dest_id,
                                   ke_task_id_t const src_id);
#endif /* BLE_BATT_SERVER */

/*! @brief @} APP_BASS_API */

#endif /* _APP_BASS_H_ */

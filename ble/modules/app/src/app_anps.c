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

#include "app_ble.h"
#include "prf_utils.h"
#if (BLE_AN_SERVER)

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static int APP_AnpsEnbleRspHandler(ke_msg_id_t const msgid,
                                   struct anps_enable_rsp *param,
                                   ke_task_id_t const dest_id,
                                   ke_task_id_t const src_id);
static int APP_AnpsNtfImmediateReqIndHandler(ke_msg_id_t const msgid,
                                             struct anps_ntf_immediate_req_ind *param,
                                             ke_task_id_t const dest_id,
                                             ke_task_id_t const src_id);
static int APP_AnpsNtfStatusUpdateIndHandler(ke_msg_id_t const msgid,
                                             struct anps_ntf_status_update_ind *param,
                                             ke_task_id_t const dest_id,
                                             ke_task_id_t const src_id);
static int APP_AnpsCmpEvtHandler(ke_msg_id_t const msgid,
                                 struct anps_cmp_evt *param,
                                 ke_task_id_t const dest_id,
                                 ke_task_id_t const src_id);

/*******************************************************************************
 * Variables
 ******************************************************************************/
const struct ke_msg_handler s_appAnpsMsgHandlerList[] = {
    {ANPS_ENABLE_RSP, (ke_msg_func_t)APP_AnpsEnbleRspHandler},
    {ANPS_NTF_IMMEDIATE_REQ_IND, (ke_msg_func_t)APP_AnpsNtfImmediateReqIndHandler},
    {ANPS_NTF_STATUS_UPDATE_IND, (ke_msg_func_t)APP_AnpsNtfStatusUpdateIndHandler},
    {ANPS_CMP_EVT, (ke_msg_func_t)APP_AnpsCmpEvtHandler},
};

const struct ke_state_handler g_AppAnpsTableHandler = KE_STATE_HANDLER(s_appAnpsMsgHandlerList);

/*******************************************************************************
 * Code
 ******************************************************************************/
void APP_AnpsAddProfileTask(void)
{
    struct anps_db_cfg *db_cfg;
    struct gapm_profile_task_add_cmd *req = KE_MSG_ALLOC_DYN(GAPM_PROFILE_TASK_ADD_CMD, TASK_GAPM, TASK_APP,
                                                             gapm_profile_task_add_cmd, sizeof(struct anps_db_cfg));
    /* Fill message */
    req->operation = GAPM_PROFILE_TASK_ADD;
    req->prf_task_id = TASK_ID_ANPS;
    req->app_task = TASK_APP;
    req->start_hdl = 0;
    /* Set parameters */
    db_cfg = (struct anps_db_cfg *)req->param;
    db_cfg->supp_new_alert_cat.cat_id_mask_0 = CFG_ANS_NEW_ALERT_CAT_ID_MASK_0;
    db_cfg->supp_new_alert_cat.cat_id_mask_1 = CFG_ANS_NEW_ALERT_CAT_ID_MASK_1;
    db_cfg->supp_unread_alert_cat.cat_id_mask_0 = CFG_ANS_UNREAD_ALERT_CAT_ID_MASK_0;
    db_cfg->supp_unread_alert_cat.cat_id_mask_1 = CFG_ANS_UNREAD_ALERT_CAT_ID_MASK_1;
    /* Send the message */
    APP_MsgSend(req);
}

void APP_AnpsEnableReq(uint16_t conhdl, uint16_t new_alert_ntf_cfg, uint16_t unread_alert_status_ntf_cfg)
{
    uint8_t conidx = CONHDL2CONIDX(conhdl);
    struct anps_enable_req *msg = KE_MSG_ALLOC(ANPS_ENABLE_REQ, KE_BUILD_ID(prf_get_task_from_id(TASK_ID_ANPS), conidx),
                                               TASK_APP, anps_enable_req);
    msg->new_alert_ntf_cfg = new_alert_ntf_cfg;
    msg->unread_alert_status_ntf_cfg = unread_alert_status_ntf_cfg;
    APP_MsgSend(msg);
}

/*!
 * @brief    This API message informs the application about the result of the operation
 *
 * @param[in] msgid     ANPS_ENABLE_RSP
 * @param[in] param     Pointer to the struct anps_enable_rsp
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_ANPS
 *
 * @return If the message was consumed or not.
 * @description
 * This API message informs the application about the result of the operation
 */
static int APP_AnpsEnbleRspHandler(ke_msg_id_t const msgid,
                                   struct anps_enable_rsp *param,
                                   ke_task_id_t const dest_id,
                                   ke_task_id_t const src_id)
{
    QPRINTF("ANPS enable status = 0x%x.\r\n", param->status);
    return (KE_MSG_CONSUMED);
}

void APP_AnpsNtfNewAlertCmd(uint16_t conhdl, uint8_t str_len, uint8_t num, uint8_t cat_id, uint8_t *str_info)
{
    uint8_t *str_info_temp;
    uint8_t i;
    struct anps_ntf_alert_cmd *msg =
        KE_MSG_ALLOC_DYN(ANPS_NTF_ALERT_CMD, KE_BUILD_ID(prf_get_task_from_id(TASK_ID_ANPS), CONHDL2CONIDX(conhdl)),
                         TASK_APP, anps_ntf_alert_cmd, str_len - 1);
    msg->operation = ANPS_UPD_NEW_ALERT_OP_CODE;
    msg->value.new_alert.info_str_len = str_len;
    msg->value.new_alert.cat_id = cat_id;
    msg->value.new_alert.nb_new_alert = num;
    str_info_temp = msg->value.new_alert.str_info;
    for (i = 0; i < str_len; i++)
    {
        str_info_temp[i] = str_info[i];
    }
    APP_MsgSend(msg);
}
void APP_AnpsNtfUnreadAlertCmd(uint16_t conhdl, uint8_t cat_id, uint8_t num)
{
    struct anps_ntf_alert_cmd *msg =
        KE_MSG_ALLOC(ANPS_NTF_ALERT_CMD, KE_BUILD_ID(prf_get_task_from_id(TASK_ID_ANPS), CONHDL2CONIDX(conhdl)),
                     TASK_APP, anps_ntf_alert_cmd);
    msg->operation = ANPS_UPD_UNREAD_ALERT_STATUS_OP_CODE;
    msg->value.unread_alert_status.cat_id = cat_id;
    msg->value.unread_alert_status.nb_unread_alert = num;
    APP_MsgSend(msg);
}

/*!
 * @brief    This message is sent to the application when the Alert Notification Control Point Characteristic value is
 * written by the peer device with a 'Notify New Incoming Alert Immediately' or 'Notify Unread Category Status
 * Immediately' command id.
 *
 * @param[in] msgid     ANPS_NTF_IMMEDIATE_REQ_IND
 * @param[in] param     Pointer to the struct anps_ntf_immediate_req_ind
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_ANPS
 *
 * @return If the message was consumed or not.
 * @description
 * This message is sent to the application when the Alert Notification Control Point Characteristic value is
 * written by the peer device with a 'Notify New Incoming Alert Immediately' or 'Notify Unread Category Status
 * Immediately' command id.
 * The cat_ntf_cfg parameter provided information about the categories that has been required by the peer device and
 * which can be notified (supported + enabled).
 */
static int APP_AnpsNtfImmediateReqIndHandler(ke_msg_id_t const msgid,
                                             struct anps_ntf_immediate_req_ind *param,
                                             ke_task_id_t const dest_id,
                                             ke_task_id_t const src_id)
{
    uint16_t cat_ntf_cfg = (param->cat_ntf_cfg.cat_id_mask_0) | (param->cat_ntf_cfg.cat_id_mask_1 << 8);
    APP_AnpsNtfImmediateReqIndCallback(param->alert_type, cat_ntf_cfg);
    return (KE_MSG_CONSUMED);
}

/*!
 * @brief    This message is sent to the application when the value of one of the two Client Characteristic
 * Configuration descriptors has been written by the peer device to enable or disable sending of notifications.
 *
 * @param[in] msgid     ANPS_NTF_STATUS_UPDATE_IND
 * @param[in] param     Pointer to the struct anps_ntf_status_update_ind
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_ANPS
 *
 * @return If the message was consumed or not.
 * @description
 * This message is sent to the application when the value of one of the two Client Characteristic
 * Configuration descriptors has been written by the peer device to enable or disable sending of notifications. It is
 * also
 * sent upon reception of a write request for the Alert Notification Control Point characteristic with the following
 * command ids:
 *  Enable New Incoming Alert Notifications
 *  Enable Unread Category Status Notifications
 *  Disable New Incoming Alert Notifications
 *  Disable Unread Category Status Notifications
 */
static int APP_AnpsNtfStatusUpdateIndHandler(ke_msg_id_t const msgid,
                                             struct anps_ntf_status_update_ind *param,
                                             ke_task_id_t const dest_id,
                                             ke_task_id_t const src_id)
{
    QPRINTF("ANPS ntf status update!\r\n");
    uint16_t cat_id = (param->cat_ntf_cfg.cat_id_mask_0) | (param->cat_ntf_cfg.cat_id_mask_1 << 8);
    switch (param->alert_type)
    {
        case ANP_NEW_ALERT:
            QPRINTF("ANP_NEW_ALERT,ntf_ccc_cfg = 0x%x, cat_id = 0x%x.\r\n", param->ntf_ccc_cfg, cat_id);
            break;
        case ANP_UNREAD_ALERT:
            QPRINTF("ANP_UNREAD_ALERT,ntf_ccc_cfg = 0x%x, cat_id = 0x%x.\r\n", param->ntf_ccc_cfg, cat_id);
            break;
    }
    return (KE_MSG_CONSUMED);
}

/*!
 * @brief    The API message is used by the ANPS task to inform the sender of a command that the procedure is over
 * and contains the status of the procedure.
 *
 * @param[in] msgid     ANPS_CMP_EVT
 * @param[in] param     Pointer to the struct anps_cmp_evt
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_ANPS
 *
 * @return If the message was consumed or not.
 * @description
 * The API message is used by the ANPS task to inform the sender of a command that the procedure is over
 * and contains the status of the procedure.
 */
static int APP_AnpsCmpEvtHandler(ke_msg_id_t const msgid,
                                 struct anps_cmp_evt *param,
                                 ke_task_id_t const dest_id,
                                 ke_task_id_t const src_id)
{
    switch (param->operation)
    {
        case ANPS_UPD_NEW_ALERT_OP_CODE:
            QPRINTF("ANPS_UPD_NEW_ALERT_OP_CODE compelete,status = 0x%x.\r\n", param->status);
            break;
        case ANPS_UPD_UNREAD_ALERT_STATUS_OP_CODE:
            QPRINTF("ANPS_UPD_UNREAD_ALERT_STATUS_OP_CODE compete,status = 0x%x.\r\n", param->status);
            break;
    }

    return (KE_MSG_CONSUMED);
}
#endif

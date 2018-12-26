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
 *   contributors may be used to endorse or promote products derived from thisA
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
#if (BLE_PAS_SERVER)

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static int APP_PaspsEnbleRspHandler(ke_msg_id_t const msgid,
                                    struct pasps_enable_rsp *param,
                                    ke_task_id_t const dest_id,
                                    ke_task_id_t const src_id);
static int APP_PaspsWrittenCharValIndHandler(ke_msg_id_t const msgid,
                                             struct pasps_written_char_val_ind *param,
                                             ke_task_id_t const dest_id,
                                             ke_task_id_t const src_id);
static int APP_PaspsCmpEvtHandler(ke_msg_id_t const msgid,
                                  struct pasps_cmp_evt *param,
                                  ke_task_id_t const dest_id,
                                  ke_task_id_t const src_id);

/*******************************************************************************
 * Variables
 ******************************************************************************/
const struct ke_msg_handler s_appPaspsMsgHandlerList[] = {
    {PASPS_ENABLE_RSP, (ke_msg_func_t)APP_PaspsEnbleRspHandler},
    {PASPS_WRITTEN_CHAR_VAL_IND, (ke_msg_func_t)APP_PaspsWrittenCharValIndHandler},
    {PASPS_CMP_EVT, (ke_msg_func_t)APP_PaspsCmpEvtHandler},

};

const struct ke_state_handler g_AppPaspsTableHandler = KE_STATE_HANDLER(s_appPaspsMsgHandlerList);

/*******************************************************************************
 * Code
 ******************************************************************************/
void APP_PaspsAddProfileTask(void)
{
    struct pasps_db_cfg *db_cfg;
    struct gapm_profile_task_add_cmd *req = KE_MSG_ALLOC_DYN(GAPM_PROFILE_TASK_ADD_CMD, TASK_GAPM, TASK_APP,
                                                             gapm_profile_task_add_cmd, sizeof(struct pasps_db_cfg));
    /* Fill message */
    req->operation = GAPM_PROFILE_TASK_ADD;
    req->prf_task_id = TASK_ID_PASPS;
    req->app_task = TASK_APP;
    req->start_hdl = 0;
    /* Set parameters */
    db_cfg = (struct pasps_db_cfg *)req->param;
    db_cfg->alert_status = CFG_PAS_ALERT_STATUS;
    db_cfg->ringer_setting = CFG_PAS_RINGER_SETTING;
    /* Send the message */
    APP_MsgSend(req);
}

void APP_PaspsEnableReq(uint16_t conhdl, uint16_t alert_status_ntf_cfg, uint16_t ringer_setting_ntf_cfg)
{
    uint8_t conidx = CONHDL2CONIDX(conhdl);
    struct pasps_enable_req *msg = KE_MSG_ALLOC(
        PASPS_ENABLE_REQ, KE_BUILD_ID(prf_get_task_from_id(TASK_ID_PASPS), conidx), TASK_APP, pasps_enable_req);
    msg->alert_status_ntf_cfg = alert_status_ntf_cfg;
    msg->ringer_setting_ntf_cfg = ringer_setting_ntf_cfg;
    APP_MsgSend(msg);
}

/*!
 * @brief    This API message informs the application about the status of the operation
 *
 * @param[in] msgid     PASPS_ENABLE_RSP
 * @param[in] param     Pointer to the struct pasps_enable_rsp
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_PASPS
 *
 * @return If the message was consumed or not.
 * @description
 * This API message informs the application about the status of the operation
 */
static int APP_PaspsEnbleRspHandler(ke_msg_id_t const msgid,
                                    struct pasps_enable_rsp *param,
                                    ke_task_id_t const dest_id,
                                    ke_task_id_t const src_id)
{
    QPRINTF("PASPS enable status = 0x%x.\r\n", param->status);
    return (KE_MSG_CONSUMED);
}

void APP_PaspsUpdateCharValCmd(uint16_t conhdl, uint8_t operation, uint8_t value)
{
    struct pasps_update_char_val_cmd *msg =
        KE_MSG_ALLOC(PASPS_UPDATE_CHAR_VAL_CMD, KE_BUILD_ID(prf_get_task_from_id(TASK_ID_PASPS), CONHDL2CONIDX(conhdl)),
                     TASK_APP, pasps_update_char_val_cmd);
    msg->operation = operation;
    msg->value = value;
    APP_MsgSend(msg);
}

/*!
 * @brief    This API message is sent to the application to inform it that the value of one of the writable attribute
 * has
 * been successfully written by the peer device.
 *
 * @param[in] msgid     PASPS_WRITTEN_CHAR_VAL_IND
 * @param[in] param     Pointer to the struct pasps_written_char_val_ind
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_PASPS
 *
 * @return If the message was consumed or not.
 * @description
 * This API message is sent to the application to inform it that the value of one of the writable attribute has
 * been successfully written by the peer device.
 * The content of the value parameter depends on the received attribute code.
 * When the ringer control point characteristic value is written, the task checks the current state of the device
 * (Ringer
 * Silent or Ringer Normal), if the state can be modified, the PASPC_WRITTEN_CHAR_VAL_IND message is sent to the
 * application which decide if the state can be modified.
 */
static int APP_PaspsWrittenCharValIndHandler(ke_msg_id_t const msgid,
                                             struct pasps_written_char_val_ind *param,
                                             ke_task_id_t const dest_id,
                                             ke_task_id_t const src_id)
{
    uint16_t conhdl = CONIDX2CONHDL(KE_IDX_GET(src_id));
    struct pasps_env_tag *pasps_env = PRF_ENV_GET(PASPS, pasps);
    QPRINTF("pasps_env->ringer_setting= %x.\r\n", pasps_env->ringer_setting);
    switch (param->att_code)
    {
        case PASPS_ALERT_STATUS_NTF_CFG:
            QPRINTF("PASPS_ALERT_STATUS_NTF_CFG = 0x%x\r\n", param->value.alert_status_ntf_cfg);
            break;
        case PASPS_RINGER_SETTING_NTF_CFG:
            QPRINTF("PASPS_RINGER_SETTING_NTF_CFG = 0x%x\r\n", param->value.ringer_setting_ntf_cfg);
            break;
        case PASPS_RINGER_CTNL_PT_CHAR_VAL:
            switch (param->value.ringer_ctnl_pt)
            {
                case PASP_SILENT_MODE:
                    if (pasps_env->ringer_setting == PASP_RINGER_NORMAL)
                    {
                        APP_PaspsSetRingerModeCallback(param->value.ringer_ctnl_pt);
                        APP_PaspsUpdateCharValCmd(conhdl, PASPS_UPD_RINGER_SETTING_OP_CODE, PASP_RINGER_SILENT);
                    }
                    break;
                case PASP_MUTE_ONCE:
                    if (pasps_env->ringer_setting == PASP_RINGER_NORMAL)
                    {
                        APP_PaspsSetRingerModeCallback(param->value.ringer_ctnl_pt);
                    }
                    break;
                case PASP_CANCEL_SILENT_MODE:
                    if (pasps_env->ringer_setting == PASP_RINGER_SILENT)
                    {
                        APP_PaspsSetRingerModeCallback(param->value.ringer_ctnl_pt);
                        APP_PaspsUpdateCharValCmd(conhdl, PASPS_UPD_RINGER_SETTING_OP_CODE, PASP_RINGER_NORMAL);
                    }
                    break;
            }
            break;
    }
    return (KE_MSG_CONSUMED);
}

/*!
 * @brief    The API message is used by the PASPS task to inform the sender of a command that the procedure is over
 * and contains the status of the procedure.
 *
 * @param[in] msgid     PASPS_CMP_EVT
 * @param[in] param     Pointer to the struct pasps_cmp_evt
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_PASPS
 *
 * @return If the message was consumed or not.
 * @description
 * The API message is used by the PASPS task to inform the sender of a command that the procedure is over
 * and contains the status of the procedure.
 */
static int APP_PaspsCmpEvtHandler(ke_msg_id_t const msgid,
                                  struct pasps_cmp_evt *param,
                                  ke_task_id_t const dest_id,
                                  ke_task_id_t const src_id)
{
    QPRINTF("Cmp evt!\r\n");
    switch (param->operation)
    {
        case PASPS_UPD_ALERT_STATUS_OP_CODE:
            QPRINTF("PASPS_UPD_ALERT_STATUS_OP_CODE,status = 0x%x.\r\n", param->status);
            break;
        case PASPS_UPD_RINGER_SETTING_OP_CODE:
            QPRINTF("PASPS_UPD_RINGER_SETTING_OP_CODE,status = 0x%x.\r\n", param->status);
            break;
    }
    return (KE_MSG_CONSUMED);
}

#endif

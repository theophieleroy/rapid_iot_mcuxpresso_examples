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
#if (BLE_TIP_SERVER)

/*******************************************************************************
 * Variables
 ******************************************************************************/
struct app_tips_env_tag g_AppTipsEnv;

const struct ke_msg_handler s_appTipsMsgHandlerList[] = {
    {TIPS_ENABLE_RSP, (ke_msg_func_t)APP_TipsEnableRspHandler},
    {TIPS_UPD_CURR_TIME_RSP, (ke_msg_func_t)APP_TipsUpdCurrTimeRspHandler},
    {TIPS_RD_REQ_IND, (ke_msg_func_t)APP_TipsRdReqIndHandler},
    {TIPS_CURRENT_TIME_CCC_IND, (ke_msg_func_t)APP_TipsCurrentTimeCccIndHandler},
    {TIPS_TIME_UPD_CTNL_PT_IND, (ke_msg_func_t)APP_TipsTimeUpdCtnlPtIndHandler},
};

const struct ke_state_handler g_AppTipsTableHandler = KE_STATE_HANDLER(s_appTipsMsgHandlerList);

/*******************************************************************************
 * Code
 ******************************************************************************/
void APP_TipsAddProfileTask(void)
{
    struct tips_db_cfg *db_cfg;
    struct gapm_profile_task_add_cmd *req = KE_MSG_ALLOC_DYN(GAPM_PROFILE_TASK_ADD_CMD, TASK_GAPM, TASK_APP,
                                                             gapm_profile_task_add_cmd, sizeof(struct tips_db_cfg));

    /* Fill message */
    req->operation = GAPM_PROFILE_TASK_ADD;
    req->sec_lvl = 0;
    req->prf_task_id = TASK_ID_TIPS;
    req->app_task = TASK_APP;
    req->start_hdl = 0;

    /* Set parameters */
    db_cfg = (struct tips_db_cfg *)req->param;
    db_cfg->features = CFG_TIPS_FEATURE;

    g_AppTipsEnv.current_time_intv = TIMER_UPDATE_INTV_VALUE;
    g_AppTipsEnv.time_update_state = TIPS_TIME_UPD_STATE_IDLE;

    /* Send the message */
    APP_MsgSend(req);
}

void APP_TipsEnableReq(uint16_t conhdl, uint16_t current_time_ntf_en)
{
    struct tips_enable_req *msg =
        KE_MSG_ALLOC(TIPS_ENABLE_REQ, KE_BUILD_ID(prf_get_task_from_id(TASK_ID_TIPS), CONHDL2CONIDX(conhdl)), TASK_APP,
                     tips_enable_req);

    msg->current_time_ntf_en = current_time_ntf_en;

    /* Send the message */
    APP_MsgSend(msg);
}

int APP_TipsEnableRspHandler(ke_msg_id_t const msgid,
                             struct tips_enable_rsp *param,
                             ke_task_id_t const dest_id,
                             ke_task_id_t const src_id)
{
    QPRINTF("APP_TipsEnableRspHandler status :0x%x.\r\n", param->status);
    return (KE_MSG_CONSUMED);
}

void APP_TipsUpdCurrTimeReq(uint16_t conhdl, struct tip_curr_time *current_time, uint8_t enable_ntf_send)
{
    struct tips_upd_curr_time_req *msg =
        KE_MSG_ALLOC(TIPS_UPD_CURR_TIME_REQ, KE_BUILD_ID(prf_get_task_from_id(TASK_ID_TIPS), CONHDL2CONIDX(conhdl)),
                     TASK_APP, tips_upd_curr_time_req);

    msg->current_time = *current_time;
    msg->enable_ntf_send = enable_ntf_send;

    /* Send the message */
    APP_MsgSend(msg);
}

int APP_TipsUpdCurrTimeRspHandler(ke_msg_id_t const msgid,
                                  struct tips_upd_curr_time_rsp *param,
                                  ke_task_id_t const dest_id,
                                  ke_task_id_t const src_id)
{
    QPRINTF("APP_TipsUpdCurrTimeRspHandler status :0x%x.\r\n", param->status);
    if (g_AppTipsEnv.time_update_state == TIPS_TIME_UPD_STATE_PENDING)
        g_AppTipsEnv.time_update_state = TIPS_TIME_UPD_STATE_IDLE;
    return (KE_MSG_CONSUMED);
}

int APP_TipsRdReqIndHandler(ke_msg_id_t const msgid,
                            struct tips_rd_req_ind *param,
                            ke_task_id_t const dest_id,
                            ke_task_id_t const src_id)
{
    QPRINTF("APP_TipsRdReqIndHandler char_code :0x%x.\r\n", param->char_code);
    APP_TipsRdReqIndCallback(param->char_code);
    return (KE_MSG_CONSUMED);
}

void APP_TipsRdCfm(uint16_t conhdl, struct tips_rd_cfm *cfm)
{
    struct tips_rd_cfm *msg = KE_MSG_ALLOC(
        TIPS_RD_CFM, KE_BUILD_ID(prf_get_task_from_id(TASK_ID_TIPS), CONHDL2CONIDX(conhdl)), TASK_APP, tips_rd_cfm);

    msg->op_code = cfm->op_code;
    msg->value = cfm->value;

    /* Send the message */
    APP_MsgSend(msg);
}

int APP_TipsCurrentTimeCccIndHandler(ke_msg_id_t const msgid,
                                     struct tips_current_time_ccc_ind *param,
                                     ke_task_id_t const dest_id,
                                     ke_task_id_t const src_id)
{
    QPRINTF("APP_TipsCurrentTimeCccIndHandler cfg_val :0x%x.\r\n", param->cfg_val);
    return (KE_MSG_CONSUMED);
}

int APP_TipsTimeUpdCtnlPtIndHandler(ke_msg_id_t const msgid,
                                    struct tips_time_upd_ctnl_pt_ind *param,
                                    ke_task_id_t const dest_id,
                                    ke_task_id_t const src_id)
{
    QPRINTF("APP_TipsTimeUpdCtnlPtIndHandler value :0x%x.\r\n", param->value);
    APP_TipsTimeUpdCtnlPtIndCallback(param->value);
    return (KE_MSG_CONSUMED);
}

#endif

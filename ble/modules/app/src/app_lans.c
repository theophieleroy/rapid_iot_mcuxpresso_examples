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
#if (BLE_LN_SENSOR)

/*******************************************************************************
 * Variables
 ******************************************************************************/
const struct ke_msg_handler s_appLansMsgHandlerList[] = {
    {LANS_ENABLE_RSP, (ke_msg_func_t)APP_LansEnableRspHandler},
    {LANS_NTF_LOC_SPEED_RSP, (ke_msg_func_t)APP_LansNtfLocSpeedRspHandler},
    {LANS_NTF_NAVIGATION_RSP, (ke_msg_func_t)APP_LansNtfNavigationRspHandler},
    {LANS_UPD_POS_Q_RSP, (ke_msg_func_t)APP_LansUpdPosQRspHandler},
    {LANS_LN_CTNL_PT_REQ_IND, (ke_msg_func_t)APP_LansLnCtnlPtReqIndHandler},
    {LANS_CFG_NTFIND_IND, (ke_msg_func_t)APP_LansCfgNtfindIndHandler},
    {LANS_CMP_EVT, (ke_msg_func_t)APP_LansCmpEvtHandler},
};

const struct ke_state_handler g_AppLansTableHandler = KE_STATE_HANDLER(s_appLansMsgHandlerList);
struct app_lans_env_tag g_AppLansEnv;

/*******************************************************************************
 * Code
 ******************************************************************************/
void APP_LansAddProfileTask(void)
{
    struct lans_db_cfg *db_cfg;
    struct gapm_profile_task_add_cmd *req = KE_MSG_ALLOC_DYN(GAPM_PROFILE_TASK_ADD_CMD, TASK_GAPM, TASK_APP,
                                                             gapm_profile_task_add_cmd, sizeof(struct lans_db_cfg));

    /* Fill message */
    req->operation = GAPM_PROFILE_TASK_ADD;
    req->sec_lvl = 0;
    req->prf_task_id = TASK_ID_LANS;
    req->app_task = TASK_APP;
    req->start_hdl = 0;

    /* Set parameters */
    db_cfg = (struct lans_db_cfg *)req->param;
    db_cfg->ln_feature = CFG_LANS_FEATURE;
    db_cfg->prfl_config = CFG_LANS_PROFILE_CFG;

    g_AppLansEnv.noti_intv = 5;

    // Send the message
    APP_MsgSend(req);
}

void APP_LansEnableReq(uint16_t conhdl, uint16_t prfl_ntf_ind_cfg)
{
    struct lans_enable_req *msg =
        KE_MSG_ALLOC(LANS_ENABLE_REQ, KE_BUILD_ID(prf_get_task_from_id(TASK_ID_LANS), CONHDL2CONIDX(conhdl)), TASK_APP,
                     lans_enable_req);

    msg->conidx = CONHDL2CONIDX(conhdl);
    msg->prfl_ntf_ind_cfg = prfl_ntf_ind_cfg;

    /* Send the message */
    APP_MsgSend(msg);
}

int APP_LansEnableRspHandler(ke_msg_id_t const msgid,
                             struct lans_enable_rsp *param,
                             ke_task_id_t const dest_id,
                             ke_task_id_t const src_id)
{
    QPRINTF("APP_LansEnableRspHandler status :0x%x.\r\n", param->status);

    return (KE_MSG_CONSUMED);
}

void APP_LansNtfLocSpeedReq(uint16_t conhdl, struct lanp_loc_speed *parameters)
{
    struct lans_ntf_loc_speed_req *msg =
        KE_MSG_ALLOC(LANS_NTF_LOC_SPEED_REQ, KE_BUILD_ID(prf_get_task_from_id(TASK_ID_LANS), CONHDL2CONIDX(conhdl)),
                     TASK_APP, lans_ntf_loc_speed_req);

    msg->parameters = *parameters;

    /* Send the message */
    APP_MsgSend(msg);
}

int APP_LansNtfLocSpeedRspHandler(ke_msg_id_t const msgid,
                                  struct lans_ntf_loc_speed_rsp *param,
                                  ke_task_id_t const dest_id,
                                  ke_task_id_t const src_id)
{
    QPRINTF("APP_LansNtfLocSpeedRspHandler status :0x%x.\r\n", param->status);
    return (KE_MSG_CONSUMED);
}

void APP_LansNtfNavigationReq(uint16_t conhdl, struct lanp_navigation *parameters)
{
    struct lans_ntf_navigation_req *msg =
        KE_MSG_ALLOC(LANS_NTF_NAVIGATION_REQ, KE_BUILD_ID(prf_get_task_from_id(TASK_ID_LANS), CONHDL2CONIDX(conhdl)),
                     TASK_APP, lans_ntf_navigation_req);

    msg->parameters = *parameters;

    /* Send the message */
    APP_MsgSend(msg);
}

int APP_LansNtfNavigationRspHandler(ke_msg_id_t const msgid,
                                    struct lans_ntf_navigation_rsp *param,
                                    ke_task_id_t const dest_id,
                                    ke_task_id_t const src_id)
{
    QPRINTF("APP_LansNtfNavigationRspHandler status :0x%x.\r\n", param->status);
    return (KE_MSG_CONSUMED);
}

void APP_LansUpdPosQReq(uint16_t conhdl, struct lanp_posq *parameters)
{
    struct lans_upd_pos_q_req *msg =
        KE_MSG_ALLOC(LANS_UPD_POS_Q_REQ, KE_BUILD_ID(prf_get_task_from_id(TASK_ID_LANS), CONHDL2CONIDX(conhdl)),
                     TASK_APP, lans_upd_pos_q_req);

    msg->parameters = *parameters;

    /* Send the message */
    APP_MsgSend(msg);
}

int APP_LansUpdPosQRspHandler(ke_msg_id_t const msgid,
                              struct lans_upd_pos_q_rsp *param,
                              ke_task_id_t const dest_id,
                              ke_task_id_t const src_id)
{
    QPRINTF("APP_LansUpdPosQRspHandler status :0x%x.\r\n", param->status);
    return (KE_MSG_CONSUMED);
}

int APP_LansLnCtnlPtReqIndHandler(ke_msg_id_t const msgid,
                                  struct lans_ln_ctnl_pt_req_ind *param,
                                  ke_task_id_t const dest_id,
                                  ke_task_id_t const src_id)
{
    QPRINTF("APP_LansLnCtnlPtReqIndHandler opcode :0x%x.\r\n", param->op_code);
    APP_LansLnCtnlPtReqIndCallback((void *)param);
    return (KE_MSG_CONSUMED);
}

void APP_LansLnCtnlPtCfm(struct lans_ln_ctnl_pt_cfm *parameters)
{
    struct lans_ln_ctnl_pt_cfm *msg =
        KE_MSG_ALLOC(LANS_LN_CTNL_PT_CFM, KE_BUILD_ID(prf_get_task_from_id(TASK_ID_LANS), parameters->conidx), TASK_APP,
                     lans_ln_ctnl_pt_cfm);

    msg->conidx = parameters->conidx;
    msg->op_code = parameters->op_code;
    msg->status = parameters->status;
    msg->value = parameters->value;

    // Send the message
    APP_MsgSend(msg);
}

int APP_LansCfgNtfindIndHandler(ke_msg_id_t const msgid,
                                struct lans_cfg_ntfind_ind *param,
                                ke_task_id_t const dest_id,
                                ke_task_id_t const src_id)
{
    QPRINTF("APP_LansCfgNtfindIndHandler char_code :0x%x ntf_cfg :0x%x.\r\n", param->char_code, param->ntf_cfg);
    return (KE_MSG_CONSUMED);
}

int APP_LansCmpEvtHandler(ke_msg_id_t const msgid,
                          struct lans_cmp_evt *param,
                          ke_task_id_t const dest_id,
                          ke_task_id_t const src_id)
{
    QPRINTF("APP_LansCmpEvtHandler status :0x%x operation :0x%x.\r\n", param->status, param->operation);
    return (KE_MSG_CONSUMED);
}

#endif

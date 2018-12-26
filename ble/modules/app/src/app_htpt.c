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
#if (BLE_HT_THERMOM)

/*******************************************************************************
 * Variables
 ******************************************************************************/
struct app_htpt_env_tag g_AppHtptEnv;

const struct ke_msg_handler s_appHtptMsgHandlerList[] = {
    {HTPT_ENABLE_RSP, (ke_msg_func_t)APP_HtptEnableRspHandler},
    {HTPT_TEMP_SEND_RSP, (ke_msg_func_t)APP_HtptTempSendRspHandler},
    {HTPT_MEAS_INTV_UPD_RSP, (ke_msg_func_t)APP_HtptMeasIntvUpdRspHandler},
    {HTPT_MEAS_INTV_CHG_REQ_IND, (ke_msg_func_t)APP_HtptMeasIntvChgReqIndHandler},
    {HTPT_CFG_INDNTF_IND, (ke_msg_func_t)APP_HtptCfgIndntfIndHandler},

};

const struct ke_state_handler g_AppHtptTableHandler = KE_STATE_HANDLER(s_appHtptMsgHandlerList);

/*******************************************************************************
 * Code
 ******************************************************************************/
void APP_HtptAddProfileTask(void)
{
    struct htpt_db_cfg *db_cfg;
    struct gapm_profile_task_add_cmd *req = KE_MSG_ALLOC_DYN(GAPM_PROFILE_TASK_ADD_CMD, TASK_GAPM, TASK_APP,
                                                             gapm_profile_task_add_cmd, sizeof(struct htpt_db_cfg));

    /* Fill message */
    req->operation = GAPM_PROFILE_TASK_ADD;
    req->sec_lvl = 0;
    req->prf_task_id = TASK_ID_HTPT;
    req->app_task = TASK_APP;
    req->start_hdl = 0;

    /* Set parameters */
    db_cfg = (struct htpt_db_cfg *)req->param;
    db_cfg->features = CFG_HTPT_FEATURE;
    db_cfg->meas_intv = g_AppHtptEnv.meas_intv = MEAS_INTL_DEFAULT_VALUE;
    db_cfg->temp_type = CFG_HTPT_TEMP_TYPE;
    db_cfg->valid_range_max = MEAS_INTL_RANGE_MAX;
    db_cfg->valid_range_min = MEAS_INTL_RANGE_MIN;

    /* Send the message */
    APP_MsgSend(req);
}

void APP_HtptEnableReq(uint16_t conhdl, uint8_t ntf_ind_cfg)
{
    struct htpt_enable_req *msg =
        KE_MSG_ALLOC(HTPT_ENABLE_REQ, KE_BUILD_ID(prf_get_task_from_id(TASK_ID_HTPT), CONHDL2CONIDX(conhdl)), TASK_APP,
                     htpt_enable_req);

    msg->conidx = CONHDL2CONIDX(conhdl);
    msg->ntf_ind_cfg = ntf_ind_cfg;

    /* Send the message */
    APP_MsgSend(msg);
}

int APP_HtptEnableRspHandler(ke_msg_id_t const msgid,
                             struct htpt_enable_rsp *param,
                             ke_task_id_t const dest_id,
                             ke_task_id_t const src_id)
{
    QPRINTF("APP_HtptEnableRspHandler status :0x%x.\r\n", param->status);
    return (KE_MSG_CONSUMED);
}

void APP_HtptTempSendReq(uint16_t conhdl, struct htp_temp_meas *temp_meas, bool stable_meas)
{
    struct htpt_temp_send_req *msg =
        KE_MSG_ALLOC(HTPT_TEMP_SEND_REQ, KE_BUILD_ID(prf_get_task_from_id(TASK_ID_HTPT), CONHDL2CONIDX(conhdl)),
                     TASK_APP, htpt_temp_send_req);

    msg->temp_meas = *temp_meas;
    msg->stable_meas = stable_meas;

    /* Send the message */
    APP_MsgSend(msg);
}

int APP_HtptTempSendRspHandler(ke_msg_id_t const msgid,
                               struct htpt_temp_send_rsp *param,
                               ke_task_id_t const dest_id,
                               ke_task_id_t const src_id)
{
    QPRINTF("APP_HtptTempSendRspHandler status :0x%x.\r\n", param->status);
    return (KE_MSG_CONSUMED);
}

void APP_HtptMeasIntvUpdReq(uint16_t conhdl, uint16_t meas_intv)
{
    struct htpt_meas_intv_upd_req *msg =
        KE_MSG_ALLOC(HTPT_MEAS_INTV_UPD_REQ, KE_BUILD_ID(prf_get_task_from_id(TASK_ID_HTPT), CONHDL2CONIDX(conhdl)),
                     TASK_APP, htpt_meas_intv_upd_req);

    msg->meas_intv = g_AppHtptEnv.temp_meas_intv = meas_intv;

    /* Send the message */
    APP_MsgSend(msg);
}

int APP_HtptMeasIntvUpdRspHandler(ke_msg_id_t const msgid,
                                  struct htpt_meas_intv_upd_rsp *param,
                                  ke_task_id_t const dest_id,
                                  ke_task_id_t const src_id)
{
    QPRINTF("APP_HtptMeasIntvUpdRspHandler status:0x%x .\r\n", param->status);

    if (GAP_ERR_NO_ERROR == param->status)
    {
        g_AppHtptEnv.meas_intv = g_AppHtptEnv.temp_meas_intv;
        QPRINTF("APP_HtptMeasIntvUpdRspHandler interval: %d.\r\n", g_AppHtptEnv.meas_intv);
    }

    return (KE_MSG_CONSUMED);
}

int APP_HtptMeasIntvChgReqIndHandler(ke_msg_id_t const msgid,
                                     struct htpt_meas_intv_chg_req_ind *param,
                                     ke_task_id_t const dest_id,
                                     ke_task_id_t const src_id)
{
    struct htpt_meas_intv_chg_cfm *msg =
        KE_MSG_ALLOC(HTPT_MEAS_INTV_CHG_CFM, KE_BUILD_ID(prf_get_task_from_id(TASK_ID_HTPT), param->conidx), TASK_APP,
                     htpt_meas_intv_chg_cfm);

    msg->conidx = param->conidx;
    msg->status = GAP_ERR_NO_ERROR;

    g_AppHtptEnv.meas_intv = param->intv;
    QPRINTF("APP_HtptMeasIntvChgReqIndHandler interval : %d \r\n", g_AppHtptEnv.meas_intv);

    /* Send the confirm message to peer device */
    APP_MsgSend(msg);

    return (KE_MSG_CONSUMED);
}

int APP_HtptCfgIndntfIndHandler(ke_msg_id_t const msgid,
                                struct htpt_cfg_indntf_ind *param,
                                ke_task_id_t const dest_id,
                                ke_task_id_t const src_id)
{
    QPRINTF("APP_HtptCfgIndntfIndHandler indication configuration: %d\r\n", param->ntf_ind_cfg);
    return (KE_MSG_CONSUMED);
}

#endif

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
#include "ke_mem.h"

#if (BLE_HID_DEVICE)

/*******************************************************************************
 * Variables
 ******************************************************************************/
struct app_hogpd_env_tag g_AppHogpdEnv;

const struct ke_msg_handler s_appHogpdMsgHandlerList[] = {
    {HOGPD_ENABLE_RSP, (ke_msg_func_t)APP_HogpdEnableRspHandler},
    {HOGPD_NTF_CFG_IND, (ke_msg_func_t)APP_HogpdNtfCfgIndHandler},
    {HOGPD_PROTO_MODE_REQ_IND, (ke_msg_func_t)APP_HogpdProtoModeReqIndHandler},
    {HOGPD_CTNL_PT_IND, (ke_msg_func_t)APP_HogpdCtnlPtIndHandler},
    {HOGPD_REPORT_UPD_RSP, (ke_msg_func_t)APP_HogpdReportUpdRspHandler},
    {HOGPD_REPORT_REQ_IND, (ke_msg_func_t)APP_HogpdReportReqIndHandler},

};

const struct ke_state_handler g_AppHogpdTableHandler = KE_STATE_HANDLER(s_appHogpdMsgHandlerList);

/*******************************************************************************
 * Code
 ******************************************************************************/
void APP_HogpdAddProfileTask(void)
{
    struct hogpd_db_cfg *db_cfg;
    struct gapm_profile_task_add_cmd *req = KE_MSG_ALLOC_DYN(GAPM_PROFILE_TASK_ADD_CMD, TASK_GAPM, TASK_APP,
                                                             gapm_profile_task_add_cmd, sizeof(struct hogpd_db_cfg));

    /* Fill message */
    req->operation = GAPM_PROFILE_TASK_ADD;
    req->sec_lvl = 0;
    req->prf_task_id = TASK_ID_HOGPD;
    req->app_task = TASK_APP;
    req->start_hdl = 0;

    /* Set parameters */
    db_cfg = (struct hogpd_db_cfg *)req->param;
    db_cfg->hids_nb = HOGPD_NB_HIDS_INST_MAX;

    db_cfg->cfg[0].svc_features = CFG_HOGPD_MOUSE_FEATURE;
    db_cfg->cfg[0].report_nb = MOUSE_REPORT_NUMBER;
    db_cfg->cfg[0].report_char_cfg[0] = HOGPD_CFG_REPORT_IN;
    db_cfg->cfg[0].report_id[0] = MOUSE_INPUT_REPORT_ID;
    db_cfg->cfg[0].hid_info.bcdHID = BINARY_CODED_DECIMAL;
    db_cfg->cfg[0].hid_info.bCountryCode = COUNTRY_CODE;
    db_cfg->cfg[0].hid_info.flags = HIDS_NORM_CONNECTABLE;

    g_AppHogpdEnv.hid_noti_intv = HID_NOTIFY_INTERVAL; /* 100ms*/
    g_AppHogpdEnv.protocol_mode = HOGP_REPORT_PROTOCOL_MODE;

    /* Send the message */
    APP_MsgSend(req);
}

void APP_HogpdEnableReq(uint16_t conhdl, uint8_t ntf_cfg[HOGPD_NB_HIDS_INST_MAX])
{
    uint8_t i;

    struct hogpd_enable_req *msg =
        KE_MSG_ALLOC(HOGPD_ENABLE_REQ, KE_BUILD_ID(prf_get_task_from_id(TASK_ID_HOGPD), CONHDL2CONIDX(conhdl)),
                     TASK_APP, hogpd_enable_req);

    msg->conidx = CONHDL2CONIDX(conhdl);
    for (i = 0; i < HOGPD_NB_HIDS_INST_MAX; i++)
    {
        msg->ntf_cfg[i] = ntf_cfg[i];
    }
    /* Send the message */
    APP_MsgSend(msg);
}

int APP_HogpdEnableRspHandler(ke_msg_id_t const msgid,
                              struct hogpd_enable_rsp *param,
                              ke_task_id_t const dest_id,
                              ke_task_id_t const src_id)
{
    QPRINTF("APP_HogpdEnableRspHandler status :0x%x.\r\n", param->status);
    return (KE_MSG_CONSUMED);
}

int APP_HogpdNtfCfgIndHandler(ke_msg_id_t const msgid,
                              struct hogpd_ntf_cfg_ind *param,
                              ke_task_id_t const dest_id,
                              ke_task_id_t const src_id)
{
    QPRINTF("APP_HogpdNtfCfgIndHandler ntf_cfg[0] :0x%x ntf_cfg[1] :0x%x.\r\n", param->ntf_cfg[0], param->ntf_cfg[1]);
    g_AppHogpdEnv.current_idx = param->conidx;

    return (KE_MSG_CONSUMED);
}

int APP_HogpdProtoModeReqIndHandler(ke_msg_id_t const msgid,
                                    struct hogpd_proto_mode_req_ind *param,
                                    ke_task_id_t const dest_id,
                                    ke_task_id_t const src_id)
{
    QPRINTF("APP_HogpdProtoModeReqIndHandler operation :0x%x proto_mode :0x%x.\r\n", param->operation,
            param->proto_mode);

    g_AppHogpdEnv.current_idx = param->conidx;

    if ((param->operation == HOGPD_OP_REPORT_WRITE) || (param->operation == HOGPD_OP_PROT_UPDATE))
        g_AppHogpdEnv.protocol_mode = param->proto_mode;

    APP_HogpdProtoModeCfm(CONIDX2CONHDL(param->conidx), GAP_ERR_NO_ERROR, param->hid_idx, param->proto_mode);

    return (KE_MSG_CONSUMED);
}

void APP_HogpdProtoModeCfm(uint16_t conhdl, uint8_t status, uint8_t hid_idx, uint8_t proto_mode)
{
    QPRINTF("APP_HogpdProtoModeCfm.\r\n");
    struct hogpd_proto_mode_cfm *msg =
        KE_MSG_ALLOC(HOGPD_PROTO_MODE_CFM, KE_BUILD_ID(prf_get_task_from_id(TASK_ID_HOGPD), CONHDL2CONIDX(conhdl)),
                     TASK_APP, hogpd_proto_mode_cfm);

    msg->conidx = CONHDL2CONIDX(conhdl);
    msg->status = status;
    msg->hid_idx = hid_idx;
    msg->proto_mode = proto_mode;

    /* Send the message */
    APP_MsgSend(msg);
}

int APP_HogpdCtnlPtIndHandler(ke_msg_id_t const msgid,
                              struct hogpd_ctnl_pt_ind *param,
                              ke_task_id_t const dest_id,
                              ke_task_id_t const src_id)
{
    QPRINTF("APP_HogpdCtnlPtIndHandler hid_ctnl_pt :0x%x.\r\n", param->hid_ctnl_pt);
    g_AppHogpdEnv.current_idx = param->conidx;

    APP_HogpdCtnlPtIndCallback(param->hid_idx, param->hid_ctnl_pt);
    return (KE_MSG_CONSUMED);
}

int APP_HogpdReportUpdRspHandler(ke_msg_id_t const msgid,
                                 struct hogpd_report_upd_rsp *param,
                                 ke_task_id_t const dest_id,
                                 ke_task_id_t const src_id)
{
    QPRINTF("APP_HogpdReportUpdRspHandler status :0x%x.\r\n", param->status);
    return (KE_MSG_CONSUMED);
}

int APP_HogpdReportReqIndHandler(ke_msg_id_t const msgid,
                                 struct hogpd_report_req_ind *param,
                                 ke_task_id_t const dest_id,
                                 ke_task_id_t const src_id)
{
    QPRINTF("APP_HogpdReportReqIndHandler operation :0x%x.\r\n", param->operation);
    g_AppHogpdEnv.current_idx = param->conidx;

    APP_HogpdReportReqIndCallback(param->operation, (void *)&(param->report));

    return (KE_MSG_CONSUMED);
}

void APP_HogpdReportCfm(uint16_t conhdl, uint8_t status, uint8_t operation, struct hogpd_report_info *report)
{
    struct hogpd_report_cfm *msg =
        KE_MSG_ALLOC_DYN(HOGPD_REPORT_CFM, KE_BUILD_ID(prf_get_task_from_id(TASK_ID_HOGPD), CONHDL2CONIDX(conhdl)),
                         TASK_APP, hogpd_report_cfm, report->length);

    msg->conidx = CONHDL2CONIDX(conhdl);
    msg->status = status;
    msg->operation = operation;
    memcpy(&msg->report, report, sizeof(struct hogpd_report_info) + report->length);

    /* Send the message */
    APP_MsgSend(msg);
}

#endif

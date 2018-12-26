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
#if (BLE_SP_SERVER)

/*******************************************************************************
 * Variables
 ******************************************************************************/
struct app_scpps_env_tag g_AppScppsEnv;

const struct ke_msg_handler s_appScppsMsgHandlerList[] = {
    {SCPPS_ENABLE_RSP, (ke_msg_func_t)APP_ScppsEnableRspHandler},
    {SCPPS_SCAN_INTV_WD_IND, (ke_msg_func_t)APP_ScppsScanIntvWdIndHandler},
    {SCPPS_SCAN_REFRESH_NTF_CFG_IND, (ke_msg_func_t)APP_ScppsScanRefreshNtfCfgIndHandler},
    {SCPPS_SCAN_REFRESH_SEND_RSP, (ke_msg_func_t)APP_ScppsScanRefreshSendRspHandler},

};

const struct ke_state_handler g_AppScppsTableHandler = KE_STATE_HANDLER(s_appScppsMsgHandlerList);

/*******************************************************************************
 * Code
 ******************************************************************************/
void APP_ScppsAddProfileTask(void)
{
    struct scpps_db_cfg *db_cfg;
    struct gapm_profile_task_add_cmd *req = KE_MSG_ALLOC_DYN(GAPM_PROFILE_TASK_ADD_CMD, TASK_GAPM, TASK_APP,
                                                             gapm_profile_task_add_cmd, sizeof(struct scpps_db_cfg));

    /* Fill message */
    req->operation = GAPM_PROFILE_TASK_ADD;
    req->sec_lvl = 0;
    req->prf_task_id = TASK_ID_SCPPS;
    req->app_task = TASK_APP;
    req->start_hdl = 0;

    /* Set parameters */
    db_cfg = (struct scpps_db_cfg *)req->param;
    db_cfg->features = CFG_SCPPS_FEATURE;

    /* Send the message */
    APP_MsgSend(req);
}

void APP_ScppsEnableReq(uint16_t conhdl, uint8_t ntf_cfg)
{
    struct scpps_enable_req *msg =
        KE_MSG_ALLOC(SCPPS_ENABLE_REQ, KE_BUILD_ID(prf_get_task_from_id(TASK_ID_SCPPS), CONHDL2CONIDX(conhdl)),
                     TASK_APP, scpps_enable_req);

    msg->conidx = CONHDL2CONIDX(conhdl);
    msg->ntf_cfg = ntf_cfg;

    /* Send the message */
    APP_MsgSend(msg);
}

int APP_ScppsEnableRspHandler(ke_msg_id_t const msgid,
                              struct scpps_enable_rsp *param,
                              ke_task_id_t const dest_id,
                              ke_task_id_t const src_id)
{
    QPRINTF("APP_ScppsEnableRspHandler status :0x%x.\r\n", param->status);
    return (KE_MSG_CONSUMED);
}

int APP_ScppsScanIntvWdIndHandler(ke_msg_id_t const msgid,
                                  struct scpps_scan_intv_wd_ind *param,
                                  ke_task_id_t const dest_id,
                                  ke_task_id_t const src_id)
{
    uint8_t conidx = param->conidx;
    QPRINTF("APP_ScppsScanIntvWdIndHandler le_scan_intv:0x%x le_scan_window:0x%x.\r\n",
            param->scan_intv_wd.le_scan_intv, param->scan_intv_wd.le_scan_window);
    g_AppScppsEnv.le_scan_intv[conidx] = param->scan_intv_wd.le_scan_intv;
    g_AppScppsEnv.le_scan_window[conidx] = param->scan_intv_wd.le_scan_window;

    return (KE_MSG_CONSUMED);
}

int APP_ScppsScanRefreshNtfCfgIndHandler(ke_msg_id_t const msgid,
                                         struct scpps_scan_refresh_ntf_cfg_ind *param,
                                         ke_task_id_t const dest_id,
                                         ke_task_id_t const src_id)
{
    QPRINTF("APP_ScppsScanRefreshNtfCfgIndHandler ntf_cfg:0x%x.\r\n", param->ntf_cfg);
    return (KE_MSG_CONSUMED);
}

void APP_ScppsScanRefreshSendReq(uint16_t conhdl)
{
    struct scpps_scan_refresh_send_req *msg = KE_MSG_ALLOC(
        SCPPS_SCAN_REFRESH_SEND_REQ, KE_BUILD_ID(prf_get_task_from_id(TASK_ID_SCPPS), CONHDL2CONIDX(conhdl)), TASK_APP,
        scpps_scan_refresh_send_req);

    msg->conidx = CONHDL2CONIDX(conhdl);

    /* Send the message */
    APP_MsgSend(msg);
}

int APP_ScppsScanRefreshSendRspHandler(ke_msg_id_t const msgid,
                                       struct scpps_scan_refresh_send_rsp *param,
                                       ke_task_id_t const dest_id,
                                       ke_task_id_t const src_id)
{
    QPRINTF("APP_ScppsScanRefreshSendRspHandler status :0x%x.\r\n", param->status);
    return (KE_MSG_CONSUMED);
}

#endif

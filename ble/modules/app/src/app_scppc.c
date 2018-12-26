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
#if BLE_SP_CLIENT

/*******************************************************************************
 * Variables
 ******************************************************************************/
const struct ke_msg_handler s_appScppcMsgHandlerList[] = {
    {SCPPC_ENABLE_RSP, (ke_msg_func_t)APP_ScppcEnableRspHandler},
    {SCPPC_SCAN_INTV_WD_WR_RSP, (ke_msg_func_t)APP_ScppcScanIntvWdWrRspHandler},
    {SCPPC_SCAN_REFRESH_NTF_CFG_RD_RSP, (ke_msg_func_t)APP_ScppcScanRefreshNtfCfgRdRspHandler},
    {SCPPC_SCAN_REFRESH_NTF_CFG_RSP, (ke_msg_func_t)APP_ScppcScanRefreshNtfCfgRspHandler},
    {SCPPC_SCAN_REFRESH_IND, (ke_msg_func_t)APP_ScppcScanRefreshIndHandler},
};

const struct ke_state_handler g_AppScppcTableHandler = KE_STATE_HANDLER(s_appScppcMsgHandlerList);

/*******************************************************************************
 * Code
 ******************************************************************************/
void APP_ScppcAddProfileTask(void)
{
    struct gapm_profile_task_add_cmd *req =
        KE_MSG_ALLOC_DYN(GAPM_PROFILE_TASK_ADD_CMD, TASK_GAPM, TASK_APP, gapm_profile_task_add_cmd, 0);
    /* Fill message */
    req->operation = GAPM_PROFILE_TASK_ADD;
    req->sec_lvl = 0;
    req->prf_task_id = TASK_ID_SCPPC;
    req->app_task = TASK_APP;
    req->start_hdl = 0;

    /* Send the message */
    APP_MsgSend(req);
}

void APP_ScppcEnableReq(uint16_t conhdl)
{
    struct scppc_enable_req *msg =
        KE_MSG_ALLOC(SCPPC_ENABLE_REQ, KE_BUILD_ID(prf_get_task_from_id(TASK_ID_SCPPC), CONHDL2CONIDX(conhdl)),
                     TASK_APP, scppc_enable_req);

    msg->con_type = PRF_CON_DISCOVERY;

    /* Send the message */
    APP_MsgSend(msg);
}

int APP_ScppcEnableRspHandler(ke_msg_id_t const msgid,
                              struct scppc_enable_rsp *param,
                              ke_task_id_t const dest_id,
                              ke_task_id_t const src_id)
{
    QPRINTF("APP_ScppcEnableRspHandler status: 0x%x.\r\n", param->status);
    return (KE_MSG_CONSUMED);
}

void APP_ScppcScanIntvWdWrReq(uint16_t conhdl, uint16_t le_scan_intv, uint16_t le_scan_window)
{
    struct scppc_scan_intv_wd_wr_req *msg =
        KE_MSG_ALLOC(SCPPC_SCAN_INTV_WD_WR_REQ, KE_BUILD_ID(prf_get_task_from_id(TASK_ID_SCPPC), CONHDL2CONIDX(conhdl)),
                     TASK_APP, scppc_scan_intv_wd_wr_req);

    msg->scan_intv_wd.le_scan_intv = le_scan_intv;
    msg->scan_intv_wd.le_scan_window = le_scan_window;

    /* Send the message */
    APP_MsgSend(msg);
}

int APP_ScppcScanIntvWdWrRspHandler(ke_msg_id_t const msgid,
                                    struct scppc_scan_intv_wd_wr_rsp *param,
                                    ke_task_id_t const dest_id,
                                    ke_task_id_t const src_id)
{
    QPRINTF("APP_ScppcScanIntvWdWrRspHandler status: 0x%x.\r\n", param->status);
    return (KE_MSG_CONSUMED);
}

void APP_ScppcScanRefreshNtfCfgRdReq(uint16_t conhdl)
{
    struct scppc_scan_refresh_ntf_cfg_rd_req *msg = KE_MSG_ALLOC(
        SCPPC_SCAN_REFRESH_NTF_CFG_RD_REQ, KE_BUILD_ID(prf_get_task_from_id(TASK_ID_SCPPC), CONHDL2CONIDX(conhdl)),
        TASK_APP, scppc_scan_refresh_ntf_cfg_rd_req);

    msg->dummy = 0;

    /* Send the message */
    APP_MsgSend(msg);
}

int APP_ScppcScanRefreshNtfCfgRdRspHandler(ke_msg_id_t const msgid,
                                           struct scppc_scan_refresh_ntf_cfg_rd_rsp *param,
                                           ke_task_id_t const dest_id,
                                           ke_task_id_t const src_id)
{
    QPRINTF("APP_ScppcScanRefreshNtfCfgRdRspHandler status: 0x%x ntf_cfg: 0x%x.\r\n", param->status, param->ntf_cfg);
    return (KE_MSG_CONSUMED);
}

void APP_ScppcScanRefreshNtfCfgReq(uint16_t conhdl, uint16_t ntf_cfg)
{
    struct scppc_scan_refresh_ntf_cfg_req *msg = KE_MSG_ALLOC(
        SCPPC_SCAN_REFRESH_NTF_CFG_REQ, KE_BUILD_ID(prf_get_task_from_id(TASK_ID_SCPPC), CONHDL2CONIDX(conhdl)),
        TASK_APP, scppc_scan_refresh_ntf_cfg_req);

    msg->ntf_cfg = ntf_cfg;

    /* Send the message */
    APP_MsgSend(msg);
}

int APP_ScppcScanRefreshNtfCfgRspHandler(ke_msg_id_t const msgid,
                                         struct scppc_scan_refresh_ntf_cfg_rsp *param,
                                         ke_task_id_t const dest_id,
                                         ke_task_id_t const src_id)
{
    QPRINTF("APP_ScppcScanRefreshNtfCfgRspHandler status: 0x%x.\r\n", param->status);
    return (KE_MSG_CONSUMED);
}

int APP_ScppcScanRefreshIndHandler(ke_msg_id_t const msgid,
                                   void *param,
                                   ke_task_id_t const dest_id,
                                   ke_task_id_t const src_id)
{
    QPRINTF("APP_ScppcScanRefreshIndHandler.\r\n");
    return (KE_MSG_CONSUMED);
}

#endif

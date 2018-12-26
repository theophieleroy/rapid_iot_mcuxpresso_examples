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
#if BLE_TIP_CLIENT

/*******************************************************************************
 * Variables
 ******************************************************************************/
const struct ke_msg_handler s_appTipcMsgHandlerList[] = {
    {TIPC_ENABLE_RSP, (ke_msg_func_t)APP_TipcEnableRspHandler},
    {TIPC_RD_CHAR_RSP, (ke_msg_func_t)APP_TipcRdCharRspHandler},
    {TIPC_CT_NTF_CFG_RSP, (ke_msg_func_t)APP_TipcCtNtfCfgRspHandler},
    {TIPC_WR_TIME_UPD_CTNL_PT_RSP, (ke_msg_func_t)APP_TipcWrTimeUpdCtnlPtRspHandler},
    {TIPC_CT_IND, (ke_msg_func_t)APP_TipcCtIndHandler},
};

const struct ke_state_handler g_AppTipcTableHandler = KE_STATE_HANDLER(s_appTipcMsgHandlerList);

/*******************************************************************************
 * Code
 ******************************************************************************/
void APP_TipcAddProfileTask(void)
{
    struct gapm_profile_task_add_cmd *req =
        KE_MSG_ALLOC_DYN(GAPM_PROFILE_TASK_ADD_CMD, TASK_GAPM, TASK_APP, gapm_profile_task_add_cmd, 0);
    /* Fill message */
    req->operation = GAPM_PROFILE_TASK_ADD;
    req->sec_lvl = 0;
    req->prf_task_id = TASK_ID_TIPC;
    req->app_task = TASK_APP;
    req->start_hdl = 0;

    /* Send the message */
    APP_MsgSend(req);
}

void APP_TipcEnableReq(uint16_t conhdl)
{
    struct tipc_enable_req *msg =
        KE_MSG_ALLOC(TIPC_ENABLE_REQ, KE_BUILD_ID(prf_get_task_from_id(TASK_ID_TIPC), CONHDL2CONIDX(conhdl)), TASK_APP,
                     tipc_enable_req);

    msg->con_type = PRF_CON_DISCOVERY;

    /* Send the message */
    APP_MsgSend(msg);
}

int APP_TipcEnableRspHandler(ke_msg_id_t const msgid,
                             struct tipc_enable_rsp *param,
                             ke_task_id_t const dest_id,
                             ke_task_id_t const src_id)
{
    QPRINTF("APP_TipcEnableRspHandler status: 0x%x.\r\n", param->status);
    return (KE_MSG_CONSUMED);
}

void APP_TipcRdCharReq(uint16_t conhdl, uint8_t char_code)
{
    struct tipc_rd_char_req *msg =
        KE_MSG_ALLOC(TIPC_RD_CHAR_REQ, KE_BUILD_ID(prf_get_task_from_id(TASK_ID_TIPC), CONHDL2CONIDX(conhdl)), TASK_APP,
                     tipc_rd_char_req);

    msg->char_code = char_code;

    /* Send the message */
    APP_MsgSend(msg);
}

int APP_TipcRdCharRspHandler(ke_msg_id_t const msgid,
                             struct tipc_rd_char_rsp *param,
                             ke_task_id_t const dest_id,
                             ke_task_id_t const src_id)
{
    QPRINTF("APP_TipcRdCharRspHandler status: 0x%x.\r\n", param->status);

    return (KE_MSG_CONSUMED);
}

void APP_TipcCtNtfCfgReq(uint16_t conhdl, uint16_t cfg_val)
{
    struct tipc_ct_ntf_cfg_req *msg =
        KE_MSG_ALLOC(TIPC_CT_NTF_CFG_REQ, KE_BUILD_ID(prf_get_task_from_id(TASK_ID_TIPC), CONHDL2CONIDX(conhdl)),
                     TASK_APP, tipc_ct_ntf_cfg_req);

    msg->cfg_val = cfg_val;

    /* Send the message */
    APP_MsgSend(msg);
}

int APP_TipcCtNtfCfgRspHandler(ke_msg_id_t const msgid,
                               struct tipc_ct_ntf_cfg_rsp *param,
                               ke_task_id_t const dest_id,
                               ke_task_id_t const src_id)
{
    QPRINTF("APP_TipcCtNtfCfgRspHandler status: 0x%x.\r\n", param->status);
    return (KE_MSG_CONSUMED);
}

void APP_TipcWrTimeUpdCtnlPtReq(uint16_t conhdl, uint8_t value)
{
    struct tipc_wr_time_udp_ctnl_pt_req *msg = KE_MSG_ALLOC(
        TIPC_WR_TIME_UPD_CTNL_PT_REQ, KE_BUILD_ID(prf_get_task_from_id(TASK_ID_TIPC), CONHDL2CONIDX(conhdl)), TASK_APP,
        tipc_wr_time_udp_ctnl_pt_req);

    msg->value = value;

    /* Send the message */
    APP_MsgSend(msg);
}

int APP_TipcWrTimeUpdCtnlPtRspHandler(ke_msg_id_t const msgid,
                                      struct tipc_wr_time_upd_ctnl_pt_rsp *param,
                                      ke_task_id_t const dest_id,
                                      ke_task_id_t const src_id)
{
    QPRINTF("APP_TipcWrTimeUpdCtnlPtRspHandler status: 0x%x.\r\n", param->status);
    return (KE_MSG_CONSUMED);
}

int APP_TipcCtIndHandler(ke_msg_id_t const msgid,
                         struct tipc_ct_ind *param,
                         ke_task_id_t const dest_id,
                         ke_task_id_t const src_id)
{
    QPRINTF("APP_TipcCtIndHandler receive time data. \r\n");
    return (KE_MSG_CONSUMED);
}

#endif

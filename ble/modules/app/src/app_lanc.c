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
#if BLE_LN_COLLECTOR

/*******************************************************************************
 * Variables
 ******************************************************************************/
const struct ke_msg_handler s_appLancMsgHandlerList[] = {
    {LANC_ENABLE_RSP, (ke_msg_func_t)APP_LancEnableRspHandler},
    {LANC_VALUE_IND, (ke_msg_func_t)APP_LancValueIndHandler},
    {LANC_LN_CTNL_PT_RSP, (ke_msg_func_t)APP_LancLnCtnlPtRspHandler},
    {LANC_CMP_EVT, (ke_msg_func_t)APP_LancCmpEvtHandler},
};

const struct ke_state_handler g_AppLancTableHandler = KE_STATE_HANDLER(s_appLancMsgHandlerList);

/*******************************************************************************
 * Code
 ******************************************************************************/
void APP_LancAddProfileTask(void)
{
    struct gapm_profile_task_add_cmd *req =
        KE_MSG_ALLOC_DYN(GAPM_PROFILE_TASK_ADD_CMD, TASK_GAPM, TASK_APP, gapm_profile_task_add_cmd, 0);
    // Fill message
    req->operation = GAPM_PROFILE_TASK_ADD;
    req->sec_lvl = 0;
    req->prf_task_id = TASK_ID_LANC;
    req->app_task = TASK_APP;
    req->start_hdl = 0;

    // Send the message
    APP_MsgSend(req);
}

void APP_LancEnableReq(uint16_t conhdl)
{
    struct lanc_enable_req *msg =
        KE_MSG_ALLOC(LANC_ENABLE_REQ, KE_BUILD_ID(prf_get_task_from_id(TASK_ID_LANC), CONHDL2CONIDX(conhdl)), TASK_APP,
                     lanc_enable_req);

    msg->con_type = PRF_CON_DISCOVERY;

    // Send the message
    APP_MsgSend(msg);
}

int APP_LancEnableRspHandler(ke_msg_id_t const msgid,
                             struct lanc_enable_rsp *param,
                             ke_task_id_t const dest_id,
                             ke_task_id_t const src_id)
{
    QPRINTF("APP_LancEnableRspHandler status: 0x%x.\r\n", param->status);
    return (KE_MSG_CONSUMED);
}

void APP_LancReadCmd(uint16_t conhdl, uint8_t operation, uint8_t read_code)
{
    struct lanc_read_cmd *msg = KE_MSG_ALLOC(
        LANC_READ_CMD, KE_BUILD_ID(prf_get_task_from_id(TASK_ID_LANC), CONHDL2CONIDX(conhdl)), TASK_APP, lanc_read_cmd);

    msg->operation = operation;
    msg->read_code = read_code;

    // Send the message
    APP_MsgSend(msg);
}

int APP_LancValueIndHandler(ke_msg_id_t const msgid,
                            struct lanc_value_ind *param,
                            ke_task_id_t const dest_id,
                            ke_task_id_t const src_id)
{
    QPRINTF("APP_LancValueIndHandler att_code: 0x%x.\r\n", param->att_code);
    return (KE_MSG_CONSUMED);
}

void APP_LancCfgNtfindCmd(uint16_t conhdl, uint8_t operation, uint8_t desc_code, uint16_t ntfind_cfg)
{
    struct lanc_cfg_ntfind_cmd *msg =
        KE_MSG_ALLOC(LANC_CFG_NTFIND_CMD, KE_BUILD_ID(prf_get_task_from_id(TASK_ID_LANC), CONHDL2CONIDX(conhdl)),
                     TASK_APP, lanc_cfg_ntfind_cmd);

    msg->operation = operation;
    msg->desc_code = desc_code;
    msg->ntfind_cfg = ntfind_cfg;

    // Send the message
    APP_MsgSend(msg);
}

void APP_LancLnCtnlPtCfgReq(uint16_t conhdl, uint8_t operation, struct lan_ln_ctnl_pt_req *ln_ctnl_pt)
{
    struct lanc_ln_ctnl_pt_cfg_req *msg =
        KE_MSG_ALLOC(LANC_LN_CTNL_PT_CFG_REQ, KE_BUILD_ID(prf_get_task_from_id(TASK_ID_LANC), CONHDL2CONIDX(conhdl)),
                     TASK_APP, lanc_ln_ctnl_pt_cfg_req);

    msg->operation = operation;
    msg->ln_ctnl_pt = *ln_ctnl_pt;

    // Send the message
    APP_MsgSend(msg);
}

int APP_LancLnCtnlPtRspHandler(ke_msg_id_t const msgid,
                               struct lanc_ln_ctnl_pt_rsp *param,
                               ke_task_id_t const dest_id,
                               ke_task_id_t const src_id)
{
    QPRINTF("APP_LancLnCtnlPtRspHandler.\r\n");
    return (KE_MSG_CONSUMED);
}

int APP_LancCmpEvtHandler(ke_msg_id_t const msgid,
                          struct lanc_cmp_evt *param,
                          ke_task_id_t const dest_id,
                          ke_task_id_t const src_id)
{
    QPRINTF("APP_LancCmpEvtHandler status : 0x%x\r\n", param->status);
    return (KE_MSG_CONSUMED);
}

#endif

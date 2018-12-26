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
#if BLE_CP_COLLECTOR

/*******************************************************************************
 * Variables
 ******************************************************************************/
const struct ke_msg_handler s_appCppcMsgHandlerList[] = {
    {CPPC_ENABLE_RSP, (ke_msg_func_t)APP_CppcEnableRspHandler},
    {CPPC_CTNL_PT_RSP, (ke_msg_func_t)APP_CppcCtnlPtRspHandler},
    {CPPC_VALUE_IND, (ke_msg_func_t)APP_CppcValueIndHandler},
    {CPPC_CMP_EVT, (ke_msg_func_t)APP_CppcCmpEvtHandler},
    {CPPC_TIMEOUT_TIMER_IND, (ke_msg_func_t)APP_CppcTimeoutTimerHandler},
};

const struct ke_state_handler g_AppCppcTableHandler = KE_STATE_HANDLER(s_appCppcMsgHandlerList);

/*******************************************************************************
 * Code
 ******************************************************************************/
void APP_CppcAddProfileTask(void)
{
    struct gapm_profile_task_add_cmd *req =
        KE_MSG_ALLOC_DYN(GAPM_PROFILE_TASK_ADD_CMD, TASK_GAPM, TASK_APP, gapm_profile_task_add_cmd, 0);
    /* Fill message */
    req->operation = GAPM_PROFILE_TASK_ADD;
    req->sec_lvl = 0;
    req->prf_task_id = TASK_ID_CPPC;
    req->app_task = TASK_APP;
    req->start_hdl = 0;

    /* Send the message */
    APP_MsgSend(req);
}

void APP_CppcEnableReq(uint16_t conhdl)
{
    struct cppc_enable_req *msg =
        KE_MSG_ALLOC(CPPC_ENABLE_REQ, KE_BUILD_ID(prf_get_task_from_id(TASK_ID_CPPC), CONHDL2CONIDX(conhdl)), TASK_APP,
                     cppc_enable_req);

    msg->con_type = PRF_CON_DISCOVERY;

    /* Send the message */
    APP_MsgSend(msg);
}

int APP_CppcEnableRspHandler(ke_msg_id_t const msgid,
                             struct cppc_enable_rsp *param,
                             ke_task_id_t const dest_id,
                             ke_task_id_t const src_id)
{
    QPRINTF("APP_CppcEnableRspHandler status: 0x%x.\r\n", param->status);
    return (KE_MSG_CONSUMED);
}

void APP_CppcReadCmd(uint16_t conhdl, uint8_t operation, uint8_t read_code)
{
    struct cppc_read_cmd *msg = KE_MSG_ALLOC(
        CPPC_READ_CMD, KE_BUILD_ID(prf_get_task_from_id(TASK_ID_CPPC), CONHDL2CONIDX(conhdl)), TASK_APP, cppc_read_cmd);

    msg->operation = operation;
    msg->read_code = read_code;

    /* Send the message */
    APP_MsgSend(msg);
}

void APP_CppcCfgNtfindCmd(uint16_t conhdl, uint8_t operation, uint8_t desc_code, uint16_t ntfind_cfg)
{
    struct cppc_cfg_ntfind_cmd *msg =
        KE_MSG_ALLOC(CPPC_CFG_NTFIND_CMD, KE_BUILD_ID(prf_get_task_from_id(TASK_ID_CPPC), CONHDL2CONIDX(conhdl)),
                     TASK_APP, cppc_cfg_ntfind_cmd);

    msg->operation = operation;
    msg->desc_code = desc_code;
    msg->ntfind_cfg = ntfind_cfg;

    /* Send the message */
    APP_MsgSend(msg);
}

void APP_CppcCtnlPtCfgReq(uint16_t conhdl, uint8_t operation, struct cpp_ctnl_pt_req ctnl_pt)
{
    struct cppc_ctnl_pt_cfg_req *msg =
        KE_MSG_ALLOC(CPPC_CTNL_PT_CFG_REQ, KE_BUILD_ID(prf_get_task_from_id(TASK_ID_CPPC), CONHDL2CONIDX(conhdl)),
                     TASK_APP, cppc_ctnl_pt_cfg_req);

    msg->operation = operation;
    msg->ctnl_pt = ctnl_pt;

    /* Send the message */
    APP_MsgSend(msg);
}

int APP_CppcCtnlPtRspHandler(ke_msg_id_t const msgid,
                             struct cppc_ctnl_pt_rsp *param,
                             ke_task_id_t const dest_id,
                             ke_task_id_t const src_id)
{
    QPRINTF("APP_CppcCtnlPtRspHandler op_code: 0x%x.\r\n", param->rsp.req_op_code);
    return (KE_MSG_CONSUMED);
}

int APP_CppcValueIndHandler(ke_msg_id_t const msgid,
                            struct cppc_value_ind *param,
                            ke_task_id_t const dest_id,
                            ke_task_id_t const src_id)
{
    QPRINTF("APP_CppcValueIndHandler att_code is: 0x%x.\r\n", param->att_code);
    return (KE_MSG_CONSUMED);
}

int APP_CppcCmpEvtHandler(ke_msg_id_t const msgid,
                          struct cppc_cmp_evt *param,
                          ke_task_id_t const dest_id,
                          ke_task_id_t const src_id)
{
    QPRINTF("APP_CppcCmpEvtHandler status is: 0x%x.\r\n", param->status);

    switch (param->operation)
    {
        case CPPC_READ_OP_CODE:
            QPRINTF("APP_CppcCmpEvtHandler operation is:CPPC_READ_OP_CODE.\r\n");
            break;

        case CPPC_CFG_NTF_IND_OP_CODE:
            QPRINTF("APP_CppcCmpEvtHandler operation is:CPPC_CFG_NTF_IND_OP_CODE.\r\n");
            break;

        case CPPC_CTNL_PT_CFG_WR_OP_CODE:
            QPRINTF("APP_CppcCmpEvtHandler operation is:CPPC_CTNL_PT_CFG_WR_OP_CODE.\r\n");
            break;

        case CPPC_CTNL_PT_CFG_IND_OP_CODE:
            QPRINTF("APP_CppcCmpEvtHandler operation is:CPPC_CTNL_PT_CFG_IND_OP_CODE.\r\n");
            break;

        default:
            break;
    }

    return (KE_MSG_CONSUMED);
}

int APP_CppcTimeoutTimerHandler(ke_msg_id_t const msgid,
                                void const *param,
                                ke_task_id_t const dest_id,
                                ke_task_id_t const src_id)
{
    QPRINTF("APP_CppcTimeoutTimerHandler .\r\n");
    return (KE_MSG_CONSUMED);
}

#endif

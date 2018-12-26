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
#if BLE_CSC_COLLECTOR

/*******************************************************************************
 * Variables
 ******************************************************************************/

const struct ke_msg_handler s_appCscpcMsgHandlerList[] = {
    {CSCPC_ENABLE_RSP, (ke_msg_func_t)APP_CscpcEnableRspHandler},
    {CSCPC_CTNL_PT_RSP, (ke_msg_func_t)APP_CscpcCtnlPtRspHandler},
    {CSCPC_VALUE_IND, (ke_msg_func_t)APP_CscpcValueIndHandler},
    {CSCPC_CMP_EVT, (ke_msg_func_t)APP_CscpcCmpEvtHandler},
};

const struct ke_state_handler g_AppCscpcTableHandler = KE_STATE_HANDLER(s_appCscpcMsgHandlerList);

/*******************************************************************************
 * Code
 ******************************************************************************/

void APP_CscpcAddProfileTask(void)
{
    struct gapm_profile_task_add_cmd *req =
        KE_MSG_ALLOC_DYN(GAPM_PROFILE_TASK_ADD_CMD, TASK_GAPM, TASK_APP, gapm_profile_task_add_cmd, 0);
    // Fill message
    req->operation = GAPM_PROFILE_TASK_ADD;
    req->prf_task_id = TASK_ID_CSCPC;
    req->app_task = TASK_APP;
    req->start_hdl = 0;

    // Send the message
    APP_MsgSend(req);
}

void APP_CscpcEnableReq(uint16_t conhdl)
{
    struct cscpc_enable_req *msg =
        KE_MSG_ALLOC(CSCPC_ENABLE_REQ, KE_BUILD_ID(prf_get_task_from_id(TASK_ID_CSCPC), CONHDL2CONIDX(conhdl)),
                     TASK_APP, cscpc_enable_req);

    msg->con_type = PRF_CON_DISCOVERY;

    // Send the message
    APP_MsgSend(msg);
}

int APP_CscpcEnableRspHandler(ke_msg_id_t const msgid,
                              struct cscpc_enable_rsp *param,
                              ke_task_id_t const dest_id,
                              ke_task_id_t const src_id)
{
    QPRINTF("cscpc enable rsp status: 0x%x.\r\n", param->status);
    return (KE_MSG_CONSUMED);
}

void APP_CscpcReadCmd(uint16_t conhdl, uint8_t operation, uint8_t read_code)
{
    struct cscpc_read_cmd *msg =
        KE_MSG_ALLOC(CSCPC_READ_CMD, KE_BUILD_ID(prf_get_task_from_id(TASK_ID_CSCPC), CONHDL2CONIDX(conhdl)), TASK_APP,
                     cscpc_read_cmd);

    msg->operation = operation;
    msg->read_code = read_code;

    // Send the message
    APP_MsgSend(msg);
}

void APP_CscpcCfgNtfindCmd(uint16_t conhdl, uint8_t operation, uint8_t desc_code, uint16_t ntfind_cfg)
{
    struct cscpc_cfg_ntfind_cmd *msg =
        KE_MSG_ALLOC(CSCPC_CFG_NTFIND_CMD, KE_BUILD_ID(prf_get_task_from_id(TASK_ID_CSCPC), CONHDL2CONIDX(conhdl)),
                     TASK_APP, cscpc_cfg_ntfind_cmd);

    msg->operation = operation;
    msg->desc_code = desc_code;
    msg->ntfind_cfg = ntfind_cfg;

    // Send the message
    APP_MsgSend(msg);
}

void APP_CscpcCtnlPtCfgReq(uint16_t conhdl, uint8_t operation, struct cscp_sc_ctnl_pt_req sc_ctnl_pt)
{
    struct cscpc_ctnl_pt_cfg_req *msg =
        KE_MSG_ALLOC(CSCPC_CTNL_PT_CFG_REQ, KE_BUILD_ID(prf_get_task_from_id(TASK_ID_CSCPC), CONHDL2CONIDX(conhdl)),
                     TASK_APP, cscpc_ctnl_pt_cfg_req);

    msg->operation = operation;
    msg->sc_ctnl_pt = sc_ctnl_pt;

    // Send the message
    APP_MsgSend(msg);
}

int APP_CscpcCtnlPtRspHandler(ke_msg_id_t const msgid,
                              struct cscpc_ctnl_pt_rsp *param,
                              ke_task_id_t const dest_id,
                              ke_task_id_t const src_id)
{
    QPRINTF("app_cscpc_ctnl_pt_rsp_handler op_code = 0x%x, resp_value = 0x%x, supp_loc = 0x%x.\r\n",
            param->ctnl_pt_rsp.req_op_code, param->ctnl_pt_rsp.resp_value, param->ctnl_pt_rsp.supp_loc);

    return (KE_MSG_CONSUMED);
}

int APP_CscpcValueIndHandler(ke_msg_id_t const msgid,
                             struct cscpc_value_ind *param,
                             ke_task_id_t const dest_id,
                             ke_task_id_t const src_id)
{
    switch (param->att_code)
    {
        case CSCPC_NTF_CSC_MEAS:
            QPRINTF(
                "CSCPC Received measurement, flags = 0x%x,cumul_crank_rev = 0x%x ,last_crank_evt_time = "
                "0x%x,last_wheel_evt_time = 0x%x,cumul_wheel_rev = 0x%x.\r\n",
                param->value.csc_meas.flags, param->value.csc_meas.cumul_crank_rev,
                param->value.csc_meas.last_crank_evt_time, param->value.csc_meas.last_wheel_evt_time,
                param->value.csc_meas.cumul_wheel_rev);
            break;
        case CSCPC_RD_CSC_FEAT:
            QPRINTF("CSCPC read sensor feature = 0x%x.\r\n", param->value.sensor_feat);
            break;
        case CSCPC_RD_SENSOR_LOC:
            QPRINTF("CSCPC read sensor location = 0x%x.\r\n", param->value.sensor_loc);
            break;
        case CSCPC_RD_WR_CSC_MEAS_CFG:
            QPRINTF("CSCPC measurement Client Characteristic Configuration Descriptor Value=0x%x.\r\n",
                    param->value.ntf_cfg);
            break;
        case CSCPC_RD_WR_SC_CTNL_PT_CFG:
            QPRINTF("CSCPC control point Client Characteristic Configuration Descriptor Value=0x%x.\r\n",
                    param->value.ntf_cfg);
            break;
    }
    return (KE_MSG_CONSUMED);
}

int APP_CscpcCmpEvtHandler(ke_msg_id_t const msgid,
                           struct cscpc_cmp_evt *param,
                           ke_task_id_t const dest_id,
                           ke_task_id_t const src_id)
{
    switch (param->operation)
    {
        case CSCPC_READ_OP_CODE:
            QPRINTF("app_cscpc_cmp_evt_handler operation is:CPPC_READ_OP_CODE.\r\n");
            break;

        case CSCPC_CFG_NTF_IND_OP_CODE:
            QPRINTF("app_cscpc_cmp_evt_handler operation is:CPPC_CFG_NTF_IND_OP_CODE.\r\n");
            break;

        case CSCPC_CTNL_PT_CFG_WR_OP_CODE:
            QPRINTF("app_cscpc_cmp_evt_handler operation is:CPPC_CTNL_PT_CFG_WR_OP_CODE.\r\n");
            break;

        case CSCPC_CTNL_PT_CFG_IND_OP_CODE:
            QPRINTF("app_cscpc_cmp_evt_handler operation is:CPPC_CTNL_PT_CFG_IND_OP_CODE.\r\n");
            break;

        default:
            break;
    }
    QPRINTF("app_cscpc_cmp_evt_handler status is: 0x%x.\r\n", param->status);

    return (KE_MSG_CONSUMED);
}

#endif

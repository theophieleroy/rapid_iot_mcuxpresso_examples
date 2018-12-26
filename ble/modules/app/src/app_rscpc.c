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
#if BLE_RSC_COLLECTOR

/*******************************************************************************
 * Variables
 ******************************************************************************/

const struct ke_msg_handler s_appRscpcMsgHandlerList[] = {
    {RSCPC_ENABLE_RSP, (ke_msg_func_t)APP_RscpcEnableRspHandler},
    {RSCPC_CTNL_PT_CFG_RSP, (ke_msg_func_t)APP_RscpcCtnlPtRspHandler},
    {RSCPC_VALUE_IND, (ke_msg_func_t)APP_RscpcValueIndHandler},
    {RSCPC_CMP_EVT, (ke_msg_func_t)APP_RscpcCmpEvtHandler},
};

const struct ke_state_handler g_AppRscpcTableHandler = KE_STATE_HANDLER(s_appRscpcMsgHandlerList);

/*******************************************************************************
 * Code
 ******************************************************************************/

void APP_RscpcAddProfileTask(void)
{
    struct gapm_profile_task_add_cmd *req =
        KE_MSG_ALLOC_DYN(GAPM_PROFILE_TASK_ADD_CMD, TASK_GAPM, TASK_APP, gapm_profile_task_add_cmd, 0);
    // Fill message
    req->operation = GAPM_PROFILE_TASK_ADD;
    req->prf_task_id = TASK_ID_RSCPC;
    req->app_task = TASK_APP;
    req->start_hdl = 0;

    // Send the message
    APP_MsgSend(req);
}

void APP_RscpcEnableReq(uint16_t conhdl)
{
    struct rscpc_enable_req *msg =
        KE_MSG_ALLOC(RSCPC_ENABLE_REQ, KE_BUILD_ID(prf_get_task_from_id(TASK_ID_RSCPC), CONHDL2CONIDX(conhdl)),
                     TASK_APP, rscpc_enable_req);

    msg->con_type = PRF_CON_DISCOVERY;

    // Send the message
    APP_MsgSend(msg);
}

int APP_RscpcEnableRspHandler(ke_msg_id_t const msgid,
                              struct rscpc_enable_rsp *param,
                              ke_task_id_t const dest_id,
                              ke_task_id_t const src_id)
{
    QPRINTF("rscpc enable rsp status: 0x%x.\r\n", param->status);
    return (KE_MSG_CONSUMED);
}

void APP_RscpcReadCmd(uint16_t conhdl, uint8_t operation, uint8_t read_code)
{
    struct rscpc_read_cmd *msg =
        KE_MSG_ALLOC(RSCPC_READ_CMD, KE_BUILD_ID(prf_get_task_from_id(TASK_ID_RSCPC), CONHDL2CONIDX(conhdl)), TASK_APP,
                     rscpc_read_cmd);

    msg->operation = operation;
    msg->read_code = read_code;

    // Send the message
    APP_MsgSend(msg);
}

void APP_RscpcCfgNtfindCmd(uint16_t conhdl, uint8_t operation, uint8_t desc_code, uint16_t ntfind_cfg)
{
    struct rscpc_cfg_ntfind_cmd *msg =
        KE_MSG_ALLOC(RSCPC_CFG_NTFIND_CMD, KE_BUILD_ID(prf_get_task_from_id(TASK_ID_RSCPC), CONHDL2CONIDX(conhdl)),
                     TASK_APP, rscpc_cfg_ntfind_cmd);

    msg->operation = operation;
    msg->desc_code = desc_code;
    msg->ntfind_cfg = ntfind_cfg;

    // Send the message
    APP_MsgSend(msg);
}

void APP_RscpcCtnlPtCfgReq(uint16_t conhdl, uint8_t operation, struct rscp_sc_ctnl_pt_req sc_ctnl_pt)
{
    struct rscpc_ctnl_pt_cfg_req *msg =
        KE_MSG_ALLOC(RSCPC_CTNL_PT_CFG_REQ, KE_BUILD_ID(prf_get_task_from_id(TASK_ID_RSCPC), CONHDL2CONIDX(conhdl)),
                     TASK_APP, rscpc_ctnl_pt_cfg_req);

    msg->operation = operation;
    msg->sc_ctnl_pt = sc_ctnl_pt;

    // Send the message
    APP_MsgSend(msg);
}

int APP_RscpcCtnlPtRspHandler(ke_msg_id_t const msgid,
                              struct rscpc_ctnl_pt_cfg_rsp *param,
                              ke_task_id_t const dest_id,
                              ke_task_id_t const src_id)
{
    QPRINTF("app_rscpc_ctnl_pt_rsp_handler op_code = 0x%x, resp_value = 0x%x, supp_loc = 0x%x.\r\n",
            param->ctnl_pt_rsp.req_op_code, param->ctnl_pt_rsp.resp_value, param->ctnl_pt_rsp.supp_loc);

    return (KE_MSG_CONSUMED);
}

int APP_RscpcValueIndHandler(ke_msg_id_t const msgid,
                             struct rscpc_value_ind *param,
                             ke_task_id_t const dest_id,
                             ke_task_id_t const src_id)
{
    switch (param->att_code)
    {
        case RSCPC_NTF_RSC_MEAS:
            QPRINTF(
                "RSCPC Received measurement, flags = 0x%x,inst_cad = 0x%x ,inst_speed = 0x%x,inst_stride_len = "
                "0x%x,total_dist = 0x%x.\r\n",
                param->value.rsc_meas.flags, param->value.rsc_meas.inst_cad, param->value.rsc_meas.inst_speed,
                param->value.rsc_meas.inst_stride_len, param->value.rsc_meas.total_dist);
            break;
        case RSCPC_RD_RSC_FEAT:
            QPRINTF("RSCPC read sensor feature = 0x%x.\r\n", param->value.sensor_feat);
            break;
        case RSCPC_RD_SENSOR_LOC:
            QPRINTF("RSCPC read sensor location = 0x%x.\r\n", param->value.sensor_loc);
            break;
        case RSCPC_RD_WR_RSC_MEAS_CFG:
            QPRINTF("RSCPC measurement Client Characteristic Configuration Descriptor Value=0x%x.\r\n",
                    param->value.ntf_cfg);
            break;
        case RSCPC_RD_WR_SC_CTNL_PT_CFG:
            QPRINTF("RSCPC control point Client Characteristic Configuration Descriptor Value=0x%x.\r\n",
                    param->value.ntf_cfg);
            break;
    }

    return (KE_MSG_CONSUMED);
}

int APP_RscpcCmpEvtHandler(ke_msg_id_t const msgid,
                           struct rscpc_cmp_evt *param,
                           ke_task_id_t const dest_id,
                           ke_task_id_t const src_id)
{
    switch (param->operation)
    {
        case RSCPC_READ_OP_CODE:
            QPRINTF("app_rscpc_cmp_evt_handler operation is:RSCPC_READ_OP_CODE.\r\n");
            break;

        case RSCPC_CFG_NTF_IND_OP_CODE:
            QPRINTF("app_rscpc_cmp_evt_handler operation is:RSCPC_CFG_NTF_IND_OP_CODE.\r\n");
            break;

        case RSCPC_CTNL_PT_CFG_WR_OP_CODE:
            QPRINTF("app_rscpc_cmp_evt_handler operation is:RSCPC_CTNL_PT_CFG_WR_OP_CODE.\r\n");
            break;

        case RSCPC_CTNL_PT_CFG_IND_OP_CODE:
            QPRINTF("app_rcscpc_cmp_evt_handler operation is:RSCPC_CTNL_PT_CFG_IND_OP_CODE.\r\n");
            break;

        default:
            break;
    }
    QPRINTF("app_rscpc_cmp_evt_handler status is: 0x%x.\r\n", param->status);

    return (KE_MSG_CONSUMED);
}

#endif

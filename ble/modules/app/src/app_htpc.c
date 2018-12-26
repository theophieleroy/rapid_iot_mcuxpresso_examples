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
#if BLE_HT_COLLECTOR

/*******************************************************************************
 * Variables
 ******************************************************************************/
const struct ke_msg_handler s_appHtpcMsgHandlerList[] = {
    {HTPC_ENABLE_RSP, (ke_msg_func_t)APP_HtpcEnableRspHandler},
    {HTPC_RD_CHAR_RSP, (ke_msg_func_t)APP_HtpcRdCharRspHandler},
    {HTPC_HEALTH_TEMP_NTF_CFG_RSP, (ke_msg_func_t)APP_HtpcHealthTempNtfCfgRspHandler},
    {HTPC_WR_MEAS_INTV_RSP, (ke_msg_func_t)APP_HtpcWrMeasIntvRspHandler},
    {HTPC_TEMP_IND, (ke_msg_func_t)APP_HtpcTempIndHandler},
    {HTPC_MEAS_INTV_IND, (ke_msg_func_t)APP_HtpcMeasIntvIndHandler},
};

const struct ke_state_handler g_AppHtpcTableHandler = KE_STATE_HANDLER(s_appHtpcMsgHandlerList);

/*******************************************************************************
 * Code
 ******************************************************************************/
void APP_HtpcAddProfileTask(void)
{
    struct gapm_profile_task_add_cmd *req =
        KE_MSG_ALLOC_DYN(GAPM_PROFILE_TASK_ADD_CMD, TASK_GAPM, TASK_APP, gapm_profile_task_add_cmd, 0);
    /* Fill message */
    req->operation = GAPM_PROFILE_TASK_ADD;
    req->sec_lvl = 0;
    req->prf_task_id = TASK_ID_HTPC;
    req->app_task = TASK_APP;
    req->start_hdl = 0;

    /* Send the message */
    APP_MsgSend(req);
}

void APP_HtpcEnableReq(uint16_t conhdl)
{
    struct htpc_enable_req *msg =
        KE_MSG_ALLOC(HTPC_ENABLE_REQ, KE_BUILD_ID(prf_get_task_from_id(TASK_ID_HTPC), CONHDL2CONIDX(conhdl)), TASK_APP,
                     htpc_enable_req);

    msg->con_type = PRF_CON_DISCOVERY;

    /* Send the message */
    APP_MsgSend(msg);
}

int APP_HtpcEnableRspHandler(ke_msg_id_t const msgid,
                             struct htpc_enable_rsp *param,
                             ke_task_id_t const dest_id,
                             ke_task_id_t const src_id)
{
    QPRINTF("APP_HtpcEnableRspHandler status: 0x%x.\r\n", param->status);
    return (KE_MSG_CONSUMED);
}

void APP_HtpcHealthTempNtfCfgReq(uint16_t conhdl, uint16_t cfg_val, uint8_t char_code)
{
    struct htpc_health_temp_ntf_cfg_req *msg = KE_MSG_ALLOC(
        HTPC_HEALTH_TEMP_NTF_CFG_REQ, KE_BUILD_ID(prf_get_task_from_id(TASK_ID_HTPC), CONHDL2CONIDX(conhdl)), TASK_APP,
        htpc_health_temp_ntf_cfg_req);

    msg->cfg_val = cfg_val;
    msg->char_code = char_code;

    /* Send the message */
    APP_MsgSend(msg);
}

int APP_HtpcHealthTempNtfCfgRspHandler(ke_msg_id_t const msgid,
                                       struct htpc_health_temp_ntf_cfg_rsp *param,
                                       ke_task_id_t const dest_id,
                                       ke_task_id_t const src_id)
{
    QPRINTF("APP_HtpcHealthTempNtfCfgRspHandler status: 0x%x.\r\n", param->status);
    return (KE_MSG_CONSUMED);
}

void APP_HtpcWrMeasIntvReq(uint16_t conhdl, uint16_t intv)
{
    struct htpc_wr_meas_intv_req *msg =
        KE_MSG_ALLOC(HTPC_WR_MEAS_INTV_REQ, KE_BUILD_ID(prf_get_task_from_id(TASK_ID_HTPC), CONHDL2CONIDX(conhdl)),
                     TASK_APP, htpc_wr_meas_intv_req);

    msg->intv = intv;

    /* Send the message */
    APP_MsgSend(msg);
}

int APP_HtpcWrMeasIntvRspHandler(ke_msg_id_t const msgid,
                                 struct htpc_wr_meas_intv_rsp *param,
                                 ke_task_id_t const dest_id,
                                 ke_task_id_t const src_id)
{
    QPRINTF("APP_HtpcWrMeasIntvRspHandler status: 0x%x.\r\n", param->status);
    return (KE_MSG_CONSUMED);
}

void APP_HtpcRdCharReq(uint16_t conhdl, uint8_t char_code)
{
    struct htpc_rd_char_req *msg =
        KE_MSG_ALLOC(HTPC_RD_CHAR_REQ, KE_BUILD_ID(prf_get_task_from_id(TASK_ID_HTPC), CONHDL2CONIDX(conhdl)), TASK_APP,
                     htpc_rd_char_req);

    msg->char_code = char_code;

    /* Send the message */
    APP_MsgSend(msg);
}

int APP_HtpcRdCharRspHandler(ke_msg_id_t const msgid,
                             struct htpc_rd_char_rsp *param,
                             ke_task_id_t const dest_id,
                             ke_task_id_t const src_id)
{
    QPRINTF("APP_HtpcRdCharRspHandler status: 0x%x.\r\n", param->info.status);

    return (KE_MSG_CONSUMED);
}

int APP_HtpcTempIndHandler(ke_msg_id_t const msgid,
                           struct htpc_temp_ind *param,
                           ke_task_id_t const dest_id,
                           ke_task_id_t const src_id)
{
    QPRINTF("APP_HtpcTempIndHandler type : 0x%x value :%ld.\r\n", param->stable_meas, param->temp_meas.temp);
    return (KE_MSG_CONSUMED);
}

int APP_HtpcMeasIntvIndHandler(ke_msg_id_t const msgid,
                               struct htpc_meas_intv_ind *param,
                               ke_task_id_t const dest_id,
                               ke_task_id_t const src_id)
{
    QPRINTF("APP_HtpcMeasIntvIndHandler interval :%d\r\n", param->intv);

    return (KE_MSG_CONSUMED);
}

#endif

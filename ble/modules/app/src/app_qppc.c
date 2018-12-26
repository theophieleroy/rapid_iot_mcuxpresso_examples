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
#if BLE_QPP_CLIENT

/*******************************************************************************
 * Variables
 ******************************************************************************/
struct app_qppc_env_tag g_AppQppcEnv[BLE_CONNECTION_MAX];

const struct ke_msg_handler s_appQppcMsgHandlerList[] = {
    {QPPC_ENABLE_RSP, (ke_msg_func_t)APP_QppcEnableRspHandler},
    {QPPC_RECV_DATA_IND, (ke_msg_func_t)APP_QppcRecvDataIndHandler},
    {QPPC_SEND_DATA_RSP, (ke_msg_func_t)APP_QppcSendDataRspHandler},
    {QPPC_RD_CHAR_RSP, (ke_msg_func_t)APP_QppcRdCharRspHandler},
    {QPPC_CFG_INDNTF_RSP, (ke_msg_func_t)APP_QppcCfgIndntfRspHandler},
};

const struct ke_state_handler g_AppQppcTableHandler = KE_STATE_HANDLER(s_appQppcMsgHandlerList);

/*******************************************************************************
 * Code
 ******************************************************************************/
void APP_QppcAddProfileTask(void)
{
    struct gapm_profile_task_add_cmd *req =
        KE_MSG_ALLOC_DYN(GAPM_PROFILE_TASK_ADD_CMD, TASK_GAPM, TASK_APP, gapm_profile_task_add_cmd, 0);
    /* Fill message */
    req->operation = GAPM_PROFILE_TASK_ADD;
    req->sec_lvl = 0;
    req->prf_task_id = TASK_ID_QPPC;
    req->app_task = TASK_APP;
    req->start_hdl = 0;

    /* Send the message */
    APP_MsgSend(req);
}

void APP_QppcEnableReq(uint16_t conhdl)
{
    struct qppc_enable_req *msg =
        KE_MSG_ALLOC(QPPC_ENABLE_REQ, KE_BUILD_ID(prf_get_task_from_id(TASK_ID_QPPC), CONHDL2CONIDX(conhdl)), TASK_APP,
                     qppc_enable_req);
    /* Connection handle */
    msg->conhdl = conhdl;

    msg->con_type = PRF_CON_DISCOVERY;

    /* Send the message */
    APP_MsgSend(msg);
}

void APP_QppcRdCharReq(uint16_t conhdl, uint8_t char_code)
{
    struct qppc_rd_char_req *msg =
        KE_MSG_ALLOC(QPPC_RD_CHAR_REQ, KE_BUILD_ID(prf_get_task_from_id(TASK_ID_QPPC), CONHDL2CONIDX(conhdl)), TASK_APP,
                     qppc_rd_char_req);
    /* Connection handle */
    msg->conhdl = conhdl;
    /* Characteristic value code */
    msg->char_code = char_code;

    /* Send the message */
    APP_MsgSend(msg);
}

void APP_QppcCfgIndntfReq(uint16_t conhdl, uint16_t cfg_val)
{
    struct qppc_cfg_indntf_req *msg =
        KE_MSG_ALLOC(QPPC_CFG_INDNTF_REQ, KE_BUILD_ID(prf_get_task_from_id(TASK_ID_QPPC), CONHDL2CONIDX(conhdl)),
                     TASK_APP, qppc_cfg_indntf_req);

    /* Connection handle */
    msg->conhdl = conhdl;
    /* Stop/notify value to configure into the peer characteristic */
    msg->cfg_val = cfg_val;
    /* Char code */
    msg->char_code = QPPC_QPPS_TX_VALUE_CLI_CFG;

    /* Send the message */
    APP_MsgSend(msg);
}

void APP_QppcSendDataReq(uint16_t conhdl, uint16_t len, uint8_t *val)
{
    struct qppc_send_data_req *msg =
        KE_MSG_ALLOC_DYN(QPPC_SEND_DATA_REQ, KE_BUILD_ID(prf_get_task_from_id(TASK_ID_QPPC), CONHDL2CONIDX(conhdl)),
                         TASK_APP, qppc_send_data_req, len);

    /* Connection handle */
    msg->conhdl = conhdl;
    msg->length = len;
    memcpy(msg->data, val, msg->length);

    /* Send the message */
    APP_MsgSend(msg);
}

int APP_QppcEnableRspHandler(ke_msg_id_t const msgid,
                             struct qppc_enable_rsp *param,
                             ke_task_id_t const dest_id,
                             ke_task_id_t const src_id)
{
    QPRINTF("QPPC enable confirmation status: 0x%x.\r\n", param->status);

    if (param->status == CO_ERROR_NO_ERROR)
    {
        uint8_t idx = KE_IDX_GET(src_id);
        g_AppQppcEnv[idx].conhdl = param->conhdl;

        APP_QppcRdCharReq(g_AppQppcEnv[idx].conhdl, QPPC_QPPS_RX_CHAR_VALUE_USER_DESP);
    }

    return (KE_MSG_CONSUMED);
}

int APP_QppcRdCharRspHandler(ke_msg_id_t const msgid,
                             struct qppc_rd_char_rsp *param,
                             ke_task_id_t const dest_id,
                             ke_task_id_t const src_id)
{
    if (param->status == CO_ERROR_NO_ERROR)
        QPRINTF("QPP version %s\r\n", param->data);
    else
        QPRINTF("QPP read version fail error %d\r\n", param->status);

    return (KE_MSG_CONSUMED);
}

int APP_QppcSendDataRspHandler(ke_msg_id_t const msgid,
                               struct qppc_send_data_rsp *param,
                               ke_task_id_t const dest_id,
                               ke_task_id_t const src_id)
{
    APP_QppcSendDataRspCallback(param->conhdl, param->status);

    return (KE_MSG_CONSUMED);
}

int APP_QppcCfgIndntfRspHandler(ke_msg_id_t const msgid,
                                struct qppc_cfg_indntf_rsp *param,
                                ke_task_id_t const dest_id,
                                ke_task_id_t const src_id)
{
    QPRINTF("QPPC cfg ntf rsp 0x%02x\r\n", param->status);

    return (KE_MSG_CONSUMED);
}

int APP_QppcRecvDataIndHandler(ke_msg_id_t const msgid,
                               struct qppc_recv_data_ind *param,
                               ke_task_id_t const dest_id,
                               ke_task_id_t const src_id)
{
    uint16_t conhdl = CONIDX2CONHDL(KE_IDX_GET(src_id));

    APP_QppcReceivedDataCallback(conhdl, param->data, param->length);

    return (KE_MSG_CONSUMED);
}

#endif

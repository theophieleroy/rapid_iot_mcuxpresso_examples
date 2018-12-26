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
#if BLE_QPP_SERVER
#include "prf_utils.h"

/*******************************************************************************
 * Variables
 ******************************************************************************/
const struct ke_msg_handler s_appQppsMsgHandlerList[] = {
    {QPPS_RECV_DATA_IND, (ke_msg_func_t)APP_QppsRecvDataIndHandler},
    {QPPS_SEND_DATA_RSP, (ke_msg_func_t)APP_QppsSendDataRspHandler},
    {QPPS_CFG_INDNTF_IND, (ke_msg_func_t)APP_QppsCfgIndntfIndHandler},

};

const struct ke_state_handler g_AppQppsTableHandler = KE_STATE_HANDLER(s_appQppsMsgHandlerList);

/*******************************************************************************
 * Code
 ******************************************************************************/
void APP_QppsAddProfileTask(void)
{
    struct gapm_profile_task_add_cmd *req =
        KE_MSG_ALLOC_DYN(GAPM_PROFILE_TASK_ADD_CMD, TASK_GAPM, TASK_APP, gapm_profile_task_add_cmd, 0);
    /* Fill message */
    req->operation = GAPM_PROFILE_TASK_ADD;
    req->sec_lvl = 0; /*mono instantiated, no forced 16 bytes EKS, no auth */
    req->prf_task_id = TASK_ID_QPPS;
    req->app_task = TASK_APP;
    req->start_hdl = 0;

    /* Send the message */
    APP_MsgSend(req);
}

int APP_QppsSendDataReq(uint16_t conhdl, uint16_t length, uint8_t *data)
{
    int status = GAP_ERR_NO_ERROR;
    struct qpps_env_tag *qpps_env = PRF_ENV_GET(QPPS, qpps);

    if (qpps_env->ntf_cfg[conhdl] == QPPS_VALUE_NTF_ON)
    {
        struct qpps_send_data_req *msg = KE_MSG_ALLOC_DYN(QPPS_SEND_DATA_REQ, prf_get_task_from_id(TASK_ID_QPPS),
                                                          TASK_APP, qpps_send_data_req, length);

        msg->conhdl = conhdl;
        msg->length = length;
        memcpy(msg->data, data, length);

        APP_MsgSend(msg);
    }
    else
    {
        status = PRF_CCCD_IMPR_CONFIGURED;
    }

    return status;
}

int APP_QppsSendDataRspHandler(ke_msg_id_t const msgid,
                               struct qpps_send_data_rsp *param,
                               ke_task_id_t const dest_id,
                               ke_task_id_t const src_id)
{
    APP_QppsSendDataRspCallback(param->conhdl, param->status);

    return (KE_MSG_CONSUMED);
}

int APP_QppsCfgIndntfIndHandler(ke_msg_id_t const msgid,
                                struct qpps_cfg_indntf_ind *param,
                                ke_task_id_t const dest_id,
                                ke_task_id_t const src_id)
{
    APP_QppsCfgIndntfIndCallback(param->conhdl, param->cfg_val);

    return (KE_MSG_CONSUMED);
}

int APP_QppsRecvDataIndHandler(ke_msg_id_t const msgid,
                               struct qpps_recv_data_ind *param,
                               ke_task_id_t const dest_id,
                               ke_task_id_t const src_id)
{
    APP_QppsReceivedDataCallback(param->conhdl, param->data, param->length);

    return (KE_MSG_CONSUMED);
}

#endif

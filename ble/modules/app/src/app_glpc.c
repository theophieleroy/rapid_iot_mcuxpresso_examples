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
#if (BLE_GL_COLLECTOR)

/*******************************************************************************
 * Variables
 ******************************************************************************/
const struct ke_msg_handler s_appGlpcMsgHandlerList[] = {
    {GLPC_ENABLE_RSP, (ke_msg_func_t)APP_GlpcEnableRspHandler},
    {GLPC_REGISTER_RSP, (ke_msg_func_t)APP_GlpcRegisterRspHandler},
    {GLPC_READ_FEATURES_RSP, (ke_msg_func_t)APP_GlpcReadFeatureRspHandler},
    {GLPC_RACP_RSP, (ke_msg_func_t)APP_GlpcRacpRspHandler},
    {GLPC_MEAS_IND, (ke_msg_func_t)APP_GlpcMeasIndHandler},
    {GLPC_MEAS_CTX_IND, (ke_msg_func_t)APP_GlpcMeasCtxIndHandler},
    {GLPC_RACP_REQ_TIMEOUT, (ke_msg_func_t)APP_GlpcRacpReq_timeout_handler},

};

const struct ke_state_handler g_AppGlpcTableHandler = KE_STATE_HANDLER(s_appGlpcMsgHandlerList);

/*******************************************************************************
 * Code
 ******************************************************************************/
void APP_GlpcAddProfileTask(void)
{
    struct gapm_profile_task_add_cmd *req =
        KE_MSG_ALLOC_DYN(GAPM_PROFILE_TASK_ADD_CMD, TASK_GAPM, TASK_APP, gapm_profile_task_add_cmd, 0);
    /* Fill message */
    req->operation = GAPM_PROFILE_TASK_ADD;
    req->sec_lvl = 0;
    req->prf_task_id = TASK_ID_GLPC;
    req->app_task = TASK_APP;
    req->start_hdl = 0;

    /* Send the message */
    APP_MsgSend(req);
}

void APP_GlpcEnableReq(uint16_t conhdl)
{
    struct glpc_enable_req *msg =
        KE_MSG_ALLOC(GLPC_ENABLE_REQ, KE_BUILD_ID(prf_get_task_from_id(TASK_ID_GLPC), CONHDL2CONIDX(conhdl)), TASK_APP,
                     glpc_enable_req);

    msg->con_type = PRF_CON_DISCOVERY;

    /* Send the message */
    APP_MsgSend(msg);
}

int APP_GlpcEnableRspHandler(ke_msg_id_t const msgid,
                             struct glpc_enable_rsp *param,
                             ke_task_id_t const dest_id,
                             ke_task_id_t const src_id)
{
    QPRINTF("APP_GlpcEnableRspHandler status: 0x%x.\r\n", param->status);
    return (KE_MSG_CONSUMED);
}

void APP_GlpcRegisterReq(uint16_t conhdl, bool ctx_en)
{
    struct glpc_register_req *msg =
        KE_MSG_ALLOC(GLPC_REGISTER_REQ, KE_BUILD_ID(prf_get_task_from_id(TASK_ID_GLPC), CONHDL2CONIDX(conhdl)),
                     TASK_APP, glpc_register_req);

    msg->meas_ctx_en = ctx_en;

    /* Send the message */
    APP_MsgSend(msg);
}

int APP_GlpcRegisterRspHandler(ke_msg_id_t const msgid,
                               struct glpc_register_rsp *param,
                               ke_task_id_t const dest_id,
                               ke_task_id_t const src_id)
{
    QPRINTF("APP_GlpcRegisterRspHandler status : 0x%x", param->status);

    return (KE_MSG_CONSUMED);
}

void APP_GlpcReadFeaturesReq(uint16_t conhdl)
{
    struct app_glpc_read_req *msg =
        KE_MSG_ALLOC(GLPC_READ_FEATURES_REQ, KE_BUILD_ID(prf_get_task_from_id(TASK_ID_GLPC), CONHDL2CONIDX(conhdl)),
                     TASK_APP, app_glpc_read_req);
    msg->dummy = 0;
    /* Send the message */
    APP_MsgSend(msg);
}

int APP_GlpcReadFeatureRspHandler(ke_msg_id_t const msgid,
                                  struct glpc_read_features_rsp *param,
                                  ke_task_id_t const dest_id,
                                  ke_task_id_t const src_id)
{
    QPRINTF("APP_GlpcReadFeatureRspHandler feature : 0x%x.\r\n", param->features);

    return (KE_MSG_CONSUMED);
}

static void APP_GlpcRacpReq(uint16_t conhdl, struct glp_racp_req *racp_req)
{
    struct glpc_racp_req *msg = KE_MSG_ALLOC(
        GLPC_RACP_REQ, KE_BUILD_ID(prf_get_task_from_id(TASK_ID_GLPC), CONHDL2CONIDX(conhdl)), TASK_APP, glpc_racp_req);

    msg->racp_req = *racp_req;
    /* Send the message */
    APP_MsgSend(msg);
}

void APP_GlpcReportStoredRecord(uint16_t conhdl)
{
    struct glp_racp_req racp_req;

    racp_req.op_code = GLP_REQ_REP_STRD_RECS;
    racp_req.filter.operator= GLP_OP_ALL_RECS;
    APP_GlpcRacpReq(conhdl, &racp_req);
}

void APP_GlpcDelStoredRecord(uint16_t conhdl)
{
    struct glp_racp_req racp_req;
    racp_req.op_code = GLP_REQ_DEL_STRD_RECS;
    racp_req.filter.operator= GLP_OP_ALL_RECS;
    APP_GlpcRacpReq(conhdl, &racp_req);
}

void APP_GlpcReportNumStoredRecord(uint16_t conhdl)
{
    struct glp_racp_req racp_req;

    racp_req.op_code = GLP_REQ_REP_NUM_OF_STRD_RECS;
    racp_req.filter.operator= GLP_OP_ALL_RECS;
    APP_GlpcRacpReq(conhdl, &racp_req);
}

void APP_GlpcAbortOperation(uint16_t conhdl)
{
    struct glp_racp_req racp_req;

    racp_req.op_code = GLP_REQ_ABORT_OP;
    APP_GlpcRacpReq(conhdl, &racp_req);
}

int APP_GlpcRacpRspHandler(ke_msg_id_t const msgid,
                           struct glpc_racp_rsp *param,
                           ke_task_id_t const dest_id,
                           ke_task_id_t const src_id)
{
    if (GAP_ERR_NO_ERROR == param->status)
    {
        switch (param->racp_rsp.op_code)
        {
            case GLP_REQ_NUM_OF_STRD_RECS_RSP:
            {
                QPRINTF("APP_GlpcRacpRspHandler record number:%d", param->racp_rsp.operand.num_of_record);
            }
            break;

            case GLP_REQ_RSP_CODE:
            {
                QPRINTF("APP_GlpcRacpRspHandler req code:%d.\r\n", param->racp_rsp.operand.rsp.op_code_req);
            }
            break;

            default:
                break;
        }
    }
    else
    {
        QPRINTF("APP_GlpcRacpRspHandler fail, the reason :0x%x \r\n", param->status);
    }

    return (KE_MSG_CONSUMED);
}

int APP_GlpcMeasIndHandler(ke_msg_id_t const msgid,
                           struct glpc_meas_ind *param,
                           ke_task_id_t const dest_id,
                           ke_task_id_t const src_id)
{
    QPRINTF("APP_GlpcMeasIndHandler seq_num : %d\r\n", param->seq_num);
    return (KE_MSG_CONSUMED);
}

int APP_GlpcMeasCtxIndHandler(ke_msg_id_t const msgid,
                              struct glpc_meas_ctx_ind *param,
                              ke_task_id_t const dest_id,
                              ke_task_id_t const src_id)
{
    QPRINTF("APP_GlpcMeasCtxIndHandler seq_num : %d\r\n", param->seq_num);
    return (KE_MSG_CONSUMED);
}

int APP_GlpcRacpReq_timeout_handler(ke_msg_id_t const msgid,
                                    void *param,
                                    ke_task_id_t const dest_id,
                                    ke_task_id_t const src_id)
{
    QPRINTF("APP_GlpcRacpReq_timeout_handler timeout.\r\n");
    return (KE_MSG_CONSUMED);
}

#endif

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
#if BLE_BP_COLLECTOR

/*******************************************************************************
 * Variables
 ******************************************************************************/
const struct ke_msg_handler s_appBlpcMsgHandlerList[] = {
    {BLPC_ENABLE_RSP, (ke_msg_func_t)APP_BlpcEnableRspHandler},
    {BLPC_RD_CHAR_RSP, (ke_msg_func_t)APP_BlpcRdCharRspHandler},
    {BLPC_CFG_INDNTF_RSP, (ke_msg_func_t)APP_BlpcCfgIndNtfRspHandler},
    {BLPC_BP_MEAS_IND, (ke_msg_func_t)APP_BlpcBpMeasIndHandler},
};

const struct ke_state_handler g_AppBlpcTableHandler = KE_STATE_HANDLER(s_appBlpcMsgHandlerList);

/*******************************************************************************
 * Code
 ******************************************************************************/
void APP_BlpcAddProfileTask(void)
{
    struct gapm_profile_task_add_cmd *req =
        KE_MSG_ALLOC_DYN(GAPM_PROFILE_TASK_ADD_CMD, TASK_GAPM, TASK_APP, gapm_profile_task_add_cmd, 0);
    /* Fill message */
    req->operation = GAPM_PROFILE_TASK_ADD;
    req->sec_lvl = 0;
    req->prf_task_id = TASK_ID_BLPC;
    req->app_task = TASK_APP;
    req->start_hdl = 0;

    /* Send the message */
    APP_MsgSend(req);
}

void APP_BlpcEnableReq(uint16_t conhdl)
{
    struct blpc_enable_req *msg =
        KE_MSG_ALLOC(BLPC_ENABLE_REQ, KE_BUILD_ID(prf_get_task_from_id(TASK_ID_BLPC), CONHDL2CONIDX(conhdl)), TASK_APP,
                     blpc_enable_req);

    msg->con_type = PRF_CON_DISCOVERY;

    /* Send the message */
    APP_MsgSend(msg);
}

int APP_BlpcEnableRspHandler(ke_msg_id_t const msgid,
                             struct blpc_enable_rsp *param,
                             ke_task_id_t const dest_id,
                             ke_task_id_t const src_id)
{
    QPRINTF("APP_BlpcEnableRspHandler status: 0x%x.\r\n", param->status);
    return (KE_MSG_CONSUMED);
}

void APP_BlpcRdCharReq(uint16_t conhdl, uint8_t char_code)
{
    struct blpc_rd_char_req *msg =
        KE_MSG_ALLOC(BLPC_RD_CHAR_REQ, KE_BUILD_ID(prf_get_task_from_id(TASK_ID_BLPC), CONHDL2CONIDX(conhdl)), TASK_APP,
                     blpc_rd_char_req);

    msg->char_code = char_code;

    /* Send the message */
    APP_MsgSend(msg);
}

int APP_BlpcRdCharRspHandler(ke_msg_id_t const msgid,
                             struct blpc_rd_char_rsp *param,
                             ke_task_id_t const dest_id,
                             ke_task_id_t const src_id)
{
    uint8_t i;
    QPRINTF("APP_BlpcRdCharRspHandler status: 0x%x.\r\n", param->info.status);
    if (param->info.status == GAP_ERR_NO_ERROR)
    {
        QPRINTF("value = 0x");
        for (i = param->info.length; i > 0; i--)
        {
            QPRINTF("%x,", param->info.value[i - 1]);
            if (i == 0)
                QPRINTF("\r\n");
        }
    }
    return (KE_MSG_CONSUMED);
}

void APP_BlpcCfgIndNtfReq(uint16_t conhdl, uint8_t char_code, uint16_t cfg_val)
{
    struct blpc_cfg_indntf_req *msg =
        KE_MSG_ALLOC(BLPC_CFG_INDNTF_REQ, KE_BUILD_ID(prf_get_task_from_id(TASK_ID_BLPC), CONHDL2CONIDX(conhdl)),
                     TASK_APP, blpc_cfg_indntf_req);

    msg->char_code = char_code;
    msg->cfg_val = cfg_val;

    APP_MsgSend(msg);
}

int APP_BlpcCfgIndNtfRspHandler(ke_msg_id_t const msgid,
                                struct tipc_ct_ntf_cfg_rsp *param,
                                ke_task_id_t const dest_id,
                                ke_task_id_t const src_id)
{
    QPRINTF("APP_BlpcCfgIndNtfRspHandler status: 0x%x.\r\n", param->status);
    return (KE_MSG_CONSUMED);
}

void APP_BlpcBpMeasIndHandler(ke_msg_id_t const msgid,
                              struct blpc_bp_meas_ind *param,
                              ke_task_id_t const dest_id,
                              ke_task_id_t const src_id)
{
    switch (param->flag_interm_cp)
    {
        case 1:
            QPRINTF("Intermediary cuff pressure measurement.\r\n");
            QPRINTF(
                "flag = 0x%x,user_id = %d,systolic= %d, diastolic=%d,mean_arterial_pressure = %d, meas_status= "
                "0x%x.\r\n",
                param->meas_val.flags, param->meas_val.user_id, param->meas_val.systolic, param->meas_val.diastolic,
                param->meas_val.mean_arterial_pressure, param->meas_val.meas_status);
            break;
        case 0:
            QPRINTF("Stable blood pressure measurement.\r\n");
            QPRINTF("Time is: %d:%d:%d,%d/%d/%d\r\n", param->meas_val.time_stamp.hour, param->meas_val.time_stamp.min,
                    param->meas_val.time_stamp.sec, param->meas_val.time_stamp.year, param->meas_val.time_stamp.month,
                    param->meas_val.time_stamp.day);
            QPRINTF(
                "flag = 0x%x,user_id = %d,systolic= %d, diastolic=%d,mean_arterial_pressure = %d, pulse_rate= "
                "%d,meas_status= 0x%x.\r\n",
                param->meas_val.flags, param->meas_val.user_id, param->meas_val.systolic, param->meas_val.diastolic,
                param->meas_val.mean_arterial_pressure, param->meas_val.pulse_rate, param->meas_val.meas_status);
            break;
    }
}

#endif

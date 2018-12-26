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
#if BLE_HID_REPORT_HOST

/*******************************************************************************
 * Variables
 ******************************************************************************/
const struct ke_msg_handler s_appHogprhMsgHandlerList[] = {
    {HOGPRH_ENABLE_RSP, (ke_msg_func_t)APP_HogprhEnableRspHandler},
    {HOGPRH_READ_INFO_RSP, (ke_msg_func_t)APP_HogprhReadInfoRspHandler},
    {HOGPRH_WRITE_RSP, (ke_msg_func_t)APP_HogprhWriteRspHandler},
    {HOGPRH_REPORT_IND, (ke_msg_func_t)APP_HogprhReportIndHandler},
};

const struct ke_state_handler g_AppHogprhTableHandler = KE_STATE_HANDLER(s_appHogprhMsgHandlerList);

/*******************************************************************************
 * Code
 ******************************************************************************/
void APP_HogprhAddProfileTask(void)
{
    struct gapm_profile_task_add_cmd *req =
        KE_MSG_ALLOC_DYN(GAPM_PROFILE_TASK_ADD_CMD, TASK_GAPM, TASK_APP, gapm_profile_task_add_cmd, 0);
    /* Fill message */
    req->operation = GAPM_PROFILE_TASK_ADD;
    req->sec_lvl = 0;
    req->prf_task_id = TASK_ID_HOGPRH;
    req->app_task = TASK_APP;
    req->start_hdl = 0;

    /* Send the message */
    APP_MsgSend(req);
}

void APP_HogprhEnableReq(uint16_t conhdl,
                         uint8_t type,
                         uint8_t hids_nb,
                         struct hogprh_content hids[HOGPRH_NB_HIDS_INST_MAX])
{
    struct hogprh_enable_req *msg =
        KE_MSG_ALLOC(HOGPRH_ENABLE_REQ, KE_BUILD_ID(prf_get_task_from_id(TASK_ID_HOGPRH), CONHDL2CONIDX(conhdl)),
                     TASK_APP, hogprh_enable_req);

    if (type == PRF_CON_DISCOVERY)
        msg->con_type = PRF_CON_DISCOVERY;
    else
    {
        msg->con_type = PRF_CON_NORMAL;
        msg->hids_nb = hids_nb;
        memcpy(&(msg->hids[0]), &(hids[0]), sizeof(struct hogprh_content) * HOGPRH_NB_HIDS_INST_MAX);
    }
    /* Send the message */
    APP_MsgSend(msg);
}

int APP_HogprhEnableRspHandler(ke_msg_id_t const msgid,
                               struct hogprh_enable_rsp *param,
                               ke_task_id_t const dest_id,
                               ke_task_id_t const src_id)
{
    QPRINTF("APP_HogprhEnableRspHandler status: 0x%x.\r\n", param->status);
    return (KE_MSG_CONSUMED);
}

void APP_HogprhReadInfoReq(uint16_t conhdl, uint8_t info, uint8_t hid_idx, uint8_t report_idx)
{
    struct hogprh_read_info_req *msg =
        KE_MSG_ALLOC(HOGPRH_READ_INFO_REQ, KE_BUILD_ID(prf_get_task_from_id(TASK_ID_HOGPRH), CONHDL2CONIDX(conhdl)),
                     TASK_APP, hogprh_read_info_req);

    msg->info = info;
    msg->hid_idx = hid_idx;
    msg->report_idx = report_idx;

    /* Send the message */
    APP_MsgSend(msg);
}

int APP_HogprhReadInfoRspHandler(ke_msg_id_t const msgid,
                                 struct hogprh_read_info_rsp *param,
                                 ke_task_id_t const dest_id,
                                 ke_task_id_t const src_id)
{
    QPRINTF("APP_HogprhReadInfoRspHandler status: 0x%x info: 0x%x report_idx: 0x%x .\r\n", param->status, param->info,
            param->report_idx);
    return (KE_MSG_CONSUMED);
}

void APP_HogprhWriteReq(
    uint16_t conhdl, uint8_t info, uint8_t hid_idx, uint8_t report_idx, bool wr_cmd, union hogprh_data *data)
{
    struct hogprh_write_req *msg;
    uint8_t data_len = 0;

    if (info == HOGPRH_REPORT)
        data_len = data->report.length;
    msg = KE_MSG_ALLOC_DYN(HOGPRH_WRITE_REQ, KE_BUILD_ID(prf_get_task_from_id(TASK_ID_HOGPRH), CONHDL2CONIDX(conhdl)),
                           TASK_APP, hogprh_write_req, data_len);

    msg->info = info;
    msg->hid_idx = hid_idx;
    msg->report_idx = report_idx;
    msg->wr_cmd = wr_cmd;
    memcpy(&msg->data, data, sizeof(union hogprh_data) + data->report.length);

    /* Send the message */
    APP_MsgSend(msg);
}

int APP_HogprhWriteRspHandler(ke_msg_id_t const msgid,
                              struct hogprh_write_rsp *param,
                              ke_task_id_t const dest_id,
                              ke_task_id_t const src_id)
{
    QPRINTF("APP_HogprhWriteRspHandler status: 0x%x info: 0x%x report_idx: 0x%x.\r\n", param->status, param->info,
            param->report_idx);
    return (KE_MSG_CONSUMED);
}

int APP_HogprhReportIndHandler(ke_msg_id_t const msgid,
                               struct hogprh_report_ind *param,
                               ke_task_id_t const dest_id,
                               ke_task_id_t const src_id)
{
    QPRINTF("APP_HogprhReportIndHandler hid_idx: 0x%x report_idx: 0x%x.\r\n", param->hid_idx, param->report_idx);
    return (KE_MSG_CONSUMED);
}

#endif

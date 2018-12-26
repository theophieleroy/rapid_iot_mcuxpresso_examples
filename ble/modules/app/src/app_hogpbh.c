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
#if BLE_HID_BOOT_HOST

/*******************************************************************************
 * Variables
 ******************************************************************************/
const struct ke_msg_handler s_appHogpbhMsgHandlerList[] = {
    {HOGPBH_ENABLE_RSP, (ke_msg_func_t)APP_HogpbhEnableRspHandler},
    {HOGPBH_READ_INFO_RSP, (ke_msg_func_t)APP_HogpbhReadInfoRspHandler},
    {HOGPBH_WRITE_RSP, (ke_msg_func_t)APP_HogpbhWriteRspHandler},
    {HOGPBH_BOOT_REPORT_IND, (ke_msg_func_t)APP_HogpbhBootReportIndHandler},
};

const struct ke_state_handler g_AppHogpbhTableHandler = KE_STATE_HANDLER(s_appHogpbhMsgHandlerList);

/*******************************************************************************
 * Code
 ******************************************************************************/
void APP_HogpbhAddProfileTask(void)
{
    struct gapm_profile_task_add_cmd *req =
        KE_MSG_ALLOC_DYN(GAPM_PROFILE_TASK_ADD_CMD, TASK_GAPM, TASK_APP, gapm_profile_task_add_cmd, 0);
    /* Fill message */
    req->operation = GAPM_PROFILE_TASK_ADD;
    req->sec_lvl = 0;
    req->prf_task_id = TASK_ID_HOGPBH;
    req->app_task = TASK_APP;
    req->start_hdl = 0;

    /* Send the message */
    APP_MsgSend(req);
}

void APP_HogpbhEnableReq(uint16_t conhdl,
                         uint8_t type,
                         uint8_t hids_nb,
                         struct hogpbh_content hids[HOGPBH_NB_HIDS_INST_MAX])
{
    struct hogpbh_enable_req *msg =
        KE_MSG_ALLOC(HOGPBH_ENABLE_REQ, KE_BUILD_ID(prf_get_task_from_id(TASK_ID_HOGPBH), CONHDL2CONIDX(conhdl)),
                     TASK_APP, hogpbh_enable_req);

    if (type == PRF_CON_DISCOVERY)
        msg->con_type = PRF_CON_DISCOVERY;
    else
    {
        msg->con_type = PRF_CON_NORMAL;
        msg->hids_nb = hids_nb;
        memcpy(&(msg->hids[0]), &(hids[0]), sizeof(struct hogpbh_content) * HOGPBH_NB_HIDS_INST_MAX);
    }

    /* Send the message */
    APP_MsgSend(msg);
}

int APP_HogpbhEnableRspHandler(ke_msg_id_t const msgid,
                               struct hogpbh_enable_rsp *param,
                               ke_task_id_t const dest_id,
                               ke_task_id_t const src_id)
{
    QPRINTF("APP_HogpbhEnableRspHandler status: 0x%x.\r\n", param->status);
    return (KE_MSG_CONSUMED);
}

void APP_HogpbhReadInfoReq(uint16_t conhdl, uint8_t info, uint8_t hid_idx)
{
    struct hogpbh_read_info_req *msg =
        KE_MSG_ALLOC(HOGPBH_READ_INFO_REQ, KE_BUILD_ID(prf_get_task_from_id(TASK_ID_HOGPBH), CONHDL2CONIDX(conhdl)),
                     TASK_APP, hogpbh_read_info_req);

    msg->info = info;
    msg->hid_idx = hid_idx;

    /* Send the message */
    APP_MsgSend(msg);
}

int APP_HogpbhReadInfoRspHandler(ke_msg_id_t const msgid,
                                 struct hogpbh_read_info_rsp *param,
                                 ke_task_id_t const dest_id,
                                 ke_task_id_t const src_id)
{
    QPRINTF("APP_HogpbhReadInfoRspHandler status: 0x%x info:0x%x .\r\n", param->status, param->info);
    return (KE_MSG_CONSUMED);
}

void APP_HogpbhWriteReq(uint16_t conhdl, uint8_t info, uint8_t hid_idx, bool wr_cmd, union hogpbh_data *data)
{
    struct hogpbh_write_req *msg =
        KE_MSG_ALLOC_DYN(HOGPBH_WRITE_REQ, KE_BUILD_ID(prf_get_task_from_id(TASK_ID_HOGPBH), CONHDL2CONIDX(conhdl)),
                         TASK_APP, hogpbh_write_req, data->report.length);

    msg->info = info;
    msg->hid_idx = hid_idx;
    msg->wr_cmd = wr_cmd;

    memcpy(&msg->data, data, sizeof(union hogpbh_data) + data->report.length);

    /* Send the message */
    APP_MsgSend(msg);
}

int APP_HogpbhWriteRspHandler(ke_msg_id_t const msgid,
                              struct hogpbh_write_rsp *param,
                              ke_task_id_t const dest_id,
                              ke_task_id_t const src_id)
{
    QPRINTF("APP_HogpbhWriteRspHandler status: 0x%x.\r\n", param->status);
    return (KE_MSG_CONSUMED);
}

int APP_HogpbhBootReportIndHandler(ke_msg_id_t const msgid,
                                   struct hogpbh_boot_report_ind *param,
                                   ke_task_id_t const dest_id,
                                   ke_task_id_t const src_id)
{
    QPRINTF("APP_HogpbhBootReportIndHandler info: 0x%x hid_idx  0x%x .\r\n", param->info, param->hid_idx);
    return (KE_MSG_CONSUMED);
}

#endif

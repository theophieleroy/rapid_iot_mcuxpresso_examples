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
#if BLE_FINDME_LOCATOR

/*******************************************************************************
 * Variables
 ******************************************************************************/
const struct ke_msg_handler s_appFindlMsgHandlerList[] = {
    {FINDL_ENABLE_RSP, (ke_msg_func_t)APP_FindlEnableRspHandler},
    {FINDL_SET_ALERT_RSP, (ke_msg_func_t)APP_FindlSetAlertRspHandler},
};

const struct ke_state_handler g_AppFindlTableHandler = KE_STATE_HANDLER(s_appFindlMsgHandlerList);

/*******************************************************************************
 * Code
 ******************************************************************************/
void APP_FindlAddProfileTask(void)
{
    struct gapm_profile_task_add_cmd *req =
        KE_MSG_ALLOC_DYN(GAPM_PROFILE_TASK_ADD_CMD, TASK_GAPM, TASK_APP, gapm_profile_task_add_cmd, 0);
    /* Fill message */
    req->operation = GAPM_PROFILE_TASK_ADD;
    req->sec_lvl = 0;
    req->prf_task_id = TASK_ID_FINDL;
    req->app_task = TASK_APP;
    req->start_hdl = 0;

    /* Send the message */
    APP_MsgSend(req);
}

void APP_FindlEnableReq(uint16_t conhdl)
{
    struct findl_enable_req *msg =
        KE_MSG_ALLOC(FINDL_ENABLE_REQ, KE_BUILD_ID(prf_get_task_from_id(TASK_ID_FINDL), CONHDL2CONIDX(conhdl)),
                     TASK_APP, findl_enable_req);

    msg->con_type = PRF_CON_DISCOVERY;

    /* Send the message */
    APP_MsgSend(msg);
}

int APP_FindlEnableRspHandler(ke_msg_id_t const msgid,
                              struct findl_enable_rsp *param,
                              ke_task_id_t const dest_id,
                              ke_task_id_t const src_id)
{
    QPRINTF("APP_FindlEnableRspHandler status: 0x%x.\r\n", param->status);
    return (KE_MSG_CONSUMED);
}

void APP_FindlSetAlertReq(uint16_t conhdl, uint8_t alert_lvl)
{
    struct findl_set_alert_req *msg =
        KE_MSG_ALLOC(FINDL_SET_ALERT_REQ, KE_BUILD_ID(prf_get_task_from_id(TASK_ID_FINDL), CONHDL2CONIDX(conhdl)),
                     TASK_APP, findl_set_alert_req);

    msg->alert_lvl = alert_lvl;

    /* Send the message */
    APP_MsgSend(msg);
}

int APP_FindlSetAlertRspHandler(ke_msg_id_t const msgid,
                                struct findl_set_alert_rsp *param,
                                ke_task_id_t const dest_id,
                                ke_task_id_t const src_id)
{
    QPRINTF("APP_FindlSetAlertRspHandler status: 0x%x.\r\n", param->status);
    return (KE_MSG_CONSUMED);
}

#endif

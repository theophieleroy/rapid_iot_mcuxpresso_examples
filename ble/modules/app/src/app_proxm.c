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
#if BLE_PROX_MONITOR

/*******************************************************************************
 * Variables
 ******************************************************************************/
const struct ke_msg_handler s_appProxmMsgHandlerList[] = {
    {PROXM_ENABLE_RSP, (ke_msg_func_t)APP_ProxmEnableRspHandler},
    {PROXM_RD_RSP, (ke_msg_func_t)APP_ProxmRdRspHandler},
    {PROXM_WR_ALERT_LVL_RSP, (ke_msg_func_t)APP_ProxmWrAlertLvlRspHandler},
};

const struct ke_state_handler g_AppProxmTableHandler = KE_STATE_HANDLER(s_appProxmMsgHandlerList);

/*******************************************************************************
 * Code
 ******************************************************************************/
void APP_ProxmAddProfileTask(void)
{
    struct gapm_profile_task_add_cmd *req =
        KE_MSG_ALLOC_DYN(GAPM_PROFILE_TASK_ADD_CMD, TASK_GAPM, TASK_APP, gapm_profile_task_add_cmd, 0);
    /* Fill message */
    req->operation = GAPM_PROFILE_TASK_ADD;
    req->sec_lvl = 0;
    req->prf_task_id = TASK_ID_PROXM;
    req->app_task = TASK_APP;
    req->start_hdl = 0;

    /* Send the message */
    APP_MsgSend(req);
}

void APP_ProxmEnableReq(uint16_t conhdl)
{
    struct proxm_enable_req *msg =
        KE_MSG_ALLOC(PROXM_ENABLE_REQ, KE_BUILD_ID(prf_get_task_from_id(TASK_ID_PROXM), CONHDL2CONIDX(conhdl)),
                     TASK_APP, proxm_enable_req);

    msg->con_type = PRF_CON_DISCOVERY;

    /* Send the message */
    APP_MsgSend(msg);
}

void APP_ProxmRdReq(uint16_t conhdl, uint8_t svc_code)
{
    struct proxm_rd_req *msg = KE_MSG_ALLOC(
        PROXM_RD_REQ, KE_BUILD_ID(prf_get_task_from_id(TASK_ID_PROXM), CONHDL2CONIDX(conhdl)), TASK_APP, proxm_rd_req);

    msg->svc_code = svc_code;

    /* Send the message */
    APP_MsgSend(msg);
}

void APP_ProxmWrAlertLvlReq(uint16_t conhdl, uint8_t svc_code, uint8_t lvl)
{
    struct proxm_wr_alert_lvl_req *msg =
        KE_MSG_ALLOC(PROXM_WR_ALERT_LVL_REQ, KE_BUILD_ID(prf_get_task_from_id(TASK_ID_PROXM), CONHDL2CONIDX(conhdl)),
                     TASK_APP, proxm_wr_alert_lvl_req);

    msg->svc_code = svc_code;
    msg->lvl = lvl;

    /* Send the message */
    APP_MsgSend(msg);
}

int APP_ProxmEnableRspHandler(ke_msg_id_t const msgid,
                              struct proxm_enable_rsp *param,
                              ke_task_id_t const dest_id,
                              ke_task_id_t const src_id)
{
    QPRINTF("APP_ProxmEnableRspHandler status: 0x%x.\r\n", param->status);

    return (KE_MSG_CONSUMED);
}

int APP_ProxmRdRspHandler(ke_msg_id_t const msgid,
                          struct proxm_rd_rsp *param,
                          ke_task_id_t const dest_id,
                          ke_task_id_t const src_id)
{
    QPRINTF("APP_ProxmRdRspHandler svc_code : 0x%x and status: 0x%x.\r\n", param->svc_code, param->status);

    return (KE_MSG_CONSUMED);
}

int APP_ProxmWrAlertLvlRspHandler(ke_msg_id_t const msgid,
                                  struct proxm_wr_alert_lvl_rsp *param,
                                  ke_task_id_t const dest_id,
                                  ke_task_id_t const src_id)
{
    QPRINTF("APP_ProxmWrAlertLvlRspHandler status: 0x%x.\r\n", param->status);
    return (KE_MSG_CONSUMED);
}

#endif

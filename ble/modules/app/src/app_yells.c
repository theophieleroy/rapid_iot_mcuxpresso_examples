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

/*!
 * @addtogroup APP_YELLS_API
 * @{
 */

#include "app_ble.h"
#include "fsl_debug_console.h"

#if BLE_YELL_SERVER

#include "yells.h"
#include "prf_utils.h"
#include "gattc.h"
#include "YellApp.h"

extern void yells_user_rx_callback(ke_msg_id_t const msgid,
                                   struct yells_get_data_ind *param,
                                   ke_task_id_t const dest_id,
                                   ke_task_id_t const src_id);

/*******************************************************************************
 * Variables
 ******************************************************************************/
struct app_yells_env_tag g_AppYellsEnv;

/*******************************************************************************
 * Code
 ******************************************************************************/
void APP_YellsAddProfileTask(void)
{
    struct yells_db_cfg *db_cfg;
    /* Allocate the YELLS_CREATE_DB_REQ */
    struct gapm_profile_task_add_cmd *req = KE_MSG_ALLOC_DYN(GAPM_PROFILE_TASK_ADD_CMD, TASK_GAPM, TASK_APP,
                                                             gapm_profile_task_add_cmd, sizeof(struct yells_db_cfg));
    /* Fill message */
    req->operation = GAPM_PROFILE_TASK_ADD;
    req->sec_lvl = PERM(SVC_AUTH, ENABLE);
    req->prf_task_id = TASK_ID_YELLS;
    req->app_task = TASK_APP;
    req->start_hdl = 0;
    /* Set parameters */
    db_cfg = (struct yells_db_cfg *)req->param;
    db_cfg->features = YELLS_MANDATORY_MASK;

    /* Send the message */
    APP_MsgSend(req);
}

void APP_YellsSendDataReq(uint16_t conhdl, uint8_t length, uint8_t *data)
{
    struct yells_send_data_req *msg = KE_MSG_ALLOC_DYN(YELLS_SEND_DATA_REQ, prf_get_task_from_id(TASK_ID_YELLS),
                                                       TASK_APP, yells_send_data_req, length);
    struct yells_env_tag *yells_env = PRF_ENV_GET(YELLS, yells);

    msg->conhdl = conhdl;
    msg->length = length;
    memcpy(msg->data, data, length);

    APP_MsgSend(msg);
}

int APP_YellsSendDataRspHandler(ke_msg_id_t const msgid,
                                struct yells_send_data_rsp *param,
                                ke_task_id_t const dest_id,
                                ke_task_id_t const src_id)
{
    if (param->status == GAP_ERR_NO_ERROR)
    {
        YELL_PushData();
    }
    else
    {
        QPRINTF("YELLS send error %02x.\r\n", param->status);
    }

    return (KE_MSG_CONSUMED);
}

int APP_YellsCfgIndntfIndHandler(ke_msg_id_t const msgid,
                                 struct yells_cfg_indntf_ind *param,
                                 ke_task_id_t const dest_id,
                                 ke_task_id_t const src_id)
{
    QPRINTF("conhdl: %d", param->conhdl);
    uint8_t conidx = CONHDL2CONIDX(param->conhdl);
    struct yells_env_tag *yells_env_tag = PRF_ENV_GET(YELLS, yells);
    if (param->cfg_val == PRF_CLI_START_NTF)
    {
        g_AppYellsEnv.ntf_cfg[conidx] = YELLS_VALUE_NTF_ON;
        ke_timer_set(YELLS_NTF_TIMER, yells_env_tag->prf_env.prf_task, 200);
    }
    else
    {
        g_AppYellsEnv.ntf_cfg[conidx] = YELLS_VALUE_NTF_OFF;
        int i;
        for (i = 0; i < BLE_CONNECTION_MAX; i++)
        {
            if (g_AppYellsEnv.ntf_cfg[conidx] == YELLS_VALUE_NTF_ON)
                break;
        }
        if (i == BLE_CONNECTION_MAX)
        {
            ke_timer_clear(YELLS_NTF_TIMER, dest_id);
        }
    }

    return (KE_MSG_CONSUMED);
}

int APP_YellsGetDataIndHandler(ke_msg_id_t const msgid,
                               struct yells_get_data_ind *param,
                               ke_task_id_t const dest_id,
                               ke_task_id_t const src_id)
{
    for (int i = 0; i < param->length; i++)
    {
        /*       QPRINTF("%02X ", param->data[i]); */
    }
    /*   QPRINTF("\r\n"); */
    yells_user_rx_callback(msgid, param, dest_id, src_id);

    return (KE_MSG_CONSUMED);
}

int APP_YellsThroughputTimerHandler(ke_msg_id_t const msgid,
                                    void const *param,
                                    ke_task_id_t const dest_id,
                                    ke_task_id_t const src_id)
{
    struct yells_env_tag *yells_env_tag = PRF_ENV_GET(YELLS, yells);
    return (KE_MSG_CONSUMED);
}

const struct ke_msg_handler s_appYellsMsgHandlerList[] = {

    {YELLS_GET_DATA_IND, (ke_msg_func_t)APP_YellsGetDataIndHandler},
    {YELLS_SEND_DATA_RSP, (ke_msg_func_t)APP_YellsSendDataRspHandler},
    {YELLS_CFG_INDNTF_IND, (ke_msg_func_t)APP_YellsCfgIndntfIndHandler},

};

const struct ke_state_handler g_AppYellsTableHandler = KE_STATE_HANDLER(s_appYellsMsgHandlerList);
#endif

/*! @brief @} APP_YELLS_API */

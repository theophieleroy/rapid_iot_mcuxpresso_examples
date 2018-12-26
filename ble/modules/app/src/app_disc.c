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
#if BLE_DIS_CLIENT
#include "disc.h"

/*******************************************************************************
 * Variables
 ******************************************************************************/
const char *g_AppDiscInfo[DIS_CHAR_MAX] = {
        [DISC_MANUFACTURER_NAME_CHAR] = "MANUFACTURER NAME: ",
        [DISC_MODEL_NB_STR_CHAR] = "MODEL NB: ",
        [DISC_SERIAL_NB_STR_CHAR] = "SERIAL NB: ",
        [DISC_HARD_REV_STR_CHAR] = "HARD REV: ",
        [DISC_FIRM_REV_STR_CHAR] = "FIRM REV: ",
        [DISC_SW_REV_STR_CHAR] = "SW REV: ",
        [DISC_SYSTEM_ID_CHAR] = "SYSTEM ID: ",
        [DISC_IEEE_CHAR] = "IEEE CHAR: ",
        [DISC_PNP_ID_CHAR] = "PNP ID: ",
};

const struct ke_msg_handler s_appDiscMsgHandlerList[] = {
    {DISC_ENABLE_RSP, (ke_msg_func_t)APP_DiscEnableRspHandler},
    {DISC_RD_CHAR_RSP, (ke_msg_func_t)APP_DiscRdCharRspHandler},
};

const struct ke_state_handler g_AppDiscTableHandler = KE_STATE_HANDLER(s_appDiscMsgHandlerList);

/*******************************************************************************
 * Code
 ******************************************************************************/
void APP_DiscAddProfileTask(void)
{
    struct gapm_profile_task_add_cmd *req =
        KE_MSG_ALLOC_DYN(GAPM_PROFILE_TASK_ADD_CMD, TASK_GAPM, TASK_APP, gapm_profile_task_add_cmd, 0);
    req->operation = GAPM_PROFILE_TASK_ADD;
    req->sec_lvl = 0; /*app task mono-instantiated */
    req->prf_task_id = TASK_ID_DISC;
    req->app_task = TASK_APP;

    /* Send the message */
    APP_MsgSend(req);
}

void APP_DiscEnableReq(uint16_t conhdl)
{
    struct disc_enable_req *msg =
        KE_MSG_ALLOC(DISC_ENABLE_REQ, KE_BUILD_ID(prf_get_task_from_id(TASK_ID_DISC), CONHDL2CONIDX(conhdl)), TASK_APP,
                     disc_enable_req);

    msg->con_type = PRF_CON_DISCOVERY;

    /* Send the message */
    APP_MsgSend(msg);
}

void APP_DiscRdCharReq(uint16_t conhdl, uint8_t char_code)
{
    struct disc_rd_char_req *msg =
        KE_MSG_ALLOC(DISC_RD_CHAR_REQ, KE_BUILD_ID(prf_get_task_from_id(TASK_ID_DISC), CONHDL2CONIDX(conhdl)), TASK_APP,
                     disc_rd_char_req);
    msg->char_code = char_code;
    /* Send the message */
    APP_MsgSend(msg);
}

int APP_DiscEnableRspHandler(ke_msg_id_t const msgid,
                             struct disc_enable_rsp *param,
                             ke_task_id_t const dest_id,
                             ke_task_id_t const src_id)
{
    QPRINTF("DISC enable response status: 0x%x.\r\n", param->status);

    return (KE_MSG_CONSUMED);
}

int APP_DiscRdCharRspHandler(ke_msg_id_t const msgid,
                             struct disc_rd_char_rsp *param,
                             ke_task_id_t const dest_id,
                             ke_task_id_t const src_id)
{
    QPRINTF("DISC read char rsp status: 0x%x.\r\n", param->info.status);

    /* Get the address of the environment */
    struct disc_env_tag *disc_env = PRF_ENV_GET(DISC, disc);
    uint8_t conidx = KE_IDX_GET(src_id);
    uint8_t char_code = DIS_CHAR_MAX;
    for (int i = 0; i < DIS_CHAR_MAX; i++)
    {
        if (disc_env->env[conidx]->dis.chars[i].val_hdl == param->info.handle)
            char_code = i;
    }

    if (param->info.status == GAP_ERR_NO_ERROR)
    {
        if (char_code <= DISC_SW_REV_STR_CHAR)
        {
            QPRINTF(g_AppDiscInfo[char_code]);
            for (int i = 0; i < (param->info.length); i++)
                QPRINTF("%c", param->info.value[i]);
            QPRINTF("\r\n");
        }
        else if (char_code < DISC_CHAR_MAX)
        {
            QPRINTF(g_AppDiscInfo[char_code]);
            for (uint8_t i = 0; i < param->info.length; i++)
                QPRINTF("%02x", param->info.value[i]);
            QPRINTF("\r\n");
        }
    }

    return (KE_MSG_CONSUMED);
}

#endif

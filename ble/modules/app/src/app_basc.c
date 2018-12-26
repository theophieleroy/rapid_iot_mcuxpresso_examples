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
#if BLE_BATT_CLIENT

/*******************************************************************************
 * Variables
 ******************************************************************************/
struct app_basc_env_tag g_AppBascEnv;

const struct ke_msg_handler s_appBascMsgHandlerList[] = {
    {BASC_ENABLE_RSP, (ke_msg_func_t)APP_BascEnableRspHandler},
    {BASC_READ_INFO_RSP, (ke_msg_func_t)APP_BascReadInfoRspHandler},
    {BASC_BATT_LEVEL_IND, (ke_msg_func_t)APP_BascBattLevelIndHandler},
    {BASC_BATT_LEVEL_NTF_CFG_RSP, (ke_msg_func_t)APP_BascBattLevelNtfCfgRsp_handler},
};

const struct ke_state_handler g_AppBascTableHandler = KE_STATE_HANDLER(s_appBascMsgHandlerList);

/*******************************************************************************
 * Code
 ******************************************************************************/
void APP_BascAddProfileTask(void)
{
    struct gapm_profile_task_add_cmd *req =
        KE_MSG_ALLOC_DYN(GAPM_PROFILE_TASK_ADD_CMD, TASK_GAPM, TASK_APP, gapm_profile_task_add_cmd, 1);
    /* Fill message parameters used by collector */
    req->operation = GAPM_PROFILE_TASK_ADD;
    req->sec_lvl = 0;
    req->prf_task_id = TASK_ID_BASC;
    req->app_task = TASK_APP;

    /* Send the message */
    APP_MsgSend(req);
}

void APP_BascEnableReq(uint16_t conhdl)
{
    struct basc_enable_req *msg =
        KE_MSG_ALLOC(BASC_ENABLE_REQ, KE_BUILD_ID(prf_get_task_from_id(TASK_ID_BASC), CONHDL2CONIDX(conhdl)), TASK_APP,
                     basc_enable_req);

    msg->con_type = PRF_CON_DISCOVERY;

    /* Send the message */
    APP_MsgSend(msg);
}

void APP_BascReadInfoReq(uint16_t conhdl, uint8_t info, uint8_t bas_nb)
{
    struct basc_read_info_req *msg =
        KE_MSG_ALLOC(BASC_READ_INFO_REQ, KE_BUILD_ID(prf_get_task_from_id(TASK_ID_BASC), CONHDL2CONIDX(conhdl)),
                     TASK_APP, basc_read_info_req);
    msg->info = info;
    msg->bas_nb = bas_nb;
    /* Send the message */
    APP_MsgSend(msg);
}

void APP_BascBattLevelNtfCfgReq(uint16_t conhdl, uint16_t ntf_cfg, uint8_t bas_nb)
{
    struct basc_batt_level_ntf_cfg_req *msg = KE_MSG_ALLOC(
        BASC_BATT_LEVEL_NTF_CFG_REQ, KE_BUILD_ID(prf_get_task_from_id(TASK_ID_BASC), CONHDL2CONIDX(conhdl)), TASK_APP,
        basc_batt_level_ntf_cfg_req);

    msg->bas_nb = bas_nb;
    msg->ntf_cfg = ntf_cfg;

    APP_MsgSend(msg);
}

int APP_BascEnableRspHandler(ke_msg_id_t const msgid,
                             struct basc_enable_rsp *param,
                             ke_task_id_t const dest_id,
                             ke_task_id_t const src_id)
{
    QPRINTF("BASC enable confirmation status: 0x%x.\r\n", param->status);

    if (param->status == GAP_ERR_NO_ERROR)
    {
        g_AppBascEnv.bas_nb[KE_IDX_GET(src_id)] = param->bas_nb;
    }

    return (KE_MSG_CONSUMED);
}

int APP_BascReadInfoRspHandler(ke_msg_id_t const msgid,
                               struct basc_read_info_rsp *param,
                               ke_task_id_t const dest_id,
                               ke_task_id_t const src_id)
{
    QPRINTF("BASC read bas_num %d info rsp status: 0x%x.\r\n", param->bas_nb, param->status);

    if (param->status == GAP_ERR_NO_ERROR)
    {
        switch (param->info)
        {
            /* Battery Level value */
            case BASC_BATT_LVL_VAL:
            {
                QPRINTF("batt level %d\r\n", param->data.batt_level);
            }
            break;
            /* Battery Level Client Characteristic Configuration */
            case BASC_NTF_CFG:
            {
                QPRINTF("Battery Level CCC: 0x%02x\r\n", param->data.ntf_cfg);
            }
            break;
            /* Battery Level Characteristic Presentation Format */
            case BASC_BATT_LVL_PRES_FORMAT:
            {
                struct prf_char_pres_fmt *fmt = &param->data.char_pres_format;
                QPRINTF("Battery Level CPF: namespace is 0x%02x, description is 0x%04x\r\n", fmt->name_space,
                        fmt->description);
            }
            break;
            default:
            {
                ASSERT_ERR(0);
            }
            break;
        }
    }

    return (KE_MSG_CONSUMED);
}

int APP_BascBattLevelIndHandler(ke_msg_id_t const msgid,
                                struct basc_batt_level_ind *param,
                                ke_task_id_t const dest_id,
                                ke_task_id_t const src_id)
{
    QPRINTF("Connection handle %d battery %d's level is %d\r\n", CONIDX2CONHDL(KE_IDX_GET(src_id)), param->bas_nb,
            param->batt_level);
    return (KE_MSG_CONSUMED);
}

int APP_BascBattLevelNtfCfgRsp_handler(ke_msg_id_t const msgid,
                                       struct basc_batt_level_ntf_cfg_rsp *param,
                                       ke_task_id_t const dest_id,
                                       ke_task_id_t const src_id)
{
    return (KE_MSG_CONSUMED);
}

#endif

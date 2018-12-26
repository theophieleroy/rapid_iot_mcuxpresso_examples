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
#if (BLE_BATT_SERVER)
#include "battery.h"

/*******************************************************************************
 * Variables
 ******************************************************************************/
struct app_bass_env_tag g_AppBassEnv;

const struct ke_msg_handler s_appBassMsgHandlerList[] = {
    {BASS_BATT_LEVEL_NTF_CFG_IND, (ke_msg_func_t)APP_BassBattLevelNtfCfgIndHandler},
    {BASS_BATT_LEVEL_UPD_RSP, (ke_msg_func_t)APP_BassBattLevelUpdRspHandler},
    {BASS_ENABLE_RSP, (ke_msg_func_t)APP_BassEnableRspHandler},
};

const struct ke_state_handler g_AppBassTableHandler = KE_STATE_HANDLER(s_appBassMsgHandlerList);

/*******************************************************************************
 * Code
 ******************************************************************************/
void APP_BassAddProfileTask(void)
{
    struct bass_db_cfg *db_cfg;
    struct gapm_profile_task_add_cmd *req = KE_MSG_ALLOC_DYN(GAPM_PROFILE_TASK_ADD_CMD, TASK_GAPM, TASK_APP,
                                                             gapm_profile_task_add_cmd, sizeof(struct bass_db_cfg));
    /* Fill message */
    req->operation = GAPM_PROFILE_TASK_ADD;
    req->sec_lvl = 0; /*mono instantiated, no forced 16 bytes EKS, no auth, enable the service */
    req->prf_task_id = TASK_ID_BASS;
    req->app_task = TASK_APP;
    req->start_hdl = 0;

    /* Set parameters */
    db_cfg = (struct bass_db_cfg *)req->param;

    /* Add a BAS instance */
    db_cfg->bas_nb = BASS_NB_BAS_INSTANCES_MAX;
    for (int i = 0; i < BASS_NB_BAS_INSTANCES_MAX; i++)
    {
        /* Sending of notifications is supported */
        db_cfg->features[i] = BAS_BATT_LVL_NTF_SUP;
    }
    if (BASS_NB_BAS_INSTANCES_MAX > 1)
    {
        for (int i = 0; i < BASS_NB_BAS_INSTANCES_MAX; i++)
        {
            db_cfg->batt_level_pres_format[i].name_space = 1; /* Bluetooth */
            db_cfg->batt_level_pres_format[i].description = i + 1;
        }
    }

    /* Send the message */
    APP_MsgSend(req);
}

void APP_BassEnableReq(uint16_t conhdl, uint8_t ntf_cfg, uint8_t *old_batt_lvl)
{
    /* Allocate the message */
    struct bass_enable_req *req =
        KE_MSG_ALLOC(BASS_ENABLE_REQ, prf_get_task_from_id(TASK_ID_BASS), TASK_APP, bass_enable_req);

    /* Fill in the parameter structure */
    req->conidx = CONHDL2CONIDX(conhdl);

    /* NTF initial status - Disabled */
    req->ntf_cfg = PRF_CLI_STOP_NTFIND;
    for (int i = 0; i < BASS_NB_BAS_INSTANCES_MAX; i++)
        req->old_batt_lvl[i] = old_batt_lvl[i];

    /* Send the message */
    APP_MsgSend(req);
}

void APP_BassBattLevelUpdReq(uint8_t bas_instance, uint8_t batt_lvl)
{
    ASSERT_ERR(batt_lvl <= BAS_BATTERY_LVL_MAX);

    /* Allocate the message */
    struct bass_batt_level_upd_req *req =
        KE_MSG_ALLOC(BASS_BATT_LEVEL_UPD_REQ, prf_get_task_from_id(TASK_ID_BASS), TASK_APP, bass_batt_level_upd_req);

    /* Fill in the parameter structure */
    req->bas_instance = bas_instance;
    req->batt_level = batt_lvl;

    /* Send the message */
    APP_MsgSend(req);
}

int APP_BassBattLevelNtfCfgIndHandler(ke_msg_id_t const msgid,
                                      struct bass_batt_level_ntf_cfg_ind const *param,
                                      ke_task_id_t const dest_id,
                                      ke_task_id_t const src_id)
{
    g_AppBassEnv.ntf_cfg[param->conidx] = param->ntf_cfg;
    return (KE_MSG_CONSUMED);
}

int APP_BassBattLevelUpdRspHandler(ke_msg_id_t const msgid,
                                   struct bass_batt_level_upd_rsp const *param,
                                   ke_task_id_t const dest_id,
                                   ke_task_id_t const src_id)
{
    return (KE_MSG_CONSUMED);
}

int APP_BassEnableRspHandler(ke_msg_id_t const msgid,
                             struct bass_enable_rsp const *param,
                             ke_task_id_t const dest_id,
                             ke_task_id_t const src_id)
{
    return (KE_MSG_CONSUMED);
}

int APP_BassBattLvlChkTimerHandler(ke_msg_id_t const msgid,
                                   void const *param,
                                   ke_task_id_t const dest_id,
                                   ke_task_id_t const src_id)
{
    ke_timer_set(APP_MSG_BASS_BATT_LVL_CHK_TIMER, TASK_APP, CFG_BASS_BATT_LVL_CHK_DUR);

    for (int i = 0; i < BASS_NB_BAS_INSTANCES_MAX; i++)
    {
        uint8_t current_lvl = BATT_GetBatteryLevel(i); /* WJ: call functin in battery.c */
        if (g_AppBassEnv.batt_lvl[i] != current_lvl)
        {
            g_AppBassEnv.batt_lvl[i] = current_lvl;
            APP_BassBattLevelUpdReq(i, current_lvl);
        }
    }
    return (KE_MSG_CONSUMED);
}

#endif /* BLE_BATT_SERVER */

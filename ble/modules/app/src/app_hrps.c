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
#if (BLE_HR_SENSOR)

/*******************************************************************************
 * Variables
 ******************************************************************************/
const struct ke_msg_handler s_appHrpsMsgHandlerList[] = {
    {HRPS_ENABLE_RSP, (ke_msg_func_t)APP_HrpsEnableRspHandler},
    {HRPS_MEAS_SEND_RSP, (ke_msg_func_t)APP_HrpsMeasSendRspHandler},
    {HRPS_CFG_INDNTF_IND, (ke_msg_func_t)APP_HrpsCfgIndntfIndHandler},
    {HRPS_ENERGY_EXP_RESET_IND, (ke_msg_func_t)APP_HrpsEnergyExpResetIndHandler},

};

const struct ke_state_handler g_AppHrpsTableHandler = KE_STATE_HANDLER(s_appHrpsMsgHandlerList);
struct app_hrps_env_tag g_AppHrpsEnv;

/*******************************************************************************
 * Code
 ******************************************************************************/
void APP_HrpsAddProfileTask(void)
{
    struct hrps_db_cfg *db_cfg;
    struct gapm_profile_task_add_cmd *req = KE_MSG_ALLOC_DYN(GAPM_PROFILE_TASK_ADD_CMD, TASK_GAPM, TASK_APP,
                                                             gapm_profile_task_add_cmd, sizeof(struct hrps_db_cfg));

    /* Fill message */
    req->operation = GAPM_PROFILE_TASK_ADD;
    req->sec_lvl = 0;
    req->prf_task_id = TASK_ID_HRPS;
    req->app_task = TASK_APP;
    req->start_hdl = 0;

    /* Set parameters */
    db_cfg = (struct hrps_db_cfg *)req->param;
    db_cfg->features = CFG_HRPS_FEATURE;
    db_cfg->body_sensor_loc = CFG_HRPS_SENSOR_LOC;

    g_AppHrpsEnv.meas_intv = HRPS_INIT_MEAS_INTVL;
    g_AppHrpsEnv.energy_exp_val = HRPS_INIT_ENERGY_EXP_VAL;

    /* Send the message */
    APP_MsgSend(req);
}

void APP_HrpsEnableReq(uint16_t conhdl, uint16_t hr_meas_ntf)
{
    struct hrps_enable_req *msg =
        KE_MSG_ALLOC(HRPS_ENABLE_REQ, KE_BUILD_ID(prf_get_task_from_id(TASK_ID_HRPS), CONHDL2CONIDX(conhdl)), TASK_APP,
                     hrps_enable_req);

    msg->conidx = CONHDL2CONIDX(conhdl);
    msg->hr_meas_ntf = hr_meas_ntf;

    /* Send the message */
    APP_MsgSend(msg);
}

int APP_HrpsEnableRspHandler(ke_msg_id_t const msgid,
                             struct hrps_enable_rsp *param,
                             ke_task_id_t const dest_id,
                             ke_task_id_t const src_id)
{
    QPRINTF("APP_HrpsEnableRspHandler status is :0x%x.\r\n", param->status);

    return (KE_MSG_CONSUMED);
}

void APP_HrpsMeasSendReq(uint16_t conhdl, struct hrs_hr_meas *meas_val)
{
    struct hrps_meas_send_req *msg =
        KE_MSG_ALLOC(HRPS_MEAS_SEND_REQ, KE_BUILD_ID(prf_get_task_from_id(TASK_ID_HRPS), CONHDL2CONIDX(conhdl)),
                     TASK_APP, hrps_meas_send_req);

    msg->meas_val = *meas_val;

    /* Send the message */
    APP_MsgSend(msg);
}

int APP_HrpsMeasSendRspHandler(ke_msg_id_t const msgid,
                               struct hrps_meas_send_rsp *param,
                               ke_task_id_t const dest_id,
                               ke_task_id_t const src_id)
{
    QPRINTF("APP_HrpsMeasSendRspHandler status :0x%x.\r\n", param->status);

    return (KE_MSG_CONSUMED);
}

int APP_HrpsCfgIndntfIndHandler(ke_msg_id_t const msgid,
                                struct hrps_cfg_indntf_ind *param,
                                ke_task_id_t const dest_id,
                                ke_task_id_t const src_id)
{
    QPRINTF("APP_HrpsCfgIndntfIndHandler cfg_val is :0x%x.\r\n", param->cfg_val);

    return (KE_MSG_CONSUMED);
}

int APP_HrpsEnergyExpResetIndHandler(ke_msg_id_t const msgid,
                                     struct hrps_energy_exp_reset_ind *param,
                                     ke_task_id_t const dest_id,
                                     ke_task_id_t const src_id)
{
    QPRINTF("APP_HrpsEnergyExpResetIndHandler conidx :0x%x.\r\n", param->conidx);
    g_AppHrpsEnv.energy_exp_val = HRPS_INIT_ENERGY_EXP_VAL;

    return (KE_MSG_CONSUMED);
}

#endif

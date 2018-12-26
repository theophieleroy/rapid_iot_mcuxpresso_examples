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
#if (BLE_CP_SENSOR)

/*******************************************************************************
 * Variables
 ******************************************************************************/
struct app_cpps_env_tag g_AppCppsEnv;

const struct ke_msg_handler s_appCppsMsgHandlerList[] = {
    {CPPS_ENABLE_RSP, (ke_msg_func_t)APP_CppsEnableRspHandler},
    {CPPS_GET_ADV_DATA_RSP, (ke_msg_func_t)APP_CppsGetAdvDataRspHandler},
    {CPPS_NTF_CP_MEAS_RSP, (ke_msg_func_t)APP_CppsNtfCpMeasRspHandler},
    {CPPS_NTF_CP_VECTOR_RSP, (ke_msg_func_t)APP_CppsNtfCpVectorRspHandler},
    {CPPS_CTNL_PT_REQ_IND, (ke_msg_func_t)APP_CppsCtnlPtReqIndHandler},
    {CPPS_CFG_NTFIND_IND, (ke_msg_func_t)APP_CppsCfgNtfindIndHandler},
    {CPPS_VECTOR_CFG_REQ_IND, (ke_msg_func_t)APP_CppsVectorCfgReqIndHandler},
    {CPPS_CMP_EVT, (ke_msg_func_t)APP_CppsCmpEvtHandler},
};

const struct ke_state_handler g_AppCppsTableHandler = KE_STATE_HANDLER(s_appCppsMsgHandlerList);

/*******************************************************************************
 * Code
 ******************************************************************************/
void APP_CppsAddProfileTask(void)
{
    struct cpps_db_cfg *db_cfg;
    struct gapm_profile_task_add_cmd *req = KE_MSG_ALLOC_DYN(GAPM_PROFILE_TASK_ADD_CMD, TASK_GAPM, TASK_APP,
                                                             gapm_profile_task_add_cmd, sizeof(struct cpps_db_cfg));

    /* Fill message */
    req->operation = GAPM_PROFILE_TASK_ADD;
    req->sec_lvl = 0;
    req->prf_task_id = TASK_ID_CPPS;
    req->app_task = TASK_APP;
    req->start_hdl = 0;

    /* Set configuration parameters */
    db_cfg = (struct cpps_db_cfg *)req->param;
    db_cfg->cp_feature = CFG_CPPS_FEATURE;
    db_cfg->prfl_config = CFG_CPPS_PROFILE;
    db_cfg->sensor_loc = CFG_CPPS_SENSOR_LOC;
    db_cfg->wheel_rev = CPPS_INIT_WHEEL_REVOL;

    /*Initialize interval value  */
    g_AppCppsEnv.meas_intv = CPPS_INIT_MEAS_INTVL;
    g_AppCppsEnv.braodcast_state = APP_BROADCAST_IDLE;

    /* Send the message */
    APP_MsgSend(req);
}

void APP_CppsEnableReq(uint16_t conhdl, uint16_t prfl_ntf_ind_cfg)
{
    struct cpps_enable_req *msg =
        KE_MSG_ALLOC(CPPS_ENABLE_REQ, KE_BUILD_ID(prf_get_task_from_id(TASK_ID_CPPS), CONHDL2CONIDX(conhdl)), TASK_APP,
                     cpps_enable_req);

    msg->conidx = CONIDX2CONHDL(conhdl);
    msg->prfl_ntf_ind_cfg = prfl_ntf_ind_cfg;

    /* Send the message */
    APP_MsgSend(msg);
}

int APP_CppsEnableRspHandler(ke_msg_id_t const msgid,
                             struct cpps_enable_rsp *param,
                             ke_task_id_t const dest_id,
                             ke_task_id_t const src_id)
{
    QPRINTF("APP_CppsEnableRspHandler status :0x%x.\r\n", param->status);

    return (KE_MSG_CONSUMED);
}

void APP_CppsGetAdvDataReq(uint16_t conhdl, struct cpp_cp_meas *parameters)
{
    struct cpps_get_adv_data_req *msg =
        KE_MSG_ALLOC(CPPS_GET_ADV_DATA_REQ, KE_BUILD_ID(prf_get_task_from_id(TASK_ID_CPPS), CONHDL2CONIDX(conhdl)),
                     TASK_APP, cpps_get_adv_data_req);

    msg->parameters = *parameters;
    /* Send the message */
    APP_MsgSend(msg);
}

int APP_CppsGetAdvDataRspHandler(ke_msg_id_t const msgid,
                                 struct cpps_get_adv_data_rsp *param,
                                 ke_task_id_t const dest_id,
                                 ke_task_id_t const src_id)
{
    QPRINTF("APP_CppsGetAdvDataRspHandler status :0x%x.\r\n", param->status);

    if (param->status == GAP_ERR_NO_ERROR)
    {
        APP_CppsGetAdvDataRspCallback(param->data_len, param->adv_data);
    }
    else
    {
        g_AppCppsEnv.braodcast_state = APP_BROADCAST_IDLE;
    }

    return (KE_MSG_CONSUMED);
}

void APP_CppsNtfCpMeasReq(uint16_t conhdl, struct cpp_cp_meas *parameters)
{
    struct cpps_ntf_cp_meas_req *msg =
        KE_MSG_ALLOC(CPPS_NTF_CP_MEAS_REQ, KE_BUILD_ID(prf_get_task_from_id(TASK_ID_CPPS), CONHDL2CONIDX(conhdl)),
                     TASK_APP, cpps_ntf_cp_meas_req);
    msg->parameters = *parameters;

    /* Send the message */
    APP_MsgSend(msg);
}

int APP_CppsNtfCpMeasRspHandler(ke_msg_id_t const msgid,
                                struct cpps_ntf_cp_meas_rsp *param,
                                ke_task_id_t const dest_id,
                                ke_task_id_t const src_id)
{
    QPRINTF("APP_CppsNtfCpMeasRspHandler status :0x%x.\r\n", param->status);
    return (KE_MSG_CONSUMED);
}

void APP_CppsNtfCpVectorReq(uint16_t conhdl, struct cpp_cp_vector *parameters)
{
    struct cpps_ntf_cp_vector_req *msg =
        KE_MSG_ALLOC(CPPS_NTF_CP_VECTOR_REQ, KE_BUILD_ID(prf_get_task_from_id(TASK_ID_CPPS), CONHDL2CONIDX(conhdl)),
                     TASK_APP, cpps_ntf_cp_vector_req);
    msg->parameters = *parameters;

    /* Send the message */
    APP_MsgSend(msg);
}

int APP_CppsNtfCpVectorRspHandler(ke_msg_id_t const msgid,
                                  struct cpps_ntf_cp_vector_rsp *param,
                                  ke_task_id_t const dest_id,
                                  ke_task_id_t const src_id)
{
    QPRINTF("APP_CppsNtfCpVectorRspHandler status :0x%x.\r\n", param->status);
    return (KE_MSG_CONSUMED);
}

int APP_CppsCtnlPtReqIndHandler(ke_msg_id_t const msgid,
                                struct cpps_ctnl_pt_req_ind *param,
                                ke_task_id_t const dest_id,
                                ke_task_id_t const src_id)
{
    APP_CppsCtnlPtReqIndCallback((void *)param);
    return (KE_MSG_CONSUMED);
}

void APP_CppsCtnlPtCfm(struct cpps_ctnl_pt_cfm *cfm)
{
    uint8_t conidx = cfm->conidx;
    struct cpps_ctnl_pt_cfm *msg = KE_MSG_ALLOC(
        CPPS_CTNL_PT_CFM, KE_BUILD_ID(prf_get_task_from_id(TASK_ID_CPPS), conidx), TASK_APP, cpps_ctnl_pt_cfm);

    msg->conidx = cfm->conidx;
    msg->op_code = cfm->op_code;
    msg->status = cfm->status;
    msg->value = cfm->value;

    /* Send the message */
    APP_MsgSend(msg);
}

int APP_CppsCfgNtfindIndHandler(ke_msg_id_t const msgid,
                                struct cpps_cfg_ntfind_ind *param,
                                ke_task_id_t const dest_id,
                                ke_task_id_t const src_id)
{
    struct cpp_cp_meas parameters;
    struct cpps_env_tag *cpps_env = PRF_ENV_GET(CPPS, cpps);

    QPRINTF("APP_CppsCfgNtfindIndHandler char_code: 0x%x ntf_cfg:0x%x \r\n", param->char_code, param->ntf_cfg);

    if (param->char_code == CPP_PRF_CFG_FLAG_SP_MEAS_NTF)
    {
        if (cpps_env->broadcast_enabled)
        {
            if (g_AppCppsEnv.braodcast_state == APP_BROADCAST_IDLE)
            {
                parameters.flags = CFG_CPPS_BROADCAST_DATA_FLAG;
                parameters.inst_power = CPPS_INSTANTANEOUS_POWER;
                parameters.pedal_power_balance = CPPS_PEDAL_POWER_BALANCE;
                parameters.accum_torque = CPPS_TORQUE_INCREMENT;
                parameters.cumul_crank_rev = CPPS_CRANK_EVENT_TIME_INCREMENT;
                parameters.last_crank_evt_time = CPPS_WHEEL_EVENT_TIME_INCREMENT;

                g_AppCppsEnv.braodcast_state = APP_BROADCAST_READY;

                /*! @brief send request to get adv data */
                APP_CppsGetAdvDataReq(CONIDX2CONHDL(param->conidx), &parameters);
            }
        }
        else
        {
            if (g_AppCppsEnv.braodcast_state == APP_BROADCAST_START)
            {
                g_AppCppsEnv.braodcast_state = APP_BROADCAST_IDLE;
                APP_GapmStopAdvertising();
            }
        }
    }

    return (KE_MSG_CONSUMED);
}

int APP_CppsVectorCfgReqIndHandler(ke_msg_id_t const msgid,
                                   struct cpps_vector_cfg_req_ind *param,
                                   ke_task_id_t const dest_id,
                                   ke_task_id_t const src_id)
{
    QPRINTF("APP_CppsVectorCfgReqIndHandler char_code: 0x%x ntf_cfg:0x%x\r\n", param->char_code, param->ntf_cfg);

    struct cpps_vector_cfg_cfm *msg =
        KE_MSG_ALLOC(CPPS_VECTOR_CFG_CFM, KE_BUILD_ID(prf_get_task_from_id(TASK_ID_CPPS), param->conidx), TASK_APP,
                     cpps_vector_cfg_cfm);

    msg->conidx = param->conidx;
    msg->ntf_cfg = param->ntf_cfg;
    msg->status = GAP_ERR_NO_ERROR;

    /* Send the message */
    APP_MsgSend(msg);

    return (KE_MSG_CONSUMED);
}

int APP_CppsCmpEvtHandler(ke_msg_id_t const msgid,
                          struct cpps_cmp_evt *param,
                          ke_task_id_t const dest_id,
                          ke_task_id_t const src_id)
{
    QPRINTF("APP_CppsCmpEvtHandler status :0x%x.\r\n", param->status);

    switch (param->operation)
    {
        case CPPS_NTF_MEAS_OP_CODE:
            QPRINTF("The operation Code:Send CP Measurement.\r\n");
            break;

        case CPPS_NTF_VECTOR_OP_CODE:
            QPRINTF("The operation Code:Send Vector.\r\n");
            break;

        default:
            break;
    }

    return (KE_MSG_CONSUMED);
}

#endif

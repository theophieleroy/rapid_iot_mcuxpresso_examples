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
#include "prf_utils.h"
#if (BLE_RSC_SENSOR)

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static void APP_RscpsScCtnlPtCfm(struct rscps_sc_ctnl_pt_cfm *cfm);

static int APP_RscpsEnbleRspHandler(ke_msg_id_t const msgid,
                                    struct rscps_enable_rsp *param,
                                    ke_task_id_t const dest_id,
                                    ke_task_id_t const src_id);

static int APP_RscpsNtfRscMeasRspHandler(ke_msg_id_t const msgid,
                                         struct rscps_ntf_rsc_meas_rsp *param,
                                         ke_task_id_t const dest_id,
                                         ke_task_id_t const src_id);

static int APP_RscpScCntlPtReqIndHandler(ke_msg_id_t const msgid,
                                         struct rscps_sc_ctnl_pt_req_ind *param,
                                         ke_task_id_t const dest_id,
                                         ke_task_id_t const src_id);

static int APP_RscpsCfgNtfindIndHandler(ke_msg_id_t const msgid,
                                        struct rscps_cfg_ntfind_ind *param,
                                        ke_task_id_t const dest_id,
                                        ke_task_id_t const src_id);

static int APP_RscpsCmpEvtHandler(ke_msg_id_t const msgid,
                                  struct rscps_cmp_evt *param,
                                  ke_task_id_t const dest_id,
                                  ke_task_id_t const src_id);

/*******************************************************************************
 * Variables
 ******************************************************************************/
const struct ke_msg_handler s_appRscpsMsgHandlerList[] = {
    {RSCPS_ENABLE_RSP, (ke_msg_func_t)APP_RscpsEnbleRspHandler},
    {RSCPS_NTF_RSC_MEAS_RSP, (ke_msg_func_t)APP_RscpsNtfRscMeasRspHandler},
    {RSCPS_SC_CTNL_PT_REQ_IND, (ke_msg_func_t)APP_RscpScCntlPtReqIndHandler},
    {RSCPS_CFG_NTFIND_IND, (ke_msg_func_t)APP_RscpsCfgNtfindIndHandler},
    {RSCPS_CMP_EVT, (ke_msg_func_t)APP_RscpsCmpEvtHandler},

};

const struct ke_state_handler g_AppRscpsTableHandler = KE_STATE_HANDLER(s_appRscpsMsgHandlerList);

/*******************************************************************************
 * Code
 ******************************************************************************/
void APP_RscpsAddProfileTask(void)
{
    struct rscps_db_cfg *db_cfg;
    struct gapm_profile_task_add_cmd *req = KE_MSG_ALLOC_DYN(GAPM_PROFILE_TASK_ADD_CMD, TASK_GAPM, TASK_APP,
                                                             gapm_profile_task_add_cmd, sizeof(struct rscps_db_cfg));
    /* Fill message */
    req->operation = GAPM_PROFILE_TASK_ADD;
    req->prf_task_id = TASK_ID_RSCPS;
    req->app_task = TASK_APP;
    req->start_hdl = 0;
    /* Set parameters */
    db_cfg = (struct rscps_db_cfg *)req->param;
    db_cfg->rsc_feature = CFG_RSC_FEATURE;
    db_cfg->sensor_loc_supp = CFG_RSC_SENSOR_LOC_SUPP;
    db_cfg->sensor_loc = CFG_RSC_SENSOR_LOC;
    /* Send the message */
    APP_MsgSend(req);
}

void APP_RscpsEnableReq(uint16_t conhdl, uint16_t rsc_meas_ntf_cfg, uint16_t sc_ctnl_pt_ntf_cfg)
{
    uint8_t conidx = CONHDL2CONIDX(conhdl);
    struct rscps_enable_req *msg =
        KE_MSG_ALLOC(RSCPS_ENABLE_REQ, prf_get_task_from_id(TASK_ID_RSCPS), TASK_APP, rscps_enable_req);
    msg->conidx = conidx;
    msg->rsc_meas_ntf_cfg = rsc_meas_ntf_cfg;
    msg->sc_ctnl_pt_ntf_cfg = sc_ctnl_pt_ntf_cfg;
    APP_MsgSend(msg);
}

/*!
 * @brief    This API message informs the application about the enabling operation.
 *
 * @param[in] msgid     RSCPS_ENABLE_RSP
 * @param[in] param     Pointer to the struct rscps_enable_rsp
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_RSCPS
 *
 * @return If the message was consumed or not.
 * @description
 * This API message informs the application about the enabling operation.
 */
static int APP_RscpsEnbleRspHandler(ke_msg_id_t const msgid,
                                    struct rscps_enable_rsp *param,
                                    ke_task_id_t const dest_id,
                                    ke_task_id_t const src_id)
{
    QPRINTF("rscps enable status = %x,conidx=%x.\r\n", param->status, param->conidx);
    return (KE_MSG_CONSUMED);
}

void APP_RscpsNtfRscMeasReq(uint16_t conhdl,
                            uint8_t flags,
                            uint16_t inst_cad,
                            uint16_t inst_speed,
                            uint16_t inst_stride_len,
                            uint16_t total_dist)
{
    struct rscps_ntf_rsc_meas_req *msg =
        KE_MSG_ALLOC(RSCPS_NTF_RSC_MEAS_REQ, KE_BUILD_ID(prf_get_task_from_id(TASK_ID_RSCPS), CONHDL2CONIDX(conhdl)),
                     TASK_APP, rscps_ntf_rsc_meas_req);
    msg->flags = flags;
    msg->inst_cad = inst_cad;
    msg->inst_speed = inst_speed;
    msg->inst_stride_len = inst_stride_len;
    msg->total_dist = total_dist;
    APP_MsgSend(msg);
}

/*!
 * @brief    This API message informs the application about the enabling operation.
 *
 * @param[in] msgid     RSCPS_CMP_EVT
 * @param[in] param     Pointer to the struct rscps_enable_rsp
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_RSCPS
 *
 * @return If the message was consumed or not.
 * @description
 * This API message informs the application about the enabling operation.
 */
static int APP_RscpsNtfRscMeasRspHandler(ke_msg_id_t const msgid,
                                         struct rscps_ntf_rsc_meas_rsp *param,
                                         ke_task_id_t const dest_id,
                                         ke_task_id_t const src_id)
{
    QPRINTF("rscps_ntf_rsc_meas_rsp status=%x.\r\n", param->status);
    return (KE_MSG_CONSUMED);
}

/*!
 * @brief  This message is sent by the application as a response to the RSCPS_SC_CTNL_PT_REQ_IND message. It contains
 * the value requested by the profile.
 *
 * @param[in] rscps_sc_ctnl_pt_cfm_value     Control Point Confirm Value.
 *
 * @response RSCPS_CMP_EVT
 * @description
 * This message is sent by the application as a response to the RSCPS_SC_CTNL_PT_REQ_IND message. It contains
 * the value requested by the profile.
 */
static void APP_RscpsScCtnlPtCfm(struct rscps_sc_ctnl_pt_cfm *cfm)
{
    struct rscps_sc_ctnl_pt_cfm *msg =
        KE_MSG_ALLOC(RSCPS_SC_CTNL_PT_CFM, prf_get_task_from_id(TASK_ID_RSCPS), TASK_APP, rscps_sc_ctnl_pt_cfm);
    msg->conidx = cfm->conidx;
    msg->op_code = cfm->op_code;
    msg->status = cfm->status;
    switch (cfm->op_code)
    {
        case RSCPS_CTNL_PT_CUMUL_VAL_OP_CODE:
            break;
        case RSCPS_CTNL_PT_UPD_LOC_OP_CODE:
            msg->value.sensor_loc = cfm->value.sensor_loc;
            break;
        case RSCPS_CTNL_PT_SUPP_LOC_OP_CODE:
            msg->value.supp_sensor_loc = cfm->value.supp_sensor_loc;
            break;
    }
    APP_MsgSend(msg);
}

/*!
 * @brief    The message is sent to the application when the SC Control Point characteristic is written by the peer
 * device.
 *
 * @param[in] msgid     RSCPS_SC_CTNL_PT_REQ_IND
 * @param[in] param     Pointer to the struct rscps_sc_ctnl_pt_req_ind
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_RSCPS
 *
 * @return If the message was consumed or not.
 * @description
 * The message is sent to the application when the SC Control Point characteristic is written by the peer device.
 */
static int APP_RscpScCntlPtReqIndHandler(ke_msg_id_t const msgid,
                                         struct rscps_sc_ctnl_pt_req_ind *param,
                                         ke_task_id_t const dest_id,
                                         ke_task_id_t const src_id)
{
    struct rscps_sc_ctnl_pt_cfm rscps_sc_ctnl_pt_cfm_value;
    rscps_sc_ctnl_pt_cfm_value.conidx = param->conidx;
    struct rscps_env_tag *rscps_env = PRF_ENV_GET(RSCPS, rscps);
    switch (param->op_code)
    {
        case RSCP_CTNL_PT_OP_SET_CUMUL_VAL:
            rscps_sc_ctnl_pt_cfm_value.status = RSCP_CTNL_PT_RESP_SUCCESS;
            rscps_sc_ctnl_pt_cfm_value.op_code = RSCPS_CTNL_PT_CUMUL_VAL_OP_CODE;
            break;

        case RSCP_CTNL_PT_OP_START_CALIB:
            rscps_sc_ctnl_pt_cfm_value.op_code = RSCPS_CTNL_PT_START_CAL_OP_CODE;
            rscps_sc_ctnl_pt_cfm_value.status = APP_RscpsCntlPtStartCalibCallback();
            break;

        case RSCP_CTNL_PT_OP_UPD_LOC:
            rscps_sc_ctnl_pt_cfm_value.status = RSCP_CTNL_PT_RESP_SUCCESS;
            rscps_sc_ctnl_pt_cfm_value.op_code = RSCPS_CTNL_PT_UPD_LOC_OP_CODE;
            rscps_sc_ctnl_pt_cfm_value.value.sensor_loc = param->value.sensor_loc;
            break;

        case RSCP_CTNL_PT_OP_REQ_SUPP_LOC:
            rscps_sc_ctnl_pt_cfm_value.status = RSCP_CTNL_PT_RESP_SUCCESS;
            rscps_sc_ctnl_pt_cfm_value.op_code = RSCPS_CTNL_PT_SUPP_LOC_OP_CODE;
            rscps_sc_ctnl_pt_cfm_value.value.supp_sensor_loc = CFG_RSC_SUPP_SENSOR_LOC;
            break;

        default:
            break;
    }
    APP_RscpsScCtnlPtCfm(&rscps_sc_ctnl_pt_cfm_value);
    return (KE_MSG_CONSUMED);
}

/*!
 * @brief    This message is sent to the application each time a peer device successfully writes the Client
 * Characteristic Configuration descriptor of either the RSC Measurement characteristic or the SC Control Point
 * characteristic.
 *
 * @param[in] msgid     RSCPS_CFG_NTFIND_IND
 * @param[in] param     Pointer to the struct rscps_cfg_ntfind_ind
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_RSCPS
 *
 * @return If the message was consumed or not.
 * @description
 * This message is sent to the application each time a peer device successfully writes the Client Characteristic
 * Configuration descriptor of either the RSC Measurement characteristic or the SC Control Point characteristic.
 */
static int APP_RscpsCfgNtfindIndHandler(ke_msg_id_t const msgid,
                                        struct rscps_cfg_ntfind_ind *param,
                                        ke_task_id_t const dest_id,
                                        ke_task_id_t const src_id)
{
    switch (param->char_code)
    {
        case RSCP_RSCS_RSC_MEAS_CHAR:
            QPRINTF("RSC Measurement ntf set 0x%x.\r\n", param->ntf_cfg);
            break;
        case RSCP_RSCS_SC_CTNL_PT_CHAR:
            QPRINTF("RSC Control Point ind set 0x%x.\r\n", param->ntf_cfg);
            break;
    }
    return (KE_MSG_CONSUMED);
}

/*!
 * @brief    The API message is used by the RSCPS task to inform the sender of a command that the procedure is over
 * and contains the status of the procedure.
 *
 * @param[in] msgid     RSCPS_CMP_EVT
 * @param[in] param     Pointer to the struct rscps_cmp_evt
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_RSCPS
 *
 * @return If the message was consumed or not.
 * @description
 * The API message is used by the RSCPS task to inform the sender of a command that the procedure is over
 * and contains the status of the procedure.
 */
static int APP_RscpsCmpEvtHandler(ke_msg_id_t const msgid,
                                  struct rscps_cmp_evt *param,
                                  ke_task_id_t const dest_id,
                                  ke_task_id_t const src_id)
{
    QPRINTF("Cmp evt!op_code= 0x%x.\r\n", param->operation);
    return (KE_MSG_CONSUMED);
}

#endif

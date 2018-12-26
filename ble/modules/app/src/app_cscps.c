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
 * @addtogroup APP_CSCPS_API
 * @{
 */

#include "app_ble.h"
#if (BLE_CSC_SENSOR)

/*******************************************************************************
 * Code
 ******************************************************************************/
void APP_CscpsAddProfileTask(void)
{
    struct cscps_db_cfg *db_cfg;
    struct gapm_profile_task_add_cmd *req = KE_MSG_ALLOC_DYN(GAPM_PROFILE_TASK_ADD_CMD, TASK_GAPM, TASK_APP,
                                                             gapm_profile_task_add_cmd, sizeof(struct cscps_db_cfg));
    /* Fill message */
    req->operation = GAPM_PROFILE_TASK_ADD;
    req->prf_task_id = TASK_ID_CSCPS;
    req->app_task = TASK_APP;
    req->start_hdl = 0;
    /* Set parameters */
    db_cfg = (struct cscps_db_cfg *)req->param;
    db_cfg->csc_feature = CFG_CSC_FEATURE;
    db_cfg->sensor_loc_supp = CFG_CSC_SENSOR_LOC_SUPP;
    db_cfg->sensor_loc = CFG_CSC_SENSOR_LOC;
    db_cfg->wheel_rev = CFG_CSC_WEEL_REV;
    /* Send the message */
    APP_MsgSend(req);
}

void APP_CscpsEnableReq(uint16_t conhdl, uint16_t csc_meas_ntf_cfg, uint16_t sc_ctnl_pt_ntf_cfg)
{
    uint8_t conidx = CONHDL2CONIDX(conhdl);
    struct cscps_enable_req *msg =
        KE_MSG_ALLOC(CSCPS_ENABLE_REQ, prf_get_task_from_id(TASK_ID_CSCPS), TASK_APP, cscps_enable_req);
    msg->conidx = conidx;
    msg->csc_meas_ntf_cfg = csc_meas_ntf_cfg;
    msg->sc_ctnl_pt_ntf_cfg = sc_ctnl_pt_ntf_cfg;
    APP_MsgSend(msg);
}

/*!
 * @brief This message corresponds to the response of setting bond data operation.
 *
 * @param[in] msgid     CSCPS_ENABLE_RSP
 * @param[in] param     Pointer to the struct cscps_enable_rsp
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_CSCPS
 *
 * @return If the message was consumed or not.
 * @description
 * This message corresponds to the response of setting bond data operation.
 */
static int APP_CscpsEnbleRspHandler(ke_msg_id_t const msgid,
                                    struct cscps_enable_rsp *param,
                                    ke_task_id_t const dest_id,
                                    ke_task_id_t const src_id)
{
    QPRINTF("blps enable status = %x,conidx=%x.\r\n", param->status, param->conidx);
    return (KE_MSG_CONSUMED);
}

void APP_CscpsNtfCscMeasReq(uint16_t conhdl,
                            uint8_t flags,
                            uint16_t cumul_crank_rev,
                            uint16_t last_crank_evt_time,
                            uint16_t last_wheel_evt_time,
                            uint16_t wheel_rev)
{
    struct cscps_ntf_csc_meas_req *msg =
        KE_MSG_ALLOC(CSCPS_NTF_CSC_MEAS_REQ, KE_BUILD_ID(prf_get_task_from_id(TASK_ID_CSCPS), CONHDL2CONIDX(conhdl)),
                     TASK_APP, cscps_ntf_csc_meas_req);
    msg->flags = flags;
    msg->cumul_crank_rev = cumul_crank_rev;
    msg->last_crank_evt_time = last_crank_evt_time;
    msg->last_wheel_evt_time = last_wheel_evt_time;
    msg->wheel_rev = wheel_rev;
    APP_MsgSend(msg);
}

/*!
 * @brief  This API message is sent once the notification has been send to the connected devices.
 *
 * @param[in] msgid     CSCPS_NTF_CSC_MEAS_RSP
 * @param[in] param     Pointer to the struct cscps_ntf_csc_meas_rsp
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_CSCPS
 *
 * @return If the message was consumed or not.
 * @description
 * This API message is sent once the notification has been send to the connected devices.
 */
static int APP_CscpsNtfCscMeasRspHandler(ke_msg_id_t const msgid,
                                         struct cscps_ntf_csc_meas_rsp *param,
                                         ke_task_id_t const dest_id,
                                         ke_task_id_t const src_id)
{
    QPRINTF("cscps_ntf_csc_meas_rsp status=%x,total wheel= %d.\r\n", param->status, param->tot_wheel_rev);
    return (KE_MSG_CONSUMED);
}

/*!
 * @brief  This message is used by BLPS to inform application that peer device has changed notification
 * configuration.
 *
 * @param[in] cscps_sc_ctnl_pt_cfm_value     Control Point Confirm Value.
 *
 * @response CSCPS_CMP_EVT
 * @description
 * This message is sent by the application as a response to the CSCPS_SC_CTNL_PT_REQ_IND message. It
 * contains the value requested by the profile.
 */
static void APP_CscpsScCtnlPtCfm(struct cscps_sc_ctnl_pt_cfm *cfm)
{
    uint8_t conidx = cfm->conidx;
    struct cscps_sc_ctnl_pt_cfm *msg = KE_MSG_ALLOC(
        CSCPS_SC_CTNL_PT_CFM, KE_BUILD_ID(prf_get_task_from_id(TASK_ID_CSCPS), conidx), TASK_APP, cscps_sc_ctnl_pt_cfm);
    msg->conidx = cfm->conidx;
    msg->op_code = cfm->op_code;
    msg->status = cfm->status;
    switch (cfm->op_code)
    {
        case CSCP_CTNL_PT_OP_SET_CUMUL_VAL:
            msg->value.cumul_wheel_rev = cfm->value.cumul_wheel_rev;
            break;
        case CSCP_CTNL_PT_OP_UPD_LOC:
            msg->value.sensor_loc = cfm->value.sensor_loc;
            break;
        case CSCP_CTNL_PT_OP_REQ_SUPP_LOC:
            msg->value.supp_sensor_loc = cfm->value.supp_sensor_loc;
            break;
    }
    APP_MsgSend(msg);
}

/*!
 * @brief   The message is sent to the application when the SC Control Point characteristic is written by the peer
 * device.
 *
 * @param[in] msgid     CSCPS_SC_CTNL_PT_REQ_IND
 * @param[in] param     Pointer to the struct cscps_sc_ctnl_pt_req_ind
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_CSCPS
 *
 * @return If the message was consumed or not.
 * @description
 * The message is sent to the application when the SC Control Point characteristic is written by the peer
 * device. The application shall answer using the CSCPS_SC_CTNL_PT_CFM message.
 */
static int APP_CscpScCntlPtReqIndHandler(ke_msg_id_t const msgid,
                                         struct cscps_sc_ctnl_pt_req_ind *param,
                                         ke_task_id_t const dest_id,
                                         ke_task_id_t const src_id)
{
    struct cscps_sc_ctnl_pt_cfm cscps_sc_ctnl_pt_cfm_value;
    cscps_sc_ctnl_pt_cfm_value.conidx = param->conidx;
    cscps_sc_ctnl_pt_cfm_value.status = CSCP_CTNL_PT_RESP_SUCCESS;
    switch (param->op_code)
    {
        case CSCP_CTNL_PT_OP_SET_CUMUL_VAL:
            cscps_sc_ctnl_pt_cfm_value.op_code = CSCPS_CTNL_PT_CUMUL_VAL_OP_CODE;
            cscps_sc_ctnl_pt_cfm_value.value.cumul_wheel_rev = param->value.cumul_value;
            break;
        case CSCP_CTNL_PT_OP_UPD_LOC:
            cscps_sc_ctnl_pt_cfm_value.op_code = CSCPS_CTNL_PT_UPD_LOC_OP_CODE;
            cscps_sc_ctnl_pt_cfm_value.value.sensor_loc = param->value.sensor_loc;
            break;
        case CSCP_CTNL_PT_OP_REQ_SUPP_LOC:
            cscps_sc_ctnl_pt_cfm_value.op_code = CSCPS_CTNL_PT_SUPP_LOC_OP_CODE;
            cscps_sc_ctnl_pt_cfm_value.value.supp_sensor_loc = CFG_CSC_SUPP_SENSOR_LOC;
            break;
        default:
            break;
    }
    APP_CscpsScCtnlPtCfm(&cscps_sc_ctnl_pt_cfm_value);
    return (KE_MSG_CONSUMED);
}

/*!
 * @brief   This message is sent to the application each time a peer device successfully writes the ClientCharacteristic
 * Configuration descriptor of either the CSC Measurement characteristic or the SC Control Point characteristic.
 *
 * @param[in] msgid     CSCPS_CFG_NTFIND_IND
 * @param[in] param     Pointer to the struct cscps_cfg_ntfind_ind
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_CSCPS
 *
 * @return If the message was consumed or not.
 * @description
 * This message is sent to the application each time a peer device successfully writes the ClientCharacteristic
 * Configuration descriptor of either the CSC Measurement characteristic or the SC Control Point characteristic.
 */
static int APP_CscpsCfgNtfindIndHandler(ke_msg_id_t const msgid,
                                        struct cscps_cfg_ntfind_ind *param,
                                        ke_task_id_t const dest_id,
                                        ke_task_id_t const src_id)
{
    switch (param->char_code)
    {
        case CSCP_CSCS_CSC_MEAS_CHAR:
            QPRINTF("CSC Measurement ntf set 0x%x.\r\n", param->ntf_cfg);
            break;
        case CSCP_CSCS_SC_CTNL_PT_CHAR:
            QPRINTF("CSC Control Point ind set 0x%x.\r\n", param->ntf_cfg);
            break;
    }
    return (KE_MSG_CONSUMED);
}

/*!
 * @brief    The API message is used by the CSCPS task to inform the sender of a command that the procedure is over
 * and contains the status of the procedure.
 *
 * @param[in] msgid     CSCPS_CMP_EVT
 * @param[in] param     Pointer to the struct cscps_cmp_evt
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_CSCPS
 *
 * @return If the message was consumed or not.
 * @description
 * The API message is used by the CSCPS task to inform the sender of a command that the procedure is over
 * and contains the status of the procedure.
 */
static int APP_CscpsCmpEvtHandler(ke_msg_id_t const msgid,
                                  struct cscps_cmp_evt *param,
                                  ke_task_id_t const dest_id,
                                  ke_task_id_t const src_id)
{
    QPRINTF("Cmp evt!\r\n");
    return (KE_MSG_CONSUMED);
}

/*******************************************************************************
 * Variables
 ******************************************************************************/
const struct ke_msg_handler s_appCscpsMsgHandlerList[] = {
    {CSCPS_ENABLE_RSP, (ke_msg_func_t)APP_CscpsEnbleRspHandler},
    {CSCPS_NTF_CSC_MEAS_RSP, (ke_msg_func_t)APP_CscpsNtfCscMeasRspHandler},
    {CSCPS_SC_CTNL_PT_REQ_IND, (ke_msg_func_t)APP_CscpScCntlPtReqIndHandler},
    {CSCPS_CFG_NTFIND_IND, (ke_msg_func_t)APP_CscpsCfgNtfindIndHandler},
    {CSCPS_CMP_EVT, (ke_msg_func_t)APP_CscpsCmpEvtHandler},

};

const struct ke_state_handler g_AppCscpsTableHandler = KE_STATE_HANDLER(s_appCscpsMsgHandlerList);

#endif

/*! @brief @} APP_CSCPS_API */

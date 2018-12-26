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
 * @addtogroup APP_BLPS_API
 * @{
 */

#include "app_ble.h"
#if (BLE_BP_SENSOR)

/*******************************************************************************
 * Code
 ******************************************************************************/
void APP_BlpsAddProfileTask(void)
{
    struct blps_db_cfg *db_cfg;
    struct gapm_profile_task_add_cmd *req = KE_MSG_ALLOC_DYN(GAPM_PROFILE_TASK_ADD_CMD, TASK_GAPM, TASK_APP,
                                                             gapm_profile_task_add_cmd, sizeof(struct blps_db_cfg));
    /* Fill message */
    req->operation = GAPM_PROFILE_TASK_ADD;
    req->prf_task_id = TASK_ID_BLPS;
    req->app_task = TASK_APP;
    req->start_hdl = 0;
    /* Set parameters */
    db_cfg = (struct blps_db_cfg *)req->param;
    db_cfg->features = CFG_BLPS_FEATURE;
    db_cfg->prfl_cfg = CFG_BLPS_PRFL_CFG;
    /* Send the message */
    APP_MsgSend(req);
}

void APP_BlpsEnableReq(uint16_t conhdl, uint16_t bp_meas_ind_en, uint16_t interm_cp_ntf_en)
{
    uint8_t conidx = CONHDL2CONIDX(conhdl);
    struct blps_enable_req *msg =
        KE_MSG_ALLOC(BLPS_ENABLE_REQ, prf_get_task_from_id(TASK_ID_BLPS), TASK_APP, blps_enable_req);
    msg->conidx = conidx;
    msg->bp_meas_ind_en = bp_meas_ind_en;
    msg->interm_cp_ntf_en = interm_cp_ntf_en;
    APP_MsgSend(msg);
}

void BLPS_MeasSendReq(uint16_t conhdl, uint16_t flag_interm_cp, struct bps_bp_meas meas_val)
{
    uint8_t conidx = CONHDL2CONIDX(conhdl);
    struct blps_meas_send_req *msg =
        KE_MSG_ALLOC(BLPS_MEAS_SEND_REQ, prf_get_task_from_id(TASK_ID_BLPS), TASK_APP, blps_meas_send_req);
    msg->conidx = conidx;
    msg->flag_interm_cp = flag_interm_cp;
    msg->meas_val = meas_val;
    APP_MsgSend(msg);
}

/*!
 * @brief  Informs the application about the result of the enabling operation.
 *
 * @param[in] msgid     BLPS_ENABLE_RSP
 * @param[in] param     Pointer to the struct blps_enable_rsp
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_BLPS
 *
 * @return If the message was consumed or not.
 * @description
 * This API message informs the application about the result of the enabling operation.
 */
static int APP_BlpsEnbleRspHandler(ke_msg_id_t const msgid,
                                   struct blps_enable_rsp *param,
                                   ke_task_id_t const dest_id,
                                   ke_task_id_t const src_id)
{
    QPRINTF("blps enable status = %x,conidx=%x.\r\n", param->status, param->conidx);
    return (KE_MSG_CONSUMED);
}

/*!
 * @brief  This message is used by BLPS to send to the application, a confirmation, or error status of a notification
 * request being sent to GATT for the Intermediate Cuff Pressure Char.
 *
 * @param[in] msgid     BLPS_MEAS_SEND_RSP
 * @param[in] param     Pointer to the struct blps_meas_send_rsp
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_BLPS
 *
 * @return If the message was consumed or not.
 * @description
 * This message is used by BLPS to send to the application, a confirmation, or error status of a notification
 * request being sent to GATT for the Intermediate Cuff Pressure Char.
 */
static int APP_BlpsMeasSendRspHandler(ke_msg_id_t const msgid,
                                      struct blps_meas_send_rsp *param,
                                      ke_task_id_t const dest_id,
                                      ke_task_id_t const src_id)
{
    QPRINTF("blps meas send status = %x,conidx=%x.\r\n", param->status, param->conidx);
    return (KE_MSG_CONSUMED);
}

/*!
 * @brief  This message is used by BLPS to inform application that peer device has changed notification
 * configuration.
 *
 * @param[in] msgid     BLPS_CFG_INDNTF_IND
 * @param[in] param     Pointer to the struct blps_cfg_indntf_ind
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_BLPS
 *
 * @return If the message was consumed or not.
 * @description
 * This message is used by BLPS to inform application that peer device has changed notification
 * configuration.
 */
static int APP_BlpsCfgIndntfIndHandler(ke_msg_id_t const msgid,
                                       struct blps_cfg_indntf_ind *param,
                                       ke_task_id_t const dest_id,
                                       ke_task_id_t const src_id)
{
    switch (param->char_code)
    {
        case BPS_BP_MEAS_CHAR:
            QPRINTF("Blood pressure measurement is configured to %x.\r\n", param->cfg_val);
            break;
        case BPS_INTM_CUFF_MEAS_CHAR:
            QPRINTF("Intermediate cuff pressure measurement is configured to %x.\r\n", param->cfg_val);
            break;
    }
    return (KE_MSG_CONSUMED);
}

/*******************************************************************************
 * Variables
 ******************************************************************************/
const struct ke_msg_handler s_appBlpsMsgHandlerList[] = {
    {BLPS_ENABLE_RSP, (ke_msg_func_t)APP_BlpsEnbleRspHandler},
    {BLPS_MEAS_SEND_RSP, (ke_msg_func_t)APP_BlpsMeasSendRspHandler},
    {BLPS_CFG_INDNTF_IND, (ke_msg_func_t)APP_BlpsCfgIndntfIndHandler},
};

const struct ke_state_handler g_AppBlpsTableHandler = KE_STATE_HANDLER(s_appBlpsMsgHandlerList);

#endif

/*! @brief @} APP_BLPS_API */

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
#if BLE_AN_CLIENT

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static int APP_AnpcEnableRspHandler(ke_msg_id_t const msgid,
                                    struct anpc_enable_rsp *param,
                                    ke_task_id_t const dest_id,
                                    ke_task_id_t const src_id);
static int APP_AnpcValueIndHandler(ke_msg_id_t const msgid,
                                   struct anpc_value_ind *param,
                                   ke_task_id_t const dest_id,
                                   ke_task_id_t const src_id);
static int APP_AnpcCmpEvtHandler(ke_msg_id_t const msgid,
                                 struct anpc_cmp_evt *param,
                                 ke_task_id_t const dest_id,
                                 ke_task_id_t const src_id);

/*******************************************************************************
 * Variables
 ******************************************************************************/

const struct ke_msg_handler s_appAnpcMsgHandlerList[] = {
    {ANPC_ENABLE_RSP, (ke_msg_func_t)APP_AnpcEnableRspHandler},
    {ANPC_VALUE_IND, (ke_msg_func_t)APP_AnpcValueIndHandler},
    {ANPC_CMP_EVT, (ke_msg_func_t)APP_AnpcCmpEvtHandler},
};

const struct ke_state_handler g_AppAnpcTableHandler = KE_STATE_HANDLER(s_appAnpcMsgHandlerList);

/*******************************************************************************
 * Code
 ******************************************************************************/
void APP_AnpcAddProfileTask(void)
{
    struct gapm_profile_task_add_cmd *req =
        KE_MSG_ALLOC_DYN(GAPM_PROFILE_TASK_ADD_CMD, TASK_GAPM, TASK_APP, gapm_profile_task_add_cmd, 0);
    /* Fill message */
    req->operation = GAPM_PROFILE_TASK_ADD;
    req->sec_lvl = 0;
    req->prf_task_id = TASK_ID_ANPC;
    req->app_task = TASK_APP;
    req->start_hdl = 0;

    /* Send the message */
    APP_MsgSend(req);
}

void APP_AnpcEnableReq(uint16_t conhdl)
{
    struct anpc_enable_req *msg =
        KE_MSG_ALLOC(ANPC_ENABLE_REQ, KE_BUILD_ID(prf_get_task_from_id(TASK_ID_ANPC), CONHDL2CONIDX(conhdl)), TASK_APP,
                     anpc_enable_req);
    msg->con_type = PRF_CON_DISCOVERY;
    /* Send the message */
    APP_MsgSend(msg);
}
/*!
 * @brief    This API message is used by the ANP Client role to inform the application of a correct enable command.
 *
 * @param[in] msgid     ANPC_ENABLE_RSP
 * @param[in] param     Pointer to the struct anpc_enable_rsp
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_ANPC
 *
 * @return If the message was consumed or not.
 * @description
 * This API message is used by the ANP Client role to inform the application of a correct enable command.
 */
static int APP_AnpcEnableRspHandler(ke_msg_id_t const msgid,
                                    struct anpc_enable_rsp *param,
                                    ke_task_id_t const dest_id,
                                    ke_task_id_t const src_id)
{
    uint16_t conhdl = CONIDX2CONHDL(KE_IDX_GET(src_id));
    QPRINTF("ANPC enable confirmation status: 0x%x.\r\n", param->status);
    if (param->status == CO_ERROR_NO_ERROR)
    {
        APP_AnpcReadCmd(conhdl, ANPC_RD_SUP_NEW_ALERT_CAT);
        APP_AnpcReadCmd(conhdl, ANPC_RD_SUP_UNREAD_ALERT_CAT);
    }
    return (KE_MSG_CONSUMED);
}

void APP_AnpcReadCmd(uint16_t conhdl, uint8_t read_code)
{
    struct anpc_read_cmd *msg = KE_MSG_ALLOC(
        ANPC_READ_CMD, KE_BUILD_ID(prf_get_task_from_id(TASK_ID_ANPC), CONHDL2CONIDX(conhdl)), TASK_APP, anpc_read_cmd);
    msg->read_code = read_code;

    /* Send the message */
    APP_MsgSend(msg);
}

void APP_AnpcWriteCmd(uint16_t conhdl, struct anpc_write_cmd write_cmd)
{
    struct anpc_write_cmd *msg =
        KE_MSG_ALLOC(ANPC_WRITE_CMD, KE_BUILD_ID(prf_get_task_from_id(TASK_ID_ANPC), CONHDL2CONIDX(conhdl)), TASK_APP,
                     anpc_write_cmd);
    msg->write_code = write_cmd.write_code;
    msg->value = write_cmd.value;
    /* Send the message */
    APP_MsgSend(msg);
}

/*!
 * @brief    This API is sent to the application once an attribute value has been received from the peer device upon a
 * notification or a read response message.
 *
 * @param[in] msgid     ANPC_VALUE_IND
 * @param[in] param     Pointer to the struct anpc_value_ind
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_ANPC
 *
 * @return If the message was consumed or not.
 * @description
 * This API is sent to the application once an attribute value has been received from the peer device upon a
 * notification or a read response message.
 */
static int APP_AnpcValueIndHandler(ke_msg_id_t const msgid,
                                   struct anpc_value_ind *param,
                                   ke_task_id_t const dest_id,
                                   ke_task_id_t const src_id)
{
    uint16_t sup_new_alert_cid;
    uint16_t sup_unread_alert_cid;
    uint8_t i;
    switch (param->att_code)
    {
        case ANPC_NTF_NEW_ALERT:
            QPRINTF("ANPC_NTF_NEW_ALERT,cat_id = 0x%x,nb_new_alert = 0x%x.\r\n", param->value.new_alert.cat_id,
                    param->value.new_alert.nb_new_alert);
            if (param->value.new_alert.info_str_len > 0)
            {
                QPRINTF("str_info=");
                for (i = 0; i < param->value.new_alert.info_str_len; i++)
                {
                    QPRINTF("%x ", param->value.new_alert.str_info[i]);
                    if (i == (param->value.new_alert.info_str_len - 1))
                        QPRINTF("\r\n");
                }
            }
            break;
        case ANPC_NTF_UNREAD_ALERT:
            QPRINTF("ANPC_NTF_UNREAD_ALERT,cat_id = 0x%x,nb_unread_alert = 0x%x.\r\n", param->value.unread_alert.cat_id,
                    param->value.unread_alert.nb_unread_alert);
            break;
        case ANPC_RD_SUP_NEW_ALERT_CAT:
            sup_new_alert_cid = (param->value.supp_cat.cat_id_mask_0) | (param->value.supp_cat.cat_id_mask_1 << 8);
            QPRINTF("ANPC_RD_SUP_NEW_ALERT_CAT = 0x%x.\r\n", sup_new_alert_cid);
            break;
        case ANPC_RD_SUP_UNREAD_ALERT_CAT:
            sup_unread_alert_cid = (param->value.supp_cat.cat_id_mask_0) | (param->value.supp_cat.cat_id_mask_1 << 8);
            QPRINTF("ANPC_RD_SUP_UNREAD_ALERT_CAT = 0x%x.\r\n", sup_unread_alert_cid);
            break;
        case ANPC_RD_WR_NEW_ALERT_CFG:
            QPRINTF("NEW_ALERT_CFG = 0x%x.\r\n", param->value.ntf_cfg);
            break;
        case ANPC_RD_WR_UNREAD_ALERT_STATUS_CFG:
            QPRINTF("UNREAD_ALERT_CFG = 0x%x.\r\n", param->value.ntf_cfg);
            break;
        default:
            break;
    }

    return (KE_MSG_CONSUMED);
}

/*!
 * @brief   The API message is used by the ANPC task to inform the sender of a command that the procedure is over
 * and contains the status of the procedure.
 *
 * @param[in] msgid     ANPC_CMP_EVT
 * @param[in] param     Pointer to the struct anpc_cmp_evt
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_ANPC
 *
 * @return If the message was consumed or not.
 * @description
 * The API message is used by the ANPC task to inform the sender of a command that the procedure is over
 * and contains the status of the procedure.
 */
static int APP_AnpcCmpEvtHandler(ke_msg_id_t const msgid,
                                 struct anpc_cmp_evt *param,
                                 ke_task_id_t const dest_id,
                                 ke_task_id_t const src_id)
{
    switch (param->operation)
    {
        case ANPC_ENABLE_OP_CODE:
            QPRINTF("ANPC Enable status = 0x%x.\r\n", param->status);
            break;
        case ANPC_READ_OP_CODE:
            QPRINTF("ANPC Read status = 0x%x.\r\n", param->status);
            break;
        case ANPC_WRITE_OP_CODE:
            QPRINTF("ANPC Write status = 0x%x.\r\n", param->status);
            break;
    }
    return (KE_MSG_CONSUMED);
}
#endif

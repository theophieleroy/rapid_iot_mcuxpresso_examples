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
#if BLE_PAS_CLIENT

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static int APP_PaspcEnableRspHandler(ke_msg_id_t const msgid,
                                     struct paspc_enable_rsp *param,
                                     ke_task_id_t const dest_id,
                                     ke_task_id_t const src_id);
static int APP_PaspcValueIndHandler(ke_msg_id_t const msgid,
                                    struct paspc_value_ind *param,
                                    ke_task_id_t const dest_id,
                                    ke_task_id_t const src_id);
static int APP_PaspsCmpEvtHandler(ke_msg_id_t const msgid,
                                  struct paspc_cmp_evt *param,
                                  ke_task_id_t const dest_id,
                                  ke_task_id_t const src_id);

/*******************************************************************************
 * Variables
 ******************************************************************************/

const struct ke_msg_handler s_appPaspcMsgHandlerList[] = {
    {PASPC_ENABLE_RSP, (ke_msg_func_t)APP_PaspcEnableRspHandler},
    {PASPC_VALUE_IND, (ke_msg_func_t)APP_PaspcValueIndHandler},
    {PASPC_CMP_EVT, (ke_msg_func_t)APP_PaspsCmpEvtHandler},
};

const struct ke_state_handler g_AppPaspcTableHandler = KE_STATE_HANDLER(s_appPaspcMsgHandlerList);

/*******************************************************************************
 * Code
 ******************************************************************************/
void APP_PaspcAddProfileTask(void)
{
    struct gapm_profile_task_add_cmd *req =
        KE_MSG_ALLOC_DYN(GAPM_PROFILE_TASK_ADD_CMD, TASK_GAPM, TASK_APP, gapm_profile_task_add_cmd, 0);
    /* Fill message */
    req->operation = GAPM_PROFILE_TASK_ADD;
    req->sec_lvl = 0;
    req->prf_task_id = TASK_ID_PASPC;
    req->app_task = TASK_APP;
    req->start_hdl = 0;

    /* Send the message */
    APP_MsgSend(req);
}

void APP_PaspcEnableReq(uint16_t conhdl)
{
    struct paspc_enable_req *msg =
        KE_MSG_ALLOC(PASPC_ENABLE_REQ, KE_BUILD_ID(prf_get_task_from_id(TASK_ID_PASPC), CONHDL2CONIDX(conhdl)),
                     TASK_APP, paspc_enable_req);

    msg->con_type = PRF_CON_DISCOVERY;

    /* Send the message */
    APP_MsgSend(msg);
}
/*!
 * @brief    This API message is used by the PASP Client role to inform the application of a correct enable command.
 *
 * @param[in] msgid     PASPC_ENABLE_RSP
 * @param[in] param     Pointer to the struct paspc_enable_rsp
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_PASPC
 *
 * @return If the message was consumed or not.
 * @description
 * This API message is used by the PASP Client role to inform the application of a correct enable command.
 */
static int APP_PaspcEnableRspHandler(ke_msg_id_t const msgid,
                                     struct paspc_enable_rsp *param,
                                     ke_task_id_t const dest_id,
                                     ke_task_id_t const src_id)
{
    uint16_t conhdl = CONIDX2CONHDL(KE_IDX_GET(src_id));
    QPRINTF("PASPC enable confirmation status: 0x%x.\r\n", param->status);
    if (param->status == CO_ERROR_NO_ERROR)
    {
        APP_PaspcReadCmd(conhdl, PASPC_RD_ALERT_STATUS);
    }
    return (KE_MSG_CONSUMED);
}

void APP_PaspcReadCmd(uint16_t conhdl, uint8_t read_code)
{
    struct paspc_read_cmd *msg =
        KE_MSG_ALLOC(PASPC_READ_CMD, KE_BUILD_ID(prf_get_task_from_id(TASK_ID_PASPC), CONHDL2CONIDX(conhdl)), TASK_APP,
                     paspc_read_cmd);
    msg->read_code = read_code;

    /* Send the message */
    APP_MsgSend(msg);
}

void APP_PaspcWriteCmd(uint16_t conhdl, struct paspc_write_cmd write_cmd)
{
    struct paspc_write_cmd *msg =
        KE_MSG_ALLOC(PASPC_WRITE_CMD, KE_BUILD_ID(prf_get_task_from_id(TASK_ID_PASPC), CONHDL2CONIDX(conhdl)), TASK_APP,
                     paspc_write_cmd);

    msg->write_code = write_cmd.write_code;
    msg->value = write_cmd.value;
    /* Send the message */
    APP_MsgSend(msg);
}

/*!
 * @brief    This API is sent to the application once an attribute value has been received from the peer device upon a
 * notification or a read response message.
 *
 * @param[in] msgid     PASPC_VALUE_IND
 * @param[in] param     Pointer to the struct paspc_value_ind
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_PASPC
 *
 * @return If the message was consumed or not.
 * @description
 * This API is sent to the application once an attribute value has been received from the peer device upon a
 * notification or a read response message.
 * The content of the value parameter depends of the attribute code value which defines the attribute that has been
 * updated.
 */
static int APP_PaspcValueIndHandler(ke_msg_id_t const msgid,
                                    struct paspc_value_ind *param,
                                    ke_task_id_t const dest_id,
                                    ke_task_id_t const src_id)
{
    switch (param->att_code)
    {
        case PASPC_RD_ALERT_STATUS:
            QPRINTF("Alert Status = 0x%x.\r\n", param->value.alert_status);
            break;
        case PASPC_RD_RINGER_SETTING:
            QPRINTF("Ringer Setting = 0x%x.\r\n", param->value.ringer_setting);
            break;
        case PASPC_RD_WR_ALERT_STATUS_CFG:
            QPRINTF("Alert Status NTF CFG = 0x%x.\r\n", param->value.alert_status_ntf_cfg);
            break;
        case PASPC_RD_WR_RINGER_SETTING_CFG:
            QPRINTF("Ringer Setting NTF CFG = 0x%x.\r\n", param->value.ringer_setting_ntf_cfg);
            break;
        default:
            break;
    }

    return (KE_MSG_CONSUMED);
}

/*!
 * @brief   The API message is used by the PASPC task to inform the sender of a command that the procedure is over
 * and contains the status of the procedure.
 *
 * @param[in] msgid     PASPC_CMP_EVT
 * @param[in] param     Pointer to the struct paspc_cmp_evt
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_PASPC
 *
 * @return If the message was consumed or not.
 * @description
 * The API message is used by the PASPC task to inform the sender of a command that the procedure is over
 * and contains the status of the procedure.
 */
static int APP_PaspsCmpEvtHandler(ke_msg_id_t const msgid,
                                  struct paspc_cmp_evt *param,
                                  ke_task_id_t const dest_id,
                                  ke_task_id_t const src_id)
{
    switch (param->operation)
    {
        case PASPC_ENABLE_OP_CODE:
            QPRINTF("PASPC Enable status = 0x%x.\r\n", param->status);
            break;
        case PASPC_READ_OP_CODE:
            QPRINTF("PASPC Read status = 0x%x.\r\n", param->status);
            break;
        case PASPC_WRITE_OP_CODE:
            QPRINTF("PASPC Write status = 0x%x.\r\n", param->status);
            break;
        default:
            break;
    }
    return (KE_MSG_CONSUMED);
}
#endif

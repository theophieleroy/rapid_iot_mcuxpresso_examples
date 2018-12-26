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

#if (BLE_APP_PRESENT)

#if (BLE_GATTC)
/*******************************************************************************
 * Variables
 ******************************************************************************/
const struct ke_msg_handler s_appGattMsgHandlerList[] = {
    /* GATTM */
    /* GATTC */
    {GATTC_CMP_EVT, (ke_msg_func_t)APP_GattcCmpEvtHandler},
    {GATTC_MTU_CHANGED_IND, (ke_msg_func_t)APP_GattcMtuChangedIndHandler},
    {GATTC_SVC_CHANGED_CFG_IND, (ke_msg_func_t)APP_GattcSvcChangedCfgIndHandler},
};

const struct ke_state_handler g_AppGattTableHandler = KE_STATE_HANDLER(s_appGattMsgHandlerList);

/*******************************************************************************
 * Code
 ******************************************************************************/
void APP_GattcExcMtuCmd(uint16_t conhdl)
{
    struct gattc_exc_mtu_cmd *msg =
        KE_MSG_ALLOC(GATTC_EXC_MTU_CMD, KE_BUILD_ID(TASK_GATTC, CONHDL2CONIDX(conhdl)), TASK_APP, gattc_exc_mtu_cmd);
    msg->operation = GATTC_MTU_EXCH;
    APP_MsgSend(msg);
}

int APP_GattcMtuChangedIndHandler(ke_msg_id_t const msgid,
                                  struct gattc_mtu_changed_ind const *param,
                                  ke_task_id_t const dest_id,
                                  ke_task_id_t const src_id)
{
    QPRINTF("Rx MTU len %d\r\n", param->mtu);

    return (KE_MSG_CONSUMED);
}

void APP_GattcSendSvcChangedCmd(uint16_t conhdl, uint16_t start_hdl, uint16_t end_hdl)
{
    struct gattc_send_svc_changed_cmd *msg =
        KE_MSG_ALLOC(GATTC_SEND_SVC_CHANGED_CMD, KE_BUILD_ID(TASK_GATTC, CONHDL2CONIDX(conhdl)), TASK_APP,
                     gattc_send_svc_changed_cmd);
    msg->operation = GATTC_SVC_CHANGED;
    msg->svc_shdl = start_hdl;
    msg->svc_ehdl = end_hdl;
}

int APP_GattcSvcChangedCfgIndHandler(ke_msg_id_t const msgid,
                                     struct gattc_svc_changed_cfg const *param,
                                     ke_task_id_t const dest_id,
                                     ke_task_id_t const src_id)
{
    if (param->ind_cfg == ATT_CCC_START_IND)
    {
        QPRINTF("Set service change indication flag\r\n");
    }
    else if (param->ind_cfg == ATT_CCC_STOP_NTFIND)
    {
        QPRINTF("Clear service change indication flag\r\n");
    }
    return (KE_MSG_CONSUMED);
}

/*!
 * @brief Handles GATTC complete event from the GATTC.
 *
 * @param[in] msgid     GATTC_CMP_EVT
 * @param[in] param     Pointer to struct gattc_cmp_evt
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_GATTC
 *
 * @return If the message was consumed or not.
 * @description
 *
 * This handler is used to inform the application that a GATT procedure finish.
 */
static int APP_GattcCmpEvtHandler(ke_msg_id_t const msgid,
                                  struct gattc_cmp_evt const *param,
                                  ke_task_id_t const dest_id,
                                  ke_task_id_t const src_id)
{
    switch (param->operation)
    {
        case GATTC_MTU_EXCH:
            QPRINTF("Perform MTU exchange\r\n");
            break;
        case GATTC_SVC_CHANGED:
            QPRINTF("Send a service changed indication\r\n");
            break;

        default:
            QPRINTF("Unhandle GATTC req_type(%#x)\r\n", param->operation);
            break;
    }

    return (KE_MSG_CONSUMED);
}
#endif
#endif /* BLE_APP_PRESENT */

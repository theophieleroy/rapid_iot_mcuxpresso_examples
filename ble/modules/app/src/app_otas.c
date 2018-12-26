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
#if BLE_OTA_SERVER
#include "gattc.h"
#include "otas.h"
#include "otap_server.h"

/*******************************************************************************
 * Variables
 ******************************************************************************/

struct app_otas_env_tag s_appOtasEnv[CFG_CON_DEV_MAX];

const struct ke_msg_handler s_appOtasMsgHandler_list[] = {
    {OTAS_ENABLE_RSP, (ke_msg_func_t)APP_OtasEnableRspHandler},
    {OTAS_RECV_DATA_IND, (ke_msg_func_t)APP_OtasRecvDataIndHandler},
    {OTAS_SEND_DATA_RSP, (ke_msg_func_t)APP_OtasSendDataRspHandler},
    {OTAS_RD_CHAR_RSP, (ke_msg_func_t)APP_OtasRdCharRspHandler},
};

const struct ke_state_handler g_AppOtasTableHandler = KE_STATE_HANDLER(s_appOtasMsgHandler_list);
static uint16_t g_OTAServerConhld = 0;

/*******************************************************************************
 * Code
 ******************************************************************************/
void otaServerRegisterConhdl(uint16_t conhdl)
{
    g_OTAServerConhld = conhdl;
}
static uint16_t otaGetCurrentValidConhdl(void)
{
    return g_OTAServerConhld;
}

uint16_t otapServerGetMtu(uint16_t conidx)
{
    return gattc_get_mtu(conidx);
}

void APP_OtasAddProfileTask(void)
{
    struct gapm_profile_task_add_cmd *req =
        KE_MSG_ALLOC(GAPM_PROFILE_TASK_ADD_CMD, TASK_GAPM, TASK_APP, gapm_profile_task_add_cmd);
    /* Fill message */
    req->operation = GAPM_PROFILE_TASK_ADD;
    req->sec_lvl = 0;
    req->prf_task_id = TASK_ID_OTAS;
    req->app_task = TASK_APP;
    req->start_hdl = 0;

    /* Send the message */
    APP_MsgSend(req);
}

void APP_OtasEnableReq(uint16_t conhdl)
{
    struct otas_enable_req *msg =
        KE_MSG_ALLOC(OTAS_ENABLE_REQ, KE_BUILD_ID(prf_get_task_from_id(TASK_ID_OTAS), CONHDL2CONIDX(conhdl)), TASK_APP,
                     otas_enable_req);
    /* Connection handle */
    msg->conhdl = conhdl;

    msg->con_type = PRF_CON_DISCOVERY;

    /* Send the message */
    APP_MsgSend(msg);
}

void app_otas_host_send_cmd(uint16_t conhdl, uint16_t len, uint8_t *val)
{
    struct otas_host_cmd_req *msg =
        KE_MSG_ALLOC_DYN(OTAS_HOST_CMD_REQ, KE_BUILD_ID(prf_get_task_from_id(TASK_ID_OTAS), CONHDL2CONIDX(conhdl)),
                         TASK_APP, otas_host_cmd_req, len);

    /* Connection handle */
    msg->conhdl = conhdl;
    msg->length = len;
    memcpy(msg->data, val, msg->length);

    /* Send the message */
    APP_MsgSend(msg);
}
void iapServerSend(uint16_t len, uint8_t *data)
{
    otapCmdIdt_t cmdId;

    cmdId = (otapCmdIdt_t)data[0];
    switch (cmdId)
    {
        case gOtapCmdIdNewImageNotification_c:
        case gOtapCmdIdNewImageInfoResponse_c:
        case gOtapCmdIdErrorNotification_c:
            otas_host_cmd_req_send(otaGetCurrentValidConhdl(), data, len, OTAS_OTAC_TARGET_RSP_VALUE);
            break;
        case gOtapCmdIdImageChunk_c:
            otas_host_cmd_req_send(otaGetCurrentValidConhdl(), data, len, OTAS_OTAC_HOST_CMD_CHAR);
            break;
    }
}

int APP_OtasEnableRspHandler(ke_msg_id_t const msgid,
                             struct otas_enable_rsp *param,
                             ke_task_id_t const dest_id,
                             ke_task_id_t const src_id)
{
    QPRINTF("OTAS enable confirmation status: 0x%x.\r\n", param->status);

    if (param->status == CO_ERROR_NO_ERROR)
    {
        uint8_t idx = KE_IDX_GET(src_id);
        s_appOtasEnv[idx].conhdl = param->conhdl;
        /*        iapc_env_init(param->conhdl);
                app_otas_rd_char_req(s_appOtasEnv[idx].conhdl, OTAS_OTAS_TARGET_RSP_NTF_CFG); */
    }

    return (KE_MSG_CONSUMED);
}

int APP_OtasRdCharRspHandler(ke_msg_id_t const msgid,
                             struct otas_rd_char_rsp *param,
                             ke_task_id_t const dest_id,
                             ke_task_id_t const src_id)
{
    if (param->status == CO_ERROR_NO_ERROR)
        QPRINTF("OTA version %s\r\n", param->data);
    else
        QPRINTF("OTA read version fail error %d\r\n", param->status);

    return (KE_MSG_CONSUMED);
}

int APP_OtasSendDataRspHandler(ke_msg_id_t const msgid,
                               struct otas_send_data_rsp *param,
                               ke_task_id_t const dest_id,
                               ke_task_id_t const src_id)
{
    QPRINTF("OTAS write rsp 0x%02x\r\n", param->status);

    return (KE_MSG_CONSUMED);
}

int APP_OtasCfgIndntfRspHandler(ke_msg_id_t const msgid,
                                struct otas_cfg_indntf_rsp *param,
                                ke_task_id_t const dest_id,
                                ke_task_id_t const src_id)
{
    QPRINTF("OTAS cfg ntf rsp 0x%02x\r\n", param->status);

    return (KE_MSG_CONSUMED);
}

int APP_OtasRecvDataIndHandler(ke_msg_id_t const msgid,
                               struct otas_recv_data_ind *param,
                               ke_task_id_t const dest_id,
                               ke_task_id_t const src_id)
{
    OtapServer_Proc(CONHDL2CONIDX(param->conhdl), param->length, param->data);
    return (KE_MSG_CONSUMED);
}

#endif

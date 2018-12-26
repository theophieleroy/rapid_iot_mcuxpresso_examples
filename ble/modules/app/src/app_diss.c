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
#if (BLE_DIS_SERVER)

/*******************************************************************************
 * Variables
 ******************************************************************************/
struct AppDissInfoTag
{
    char *info;
    uint8_t len;
};
struct AppDissInfoTag g_AppDissInfoTag[DIS_CHAR_MAX] = {
        [DIS_MANUFACTURER_NAME_CHAR] = {CFG_DIS_MANU_NAME_VAL, sizeof(CFG_DIS_MANU_NAME_VAL) - 1},
        [DIS_MODEL_NB_STR_CHAR] = {CFG_DIS_MODEL_NB_VAL, sizeof(CFG_DIS_MODEL_NB_VAL) - 1},
        [DIS_SERIAL_NB_STR_CHAR] = {CFG_DIS_SERIAL_NB_VAL, sizeof(CFG_DIS_SERIAL_NB_VAL) - 1},
        [DIS_HARD_REV_STR_CHAR] = {CFG_DIS_HW_REV_VAL, sizeof(CFG_DIS_HW_REV_VAL) - 1},
        [DIS_FIRM_REV_STR_CHAR] = {CFG_DIS_FW_REV_VAL, sizeof(CFG_DIS_FW_REV_VAL) - 1},
        [DIS_SW_REV_STR_CHAR] = {CFG_DIS_SW_REV_VAL, sizeof(CFG_DIS_SW_REV_VAL) - 1},
        [DIS_SYSTEM_ID_CHAR] = {CFG_DIS_SYS_ID_VAL, 8},
        [DIS_IEEE_CHAR] = {CFG_DIS_IEEE_CERTIF_VAL, 6},
        [DIS_PNP_ID_CHAR] = {CFG_DIS_PNP_ID_VAL, 7},
};

const struct ke_msg_handler s_appDissMsgHandlerList[] = {
    {DISS_VALUE_REQ_IND, (ke_msg_func_t)APP_DissValueReqIndHandler},
};

const struct ke_state_handler g_AppDissTableHandler = KE_STATE_HANDLER(s_appDissMsgHandlerList);

/*******************************************************************************
 * Code
 ******************************************************************************/
void APP_DissAddProfileTask(void)
{
    struct diss_db_cfg *db_cfg;
    struct gapm_profile_task_add_cmd *req = KE_MSG_ALLOC_DYN(GAPM_PROFILE_TASK_ADD_CMD, TASK_GAPM, TASK_APP,
                                                             gapm_profile_task_add_cmd, sizeof(struct diss_db_cfg));
    /* Fill message */
    req->operation = GAPM_PROFILE_TASK_ADD;
    req->sec_lvl = 0; /*mono instantiated, no forced 16 bytes EKS, no auth, enable the service */
    req->prf_task_id = TASK_ID_DISS;
    req->app_task = TASK_APP;
    req->start_hdl = 0;
    /* Set parameters */
    db_cfg = (struct diss_db_cfg *)req->param;
    db_cfg->features = CFG_DIS_FEAT_SUP;
    /* Send the message */
    APP_MsgSend(req);
}

int APP_DissValueReqIndHandler(ke_msg_id_t const msgid,
                               struct diss_value_req_ind *param,
                               ke_task_id_t const dest_id,
                               ke_task_id_t const src_id)
{
    uint8_t value_len = 0;
    uint8_t *value;

    if (param->value < DIS_CHAR_MAX)
    {
        value = (unsigned char *)g_AppDissInfoTag[param->value].info;
        value_len = g_AppDissInfoTag[param->value].len;
        struct diss_value_cfm *msg =
            KE_MSG_ALLOC_DYN(DISS_VALUE_CFM, prf_get_task_from_id(TASK_ID_DISS), TASK_APP, diss_value_cfm, value_len);

        msg->value = param->value;
        msg->length = value_len;
        memcpy(msg->data, value, value_len);
        APP_MsgSend(msg);
    }

    return (KE_MSG_CONSUMED);
}

#endif /* BLE_DIS_SERVER */

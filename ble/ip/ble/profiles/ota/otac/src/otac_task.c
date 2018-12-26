/**
 ****************************************************************************************
 *
 * @file otac_task.c
 *
 * @brief OTA Profile Client task implementation.
 *
 * Copyright(C) 2015 NXP Semiconductors N.V.
 *
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup OTACTASK
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"

#if (BLE_OTA_CLIENT)
#include <stdio.h>
#include "prf_utils.h"
#include "ke_timer.h"
#include "co_utils.h"
#include "gattc.h"
#include "otac.h"
#include "otap_client.h"
#include "otap_support.h"
/*
 *  OTA PROFILE ATTRIBUTES
 ****************************************************************************************
 */
/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */
extern void otac_send_cmd(uint16_t len, uint8_t *data);

/*
 * LOCAL FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Handles reception of the @ref OTAC_SEND_TARGET_RSP_REQ message.
 * @param[in] msgid Id of the message received.
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance.
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int otac_send_target_rsp_req_handler(ke_msg_id_t const msgid,
                                            struct otac_send_target_rsp_req *param,
                                            ke_task_id_t const dest_id,
                                            ke_task_id_t const src_id)
{
    uint8_t conidx = CONHDL2CONIDX(param->conhdl);
    uint8_t state = ke_state_get(dest_id);
    uint8_t msg_status = KE_MSG_CONSUMED;

    struct otac_env_tag *otac_env = PRF_ENV_GET(OTAC, otac);

    if (otac_env->ntf_cfg[conidx] == 1)
    {
        uint16_t length;
        uint16_t mpu = gattc_get_mtu(conidx) - 3;

        if (otac_env->target_rsp == NULL)
        {
            otac_env->target_rsp = param;
        }

        if (param->length <= mpu)
        {
            otac_env->target_rsp = NULL;
            length = param->length;
        }
        else
        {
            length = mpu;
            msg_status = KE_MSG_NO_FREE;
        }
        struct gattc_send_evt_cmd *targegt_rsp =
            KE_MSG_ALLOC_DYN(GATTC_SEND_EVT_CMD, KE_BUILD_ID(TASK_GATTC, conidx), dest_id, gattc_send_evt_cmd, length);

        /* Fill in the parameter structure */

        targegt_rsp->operation = GATTC_INDICATE;
        targegt_rsp->handle = OTAC_HANDLE(OTAC_IDX_TARGET_RSP_VAL);
        targegt_rsp->length = length;
        memcpy(&targegt_rsp->value[0], &param->data[param->offset], length);

        ke_msg_send(targegt_rsp);

        param->length -= length;
        param->offset += length;
    }

    return (msg_status);
}

/**
 ****************************************************************************************
 * @brief Handles @ref GATTC_CMP_EVT for GATTC_NOTIFY
 *
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance
 * @param[in] src_id    ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gattc_cmp_evt_handler(ke_msg_id_t const msgid,
                                 struct gattc_cmp_evt const *param,
                                 ke_task_id_t const dest_id,
                                 ke_task_id_t const src_id)
{
    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles reception of the @ref GATTC_WRITE_REQ_IND message.
 * The handler compares the new values with current ones and notifies them if they changed.
 * @param[in] msgid Id of the message received.
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance.
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gattc_write_req_ind_handler(ke_msg_id_t const msgid,
                                       struct gattc_write_req_ind *param,
                                       ke_task_id_t const dest_id,
                                       ke_task_id_t const src_id)
{
    struct otac_env_tag *otac_env = PRF_ENV_GET(OTAC, otac);
    uint8_t conidx = KE_IDX_GET(src_id);
    uint8_t status = GAP_ERR_NO_ERROR;
    uint8_t att_idx = param->handle - otac_env->start_hdl;
    otapClientInterface_t client_interface;

    if (param->offset != 0)
    {
        status = ATT_ERR_UNLIKELY_ERR;
    }
    else if ((att_idx == OTAC_IDX_HOST_CMD_VAL) || (att_idx == OTAC_IDX_TARGET_RSP_VAL))
    {
        struct otac_process_req *msg =
            KE_MSG_ALLOC_DYN(OTAC_PROCESS_REQ, dest_id, dest_id, otac_process_req, param->length);
        msg->length = param->length;
        memcpy(msg->value, param->value, param->length);

        ke_msg_send(msg);
    }
    else if (att_idx == OTAC_IDX_TARGET_RSP_NTF_CFG)
    {
        uint16_t value = co_read16p(param->value);

        if (value == PRF_CLI_STOP_NTFIND)
        {
            otac_env->ntf_cfg[conidx] = 0;
        }
        else if (value == PRF_CLI_START_NTF) /* PRF_CLI_START_NTF */
        {
            otac_env->ntf_cfg[conidx] = 1;

            client_interface.sendData = otac_send_cmd;
            client_interface.usrDataProc = otac_process_received_data;
            client_interface.ImgInfoExchange = OtapClient_ImageInformationExchange;
            client_interface.IsImageFileHeaderValid = OtapClient_IsImageFileHeaderValid;
            client_interface.getFlashDstAddrByImageType = OtapClient_ReceivedFileType;
            client_interface.getMaxMtu = otac_get_max_mtu;
            client_interface.uploadImageComplete = otac_download_Complete;

            otaClientRegisterConhdl(CONIDX2CONHDL(conidx));
            OtapClient_InterfaceRegister(&client_interface);
        }
        else if (value == PRF_CLI_START_IND) /* PRF_CLI_START_NTF */
        {
            otac_env->ntf_cfg[conidx] = 1;

            client_interface.sendData = otac_send_cmd;
            client_interface.usrDataProc = otac_process_received_data;
            client_interface.ImgInfoExchange = OtapClient_ImageInformationExchange;
            client_interface.IsImageFileHeaderValid = OtapClient_IsImageFileHeaderValid;
            client_interface.getFlashDstAddrByImageType = OtapClient_ReceivedFileType;
            client_interface.getMaxMtu = otac_get_max_mtu;
            client_interface.uploadImageComplete = otac_download_Complete;

            otaClientRegisterConhdl(CONIDX2CONHDL(conidx));
            OtapClient_InterfaceRegister(&client_interface);
        }
        else
        {
            status = PRF_APP_ERROR;
        }
    }
    else
    {
        status = PRF_ERR_INVALID_PARAM;
    }

    struct gattc_write_cfm *cfm;
    cfm = KE_MSG_ALLOC(GATTC_WRITE_CFM, src_id, dest_id, gattc_write_cfm);
    cfm->handle = param->handle;
    cfm->status = status;
    ke_msg_send(cfm);

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles reception of the read request from peer device
 *
 * @param[in] msgid Id of the message received.
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance.
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */

static int gattc_read_req_ind_handler(ke_msg_id_t const msgid,
                                      struct gattc_read_req_ind const *param,
                                      ke_task_id_t const dest_id,
                                      ke_task_id_t const src_id)
{
    ke_state_t state = ke_state_get(dest_id);
    uint8_t conidx = KE_IDX_GET(dest_id);
    struct otac_env_tag *otac_env = PRF_ENV_GET(OTAC, otac);
    uint16_t cfg_val = otac_env->ntf_cfg[conidx];

    struct gattc_read_cfm *msg = KE_MSG_ALLOC_DYN(GATTC_READ_CFM, src_id, dest_id, gattc_read_cfm, sizeof(cfg_val));
    msg->handle = param->handle;
    msg->length = sizeof(cfg_val);
    msg->status = ATT_ERR_NO_ERROR;
    co_write16p(msg->value, cfg_val);

    ke_msg_send(msg);

    return (KE_MSG_CONSUMED);
}

static int gapm_use_enc_block_ind_handler(ke_msg_id_t const msgid,
                                          struct gapm_use_enc_block_ind const *param,
                                          ke_task_id_t const dest_id,
                                          ke_task_id_t const src_id)
{
    return (KE_MSG_CONSUMED);
}
static int otac_process_req_handler(ke_msg_id_t const msgid,
                                    struct otac_process_req *param,
                                    ke_task_id_t const dest_id,
                                    ke_task_id_t const src_id)
{
    OtapClient_Proc(param->length, param->value);

    return (KE_MSG_CONSUMED);
}

int otac_timeout_ind_handler(ke_msg_id_t const msgid,
                             void const *param,
                             ke_task_id_t const dest_id,
                             ke_task_id_t const src_id)
{
    RESET_SetPeripheralReset(kREBOOT_RST_SHIFT_RSTn);
    return (KE_MSG_CONSUMED);
}

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

/// Default State handlers definition
const struct ke_msg_handler otac_default_state[] = {
    {OTAC_SEND_TARGET_RSP_REQ, (ke_msg_func_t)otac_send_target_rsp_req_handler},
    {GATTC_WRITE_REQ_IND, (ke_msg_func_t)gattc_write_req_ind_handler},
    {GATTC_CMP_EVT, (ke_msg_func_t)gattc_cmp_evt_handler},
    {GATTC_READ_REQ_IND, (ke_msg_func_t)gattc_read_req_ind_handler},
    {OTAC_PROCESS_REQ, (ke_msg_func_t)otac_process_req_handler},
    {GAPM_USE_ENC_BLOCK_IND, (ke_msg_func_t)gapm_use_enc_block_ind_handler},
    {OTAC_TIMEOUT_IND, (ke_msg_func_t)otac_timeout_ind_handler},
};

/// Specifies the message handlers that are common to all states.
const struct ke_state_handler otac_default_handler = KE_STATE_HANDLER(otac_default_state);

#endif /* #if (BLE_OTA_CLIENT) */

/// @} OTACTASK

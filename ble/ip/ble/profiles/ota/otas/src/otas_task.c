/**
 ****************************************************************************************
 *
 * @file otas_task.c
 *
 * @brief OTA Server Task implementation.
 *
 * Copyright(C) 2015 NXP Semiconductors N.V.
 * All rights reserved.
 *
 * $Rev: $
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup OTASTASK
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"

#if (BLE_OTA_SERVER)
#include "otas_task.h"
#include "otas.h"
#include "prf_utils.h"
#include "ke_mem.h"
#include "co_math.h"
#include "gattc.h"

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

/*
 * DEFINES
 ****************************************************************************************
 */

/// Used to retrieve OTA service characteristics information
const struct prf_char_128_def otas_otas_char[2] = {
        /// OTA Profile Target Response Value
        [OTAS_OTAC_TARGET_RSP_VALUE] = {ATT_UUID_128_LEN, ATT_CHAR_OTA_TARGET_RSP_UUID, ATT_MANDATORY,
                                        ATT_CHAR_PROP_IND | ATT_CHAR_PROP_WR},
        /// OTA Profile Host Command Value
        [OTAS_OTAC_HOST_CMD_CHAR] = {ATT_UUID_128_LEN, ATT_CHAR_OTA_HOST_CMD_UUID, ATT_MANDATORY,
                                     ATT_CHAR_PROP_WR_NO_RESP},
};

/// Used to retrieve OTA service characteristic description information
const struct prf_char_desc_def otas_otas_char_desc[2] = {
        /// OTA server version
        [OTAS_OTAC_TARGET_RSP_NTF_CFG] = {ATT_DESC_CLIENT_CHAR_CFG, ATT_MANDATORY, OTAS_OTAC_TARGET_RSP_VALUE},
        /// OTA Service target response config
        [OTAS_OTAC_TARGET_RSP_USER_DESP] = {ATT_DESC_CHAR_USER_DESCRIPTION, ATT_MANDATORY, OTAS_OTAC_TARGET_RSP_VALUE},
};

/*
 * MACROS
 ****************************************************************************************
 */

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

/*
 * LOCAL FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Handles reception of the @ref OTAS_ENABLE_REQ message.
 * The handler enables the OTA Profile Server Role.
 * @param[in] msgid Id of the message received .
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance.
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int otas_enable_req_handler(ke_msg_id_t const msgid,
                                   struct otas_enable_req const *param,
                                   ke_task_id_t const dest_id,
                                   ke_task_id_t const src_id)
{
    // Status
    uint8_t status = GAP_ERR_NO_ERROR;

    uint8_t state = ke_state_get(dest_id);
    uint8_t conidx = KE_IDX_GET(dest_id);
    // OTA Server Role Task Environment
    struct otas_env_tag *otas_env = PRF_ENV_GET(OTAS, otas);

    ASSERT_INFO(otas_env != NULL, dest_id, src_id);
    if ((state == OTAS_IDLE) && (otas_env->env[conidx] == NULL))
    {
        // allocate environment variable for task instance
        otas_env->env[conidx] = (struct otas_cnx_env *)ke_malloc(sizeof(struct otas_cnx_env), KE_MEM_ATT_DB);
        memset(otas_env->env[conidx], 0, sizeof(struct otas_cnx_env));

        otas_env->env[conidx]->host_cmd = NULL;
        otas_env->env[conidx]->data = ke_malloc(OTA_BLOCK_MAX_LEN, KE_MEM_ENV);
        otas_env->env[conidx]->state = OTAS_STATE_INIT;
        otas_env->env[conidx]->encrypt_enable = 1;

        // Config connection, start discovering
        if (param->con_type == PRF_CON_DISCOVERY)
        {
            // start discovering OTA on peer
            prf_disc_svc_by_uuid_send(&(otas_env->prf_env), conidx, ATT_UUID_128_LEN, ATT_SVC_OTA_UUID);

            // Go to DISCOVERING state
            ke_state_set(dest_id, OTAS_DISCOVERING);
        }
        // normal connection, get saved att details
        else
        {
            otas_env->env[conidx]->otas = param->otas;

            // send APP confirmation
            otas_enable_rsp_send(otas_env, conidx, GAP_ERR_NO_ERROR);
        }
    }

    else if (state != OTAS_FREE)
    {
        status = PRF_ERR_REQ_DISALLOWED;
    }

    // send an error if request fails
    if (status != GAP_ERR_NO_ERROR)
    {
        otas_enable_rsp_send(otas_env, conidx, status);
    }
    return (KE_MSG_CONSUMED);
}
/**
 ****************************************************************************************
 * @brief Handles reception of the @ref GATTC_SDP_SVC_IND_HANDLER message.
 * The handler stores the found service details for service discovery.
 * @param[in] msgid Id of the message received.
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance.
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gattc_sdp_svc_ind_handler(ke_msg_id_t const msgid,
                                     struct gattc_sdp_svc_ind const *ind,
                                     ke_task_id_t const dest_id,
                                     ke_task_id_t const src_id)
{
    uint8_t state = ke_state_get(dest_id);

    if (state == OTAS_DISCOVERING)
    {
        uint8_t conidx = KE_IDX_GET(dest_id);

        struct otas_env_tag *otas_env = PRF_ENV_GET(OTAS, otas);

        ASSERT_INFO(otas_env != NULL, dest_id, src_id);
        ASSERT_INFO(otas_env->env[conidx] != NULL, dest_id, src_id);

        if (otas_env->env[conidx]->nb_svc == 0)
        {
            // Retrieve OTA characteristics and descriptors
            prf_extract_svc_128_info(ind, OTAS_CHAR_MAX, &otas_otas_char[0], &otas_env->env[conidx]->otas.chars[0],
                                     OTAS_DESC_MAX, &otas_otas_char_desc[0], &otas_env->env[conidx]->otas.descs[0]);

            // Even if we get multiple responses we only store 1 range
            otas_env->env[conidx]->otas.svc.shdl = ind->start_hdl;
            otas_env->env[conidx]->otas.svc.ehdl = ind->end_hdl;
        }

        otas_env->env[conidx]->nb_svc++;
    }

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles reception of the @ref GATTC_CMP_EVT message.
 * This generic event is received for different requests, so need to keep track.
 * @param[in] msgid Id of the message received.
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance.
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gattc_cmp_evt_handler(ke_msg_id_t const msgid,
                                 struct gattc_cmp_evt const *param,
                                 ke_task_id_t const dest_id,
                                 ke_task_id_t const src_id)
{
    uint8_t state = ke_state_get(dest_id);
    uint8_t status;
    // Get the address of the environment
    struct otas_env_tag *otas_env = PRF_ENV_GET(OTAS, otas);
    uint8_t conidx = KE_IDX_GET(dest_id);

    if (state == OTAS_DISCOVERING)
    {
        status = param->status;

        if (param->status == ATT_ERR_NO_ERROR)
        {
            // check characteristic validity
            if (otas_env->env[conidx]->nb_svc == 1)
            {
                status =
                    prf_check_svc_char_128_validity(OTAS_CHAR_MAX, otas_env->env[conidx]->otas.chars, otas_otas_char);
            }
            // too much services, OTA has only 1 service
            else if (otas_env->env[conidx]->nb_svc > 1)
            {
                status = PRF_ERR_MULTIPLE_SVC;
            }
            // no services found
            else
            {
                status = PRF_ERR_STOP_DISC_CHAR_MISSING;
                if (otas_env->env[conidx] != NULL)
                {
                    ke_free(otas_env->env[conidx]);
                    otas_env->env[conidx] = NULL;
                }
                ke_state_set(dest_id, OTAS_FREE);
            }

            // check descriptor validity
            if (status == ATT_ERR_NO_ERROR)
            {
                status = prf_check_svc_char_desc_validity(OTAS_DESC_MAX, otas_env->env[conidx]->otas.descs,
                                                          otas_otas_char_desc, otas_env->env[conidx]->otas.chars);
            }
            otas_env->env[conidx]->mtu = gattc_get_mtu(conidx) - 3;
        }

        otas_enable_rsp_send(otas_env, conidx, status);

        // enable otas rseponse
        if (status == ATT_ERR_NO_ERROR)
        {
            prf_gatt_write_ntf_ind(&otas_env->prf_env, conidx,
                                   otas_env->env[conidx]->otas.descs[OTAS_OTAC_TARGET_RSP_NTF_CFG].desc_hdl,
                                   PRF_CLI_START_NTF);
        }
    }
    else if (state == OTAS_IDLE)
    {
        switch (param->operation)
        {
            case GATTC_WRITE:
            {
                break;
            }
            case GATTC_WRITE_NO_RESPONSE:
            {
                // use 0x09 (gOtapCmdIdImageChunkRsp_c) here for no need to include "otap_interface.h"
                uint8_t cmdId = 0x09;

                struct otas_process_req *msg =
                    KE_MSG_ALLOC_DYN(OTAS_PROCESS_REQ, dest_id, dest_id, otas_process_req, sizeof(cmdId));
                msg->length = 2;
                msg->value[0] = cmdId;

                ke_msg_send(msg);

                break;
            }
            case GATTC_READ:
            {
                if (param->status != ATT_ERR_NO_ERROR)
                {
                    struct otas_rd_char_rsp *rsp = KE_MSG_ALLOC_DYN(
                        OTAS_RD_CHAR_RSP, prf_dst_task_get(&(otas_env->prf_env), conidx), dest_id, otas_rd_char_rsp, 1);

                    // it will be a GATT status code
                    rsp->status = param->status;
                    rsp->conhdl = CONIDX2CONHDL(conidx);
                    // send the message
                    ke_msg_send(rsp);
                }
                break;
            }

            default:
                break;
        }
    }

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles reception of the @ref GATT_READ_CHAR_RESP message.
 * Generic event received after every simple read command sent to peer server.
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gatt_rd_char_rsp_handler(ke_msg_id_t const msgid,
                                    struct gattc_read_ind const *param,
                                    ke_task_id_t const dest_id,
                                    ke_task_id_t const src_id)
{
    uint8_t state = ke_state_get(dest_id);

    if (state == OTAS_IDLE)
    {
        uint8_t conidx = KE_IDX_GET(dest_id);

        struct otas_env_tag *otas_env = PRF_ENV_GET(OTAS, otas);

        ASSERT_INFO(otas_env != NULL, dest_id, src_id);
        ASSERT_INFO(otas_env->env[conidx] != NULL, dest_id, src_id);

        struct otas_rd_char_rsp *rsp = KE_MSG_ALLOC_DYN(
            OTAS_RD_CHAR_RSP, prf_dst_task_get(&(otas_env->prf_env), conidx), dest_id, otas_rd_char_rsp, param->length);
        // set error status
        rsp->status = ATT_ERR_NO_ERROR;
        rsp->length = param->length;
        rsp->conhdl = CONIDX2CONHDL(conidx);

        memcpy(rsp->data, param->value, param->length);

        ke_msg_send(rsp);
    }

    return (KE_MSG_CONSUMED);
}

/*
****************************************************************************************
* @brief Handles reception of the @ref OTAS_SEND_DATA_REQ message.
* Check if the handle exists in profile(already discovered) and send request, otherwise
* error to APP.
* @param[in] msgid Id of the message received (probably unused).
* @param[in] param Pointer to the parameters of the message.
* @param[in] dest_id ID of the receiving task instance (probably unused).
* @param[in] src_id ID of the sending task instance.
* @return If the message was consumed or not.
****************************************************************************************
*/
static int otas_send_data_req_handler(ke_msg_id_t const msgid,
                                      struct otas_host_cmd_req *param,
                                      ke_task_id_t const dest_id,
                                      ke_task_id_t const src_id)
{
    uint8_t state = ke_state_get(dest_id);
    uint8_t status = PRF_ERR_REQ_DISALLOWED;
    uint8_t msg_status = KE_MSG_CONSUMED;
    uint8_t attProp;
    uint8_t operation;

    if (state == OTAS_IDLE)
    {
        uint8_t conidx = KE_IDX_GET(dest_id);

        struct otas_env_tag *otas_env = PRF_ENV_GET(OTAS, otas);

        ASSERT_INFO(otas_env != NULL, dest_id, src_id);

        // environment variable not ready
        if (otas_env->env[conidx] == NULL)
        {
            status = PRF_APP_ERROR;
        }
        else if (param->length > OTA_BLOCK_MAX_LEN)
        {
            status = PRF_ERR_UNEXPECTED_LEN;
        }
        else
        {
            if (param->charCode == OTAS_OTAC_TARGET_RSP_VALUE)
            {
                attProp = ATT_CHAR_PROP_WR;
                operation = GATTC_WRITE;
            }
            else if (param->charCode == OTAS_OTAC_HOST_CMD_CHAR)
            {
                attProp = ATT_CHAR_PROP_WR_NO_RESP;
                operation = GATTC_WRITE_NO_RESPONSE;
            }

            if ((otas_env->env[conidx]->otas.chars[param->charCode].char_hdl != ATT_INVALID_SEARCH_HANDLE) &&
                ((otas_env->env[conidx]->otas.chars[param->charCode].prop & attProp) == attProp) &&
                ((operation == GATTC_WRITE_NO_RESPONSE) || (operation == GATTC_WRITE)))
            {
                uint16_t length;

                if (otas_env->env[conidx]->host_cmd == NULL)
                {
                    otas_env->env[conidx]->host_cmd = param;
                }

                if (param->length <= otas_env->env[conidx]->mtu)
                {
                    otas_env->env[conidx]->host_cmd = NULL;
                    length = param->length;
                }
                else
                {
                    length = otas_env->env[conidx]->mtu;
                    msg_status = KE_MSG_NO_FREE;
                }
                // Send GATT Write Request, simple write, no response
                prf_gatt_write(&otas_env->prf_env, conidx, otas_env->env[conidx]->otas.chars[param->charCode].val_hdl,
                               (uint8_t *)(param->data + param->offset), length, operation);

                param->length -= length;
                param->offset += length;

                status = GAP_ERR_NO_ERROR;
            }
        }
    }

    // request cannot be performed
    if (status != GAP_ERR_NO_ERROR)
    {
        struct otas_send_data_rsp *rsp = KE_MSG_ALLOC(OTAS_SEND_DATA_RSP, src_id, dest_id, otas_send_data_rsp);

        // it will be a status code
        rsp->status = status;
        rsp->conhdl = KE_IDX_GET(dest_id);
        // send the message
        ke_msg_send(rsp);
    }
    return (msg_status);
}

/**
 ****************************************************************************************
 * @brief Handles reception of the @ref GATTC_EVENT_REQ_IND message.
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gattc_event_req_ind_handler(ke_msg_id_t const msgid,
                                       struct gattc_event_ind *param,
                                       ke_task_id_t const dest_id,
                                       ke_task_id_t const src_id)
{
    uint8_t state = ke_state_get(dest_id);

    if (state != OTAS_FREE)
    {
        uint8_t conidx = KE_IDX_GET(dest_id);

        struct otas_env_tag *otas_env = PRF_ENV_GET(OTAS, otas);

        ASSERT_INFO(otas_env != NULL, dest_id, src_id);
        ASSERT_INFO(otas_env->env[conidx] != NULL, dest_id, src_id);

        switch (param->type)
        {
            case GATTC_INDICATE:
            {
                // confirm that indication has been correctly received
                struct gattc_event_cfm *cfm = KE_MSG_ALLOC(GATTC_EVENT_CFM, src_id, dest_id, gattc_event_cfm);
                cfm->handle = param->handle;
                ke_msg_send(cfm);

                if (otas_env->env[conidx]->otas.chars[OTAS_OTAC_TARGET_RSP_VALUE].val_hdl == param->handle)
                {
                    struct otas_process_req *msg =
                        KE_MSG_ALLOC_DYN(OTAS_PROCESS_REQ, dest_id, dest_id, otas_process_req, param->length);
                    msg->length = param->length;
                    memcpy(msg->value, param->value, param->length);

                    ke_msg_send(msg);
                }
            }
            break;
            default:
                break;
        }
    }

    return (KE_MSG_CONSUMED);
}

static int gapm_use_enc_block_ind_handler(ke_msg_id_t const msgid,
                                          struct gapm_use_enc_block_ind *param,
                                          ke_task_id_t const dest_id,
                                          ke_task_id_t const src_id)
{
    return (KE_MSG_CONSUMED);
}
static int otas_process_req_handler(ke_msg_id_t const msgid,
                                    struct otas_process_req *param,
                                    ke_task_id_t const dest_id,
                                    ke_task_id_t const src_id)
{
    uint8_t conidx = KE_IDX_GET(dest_id);
    struct otas_env_tag *otas_env = PRF_ENV_GET(OTAS, otas);
    struct otas_recv_data_ind *ind = KE_MSG_ALLOC_DYN(
        OTAS_RECV_DATA_IND, prf_dst_task_get(&(otas_env->prf_env), conidx), dest_id, otas_recv_data_ind, param->length);
    ind->length = param->length;
    ind->conhdl = CONIDX2CONHDL(conidx);
    memcpy(ind->data, param->value, ind->length);
    ke_msg_send(ind);

    return (KE_MSG_CONSUMED);
}

/// Default State handlers definition
const struct ke_msg_handler otas_default_state[] = {
    {OTAS_ENABLE_REQ, (ke_msg_func_t)otas_enable_req_handler},
    {GATTC_SDP_SVC_IND, (ke_msg_func_t)gattc_sdp_svc_ind_handler},
    {GATTC_CMP_EVT, (ke_msg_func_t)gattc_cmp_evt_handler},
    {GATTC_READ_IND, (ke_msg_func_t)gatt_rd_char_rsp_handler},
    {OTAS_HOST_CMD_REQ, (ke_msg_func_t)otas_send_data_req_handler},
    {GATTC_EVENT_REQ_IND, (ke_msg_func_t)gattc_event_req_ind_handler},
    {OTAS_PROCESS_REQ, (ke_msg_func_t)otas_process_req_handler},
    {GAPM_USE_ENC_BLOCK_IND, (ke_msg_func_t)gapm_use_enc_block_ind_handler},
};

// Specifies the message handlers that are common to all states.
const struct ke_state_handler otas_default_handler = KE_STATE_HANDLER(otas_default_state);

#endif /* (BLE_OTA_SERVER) */
/// @} OTASTASK

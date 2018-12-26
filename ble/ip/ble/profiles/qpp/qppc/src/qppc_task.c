/**
 ****************************************************************************************
 *
 * @file qppc_task.c
 *
 * @brief QBlue Private Profile Client Task implementation.
 *
 * Copyright(C) 2015 NXP Semiconductors N.V.
 * All rights reserved.
 *
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup QPPCTASK
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"

#if (BLE_QPP_CLIENT)
#include "qppc_task.h"
#include "qppc.h"
#include "prf_utils.h"
#include "ke_mem.h"
#include "co_math.h"

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

/*
 * DEFINES
 ****************************************************************************************
 */

/// Used to retrieve QBlue Private service characteristics information
const struct prf_char_128_def qppc_qpps_char[2] = {
        /// QBlue Private Profile Intput Value
        [QPPC_QPPS_RX_CHAR_VALUE] = {ATT_UUID_128_LEN, QPPS_RX_CHAR_UUID, ATT_MANDATORY, ATT_CHAR_PROP_WR_NO_RESP},
        /// QBlue Private Profile Output Value
        [QPPC_QPPS_TX_CHAR_VALUE] = {ATT_UUID_128_LEN, QPPS_TX_CHAR_UUID, ATT_MANDATORY, ATT_CHAR_PROP_NTF},
};

/// Used to retrieve QBlue Private service characteristic description information
const struct prf_char_desc_def qppc_qpps_char_desc[2] = {
        /// QPP server version
        [QPPC_QPPS_RX_CHAR_VALUE_USER_DESP] = {ATT_DESC_CHAR_USER_DESCRIPTION, ATT_MANDATORY, QPPC_QPPS_RX_CHAR_VALUE},
        /// QBlue Private Service Send data config
        [QPPC_QPPS_TX_VALUE_CLI_CFG] = {ATT_DESC_CLIENT_CHAR_CFG, ATT_MANDATORY, QPPC_QPPS_TX_CHAR_VALUE},
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
 * @brief Handles reception of the @ref QPPC_ENABLE_REQ message.
 * The handler enables the QBlue Private Profile Client Role.
 * @param[in] msgid Id of the message received .
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance.
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int qppc_enable_req_handler(ke_msg_id_t const msgid,
                                   struct qppc_enable_req const *param,
                                   ke_task_id_t const dest_id,
                                   ke_task_id_t const src_id)
{
    // Status
    uint8_t status = GAP_ERR_NO_ERROR;

    uint8_t state = ke_state_get(dest_id);
    uint8_t conidx = KE_IDX_GET(dest_id);
    // QBlue Private Profile Client Role Task Environment
    struct qppc_env_tag *qppc_env = PRF_ENV_GET(QPPC, qppc);

    ASSERT_INFO(qppc_env != NULL, dest_id, src_id);
    if ((state == QPPC_IDLE) && (qppc_env->env[conidx] == NULL))
    {
        // allocate environment variable for task instance
        qppc_env->env[conidx] = (struct qppc_cnx_env *)ke_malloc(sizeof(struct qppc_cnx_env), KE_MEM_ATT_DB);
        memset(qppc_env->env[conidx], 0, sizeof(struct qppc_cnx_env));

        // Config connection, start discovering
        if (param->con_type == PRF_CON_DISCOVERY)
        {
            // start discovering QPP on peer
            prf_disc_svc_by_uuid_send(&(qppc_env->prf_env), conidx, ATT_UUID_128_LEN, QPP_SVC_PRIVATE_UUID);

            // Go to DISCOVERING state
            ke_state_set(dest_id, QPPC_DISCOVERING);
        }
        // normal connection, get saved att details
        else
        {
            qppc_env->env[conidx]->qpps = param->qpps;

            // send APP confirmation
            qppc_enable_rsp_send(qppc_env, conidx, GAP_ERR_NO_ERROR);
        }
    }

    else if (state != QPPC_FREE)
    {
        status = PRF_ERR_REQ_DISALLOWED;
    }

    // send an error if request fails
    if (status != GAP_ERR_NO_ERROR)
    {
        qppc_enable_rsp_send(qppc_env, conidx, status);
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

    if (state == QPPC_DISCOVERING)
    {
        uint8_t conidx = KE_IDX_GET(dest_id);

        struct qppc_env_tag *qppc_env = PRF_ENV_GET(QPPC, qppc);

        ASSERT_INFO(qppc_env != NULL, dest_id, src_id);
        ASSERT_INFO(qppc_env->env[conidx] != NULL, dest_id, src_id);

        if (qppc_env->env[conidx]->nb_svc == 0)
        {
            // Retrieve QPPS characteristics and descriptors
            prf_extract_svc_128_info(ind, QPPC_CHAR_MAX, &qppc_qpps_char[0], &qppc_env->env[conidx]->qpps.chars[0],
                                     QPPC_DESC_MAX, &qppc_qpps_char_desc[0], &qppc_env->env[conidx]->qpps.descs[0]);

            // Even if we get multiple responses we only store 1 range
            qppc_env->env[conidx]->qpps.svc.shdl = ind->start_hdl;
            qppc_env->env[conidx]->qpps.svc.ehdl = ind->end_hdl;
        }

        qppc_env->env[conidx]->nb_svc++;
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
    struct qppc_env_tag *qppc_env = PRF_ENV_GET(QPPC, qppc);
    uint8_t conidx = KE_IDX_GET(dest_id);

    if (state == QPPC_DISCOVERING)
    {
        status = param->status;

        if (param->status == ATT_ERR_NO_ERROR)
        {
            // check characteristic validity
            if (qppc_env->env[conidx]->nb_svc == 1)
            {
                status =
                    prf_check_svc_char_128_validity(QPPC_CHAR_MAX, qppc_env->env[conidx]->qpps.chars, qppc_qpps_char);
            }
            // too much services, QPP has only 1 service
            else if (qppc_env->env[conidx]->nb_svc > 1)
            {
                status = PRF_ERR_MULTIPLE_SVC;
            }
            // no services found
            else
            {
                status = PRF_ERR_STOP_DISC_CHAR_MISSING;
                if (qppc_env->env[conidx] != NULL)
                {
                    ke_free(qppc_env->env[conidx]);
                    qppc_env->env[conidx] = NULL;
                }
                ke_state_set(dest_id, QPPC_FREE);
            }

            // check descriptor validity
            if (status == GAP_ERR_NO_ERROR)
            {
                status = prf_check_svc_char_desc_validity(QPPC_DESC_MAX, qppc_env->env[conidx]->qpps.descs,
                                                          qppc_qpps_char_desc, qppc_env->env[conidx]->qpps.chars);
            }
        }

        qppc_enable_rsp_send(qppc_env, conidx, status);
    }
    else if (state == QPPC_IDLE)
    {
        switch (param->operation)
        {
            case GATTC_WRITE:
            {
                struct qppc_cfg_indntf_rsp *rsp = KE_MSG_ALLOC(
                    QPPC_CFG_INDNTF_RSP, prf_dst_task_get(&(qppc_env->prf_env), conidx), dest_id, qppc_cfg_indntf_rsp);
                // it will be a GATT status code
                rsp->status = param->status;
                rsp->conhdl = CONIDX2CONHDL(conidx);
                // send the message
                ke_msg_send(rsp);

                break;
            }
            case GATTC_WRITE_NO_RESPONSE:
            {
                struct qppc_send_data_rsp *rsp = KE_MSG_ALLOC(
                    QPPC_SEND_DATA_RSP, prf_dst_task_get(&(qppc_env->prf_env), conidx), dest_id, qppc_send_data_rsp);

                // it will be a GATT status code
                rsp->status = param->status;
                rsp->conhdl = CONIDX2CONHDL(conidx);
                // send the message
                ke_msg_send(rsp);
                break;
            }
            case GATTC_READ:
            {
                if (param->status != ATT_ERR_NO_ERROR)
                {
                    struct qppc_rd_char_rsp *rsp = KE_MSG_ALLOC_DYN(
                        QPPC_RD_CHAR_RSP, prf_dst_task_get(&(qppc_env->prf_env), conidx), dest_id, qppc_rd_char_rsp, 1);

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
 * @brief Handles reception of the @ref QPPC_RD_CHAR_REQ message.
 * Check if the handle exists in profile(already discovered) and send request, otherwise
 * error to APP.
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int qppc_rd_char_req_handler(ke_msg_id_t const msgid,
                                    struct qppc_rd_char_req const *param,
                                    ke_task_id_t const dest_id,
                                    ke_task_id_t const src_id)
{
    uint8_t state = ke_state_get(dest_id);
    uint8_t status = PRF_ERR_REQ_DISALLOWED;
    uint8_t conidx = KE_IDX_GET(dest_id);

    if (state == QPPC_IDLE)
    {
        struct qppc_env_tag *qppc_env = PRF_ENV_GET(QPPC, qppc);
        uint16_t search_hdl = ATT_INVALID_HANDLE;

        ASSERT_INFO(qppc_env != NULL, dest_id, src_id);

        // environment variable not ready
        if (qppc_env->env[conidx] == NULL)
        {
            status = PRF_APP_ERROR;
        }
        else
        {
            // check if feature val characteristic exists
            search_hdl = qppc_env->env[conidx]->qpps.descs[param->char_code].desc_hdl;
            if (search_hdl == ATT_INVALID_HANDLE)
                status = PRF_ERR_INEXISTENT_HDL;
            else
                status = GAP_ERR_NO_ERROR;

            // request can be performed
            if (status == GAP_ERR_NO_ERROR)
            {
                // read qpp feature
                prf_read_char_send(&(qppc_env->prf_env), conidx, qppc_env->env[conidx]->qpps.svc.shdl,
                                   qppc_env->env[conidx]->qpps.svc.ehdl, search_hdl);
            }
        }
    }

    // request cannot be performed
    if (status != GAP_ERR_NO_ERROR)
    {
        struct qppc_rd_char_rsp *rsp = KE_MSG_ALLOC_DYN(QPPC_RD_CHAR_RSP, src_id, dest_id, qppc_rd_char_rsp, 0);
        // set error status
        rsp->status = status;
        rsp->conhdl = CONIDX2CONHDL(conidx);

        ke_msg_send(rsp);
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

    if (state == QPPC_IDLE)
    {
        uint8_t conidx = KE_IDX_GET(dest_id);

        struct qppc_env_tag *qppc_env = PRF_ENV_GET(QPPC, qppc);

        ASSERT_INFO(qppc_env != NULL, dest_id, src_id);
        ASSERT_INFO(qppc_env->env[conidx] != NULL, dest_id, src_id);

        struct qppc_rd_char_rsp *rsp = KE_MSG_ALLOC_DYN(
            QPPC_RD_CHAR_RSP, prf_dst_task_get(&(qppc_env->prf_env), conidx), dest_id, qppc_rd_char_rsp, param->length);
        // set error status
        rsp->status = ATT_ERR_NO_ERROR;
        rsp->length = param->length;
        rsp->conhdl = CONIDX2CONHDL(conidx);

        memcpy(rsp->data, param->value, param->length);

        ke_msg_send(rsp);
    }

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles reception of the @ref QPPC_CFG_INDNTF_REQ message.
 * It allows configuration of the peer ind/ntf/stop characteristic for a specified characteristic.
 * Will return an error code if that cfg char does not exist.
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int qppc_cfg_indntf_req_handler(ke_msg_id_t const msgid,
                                       struct qppc_cfg_indntf_req const *param,
                                       ke_task_id_t const dest_id,
                                       ke_task_id_t const src_id)
{
    uint8_t state = ke_state_get(dest_id);
    uint8_t status = PRF_ERR_REQ_DISALLOWED;
    uint8_t conidx = KE_IDX_GET(dest_id);

    if (state == QPPC_IDLE)
    {
        uint8_t conidx = KE_IDX_GET(dest_id);
        // Get the address of the environment
        struct qppc_env_tag *qppc_env = PRF_ENV_GET(QPPC, qppc);
        ASSERT_INFO(qppc_env != NULL, dest_id, src_id);
        // environment variable not ready
        if (qppc_env->env[conidx] == NULL)
        {
            status = PRF_APP_ERROR;
        }
        else if (!((param->cfg_val == PRF_CLI_STOP_NTFIND) || (param->cfg_val == PRF_CLI_START_NTF)))
        {
            status = PRF_ERR_INVALID_PARAM;
        }
        else
        {
            uint16_t handle = qppc_env->env[conidx]->qpps.descs[param->char_code].desc_hdl;
            status = PRF_ERR_INEXISTENT_HDL;

            if (handle == ATT_INVALID_HDL)
            {
                status = PRF_ERR_INEXISTENT_HDL;
            }
            else
            {
                status = GAP_ERR_NO_ERROR;
                // Send GATT Write Request
                prf_gatt_write_ntf_ind(&qppc_env->prf_env, conidx, handle, param->cfg_val);
            }
        }
    }

    if (status != GAP_ERR_NO_ERROR)
    {
        struct qppc_cfg_indntf_rsp *rsp = KE_MSG_ALLOC(QPPC_CFG_INDNTF_RSP, src_id, dest_id, qppc_cfg_indntf_rsp);
        // it will be a status code
        rsp->status = status;
        rsp->conhdl = CONIDX2CONHDL(conidx);
        // send the message
        ke_msg_send(rsp);
    }

    return (KE_MSG_CONSUMED);
}

/*
****************************************************************************************
* @brief Handles reception of the @ref QPPC_SEND_DATA_REQ message.
* Check if the handle exists in profile(already discovered) and send request, otherwise
* error to APP.
* @param[in] msgid Id of the message received (probably unused).
* @param[in] param Pointer to the parameters of the message.
* @param[in] dest_id ID of the receiving task instance (probably unused).
* @param[in] src_id ID of the sending task instance.
* @return If the message was consumed or not.
****************************************************************************************
*/
static int qppc_send_data_req_handler(ke_msg_id_t const msgid,
                                      struct qppc_send_data_req const *param,
                                      ke_task_id_t const dest_id,
                                      ke_task_id_t const src_id)
{
    uint8_t state = ke_state_get(dest_id);
    uint8_t status = PRF_ERR_REQ_DISALLOWED;

    if (state == QPPC_IDLE)
    {
        uint8_t conidx = KE_IDX_GET(dest_id);

        struct qppc_env_tag *qppc_env = PRF_ENV_GET(QPPC, qppc);

        ASSERT_INFO(qppc_env != NULL, dest_id, src_id);

        // environment variable not ready
        if (qppc_env->env[conidx] == NULL)
        {
            status = PRF_APP_ERROR;
        }
        else if (param->length > QPP_DATA_MAX_LEN)
        {
            status = PRF_ERR_UNEXPECTED_LEN;
        }
        else
        {
            if (qppc_env->env[conidx]->qpps.chars[QPPC_QPPS_RX_CHAR_VALUE].char_hdl != ATT_INVALID_SEARCH_HANDLE)
            {
                if ((qppc_env->env[conidx]->qpps.chars[QPPC_QPPS_RX_CHAR_VALUE].prop & ATT_CHAR_PROP_WR_NO_RESP) ==
                    ATT_CHAR_PROP_WR_NO_RESP)
                {
                    // Send GATT Write Request, simple write, no response
                    prf_gatt_write(
                        &qppc_env->prf_env, conidx, qppc_env->env[conidx]->qpps.chars[QPPC_QPPS_RX_CHAR_VALUE].val_hdl,
                        (uint8_t *)param->data, co_min(param->length, QPP_DATA_MAX_LEN), GATTC_WRITE_NO_RESPONSE);

                    status = GAP_ERR_NO_ERROR;
                }
            }
        }
    }

    // request cannot be performed
    if (status != GAP_ERR_NO_ERROR)
    {
        struct qppc_send_data_rsp *rsp = KE_MSG_ALLOC(QPPC_SEND_DATA_RSP, src_id, dest_id, qppc_send_data_rsp);

        // it will be a status code
        rsp->status = status;
        rsp->conhdl = KE_IDX_GET(dest_id);
        // send the message
        ke_msg_send(rsp);
    }
    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles reception of the @ref GATTC_EVENT_IND message.
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gattc_event_ind_handler(ke_msg_id_t const msgid,
                                   struct gattc_event_ind const *param,
                                   ke_task_id_t const dest_id,
                                   ke_task_id_t const src_id)
{
    uint8_t state = ke_state_get(dest_id);

    if (state != QPPC_FREE)
    {
        uint8_t conidx = KE_IDX_GET(dest_id);

        struct qppc_env_tag *qppc_env = PRF_ENV_GET(QPPC, qppc);

        ASSERT_INFO(qppc_env != NULL, dest_id, src_id);
        ASSERT_INFO(qppc_env->env[conidx] != NULL, dest_id, src_id);

        switch (param->type)
        {
            case GATTC_NOTIFY:
            {
                if (qppc_env->env[conidx]->qpps.chars[QPPC_QPPS_TX_CHAR_VALUE].val_hdl == param->handle)
                {
                    struct qppc_recv_data_ind *ind =
                        KE_MSG_ALLOC_DYN(QPPC_RECV_DATA_IND, prf_dst_task_get(&(qppc_env->prf_env), conidx), dest_id,
                                         qppc_recv_data_ind, param->length);
                    ind->length = param->length;
                    ind->conhdl = CONIDX2CONHDL(conidx);
                    memcpy(ind->data, param->value, ind->length);
                    ke_msg_send(ind);
                }
            }
            break;
            default:
                break;
        }
    }

    return (KE_MSG_CONSUMED);
}

/// Default State handlers definition
const struct ke_msg_handler qppc_default_state[] = {
    {QPPC_ENABLE_REQ, (ke_msg_func_t)qppc_enable_req_handler},
    {GATTC_SDP_SVC_IND, (ke_msg_func_t)gattc_sdp_svc_ind_handler},
    {GATTC_CMP_EVT, (ke_msg_func_t)gattc_cmp_evt_handler},
    {QPPC_RD_CHAR_REQ, (ke_msg_func_t)qppc_rd_char_req_handler},
    {GATTC_READ_IND, (ke_msg_func_t)gatt_rd_char_rsp_handler},
    {QPPC_CFG_INDNTF_REQ, (ke_msg_func_t)qppc_cfg_indntf_req_handler},
    {QPPC_SEND_DATA_REQ, (ke_msg_func_t)qppc_send_data_req_handler},
    {GATTC_EVENT_IND, (ke_msg_func_t)gattc_event_ind_handler},
};

// Specifies the message handlers that are common to all states.
const struct ke_state_handler qppc_default_handler = KE_STATE_HANDLER(qppc_default_state);

#endif /* (BLE_QPP_CLIENT) */
/// @} QPPCTASK

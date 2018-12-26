/**
 ****************************************************************************************
 *
 * @file qpps_task.c
 *
 * @brief QBlue Private Profile Server task implementation.
 *
 * Copyright(C) 2015 NXP Semiconductors N.V.
 *
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup QPPSTASK
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"

#if (BLE_QPP_SERVER)
#include <stdio.h>
#include "prf_utils.h"
#include "ke_timer.h"
#include "co_utils.h"
#include "qpps.h"
/*
 *  QPP PROFILE ATTRIBUTES
 ****************************************************************************************
 */
/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */
#if defined(CFG_QPP_SHOW_THROUGHPUT)
struct qpp_throughput_stats qpps_stats[BLE_CONNECTION_MAX];
#endif
/*
 * LOCAL FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Handles reception of the @ref QPPS_SEND_DATA_REQ message.
 * @param[in] msgid Id of the message received.
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance.
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int qpps_send_data_req_handler(ke_msg_id_t const msgid,
                                      struct qpps_send_data_req const *param,
                                      ke_task_id_t const dest_id,
                                      ke_task_id_t const src_id)
{
    uint8_t conidx = CONHDL2CONIDX(param->conhdl);
    uint8_t state  = ke_state_get(dest_id);
    uint8_t status = PRF_ERR_REQ_DISALLOWED;

    if(param->length > QPP_DATA_MAX_LEN)
    {
        status = PRF_ERR_UNEXPECTED_LEN;
    }
    else
    {
        struct qpps_env_tag *qpps_env = PRF_ENV_GET(QPPS, qpps);

        if(qpps_env->ntf_cfg[conidx] == QPPS_VALUE_NTF_ON)
        {
            // Allocate the GATT indication message
            struct gattc_send_evt_cmd *qpps_send_val = KE_MSG_ALLOC_DYN(GATTC_SEND_EVT_CMD,
                                                                        KE_BUILD_ID(TASK_GATTC, conidx),
                                                                        dest_id,
                                                                        gattc_send_evt_cmd,
                                                                        param->length);

            // Fill in the parameter structure
            qpps_send_val->operation = GATTC_NOTIFY;
            qpps_send_val->handle = QPPS_HANDLE(QPPS_IDX_TX_DATA_VAL);
            qpps_send_val->length = param->length;
            memcpy(&qpps_send_val->value[0], &param->data[0], param->length);

            ke_msg_send(qpps_send_val);

            #if defined(CFG_QPP_SHOW_THROUGHPUT)
            qpps_stats[conidx].curr_tx_bytes += param->length;
            #endif

            status = (GAP_ERR_NO_ERROR);
        }
    }

    // send command complete if an error occurs
    if(status != GAP_ERR_NO_ERROR)
    {
        // send completed information to APP task that contains error status
        struct qpps_send_data_rsp *send_rsp = KE_MSG_ALLOC(QPPS_SEND_DATA_RSP,
                                                           src_id,
                                                           dest_id,
                                                           qpps_send_data_rsp);
        send_rsp->status = status;
        send_rsp->conhdl = param->conhdl;

        ke_msg_send(send_rsp);
    }

    return (KE_MSG_CONSUMED);
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
static int gattc_cmp_evt_handler(ke_msg_id_t const msgid, struct gattc_cmp_evt const *param,
                                 ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    uint8_t conidx = KE_IDX_GET(src_id);
    uint8_t state  = ke_state_get(dest_id);

    // Get the address of the environment
    struct qpps_env_tag *qpps_env = PRF_ENV_GET(QPPS, qpps);

    if (param->operation == GATTC_NOTIFY)
    {
        struct qpps_send_data_rsp *send_rsp = KE_MSG_ALLOC(QPPS_SEND_DATA_RSP,
                                                           prf_dst_task_get(&(qpps_env->prf_env), conidx),
                                                           dest_id,
                                                           qpps_send_data_rsp);
        send_rsp->status = param->status;
        send_rsp->conhdl = CONIDX2CONHDL(conidx);
        ke_msg_send(send_rsp);
    }

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
    uint8_t state  = ke_state_get(dest_id);
    uint8_t conidx = KE_IDX_GET(src_id);

    uint8_t status = GAP_ERR_NO_ERROR;
    struct gattc_write_cfm * cfm;

    // Get the address of the environment
    struct qpps_env_tag *qpps_env = PRF_ENV_GET(QPPS, qpps);
    uint8_t att_idx = param->handle-qpps_env->start_hdl;

    if(param->offset != 0)
    {
        status = ATT_ERR_UNLIKELY_ERR;
    }
    // check if it's a client configuration char
    else if(att_idx == QPPS_IDX_TX_DATA_NTF_CFG)
    {
        uint16_t value = co_read16p(param->value);

        if (value == PRF_CLI_STOP_NTFIND)
        {
            qpps_env->ntf_cfg[conidx]= QPPS_VALUE_NTF_OFF;
        }
        else if(value == PRF_CLI_START_NTF)//PRF_CLI_START_NTF
        {
            qpps_env->ntf_cfg[conidx] = QPPS_VALUE_NTF_ON;
        }
        else
        {
            status = PRF_APP_ERROR;
        }

        if (status == GAP_ERR_NO_ERROR)
        {
            //Inform APP of configuration change
            struct qpps_cfg_indntf_ind * ind = KE_MSG_ALLOC(QPPS_CFG_INDNTF_IND,
                                                            prf_dst_task_get(&(qpps_env->prf_env), conidx),
                                                            dest_id,
                                                            qpps_cfg_indntf_ind);
            ind->cfg_val = qpps_env->ntf_cfg[conidx];
            ind->conhdl = CONIDX2CONHDL(conidx);
            ke_msg_send(ind);
        }
    }
    else if (att_idx == QPPS_IDX_RX_DATA_VAL)
    {
        if (param->length <= QPP_DATA_MAX_LEN)
        {
            //inform APP of configuration change
            struct qpps_recv_data_ind * ind = KE_MSG_ALLOC_DYN(QPPS_RECV_DATA_IND,
                                                               prf_dst_task_get(&(qpps_env->prf_env), conidx),
                                                               dest_id,
                                                               qpps_recv_data_ind,
                                                               param->length);
            //Send received data to app
            ind->length = param->length;
            ind->conhdl = CONIDX2CONHDL(conidx);
            memcpy(ind->data, param->value, param->length);
            ke_msg_send(ind);

            #if defined(CFG_QPP_SHOW_THROUGHPUT)
            qpps_stats[conidx].curr_rx_bytes += param->length;
            #endif
        }
        else
        {
            status = ATT_ERR_INVALID_ATTRIBUTE_VAL_LEN;
        }
    }
    else
    {
        status = PRF_ERR_INVALID_PARAM;
    }

    //Send write response
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
    struct qpps_env_tag *qpps_env = PRF_ENV_GET(QPPS, qpps);
    uint16_t cfg_val = qpps_env->ntf_cfg[conidx];

    struct gattc_read_cfm *msg = KE_MSG_ALLOC_DYN(GATTC_READ_CFM, src_id, dest_id,
                                                  gattc_read_cfm, sizeof(cfg_val));
    msg->handle = param->handle;
    msg->length = sizeof(cfg_val);
    msg->status = ATT_ERR_NO_ERROR;
    co_write16p(msg->value, cfg_val);

    ke_msg_send(msg);

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles the reception of QPPS_THROUGHPUT_STATISTICS_TIMER message
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance.
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
#if defined(CFG_QPP_SHOW_THROUGHPUT)
int qpps_throughput_statistics_timer_handler(ke_msg_id_t const msgid,
                                             void const *param,
                                             ke_task_id_t const dest_id,
                                             ke_task_id_t const src_id)
{
    struct qpps_env_tag *qpps_env = PRF_ENV_GET(QPPS, qpps);
    uint32_t tx_bps, rx_bps;
    uint32_t stats_duration = QPP_THROUGHPUT_STATS_TIMEOUT / 100; // seconds

    for(uint8_t conhdl=0; conhdl<BLE_CONNECTION_MAX; conhdl++)
    {
        if (qpps_env->ntf_cfg[CONHDL2CONIDX(conhdl)] == QPPS_VALUE_NTF_ON)
        {
            tx_bps = (qpps_stats[conhdl].curr_tx_bytes - qpps_stats[conhdl].prev_tx_bytes) * 8 / stats_duration;
            rx_bps = (qpps_stats[conhdl].curr_rx_bytes - qpps_stats[conhdl].prev_rx_bytes) * 8 / stats_duration;

            qpps_stats[conhdl].prev_tx_bytes = qpps_stats[conhdl].curr_tx_bytes;
            qpps_stats[conhdl].prev_rx_bytes = qpps_stats[conhdl].curr_rx_bytes;

            printf("QPPS conhdl %d - tx rates:%dbps; rx rates:%dbps\r\n", conhdl, tx_bps, rx_bps);
        }
    }

    ke_timer_set(QPPS_THROUGHPUT_STATISTICS_TIMER, qpps_env->prf_env.prf_task, QPP_THROUGHPUT_STATS_TIMEOUT);

    return (KE_MSG_CONSUMED);
}
#endif

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

/// Default State handlers definition
const struct ke_msg_handler qpps_default_state[] =
{
    {QPPS_SEND_DATA_REQ,                    (ke_msg_func_t)qpps_send_data_req_handler},
    {GATTC_WRITE_REQ_IND,                   (ke_msg_func_t)gattc_write_req_ind_handler},
    {GATTC_CMP_EVT,                         (ke_msg_func_t)gattc_cmp_evt_handler},
    {GATTC_READ_REQ_IND,                    (ke_msg_func_t)gattc_read_req_ind_handler},
#if defined(CFG_QPP_SHOW_THROUGHPUT)
    {QPPS_THROUGHPUT_STATISTICS_TIMER,      (ke_msg_func_t)qpps_throughput_statistics_timer_handler},
#endif
};

/// Specifies the message handlers that are common to all states.
const struct ke_state_handler qpps_default_handler = KE_STATE_HANDLER(qpps_default_state);

#endif /* #if (BLE_QPP_SERVER) */

/// @} QPPSTASK

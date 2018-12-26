/**
 ****************************************************************************************
 *
 * @file yells_task.c
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
 * @addtogroup YELLSTASK
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"

#if (BLE_YELL_SERVER)

#include "gap.h"
#include "gattc_task.h"
#include "yells.h"
#include "yells_task.h"
#include "app_yells.h"
#include "prf_utils.h"
#include "co_utils.h"
#include "ke_timer.h"

/*
 *  GLUCOSE PROFILE ATTRIBUTES
 ****************************************************************************************
 */
/*
 * LOCAL FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */

extern int yells_user_read_callback(void **data);
extern void yells_user_read_done_callback(void);

/**
 ****************************************************************************************
 * @brief Handles reception of the @ref YELLS_SEND_DATA_REQ message.
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int yells_send_data_req_handler(ke_msg_id_t const msgid,
                                       struct yells_send_data_req const *param,
                                       ke_task_id_t const dest_id,
                                       ke_task_id_t const src_id)
{
    uint8_t conidx = CONHDL2CONIDX(param->conhdl);
    uint8_t state = ke_state_get(dest_id);
    uint8_t status = PRF_ERR_REQ_DISALLOWED;

    if (state == YELLS_IDLE)
    {
        struct yells_env_tag *yells_env = PRF_ENV_GET(YELLS, yells);

        if (yells_env->ntf_cfg[conidx] == YELLS_VALUE_NTF_ON)
        {
            // struct yells_send_data_rsp yells_data_send_rsp;
            // Allocate the GATT indication message
            struct gattc_send_evt_cmd *yells_send_val = KE_MSG_ALLOC_DYN(
                GATTC_SEND_EVT_CMD, KE_BUILD_ID(TASK_GATTC, conidx), dest_id, gattc_send_evt_cmd, param->length);

            // Fill in the parameter structure
            yells_send_val->operation = GATTC_NOTIFY;
            yells_send_val->handle = YELLS_HANDLE(YELLS_IDX_TX_VAL);
            yells_send_val->length = param->length;
            memcpy(&yells_send_val->value[0], &param->data[0], param->length);
            ke_msg_send(yells_send_val);
            status = (GAP_ERR_NO_ERROR);
        }
    }
    // send command complete if an error occurs
    if (status != GAP_ERR_NO_ERROR)
    {
        // send completed information to APP task that contains error status
        struct yells_send_data_rsp *send_rsp = KE_MSG_ALLOC(YELLS_SEND_DATA_RSP, src_id, dest_id, yells_send_data_rsp);

        send_rsp->status = status;
        send_rsp->conhdl = param->conhdl;

        ke_msg_send(send_rsp);
    }
    return (KE_MSG_CONSUMED);
}
/**
 ****************************************************************************************
 * @brief Handles @ref GATTC_CMP_EVT for GATTC_NOTIFY and GATT_INDICATE message meaning
 * that Measurement notification/indication has been correctly sent to peer device
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
    uint8_t conidx = KE_IDX_GET(src_id);
    uint8_t state = ke_state_get(dest_id);

    if (state == YELLS_IDLE)
    {
        // Get the address of the environment
        struct yells_env_tag *yells_env = PRF_ENV_GET(YELLS, yells);

        switch (param->operation)
        {
            case GATTC_NOTIFY:
            {
                struct yells_send_data_rsp *send_rsp = KE_MSG_ALLOC(
                    YELLS_SEND_DATA_RSP, prf_dst_task_get(&(yells_env->prf_env), conidx), dest_id, yells_send_data_rsp);

                send_rsp->status = param->status;
                send_rsp->conhdl = CONIDX2CONHDL(conidx);
                ke_msg_send(send_rsp);
            }
            break;
            default:
                break;
        }
    }

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles reception of the @ref GATT_CODE_ATT_WR_CMD_IND message.
 * The handler compares the new values with current ones and notifies them if they changed.
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gattc_write_req_ind_handler(ke_msg_id_t const msgid,
                                       struct gattc_write_req_ind *param,
                                       ke_task_id_t const dest_id,
                                       ke_task_id_t const src_id)
{
    uint8_t state = ke_state_get(dest_id);
    uint8_t conidx = KE_IDX_GET(src_id);

    if (state == YELLS_IDLE)
    {
        uint8_t status = GAP_ERR_NO_ERROR;
        struct gattc_write_cfm *cfm;

        // Get the address of the environment
        struct yells_env_tag *yells_env = PRF_ENV_GET(YELLS, yells);
        uint8_t att_idx = param->handle - yells_env->start_hdl;

        if (param->offset != 0)
        {
            status = ATT_ERR_UNLIKELY_ERR;
        }
        // check if it's a client configuration char
        else if (att_idx == YELLS_IDX_TX_VAL_NTF_CFG)
        {
            uint16_t value = co_read16p(param->value);

            if (value == PRF_CLI_STOP_NTFIND)
            {
                yells_env->ntf_cfg[conidx] = YELLS_VALUE_NTF_OFF;
            }
            else if (value == PRF_CLI_START_NTF) // PRF_CLI_START_NTF
            {
                yells_env->ntf_cfg[conidx] = YELLS_VALUE_NTF_ON;
            }
            else
            {
                status = PRF_APP_ERROR;
            }

            if (status == GAP_ERR_NO_ERROR)
            {
                // Inform APP of configuration change
                struct yells_cfg_indntf_ind *ind =
                    KE_MSG_ALLOC(YELLS_CFG_INDNTF_IND, prf_dst_task_get(&(yells_env->prf_env), conidx), dest_id,
                                 yells_cfg_indntf_ind);
                ind->cfg_val = yells_env->ntf_cfg[conidx];
                ind->conhdl = CONIDX2CONHDL(conidx);
                ke_msg_send(ind);
            }
        }

        else if (att_idx == YELLS_IDX_RX_DATA_VAL)
        {
            if (param->length <= YELL_DATA_MAX_LEN)
            {
                // inform APP of configuration change
                struct yells_get_data_ind *ind =
                    KE_MSG_ALLOC_DYN(YELLS_GET_DATA_IND, prf_dst_task_get(&(yells_env->prf_env), conidx), dest_id,
                                     yells_get_data_ind, param->length);

                // Send received data to app value
                ind->length = param->length;
                ind->conhdl = CONIDX2CONHDL(conidx);
                memcpy(ind->data, param->value, param->length);

                ke_msg_send(ind);
            }
            else
            {
                status = YELLS_ERR_RX_DATA_EXCEED_MAX_LENGTH;
            }
        }
        else
        {
            status = PRF_ERR_INVALID_PARAM;
        }
        // Send write response
        cfm = KE_MSG_ALLOC(GATTC_WRITE_CFM, src_id, dest_id, gattc_write_cfm);
        cfm->handle = param->handle;
        cfm->status = status;
        ke_msg_send(cfm);
    }

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles reception of the read request from peer device
 *
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
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
    // uint8_t conidx = KE_IDX_GET(dest_id);
    struct yells_env_tag *yells_env = PRF_ENV_GET(YELLS, yells);
    void *data;
    uint32_t size;

    if (state == YELLS_IDLE)
    {
        if ((param->handle - yells_env->start_hdl) == YELLS_IDX_ENUM_VAL)
        {
            size = yells_user_read_callback(&data);
            struct gattc_read_cfm *msg = KE_MSG_ALLOC_DYN(GATTC_READ_CFM, src_id, dest_id, gattc_read_cfm, size);
            msg->handle = param->handle;
            msg->length = size;
            msg->status = ATT_ERR_NO_ERROR;
            memcpy(&msg->value[0], data, msg->length);
            yells_user_read_done_callback();
            ke_msg_send(msg);
        }
    }

    return (KE_MSG_CONSUMED);
}

static int yells_notify_timer_handler(ke_msg_id_t const msgid,
                                      void const *param,
                                      ke_task_id_t const dest_id,
                                      ke_task_id_t const src_id)
{
    uint8_t conidx = KE_IDX_GET(dest_id);
    struct yells_env_tag *yells_env_tag = PRF_ENV_GET(YELLS, yells);
    if (yells_env_tag->ntf_cfg[conidx] == YELLS_VALUE_NTF_ON)
    {
        ke_timer_set(YELLS_NTF_TIMER, yells_env_tag->prf_env.prf_task, 200);
    }
    else
    {
        ke_timer_clear(YELLS_NTF_TIMER, dest_id);
    }
    return (KE_MSG_CONSUMED);
}

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

/// Default State handlers definition
const struct ke_msg_handler yells_default_state[] = {
    {YELLS_SEND_DATA_REQ, (ke_msg_func_t)yells_send_data_req_handler},
    {GATTC_WRITE_REQ_IND, (ke_msg_func_t)gattc_write_req_ind_handler},
    {GATTC_CMP_EVT, (ke_msg_func_t)gattc_cmp_evt_handler},
    {GATTC_READ_REQ_IND, (ke_msg_func_t)gattc_read_req_ind_handler},
    {YELLS_NTF_TIMER, (ke_msg_func_t)yells_notify_timer_handler},
};

/// Specifies the message handlers that are common to all states.
const struct ke_state_handler yells_default_handler = KE_STATE_HANDLER(yells_default_state);

#endif /* #if (BLE_GL_SENSOR) */

/// @} YELLSTASK

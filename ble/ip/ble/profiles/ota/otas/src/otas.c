/**
 ****************************************************************************************
 *
 * @file OTAS.c
 *
 * @brief OTA server implementation.
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
 * @addtogroup OTAS
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
#include "prf_utils.h"
#include "otas.h"
#include "ke_mem.h" // WJ: remove later
#include "ke_timer.h"

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

/*
 * LOCAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */
/**
 ****************************************************************************************
 * @brief Initialization of the OTAS module.
 * This function performs all the initializations of the Profile module.
 *  - Creation of database (if it's a service)
 *  - Allocation of profile required memory
 *  - Initialization of task descriptor to register application
 *      - Task State array
 *      - Number of tasks
 *      - Default task handler
 *
 * @param[out]    env        Collector or Service allocated environment data.
 * @param[in|out] start_hdl  Service start handle (0 - dynamically allocated), only applies for services.
 * @param[in]     app_task   Application task number.
 * @param[in]     sec_lvl    Security level (AUTH, EKS and MI field of @see enum attm_value_perm_mask)
 * @param[in]     param      Configuration parameters of profile collector or service (32 bits aligned)
 *
 * @return status code to know if profile initialization succeed or not.
 ****************************************************************************************
 */
static uint8_t otas_init(
    struct prf_task_env *env, uint16_t *start_hdl, uint16_t app_task, uint8_t sec_lvl, void *params)
{
    uint8_t idx;
    //-------------------- allocate memory required for the profile  ---------------------

    struct otas_env_tag *otas_env = (struct otas_env_tag *)ke_malloc(
        sizeof(struct otas_env_tag) + sizeof(ke_state_t) * OTAS_IDX_MAX, KE_MEM_ATT_DB);

    otas_env->env = (struct otas_cnx_env **)ke_malloc(sizeof(struct otas_cnx_env *) * OTAS_IDX_MAX, KE_MEM_ATT_DB);

    // allocate OTAS required environment variable
    env->env = (prf_env_t *)otas_env;

    otas_env->prf_env.app_task = app_task | (PERM_GET(sec_lvl, SVC_MI) ? PERM(PRF_MI, ENABLE) : PERM(PRF_MI, DISABLE));
    otas_env->prf_env.prf_task = env->task | PERM(PRF_MI, ENABLE);

    struct otas_encrypt_tag *encrypt = (struct otas_encrypt_tag *)params;
    memcpy(otas_env->encrypt.ctr, encrypt->ctr, 16);
    memcpy(otas_env->encrypt.key, encrypt->key, 16);

    // initialize environment variable
    env->id = TASK_ID_OTAS;
    env->desc.idx_max = OTAS_IDX_MAX;
    env->desc.state = otas_env->state;
    env->desc.default_handler = &otas_default_handler;

    for (idx = 0; idx < OTAS_IDX_MAX; idx++)
    {
        otas_env->env[idx] = NULL;
        // service is ready, go into an Idle state
        ke_state_set(KE_BUILD_ID(env->task, idx), OTAS_FREE);
    }

    return GAP_ERR_NO_ERROR;
}

/**
 ****************************************************************************************
 * @brief Destruction of the OTAS module - due to a reset for instance.
 * This function clean-up allocated memory (attribute database is destroyed by another
 * procedure)
 *
 * @param[in|out]    env        Collector or Service allocated environment data.
 ****************************************************************************************
 */
static void otas_destroy(struct prf_task_env *env)
{
    uint8_t idx;
    struct otas_env_tag *otas_env = (struct otas_env_tag *)env->env;

    // cleanup environment variable for each task instances
    for (idx = 0; idx < OTAS_IDX_MAX; idx++)
    {
        if (otas_env->env[idx] != NULL)
        {
            if (otas_env->env[idx]->host_cmd != NULL)
            {
                ke_msg_free(ke_param2msg(otas_env->env[idx]->host_cmd));
                otas_env->env[idx]->host_cmd = NULL;
            }

            if (otas_env->env[idx]->data != NULL)
            {
                ke_free(otas_env->env[idx]->data);
                otas_env->env[idx]->data = NULL;
            }

            ke_free(otas_env->env[idx]);
        }
    }

    ke_free(otas_env->env);

    // free profile environment variables
    env->env = NULL;
    ke_free(otas_env);
}

/**
 ****************************************************************************************
 * @brief Handles Connection creation
 *
 * @param[in|out]    env        Collector or Service allocated environment data.
 * @param[in]        conidx     Connection index
 ****************************************************************************************
 */
static void otas_create(struct prf_task_env *env, uint8_t conidx)
{
    /* Put OTA server in Idle state */
    ke_state_set(KE_BUILD_ID(env->task, conidx), OTAS_IDLE);
}

/**
 ****************************************************************************************
 * @brief Handles Disconnection
 *
 * @param[in|out]    env        Collector or Service allocated environment data.
 * @param[in]        conidx     Connection index
 * @param[in]        reason     Detach reason
 ****************************************************************************************
 */
static void otas_cleanup(struct prf_task_env *env, uint8_t conidx, uint8_t reason)
{
    struct otas_env_tag *otas_env = (struct otas_env_tag *)env->env;

    // clean-up environment variable allocated for task instance
    if (otas_env->env[conidx] != NULL)
    {
        if (otas_env->env[conidx]->host_cmd != NULL)
        {
            ke_msg_free(ke_param2msg(otas_env->env[conidx]->host_cmd));
            otas_env->env[conidx]->host_cmd = NULL;
        }

        if (otas_env->env[conidx]->data != NULL)
        {
            ke_free(otas_env->env[conidx]->data);
            otas_env->env[conidx]->data = NULL;
        }

        ke_free(otas_env->env[conidx]);
        otas_env->env[conidx] = NULL;
    }

    /* Put OTA Client in Free state */
    ke_state_set(KE_BUILD_ID(env->task, conidx), OTAS_FREE);
}

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

/// OTAS Task interface required by profile manager
const struct prf_task_cbs otas_itf = {
    otas_init, otas_destroy, otas_create, otas_cleanup,
};

/*
 * EXPORTED FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */
const struct prf_task_cbs *otas_prf_itf_get(void)
{
    return &otas_itf;
}

void otas_enable_rsp_send(struct otas_env_tag *otas_env, uint8_t conidx, uint8_t status)
{
    // Send to APP the details of the discovered attributes on OTAS
    struct otas_enable_rsp *rsp = KE_MSG_ALLOC(OTAS_ENABLE_RSP, prf_dst_task_get(&(otas_env->prf_env), conidx),
                                               prf_src_task_get(&(otas_env->prf_env), conidx), otas_enable_rsp);
    rsp->status = status;
    rsp->conhdl = CONIDX2CONHDL(conidx);

    if (status == GAP_ERR_NO_ERROR)
    {
        rsp->otas = otas_env->env[conidx]->otas;
        // Register OTAS task in gatt for indication/notifications
        prf_register_atthdl2gatt(&(otas_env->prf_env), conidx, &(otas_env->env[conidx]->otas.svc));
    }
    // Go to IDLE state
    ke_state_set(prf_src_task_get(&(otas_env->prf_env), conidx), OTAS_IDLE);

    ke_msg_send(rsp);
}

void otas_host_cmd_req_send(uint16_t conhdl, uint8_t *pdata, size_t len, uint8_t charCode)
{
    struct otas_env_tag *otas_env = PRF_ENV_GET(OTAS, otas);

    struct otas_host_cmd_req *msg =
        KE_MSG_ALLOC_DYN(OTAS_HOST_CMD_REQ, prf_src_task_get(&otas_env->prf_env, CONHDL2CONIDX(conhdl)),
                         prf_dst_task_get(&otas_env->prf_env, CONHDL2CONIDX(conhdl)), otas_host_cmd_req, len);

    /// Connection handle
    msg->conhdl = conhdl;
    msg->length = len;
    msg->offset = 0;
    msg->cusor = 0;
    msg->charCode = charCode;
    memcpy(msg->data, pdata, len);

    // Send the message
    ke_msg_send(msg);
}

void otas_start_encrypt_req(uint16_t conhdl, uint8_t key[16], uint8_t plain_data[16])
{
    struct otas_env_tag *otas_env = PRF_ENV_GET(OTAS, otas);

    struct gapm_use_enc_block_cmd *msg =
        KE_MSG_ALLOC(GAPM_USE_ENC_BLOCK_CMD, TASK_GAPM, prf_src_task_get(&otas_env->prf_env, CONHDL2CONIDX(conhdl)),
                     gapm_use_enc_block_cmd);
    // Ask to the AES to generate the SK by giving the SKD an LTK
    msg->operation = GAPM_USE_ENC_BLOCK;
    memcpy(msg->operand_1, key, 16);
    memcpy(msg->operand_2, plain_data, 16);
    // Send the command to start the SK generation
    ke_msg_send(msg);
}

#endif /* (BLE_OTA_SERVER) */

/// @} OTAS

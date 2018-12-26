/**
 ****************************************************************************************
 *
 * @file qppc.c
 *
 * @brief QBlue Private Profile Client implementation.
 *
 * Copyright(C) 2015 NXP Semiconductors N.V.
 * All rights reserved.
 *
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup QPPC
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
#include "prf_utils.h"
#include "qppc.h"
#include "ke_mem.h" // WJ: remove later

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
 * @brief Initialization of the QPPC module.
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
static uint8_t qppc_init(
    struct prf_task_env *env, uint16_t *start_hdl, uint16_t app_task, uint8_t sec_lvl, void *params)
{
    uint8_t idx;
    //-------------------- allocate memory required for the profile  ---------------------

    struct qppc_env_tag *qppc_env = (struct qppc_env_tag *)ke_malloc(
        sizeof(struct qppc_env_tag) + sizeof(ke_state_t) * QPPC_IDX_MAX, KE_MEM_ATT_DB);

    qppc_env->env = (struct qppc_cnx_env **)ke_malloc(sizeof(struct qppc_cnx_env *) * QPPC_IDX_MAX, KE_MEM_ATT_DB);

    // allocate QPPC required environment variable
    env->env = (prf_env_t *)qppc_env;

    qppc_env->prf_env.app_task = app_task | (PERM_GET(sec_lvl, SVC_MI) ? PERM(PRF_MI, ENABLE) : PERM(PRF_MI, DISABLE));
    qppc_env->prf_env.prf_task = env->task | PERM(PRF_MI, ENABLE);

    // initialize environment variable
    env->id = TASK_ID_QPPC;
    env->desc.idx_max = QPPC_IDX_MAX;
    env->desc.state = qppc_env->state;
    env->desc.default_handler = &qppc_default_handler;

    for (idx = 0; idx < QPPC_IDX_MAX; idx++)
    {
        qppc_env->env[idx] = NULL;
        // service is ready, go into an Idle state
        ke_state_set(KE_BUILD_ID(env->task, idx), QPPC_FREE);
    }

    return GAP_ERR_NO_ERROR;
}

/**
 ****************************************************************************************
 * @brief Destruction of the QPPC module - due to a reset for instance.
 * This function clean-up allocated memory (attribute database is destroyed by another
 * procedure)
 *
 * @param[in|out]    env        Collector or Service allocated environment data.
 ****************************************************************************************
 */
static void qppc_destroy(struct prf_task_env *env)
{
    uint8_t idx;
    struct qppc_env_tag *qppc_env = (struct qppc_env_tag *)env->env;

    // cleanup environment variable for each task instances
    for (idx = 0; idx < QPPC_IDX_MAX; idx++)
    {
        if (qppc_env->env[idx] != NULL)
        {
            ke_free(qppc_env->env[idx]);
        }
    }

    ke_free(qppc_env->env);

    // free profile environment variables
    env->env = NULL;
    ke_free(qppc_env);
}

/**
 ****************************************************************************************
 * @brief Handles Connection creation
 *
 * @param[in|out]    env        Collector or Service allocated environment data.
 * @param[in]        conidx     Connection index
 ****************************************************************************************
 */
static void qppc_create(struct prf_task_env *env, uint8_t conidx)
{
    /* Put QPP Client in Idle state */
    ke_state_set(KE_BUILD_ID(env->task, conidx), QPPC_IDLE);
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
static void qppc_cleanup(struct prf_task_env *env, uint8_t conidx, uint8_t reason)
{
    struct qppc_env_tag *qppc_env = (struct qppc_env_tag *)env->env;

    // clean-up environment variable allocated for task instance
    if (qppc_env->env[conidx] != NULL)
    {
        ke_free(qppc_env->env[conidx]);
        qppc_env->env[conidx] = NULL;
    }

    /* Put QPP Client in Free state */
    ke_state_set(KE_BUILD_ID(env->task, conidx), QPPC_FREE);
}

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

/// QPPC Task interface required by profile manager
const struct prf_task_cbs qppc_itf = {
    qppc_init, qppc_destroy, qppc_create, qppc_cleanup,
};

/*
 * EXPORTED FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */
const struct prf_task_cbs *qppc_prf_itf_get(void)
{
    return &qppc_itf;
}

void qppc_enable_rsp_send(struct qppc_env_tag *qppc_env, uint8_t conidx, uint8_t status)
{
    // Send to APP the details of the discovered attributes on QPPS
    struct qppc_enable_rsp *rsp = KE_MSG_ALLOC(QPPC_ENABLE_RSP, prf_dst_task_get(&(qppc_env->prf_env), conidx),
                                               prf_src_task_get(&(qppc_env->prf_env), conidx), qppc_enable_rsp);
    rsp->status = status;
    rsp->conhdl = CONIDX2CONHDL(conidx);

    if (status == GAP_ERR_NO_ERROR)
    {
        rsp->qpps = qppc_env->env[conidx]->qpps;
        // Register QPPC task in gatt for indication/notifications
        prf_register_atthdl2gatt(&(qppc_env->prf_env), conidx, &(qppc_env->env[conidx]->qpps.svc));
        // Go to IDLE state
        ke_state_set(prf_src_task_get(&(qppc_env->prf_env), conidx), QPPC_IDLE);
    }

    ke_msg_send(rsp);
}

#endif /* (BLE_QPP_CLIENT) */

/// @} QPPC

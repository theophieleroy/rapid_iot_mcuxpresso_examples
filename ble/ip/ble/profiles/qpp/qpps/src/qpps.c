/**
 ****************************************************************************************
 *
 * @file qpps.c
 *
 * @brief QBlue Private Profile Server implementation.
 *
 * Copyright(C) 2015 NXP Semiconductors N.V.
 *
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup QPPS
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "rwip_config.h"

#if BLE_QPP_SERVER
#include "prf_utils.h"
#include "ke_mem.h"
#include "ke_timer.h"
#include "qpps.h"

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

/// Full QPP Database Description - Used to add attributes into the database
const struct attm_desc_128 qpps_att_db[QPPS_IDX_NB] =
{
    // Service Declaration
    [QPPS_IDX_SVC]                  =   {"\x00\x28", PERM(RD, ENABLE), (PERM_UUID_128 <<PERM_POS_SVC_UUID_LEN), 0},

    // Rx Data Characteristic Definition

    // Characteristic Declaration
    [QPPS_IDX_RX_DATA_CHAR]         =   {"\x03\x28", PERM(RD, ENABLE), 0, 0},
    // Characteristic Value
    [QPPS_IDX_RX_DATA_VAL]          =   {QPPS_RX_CHAR_UUID, PERM(WRITE_COMMAND,ENABLE), PERM(RI, ENABLE)|(PERM_UUID_128 <<PERM_POS_UUID_LEN),  QPP_DATA_MAX_LEN},
    // User Descriptor
    [QPPS_IDX_RX_DATA_USER_DESP]    =   {"\x01\x29", PERM(RD, ENABLE), 0, sizeof(QPPS_VERSION)},

    // Tx Data Characteristic Definition

    // Characteristic Declaration
    [QPPS_IDX_TX_DATA_CHAR]         =   {"\x03\x28", PERM(RD, ENABLE), 0, 0},
    // Characteristic Value
    [QPPS_IDX_TX_DATA_VAL]          =   {QPPS_TX_CHAR_UUID, PERM(NTF, ENABLE), PERM(RI, ENABLE)|(PERM_UUID_128 <<PERM_POS_UUID_LEN), QPP_DATA_MAX_LEN},
    // Client Characteristic Configuration Descriptor
    [QPPS_IDX_TX_DATA_NTF_CFG]      =   {"\x02\x29", PERM(RD, ENABLE)|PERM(WRITE_REQ, ENABLE), 0, 0},
};

/*
 * DEFINES
 ****************************************************************************************
 */

/*
 * LOCAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */
/**
 ****************************************************************************************
 * @brief Initialization of the QPPS module.
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
static uint8_t qpps_init (struct prf_task_env* env, uint16_t* start_hdl
                , uint16_t app_task, uint8_t sec_lvl, struct qpps_db_cfg* params)
{
    //------------------ create the attribute database for the profile -------------------
    // Service content flag
    uint32_t cfg_flag = QPPS_MANDATORY_MASK;
    // DB Creation Status
    uint8_t status = ATT_ERR_NO_ERROR;
    // Service permission
    uint8_t svc_perm = ((sec_lvl ) & (PERM_MASK_SVC_DIS | PERM_MASK_SVC_AUTH | PERM_MASK_SVC_EKS))|(PERM_UUID_128<<PERM_POS_SVC_UUID_LEN);

    // Create QPPS Database
    status = attm_svc_create_db_128(start_hdl, (uint8_t*)&QPP_SVC_PRIVATE_UUID[0], (uint8_t *)&cfg_flag,
             QPPS_IDX_NB, NULL, env->task, &qpps_att_db[0], svc_perm);

    //-------------------- allocate memory required for the profile  ---------------------

    if (status == ATT_ERR_NO_ERROR)
    {
        // Set QPPS version
        attm_att_set_value(*start_hdl+QPPS_IDX_RX_DATA_USER_DESP, sizeof(QPPS_VERSION), 0, (uint8_t *)QPPS_VERSION);

        // Allocate QPPS required environment variable
        struct qpps_env_tag* qpps_env =
                (struct qpps_env_tag* ) ke_malloc(sizeof(struct qpps_env_tag) + sizeof(uint8_t) * BLE_CONNECTION_MAX, KE_MEM_ATT_DB);

        // Initialize QPPS environment
        env->env                      = (prf_env_t*) qpps_env;

        qpps_env->start_hdl           = *start_hdl;
        qpps_env->prf_env.app_task    = app_task
                | (PERM_GET(sec_lvl, SVC_MI) ? PERM(PRF_MI, ENABLE) : PERM(PRF_MI, DISABLE));
        qpps_env->prf_env.prf_task    = env->task | PERM(PRF_MI, DISABLE);

        // initialize environment variable
        env->id                     = TASK_ID_QPPS;
        env->desc.idx_max           = QPPS_IDX_MAX;
        env->desc.state             = qpps_env->state;
        env->desc.default_handler   = &qpps_default_handler;

        // service is ready, go into an idle state
        ke_state_set((env->task), QPPS_IDLE);

#if defined(CFG_QPP_SHOW_THROUGHPUT)
        ke_timer_set(QPPS_THROUGHPUT_STATISTICS_TIMER, qpps_env->prf_env.prf_task, QPP_THROUGHPUT_STATS_TIMEOUT);
#endif
    }

    return status;
}

/**
 ****************************************************************************************
 * @brief Destruction of the QPPS module - due to a reset for instance.
 * This function clean-up allocated memory (attribute database is destroyed by another
 * procedure)
 *
 * @param[in|out]    env        Collector or Service allocated environment data.
 ****************************************************************************************
 */
static void qpps_destroy(struct prf_task_env* env)
{
    struct qpps_env_tag* qpps_env = (struct qpps_env_tag*) env->env;

    // free profile environment variables
    env->env = NULL;
    ke_free(qpps_env);
}

/**
 ****************************************************************************************
 * @brief Handles Connection creation
 *
 * @param[in|out]    env        Collector or Service allocated environment data.
 * @param[in]        conidx     Connection index
 ****************************************************************************************
 */
static void qpps_create(struct prf_task_env* env, uint8_t conidx)
{
    struct qpps_env_tag *qpps_env = (struct qpps_env_tag *)env->env;
    ASSERT_ERR(conidx < BLE_CONNECTION_MAX);

    // force notification config to zero when peer device is connected
    qpps_env->ntf_cfg[conidx] = 0;
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
static void qpps_cleanup(struct prf_task_env* env, uint8_t conidx, uint8_t reason)
{
    struct qpps_env_tag *qpps_env = (struct qpps_env_tag *)env->env;
    ASSERT_ERR(conidx < BLE_CONNECTION_MAX);

    // force notification config to zero when peer device is connected
    qpps_env->ntf_cfg[conidx] = 0;

#if defined(CFG_QPP_SHOW_THROUGHPUT)
    // clear statistics for this connection
    memset(&qpps_stats[conidx], 0, sizeof(struct qpp_throughput_stats));
#endif
}

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

/// QPPS Task interface required by profile manager
const struct prf_task_cbs qpps_itf =
{
        (prf_init_fnct) qpps_init,
        qpps_destroy,
        qpps_create,
        qpps_cleanup,
};

/*
 * EXPORTED FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */

const struct prf_task_cbs* qpps_prf_itf_get(void)
{
   return &qpps_itf;
}

#endif /* BLE_QPPS */
/// @} QPPS

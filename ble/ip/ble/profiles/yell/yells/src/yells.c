/**
 ****************************************************************************************
 *
 * @file yells.c
 *
 * @brief YELL Server implementation.
 *
 * Copyright(C) 2015 NXP Semiconductors N.V.
 *
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup YELLS
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "rwip_config.h"

#if BLE_YELL_SERVER
#include "yells.h"
#include "yells_task.h"
#include "prf_utils.h"
#include "ke_mem.h"
#include "ke_timer.h"

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

/// Full YELL Database Description - Used to add attributes into the database
const struct attm_desc_128 yells_att_db[YELLS_IDX_NB] = {
        // Service Declaration
        [YELLS_IDX_SVC] = {"\x00\x28", PERM(RD, ENABLE), (PERM_UUID_128 << PERM_POS_SVC_UUID_LEN), 0},

        // Received data Characteristic Declaration
        [YELLS_IDX_RX_DATA_CHAR] = {"\x03\x28", PERM(RD, ENABLE), 0, 0},
        // Received data Characteristic Value
        [YELLS_IDX_RX_DATA_VAL] = {YELLS_RX_CHAR_UUID, PERM(WRITE_COMMAND, ENABLE),
                                   PERM(RI, ENABLE) | (PERM_UUID_128 << PERM_POS_UUID_LEN), YELL_DATA_MAX_LEN},
        // User Descriptor
        [YELLS_IDX_RX_DATA_USER_DESP] = {"\x01\x29", PERM(RD, ENABLE), 0, sizeof(YELLS_VERSION)},

        // Tx data to client with these Characters

        // Characteristic Declaration
        [YELLS_IDX_TX_VAL_CHAR] = {"\x03\x28", PERM(RD, ENABLE), 0, 0},
        // Characteristic Value
        [YELLS_IDX_TX_VAL] = {YELLS_TX_CHAR_UUID, PERM(NTF, ENABLE),
                              PERM(RI, ENABLE) | (PERM_UUID_128 << PERM_POS_UUID_LEN), YELL_DATA_MAX_LEN},
        // Client Characteristic Configuration Descriptor
        [YELLS_IDX_TX_VAL_NTF_CFG] = {"\x02\x29", PERM(RD, ENABLE) | PERM(WRITE_REQ, ENABLE), 0, 0},

        // Tx data to client with these Characters

        // Characteristic Declaration
        [YELLS_IDX_ENUM_CHAR] = {"\x03\x28", PERM(RD, ENABLE), 0, 0},
        // Characteristic Value
        [YELLS_IDX_ENUM_VAL] = {YELLS_ENUM_CHAR_UUID, PERM(RD, ENABLE),
                                PERM(RI, ENABLE) | (PERM_UUID_128 << PERM_POS_UUID_LEN), YELL_DATA_MAX_LEN},
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
 * @brief Initialization of the YELLS module.
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

static uint8_t yells_init(
    struct prf_task_env *env, uint16_t *start_hdl, uint16_t app_task, uint8_t sec_lvl, struct yells_db_cfg *params)
{
    //------------------ create the attribute database for the profile -------------------
    // Service content flag
    uint32_t cfg_flag = YELLS_MANDATORY_MASK;
    // DB Creation Statis
    uint8_t status = ATT_ERR_NO_ERROR;

    // Create YELLS Database
    status = attm_svc_create_db_128(start_hdl, (uint8_t *)&YELL_SVC_PRIVATE_UUID[0], (uint8_t *)&cfg_flag, YELLS_IDX_NB,
                                    NULL, env->task, &yells_att_db[0],
                                    ((sec_lvl) & (PERM_MASK_SVC_DIS | PERM_MASK_SVC_AUTH | PERM_MASK_SVC_EKS)) |
                                        (PERM_UUID_128 << PERM_POS_SVC_UUID_LEN));

    //-------------------- allocate memory required for the profile  ---------------------

    if (status == ATT_ERR_NO_ERROR)
    {
        // Allocate YELLS required environment variable
        struct yells_env_tag *yells_env = (struct yells_env_tag *)ke_malloc(
            sizeof(struct yells_env_tag) + sizeof(uint8_t) * BLE_CONNECTION_MAX, KE_MEM_ATT_DB);

        attm_att_set_value(*start_hdl + YELLS_IDX_RX_DATA_USER_DESP, sizeof(YELLS_VERSION), 0,
                           (uint8_t *)YELLS_VERSION);

        yells_env->start_hdl = *start_hdl;
        *start_hdl += YELLS_IDX_NB;
        env->env = (prf_env_t *)yells_env;

        yells_env->prf_env.app_task =
            app_task | (PERM_GET(sec_lvl, SVC_MI) ? PERM(PRF_MI, ENABLE) : PERM(PRF_MI, DISABLE));
        yells_env->prf_env.prf_task = env->task | PERM(PRF_MI, DISABLE);

        // initialize environment variable
        env->id = TASK_ID_YELLS;
        env->desc.idx_max = YELLS_IDX_MAX;
        env->desc.state = yells_env->state;
        env->desc.default_handler = &yells_default_handler;

        ke_state_set((env->task), YELLS_IDLE);
    }

    return status;
}

/**
 ****************************************************************************************
 * @brief Destruction of the YELLS module - due to a reset for instance.
 * This function clean-up allocated memory (attribute database is destroyed by another
 * procedure)
 *
 * @param[in|out]    env        Collector or Service allocated environment data.
 ****************************************************************************************
 */
static void yells_destroy(struct prf_task_env *env)
{
    struct yells_env_tag *yells_env = (struct yells_env_tag *)env->env;
    // free profile environment variables
    env->env = NULL;
    ke_free(yells_env);
}

/**
 ****************************************************************************************
 * @brief Handles Connection creation
 *
 * @param[in|out]    env        Collector or Service allocated environment data.
 * @param[in]        conidx     Connection index
 ****************************************************************************************
 */
static void yells_create(struct prf_task_env *env, uint8_t conidx)
{
    struct yells_env_tag *yells_env = (struct yells_env_tag *)env->env;
    ASSERT_ERR(conidx < BLE_CONNECTION_MAX);

    // force notification config to zero when peer device is connected
    yells_env->ntf_cfg[conidx] = 0;
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
static void yells_cleanup(struct prf_task_env *env, uint8_t conidx, uint8_t reason)
{
    struct yells_env_tag *yells_env = (struct yells_env_tag *)env->env;
    ASSERT_ERR(conidx < BLE_CONNECTION_MAX);

    // force notification config to zero when peer device is connected
    yells_env->ntf_cfg[conidx] = 0;
}

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

/// YELLS Task interface required by profile manager
const struct prf_task_cbs yells_itf = {
    (prf_init_fnct)yells_init, yells_destroy, yells_create, yells_cleanup,
};

/*
 * EXPORTED FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */

const struct prf_task_cbs *yells_prf_itf_get(void)
{
    return &yells_itf;
}

#endif /* BLE_YELLS */
/// @} YELLS

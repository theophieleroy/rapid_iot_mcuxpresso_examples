/**
 ****************************************************************************************
 *
 * @file otac.c
 *
 * @brief OTA Profile Client implementation.
 *
 * Copyright(C) 2015 NXP Semiconductors N.V.
 *
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup OTAC
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "rwip_config.h"

#if BLE_OTA_CLIENT
#include "prf_utils.h"
#include "ke_mem.h"
#include "ke_timer.h"
#include "gattc.h"
#include "otac.h"
#include "otap_interface.h"
#include "otap_support.h"
#include "otap_client.h"

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */
void otac_transmit_rsp_to_client(uint16_t conhdl, uint8_t *pdata, size_t len);
/// Full OTA Database Description - Used to add attributes into the database
const struct attm_desc_128 otac_att_db[OTAC_IDX_NB] = {
        // OTA Service Declaration
        [OTAC_IDX_SVC] = {"\x00\x28", PERM(RD, ENABLE), 0, 0},

        // Target Response Characteristic Declaration
        [OTAC_IDX_TARGET_RSP_CHAR] = {"\x03\x28", PERM(RD, ENABLE), 0, 0},
        // Target Response Characteristic Value
        [OTAC_IDX_TARGET_RSP_VAL] = {ATT_CHAR_OTA_TARGET_RSP_UUID, PERM(IND, ENABLE) | PERM(WRITE_REQ, ENABLE),
                                     PERM(RI, ENABLE) | (PERM_UUID_128 << PERM_POS_UUID_LEN), OTA_BLOCK_MAX_LEN},
        // Target Response Client Characteristic Configuration Descriptor
        [OTAC_IDX_TARGET_RSP_NTF_CFG] = {"\x02\x29", PERM(RD, ENABLE) | PERM(WRITE_REQ, ENABLE), 0, 0},
        // Target Response User Descriptor
        [OTAC_IDX_TARGET_RSP_USER_DESP] = {"\x01\x29", PERM(RD, ENABLE) | PERM(RD, UNAUTH), 0, sizeof(OTAC_VERSION)},

        // Host Command Characteristic Declaration
        [OTAC_IDX_HOST_CMD_CHAR] = {"\x03\x28", PERM(RD, ENABLE), 0, 0},
        // Host Command Characteristic Value
        [OTAC_IDX_HOST_CMD_VAL] = {ATT_CHAR_OTA_HOST_CMD_UUID, PERM(WRITE_COMMAND, ENABLE),
                                   PERM(RI, ENABLE) | (PERM_UUID_128 << PERM_POS_UUID_LEN), OTA_BLOCK_MAX_LEN},

};

static uint16_t gOTAClientConhld = 0;

int otac_process_received_data(uint16_t length, uint8_t *pData)
{
    OtapClient_WriteImageToFlash(length, pData);
    return gOtaSucess_c;
}

/*
 * DEFINES
 ****************************************************************************************
 */

/*
 * LOCAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */

void otaClientRegisterConhdl(uint16_t conhdl)
{
    gOTAClientConhld = conhdl;
}

static uint16_t otaGetCurrentValidConhdl(void)
{
    return gOTAClientConhld;
}

uint16_t otac_get_max_mtu()
{
    return gattc_get_mtu(CONHDL2CONIDX(gOTAClientConhld));
}

void otac_download_Complete(void)
{
    struct otac_env_tag *otac_env = PRF_ENV_GET(OTAC, otac);

    OtapClient_BootInfoChange();
    struct gapc_disconnect_cmd *cmd = KE_MSG_ALLOC(
        GAPC_DISCONNECT_CMD, KE_BUILD_ID(TASK_GAPC, CONHDL2CONIDX(gOTAClientConhld)), TASK_APP, gapc_disconnect_cmd);

    cmd->operation = GAPC_DISCONNECT;
    cmd->reason = CO_ERROR_REMOTE_USER_TERM_CON;

    ke_msg_send(cmd);

    ke_timer_set(OTAC_TIMEOUT_IND, otac_env->prf_env.prf_task, 20);
}
/**
 ****************************************************************************************
 * @brief Initialization of the OTAC module.
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
static uint8_t otac_init(
    struct prf_task_env *env, uint16_t *start_hdl, uint16_t app_task, uint8_t sec_lvl, struct otac_db_cfg *params)
{
    //------------------ create the attribute database for the profile -------------------
    // Service content flag
    uint32_t cfg_flag = OTAC_MANDATORY_MASK;
    // DB Creation Status
    uint8_t status = ATT_ERR_NO_ERROR;
    // Service permission
    uint8_t svc_perm = ((sec_lvl) & (PERM_MASK_SVC_DIS | PERM_MASK_SVC_AUTH | PERM_MASK_SVC_EKS));

    // Create OTAC Database
    status = attm_svc_create_db_128(start_hdl, (uint8_t *)&ATT_SVC_OTA_UUID[0], (uint8_t *)&cfg_flag, OTAC_IDX_NB, NULL,
                                    env->task, &otac_att_db[0], svc_perm | (PERM_UUID_128 << PERM_POS_SVC_UUID_LEN));

    //-------------------- allocate memory required for the profile  ---------------------

    if (status == ATT_ERR_NO_ERROR)
    {
        // Set OTAC version
        attm_att_set_value(*start_hdl + OTAC_IDX_TARGET_RSP_USER_DESP, sizeof(OTAC_VERSION), 0,
                           (uint8_t *)OTAC_VERSION);

        // Allocate OTAC required environment variable
        struct otac_env_tag *otac_env = (struct otac_env_tag *)ke_malloc(sizeof(struct otac_env_tag), KE_MEM_ATT_DB);

        // Initialize OTAC environment
        env->env = (prf_env_t *)otac_env;

        otac_env->start_hdl = *start_hdl;
        otac_env->prf_env.app_task =
            app_task | (PERM_GET(sec_lvl, SVC_MI) ? PERM(PRF_MI, ENABLE) : PERM(PRF_MI, DISABLE));
        otac_env->prf_env.prf_task = env->task | PERM(PRF_MI, DISABLE);
        otac_env->target_rsp = NULL;

        // initialize environment variable
        env->id = TASK_ID_OTAC;
        env->desc.idx_max = OTAC_IDX_MAX;
        env->desc.state = otac_env->state;
        env->desc.default_handler = &otac_default_handler;

        // service is ready, go into an idle state
        ke_state_set((env->task), OTAC_IDLE);
    }

    return status;
}

/**
 ****************************************************************************************
 * @brief Destruction of the OTAC module - due to a reset for instance.
 * This function clean-up allocated memory (attribute database is destroyed by another
 * procedure)
 *
 * @param[in|out]    env        Collector or Service allocated environment data.
 ****************************************************************************************
 */
static void otac_destroy(struct prf_task_env *env)
{
    struct otac_env_tag *otac_env = (struct otac_env_tag *)env->env;

    if (otac_env->target_rsp != NULL)
    {
        ke_msg_free(ke_param2msg(otac_env->target_rsp));
        otac_env->target_rsp = NULL;
    }

    // free profile environment variables
    env->env = NULL;
    ke_free(otac_env);
}

/**
 ****************************************************************************************
 * @brief Handles Connection creation
 *
 * @param[in|out]    env        Collector or Service allocated environment data.
 * @param[in]        conidx     Connection index
 ****************************************************************************************
 */
static void otac_create(struct prf_task_env *env, uint8_t conidx)
{
    struct otac_env_tag *otac_env = (struct otac_env_tag *)env->env;
    ASSERT_ERR(conidx < BLE_CONNECTION_MAX);

    // force notification config to zero when peer device is connected
    otac_env->ntf_cfg[conidx] = 0;
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
static void otac_cleanup(struct prf_task_env *env, uint8_t conidx, uint8_t reason)
{
    struct otac_env_tag *otac_env = (struct otac_env_tag *)env->env;
    ASSERT_ERR(conidx < BLE_CONNECTION_MAX);

    // force notification config to zero when peer device is connected
    otac_env->ntf_cfg[conidx] = 0;
}

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

/// OTAC Task interface required by profile manager
const struct prf_task_cbs otac_itf = {
    (prf_init_fnct)otac_init, otac_destroy, otac_create, otac_cleanup,
};

/*
 * EXPORTED FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */

const struct prf_task_cbs *otac_prf_itf_get(void)
{
    return &otac_itf;
}

void otac_start_encrypt_req(uint8_t key[16], uint8_t plain_data[16])
{
    struct otac_env_tag *otac_env = PRF_ENV_GET(OTAC, otac);

    struct gapm_use_enc_block_cmd *msg =
        KE_MSG_ALLOC(GAPM_USE_ENC_BLOCK_CMD, TASK_GAPM, prf_get_task_from_id(TASK_ID_OTAC), gapm_use_enc_block_cmd);
    // Ask to the AES to generate the SK by giving the SKD an LTK
    msg->operation = GAPM_USE_ENC_BLOCK;
    memcpy(msg->operand_1, key, 16);
    memcpy(msg->operand_2, plain_data, 16);
    // Send the command to start the SK generation
    ke_msg_send(msg);
}
void otac_encrypt_req(uint16_t conhdl, uint8_t key[16], uint8_t plain_data[16])
{
    otac_start_encrypt_req(key, plain_data);
}

void otac_transmit_rsp_to_client(uint16_t conhdl, uint8_t *pdata, size_t len)
{
    struct otac_env_tag *otac_env = PRF_ENV_GET(OTAC, otac);
    ke_task_id_t id = prf_src_task_get(&otac_env->prf_env, CONHDL2CONIDX(conhdl));

    struct otac_send_target_rsp_req *msg =
        KE_MSG_ALLOC_DYN(OTAC_SEND_TARGET_RSP_REQ, id, id, otac_send_target_rsp_req, len);

    /// Connection handle
    msg->conhdl = conhdl;
    msg->length = len;
    msg->offset = 0;

    memcpy(msg->data, pdata, len);

    // Send the message
    ke_msg_send(msg);
}
void otac_send_cmd(uint16_t len, uint8_t *data)
{
    otapCmdIdt_t cmdId;

    cmdId = (otapCmdIdt_t)data[0];
    switch (cmdId)
    {
        case gOtapCmdIdNewImageInfoRequest_c:
        case gOtapCmdIdImageBlockRequest_c:
        case gOtapCmdIdImageTransferComplete_c:
        case gOtapCmdIdStopImageTransfer_c:
        case gOtapCmdIdErrorNotification_c:
            otac_transmit_rsp_to_client(otaGetCurrentValidConhdl(), data, len);
            break;
    }
}

#endif /* BLE_OTAC */
/// @} OTAC

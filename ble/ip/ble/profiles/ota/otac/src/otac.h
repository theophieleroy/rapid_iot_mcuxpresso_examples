/**
 ****************************************************************************************
 *
 * @file otac.h
 *
 * @brief Header file - OTA Profile Client.
 *
 * Copyright(C) 2015 NXP Semiconductors N.V.
 * All rights reserved.
 *
 * $Rev: $
 *
 ****************************************************************************************
 */

#ifndef _OTAC_H_
#define _OTAC_H_

/**
 ****************************************************************************************
 * @addtogroup OTA profile Client
 * @ingroup_OTA
 * @brief OTA profile Client
 *
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "rwip_config.h"
#if (BLE_OTA_CLIENT)
#include "ota_common.h"
#include "otac_task.h"
#include "prf_types.h"
#include "prf.h"

/*
 * MACROS
 ****************************************************************************************
 */

/// Maximum number of OTA profile task instances
#define OTAC_IDX_MAX (0x01)
#define OTAC_MANDATORY_MASK (0x007f)
#define OTAC_VERSION "OTAC v2.00"
#define OTAC_HANDLE(idx) (otac_env->start_hdl + (idx))

/*
 * ENUMS
 ****************************************************************************************
 */
/// Possible states of the_OTAC task
enum
{
    /// Idle state
    OTAC_IDLE,
    /// Number of defined states.
    OTAC_STATE_MAX,
};

/// Attributes State Machine
enum
{
    OTAC_IDX_SVC = 0,

    OTAC_IDX_TARGET_RSP_CHAR,
    OTAC_IDX_TARGET_RSP_VAL,
    OTAC_IDX_TARGET_RSP_NTF_CFG,
    OTAC_IDX_TARGET_RSP_USER_DESP,

    OTAC_IDX_HOST_CMD_CHAR,
    OTAC_IDX_HOST_CMD_VAL,

    OTAC_IDX_NB,
};

/// OTA profile environment variable
struct otac_env_tag
{
    /// profile environment
    prf_env_t prf_env;
    /// Service Start Handle
    uint16_t start_hdl;
    struct otac_send_target_rsp_req *target_rsp;
    /// State of different task instances
    ke_state_t state[OTAC_IDX_MAX];
    /// Notification configuration of peer devices.
    uint8_t ntf_cfg[BLE_CONNECTION_MAX];
};
/// Parameters for the database creation
struct otac_db_cfg
{
    /// Database configuration
    uint16_t dummy;
};

/*
 * MACROS
 ****************************************************************************************
 */

/*
 * GLOBAL VARIABLE DECLARATIONS
 ****************************************************************************************
 */
extern struct otac_env_tag otac_env;

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */
void otaClientRegisterConhdl(uint16_t conhdl);
/**
 ****************************************************************************************
 * @brief Retrieve_OTA client profile interface
 *
 * @return OTA client profile interface
 ****************************************************************************************
 */
const struct prf_task_cbs *otac_prf_itf_get(void);

/**
 ****************************************************************************************
 * @brief Get max MTU size
 *
 * @return Max MTU size
 ****************************************************************************************
 */
uint16_t otac_get_max_mtu(void);

/**
 ****************************************************************************************
 * @brief Ota client process data received from ota server.
 *
 * @param[in] length length of data
 * @param[in] pData pointer of data
 * @return  ota client process status
 ****************************************************************************************
 */
int otac_process_received_data(uint16_t length, uint8_t *pData);

/**
 ****************************************************************************************
 * @brief Do some perations after ota process done.
 ****************************************************************************************
 */
void otac_download_Complete(void);

/*
 * TASK DESCRIPTOR DECLARATIONS
 ****************************************************************************************
 */
extern const struct ke_state_handler otac_default_handler;

#endif /* #if (BLE_OTA_CLIENT) */

/// @}_OTAC

#endif /* _OTAC_H_ */

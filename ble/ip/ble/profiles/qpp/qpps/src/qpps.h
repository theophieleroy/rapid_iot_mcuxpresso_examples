/**
 ****************************************************************************************
 *
 * @file qpps.h
 *
 * @brief Header file - QBlue Private Profile Server.
 *
 * Copyright(C) 2015 NXP Semiconductors N.V.
 * All rights reserved.
 *
 *
 ****************************************************************************************
 */

#ifndef _QPPS_H_
#define _QPPS_H_

/**
 ****************************************************************************************
 * @addtogroup QBlue private profile Server
 * @ingroup QPP
 * @brief QBlue private profile Server
 *
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "rwip_config.h"
#if (BLE_QPP_SERVER)
#include "qpp_common.h"
#include "qpps_task.h"
#include "prf_types.h"
#include "prf.h"

/*
 * MACROS
 ****************************************************************************************
 */

/// Maximum number of QBlue private profile task instances
#define QPPS_IDX_MAX (0x01)
#define QPPS_MANDATORY_MASK (0x007f)
#define QPPS_VERSION "2.0"

#define QPPS_HANDLE(idx) (qpps_env->start_hdl + (idx))

/*
 * ENUMS
 ****************************************************************************************
 */
/// Possible states of the QPPS task
enum
{
    /// Idle state
    QPPS_IDLE,
    /// Number of defined states.
    QPPS_STATE_MAX,
};

/// Attributes State Machine
enum
{
    QPPS_IDX_SVC,

    QPPS_IDX_RX_DATA_CHAR,
    QPPS_IDX_RX_DATA_VAL,
    QPPS_IDX_RX_DATA_USER_DESP,

    QPPS_IDX_TX_DATA_CHAR,
    QPPS_IDX_TX_DATA_VAL,
    QPPS_IDX_TX_DATA_NTF_CFG,

    QPPS_IDX_NB,
};

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

/// QBlue private profile environment variable
struct qpps_env_tag
{
    /// profile environment
    prf_env_t prf_env;
    /// Service Start Handle
    uint16_t start_hdl;
    /// State of different task instances
    ke_state_t state[QPPS_IDX_MAX];
    /// Notification configuration of peer devices.
    uint8_t ntf_cfg[__ARRAY_EMPTY];
};
/// Parameters for the database creation
struct qpps_db_cfg
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
#if defined(CFG_QPP_SHOW_THROUGHPUT)
extern struct qpp_throughput_stats qpps_stats[BLE_CONNECTION_MAX];
#endif

extern struct qpps_env_tag qpps_env;

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */
/**
 ****************************************************************************************
 * @brief Retrieve QPP service profile interface
 *
 * @return QPP service profile interface
 ****************************************************************************************
 */
const struct prf_task_cbs *qpps_prf_itf_get(void);

/*
 * TASK DESCRIPTOR DECLARATIONS
 ****************************************************************************************
 */
extern const struct ke_state_handler qpps_default_handler;

#endif /* #if (BLE_QPP_SERVER) */

/// @} QPPS

#endif /* _QPPS_H_ */

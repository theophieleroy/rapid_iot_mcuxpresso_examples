/**
 ****************************************************************************************
 *
 * @file qppc.h
 *
 * @brief Header file - QBlue Private Profile Client Role.
 *
 * Copyright(C) 2015 NXP Semiconductors N.V.
 * All rights reserved.
 *
 *
 ****************************************************************************************
 */

#ifndef _QPPC_H_
#define _QPPC_H_

/**
 ****************************************************************************************
 * @addtogroup QPPC QBlue Private Profile Client
 * @ingroup QPP
 * @brief QBlue Private Profile Client
 *
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#if (BLE_QPP_CLIENT)
#include "qpp_common.h"
#include "prf_types.h"
#include "prf.h"

/*
 * DEFINES
 ****************************************************************************************
 */
/// Maximum number of QPPC task instances
#define QPPC_IDX_MAX (BLE_CONNECTION_MAX)

/// Possible states of the QPPC task
enum
{
    /// Not Connected state
    QPPC_FREE,
    /// IDLE state
    QPPC_IDLE,
    /// Discovering QPPS SVC and Chars
    QPPC_DISCOVERING,
    // Number of defined states.
    QPPC_STATE_MAX
};

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */
/// Environment variable for each Connections
struct qppc_cnx_env
{
    /// QPP Service content
    struct qpps_content qpps;
    /// Last char. code requested to read.
    uint8_t last_char_code;
    /// counter used to check service uniqueness
    uint8_t nb_svc;
};

/// QPP Client environment variable
struct qppc_env_tag
{
    /// profile environment
    prf_env_t prf_env;
    /// Environment variable pointer for each connections
    struct qppc_cnx_env **env;
    /// State of different task instances
    ke_state_t state[__ARRAY_EMPTY];
};

/*
 * MACROS
 ****************************************************************************************
 */

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */
/**
 ****************************************************************************************
 * @brief Retrieve QPP client profile interface
 *
 * @return QPP client profile interface
 ****************************************************************************************
 */
const struct prf_task_cbs *qppc_prf_itf_get(void);

/**
 ****************************************************************************************
 * @brief Send ATT DB discovery results to QPPC host.
 ****************************************************************************************
 */
void qppc_enable_rsp_send(struct qppc_env_tag *qppc_env, uint8_t conidx, uint8_t status);

/*
 * TASK DESCRIPTOR DECLARATIONS
 ****************************************************************************************
 */
extern const struct ke_state_handler qppc_default_handler;

#endif /* (BLE_QPP_CLIENT) */

/// @} QPPC

#endif /* _QPPC_H_ */

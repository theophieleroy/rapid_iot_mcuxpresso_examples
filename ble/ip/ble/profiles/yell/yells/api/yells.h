/**
 ****************************************************************************************
 *
 * @file yells.h
 *
 * @brief Header file - QBlue Private Profile Server.
 *
 * Copyright(C) 2015 NXP Semiconductors N.V.
 * All rights reserved.
 *
 * $Rev: $
 *
 ****************************************************************************************
 */

#ifndef _YELLS_H_
#define _YELLS_H_

/**
 ****************************************************************************************
 * @addtogroup QBlue private profile Server
 * @ingroup YELL
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
#if (BLE_YELL_SERVER)
#include "yell_common.h"
#include "yells_task.h"
#include "prf_types.h"
#include "prf.h"
#include "attm.h"
#include "atts.h"
#include "attm_db.h"

/*
 * DEFINES
 ****************************************************************************************
 */

#define YELLS_MANDATORY_MASK (0x01ff)
#define YELLS_VERSION "1.0"

/*
 * MACROS
 ****************************************************************************************
 */
#define YELLS_HANDLE(idx) (yells_env->start_hdl + (idx))
/// Attributes State Machine
enum
{
    YELLS_IDX_SVC,

    YELLS_IDX_RX_DATA_CHAR,
    YELLS_IDX_RX_DATA_VAL,
    YELLS_IDX_RX_DATA_USER_DESP,

    YELLS_IDX_TX_VAL_CHAR,
    YELLS_IDX_TX_VAL,
    YELLS_IDX_TX_VAL_NTF_CFG,

    YELLS_IDX_ENUM_CHAR,
    YELLS_IDX_ENUM_VAL,

    YELLS_IDX_NB,
};

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

/// QBlue private profile environment variable
struct yells_env_tag
{
    /// profile environment
    prf_env_t prf_env;
    /// Service Start Handle
    uint16_t start_hdl;
    /// State of different task instances
    ke_state_t state[YELLS_IDX_MAX];
    /// Notification configuration of peer devices.
    uint8_t ntf_cfg[__ARRAY_EMPTY];
};

/*
 * MACROS
 ****************************************************************************************
 */

/*
 * GLOBAL VARIABLE DECLARATIONS
 ****************************************************************************************
 */

extern struct yells_env_tag yells_env;

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */
/**
 ****************************************************************************************
 * @brief Retrieve GLP service profile interface
 *
 * @return GLP service profile interface
 ****************************************************************************************
 */
const struct prf_task_cbs *yells_prf_itf_get(void);

#endif /* #if (BLE_YELL_SERVER) */

/// @} YELLS

#endif /* _YELLS_H_ */

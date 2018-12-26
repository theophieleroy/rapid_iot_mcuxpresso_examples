/**
 ****************************************************************************************
 *
 * @file yells_task.h
 *
 * @brief Header file - QBlue Private Profile Server Task.
 *
 * Copyright(C) 2015 NXP Semiconductors N.V.
 * All rights reserved.
 *
 * $Rev: $
 *
 ****************************************************************************************
 */

#ifndef _YELLS_TASK_H_
#define _YELLS_TASK_H_

/// @cond

/**
 ****************************************************************************************
 * @addtogroup YELLS Task
 * @ingroup YELLS
 * @brief QBlue private profile Task.
 *
 * The YELLS TASK is responsible for handling the messages coming in and out of the
 * @ref YELLS collector block of the BLE Host.
 *
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#if BLE_YELL_SERVER
#include <stdint.h>
#include "rwip_task.h"
#include "yell_common.h"

/*
 * DEFINES
 ****************************************************************************************
 */

/// Maximum number of QBlue private profile task instances
#define YELLS_IDX_MAX (0x01)

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

/// Possible states of the YELLS task
enum
{
    /// Idle state
    YELLS_IDLE,
    /// Number of defined states.
    YELLS_STATE_MAX,
};

/// Messages for QBlue private Profile Server
enum
{
    /// Send value from APP
    YELLS_SEND_DATA_REQ = TASK_FIRST_MSG(TASK_ID_YELLS),
    /// Send data response to APP if correctly sent.
    YELLS_SEND_DATA_RSP,
    /// Inform APP of new configuration value
    YELLS_CFG_INDNTF_IND,
    /// Get data from Client
    YELLS_GET_DATA_IND,

    YELLS_READ_REQ_IND,

    YELLS_NTF_TIMER
};

/////Parameters of the @ref YELLS_SEND_DATA_REQ message
struct yells_send_data_req
{
    /// Connection handle
    uint16_t conhdl;
    /// Length
    uint8_t length;
    /// Data
    uint8_t data[__ARRAY_EMPTY];
};

/// @endcond

/**
 ****************************************************************************************
 * @addtogroup APP_YELLS_TASK
 * @{
 ****************************************************************************************
 */
/// Parameters for the database creation
struct yells_db_cfg
{
    /// Database configuration @see enum yells_features
    uint16_t features;
};

/// Parameters of the @ref YELLS_DISABLE_IND message
struct yells_disable_ind
{
    uint16_t conhdl;
    /// Notification configuration
    uint16_t ntf_en;
};

/// Parameters of the @ref YELLS_ERROR_IND message
struct yells_error_ind
{
    /// Connection handle
    uint16_t conhdl;
    /// Status
    uint8_t status;
};

/// Parameters of the @ref YELLS_CFG_INDNTF_IND message
struct yells_cfg_indntf_ind
{
    /// Connection handle
    uint16_t conhdl;
    /// Stop/notify/indicate value to configure into the peer characteristic
    uint16_t cfg_val;
};

/// Parameters of the @ref YELLS_SEND_DATA_RSP message
struct yells_send_data_rsp
{
    /// Connection handle
    uint16_t conhdl;
    /// Status
    uint8_t status;
};

/// Parameters of the @ref YELLS_GET_DATA_IND message
struct yells_get_data_ind
{
    /// Connection handle
    uint16_t conhdl;
    uint8_t length;
    uint8_t data[1];
};

/// Parameters of the @ref YELLS_READ_DATA_IND message
struct yells_read_req_ind
{
    /// Connection handle
    uint16_t conhdl;
    uint8_t index;
};
/// @} APP_YELLS_TASK

/// @cond

/*
 * MACROS
 ****************************************************************************************
 */

/*
 * GLOBAL VARIABLE DECLARATIONS
 ****************************************************************************************
 */

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

typedef int (*yells_user_read_callback_t)(void **);
typedef void (*yells_user_read_done_callback_t)();

/*
 * TASK DESCRIPTOR DECLARATIONS
 ****************************************************************************************
 */
extern const struct ke_state_handler yells_default_handler;

#endif /* BLE_YELL_SERVER */

/// @} YELLSTASK
/// @endcond
#endif /* _YELLS_TASK_H_ */

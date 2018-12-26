/**
 ****************************************************************************************
 *
 * @file otac_task.h
 *
 * @brief Header file - OTA Profile Client Task.
 *
 * Copyright(C) 2015 NXP Semiconductors N.V.
 * All rights reserved.
 *
 * $Rev: $
 *
 ****************************************************************************************
 */

#ifndef _OTAC_TASK_H_
#define _OTAC_TASK_H_

/// @cond

/**
 ****************************************************************************************
 * @addtogroup OTAC Task
 * @ingroup OTAC
 * @brief OTA profile Client Task.
 *
 * The OTAC TASK is responsible for handling the messages coming in and out of the
 * @ref OTAC collector block of the BLE Host.
 *
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "rwip_task.h"

/*
 * DEFINES
 ****************************************************************************************
 */

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

/// Messages for OTA Profile Client
enum
{
    /// Send data from APP
    OTAC_SEND_TARGET_RSP_REQ = TASK_FIRST_MSG(TASK_ID_OTAC),
    /// Send data response to APP if correctly sent.
    OTAC_SEND_DATA_RSP,
    /// Inform APP of new configuration value
    OTAC_CFG_INDNTF_IND,
    /// Receive data from Client
    OTAC_RECV_DATA_IND,
    OTAC_PROCESS_REQ,
    OTAC_TIMEOUT_IND,
};

/////Parameters of the @ref OTAC_SEND_TARGET_RSP_REQ message
struct otac_send_target_rsp_req
{
    /// Connection handle
    uint16_t conhdl;
    /// Length
    uint16_t length;
    uint16_t offset;
    /// Data
    uint8_t data[__ARRAY_EMPTY];
};

/// @endcond

/**
 ****************************************************************************************
 * @addtogroup APP_OTAC_TASK
 * @{
 ****************************************************************************************
 */
/// Parameters of the @ref OTAC_CFG_INDNTF_IND message
struct otac_cfg_indntf_ind
{
    /// Connection handle
    uint16_t conhdl;
    /// Stop/notify/indicate value to configure into the peer characteristic
    uint16_t cfg_val;
};

/// Parameters of the @ref OTAC_SEND_DATA_RSP message
struct otac_send_data_rsp
{
    /// Connection handle
    uint16_t conhdl;
    /// Status
    uint8_t status;
};

/// Parameters of the @ref OTAC_RECV_DATA_IND message
struct otac_recv_data_ind
{
    /// Connection handle
    uint16_t conhdl;
    uint16_t length;
    uint8_t data[__ARRAY_EMPTY];
};
///
struct otac_process_req
{
    /// Data length
    uint16_t length;
    /// Event Value
    uint8_t value[__ARRAY_EMPTY];
};

/// @} APP_OTAC_TASK

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/// @} OTACTASK
/// @endcond
#endif /* _OTAC_TASK_H_ */

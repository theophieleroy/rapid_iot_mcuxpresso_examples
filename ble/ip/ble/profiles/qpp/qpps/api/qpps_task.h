/**
 ****************************************************************************************
 *
 * @file qpps_task.h
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

#ifndef _QPPS_TASK_H_
#define _QPPS_TASK_H_

/// @cond

/**
 ****************************************************************************************
 * @addtogroup QPPS Task
 * @ingroup QPPS
 * @brief QBlue private profile Task.
 *
 * The QPPS TASK is responsible for handling the messages coming in and out of the
 * @ref QPPS collector block of the BLE Host.
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

/// Messages for QBlue private Profile Server
enum
{
    ///Send data from APP
    QPPS_SEND_DATA_REQ = TASK_FIRST_MSG(TASK_ID_QPPS),
    ///Send data response to APP if correctly sent.
    QPPS_SEND_DATA_RSP,
    ///Inform APP of new configuration value
    QPPS_CFG_INDNTF_IND,
    ///Receive data from Client
    QPPS_RECV_DATA_IND,
#if defined(CFG_QPP_SHOW_THROUGHPUT)
    ///QPPS throughput statistics timer
    QPPS_THROUGHPUT_STATISTICS_TIMER,
#endif
};

/////Parameters of the @ref QPPS_SEND_DATA_REQ message
struct qpps_send_data_req
{
    /// Connection handle
    uint16_t conhdl;
    /// Length
    uint16_t length;
    /// Data
    uint8_t data[__ARRAY_EMPTY];
};

/// @endcond

/**
 ****************************************************************************************
 * @addtogroup APP_QPPS_TASK
 * @{
 ****************************************************************************************
 */
///Parameters of the @ref QPPS_CFG_INDNTF_IND message
struct qpps_cfg_indntf_ind
{
    ///Connection handle
    uint16_t conhdl;
    ///Stop/notify/indicate value to configure into the peer characteristic
    uint16_t cfg_val;
};

///Parameters of the @ref QPPS_SEND_DATA_RSP message
struct qpps_send_data_rsp
{
    ///Connection handle
    uint16_t conhdl;
    ///Status
    uint8_t status;
};

///Parameters of the @ref QPPS_RECV_DATA_IND message
struct qpps_recv_data_ind
{
    ///Connection handle
    uint16_t conhdl;
    uint16_t length;
    uint8_t data[__ARRAY_EMPTY];
};
/// @} APP_QPPS_TASK

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/// @} QPPSTASK
/// @endcond
#endif /* _QPPS_TASK_H_ */

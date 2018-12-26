/**
 ****************************************************************************************
 *
 * @file qppc_task.h
 *
 * @brief Header file - QBlue Private Profile Client Task.
 *
 * Copyright(C) 2015 NXP Semiconductors N.V.
 * All rights reserved.
 *
 * $Rev: $
 *
 ****************************************************************************************
 */

#ifndef _QPPC_TASK_H_
#define _QPPC_TASK_H_

/// @cond

/**
 ****************************************************************************************
 * @addtogroup QPPC TASK QBlue Private Profile Client Task
 * @ingroup QPPC
 * @brief QBlue Private Profile Client Task
 *
 * The QPPC TASK is responsible for handling the messages coming in and out of the
 * @ref QPPC client block of the BLE Host.
 *
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "rwip_task.h"
#include "prf_types.h"

/*
 * DEFINES
 ****************************************************************************************
 */

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */
/// Characteristics
enum
{
    /// QBlue Private Service RX Value
    QPPC_QPPS_RX_CHAR_VALUE,
    /// QBlue Private Service TX Value
    QPPC_QPPS_TX_CHAR_VALUE,

    QPPC_CHAR_MAX,
};

/// Characteristic descriptors
enum
{
    /// QPPS RX Value User Desp
    QPPC_QPPS_RX_CHAR_VALUE_USER_DESP,
    /// Output Value Client config
    QPPC_QPPS_TX_VALUE_CLI_CFG,

    QPPC_DESC_MAX,
};

enum
{
    /// Start the QBlue Private Profile - at connection
    QPPC_ENABLE_REQ = TASK_FIRST_MSG(TASK_ID_QPPC),
    ///Confirm that cfg connection has finished with discovery results, or that normal cnx started
    QPPC_ENABLE_RSP,
    /// Generic message to read a QPPS characteristic value
    QPPC_RD_CHAR_REQ,
    ///Generic message for read responses for APP
    QPPC_RD_CHAR_RSP,
    ///Generic message for configuring the characteristics that can be handled
    QPPC_CFG_INDNTF_REQ,
    ///Generic message for write characteristic response status to APP
    QPPC_CFG_INDNTF_RSP,
    ///APP send data request to server
    QPPC_SEND_DATA_REQ,
    ///Generic message for write characteristic response status to APP
    QPPC_SEND_DATA_RSP,
    /// value send to APP
    QPPC_RECV_DATA_IND,
};
///Structure containing the characteristics handles, value handles and descriptors
struct qpps_content
{
    /// service info
    struct prf_svc svc;

    /// characteristic info:
    struct prf_char_inf chars[QPPC_CHAR_MAX];

    /// Descriptor handles:
    struct prf_char_desc_inf descs[QPPC_DESC_MAX];
};

/// Parameters of the @ref QPPC_ENABLE_REQ message
struct qppc_enable_req
{
    /// Connection handle
    uint16_t conhdl;
    ///Connection type
    uint8_t con_type;
    ///Existing handle values qpps
    struct qpps_content qpps;
};

///Parameters of the @ref QPPC_RD_CHAR_REQ message
struct qppc_rd_char_req
{
    ///Connection handle
    uint16_t conhdl;
    ///Characteristic value code
    uint8_t  char_code;
};

///Parameters of the @ref QPPC_CFG_INDNTF_REQ message
struct qppc_cfg_indntf_req
{
    ///Connection handle
    uint16_t conhdl;
    ///Stop/Start notify value to configure into the peer characteristic
    uint16_t cfg_val;
    /// Last char. code requested to config
    uint8_t char_code;
};

///Parameters of the @ref QPPC_SEND_DATA_REQ message
struct qppc_send_data_req
{
    ///Connection handle
    uint16_t conhdl;
    /// Length
    uint16_t length;
    /// Data
    uint8_t data[__ARRAY_EMPTY];
};


/**
 * Structure containing the characteristics handles, value handles and descriptors for
 * the QPP Service
 */
struct qppc_dis_content
{
    /// service info
    struct prf_svc svc;

    /// Characteristic info:
    struct prf_char_inf chars[QPPC_CHAR_MAX];
};


/// @endcond

/**
 ****************************************************************************************
 * @addtogroup APP_QPPC_TASK
 * @{
 ****************************************************************************************
 */

/// Parameters of the @ref QPPC_ENABLE_RSP message
struct qppc_enable_rsp
{
    /// Connection handle
    uint16_t conhdl;
    ///status
    uint8_t status;
    ///Existing handle values qpps
    struct qpps_content qpps;
};

///Parameters of the @ref QPPC_RD_CHAR_RSP message
struct qppc_rd_char_rsp
{
    ///Connection handle
    uint16_t conhdl;
    ///Status
    uint8_t  status;
    ///Data length
    uint8_t length;
    ///Holder of retrieved data
    uint8_t data[__ARRAY_EMPTY];
};
///Parameters of the @ref QPPC_CFG_INDNTF_RSP message
struct qppc_cfg_indntf_rsp
{
    ///Connection handle
    uint16_t conhdl;
    ///Status
    uint8_t  status;
};

///Parameters of the @ref QPPC_SEND_DATA_RSP message
struct qppc_send_data_rsp
{
    ///Connection handle
    uint16_t conhdl;
    ///Status
    uint8_t  status;
};

///Parameters of the @ref QPPC_RECV_DATA_IND message
struct qppc_recv_data_ind
{
    ///Connection handle
    uint16_t conhdl;
    /// Length
    uint16_t length;
    /// Data
    uint8_t data[__ARRAY_EMPTY];
};

/// @} APP_QPPC_TASK

/// @} QPPCTASK
/// @endcond
#endif /* _QPPC_TASK_H_ */

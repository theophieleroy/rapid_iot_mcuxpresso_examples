/**
 ****************************************************************************************
 *
 * @file otas_task.h
 *
 * @brief Header file - OTA Server Task.
 *
 * Copyright(C) 2015 NXP Semiconductors N.V.
 * All rights reserved.
 *
 * $Rev: $
 *
 ****************************************************************************************
 */

#ifndef _OTAS_TASK_H_
#define _OTAS_TASK_H_

/// @cond

/**
 ****************************************************************************************
 * @addtogroup OTA Profile Server Task
 * @ingroup OTAS
 * @brief OTA Profile Server Task
 *
 * The OTAS TASK is responsible for handling the messages coming in and out of the
 * @ref OTAS server block of the BLE Host.
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
    /// OTA Client Response Value
    OTAS_OTAC_TARGET_RSP_VALUE,
    /// OTA Server Command Value
    OTAS_OTAC_HOST_CMD_CHAR,

    OTAS_CHAR_MAX,
};

/// Characteristic descriptors
enum
{
    /// OTA Client Response Desp
    OTAS_OTAC_TARGET_RSP_NTF_CFG,
    /// OTA Client config
    OTAS_OTAC_TARGET_RSP_USER_DESP,

    OTAS_DESC_MAX,
};

enum
{
    /// Start the OTA Profile - at connection
    OTAS_ENABLE_REQ = TASK_FIRST_MSG(TASK_ID_OTAS),
    /// Confirm that cfg connection has finished with discovery results, or that normal cnx started
    OTAS_ENABLE_RSP,
    /// Generic message to read a OTA characteristic value
    OTAS_RD_CHAR_REQ,
    /// Generic message for read responses for APP
    OTAS_RD_CHAR_RSP,
    /// Generic message for configuring the characteristics that can be handled
    OTAS_CFG_INDNTF_REQ,
    /// Generic message for write characteristic response status to APP
    OTAS_CFG_INDNTF_RSP,
    /// APP send data request to server
    OTAS_HOST_CMD_REQ,
    /// Generic message for write characteristic response status to APP
    OTAS_SEND_DATA_RSP,
    /// value send to APP
    OTAS_RECV_DATA_IND,
    /// OTA process data request
    OTAS_PROCESS_REQ,
    /// OTA time out handler
    OTAS_TIMEOUT_IND,
};
/// Structure containing the characteristics handles, value handles and descriptors
struct otas_content
{
    /// service info
    struct prf_svc svc;

    /// characteristic info:
    struct prf_char_inf chars[OTAS_CHAR_MAX];

    /// Descriptor handles:
    struct prf_char_desc_inf descs[OTAS_DESC_MAX];
};

/// Parameters of the @ref OTAS_ENABLE_REQ message
struct otas_enable_req
{
    /// Connection handle
    uint16_t conhdl;
    /// Connection type
    uint8_t con_type;
    /// Existing handle values otas
    struct otas_content otas;
};

/// Parameters of the @ref OTAS_RD_CHAR_REQ message
struct otas_rd_char_req
{
    /// Connection handle
    uint16_t conhdl;
    /// Characteristic value code
    uint8_t char_code;
};

/// Parameters of the @ref OTAS_CFG_INDNTF_REQ message
struct otas_cfg_indntf_req
{
    /// Connection handle
    uint16_t conhdl;
    /// Stop/Start notify value to configure into the peer characteristic
    uint16_t cfg_val;
    /// Last char. code requested to config
    uint8_t char_code;
};

/// Parameters of the @ref OTAS_HOST_CMD_REQ message
struct otas_host_cmd_req
{
    /// Connection handle
    uint16_t conhdl;
    /// Length
    uint16_t length;
    uint16_t cusor;
    uint16_t offset;
    uint8_t charCode;
    /// Data
    uint8_t data[__ARRAY_EMPTY];
};

/// Parameters of the @OTAS_PROCESS_REQ message
struct otas_process_req
{
    /// Data length
    uint16_t length;
    /// Event Value
    uint8_t value[__ARRAY_EMPTY];
};

/**
 * Structure containing the characteristics handles, value handles and descriptors for
 * the OTA Service
 */
struct otas_dis_content
{
    /// service info
    struct prf_svc svc;

    /// Characteristic info:
    struct prf_char_inf chars[OTAS_CHAR_MAX];
};

/// @endcond

/**
 ****************************************************************************************
 * @addtogroup APP_OTAS_TASK
 * @{
 ****************************************************************************************
 */

/// Parameters of the @ref OTAS_ENABLE_RSP message
struct otas_enable_rsp
{
    /// Connection handle
    uint16_t conhdl;
    /// status
    uint8_t status;
    /// Existing handle values otas
    struct otas_content otas;
};

/// Parameters of the @ref OTAS_RD_CHAR_RSP message
struct otas_rd_char_rsp
{
    /// Connection handle
    uint16_t conhdl;
    /// Status
    uint8_t status;
    /// Data length
    uint8_t length;
    /// Holder of retrieved data
    uint8_t data[__ARRAY_EMPTY];
};
/// Parameters of the @ref OTAS_CFG_INDNTF_RSP message
struct otas_cfg_indntf_rsp
{
    /// Connection handle
    uint16_t conhdl;
    /// Status
    uint8_t status;
};

/// Parameters of the @ref OTAS_SEND_DATA_RSP message
struct otas_send_data_rsp
{
    /// Connection handle
    uint16_t conhdl;
    /// Status
    uint8_t status;
};

/// Parameters of the @ref OTAS_RECV_DATA_IND message
struct otas_recv_data_ind
{
    /// Connection handle
    uint16_t conhdl;
    /// Length
    uint16_t length;
    /// Data
    uint8_t data[__ARRAY_EMPTY];
};

/// @} APP_OTAS_TASK

/// @} OTASTASK
/// @endcond
#endif /* _OTAS_TASK_H_ */

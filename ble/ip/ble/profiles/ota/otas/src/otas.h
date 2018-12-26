/**
 ****************************************************************************************
 *
 * @file OTAS.h
 *
 * @brief Header file - OTA Server Role.
 *
 * Copyright(C) 2015 NXP Semiconductors N.V.
 * All rights reserved.
 *
 * $Rev: $
 *
 ****************************************************************************************
 */

#ifndef _OTAS_H_
#define _OTAS_H_

/**
 ****************************************************************************************
 * @addtogroup OTAS Server
 * @ingroup OTA
 * @brief OTA Profile Server
 *
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#if (BLE_OTA_SERVER)
#include "ota_common.h"
#include "prf_types.h"
#include "prf.h"

/*
 * DEFINES
 ****************************************************************************************
 */
/// Maximum number of OTAS task instances
#define OTAS_IDX_MAX (BLE_CONNECTION_MAX)
#define OTAS_FSM_TIMEOUT (3000)
/// Possible states of the OTAS task
enum
{
    /// Not Connected state
    OTAS_FREE,
    /// IDLE state
    OTAS_IDLE,
    /// Discovering OTAS SVC and Chars
    OTAS_DISCOVERING,
    // Number of defined states.
    OTAS_STATE_MAX
};

enum otas_state
{
    OTAS_STATE_INIT,
    OTAS_UPGRADE_RSP_WAIT,
    OTAS_DATA_RSP_WAIT,
    OTA_VERIFI_WAIT,
};

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */
/// Environment variable for each Connections
struct otas_cnx_env
{
    /// OTA Service content
    struct otas_content otas;
    /// Last char. code requested to read.
    uint8_t last_char_code;
    /// counter used to check service uniqueness
    uint8_t nb_svc;
    uint8_t state;
    uint8_t encrypt_enable;
    uint8_t encrypt_pending;
    uint8_t encrypt_offset;
    uint16_t mtu;
    uint16_t length;
    struct otas_host_cmd_req *host_cmd;
    uint8_t *data;
};

struct otas_encrypt_tag
{
    uint8_t key[16];
    uint8_t ctr[16];
};

/// OTA Server environment variable
struct otas_env_tag
{
    /// profile environment
    prf_env_t prf_env;
    /// Environment variable pointer for each connections
    struct otas_cnx_env **env;
    struct otas_encrypt_tag encrypt;
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
 * @brief Retrieve OTA server profile interface
 *
 * @return OTA server profile interface
 ****************************************************************************************
 */
const struct prf_task_cbs *otas_prf_itf_get(void);

/**
 ****************************************************************************************
 * @brief Send ATT DB discovery results to OTAS host.
 *
 * @param[in] otas_env  Otas task enviroment.
 * @param[in] conidx    Connection inedx.
 * @param[in] status    Code of the response status.
 ****************************************************************************************
 */
void otas_enable_rsp_send(struct otas_env_tag *otas_env, uint8_t conidx, uint8_t status);

/**
 ****************************************************************************************
 * @brief OTA server register connection handle, OTA image will be send on this
 *        connection handle.
 *
 * @param[in] conhdl    Connection handle.
 ****************************************************************************************
 */
void otaServerRegisterConhdl(uint16_t conhdl);

/**
 ****************************************************************************************
 * @brief OTA server send command request function.
 *
 * @param[in] conhdl    Connection handle.
 * @param[in] pData     Command data to be send.
 * @param[in] len       Data length.
 * @param[in] charCode  Command code.
 ****************************************************************************************
 */
void otas_host_cmd_req_send(uint16_t conhdl, uint8_t *pdata, size_t len, uint8_t charCode);
/*
 * TASK DESCRIPTOR DECLARATIONS
 ****************************************************************************************
 */
extern const struct ke_state_handler otas_default_handler;

#endif /* (BLE_OTA_SERVER) */

/// @} OTAS

#endif /* _OTAS_H_ */

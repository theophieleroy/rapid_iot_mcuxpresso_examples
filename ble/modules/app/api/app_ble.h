/*
 * The Clear BSD License
 * Copyright (c) 2016, NXP Semiconductors, N.V.
 * All rights reserved.
 *
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided
 *  that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of Freescale Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS LICENSE.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef _APP_BLE_H_
#define _APP_BLE_H_

/*!
 * @addtogroup APP_BLE_API
 * @{
 */

#include <stdint.h>
#include <string.h>
#include <stdlib.h>

#include "app_config.h"

#include "app_gap.h"
#include "app_gatt.h"
#include "app_ble_task.h" /* Application BLE task Definition */
#include "reg_blecore.h"
#include "qnble.h"

#if PLF_NVDS
#include "nvds.h"
#endif

#if (BLE_APP_PRESENT)

#if defined(CFG_DEMO_MENU)
#include "app_menu.h"
#endif

#if BLE_BATT_CLIENT
#include "app_basc.h"
#endif
#if BLE_BATT_SERVER
#include "app_bass.h"
#endif
#if BLE_DIS_CLIENT
#include "app_disc.h"
#endif
#if BLE_DIS_SERVER
#include "app_diss.h"
#endif
#if BLE_GL_SENSOR
#include "app_glps.h"
#endif
#if (BLE_GL_COLLECTOR)
#include "app_glpc.h"
#endif
#if BLE_PROX_MONITOR
#include "app_proxm.h"
#endif
#if BLE_PROX_REPORTER
#include "app_proxr.h"
#endif
#if BLE_QPP_SERVER
#include "app_qpps.h"
#endif
#if BLE_QPP_CLIENT
#include "app_qppc.h"
#endif
#if (BLE_FINDME_LOCATOR)
#include "app_findl.h"
#endif
#if BLE_FINDME_TARGET
#include "app_findt.h"
#endif
#if BLE_HT_COLLECTOR
#include "app_htpc.h"
#endif
#if BLE_HT_THERMOM
#include "app_htpt.h"
#endif
#if BLE_YELL_SERVER
#include "app_yells.h"
#endif
#if BLE_BP_SENSOR
#include "app_blps.h"
#endif
#if BLE_CSC_SENSOR
#include "app_cscps.h"
#endif
#if BLE_CP_SENSOR
#include "app_cpps.h"
#endif
#if (BLE_CP_COLLECTOR)
#include "app_cppc.h"
#endif
#if (BLE_CSC_COLLECTOR)
#include "app_cscpc.h"
#endif
#if (BLE_HR_SENSOR)
#include "app_hrps.h"
#endif
#if (BLE_HR_COLLECTOR)
#include "app_hrpc.h"
#endif
#if (BLE_LN_SENSOR)
#include "app_lans.h"
#endif
#if (BLE_LN_COLLECTOR)
#include "app_lanc.h"
#endif
#if (BLE_SP_SERVER)
#include "app_scpps.h"
#endif
#if (BLE_SP_CLIENT)
#include "app_scppc.h"
#endif
#if (BLE_RSC_SENSOR)
#include "app_rscps.h"
#endif
#if (BLE_PAS_SERVER)
#include "app_pasps.h"
#endif
#if (BLE_PAS_CLIENT)
#include "app_paspc.h"
#endif
#if (BLE_HID_DEVICE)
#include "app_hogpd.h"
#endif
#if (BLE_HID_BOOT_HOST)
#include "app_hogpbh.h"
#endif
#if (BLE_HID_REPORT_HOST)
#include "app_hogprh.h"
#endif
#if (BLE_AN_SERVER)
#include "app_anps.h"
#endif
#if (BLE_AN_CLIENT)
#include "app_anpc.h"
#endif
#if (BLE_TIP_SERVER)
#include "app_tips.h"
#endif
#if (BLE_TIP_CLIENT)
#include "app_tipc.h"
#endif
#if (BLE_BP_COLLECTOR)
#include "app_blpc.h"
#endif
#if (BLE_RSC_COLLECTOR)
#include "app_rscpc.h"
#endif
#if BLE_OTA_CLIENT
#include "app_otac.h"
#endif
#if BLE_OTA_SERVER
#include "app_otas.h"
#endif
#endif /* BLE_APP_PRESENT */

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/*! @brief Invalid index */
#define APP_INVALID_IDX (0xFF)

#if (BLE_APP_PRESENT)
#define APP_TIMER_SET(timer_msg_id, delay, cb) APP_TimerCfg(timer_msg_id, delay, cb)
#define APP_TIMER_CLR(timer_msg_id) APP_TimerCfg(timer_msg_id, 0, NULL)

/*! @brief APP operation type */
enum app_operation
{
    APP_OP_NOP = 0x00,  /*!< No Operation (if nothing has been requested) */
    APP_OP_ADVERTISING, /*!< Advertising */
    APP_OP_SCANING,     /*!< Scaning */
    APP_OP_INITIATING,  /*!< Initiating */
    APP_OP_CONNECTING,  /*!< The link was Connected, but the host layer connection procedure is in progress */
    APP_OP_LAST
};

/*! @brief Pairing keys information structure */
struct app_pairing_keys_info
{
    struct gapc_irk irk;     /*!< IRK */
    struct gap_sec_key csrk; /*!< CSRK */
    struct gapc_ltk ltk;     /*!< LTK */
};

/*! @brief device bond information structure */
struct app_bond_info
{
    uint8_t auth;                            /*!< Authentication level */
    uint8_t ltk_present;                     /*!< LTK exchanged during pairing. */
    uint8_t svc_changed_ind_enable;          /*!< Service Changed Indication enabled */
    uint8_t peer_addr_type;                  /*!< Address type of the peer device 0=public/1=private random */
    bd_addr_t peer_addr;                     /*!< Peer BD Address of device */
    uint32_t local_sign_counter;             /*!< Local SignCounter value */
    uint32_t peer_sign_counter;              /*!< Peer SignCounter value */
    struct app_pairing_keys_info local_keys; /*!< Local device key information */
    struct app_pairing_keys_info peer_keys;  /*!< Peer device key information */
#if (BLE_BATT_SERVER)
    struct app_bass_bond_data bass; /*!< Battery information */
#endif
};

/*! @brief Bonded device record structure */
struct app_bond_dev_record
{
    bool free;                      /*!< Status of the record */
    struct app_bond_info bond_info; /*!< bonded deivce information */
};

/*! @brief device connection information structure */
struct app_conn_info
{
    uint8_t bond_index;                      /*!< Device bond index: APP_INVALID_IDX means not bonded */
    uint8_t peer_addr_type;                  /*!< Address type of the peer device 0=public/1=private random */
    bd_addr_t peer_addr;                     /*!< Peer BD Address of device */
    struct app_pairing_keys_info local_keys; /*!< Local device key information */
    struct app_pairing_keys_info peer_keys;  /*!< Peer device key information */
};

/*! @brief connected device record structure */
struct app_conn_dev_record
{
    bool free;                      /*!< Status of the record */
    struct app_conn_info conn_info; /*!< connected device information */
};

/*! @brief Application environment structure */
struct app_env_tag
{
    uint16_t conhdl;  /*!< Connection handle */
    uint8_t state;    /*!< Operation state. */
    uint8_t next_prf; /*!< Last initialized profile */

    uint8_t scan_count; /*!< Scanned devices count */
    uint8_t conn_count; /*!< Connected devices count */
    uint8_t bond_count; /*!< Bonded devices count */

#if (BLE_CENTRAL)
    struct gap_bdaddr scan_dev_list[CFG_SCAN_DEV_MAX]; /*!< Scanned devices list */
#endif

#if (BLE_PERIPHERAL || BLE_BROADCASTER)
    uint8_t adv_data_len; /*!< Advertising data length - maximum 28 bytes, 3 bytes are reserved to set advertising AD
                             type flags, shall not be set in advertising data */
    uint8_t adv_data[GAP_ADV_DATA_LEN - 3];       /*!< Advertising data */
    uint8_t scan_rsp_data_len;                    /*!< Scan response data length- maximum 31 bytes */
    uint8_t scan_rsp_data[GAP_SCAN_RSP_DATA_LEN]; /*!< Scan response data */
#endif

    struct app_conn_dev_record conn_dev_list[BLE_CONNECTION_MAX]; /*!< Connected devices list */
    struct app_bond_dev_record
        bond_dev_list[CFG_BOND_DEV_MAX]; /*!< Bonded devices list (which will be write to flash) */
    struct gap_sec_key irk;              /*!< Initial local IRK */

#if defined(CFG_DEMO_MENU)
    uint8_t menu_id;    /*!< Menu ID */
    uint8_t pre_id;     /*!< Previous menu id */
    uint8_t input[10];  /*!< Input character */
    uint8_t select_idx; /*!< Selected device index */
#endif
};

/*******************************************************************************
 * Variables
 ******************************************************************************/
/*! @brief Application environment variable */
extern struct app_env_tag g_AppEnv;
/*! @brief Application pairing parameters */
extern const struct gapc_pairing s_appPairingParameters;

/*******************************************************************************
 * API
 ******************************************************************************/

/*!
 * @brief Initialize the application task.
 * @return None
 */
void APP_Init(void);

/*!
 * @brief Create new task for specific profile.
 * @return Whether more profiles need to be added or not
 */
bool APP_AddPrfTask(void);

/*!
 * @brief Construct adv data from user defined data and device name.
 */
#ifdef MBED_PORT
void APP_ConstructAdvData(uint8_t const *adv_data,
                          uint8_t adv_data_len,
                          uint8_t const *scan_rsp_data,
                          uint8_t scan_rsp_data_len);
#else
void APP_ConstructAdvData(uint8_t *adv_data, uint8_t adv_data_len, uint8_t *scan_rsp_data, uint8_t scan_rsp_data_len);
#endif

/*!
 * @brief Set BD address.
 * @param[in] addr_type      Address type
 * - GAPM_CFG_ADDR_PUBLIC
 * - GAPM_CFG_ADDR_PRIVATE
 * @param[in] addr           Provided address that will be set to the low layers
 * @description
 * This function is used to set public or random static address for BLE device without re-initializing the BLE stack,
 * but this API should be called while no any air operation is in progress, such as advertising, scaning and connecting.
 */
void APP_SetBDAddress(uint8_t addr_type, struct bd_addr *addr);

/*!
 * @brief Get the connection number of the device record.
 */
uint8_t APP_GetLinkNb(void);

/*!
 * @brief Get Connection status by connection handle.
 * @return linked or not
 */
bool APP_GetLinkStatusByConhdl(uint16_t conhdl);

/*!
 * @brief Get Peer BD address by connection handle.
 *
 * @param[in] conhdl    Connection handle.
 * @param[in] addr      Pointer to the result structure.
 * @return Whether the BD address is found or not
 */
bool APP_GetPeerBdAddrByConhdl(uint16_t conhdl, bd_addr_t *addr, uint8_t *addr_type);

/*!
 * @brief Set connection information by connection handle.
 *
 * @param[in] conhdl    Connection handle.
 * @param[in] conn_info Pointer to the connection information.
 * @param[in] conn      Connected or disconnected.
 * @return None
 */
void APP_SetLinkInfoByConhdl(uint16_t conhdl, struct gapc_connection_req_ind const *conn_info, bool conn);

/*!
 * @brief Find the bonded device by device address.
 *
 * @param[in] addr      Pointer to the BD address structure.
 * @param[in] addr_type Address type.
 * @return The index of bonded device.
 */
uint8_t APP_FindBondedDevByAddr(bd_addr_t const *addr, uint8_t addr_type);

/*!
 * @brief Find the bonded device by local key information.
 *
 * @param[in] conhdl   Connection handle.
 * @param[in] key      Security key information.
 * @param[in] key_type Security key type.
 * @return The index of bonded device.
 */
uint8_t APP_FindBondedDevByLocalKey(uint16_t conhdl, struct app_pairing_keys_info *key, uint8_t key_type);

/*!
 * @brief Find the bonded device by peer key information.
 *
 * @param[in] key      Security key information.
 * @param[in] key_type Security key type.
 * @return The index of bonded device.
 */
uint8_t APP_FindBondedDevByPeerKey(struct app_pairing_keys_info *key, uint8_t key_type);

/*!
 * @brief Add a bonded device.
 *
 * @param[in] bonded_dev    Pointer to the bond information.
 * @return The index of bonded device.
 */
uint8_t APP_AddBondedDev(struct app_conn_info *bonded_dev);

/*!
 * @brief Remove a bonded device.
 *
 * @param[in] idx    The index of a bonded device.
 * @return Remove the bonded device sucessful or failed.
 */
bool APP_RemoveBondedDev(uint8_t idx);

/*!
 * @brief Clear all of the bonded device information.
 * @return None.
 */
void APP_ClearBondInf(void);

/*!
 * @brief Generates pin code
 *
 * @return pin code
 */
uint32_t APP_SecGenTk(void);

/*!
 * @brief Generates long term key.
 *
 * @param[in] conhdl        Connection handle.
 * @param[in] key_size      Generated key's size.
 *
 * @return void
 */
void APP_SecGenLtk(uint16_t conhdl, uint8_t key_size);

/*!
 * @brief Generates security keys.
 *
 * @param[in] key           Pointer to the key buffer.
 * @param[in] key_size      Generated key's size.
 *
 * @return void
 */
void APP_SecGenKey(uint8_t *key, uint8_t key_size);

/*!
 * @brief APP Message sending.
 *
 * Send a message previously allocated with any ke_msg_alloc()-like functions.
 *
 * The kernel will take care of freeing the message memory.
 *
 * Once the function have been called, it is not possible to access its data
 * anymore as the kernel may have copied the message and freed the original
 * memory.
 *
 * @param[in] cmd  Pointer to the parameter member of the message that should be sent.
 */
void APP_MsgSend(void *cmd);

/*!
 * @brief Register APP message handler.
 *
 * @param[in] id        Id of the handled message
 * @param[in] func      Pointer to the handler function for the msgid.
 *
 * @return void
 */
void APP_RegisterMessageHandler(ke_msg_id_t id, ke_msg_func_t func);

/*!
 * @brief Configure a timer.
 *
 * The function first cancel the timer if it is already existing, then
 * it creates a new one. The timer can be one-shot or periodic, i.e. it
 * will be automatically set again after each trigger.
 *
 * When the timer expires, a message is sent to APP task with the timer message id.
 *
 * The timer is programmed in time units (TU is 10ms).
 *
 * @param[in] timer_msg_id  Timer identifier (message identifier type).
 * @param[in] delay         Delay in time units.
 * @param[in] func          timeout callback function.
 */
void APP_TimerCfg(ke_msg_id_t const timer_msg_id, uint32_t delay, ke_msg_func_t func);

/*!
 * @brief gather Packet Error Rate statistics and output them via log
 */
void APP_StartStatistic(uint16_t conhdl);
void APP_StopStatistic(uint16_t conhdl);
void APP_ShowStatistic(void);
void APP_PktStatistic(uint16_t rx_st, uint16_t conhdl);

#else
#define APP_TIMER_SET(timer_msg_id, delay, cb)
#define APP_TIMER_CLR(timer_msg_id)
#endif /* BLE_APP_PRESENT */

/*!
 * @brief Check if the BLE is active or not.
 * @return Returns true if BLE is active.
 */
static bool APP_BleIsActive(void)
{
    return CFG_API_FLAG_GET(SLEEP) ? false : true;
}

/*!
 * @brief Wakeup BLE core by software request.
 */
static void APP_BleSoftwareWakeup(void)
{
    ble_soft_wakeup_req_setf(1U);
}

/*!
 * @brief Register Semaphore give API.
 */
void App_RegisterSemaphoreGive(void (*func)(void));

/*!
 * @brief Register BLE interrupt callback function.
 */
void App_RegisterBLEInterruptCallback(void (*callback)(uint32_t status));

/*! @brief @} APP_API */

#endif /* _APP_BLE_H_ */

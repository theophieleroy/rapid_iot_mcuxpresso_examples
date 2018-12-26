/**
****************************************************************************************
*
* @file rwip.h
*
* @brief RW IP SW main module
*
* Copyright (C) RivieraWaves 2009-2015
*
*
****************************************************************************************
*/
#ifndef _RWIP_H_
#define _RWIP_H_

/**
 ****************************************************************************************
 * @addtogroup ROOT
 * @brief Entry points of the RW IP stacks/modules
 *
 * This module contains the primitives that allow an application accessing and running the
 * RW IP protocol stacks / modules.
 *
 * @{
 ****************************************************************************************
 */

#include "rwip_config.h" // stack configuration

#include <stdint.h>  // standard integer definitions
#include <stdbool.h> // standard boolean definitions
#include "ke_msg.h"
#include "prf.h"
#include "co_bt.h"
#include "co_hci.h"

// Heap header size is 12 bytes
#define RWIP_HEAP_HEADER (12 / sizeof(uint32_t))
#define RWIP_MEM_ALIGN(len) (((len) + (sizeof(uint32_t) - 1)) / sizeof(uint32_t))

// ceil(len/sizeof(uint32_t)) + RWIP_HEAP_HEADER
#define RWIP_CALC_HEAP_LEN(len) (RWIP_MEM_ALIGN(len) + RWIP_HEAP_HEADER)

/// retrieve 10ms time according to clock time
#define RWIP_CLOCK_TO_10MS_TIME(clock) ((clock) >> 4)
/// retrieve clock time according to 10ms time
#define RWIP_10MS_TIME_TO_CLOCK(time) ((time) << 4)
/// Invalid target time
#define RWIP_INVALID_TARGET_TIME (0xFFFFFFFFL)

#if (DEEP_SLEEP)
/// Sleep Duration Value in periodic wake-up mode
#define MAX_SLEEP_DURATION_PERIODIC_WAKEUP 0x0320 // 0.5s
/// Sleep Duration Value in external wake-up mode
#define MAX_SLEEP_DURATION_EXTERNAL_WAKEUP 0x3E80 // 10s

/// Definition of the bits preventing the system from sleeping
enum prevent_sleep
{
    /// Flag indicating that the wake up process is ongoing
    RW_WAKE_UP_ONGOING = 0x0001,
    /// Flag indicating that an TX transfer is ongoing on Transport Layer
    RW_TL_TX_ONGOING = 0x0002,
    /// Flag indicating that an RX transfer is ongoing on Transport Layer
    RW_TL_RX_ONGOING = 0x0004,
    /// Flag indicating HCI timeout is ongoing
    RW_AHI_TIMEOUT = 0x0008,
    /// Flag indicating that an encryption is ongoing
    RW_CRYPT_ONGOING = 0x0010,
    /// Flag indicating that a element deletion is on going
    RW_DELETE_ELT_ONGOING = 0x0020,
    /// Flag indicating that controller shall not sleep due to not CSB LPO_Allowed
    RW_CSB_NOT_LPO_ALLOWED = 0x0040,
    /// Flag indicating the MWS/WLAN Event Generator is in operation
    RW_MWS_WLAN_EVENT_GENERATOR_ACTIVE = 0x0080
};
#endif // DEEP_SLEEP

/**
 * External interface type types.
 */
enum rwip_eif_types
{
    /// Host Controller Interface - Controller part
    RWIP_EIF_HCIC,

    /// Host Controller Interface - Host part
    RWIP_EIF_HCIH,

    /// Application Host interface
    RWIP_EIF_AHI,
};

/// Enumeration of External Interface status codes
enum rwip_eif_status
{
    /// EIF status OK
    RWIP_EIF_STATUS_OK,
    /// EIF status KO
    RWIP_EIF_STATUS_ERROR,

#if (BLE_EMB_PRESENT == 0)
    /// External interface detached
    RWIP_EIF_STATUS_DETACHED,
    /// External interface attached
    RWIP_EIF_STATUS_ATTACHED,
#endif // (BLE_EMB_PRESENT == 0)
};

/// Enumeration of RF modulations
enum rwip_rf_mod
{
    MOD_GFSK = 0x01,
    MOD_DQPSK = 0x02,
    MOD_8DPSK = 0x03,
};

/// API functions of the RF driver that are used by the BLE or BT software
struct rwip_rf_api
{
    /// Function called upon HCI reset command reception
    void (*reset)(void);
    /// Function called to enable/disable force AGC mechanism (true: en / false : dis)
    void (*force_agc_enable)(bool);
    /// Function called to convert a TX power CS power field into the corresponding value in dBm
    int8_t (*txpwr_dbm_get)(uint8_t);
    /// Function called to convert the RSSI read from the control structure into a real RSSI
    int8_t (*rssi_convert)(uint8_t);
    /// Index of maximum TX power
    uint8_t txpwr_max;
};

/// Internal API for priority
struct rwip_prio
{
    /// value
    uint8_t value;
    /// Increment
    uint8_t increment;
};

/// Internal API for COEX
struct rwip_coex
{
    /// Coexistence control field
    uint8_t coex_cntl;
};

/**
 ****************************************************************************************
 * @brief Function called when packet transmission/reception is finished.

 * @param[in]  dummy  Dummy data pointer returned to callback when operation is over.
 * @param[in]  status Ok if action correctly performed, else reason status code.
 *****************************************************************************************
 */
typedef void (*rwip_eif_callback)(void *, uint8_t);

/**
 * Transport layer communication interface.
 */
struct rwip_eif_api
{
    /**
     *************************************************************************************
     * @brief Starts a data reception.
     *
     * @param[out] bufptr      Pointer to the RX buffer
     * @param[in]  size        Size of the expected reception
     * @param[in]  callback    Pointer to the function called back when transfer finished
     * @param[in]  dummy       Dummy data pointer returned to callback when reception is finished
     *************************************************************************************
     */
    void (*read)(uint8_t *bufptr, uint32_t size, rwip_eif_callback callback, void *dummy);

    /**
     *************************************************************************************
     * @brief Starts a data transmission.
     *
     * @param[in]  bufptr      Pointer to the TX buffer
     * @param[in]  size        Size of the transmission
     * @param[in]  callback    Pointer to the function called back when transfer finished
     * @param[in]  dummy       Dummy data pointer returned to callback when transmission is finished
     *************************************************************************************
     */
    void (*write)(uint8_t *bufptr, uint32_t size, rwip_eif_callback callback, void *dummy);

    /**
     *************************************************************************************
     * @brief Enable Interface flow.
     *************************************************************************************
     */
    void (*flow_on)(void);

    /**
     *************************************************************************************
     * @brief Disable Interface flow.
     *
     * @return True if flow has been disabled, False else.
     *************************************************************************************
     */
    bool (*flow_off)(void);
};

/*
 * VARIABLE DECLARATION
*****************************************************************************************
 */

/// API for RF driver
extern struct rwip_rf_api *rwip_rf;
#if (BLE_EMB_PRESENT || BT_EMB_PRESENT)
/// API for dual mode priority
// extern const struct rwip_prio rwip_priority[RWIP_PRIO_IDX_MAX];
extern const struct rwip_prio *rwip_priority;
/// API for COEX
// extern const uint8_t rwip_coex_cfg[RWIP_COEX_CFG_MAX];
extern const uint8_t *rwip_coex_cfg;
#endif //#if (BLE_EMB_PRESENT || BT_EMB_PRESENT)

/*
 * MACROS
 ****************************************************************************************
 */

/// Get Event status flag
#define RWIP_COEX_GET(coex_cfg_idx, bit_field) \
    (uint8_t)(((rwip_coex_cfg[RWIP_COEX_##coex_cfg_idx##_IDX]) >> RWIP_##bit_field##_POS) & RWIP_COEX_BIT_MASK)

/*
 * FUNCTION DECLARATION
*****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Initializes the RW BT SW.
 *
 ****************************************************************************************
 */
void rwip_init(uint32_t error);

/**
 ****************************************************************************************
 * @brief Reset the RW BT SW.
 *
 ****************************************************************************************
 */
void rwip_reset(void);

/**
 ****************************************************************************************
 * @brief Gives FW/HW versions of RW-BT stack.
 *
 ****************************************************************************************
 */
void rwip_version(uint8_t *fw_version, uint8_t *hw_version);

/**
 ****************************************************************************************
 * @brief Schedule all pending events.
 *
 ****************************************************************************************
 */
void rwip_schedule(void);

/**
 ****************************************************************************************
 * @brief Invoke the sleep function.
 *
 * @return  true: processor sleep allowed, false otherwise
 ****************************************************************************************
 */
uint8_t rwip_sleep(void);

#if DEEP_SLEEP
/**
 ****************************************************************************************
 * @brief Handle wake-up.
 ****************************************************************************************
 */
void rwip_wakeup(void);

/**
 ****************************************************************************************
 * @brief Handle end of wake-up.
 ****************************************************************************************
 */
void rwip_wakeup_end(void);

/**
 ****************************************************************************************
 * @brief Set the wake-up delay
 *
 * @param[in] wakeup_delay   Wake-up delay in us
 ****************************************************************************************
 */
void rwip_wakeup_delay_set(uint16_t wakeup_delay);

/**
 ****************************************************************************************
 * @brief Set a bit in the prevent sleep bit field, in order to prevent the system from
 *        going to sleep
 *
 * @param[in] prv_slp_bit   Bit to be set in the prevent sleep bit field
 ****************************************************************************************
 */
void rwip_prevent_sleep_set(uint16_t prv_slp_bit);

/**
 ****************************************************************************************
 * @brief Clears a bit in the prevent sleep bit field, in order to allow the system
 *        going to sleep
 *
 * @param[in] prv_slp_bit   Bit to be cleared in the prevent sleep bit field
 ****************************************************************************************
 */
void rwip_prevent_sleep_clear(uint16_t prv_slp_bit);

/**
 ****************************************************************************************
 * @brief Check if sleep mode is enable
 *
 * @return    true if sleep is enable, false otherwise
 ****************************************************************************************
 */
bool rwip_sleep_enable(void);

/**
 ****************************************************************************************
 * @brief Enable/disable BLE sleep algorithm
 *
 * @param    true: enable ble sleep; false: disable ble sleep
 ****************************************************************************************
 */
void enable_ble_sleep(bool enable);

/**
 ****************************************************************************************
 * @brief Check if external wake-up is enable
 *
 * @return    true if external wakeup is enable, false otherwise
 ****************************************************************************************
 */
bool rwip_ext_wakeup_enable(void);

/**
 ****************************************************************************************
 * @brief Converts a duration in lp cycles into a duration is us.
 *
 * The function converts a duration in lp cycles into a duration is us, according to the
 * low power clock frequency.
 *
 * @param[in] lpcycles    duration in lp cycles
 *
 * @return duration in us
 ****************************************************************************************
 */
uint32_t rwip_lpcycles_2_us(uint32_t lpcycles);

/**
 ****************************************************************************************
 * @brief Converts a duration in us into a duration in lp cycles.
 *
 * The function converts a duration in us into a duration is lp cycles, according to the
 * low power clock frequency.
 *
 * @param[in] us    duration in us
 *
 * @return duration in lpcycles
 ****************************************************************************************
 */
uint32_t rwip_us_2_lpcycles(uint32_t us);
#endif // DEEP_SLEEP

#if (BT_EMB_PRESENT)

#if PCA_SUPPORT
/**
 ****************************************************************************************
 * @brief Check if clock dragging limitation
 *
 * @return    true if clock dragging must be used
 ****************************************************************************************
 */
bool rwip_pca_clock_dragging_only(void);
#endif // PCA_SUPPORT
#endif // (BT_EMB_PRESENT)

#if (BT_EMB_PRESENT || BLE_EMB_PRESENT)
#if (RW_MWS_COEX)
/**
 ****************************************************************************************
 * @brief Enable/Disable the MWS coexistence interface.
 *
 * @param[in]   CoexSetting     Coexistence value
 *
 ****************************************************************************************
 */
void rwip_mwscoex_set(bool state);
#endif // RW_MWS_COEX

#if (RW_WLAN_COEX)
/**
 ****************************************************************************************
 * @brief Enable/Disable the Wireless LAN coexistence interface.
 *
 * @param[in]   CoexSetting     Coexistence value
 *
 ****************************************************************************************
 */
void rwip_wlcoex_set(bool state);
#endif // RW_WLAN_COEX
#endif //(BT_EMB_PRESENT || BLE_EMB_PRESENT)

/**
 ****************************************************************************************
 * @brief Function to implement in platform in order to retrieve expected external
 * interface such as UART for Host Control Interface.
 *
 * @param[in] type external interface type (@see rwip_eif_types)
 *
 * @return External interface api structure
 ****************************************************************************************
 */
extern const struct rwip_eif_api *rwip_eif_get(uint8_t type);

#if RW_DEBUG
/**
 ****************************************************************************************
 * @brief Raises an assertion message to the control interface (if present)
 *
 * @param[in] file    File name
 * @param[in] line    Line number
 * @param[in] param0  Parameter 0 (custom value given by the assert instruction)
 * @param[in] param1  Parameter 1 (custom value given by the assert instruction)
 ****************************************************************************************
 */
void rwip_assert_err(const char *file, int line, int param0, int param1);
#endif // RW_DEBUG

/**
 ****************************************************************************************
 * @brief Function to implement in platform in order to retrieve expected external
 * interface such as UART for Host Control Interface.
 *
 * @param[in] type external interface type (@see rwip_eif_types)
 *
 * @return External interface api structure
 ****************************************************************************************
 */
#define CFG_API_FLAG_HCI 0x01
#define CFG_API_FLAG_ACI 0x02
#define CFG_API_FLAG_NO_ADV_DLY 0x04
#define CFG_API_FLAG_FAST_CORRECT 0x08
#define CFG_API_FLAG_SLEEP 0x20

/// Set configuration flag
#define CFG_API_FLAG_SET(flag) (ble_config.flags |= (CFG_API_FLAG_##flag))
/// Reset configuration flag
#define CFG_API_FLAG_RESET(flag) (ble_config.flags &= (~(CFG_API_FLAG_##flag)))
/// Get configuration flag
#define CFG_API_FLAG_GET(flag) (ble_config.flags & (CFG_API_FLAG_##flag))
/// Clear configuration flag
#define CFG_API_FLAG_CLR() (ble_config.flags = 0)

/// Description of unloaded RAM area content
struct unloaded_area_tag
{
    // status error
    uint32_t error;
};

struct fw_static_cfg
{
    // Stack
    uint8_t ble_whitelist_max;
    uint8_t ble_resol_addr_list_max;
    uint8_t ble_duplicate_filter_max;
    uint8_t gapm_scan_filter_size;
    uint16_t gap_max_le_mtu;
    uint32_t gap_tmr_lim_adv_timeout;

    uint32_t att_trans_rtx;
    uint16_t smpc_rep_att_timer_def_val;
    uint16_t smpc_rep_att_timer_max_val;
    uint16_t smpc_rep_att_timer_mult;
    uint16_t smpc_timout_timer_duration;

    void (*app_pkt_statistic)(uint16_t rx_st, uint16_t conhdl);
};

struct app_static_cfg
{
    // Platform
    struct unloaded_area_tag *unloaded_area; // Map unloaded RAM area pointer onto RAM memory
    void (*plf_reset_cb)(uint32_t err);

    // HCI extend function
    uint8_t (*hci_user_ext_func)(struct hci_user_ext_func_cmd *params);
    int (*hci_common_callback)(ke_msg_id_t const msgid,
                               void const *param,
                               ke_task_id_t const dest_id,
                               ke_task_id_t const src_id);

    // BLE heap
    uint32_t *rwip_heap_env;
    uint32_t *rwip_heap_db;
    uint32_t *rwip_heap_msg;
    uint32_t *rwip_heap_non_ret;
    uint16_t rwip_heap_env_size;
    uint16_t rwip_heap_db_size;
    uint16_t rwip_heap_msg_size;
    uint16_t rwip_heap_non_ret_size;

    // Profile
    void (*prf_init)(bool reset);
    void (*prf_cleanup)(uint8_t conidx, uint8_t reason);
    void (*prf_create)(uint8_t conidx);
    ke_task_id_t (*prf_get_id_from_task)(ke_msg_id_t task);
    ke_task_id_t (*prf_get_task_from_id)(ke_msg_id_t id);
#if (BLE_PROFILES)
    uint8_t (*prf_add_profile)(struct gapm_profile_task_add_cmd *params, ke_task_id_t *prf_task);
    prf_env_t *(*prf_env_get)(uint16_t prf_id);
#else
    void (*prf_add_profile)(void);
    void (*prf_env_get)(void);
#endif
    // HCI / ACI
    struct rwip_eif_api eif_api;

    // RF
    struct rwip_rf_api *rf_api;

    // Kernel
    uint8_t ke_task_max;
    const struct ke_task_desc *ble_task_desc;

    uint16_t app_main_task;
    uint8_t ble_con_max;

    // Sleep
    uint8_t ea_clock_corr_lat;
    uint8_t prog_latency_def;
    // background sleep timer setting
    uint32_t bg_sleep_duration;
};

struct em_offset_cfg
{
    // EM item's offset
    uint32_t em_ble_wpb_offset;
    uint32_t em_ble_wpv_offset;
    uint32_t em_ble_ral_offset;
    uint32_t em_ble_tx_desc_offset;
    uint32_t em_ble_rx_desc_offset;
    uint32_t em_ble_tx_buffer_ctrl_offset;
    uint32_t em_ble_tx_buffer_data_offset;
    uint32_t em_ble_tx_buf_data_cnt;
    uint32_t em_ble_rx_buffer_offset;
    uint32_t em_ble_rx_buffer_size;
    uint32_t em_ble_rx_buffer_cnt;
};

struct ble_config_st
{
    struct fw_static_cfg *fw;
    struct app_static_cfg *app;
    struct em_offset_cfg *em;

    bool sw_32k_calib_enable; /*!< true: software calibrates 32k clock, 32k RCO has to use this mode.
                                  false: do not use software to calibrate 32k clock, 32k XTAL usually uses this mode. */
    int32_t ppm_32k;          /*!< the result of 32k clock software calibration */

    // Flag
    volatile uint8_t flags;

    uint32_t fw_seed; /*!< Pass a seed for firmware's srand() */

    // Interrupt mask
    uint32_t int_mask0;
    uint32_t int_mask1;
};

/// BLE stack configuration interface.
extern struct ble_config_st ble_config;

/// Variable storing the reason of platform reset
extern uint32_t error;

void unloaded_area_init(void);

void set_32k_ppm(int32_t ppm);

/**
 ****************************************************************************************
 * @brief Re-boot FW.
 *
 * This function is used to re-boot the FW when error has been detected, it is the end of
 * the current FW execution.
 * After waiting transfers on UART to be finished, and storing the information that
 * FW has re-booted by itself in a non-loaded area, the FW restart by branching at FW
 * entry point.
 *
 * Note: when calling this function, the code after it will not be executed.
 *
 * @param[in] error      Error detected by FW
 ****************************************************************************************
 */
void platform_reset(uint32_t error);

#if BLE_FW_PRESENT
__FORCEINLINE void prf_init(bool reset)
{
    if (ble_config.app->prf_init != NULL)
        ble_config.app->prf_init(reset);
}

__FORCEINLINE void prf_cleanup(uint8_t conidx, uint8_t reason)
{
    if (ble_config.app->prf_cleanup != NULL)
        ble_config.app->prf_cleanup(conidx, reason);
}

__FORCEINLINE void prf_create(uint8_t conidx)
{
    if (ble_config.app->prf_create != NULL)
        ble_config.app->prf_create(conidx);
}

__FORCEINLINE ke_task_id_t prf_get_id_from_task(ke_msg_id_t task)
{
    if (ble_config.app->prf_get_id_from_task != NULL)
        return ble_config.app->prf_get_id_from_task(task);
    return TASK_ID_INVALID;
}

__FORCEINLINE ke_task_id_t prf_get_task_from_id(ke_msg_id_t id)
{
    if (ble_config.app->prf_get_task_from_id != NULL)
        return ble_config.app->prf_get_task_from_id(id);
    return TASK_NONE;
}

__FORCEINLINE uint8_t prf_add_profile(struct gapm_profile_task_add_cmd *params, ke_task_id_t *prf_task)
{
    if (ble_config.app->prf_add_profile != NULL)
        return ble_config.app->prf_add_profile(params, prf_task);
    return GAP_ERR_NO_ERROR;
}

__FORCEINLINE prf_env_t *prf_env_get(uint16_t prf_id)
{
    if (ble_config.app->prf_env_get != NULL)
        return ble_config.app->prf_env_get(prf_id);
    return NULL;
}
#endif

///@} ROOT

#endif // _RWIP_H_

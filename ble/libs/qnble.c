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

#include "rwip_config.h" /* RW SW configuration */

#include "dbg_assert.h"
#include <stdlib.h> /* standard lib functions */
#include <stddef.h> /* standard definitions */
#include "intc.h"   /* Interrupt initialization */

#include "em_map_ble.h"
#include "em_map.h"

#include "l2cc_int.h"
#include "gattm_int.h"
#include "gattc_int.h"
#include "gapm_int.h"
#include "gapc_int.h"

#include "llm_task.h"
#include "llc_task.h"
#include "dbg_task.h"

#include "llc.h"
#include "hci.h"
#include "ke_task.h"

#include "rwble_hl.h" /* BLE HL definitions */
#include "gapc.h"
#include "smpc.h"
#include "gattc.h"
#include "attc.h"
#include "atts.h"
#include "l2cc.h"

#include "app_ble_task.h"
#include "ahi_task.h"
#include "fsl_reset.h"
#include "app_config.h"
#include "app_ble.h"
#include "eif_adapter.h"
#include "rf.h"
#include "fsl_rng.h"
#include "fsl_power.h"
#include "fsl_iocon.h"

const uint8_t user_rwip_coex_cfg[RWIP_COEX_CFG_MAX]=
{
    [RWIP_COEX_CON_IDX]     = (uint8_t)((RWIP_PTI_TXDIS << RWIP_TXBSY_POS)  | (RWIP_PTI_RXDIS << RWIP_RXBSY_POS)  | (RWIP_PTI_DNABORTDIS << RWIP_DNABORT_POS)),
    [RWIP_COEX_CON_DATA_IDX]= (uint8_t)((RWIP_PTI_TXEN << RWIP_TXBSY_POS)  | (RWIP_PTI_RXEN << RWIP_RXBSY_POS)  | (RWIP_PTI_DNABORTDIS << RWIP_DNABORT_POS)),
    [RWIP_COEX_ADV_IDX]     = (uint8_t)((RWIP_PTI_TXEN << RWIP_TXBSY_POS)  | (RWIP_PTI_RXDIS << RWIP_RXBSY_POS) | (RWIP_PTI_DNABORTDIS << RWIP_DNABORT_POS)),
    [RWIP_COEX_SCAN_IDX]    = (uint8_t)((RWIP_PTI_TXDIS << RWIP_TXBSY_POS) | (RWIP_PTI_RXEN << RWIP_RXBSY_POS)  | (RWIP_PTI_DNABORTDIS << RWIP_DNABORT_POS)),
    [RWIP_COEX_INIT_IDX]    = (uint8_t)((RWIP_PTI_TXEN << RWIP_TXBSY_POS)  | (RWIP_PTI_RXEN << RWIP_RXBSY_POS)  | (RWIP_PTI_DNABORTDIS << RWIP_DNABORT_POS)),
};

void APP_InitWlanCoex(void)
{
    IOCON_PinMuxSet(IOCON, 0, 10, IOCON_FUNC6);
    IOCON_PinMuxSet(IOCON, 0, 11, IOCON_FUNC6);
    
    rwip_coex_cfg = user_rwip_coex_cfg;
}

/*! @brief Exchange memory definitions */

#define CFG_EM_BLE_CS_COUNT (BLE_CONNECTION_MAX + 1)
#define CFG_EM_BLE_TX_DESC_COUNT (CFG_BLE_TX_DESC_CNT)
#define CFG_EM_BLE_RX_DESC_COUNT (CFG_BLE_RX_BUFFER_CNT)
#define CFG_EM_BLE_TX_BUFF_CNTL_COUNT (CFG_BLE_TX_BUFF_CNTL_CNT)
#define CFG_EM_BLE_TX_BUFF_DATA_COUNT (CFG_BLE_TX_BUFF_DATA_CNT)

/*! @brief Offset of the public white list area */
#define APP_EM_BLE_WPB_OFFSET (EM_BLE_CS_OFFSET + CFG_EM_BLE_CS_COUNT * REG_BLE_EM_CS_SIZE)
/*! @brief Offset of the private white list area */
#define APP_EM_BLE_WPV_OFFSET (APP_EM_BLE_WPB_OFFSET + CFG_BLE_WHITELIST_MAX * REG_BLE_EM_WPB_SIZE)
/*! @brief Offset of the private white list area */
#define APP_EM_BLE_RAL_OFFSET (APP_EM_BLE_WPV_OFFSET + CFG_BLE_WHITELIST_MAX * REG_BLE_EM_WPV_SIZE)
/*! @brief Offset of the TX descriptor area */
#define APP_EM_BLE_TX_DESC_OFFSET (APP_EM_BLE_RAL_OFFSET + CFG_BLE_RESOL_ADDR_LIST_MAX * REG_BLE_EM_RAL_SIZE)
/*! @brief Offset of the RX descriptor area */
#define APP_EM_BLE_RX_DESC_OFFSET (APP_EM_BLE_TX_DESC_OFFSET + CFG_EM_BLE_TX_DESC_COUNT * REG_BLE_EM_TX_DESC_SIZE)
/*! @brief Offset of the TX buffer area */
#define APP_EM_BLE_TX_BUFFER_CNTL_OFFSET \
    (APP_EM_BLE_RX_DESC_OFFSET + CFG_EM_BLE_RX_DESC_COUNT * REG_BLE_EM_RX_DESC_SIZE)
/*! @brief Offset of the TX buffer area */
#define APP_EM_BLE_TX_BUFFER_DATA_OFFSET \
    (APP_EM_BLE_TX_BUFFER_CNTL_OFFSET +  \
     (CFG_EM_BLE_TX_BUFF_CNTL_COUNT + EM_BLE_TX_BUFF_ADV_COUNT) * REG_BLE_EM_TX_BUFFER_CNTL_SIZE)
/*! @brief Offset of the RX buffer area */
#define APP_EM_BLE_RX_BUFFER_OFFSET \
    (APP_EM_BLE_TX_BUFFER_DATA_OFFSET + CFG_EM_BLE_TX_BUFF_DATA_COUNT * CFG_REG_BLE_EM_TX_BUFFER_DATA_SIZE)
/*! @brief End of BLE EM */
#define APP_EM_BLE_END (APP_EM_BLE_RX_BUFFER_OFFSET + CFG_BLE_RX_BUFFER_CNT * CFG_REG_BLE_EM_RX_BUFFER_SIZE)

/*! @brief Exchange memory */
uint32_t rwip_exchange_memory[RWIP_MEM_ALIGN(APP_EM_BLE_END)];

/*! @brief Heap definitions - use uint32 to ensure that memory blocks are 32bits aligned. */

/*! @brief Memory allocated for environment variables */
uint32_t rwip_heap_env[RWIP_CALC_HEAP_LEN(RWIP_HEAP_ENV_SIZE)];
/*! @brief Memory allocated for Attribute database */
uint32_t rwip_heap_db[RWIP_CALC_HEAP_LEN(RWIP_HEAP_DB_SIZE)];
/*! @brief Memory allocated for kernel messages */
uint32_t rwip_heap_msg[RWIP_CALC_HEAP_LEN(RWIP_HEAP_MSG_SIZE)];
/*! @brief Non Retention memory block */
uint32_t rwip_heap_non_ret[RWIP_CALC_HEAP_LEN(RWIP_HEAP_NON_RET_SIZE)];

/*! @brief GAPC task instance. */
ke_state_t gapc_state[BLE_CONNECTION_MAX];
/*! @brief GATT task instance. */
ke_state_t gattc_state[BLE_CONNECTION_MAX];
/*! @brief Defines the place holder for the states of all the task instances. */
ke_state_t l2cc_state[BLE_CONNECTION_MAX];
/*! @brief Defines the place holder for the states of all the task instances. */
ke_state_t llc_state[BLE_CONNECTION_MAX];

const struct ke_task_desc ble_task_desc[] = {
        [TASK_LLM] = {NULL, &llm_default_handler, llm_state, LLM_STATE_MAX, LLM_IDX_MAX},
#if (BLE_PERIPHERAL || BLE_CENTRAL)
        [TASK_LLC] = {NULL, &llc_default_handler, llc_state, LLC_STATE_MAX, BLE_CONNECTION_MAX},
#endif
        [TASK_LLD] = {NULL, NULL, NULL, 1, 1},
        [TASK_DBG] = {NULL, &dbg_default_handler, dbg_state, DBG_STATE_MAX, DBG_IDX_MAX},
#if (BLE_APP_PRESENT)
        [TASK_APP] = {NULL, &app_default_handler, app_state, APP_STATE_MAX, APP_IDX_MAX},
#endif
        [TASK_AHI] = {NULL, &ahi_default_handler, ahi_state, AHI_STATE_MAX, AHI_IDX_MAX},
#if (BLE_PERIPHERAL || BLE_CENTRAL)
        [TASK_L2CC] = {NULL, &l2cc_default_handler, l2cc_state, L2CC_STATE_MAX, BLE_CONNECTION_MAX},
        [TASK_GATTM] = {NULL, &gattm_default_handler, gattm_state, GATTM_STATE_MAX, GATTM_IDX_MAX},
        [TASK_GATTC] = {NULL, &gattc_default_handler, gattc_state, GATTC_STATE_MAX, BLE_CONNECTION_MAX},
#endif
        [TASK_GAPM] = {NULL, &gapm_default_handler, gapm_state, GAPM_STATE_MAX, GAPM_IDX_MAX},
#if (BLE_PERIPHERAL || BLE_CENTRAL)
        [TASK_GAPC] = {NULL, &gapc_default_handler, gapc_state, GAPC_STATE_MAX, BLE_CONNECTION_MAX},
#endif
};

void APP_PlfReset(uint32_t err)
{
    RESET_SetPeripheralReset(kREBOOT_RST_SHIFT_RSTn);
}

const struct app_static_cfg app_configuration = {
    .unloaded_area = (struct unloaded_area_tag *)0x2000fffc,
    .plf_reset_cb = APP_PlfReset,
    .hci_user_ext_func = NULL,
    .hci_common_callback = hci_vs_command_callback_handler,
    .rwip_heap_env = rwip_heap_env,
    .rwip_heap_db = rwip_heap_db,
    .rwip_heap_msg = rwip_heap_msg,
    .rwip_heap_non_ret = rwip_heap_non_ret,
    .rwip_heap_env_size = RWIP_HEAP_ENV_SIZE,
    .rwip_heap_db_size = RWIP_HEAP_DB_SIZE,
    .rwip_heap_msg_size = RWIP_HEAP_MSG_SIZE,
    .rwip_heap_non_ret_size = RWIP_HEAP_NON_RET_SIZE,
#if (BLE_PROFILES)
    .prf_init = prf_init,
    .prf_cleanup = prf_cleanup,
    .prf_create = prf_create,
    .prf_get_id_from_task = prf_get_id_from_task,
    .prf_get_task_from_id = prf_get_task_from_id,
    .prf_add_profile = prf_add_profile,
    .prf_env_get = prf_env_get,
#else
    .prf_init = NULL,
    .prf_cleanup = NULL,
    .prf_create = NULL,
    .prf_get_id_from_task = NULL,
    .prf_get_task_from_id = NULL,
    .prf_add_profile = NULL,
    .prf_env_get = NULL,
#endif
    {/* struct rwip_eif_api */
     .read = EIF_Read,
     .write = EIF_Write,
     .flow_on = EIF_FlowOn,
     .flow_off = EIF_FlowOff},
    /* RF */
    .rf_api = NULL,

    .ke_task_max = CFG_TASK_MAX,
    .ble_task_desc = ble_task_desc,

#if defined(CFG_WM_NP)
    .app_main_task = TASK_AHI,
#else
    .app_main_task = TASK_APP,
#endif
    .ble_con_max = BLE_CONNECTION_MAX,

    .ea_clock_corr_lat = NULL,
    .prog_latency_def = CFG_BLE_PROG_LATENCY_DFT,
    .bg_sleep_duration = MAX_SLEEP_DURATION_EXTERNAL_WAKEUP,
};

const struct fw_static_cfg fw_configuration = {
    .ble_whitelist_max = CFG_BLE_WHITELIST_MAX,
    .ble_resol_addr_list_max = CFG_BLE_RESOL_ADDR_LIST_MAX,
    .ble_duplicate_filter_max = CFG_BLE_DUPLICATE_FILTER_MAX,
    .gapm_scan_filter_size = CFG_GAPM_SCAN_FILTER_SIZE,
    .gap_max_le_mtu = CFG_GAP_MAX_LE_MTU,
    .gap_tmr_lim_adv_timeout = CFG_GAP_TMR_LIM_ADV_TIMEOUT,
    .att_trans_rtx = CFG_ATT_TRANS_RTX,
#if (BLE_PERIPHERAL || BLE_CENTRAL)
    .smpc_rep_att_timer_def_val = CFG_SMPC_REP_ATTEMPTS_TIMER_DEF_VAL,
    .smpc_rep_att_timer_max_val = CFG_SMPC_REP_ATTEMPTS_TIMER_MAX_VAL,
    .smpc_rep_att_timer_mult = CFG_SMPC_REP_ATTEMPTS_TIMER_MULT,
    .smpc_timout_timer_duration = CFG_SMPC_TIMEOUT_TIMER_DURATION,
#if defined(CFG_PER_TEST)
    .app_pkt_statistic = APP_PktStatistic,
#else
    .app_pkt_statistic = NULL,
#endif
#endif
};

const struct em_offset_cfg em_offset_config = {
    .em_ble_wpb_offset = APP_EM_BLE_WPB_OFFSET,
    .em_ble_wpv_offset = APP_EM_BLE_WPV_OFFSET,
    .em_ble_ral_offset = APP_EM_BLE_RAL_OFFSET,
    .em_ble_tx_desc_offset = APP_EM_BLE_TX_DESC_OFFSET,
    .em_ble_rx_desc_offset = APP_EM_BLE_RX_DESC_OFFSET,
    .em_ble_tx_buffer_ctrl_offset = APP_EM_BLE_TX_BUFFER_CNTL_OFFSET,
    .em_ble_tx_buffer_data_offset = APP_EM_BLE_TX_BUFFER_DATA_OFFSET,
    .em_ble_rx_buffer_offset = APP_EM_BLE_RX_BUFFER_OFFSET,
    .em_ble_tx_buf_data_cnt = CFG_BLE_TX_BUFF_DATA_CNT,
    .em_ble_rx_buffer_size = CFG_REG_BLE_EM_RX_BUFFER_SIZE,
    .em_ble_rx_buffer_cnt = CFG_BLE_RX_BUFFER_CNT,
};

uint32_t APP_GetRandNumber(void)
{
    uint32_t rand;

    POWER_EnableADC(true);
    RNG_Init(RNG);
    RNG_Enable(RNG, true);

    RNG_GetRandomData(RNG, (uint8_t *)&rand, 4);

    RNG_Enable(RNG, false);
    RNG_Deinit(RNG);
    POWER_EnableADC(false);

    return rand;
}

/*! @brief Initialize BLE configuration. */
void APP_InitBleCfg(struct ble_config_st *cfg)
{
    cfg->fw = (struct fw_static_cfg *)&fw_configuration;
    cfg->app = (struct app_static_cfg *)&app_configuration;
    cfg->em = (struct em_offset_cfg *)&em_offset_config;

#if (defined(BOARD_XTAL1_CLK_HZ) && (BOARD_XTAL1_CLK_HZ == CLK_XTAL_32KHZ))
    cfg->sw_32k_calib_enable = false;
#else
    cfg->sw_32k_calib_enable = true;
#endif

    cfg->ppm_32k = -0x6000;

    /* Flag */
    CFG_API_FLAG_SET(FAST_CORRECT);

    /* Interrupt mask in GLOBAL_INT_DISABLE() */
    cfg->int_mask0 = 0xFFFFFFFF;
    cfg->int_mask1 = 0xFFFFFFFF;

    cfg->fw_seed = APP_GetRandNumber();
    srand(cfg->fw_seed);

    /* Config BLE's high frequency clock to 8MHz */
    CLOCK_AttachClk(k8M_to_BLE_CLK);
}

void BLE_Init(void)
{
    APP_InitBleCfg(&ble_config);

    /* Initialize unloaded RAM area */
    unloaded_area_init();

    /* Initialize the exchange memory interface */
    SYSCON->SRAM_CTRL = (SYSCON->SRAM_CTRL & 0xffff8000) | (((int)rwip_exchange_memory & 0x1FFFF) >> 2);

    REG_BLE_WR(0x20024040, 0x02010025);
    REG_BLE_WR(0x20024044, 0x06050403);
    REG_BLE_WR(0x20024048, 0x0A090807);
    REG_BLE_WR(0x2002404C, 0x0D0C0B26);
    REG_BLE_WR(0x20024050, 0x11100F0E);
    REG_BLE_WR(0x20024054, 0x15141312);
    REG_BLE_WR(0x20024058, 0x19181716);
    REG_BLE_WR(0x2002405C, 0x1D1C1B1A);
    REG_BLE_WR(0x20024060, 0x21201F1E);
    REG_BLE_WR(0x20024064, 0x27242322);

    /* BLE radio */
    REG_BLE_WR(BLE_RADIOCNTL0_ADDR, 0x00000000);
    REG_BLE_WR(BLE_RADIOCNTL1_ADDR, 0x00020000);
    REG_BLE_WR(BLE_RADIOPWRUPDN_ADDR, 0x39660324);

    /* to improve the PRE performance drop at -73dBm issue */
    AGC->CTRL1 = (AGC->CTRL1 & ~AGC_CTRL1_PD3_TH_REG_MASK) | AGC_CTRL1_PD3_TH_REG(4U);

    ble_radio_sleep_en_setf(1);
    ble_osc_sleep_en_setf(1);

    /* Initialize BLE stack */
    rwip_init(error);

    enable_ble_sleep(true);

    /* Enable BLE Interrupt */
    NVIC_EnableIRQ(BLE_IRQn);
}

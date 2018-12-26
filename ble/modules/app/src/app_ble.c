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
#include "clock_config.h"
#include "app_ble.h"
#include "app_ble_task.h"
#include "fsl_power.h"
#include "fsl_rtc.h"
#include "rco32k_calibration.h"
#if (BLE_APP_PRESENT)
#include "led.h"
#include "button.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
extern void llm_util_set_public_addr(struct bd_addr *bd_addr);
extern void llm_util_set_rand_addr(struct bd_addr *bd_addr);
extern void gapm_addr_set(uint8_t addr_type, struct bd_addr *bd_addr);
typedef void (*app_add_prf_func_t)(void);

/*******************************************************************************
 * Variables
 ******************************************************************************/
/*! @brief Application environment variable */
struct app_env_tag g_AppEnv;

/*! @brief Application pairing parameters */
const struct gapc_pairing s_appPairingParameters = {
    .iocap = CFG_IO_CAPABILITY,
    .oob = GAP_OOB_AUTH_DATA_NOT_PRESENT,
    .auth = CFG_AUTH_REQ,
    .key_size = CFG_LTK_LEN,
    .ikey_dist = GAP_KDIST_IDKEY | GAP_KDIST_SIGNKEY | GAP_KDIST_ENCKEY,
    .rkey_dist = GAP_KDIST_IDKEY | GAP_KDIST_SIGNKEY | GAP_KDIST_ENCKEY,
    .sec_req = CFG_SEC_MODE_LEVEL,
};

#if (BLE_PROFILES)

#ifdef MBED_PORT
static const app_add_prf_func_t s_appAddPrfFuncList[1] = {NULL};
#else

/*! @brief List of functions used to create the database */
static const app_add_prf_func_t s_appAddPrfFuncList[BLE_NB_PROFILES] = {
/* Server */
#if (BLE_DIS_SERVER)
    (app_add_prf_func_t)APP_DissAddProfileTask,
#endif
#if (BLE_BATT_SERVER)
    (app_add_prf_func_t)APP_BassAddProfileTask,
#endif
#if (BLE_QPP_SERVER)
    (app_add_prf_func_t)APP_QppsAddProfileTask,
#endif
#if (BLE_YELL_SERVER)
    (app_add_prf_func_t)APP_YellsAddProfileTask,
#endif
#if (BLE_GL_SENSOR)
    (app_add_prf_func_t)APP_GlpsAddProfileTask,
#endif
#if (BLE_FINDME_TARGET)
    (app_add_prf_func_t)APP_FindtAddProfileTask,
#endif
#if (BLE_PROX_REPORTER)
    (app_add_prf_func_t)APP_ProxrAddProfileTask,
#endif
#if (BLE_BP_SENSOR)
    (app_add_prf_func_t)APP_BlpsAddProfileTask,
#endif
#if (BLE_HT_THERMOM)
    (app_add_prf_func_t)APP_HtptAddProfileTask,
#endif
#if (BLE_CSC_SENSOR)
    (app_add_prf_func_t)APP_CscpsAddProfileTask,
#endif
#if (BLE_CP_SENSOR)
    (app_add_prf_func_t)APP_CppsAddProfileTask,
#endif
#if (BLE_HR_SENSOR)
    (app_add_prf_func_t)APP_HrpsAddProfileTask,
#endif
#if (BLE_LN_SENSOR)
    (app_add_prf_func_t)APP_LansAddProfileTask,
#endif
#if (BLE_SP_SERVER)
    (app_add_prf_func_t)APP_ScppsAddProfileTask,
#endif
#if (BLE_HID_DEVICE)
    (app_add_prf_func_t)APP_HogpdAddProfileTask,
#endif
#if (BLE_RSC_SENSOR)
    (app_add_prf_func_t)APP_RscpsAddProfileTask,
#endif
#if (BLE_PAS_SERVER)
    (app_add_prf_func_t)APP_PaspsAddProfileTask,
#endif
#if (BLE_AN_SERVER)
    (app_add_prf_func_t)APP_AnpsAddProfileTask,
#endif
#if (BLE_TIP_SERVER)
    (app_add_prf_func_t)APP_TipsAddProfileTask,
#endif

/* Client */
#if (BLE_DIS_CLIENT)
    (app_add_prf_func_t)APP_DiscAddProfileTask,
#endif
#if (BLE_BATT_CLIENT)
    (app_add_prf_func_t)APP_BascAddProfileTask,
#endif

#if (BLE_FINDME_LOCATOR)
    (app_add_prf_func_t)APP_FindlAddProfileTask,
#endif

#if (BLE_QPP_CLIENT)
    (app_add_prf_func_t)APP_QppcAddProfileTask,
#endif

#if (BLE_PROX_MONITOR)
    (app_add_prf_func_t)APP_ProxmAddProfileTask,
#endif

#if (BLE_GL_COLLECTOR)
    (app_add_prf_func_t)APP_GlpcAddProfileTask,
#endif

#if (BLE_HT_COLLECTOR)
    (app_add_prf_func_t)APP_HtpcAddProfileTask,
#endif

#if (BLE_CP_COLLECTOR)
    (app_add_prf_func_t)APP_CppcAddProfileTask,
#endif

#if (BLE_CSC_COLLECTOR)
    (app_add_prf_func_t)APP_CscpcAddProfileTask,
#endif

#if (BLE_LN_COLLECTOR)
    (app_add_prf_func_t)APP_LancAddProfileTask,
#endif

#if (BLE_SP_CLIENT)
    (app_add_prf_func_t)APP_ScppcAddProfileTask,
#endif

#if (BLE_PAS_CLIENT)
    (app_add_prf_func_t)APP_PaspcAddProfileTask,
#endif

#if (BLE_HID_BOOT_HOST)
    (app_add_prf_func_t)APP_HogpbhAddProfileTask,
#endif

#if (BLE_HID_REPORT_HOST)
    (app_add_prf_func_t)APP_HogprhAddProfileTask,
#endif
#if (BLE_AN_CLIENT)
    (app_add_prf_func_t)APP_AnpcAddProfileTask,
#endif
#if (BLE_TIP_CLIENT)
    (app_add_prf_func_t)APP_TipcAddProfileTask,
#endif
#if (BLE_HR_COLLECTOR)
    (app_add_prf_func_t)APP_HrpcAddProfileTask,
#endif
#if (BLE_BP_COLLECTOR)
    (app_add_prf_func_t)APP_BlpcAddProfileTask,
#endif
#if (BLE_RSC_COLLECTOR)
    (app_add_prf_func_t)APP_RscpcAddProfileTask,
#endif
#if (BLE_OTA_CLIENT)
    (app_add_prf_func_t)APP_OtacAddProfileTask,
#endif
#if (BLE_OTA_SERVER)
    (app_add_prf_func_t)APP_OtasAddProfileTask,
#endif
};
#endif
#endif

/*******************************************************************************
 * Code
 ******************************************************************************/
/*!
 * @brief Initialize the SMP environment variable.
 * @return None
 */
static void APP_InitSmp(void)
{
    APP_SecGenKey(g_AppEnv.irk.key, GAP_KEY_LEN);
}

/*!
 * @brief Initialize the application environment variable.
 * @return None
 */
static void APP_InitEnv(void)
{
    /* Reset the environment */
    memset(&g_AppEnv, 0, sizeof(g_AppEnv));

    for (uint8_t idx = 0; idx < BLE_CONNECTION_MAX; idx++)
    {
        g_AppEnv.conn_dev_list[idx].free = true;
    }

    for (uint8_t idx = 0; idx < CFG_BOND_DEV_MAX; idx++)
    {
        g_AppEnv.bond_dev_list[idx].free = true;
    }

#if (BLE_PERIPHERAL || BLE_BROADCASTER)
    APP_ConstructAdvData(CFG_ADV_DATA, CFG_ADV_DATA_LEN, CFG_SCAN_RSP_DATA, CFG_SCAN_RSP_DATA_LEN);
#endif

    APP_InitSmp();
}

void APP_Init(void)
{
    if (CFG_API_FLAG_GET(HCI))
    {
        return;
    }

    APP_InitEnv();

    /* Create APP task */
    ke_task_create(TASK_APP, &ble_config.app->ble_task_desc[TASK_APP]);

    /* Initialize task state */
    ke_state_set(TASK_APP, APP_STATE_INIT);

    /* Register APP message handler */
    APP_RegisterMessageHandler(APP_MSG_BUTTON_DOWN, (ke_msg_func_t)APP_ButtonDownHandler);

#if defined(CFG_DEMO_MENU)
    APP_MenuUartInit();
    APP_RegisterMessageHandler(APP_MSG_MENU_UART_DATA_IND, (ke_msg_func_t)APP_MenuUartDataIndHandler);
#endif
}

bool APP_AddPrfTask(void)
{
    /* Indicate if more profiles need to be added in the database */
    bool more_prf = false;

#if (BLE_PROFILES)
    /* Check if another should be added in the database */
    if (g_AppEnv.next_prf != BLE_NB_PROFILES)
    {
        ASSERT_ERR(s_appAddPrfFuncList[g_AppEnv.next_prf] != NULL);
        /* Call the function used to add the required profile */
        s_appAddPrfFuncList[g_AppEnv.next_prf]();
        /* Select the next profile to add */
        g_AppEnv.next_prf++;
        more_prf = true;
    }
    else
#endif
    {
        ke_state_set(TASK_APP, APP_STATE_IDLE);

#if defined(CFG_DEMO_MENU)
        APP_MenuShowStart();
#endif

#if (defined(BOARD_XTAL1_CLK_HZ) && (BOARD_XTAL1_CLK_HZ == CLK_XTAL_32KHZ))
        RTC_Calibration(RTC, kRTC_BackwardCalibration, 0x6000);
#else
        RCO32K_InitSwCalib(1000);
#endif
        APP_BleReadyCallback();
    }

    return more_prf;
}

#if (BLE_PERIPHERAL || BLE_BROADCASTER)
#ifdef MBED_PORT
void APP_ConstructAdvData(uint8_t const *adv_data,
                          uint8_t adv_data_len,
                          uint8_t const *scan_rsp_data,
                          uint8_t scan_rsp_data_len)
#else
void APP_ConstructAdvData(uint8_t *adv_data, uint8_t adv_data_len, uint8_t *scan_rsp_data, uint8_t scan_rsp_data_len)
#endif
{
    ASSERT_ERR(adv_data_len <= (GAP_ADV_DATA_LEN - 3));
    ASSERT_ERR(scan_rsp_data_len <= GAP_SCAN_RSP_DATA_LEN);

    /* Save the data provided by user */
    memcpy(g_AppEnv.adv_data, adv_data, adv_data_len);
    memcpy(g_AppEnv.scan_rsp_data, scan_rsp_data, scan_rsp_data_len);
    g_AppEnv.adv_data_len = adv_data_len;
    g_AppEnv.scan_rsp_data_len = scan_rsp_data_len;

    /* Get device name from NVDS */
    uint8_t dev_name[NVDS_LEN_DEVICE_NAME];
    nvds_tag_len_t dev_name_len = NVDS_LEN_DEVICE_NAME;

    if (nvds_get(NVDS_TAG_DEVICE_NAME, &dev_name_len, dev_name) != NVDS_OK)
    {
        dev_name_len = sizeof(GAP_DEV_NAME);
        memcpy(dev_name, GAP_DEV_NAME, dev_name_len);
    }

    /* Get available space in adv and scan response */
    uint8_t adv_avail_len = ADV_DATA_LEN - 3 - adv_data_len;
    uint8_t scan_avail_len = SCAN_RSP_DATA_LEN - scan_rsp_data_len;

    /* Try to put device name in the advertising packet or scan response packet */
    if (dev_name_len + 2 <= adv_avail_len) /* there is enough space to save complete local name in adv */
    {
        g_AppEnv.adv_data[adv_data_len] = dev_name_len + 1;
        g_AppEnv.adv_data[adv_data_len + 1] = GAP_AD_TYPE_COMPLETE_NAME;
        memcpy(&g_AppEnv.adv_data[adv_data_len + 2], dev_name, dev_name_len);
        g_AppEnv.adv_data_len += (dev_name_len + 2);
    }
    else if (dev_name_len + 2 <= scan_avail_len) /* there is enough space to save complete local name in scan_rsp */
    {
        g_AppEnv.scan_rsp_data[scan_rsp_data_len] = dev_name_len + 1;
        g_AppEnv.scan_rsp_data[scan_rsp_data_len + 1] = GAP_AD_TYPE_COMPLETE_NAME;
        memcpy(&g_AppEnv.scan_rsp_data[scan_rsp_data_len + 2], dev_name, dev_name_len);
        g_AppEnv.scan_rsp_data_len += (dev_name_len + 2);
    }
    else if (adv_avail_len >= 3) /* there is some space to save shortened local name in adv */
    {
        g_AppEnv.adv_data[adv_data_len] = adv_avail_len - 1;
        g_AppEnv.adv_data[adv_data_len + 1] = GAP_AD_TYPE_SHORTENED_NAME;
        memcpy(&g_AppEnv.adv_data[adv_data_len + 2], dev_name, adv_avail_len - 2);
        g_AppEnv.adv_data_len = ADV_DATA_LEN - 3;
    }
    else if (scan_avail_len >= 3) /* there is some space to save shortened local name in scan_rsp */
    {
        g_AppEnv.scan_rsp_data[scan_rsp_data_len] = scan_avail_len - 1;
        g_AppEnv.scan_rsp_data[scan_rsp_data_len + 1] = GAP_AD_TYPE_SHORTENED_NAME;
        memcpy(&g_AppEnv.scan_rsp_data[scan_rsp_data_len + 2], dev_name, scan_avail_len - 2);
        g_AppEnv.scan_rsp_data_len = SCAN_RSP_DATA_LEN;
    }
}
#endif

void APP_SetBDAddress(uint8_t addr_type, struct bd_addr *addr)
{
    ASSERT_ERR(addr_type == GAPM_CFG_ADDR_PUBLIC || addr_type == GAPM_CFG_ADDR_PRIVATE);
    ASSERT_ERR(addr != NULL);

    if (addr_type == GAPM_CFG_ADDR_PUBLIC)
    {
        llm_util_set_public_addr(addr);
    }
    else if (addr_type == GAPM_CFG_ADDR_PRIVATE)
    {
        llm_util_set_rand_addr(addr);
    }

    gapm_addr_set(addr_type, addr);
}

uint8_t APP_GetLinkNb(void)
{
    return g_AppEnv.conn_count;
}

bool APP_GetLinkStatusByConhdl(uint16_t conhdl)
{
    bool r = false;
    if (conhdl < BLE_CONNECTION_MAX)
    {
        r = !g_AppEnv.conn_dev_list[conhdl].free;
    }
    return r;
}

bool APP_GetPeerBdAddrByConhdl(uint16_t conhdl, bd_addr_t *addr, uint8_t *addr_type)
{
    bool r = false;

    if (addr && (g_AppEnv.conn_dev_list[conhdl].free == false))
    {
        *addr = g_AppEnv.conn_dev_list[conhdl].conn_info.peer_addr;
        *addr_type = g_AppEnv.conn_dev_list[conhdl].conn_info.peer_addr_type;
        r = true;
    }

    return r;
}

void APP_SetLinkInfoByConhdl(uint16_t conhdl, struct gapc_connection_req_ind const *conn_info, bool conn)
{
    /* The range of valid connection index is from 0 to BLE_CONNECTION_MAX, so it could be used as array index directly
     */
    if (conn == true)
    {
        g_AppEnv.conn_dev_list[conhdl].free = false;
        g_AppEnv.conn_dev_list[conhdl].conn_info.bond_index = APP_INVALID_IDX;
        g_AppEnv.conn_dev_list[conhdl].conn_info.peer_addr_type = conn_info->peer_addr_type;
        g_AppEnv.conn_dev_list[conhdl].conn_info.peer_addr = conn_info->peer_addr;
        g_AppEnv.conn_count++;
    }
    else
    {
        g_AppEnv.conn_dev_list[conhdl].free = true;
        g_AppEnv.conn_count--;
    }
}

uint8_t APP_FindBondedDevByAddr(bd_addr_t const *addr, uint8_t addr_type)
{
    for (int i = 0; i < CFG_BOND_DEV_MAX; i++)
    {
        if ((g_AppEnv.bond_dev_list[i].free == false) &&
            co_bdaddr_compare((struct bd_addr *)addr,
                              (struct bd_addr *)&g_AppEnv.bond_dev_list[i].bond_info.peer_addr) &&
            (g_AppEnv.bond_dev_list[i].bond_info.peer_addr_type == addr_type))
        {
            return i;
        }
    }

    return APP_INVALID_IDX;
}

uint8_t APP_FindBondedDevByLocalKey(uint16_t conhdl, struct app_pairing_keys_info *key, uint8_t key_type)
{
    switch (key_type)
    {
        case GAP_KDIST_ENCKEY: /* find LTK by rand number and ediv */
            for (int i = 0; i < CFG_BOND_DEV_MAX; i++)
            {
                if ((g_AppEnv.bond_dev_list[i].free == false) &&
                    !memcmp(&key->ltk.randnb, &g_AppEnv.bond_dev_list[i].bond_info.local_keys.ltk.randnb,
                            RAND_NB_LEN) &&
                    (key->ltk.ediv == g_AppEnv.bond_dev_list[i].bond_info.local_keys.ltk.ediv) &&
                    g_AppEnv.conn_dev_list[conhdl].conn_info.bond_index == i)
                {
                    return i;
                }
            }
            break;

        case GAP_KDIST_IDKEY: /* find bonded index by IRK */
            for (int i = 0; i < CFG_BOND_DEV_MAX; i++)
            {
                if ((g_AppEnv.bond_dev_list[i].free == false) &&
                    !memcmp(&key->irk, &g_AppEnv.bond_dev_list[i].bond_info.local_keys.irk.irk, GAP_KEY_LEN))
                {
                    return i;
                }
            }
            break;

        case GAP_KDIST_SIGNKEY:
            break;

        default:
            break;
    }

    return APP_INVALID_IDX;
}

uint8_t APP_FindBondedDevByPeerKey(struct app_pairing_keys_info *key, uint8_t key_type)
{
    switch (key_type)
    {
        case GAP_KDIST_ENCKEY: /* find LTK by rand number and ediv */
            for (int i = 0; i < CFG_BOND_DEV_MAX; i++)
            {
                if ((g_AppEnv.bond_dev_list[i].free == false) &&
                    !memcmp(&key->ltk.randnb, &g_AppEnv.bond_dev_list[i].bond_info.peer_keys.ltk.randnb, RAND_NB_LEN) &&
                    (key->ltk.ediv == g_AppEnv.bond_dev_list[i].bond_info.peer_keys.ltk.ediv))
                {
                    return i;
                }
            }
            break;

        case GAP_KDIST_IDKEY: /* find bonded index by IRK */
            for (int i = 0; i < CFG_BOND_DEV_MAX; i++)
            {
                if ((g_AppEnv.bond_dev_list[i].free == false) &&
                    !memcmp(&key->irk, &g_AppEnv.bond_dev_list[i].bond_info.peer_keys.irk.irk, GAP_KEY_LEN))
                {
                    return i;
                }
            }
            break;

        case GAP_KDIST_SIGNKEY:
            break;

        default:
            break;
    }

    return APP_INVALID_IDX;
}

uint8_t APP_AddBondedDev(struct app_conn_info *bonded_dev)
{
    uint8_t idx = APP_FindBondedDevByAddr(&bonded_dev->peer_addr, bonded_dev->peer_addr_type);

    if (APP_INVALID_IDX != idx)
    { /* replace */
        g_AppEnv.bond_dev_list[idx].bond_info.local_keys = bonded_dev->local_keys;
        g_AppEnv.bond_dev_list[idx].bond_info.peer_keys = bonded_dev->peer_keys;
        g_AppEnv.bond_dev_list[idx].bond_info.peer_addr_type = bonded_dev->peer_addr_type;
        g_AppEnv.bond_dev_list[idx].bond_info.peer_addr = bonded_dev->peer_addr;
        return idx;
    }
    else if (g_AppEnv.bond_count < CFG_BOND_DEV_MAX) /* add */
    {
        for (int i = 0; i < CFG_BOND_DEV_MAX; i++)
        {
            if (g_AppEnv.bond_dev_list[i].free == true)
            {
                g_AppEnv.bond_dev_list[i].bond_info.local_keys = bonded_dev->local_keys;
                g_AppEnv.bond_dev_list[i].bond_info.peer_keys = bonded_dev->peer_keys;
                g_AppEnv.bond_dev_list[i].bond_info.peer_addr_type = bonded_dev->peer_addr_type;
                g_AppEnv.bond_dev_list[i].bond_info.peer_addr = bonded_dev->peer_addr;
                g_AppEnv.bond_dev_list[i].free = false;
                g_AppEnv.bond_count++;
                return i;
            }
        }
    }

    return APP_INVALID_IDX;
}

bool APP_RemoveBondedDev(uint8_t idx)
{
    bool r = false;
    if (idx < CFG_BOND_DEV_MAX)
    {
        if (g_AppEnv.bond_dev_list[idx].free == false)
        {
            g_AppEnv.bond_dev_list[idx].free = true;
            memset((uint8_t *)&g_AppEnv.bond_dev_list[idx].bond_info, 0, sizeof(struct app_bond_info));
            g_AppEnv.bond_count--;
            r = true;
        }
    }
    return r;
}

void APP_ClearBondInf(void)
{
    g_AppEnv.bond_count = 0;
    memset((uint8_t *)&g_AppEnv.bond_dev_list, 0, sizeof(g_AppEnv.bond_dev_list));
}

uint32_t APP_SecGenTk(void)
{
    /* Generate a PIN Code (Between 100000 and 999999) */
    return (100000 + (rand() % 900000));
}

void APP_SecGenLtk(uint16_t conhdl, uint8_t key_size)
{
    /* Counter */
    uint8_t i;
    g_AppEnv.conn_dev_list[conhdl].conn_info.local_keys.ltk.key_size = key_size;

    /* Randomly generate the LTK and the Random Number */
    for (i = 0; i < RAND_NB_LEN; i++)
    {
        g_AppEnv.conn_dev_list[conhdl].conn_info.local_keys.ltk.randnb.nb[i] = rand() % 256;
    }

    /* Randomly generate the end of the LTK */
    for (i = 0; i < GAP_KEY_LEN; i++)
    {
        g_AppEnv.conn_dev_list[conhdl].conn_info.local_keys.ltk.ltk.key[i] = ((i < key_size) ? rand() % 256 : 0);
    }

    /* Randomly generate the EDIV */
    g_AppEnv.conn_dev_list[conhdl].conn_info.local_keys.ltk.ediv = rand() % 65536;
}

void APP_SecGenKey(uint8_t *key, uint8_t key_size)
{
    /* Randomly generate the IRK or CSRK */
    for (uint8_t i = 0; i < GAP_KEY_LEN; i++)
    {
        key[i] = ((i < key_size) ? rand() % 256 : 0);
    }
}

void APP_MsgSend(void *cmd)
{
    if (!APP_BleIsActive())
    {
        /* To wakeup BLE module */
        APP_BleSoftwareWakeup();
    }
    ke_msg_send(cmd);
}

void APP_RegisterMessageHandler(ke_msg_id_t id, ke_msg_func_t func)
{
    ASSERT_ERR((id > APP_MSG_DEFAULT) && (id < APP_MSG_MAX));
    g_AppDefaultState[id - APP_MSG_DEFAULT].id = id;
    g_AppDefaultState[id - APP_MSG_DEFAULT].func = func;
}

void APP_TimerCfg(ke_msg_id_t const timer_msg_id, uint32_t delay, ke_msg_func_t func)
{
    ASSERT_ERR((timer_msg_id > APP_MSG_DEFAULT) && (timer_msg_id < APP_MSG_MAX));
    /* set or clear timer message callback function */
    APP_RegisterMessageHandler(timer_msg_id, func);

    if (APP_BleIsActive() && !__get_IPSR())
    {
        if (delay)
        {
            ke_timer_set(timer_msg_id, TASK_APP, delay);
        }
        else
        {
            ke_timer_clear(timer_msg_id, TASK_APP);
        }
    }
    else
    {
        /* send a message to set the timer */
        struct app_timer_cfg_cmd *cmd = KE_MSG_ALLOC(APP_MSG_TIMER_CFG, TASK_APP, TASK_APP, app_timer_cfg_cmd);

        cmd->timer_id = timer_msg_id;
        cmd->duration = delay;
        APP_MsgSend(cmd);
    }
}

#if defined(CFG_PER_TEST)
#include "lld_evt.h"
#include "reg_ble_em_rx_desc.h"
#include <stdio.h>
#include "app_config.h" /* to refer to QPRINTF */

struct per_info_t
{
    /*! @brief to indication if this structure is in use */
    bool in_use;
    uint32_t total_pkt;
    uint32_t total_err;
    uint32_t sync_err_pkt;
    uint32_t type_err_pkt;
    uint32_t len_err_pkt;
    uint32_t crc_err_pkt;
    uint32_t mic_err_pkt;
    uint32_t sn_err_pkt;
    uint32_t nesn_err_pkt;
};

struct per_info_t s_perInfo[BLE_CONNECTION_MAX] = {0};

void APP_PktStatistic(uint16_t rx_st, uint16_t conhdl)
{
    struct per_info_t *info;

    if (true == s_perInfo[conhdl].in_use)
    {
        info = &(s_perInfo[conhdl]);

        info->total_pkt++;

        if (rx_st & 0x1f)
            info->total_err++;

        if (rx_st & BLE_SYNC_ERR_BIT)
            info->sync_err_pkt++;

        if (rx_st & BLE_TYPE_ERR_BIT)
            info->type_err_pkt++;

        if (rx_st & BLE_LEN_ERR_BIT)
            info->len_err_pkt++;

        if (rx_st & BLE_CRC_ERR_BIT)
            info->crc_err_pkt++;

        if (rx_st & BLE_MIC_ERR_BIT)
            info->mic_err_pkt++;

        if (rx_st & BLE_SN_ERR_BIT)
            info->sn_err_pkt++;

        if (rx_st & BLE_NESN_ERR_BIT)
            info->nesn_err_pkt++;
    }
}

void APP_StartStatistic(uint16_t conhdl)
{
    struct per_info_t *info;

    APP_TIMER_SET(APP_MSG_PER_TEST_TIMER, 500, APP_PerTestTimerHandler);

    if (false == s_perInfo[conhdl].in_use)
    {
        info = &(s_perInfo[conhdl]);
        info->in_use = true;
        info->total_pkt = 0U;
        info->total_err = 0U;
        info->sync_err_pkt = 0U;
        info->type_err_pkt = 0U;
        info->len_err_pkt = 0U;
        info->crc_err_pkt = 0U;
        info->mic_err_pkt = 0U;
        info->sn_err_pkt = 0U;
        info->nesn_err_pkt = 0U;
    }
}

void APP_StopStatistic(uint16_t conhdl)
{
    s_perInfo[conhdl].in_use = false;

    for (uint8_t i = 0U; i < BLE_CONNECTION_MAX; i++)
    {
        if (s_perInfo[i].in_use == true)
            break;
        if (i == BLE_CONNECTION_MAX)
            APP_TIMER_CLR(APP_MSG_PER_TEST_TIMER);
    }
}

void APP_ShowStatistic(void)
{
    struct per_info_t *info;
    uint8_t i;

    for (i = 0; i < BLE_CONNECTION_MAX; i++)
    {
        if (s_perInfo[i].in_use == true)
        {
            info = &(s_perInfo[i]);
            QPRINTF("link_num: %d, total_pkt: %d, err_pkt: %d\r\n", i, info->total_pkt, info->total_err);
#if 0
            QPRINTF("    err_detail: %d, %d, %d, %d, %d. sn&nesn: %d, %d\r\n",
                    info->sync_err_pkt, info->type_err_pkt, info->len_err_pkt,  info->crc_err_pkt,
                    info->mic_err_pkt,  info->sn_err_pkt,   info->nesn_err_pkt);
#endif
        }
    }
}

#endif

#endif /* BLE_APP_PRESENT */

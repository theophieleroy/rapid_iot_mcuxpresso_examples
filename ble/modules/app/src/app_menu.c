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
#include "app_ble.h"
#if defined(CFG_DEMO_MENU)
#include "ke_mem.h"
#include "ke_msg.h"
#include "uart_adapter.h"

#if BLE_OTA_SERVER
#include "otap_support.h"
#include "otap_server.h"
#include "otas.h"

#define OTA_NEW_IMG_SECOTR 125

#endif
/*******************************************************************************
 * Variables
 ******************************************************************************/
struct app_menu_uart_env_tag s_appMenuUartEnv;

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static void APP_MenuShow(void);
static void APP_MenuHandler(void);

#if (BLE_QPP_CLIENT)
void APP_SendTestData(uint16_t conhdl);
#endif

/*******************************************************************************
 * Code
 ******************************************************************************/
/*!
 * @brief UART receive done handler.
 */
static void APP_MenuUartRxDone(void)
{
    if ((s_appMenuUartEnv.len > 0) && (s_appMenuUartEnv.buf_rx[s_appMenuUartEnv.len - 1] == 0x0D) &&
        (s_appMenuUartEnv.buf_rx[s_appMenuUartEnv.len] == 0x0A)) /* check \r\n */
    {
        if (s_appMenuUartEnv.len > 1)
        {
            struct app_menu_uart_data_req *req = KE_MSG_ALLOC_DYN(APP_MSG_MENU_UART_DATA_IND, TASK_APP, TASK_APP,
                                                                  app_menu_uart_data_req, s_appMenuUartEnv.len - 1);

            req->len = s_appMenuUartEnv.len - 1;
            memcpy(req->data, s_appMenuUartEnv.buf_rx, req->len);
            APP_MsgSend(req);
        }
        else
        {
            /* ignore null enter */
        }
        s_appMenuUartEnv.len = 0;
    }
    else
    {
        s_appMenuUartEnv.len++;
        if (s_appMenuUartEnv.len == QN_UART_RX_LEN)
        {
            s_appMenuUartEnv.len = 0;
        }
    }

    ADAPTER_UartReceiveInt(s_appMenuUartEnv.buf_rx + s_appMenuUartEnv.len, 1, APP_MenuUartRxDone);
}

void APP_MenuUartInit(void)
{
    ADAPTER_UartInit();

    s_appMenuUartEnv.len = 0;
    ADAPTER_UartReceiveInt(s_appMenuUartEnv.buf_rx, 1, APP_MenuUartRxDone);
}

int APP_MenuUartDataIndHandler(ke_msg_id_t const msgid,
                               const void *param,
                               ke_task_id_t const dest_id,
                               ke_task_id_t const src_id)
{
    struct app_menu_uart_data_req *uart_data = (struct app_menu_uart_data_req *)param;

    if (uart_data->len != 0)
    {
        memcpy(g_AppEnv.input, uart_data->data, uart_data->len);
        APP_MenuHandler();
    }

    return (KE_MSG_CONSUMED);
}

static inline void APP_MenuShowMenu(const char *const menu[], uint8_t size)
{
    QPRINTF("*----------------------------------------\r\n");

    for (uint32_t i = 0; i < size; i++)
    {
        QPRINTF(menu[i]);
        QPRINTF("\r\n");
    }

    QPRINTF("*----------------------------------------\r\n");
}

const char *const s_appMenuMain[] = {
    "* BLE Main Menu", "* 1. GAP   Menu", "* 2. GATT  Menu",
/* "* 3. L2CC  Menu" reserved for L2CAP */
#if BLE_QPP_CLIENT
    "* 4. QPPC  Menu",
#endif
#if BLE_DIS_CLIENT
    "* 5. DISC  Menu",
#endif
#if BLE_BATT_CLIENT
    "* 6. BASC  Menu",
#endif
#if BLE_GL_COLLECTOR
    "* 7. GLPC  Menu",
#endif
#if BLE_PROX_MONITOR
    "* 8. PROXM Menu",
#endif
#if BLE_FINDME_LOCATOR
    "* 9. FMPL  Menu",
#endif
#if BLE_HT_COLLECTOR
    "* a. HTPC  Menu",
#endif
#if BLE_CP_COLLECTOR
    "* b. CPPC  Menu",
#endif
#if BLE_CSC_COLLECTOR
    "* c. CSCPC Menu",
#endif
#if BLE_LN_COLLECTOR
    "* d. LANC  Menu",
#endif
#if BLE_SP_CLIENT
    "* e. SCPPC Menu",
#endif
#if BLE_PAS_CLIENT
    "* f. PASPC Menu",
#endif
#if BLE_HID_REPORT_HOST
    "* g. HOGPH Menu",
#endif
#if BLE_HID_BOOT_HOST
    "* g. HOGPH Menu",
#endif
#if BLE_AN_CLIENT
    "* h. ANPC Menu",
#endif
#if BLE_TIP_CLIENT
    "* i. TIPC  Menu",
#endif
#if BLE_HR_COLLECTOR
    "* j. HRPC  Menu",
#endif
#if BLE_BP_COLLECTOR
    "* k. BLPC  Menu",
#endif
#if BLE_RSC_COLLECTOR
    "* l. RSCPC Menu",
#endif
#if BLE_OTA_SERVER
    "* m. OTAC Menu",
#endif
    "* r. Upper Menu", "* s. Show  Menu",
};

static void APP_MenuShowMain(void)
{
    APP_MenuShowMenu(s_appMenuMain, sizeof(s_appMenuMain) / 4);
}

static void APP_MenuMainHandler(void)
{
    switch (g_AppEnv.input[0])
    {
        case '1':
            g_AppEnv.menu_id = menu_gap;
            break;
        case '2':
            g_AppEnv.menu_id = menu_gatt;
            break;
        case '3':
            /* TODO: L2CC */
            break;
#if BLE_QPP_CLIENT
        case '4':
            g_AppEnv.menu_id = menu_qppc;
            break;
#endif
#if BLE_DIS_CLIENT
        case '5':
            g_AppEnv.menu_id = menu_disc;
            break;
#endif
#if BLE_BATT_CLIENT
        case '6':
            g_AppEnv.menu_id = menu_basc;
            break;
#endif
#if BLE_GL_COLLECTOR
        case '7':
            g_AppEnv.menu_id = menu_glpc;
            break;
#endif
#if BLE_PROX_MONITOR
        case '8':
            g_AppEnv.menu_id = menu_proxm;
            break;
#endif
#if BLE_FINDME_LOCATOR
        case '9':
            g_AppEnv.menu_id = menu_findl;
            break;
#endif
#if BLE_HT_COLLECTOR
        case 'a':
            g_AppEnv.menu_id = menu_htpc;
            break;
#endif

#if BLE_CP_COLLECTOR
        case 'b':
            g_AppEnv.menu_id = menu_cppc;
            break;
#endif

#if BLE_CSC_COLLECTOR
        case 'c':
            g_AppEnv.menu_id = menu_cscpc;
            break;
#endif

#if BLE_LN_COLLECTOR
        case 'd':
            g_AppEnv.menu_id = menu_lanc;
            break;
#endif

#if BLE_SP_CLIENT
        case 'e':
            g_AppEnv.menu_id = menu_scppc;
            break;
#endif
#if BLE_PAS_CLIENT
        case 'f':
            g_AppEnv.menu_id = menu_paspc;
            break;
#endif
#if BLE_HID_REPORT_HOST
        case 'g':
            g_AppEnv.menu_id = menu_hogph;
            break;
#endif
#if BLE_HID_BOOT_HOST
        case 'g':
            g_AppEnv.menu_id = menu_hogph;
            break;
#endif
#if BLE_AN_CLIENT
        case 'h':
            g_AppEnv.menu_id = menu_anpc;
            break;
#endif
#if BLE_TIP_CLIENT
        case 'i':
            g_AppEnv.menu_id = menu_tipc;
            break;
#endif
#if BLE_HR_COLLECTOR
        case 'j':
            g_AppEnv.menu_id = menu_hrpc;
            break;

#endif
#if BLE_BP_COLLECTOR
        case 'k':
            g_AppEnv.menu_id = menu_blpc;
            break;
#endif
#if BLE_RSC_COLLECTOR
        case 'l':
            g_AppEnv.menu_id = menu_rscpc;
            break;
#endif
#if BLE_OTA_SERVER
        case 'm':
            g_AppEnv.menu_id = menu_otac;
            break;
#endif
        default:
            break;
    }

    APP_MenuShow();
}

const char *const s_appMenuGap[] = {
    "* BLE GAP menu",
#if (BLE_PERIPHERAL)
    "* 1. Advertising start",
#endif
#if (BLE_CENTRAL)
    "* 2. Scanning start",
    "* 3. Initiate a new connection",
    "* 4. Terminate a connection",
    "* 5. Pairing request",
    "* 6. Encryption request",
#endif
#if (BLE_PERIPHERAL)
    "* 7. Security request",
#endif
    "* 8. Bonding remove",
    "* 9. Cancel advertising/scanning/initiating",
};

static void app_MenuShowGap(void)
{
    APP_MenuShowMenu(s_appMenuGap, sizeof(s_appMenuGap) / 4);
}

static void APP_MenuShowGapScanRecord(void)
{
    if (g_AppEnv.scan_count > 0)
    {
        QPRINTF("\r\nScanned device record:\r\n");
        for (uint8_t i = 0; i < g_AppEnv.scan_count; i++)
        {
            if (i < 16)
            {
                QPRINTF("%01x. ", i);
            }
            else
            {
                QPRINTF("%c. ", i - 10 + 'a');
            }

            QPRINTF("%02x%02x%02x%02x%02x%02x\r\n", g_AppEnv.scan_dev_list[i].addr.addr[5],
                    g_AppEnv.scan_dev_list[i].addr.addr[4], g_AppEnv.scan_dev_list[i].addr.addr[3],
                    g_AppEnv.scan_dev_list[i].addr.addr[2], g_AppEnv.scan_dev_list[i].addr.addr[1],
                    g_AppEnv.scan_dev_list[i].addr.addr[0]);
        }
        QPRINTF("Please input the device index:\r\n");

        g_AppEnv.pre_id = menu_gap;
    }
    else
    {
        QPRINTF("Scanned device record is NULL.\r\n");
        g_AppEnv.menu_id = menu_gap;
    }
}

static void APP_MenuShowGapConnRecord(uint8_t pre_id)
{
    if (APP_GetLinkNb())
    {
        QPRINTF("\r\nConnected device records (%d):\r\n", APP_GetLinkNb());
        for (uint8_t conhdl = 0; conhdl < BLE_CONNECTION_MAX; conhdl++)
        {
            if (APP_GetLinkStatusByConhdl(conhdl))
            {
                if (conhdl < 16)
                {
                    QPRINTF("%01x. ", conhdl);
                }
                else
                {
                    QPRINTF("%c. ", conhdl - 10 + 'a');
                }
                QPRINTF("%02x%02x%02x%02x%02x%02x\r\n", g_AppEnv.conn_dev_list[conhdl].conn_info.peer_addr.addr[5],
                        g_AppEnv.conn_dev_list[conhdl].conn_info.peer_addr.addr[4],
                        g_AppEnv.conn_dev_list[conhdl].conn_info.peer_addr.addr[3],
                        g_AppEnv.conn_dev_list[conhdl].conn_info.peer_addr.addr[2],
                        g_AppEnv.conn_dev_list[conhdl].conn_info.peer_addr.addr[1],
                        g_AppEnv.conn_dev_list[conhdl].conn_info.peer_addr.addr[0]);
            }
        }
        if (g_AppEnv.menu_id == menu_gap_disconnection)
        {
            QPRINTF("x. All the devices\r\n");
        }
        QPRINTF("Please input the device connection handle:\r\n");

        g_AppEnv.pre_id = pre_id;
    }
    else
    {
        QPRINTF("Connected device record is NULL.\r\n");

        g_AppEnv.menu_id = pre_id;
    }
}

static void APP_MenuShowGapBondRecord(void)
{
    uint8_t nb = g_AppEnv.bond_count;

    if (nb > 0)
    {
        QPRINTF("\r\nBonded device records (%d):\r\n", nb);
        for (uint8_t idx = 0; idx < CFG_BOND_DEV_MAX; idx++)
        {
            if (g_AppEnv.bond_dev_list[idx].free == false)
            {
                if (idx < 16)
                {
                    QPRINTF("%01x. ", idx);
                }
                else
                {
                    QPRINTF("%c. ", idx - 10 + 'a');
                }

                QPRINTF("%02x%02x%02x%02x%02x%02x\r\n", g_AppEnv.bond_dev_list[idx].bond_info.peer_addr.addr[5],
                        g_AppEnv.bond_dev_list[idx].bond_info.peer_addr.addr[4],
                        g_AppEnv.bond_dev_list[idx].bond_info.peer_addr.addr[3],
                        g_AppEnv.bond_dev_list[idx].bond_info.peer_addr.addr[2],
                        g_AppEnv.bond_dev_list[idx].bond_info.peer_addr.addr[1],
                        g_AppEnv.bond_dev_list[idx].bond_info.peer_addr.addr[0]);
            }
        }
        QPRINTF("Please input the device index:\r\n");

        g_AppEnv.pre_id = menu_gap;
    }
    else
    {
        QPRINTF("Bonded device record is NULL.\r\n");
        g_AppEnv.menu_id = menu_gap;
    }
}

static void APP_MenuGapDevSelectHandler(void)
{
    if (g_AppEnv.input[0] >= 'a')
    {
        g_AppEnv.select_idx = g_AppEnv.input[0] - 'a' + 10;
    }
    else
    {
        g_AppEnv.select_idx = g_AppEnv.input[0] - '0';
    }

    uint16_t conhdl = CONIDX2CONHDL(g_AppEnv.select_idx);

    if (g_AppEnv.menu_id == menu_gap_create_connection) /* process scan list */
    {
        if (g_AppEnv.select_idx < g_AppEnv.scan_count)
        {
            struct gap_bdaddr addr;

            addr.addr = g_AppEnv.scan_dev_list[g_AppEnv.select_idx].addr;
            addr.addr_type = g_AppEnv.scan_dev_list[g_AppEnv.select_idx].addr_type;

            APP_GapmStartConnectionCmd(GAPM_CONNECTION_DIRECT, GAPM_STATIC_ADDR, CFG_CONN_MIN_INTV, CFG_CONN_MAX_INTV,
                                       CFG_CONN_LATENCY, CFG_CONN_SUPERV_TIMEOUT, 1, &addr);
            g_AppEnv.menu_id = menu_gap;
        }
        else if (g_AppEnv.input[0] == 'r')
        {
            g_AppEnv.menu_id = menu_gap;
            APP_MenuShow();
        }
        else if (g_AppEnv.input[0] == 's')
        {
            APP_MenuShow();
        }
        else
        {
            QPRINTF("Input index is out of range.\r\n");
        }
    }
    else if (g_AppEnv.menu_id == menu_gap_unbond) /* process bond list */
    {
        if (APP_RemoveBondedDev(g_AppEnv.select_idx))
        {
            QPRINTF("Remove successful.\r\n");
            g_AppEnv.menu_id = menu_gap;
        }
        else if (g_AppEnv.input[0] == 'r')
        {
            g_AppEnv.menu_id = menu_gap;
            APP_MenuShow();
        }
        else if (g_AppEnv.input[0] == 's')
        {
            APP_MenuShow();
        }
        else
        {
            QPRINTF("Input index is not exist.\r\n");
        }
    }
    else /* process connection list */
    {
        if (APP_GetLinkStatusByConhdl(conhdl))
        {
            switch (g_AppEnv.menu_id)
            {
                case menu_gap_pair:
                    APP_GapcBondCmd(conhdl, (struct gapc_pairing *)&s_appPairingParameters);
                    break;
                case menu_gap_encrypt:
                {
                    uint8_t bonded_dev_idx = g_AppEnv.conn_dev_list[g_AppEnv.select_idx].conn_info.bond_index;
                    if ((bonded_dev_idx != APP_INVALID_IDX) &&
                        (g_AppEnv.bond_dev_list[bonded_dev_idx].bond_info.ltk_present)) /* LTK is present */
                    {
                        APP_GapcEncryptCmd(g_AppEnv.select_idx,
                                           &g_AppEnv.bond_dev_list[bonded_dev_idx].bond_info.peer_keys.ltk);
                    }
                    else
                    {
                        QPRINTF("LTK is not exist.\r\n");
                    }
                    break;
                }
                case menu_gap_security:
                    APP_GapcSecurityCmd(conhdl, s_appPairingParameters.auth);
                    break;
                case menu_gap_disconnection:
#ifdef MBED_PORT
                    APP_GapcDisconnectCmd(conhdl, CO_ERROR_REMOTE_USER_TERM_CON);
#else
                    APP_GapcDisconnectCmd(conhdl);
#endif
                    break;
                case menu_gatt_exc_mtu:
                    APP_GattcExcMtuCmd(conhdl);
                    break;
                case menu_qppc_enable:
                case menu_otac_enable:
                case menu_disc_enable:
                case menu_basc_enable:
                case menu_glpc_enable:
                case menu_proxm_enable:
                case menu_findl_enable:
                case menu_htpc_enable:
                case menu_cppc_enable:
                case menu_cscpc_enable:
                case menu_lanc_enable:
                case menu_scppc_enable:
                case menu_paspc_enable:
                case menu_hogprh_enable:
                case menu_hogpbh_enable:
                case menu_anpc_enable:
                case menu_tipc_enable:
                case menu_hrpc_enable:
                case menu_blpc_enable:
                case menu_rscpc_enable:
                    QPRINTF("Select success.\r\n");
                    break;
                default:
                    break;
            }
        }
        else if (g_AppEnv.input[0] == 'x') /* All */
        {
            if (g_AppEnv.menu_id == menu_gap_disconnection)
            {
                for (uint16_t conhdl = 0; conhdl < BLE_CONNECTION_MAX; conhdl++)
                {
                    if (APP_GetLinkStatusByConhdl(conhdl))
                    {
#ifdef MBED_PORT
                        APP_GapcDisconnectCmd(conhdl, CO_ERROR_REMOTE_USER_TERM_CON);
#else
                        APP_GapcDisconnectCmd(conhdl);
#endif
                    }
                }
            }
        }
        else if (g_AppEnv.input[0] == 'r')
        {
            APP_MenuShow();
        }
        else if (g_AppEnv.input[0] == 's')
        {
            APP_MenuShow();
        }
        else
        {
            QPRINTF("Input index is not exist.\r\n");
        }
        g_AppEnv.menu_id = g_AppEnv.pre_id;
    }
}

static void APP_MenuGapHander(void)
{
    switch (g_AppEnv.input[0])
    {
#if (BLE_PERIPHERAL)
        case '1':
        {
            APP_GapmStartAdvertiseCmd(GAPM_ADV_UNDIRECT, GAPM_STATIC_ADDR, GAP_GEN_DISCOVERABLE, CFG_ADV_INT_MIN,
                                      CFG_ADV_INT_MAX, g_AppEnv.adv_data_len, g_AppEnv.adv_data,
                                      g_AppEnv.scan_rsp_data_len, g_AppEnv.scan_rsp_data, NULL);
            break;
        }
#endif
#if (BLE_CENTRAL)
        case '2':
            APP_GapmStartScanCmd(GAPM_SCAN_ACTIVE, GAPM_STATIC_ADDR, GAP_GEN_DISCOVERY, CFG_SCAN_INTV, CFG_SCAN_WIND,
                                 SCAN_ALLOW_ADV_ALL, SCAN_FILT_DUPLIC_EN);
            break;
        case '3':
            g_AppEnv.menu_id = menu_gap_create_connection;
            APP_MenuShowGapScanRecord();
            break;
        case '4':
            g_AppEnv.menu_id = menu_gap_disconnection;
            APP_MenuShowGapConnRecord(menu_gap);
            break;
        case '5':
            g_AppEnv.menu_id = menu_gap_pair;
            APP_MenuShowGapConnRecord(menu_gap);
            break;
        case '6':
            g_AppEnv.menu_id = menu_gap_encrypt;
            APP_MenuShowGapConnRecord(menu_gap);
            break;
#endif
#if (BLE_PERIPHERAL)
        case '7':
            g_AppEnv.menu_id = menu_gap_security;
            APP_MenuShowGapConnRecord(menu_gap);
            break;
#endif
        case '8':
            g_AppEnv.menu_id = menu_gap_unbond;
            APP_MenuShowGapBondRecord();
            break;
        case '9':
            APP_GapmCancelCmd();
            break;
        case 'r':
            g_AppEnv.menu_id = menu_main;
        /* no break */
        default:
            APP_MenuShow();
            break;
    }
}

const char *const s_appMenuGatt[] = {
    "* 1. Exchange MTU",
};

static void APP_MenuShowGatt(void)
{
    APP_MenuShowMenu(s_appMenuGatt, sizeof(s_appMenuGatt) / 4);
}

static void APP_MenuGattHandler(void)
{
    switch (g_AppEnv.input[0])
    {
        case '1':
            g_AppEnv.menu_id = menu_gatt_exc_mtu;
            APP_MenuShowGapConnRecord(menu_gatt);
            break;
        case 'r':
            g_AppEnv.menu_id = menu_main;
        /* no break */
        default:
            APP_MenuShow();
            break;
    }
}

#if BLE_QPP_CLIENT
const char *const s_appMenuQppc[] = {
    "* QN BLE QPPC Menu", "* 0. Select device", "* 1. Enable",
    "* 2. Send Data",     "* 3. Stop notify",   "* 4. Start notify",
};

static void APP_MenuShowQppc(void)
{
    APP_MenuShowMenu(s_appMenuQppc, sizeof(s_appMenuQppc) / 4);
}

static void APP_MenuQppcHandler(void)
{
    uint16_t conhdl = CONIDX2CONHDL(g_AppEnv.select_idx);

    switch (g_AppEnv.input[0])
    {
        case '0':
            g_AppEnv.menu_id = menu_qppc_enable;
            APP_MenuShowGapConnRecord(menu_qppc);
            break;
        case '1':
            APP_QppcEnableReq(conhdl);
            break;
        case '2':
        {
            APP_SendTestData(conhdl);
            break;
        }
        case '3':
            APP_QppcCfgIndntfReq(conhdl, PRF_CLI_STOP_NTFIND);
            break;
        case '4':
            APP_QppcCfgIndntfReq(conhdl, PRF_CLI_START_NTF);
            break;
        case 'r':
            g_AppEnv.menu_id = menu_main;
            break;
        default:
            break;
    }

    APP_MenuShow();
}
#endif
#if BLE_OTA_SERVER
const char *const s_appMenuOtac[] = {
    "* QN BLE OTAC Menu", "* 0. Select device", "* 1. Enable", "* 2. Upgrade req",
};
static void app_menu_show_otac(void)
{
    APP_MenuShowMenu(s_appMenuOtac, sizeof(s_appMenuOtac) / 4);
}
extern void iapServerSend(uint16_t len, uint8_t *data);
static void app_menu_otac_handler(void)
{
    uint16_t conhdl = CONIDX2CONHDL(g_AppEnv.select_idx);
    otapServerInterface_t otaServerInterface;

    switch (g_AppEnv.input[0])
    {
        case '0':
            g_AppEnv.menu_id = menu_otac_enable;
            APP_MenuShowGapConnRecord(menu_otac);
            break;
        case '1':
            otaServerInterface.sendData = iapServerSend;
            otaServerInterface.imageInfoProc = OtapServer_ImageInfoProc;
            otaServerInterface.getMaxMtu = otapServerGetMtu;

            otaServerRegisterConhdl(conhdl);
            OtapServer_init(CONHDL2CONIDX(conhdl), &otaServerInterface);
            APP_OtasEnableReq(conhdl);
            OtapServer_LoadImage(CONHDL2CONIDX(conhdl),
                                 (uint32_t *)(FLASH_START_ADDRESS + FLASH_SECTOR_SIZE * OTA_NEW_IMG_SECOTR));

            break;
        case '2':
        {
            OtapServer_SendNewImageNotify(CONHDL2CONIDX(conhdl));
            break;
        }
        case 'r':
            g_AppEnv.menu_id = menu_main;
            break;
        default:
            break;
    }
    APP_MenuShow();
}
#endif

#if BLE_DIS_CLIENT
const char *const s_appMenuDisc[] = {
    "* BLE DISC Menu",
    "* 0. Select device",
    "* 1. Enable\r\n",
    "* 2. Read Manufacturer name",
    "* 3. Read Model Number",
    "* 4. Read Serial Number",
    "* 5. Read Hardware Revision",
    "* 6. Read Firmware Revision",
    "* 7. Read Software Revision",
    "* 8. Read System ID",
    "* 9. Read IEEE",
    "* a. Read PnP ID",
};

static void APP_MenuShowDisc(void)
{
    APP_MenuShowMenu(s_appMenuDisc, sizeof(s_appMenuDisc) / 4);
}

static void APP_MenuDiscHandler(void)
{
    uint16_t conhdl = CONIDX2CONHDL(g_AppEnv.select_idx);

    switch (g_AppEnv.input[0])
    {
        case '0':
            g_AppEnv.menu_id = menu_disc_enable;
            APP_MenuShowGapConnRecord(menu_disc);
            break;
        case '1':
            APP_DiscEnableReq(conhdl);
            break;
        case '2': /* DISC_MANUFACTURER_NAME_CHAR */
        case '3': /* DISC_MODEL_NB_STR_CHAR */
        case '4': /* DISC_SERIAL_NB_STR_CHAR */
        case '5': /* DISC_HARD_REV_STR_CHAR */
        case '6': /* DISC_FIRM_REV_STR_CHAR */
        case '7': /* DISC_SW_REV_STR_CHAR */
        case '8': /* DISC_SYSTEM_ID_CHAR */
        case '9': /* DISC_IEEE_CHAR */
            APP_DiscRdCharReq(conhdl, g_AppEnv.input[0] - '2');
            break;
        case 'a':
            APP_DiscRdCharReq(conhdl, DISC_PNP_ID_CHAR);
            break;
        case 'r':
            g_AppEnv.menu_id = menu_main;
            break;
        default:
            break;
    }

    APP_MenuShow();
}
#endif

#if BLE_BATT_CLIENT
const char *const s_appMenuBasc[] = {
    "* BLE BASC Menu",
    "* 0. Select device",
    "* 1. Enable",
    "* 2. Read Battery Level",
    "* 3. Read Battery Level Cfg",
    "* 4. Read Battery Level Format",
    "* 5. Enable Battery Notify",
    "* 6. Disable Battery Notify",
};

static void APP_MenuShowBasc(void)
{
    APP_MenuShowMenu(s_appMenuBasc, sizeof(s_appMenuBasc) / 4);
}

static void APP_MenuBascHandler(void)
{
    uint16_t conhdl = CONIDX2CONHDL(g_AppEnv.select_idx);

    switch (g_AppEnv.input[0])
    {
        case '0':
            g_AppEnv.menu_id = menu_basc_enable;
            APP_MenuShowGapConnRecord(menu_basc);
            break;
        case '1':
            APP_BascEnableReq(conhdl);
            break;
        case '2':
            APP_BascReadInfoReq(conhdl, BASC_BATT_LVL_VAL, 0);
            break;
        case '3':
            APP_BascReadInfoReq(conhdl, BASC_NTF_CFG, 0);
            break;
        case '4':
            APP_BascReadInfoReq(conhdl, BASC_BATT_LVL_PRES_FORMAT, 0);
            break;
        case '5':
            APP_BascBattLevelNtfCfgReq(conhdl, PRF_CLI_START_NTF, 0);
            break;
        case '6':
            APP_BascBattLevelNtfCfgReq(conhdl, PRF_CLI_STOP_NTFIND, 0);
            break;
        case 'r':
            g_AppEnv.menu_id = menu_main;
            break;
        default:
            break;
    }

    APP_MenuShow();
}
#endif

#if BLE_GL_COLLECTOR
const char *const s_appMenuGlpc[] = {
    "* BLE GLPC Menu",
    "* 0. Select device",
    "* 1. Enable",
    "* 2. Read Features",
    "* 3. Register",
    "* 4. Report Stored Records  ",
    "* 5. Delete Stored Records ",
    "* 6. Abort Current Operation",
    "* 7. Report Number of Stored Records",
};

static void APP_MenuShowGlpc(void)
{
    APP_MenuShowMenu(s_appMenuGlpc, sizeof(s_appMenuGlpc) / 4);
}

static void APP_MenuGlpcHandler(void)
{
    uint16_t conhdl = CONIDX2CONHDL(g_AppEnv.select_idx);

    switch (g_AppEnv.input[0])
    {
        case '0':
            g_AppEnv.menu_id = menu_glpc_enable;
            APP_MenuShowGapConnRecord(menu_glpc);
            break;
        case '1':
            APP_GlpcEnableReq(conhdl);
            break;
        case '2':
            APP_GlpcReadFeaturesReq(conhdl);
            break;
        case '3':
            APP_GlpcRegisterReq(conhdl, true);
            break;
        case '4':
            APP_GlpcReportStoredRecord(conhdl);
            break;
        case '5':
            APP_GlpcDelStoredRecord(conhdl);
            break;
        case '6':
            APP_GlpcAbortOperation(conhdl);
            break;
        case '7':
            APP_GlpcReportNumStoredRecord(conhdl);
            break;
        case 'r':
            g_AppEnv.menu_id = menu_main;
            break;
        default:
            break;
    }

    APP_MenuShow();
}
#endif

#if BLE_PROX_MONITOR
const char *const s_appMenuProxm[] = {
    "* BLE PROXM Menu",
    "* 0. Select device",
    "* 1. Enable",
    "* 2. Set Link Loss None",
    "* 3. Set Link Loss Mild",
    "* 4. Set Link Loss High",
    "* 5. Set Immediate Alert None",
    "* 6. Set Immediate Alert Mild",
    "* 7. Set Immediate Alert High",
    "* 8. Read LLS Level",
    "* 9. Read TX Power",
};

static void APP_MenuShowProxm(void)
{
    APP_MenuShowMenu(s_appMenuProxm, sizeof(s_appMenuProxm) / 4);
}

static void APP_MenuProxmHandler(void)
{
    uint16_t conhdl = CONIDX2CONHDL(g_AppEnv.select_idx);

    switch (g_AppEnv.input[0])
    {
        case '0':
            g_AppEnv.menu_id = menu_proxm_enable;
            APP_MenuShowGapConnRecord(menu_proxm);
            break;

        case '1':
            APP_ProxmEnableReq(conhdl);
            break;

        case '2':
            APP_ProxmWrAlertLvlReq(conhdl, PROXM_SET_LK_LOSS_ALERT, PROXM_ALERT_NONE);
            break;

        case '3':
            APP_ProxmWrAlertLvlReq(conhdl, PROXM_SET_LK_LOSS_ALERT, PROXM_ALERT_MILD);
            break;

        case '4':
            APP_ProxmWrAlertLvlReq(conhdl, PROXM_SET_LK_LOSS_ALERT, PROXM_ALERT_HIGH);
            break;

        case '5':
            APP_ProxmWrAlertLvlReq(conhdl, PROXM_SET_IMMDT_ALERT, PROXM_ALERT_NONE);
            break;

        case '6':
            APP_ProxmWrAlertLvlReq(conhdl, PROXM_SET_IMMDT_ALERT, PROXM_ALERT_MILD);
            break;

        case '7':
            APP_ProxmWrAlertLvlReq(conhdl, PROXM_SET_IMMDT_ALERT, PROXM_ALERT_HIGH);
            break;

        case '8':
            APP_ProxmRdReq(conhdl, PROXM_RD_LL_ALERT_LVL);
            break;

        case '9':
            APP_ProxmRdReq(conhdl, PROXM_RD_TX_POWER_LVL);
            break;

        case 'r':
            g_AppEnv.menu_id = menu_main;
            break;
        default:
            break;
    }

    APP_MenuShow();
}
#endif

#if BLE_FINDME_LOCATOR
const char *const s_appMenuFindl[] = {
    "* BLE FMPL Menu", "* 0. Select device", "* 1. Enable", "* 2. Alert None", "* 3. Alert Mild", "* 4. Alert High",
};

static void APP_MenuShowFindl(void)
{
    APP_MenuShowMenu(s_appMenuFindl, sizeof(s_appMenuFindl) / 4);
}

static void APP_MenuFindlHandler(void)
{
    uint16_t conhdl = CONIDX2CONHDL(g_AppEnv.select_idx);

    switch (g_AppEnv.input[0])
    {
        case '0':
            g_AppEnv.menu_id = menu_findl_enable;
            APP_MenuShowGapConnRecord(menu_findl);
            break;

        case '1':
            APP_FindlEnableReq(conhdl);
            break;

        case '2':
            APP_FindlSetAlertReq(conhdl, FINDL_ALERT_NONE);
            break;

        case '3':
            APP_FindlSetAlertReq(conhdl, FINDL_ALERT_MILD);
            break;

        case '4':
            APP_FindlSetAlertReq(conhdl, FINDL_ALERT_HIGH);
            break;

        case 'r':
            g_AppEnv.menu_id = menu_main;
            break;

        default:
            break;
    }

    APP_MenuShow();
}
#endif
#if BLE_HT_COLLECTOR
const char *const s_appMenuHtpc[] = {
    "* BLE HTPC Menu",
    "* 0. Select device",
    "* 1. Enable",
    "* 2. Read temp type",
    "* 3. Read meas interval",
    "* 4. Temp Meas indication",
    "* 5. Intm notification",
    "* 6. Intv indication",
    "* 7. Write new intv",
    "* 8. Stop Temp notification",
    "* 9. Stop Intm notification",
    "* a. Stop Intv notification",
};

static void APP_MenuShowHtpc(void)
{
    APP_MenuShowMenu(s_appMenuHtpc, sizeof(s_appMenuHtpc) / 4);
}
static void APP_MenuHtpcHandler(void)
{
    uint16_t conhdl = CONIDX2CONHDL(g_AppEnv.select_idx);

    switch (g_AppEnv.input[0])
    {
        case '0':
            g_AppEnv.menu_id = menu_htpc_enable;
            APP_MenuShowGapConnRecord(menu_htpc);
            break;

        case '1':
            APP_HtpcEnableReq(conhdl);
            break;

        case '2':
            APP_HtpcRdCharReq(conhdl, HTPC_RD_TEMP_TYPE);
            break;

        case '3':
            APP_HtpcRdCharReq(conhdl, HTPC_RD_MEAS_INTV);
            break;

        case '4':
            APP_HtpcHealthTempNtfCfgReq(conhdl, PRF_CLI_START_IND, HTPC_CHAR_HTS_TEMP_MEAS);
            break;

        case '5':
            APP_HtpcHealthTempNtfCfgReq(conhdl, PRF_CLI_START_NTF, HTPC_CHAR_HTS_INTM_TEMP);
            break;

        case '6':
            APP_HtpcHealthTempNtfCfgReq(conhdl, PRF_CLI_START_IND, HTPC_CHAR_HTS_MEAS_INTV);
            break;

        case '7':
            APP_HtpcWrMeasIntvReq(conhdl, 5);
            break;

        case '8':
            APP_HtpcHealthTempNtfCfgReq(conhdl, PRF_CLI_STOP_NTFIND, HTPC_CHAR_HTS_TEMP_MEAS);
            break;

        case '9':
            APP_HtpcHealthTempNtfCfgReq(conhdl, PRF_CLI_STOP_NTFIND, HTPC_CHAR_HTS_INTM_TEMP);
            break;

        case 'a':
            APP_HtpcHealthTempNtfCfgReq(conhdl, PRF_CLI_STOP_NTFIND, HTPC_CHAR_HTS_MEAS_INTV);
            break;

        case 'r':
            g_AppEnv.menu_id = menu_main;
            break;

        default:
            break;
    }

    APP_MenuShow();
}

#endif

#if BLE_CP_COLLECTOR
const char *const s_appMenuCppc[] = {
    "* BLE CPPC Menu",
    "* 0. Select device",
    "* 1. Enable",
    "* 2. Read Feature",
    "* 3. Write Control Point Client Char",
    "* 4. Stop Control Point Client Char",
    "* 5. Set Cumulative Value",
    "* 6. Write Measurement Client Char",
    "* 7. Write Measurement Server Char",
    "* 8. Write Vector Client Char",
    "* 9. Stop Measurement Client Noti",
    "* a. Stop Measurement Server Noti",
    "* b. Stop Vector Client Noti",

};

static void APP_MenuShowCppc(void)
{
    APP_MenuShowMenu(s_appMenuCppc, sizeof(s_appMenuCppc) / 4);
}

static void APP_MenuCppcHandler(void)
{
    uint16_t conhdl = CONIDX2CONHDL(g_AppEnv.select_idx);
    struct cpp_ctnl_pt_req ctnl_pt;

    switch (g_AppEnv.input[0])
    {
        case '0':
            g_AppEnv.menu_id = menu_cppc_enable;
            APP_MenuShowGapConnRecord(menu_cppc);
            break;

        case '1':
            APP_CppcEnableReq(conhdl);
            break;

        case '2':
            APP_CppcReadCmd(conhdl, CPPC_READ_OP_CODE, CPPC_RD_CP_FEAT);
            break;

        case '3':
            APP_CppcCfgNtfindCmd(conhdl, CPPC_CFG_NTF_IND_OP_CODE, CPPC_RD_WR_CTNL_PT_CFG, PRF_CLI_START_IND);
            break;

        case '4':
            APP_CppcCfgNtfindCmd(conhdl, CPPC_CFG_NTF_IND_OP_CODE, CPPC_RD_WR_CTNL_PT_CFG, PRF_CLI_STOP_NTFIND);
            break;

        case '5':
        {
            ctnl_pt.op_code = CPP_CTNL_PT_SET_CUMUL_VAL;
            ctnl_pt.value.cumul_val = 4;
            APP_CppcCtnlPtCfgReq(conhdl, CPPC_CTNL_PT_CFG_WR_OP_CODE, ctnl_pt);
        }
        break;

        case '6':
            APP_CppcCfgNtfindCmd(conhdl, CPPC_CFG_NTF_IND_OP_CODE, CPPC_RD_WR_CP_MEAS_CL_CFG, PRF_CLI_START_NTF);
            break;

        case '7':
            APP_CppcCfgNtfindCmd(conhdl, CPPC_CFG_NTF_IND_OP_CODE, CPPC_RD_WR_CP_MEAS_SV_CFG, PRF_CLI_START_NTF);
            break;

        case '8':
            APP_CppcCfgNtfindCmd(conhdl, CPPC_CFG_NTF_IND_OP_CODE, CPPC_RD_WR_VECTOR_CFG, PRF_CLI_START_NTF);
            break;

        case '9':
            APP_CppcCfgNtfindCmd(conhdl, CPPC_CFG_NTF_IND_OP_CODE, CPPC_RD_WR_CP_MEAS_CL_CFG, PRF_CLI_STOP_NTFIND);
            break;

        case 'a':
            APP_CppcCfgNtfindCmd(conhdl, CPPC_CFG_NTF_IND_OP_CODE, CPPC_RD_WR_CP_MEAS_SV_CFG, PRF_CLI_STOP_NTFIND);
            break;

        case 'b':
            APP_CppcCfgNtfindCmd(conhdl, CPPC_CFG_NTF_IND_OP_CODE, CPPC_RD_WR_VECTOR_CFG, PRF_CLI_STOP_NTFIND);
            break;

        case 'r':
            g_AppEnv.menu_id = menu_main;
            break;

        default:
            break;
    }

    APP_MenuShow();
}

#endif

#if BLE_CSC_COLLECTOR
const char *const s_appMenuCscpc[] = {
    "* BLE CSCPC Menu",
    "* 0. Select device",
    "* 1. Enable",
    "* 2. Read Feature",
    "* 3. Read Sensor Location",
    "* 4. Read CSC Measurement Client Characteristic Configuration Descriptor",
    "* 5. Read SC Control Point Client Characteristic Configuration Descriptor",
    "* 6. Enable CSC Measurement Notify",
    "* 7. Stop CSC Measurement Notify",
    "* 8. Enable SC Control Point Indicate",
    "* 9. Stop SC Control Point Indicate",
    "* a. Set Cumulative Value",
    "* b. Update Sensor Location",
    "* c. Request Supported Sensor Locations",
};
static void APP_MenuShowCscps(void)
{
    APP_MenuShowMenu(s_appMenuCscpc, sizeof(s_appMenuCscpc) / 4);
}

static void APP_MenuCscpcHandler(void)
{
    uint16_t conhdl = CONIDX2CONHDL(g_AppEnv.select_idx);
    struct cscp_sc_ctnl_pt_req sc_ctnl_pt;
    switch (g_AppEnv.input[0])
    {
        case '0':
            g_AppEnv.menu_id = menu_cscpc_enable;
            APP_MenuShowGapConnRecord(menu_cscpc);
            break;
        case '1':
            APP_CscpcEnableReq(conhdl);
            break;
        case '2':
            APP_CscpcReadCmd(conhdl, CSCPC_READ_OP_CODE, CSCPC_RD_CSC_FEAT);
            break;
        case '3':
            APP_CscpcReadCmd(conhdl, CSCPC_READ_OP_CODE, CSCPC_RD_SENSOR_LOC);
            break;
        case '4':
            APP_CscpcReadCmd(conhdl, CSCPC_READ_OP_CODE, CSCPC_RD_WR_CSC_MEAS_CFG);
            break;
        case '5':
            APP_CscpcReadCmd(conhdl, CSCPC_READ_OP_CODE, CSCPC_RD_WR_SC_CTNL_PT_CFG);
            break;
        case '6':
            APP_CscpcCfgNtfindCmd(conhdl, CSCPC_CFG_NTF_IND_OP_CODE, CSCPC_RD_WR_CSC_MEAS_CFG, PRF_CLI_START_NTF);
            break;
        case '7':
            APP_CscpcCfgNtfindCmd(conhdl, CSCPC_CFG_NTF_IND_OP_CODE, CSCPC_RD_WR_CSC_MEAS_CFG, PRF_CLI_STOP_NTFIND);
            break;
        case '8':
            APP_CscpcCfgNtfindCmd(conhdl, CSCPC_CFG_NTF_IND_OP_CODE, CSCPC_RD_WR_SC_CTNL_PT_CFG, PRF_CLI_START_IND);
            break;
        case '9':
            APP_CscpcCfgNtfindCmd(conhdl, CSCPC_CFG_NTF_IND_OP_CODE, CSCPC_RD_WR_SC_CTNL_PT_CFG, PRF_CLI_STOP_NTFIND);
            break;
        case 'a':
            sc_ctnl_pt.op_code = CSCP_CTNL_PT_OP_SET_CUMUL_VAL;
            sc_ctnl_pt.value.cumul_val = 0;
            APP_CscpcCtnlPtCfgReq(conhdl, CSCPC_CTNL_PT_CFG_WR_OP_CODE, sc_ctnl_pt);
            break;
        case 'b':
            sc_ctnl_pt.op_code = CSCP_CTNL_PT_OP_UPD_LOC;
            sc_ctnl_pt.value.sensor_loc = 4;
            APP_CscpcCtnlPtCfgReq(conhdl, CSCPC_CTNL_PT_CFG_WR_OP_CODE, sc_ctnl_pt);
            break;
        case 'c':
            sc_ctnl_pt.op_code = CSCP_CTNL_PT_OP_REQ_SUPP_LOC;
            APP_CscpcCtnlPtCfgReq(conhdl, CSCPC_CTNL_PT_CFG_WR_OP_CODE, sc_ctnl_pt);
            break;
    }
    APP_MenuShow();
}

#endif

#if BLE_LN_COLLECTOR
const char *const s_appMenuLanc[] = {
    "* BLE LANC Menu",
    "* 0. Select device",
    "* 1. Enable",
    "* 2. Read Feature",
    "* 3. Start Loc&Speed Noti",
    "* 4. Stop Loc&Speed Noti",
    "* 5. Start Navigation Noti",
    "* 6. Stop Navigation Noti",
    "* 7. Start CP Noti",
    "* 8. Enable Navigation Control",
    "* 9. Stop CP Noti",

};

static void APP_MenuShowLanc(void)
{
    APP_MenuShowMenu(s_appMenuLanc, sizeof(s_appMenuLanc) / 4);
}

static void APP_MenuLancHandler(void)
{
    uint16_t conhdl = CONIDX2CONHDL(g_AppEnv.select_idx);

    switch (g_AppEnv.input[0])
    {
        case '0':
            g_AppEnv.menu_id = menu_lanc_enable;
            APP_MenuShowGapConnRecord(menu_lanc);
            break;

        case '1':
            APP_LancEnableReq(conhdl);
            break;

        case '2':
            APP_LancReadCmd(conhdl, LANC_READ_OP_CODE, LANC_RD_LN_FEAT);
            break;

        case '3':
            APP_LancCfgNtfindCmd(conhdl, LANC_CFG_NTF_IND_OP_CODE, LANC_RD_WR_LOC_SPEED_CL_CFG, PRF_CLI_START_NTF);
            break;

        case '4':
            APP_LancCfgNtfindCmd(conhdl, LANC_CFG_NTF_IND_OP_CODE, LANC_RD_WR_LOC_SPEED_CL_CFG, PRF_CLI_STOP_NTFIND);
            break;

        case '5':
            APP_LancCfgNtfindCmd(conhdl, LANC_CFG_NTF_IND_OP_CODE, LANC_RD_WR_NAVIGATION_CFG, PRF_CLI_START_NTF);
            break;

        case '6':
            APP_LancCfgNtfindCmd(conhdl, LANC_CFG_NTF_IND_OP_CODE, LANC_RD_WR_NAVIGATION_CFG, PRF_CLI_STOP_NTFIND);
            break;

        case '7':
            APP_LancCfgNtfindCmd(conhdl, LANC_CFG_NTF_IND_OP_CODE, LANC_RD_WR_LN_CTNL_PT_CFG, PRF_CLI_START_IND);
            break;

        case '8':
        {
            struct lan_ln_ctnl_pt_req ln_ctnl_pt;
            ln_ctnl_pt.op_code = LANP_LN_CTNL_PT_NAVIGATION_CONTROL;
            ln_ctnl_pt.value.control_value = LANP_LN_CTNL_START_NAVI;
            APP_LancLnCtnlPtCfgReq(conhdl, LANC_LN_CTNL_PT_CFG_WR_OP_CODE, &ln_ctnl_pt);
        }
        break;

        case '9':
            APP_LancCfgNtfindCmd(conhdl, LANC_CFG_NTF_IND_OP_CODE, LANC_RD_WR_LN_CTNL_PT_CFG, PRF_CLI_STOP_NTFIND);
            break;

        case 'r':
            g_AppEnv.menu_id = menu_main;
            break;

        default:
            break;
    }

    APP_MenuShow();
}

#endif

#if BLE_SP_CLIENT
const char *const s_appMenuScppc[] = {
    "* BLE SCPPC Menu",
    "* 0. Select device",
    "* 1. Enable",
    "* 2. Write Scan Parameter",
    "* 3. Read Notification Configuration",
    "* 4. Start Refresh Notification",
    "* 5. Stop Refresh Notification",

};

static void APP_MenuShowScppc(void)
{
    APP_MenuShowMenu(s_appMenuScppc, sizeof(s_appMenuScppc) / 4);
}
static void APP_MenuScppcHandler(void)
{
    uint16_t conhdl = CONIDX2CONHDL(g_AppEnv.select_idx);

    switch (g_AppEnv.input[0])
    {
        case '0':
            g_AppEnv.menu_id = menu_scppc_enable;
            APP_MenuShowGapConnRecord(menu_scppc);
            break;

        case '1':
            APP_ScppcEnableReq(conhdl);
            break;

        case '2':
            APP_ScppcScanIntvWdWrReq(conhdl, 0x20, 0x20);
            break;

        case '3':
            APP_ScppcScanRefreshNtfCfgRdReq(conhdl);
            break;

        case '4':
            APP_ScppcScanRefreshNtfCfgReq(conhdl, PRF_CLI_START_NTF);
            break;

        case '5':
            APP_ScppcScanRefreshNtfCfgReq(conhdl, PRF_CLI_STOP_NTFIND);
            break;

        case 'r':
            g_AppEnv.menu_id = menu_main;
            break;

        default:
            break;
    }

    APP_MenuShow();
}

#endif

#if BLE_PAS_CLIENT
const char *const s_appMenuPaspc[] = {
    "* QN BLE PASPC Menu",
    "* 0. Select device",
    "* 1. Enable",
    "* 2. Read Alert Status",
    "* 3. Read Ringer Setting",
    "* 4. Read Alert Status ntf cfg ",
    "* 5. Read Ringer Setting ntf cfg",
    "* 6.Alert Status NTF  Enable",
    "* 7.Alert Status NTF  Disable",
    "* 8. Ringer Setting NTF Enable",
    "* 9. Ringer Setting NTF Disable",
    "* a. Set Silent Mode",
    "* b. Set Mute Once",
    "* c. Cancel Silent Mode",
};

static void APP_MenuShowPaspc(void)
{
    APP_MenuShowMenu(s_appMenuPaspc, sizeof(s_appMenuPaspc) / 4);
}

static void APP_MenuPaspcHandler(void)
{
    uint16_t conhdl = CONIDX2CONHDL(g_AppEnv.select_idx);
    struct paspc_write_cmd write_cmd;
    switch (g_AppEnv.input[0])
    {
        case '0':
            g_AppEnv.menu_id = menu_paspc_enable;
            APP_MenuShowGapConnRecord(menu_paspc);
            break;
        case '1':
            APP_PaspcEnableReq(conhdl);
            break;
        case '2':
        {
            APP_PaspcReadCmd(conhdl, PASPC_RD_ALERT_STATUS);
            break;
        }
        case '3':
            APP_PaspcReadCmd(conhdl, PASPC_RD_RINGER_SETTING);
            break;
        case '4':
            APP_PaspcReadCmd(conhdl, PASPC_RD_WR_ALERT_STATUS_CFG);
            break;
        case '5':
            APP_PaspcReadCmd(conhdl, PASPC_RD_WR_RINGER_SETTING_CFG);
            break;
        case '6':
            write_cmd.write_code = PASPC_RD_WR_ALERT_STATUS_CFG;
            write_cmd.value.alert_status_ntf_cfg = PRF_CLI_START_NTF;
            APP_PaspcWriteCmd(conhdl, write_cmd);
            break;
        case '7':
            write_cmd.write_code = PASPC_RD_WR_ALERT_STATUS_CFG;
            write_cmd.value.alert_status_ntf_cfg = PRF_CLI_STOP_NTFIND;
            APP_PaspcWriteCmd(conhdl, write_cmd);
            break;
        case '8':
            write_cmd.write_code = PASPC_RD_WR_RINGER_SETTING_CFG;
            write_cmd.value.ringer_setting_ntf_cfg = PRF_CLI_START_NTF;
            APP_PaspcWriteCmd(conhdl, write_cmd);
            break;
        case '9':
            write_cmd.write_code = PASPC_RD_WR_RINGER_SETTING_CFG;
            write_cmd.value.ringer_setting_ntf_cfg = PRF_CLI_STOP_NTFIND;
            APP_PaspcWriteCmd(conhdl, write_cmd);
            break;
        case 'a':
            write_cmd.write_code = PASPC_WR_RINGER_CTNL_PT;
            write_cmd.value.ringer_ctnl_pt = PASP_SILENT_MODE;
            APP_PaspcWriteCmd(conhdl, write_cmd);
            break;
        case 'b':
            write_cmd.write_code = PASPC_WR_RINGER_CTNL_PT;
            write_cmd.value.ringer_ctnl_pt = PASP_MUTE_ONCE;
            APP_PaspcWriteCmd(conhdl, write_cmd);
            break;
        case 'c':
            write_cmd.write_code = PASPC_WR_RINGER_CTNL_PT;
            write_cmd.value.ringer_ctnl_pt = PASP_CANCEL_SILENT_MODE;
            APP_PaspcWriteCmd(conhdl, write_cmd);
            break;
        case 'r':
            g_AppEnv.menu_id = menu_main;
            break;
        default:
            break;
    }

    APP_MenuShow();
}
#endif

#if BLE_HID_BOOT_HOST
const char *const s_appMenuHogph[] = {
    "* BLE HOGPBH Menu",      
    "* 0. Select device",           
    "* 1. Enable",
    "* 2. Read Info",          
    "* 3. Read Mouse Protocol Mode",
    "* 4. Write Mouse Boot Host Mode", 
    "* 5. Start Mouse Notification",
    "* 6. Stop Mouse Notification",    

};

static void APP_MenuShowHogph(void)
{
    APP_MenuShowMenu(s_appMenuHogph, sizeof(s_appMenuHogph) / 4);
}

static void APP_MenuHogphHandler(void)
{
    uint16_t conhdl = CONIDX2CONHDL(g_AppEnv.select_idx);

    switch (g_AppEnv.input[0])
    {
        case '0':
            g_AppEnv.menu_id = menu_hogpbh_enable;
            APP_MenuShowGapConnRecord(menu_hogph);
            break;

        case '1':
        {
            uint8_t hids_nb = 0;
            uint8_t type = PRF_CON_DISCOVERY;
            struct hogpbh_content hids[HOGPBH_NB_HIDS_INST_MAX];
            APP_HogpbhEnableReq(conhdl, type, hids_nb, hids);
            break;
        }

        case '2':
        {
            uint8_t info = HOGPBH_BOOT_MOUSE_IN_REPORT;
            uint8_t hid_idx = 0;

            APP_HogpbhReadInfoReq(conhdl, info, hid_idx);
            break;
        }

        case '3':
        {
            uint8_t info = HOGPBH_PROTO_MODE;
            uint8_t hid_idx = 0;

            APP_HogpbhReadInfoReq(conhdl, info, hid_idx);
            break;
        }

        case '4':
        {
            uint8_t info = HOGPBH_PROTO_MODE;
            uint8_t hid_idx = 0;
            bool wr_cmd = true;
            union hogpbh_data data;
            data.proto_mode = HOGP_BOOT_PROTOCOL_MODE;
            APP_HogpbhWriteReq(conhdl, info, hid_idx, wr_cmd, &data);
            break;
        }

        case '5':
        {
            uint8_t info = HOGPBH_BOOT_MOUSE_IN_NTF_CFG;
            uint8_t hid_idx = 0;
            bool wr_cmd = true;
            union hogpbh_data data;
            data.ntf_cfg = PRF_CLI_START_NTF;
            APP_HogpbhWriteReq(conhdl, info, hid_idx, wr_cmd, &data);
            break;
        }

        case '6':
        {
            uint8_t info = HOGPBH_BOOT_MOUSE_IN_NTF_CFG;
            uint8_t hid_idx = 0;
            bool wr_cmd = true;
            union hogpbh_data data;
            data.ntf_cfg = PRF_CLI_STOP_NTFIND;
            APP_HogpbhWriteReq(conhdl, info, hid_idx, wr_cmd, &data);
            break;
        }

        case 'r':
            g_AppEnv.menu_id = menu_main;
            break;

        default:
            break;
    }

    APP_MenuShow();
}

#endif

#if BLE_HID_REPORT_HOST
const char *const s_appMenuHogph[] = {
    "* BLE HOGPRH Menu",
    "* 0. Select device",
    "* 1. Enable",
    "* 2. Read Report Map",
    "* 3. Read Mouse Protocol Mode",
    "* 4. Write Mouse Report Host Mode",
    "* 5. Set Mouse CP Suspend",
    "* 6. Exit Mouse CP Suspend",
    "* 7. Start Mouse Report Notification",
    "* 8. Stop Mouse Report Notification",

};

static void APP_MenuShowHogph(void)
{
    APP_MenuShowMenu(s_appMenuHogph, sizeof(s_appMenuHogph) / 4);
}

static void APP_MenuHogphHandler(void)
{
    uint16_t conhdl = CONIDX2CONHDL(g_AppEnv.select_idx);

    switch (g_AppEnv.input[0])
    {
        case '0':
            g_AppEnv.menu_id = menu_hogprh_enable;
            APP_MenuShowGapConnRecord(menu_hogph);
            break;

        case '1':
        {
            uint8_t hids_nb = 0;
            uint8_t type = PRF_CON_DISCOVERY;
            struct hogprh_content hids[HOGPRH_NB_HIDS_INST_MAX];
            APP_HogprhEnableReq(conhdl, type, hids_nb, hids);
            break;
        }

        case '2':
        {
            uint8_t info = HOGPRH_REPORT_MAP;
            uint8_t hid_idx = 0;
            uint8_t report_idx = 0;
            APP_HogprhReadInfoReq(conhdl, info, hid_idx, report_idx);
            break;
        }

        case '3':
        {
            uint8_t info = HOGPRH_PROTO_MODE;
            uint8_t hid_idx = 0;
            uint8_t report_idx = 0;
            APP_HogprhReadInfoReq(conhdl, info, hid_idx, report_idx);
            break;
        }
        
        case '4':
        {
            uint8_t info = HOGPRH_PROTO_MODE;
            uint8_t hid_idx = 0;
            uint8_t report_idx = 0;
            bool wr_cmd = true;
            union hogprh_data data;
            data.proto_mode = HOGP_REPORT_PROTOCOL_MODE;
            APP_HogprhWriteReq(conhdl, info, hid_idx, report_idx, wr_cmd, &data);
            break;
        }


        case '5':
        {
            uint8_t info = HOGPRH_HID_CTNL_PT;
            uint8_t hid_idx = 0;
            uint8_t report_idx = 0;
            bool wr_cmd = true;
            union hogprh_data data;
            data.hid_ctnl_pt = HOGP_CTNL_PT_SUSPEND;
            APP_HogprhWriteReq(conhdl, info, hid_idx, report_idx, wr_cmd, &data);
            break;
        }

        case '6':
        {
            uint8_t info = HOGPRH_HID_CTNL_PT;
            uint8_t hid_idx = 0;
            uint8_t report_idx = 0;
            bool wr_cmd = true;
            union hogprh_data data;
            data.hid_ctnl_pt = HOGP_CTNL_PT_EXIT_SUSPEND;
            APP_HogprhWriteReq(conhdl, info, hid_idx, report_idx, wr_cmd, &data);
            break;
        }

        case '7':
        {
            uint8_t info = HOGPRH_REPORT_NTF_CFG;
            uint8_t hid_idx = 0;
            uint8_t report_idx = 0;
            bool wr_cmd = true;
            union hogprh_data data;
            data.report_cfg = PRF_CLI_START_NTF;
            APP_HogprhWriteReq(conhdl, info, hid_idx, report_idx, wr_cmd, &data);
            break;
        }

        case '8':
        {
            uint8_t info = HOGPRH_REPORT_NTF_CFG;
            uint8_t hid_idx = 0;
            uint8_t report_idx = 0;
            bool wr_cmd = true;
            union hogprh_data data;
            data.report_cfg = PRF_CLI_STOP_NTFIND;
            APP_HogprhWriteReq(conhdl, info, hid_idx, report_idx, wr_cmd, &data);
            break;
        }

        case 'r':
            g_AppEnv.menu_id = menu_main;
            break;

        default:
            break;
    }

    APP_MenuShow();
}

#endif

#if BLE_AN_CLIENT
const char *const s_appMenuAnpc[] = {
    "* QN BLE ANPC Menu",
    "* 0. Select device",
    "* 1. Enable",
    "* 2. New alert NTF enable",
    "* 3. Unread alert ntf enable",
    "* 4. New alert NTF disable ",
    "* 5. Unread alert ntf disable",
    "* 6. Enable all new alert notification",
    "* 7. Enable all unread alert notification",
    "* 8. Disable all new alert notification",
    "* 9. Disable all unread alert notification",
    "* a. Notify new alert immediately",
    "* b. Notify unread alert immediately",
    "* c. read new alert ccc",
    "* d. read unread alert ccc",
};

static void APP_MenuShowAnpc(void)
{
    APP_MenuShowMenu(s_appMenuAnpc, sizeof(s_appMenuAnpc) / 4);
}

static void APP_MenuAnpcHandler(void)
{
    uint16_t conhdl = CONIDX2CONHDL(g_AppEnv.select_idx);
    struct anpc_write_cmd write_cmd;
    switch (g_AppEnv.input[0])
    {
        case '0':
            g_AppEnv.menu_id = menu_anpc_enable;
            APP_MenuShowGapConnRecord(menu_anpc);
            break;
        case '1':
            APP_AnpcEnableReq(conhdl);
            break;
        case '2':
        {
            write_cmd.write_code = ANPC_RD_WR_NEW_ALERT_CFG;
            write_cmd.value.new_alert_ntf_cfg = PRF_CLI_START_NTF;
            APP_AnpcWriteCmd(conhdl, write_cmd);
            break;
        }
        case '3':
            write_cmd.write_code = ANPC_RD_WR_UNREAD_ALERT_STATUS_CFG;
            write_cmd.value.new_alert_ntf_cfg = PRF_CLI_START_NTF;
            APP_AnpcWriteCmd(conhdl, write_cmd);
            break;
        case '4':
            write_cmd.write_code = ANPC_RD_WR_NEW_ALERT_CFG;
            write_cmd.value.new_alert_ntf_cfg = PRF_CLI_STOP_NTFIND;
            APP_AnpcWriteCmd(conhdl, write_cmd);
            break;
        case '5':
            write_cmd.write_code = ANPC_RD_WR_UNREAD_ALERT_STATUS_CFG;
            write_cmd.value.new_alert_ntf_cfg = PRF_CLI_STOP_NTFIND;
            APP_AnpcWriteCmd(conhdl, write_cmd);
            break;
        case '6':
            write_cmd.write_code = ANPC_WR_ALERT_NTF_CTNL_PT;
            write_cmd.value.ctnl_pt.cmd_id = CMD_ID_EN_NEW_IN_ALERT_NTF;
            write_cmd.value.ctnl_pt.cat_id = CAT_ID_ALL_SUPPORTED_CAT;
            APP_AnpcWriteCmd(conhdl, write_cmd);
            break;
        case '7':
            write_cmd.write_code = ANPC_WR_ALERT_NTF_CTNL_PT;
            write_cmd.value.ctnl_pt.cmd_id = CMD_ID_EN_UNREAD_CAT_STATUS_NTF;
            write_cmd.value.ctnl_pt.cat_id = CAT_ID_ALL_SUPPORTED_CAT;
            APP_AnpcWriteCmd(conhdl, write_cmd);
            break;
        case '8':
            write_cmd.write_code = ANPC_WR_ALERT_NTF_CTNL_PT;
            write_cmd.value.ctnl_pt.cmd_id = CMD_ID_DIS_NEW_IN_ALERT_NTF;
            write_cmd.value.ctnl_pt.cat_id = CAT_ID_ALL_SUPPORTED_CAT;
            APP_AnpcWriteCmd(conhdl, write_cmd);
            break;
        case '9':
            write_cmd.write_code = ANPC_WR_ALERT_NTF_CTNL_PT;
            write_cmd.value.ctnl_pt.cmd_id = CMD_ID_DIS_UNREAD_CAT_STATUS_NTF;
            write_cmd.value.ctnl_pt.cat_id = CAT_ID_ALL_SUPPORTED_CAT;
            APP_AnpcWriteCmd(conhdl, write_cmd);
            break;
        case 'a':
            write_cmd.write_code = ANPC_WR_ALERT_NTF_CTNL_PT;
            write_cmd.value.ctnl_pt.cmd_id = CMD_ID_NTF_NEW_IN_ALERT_IMM;
            write_cmd.value.ctnl_pt.cat_id = CAT_ID_ALL_SUPPORTED_CAT;
            APP_AnpcWriteCmd(conhdl, write_cmd);
            break;
        case 'b':
            write_cmd.write_code = ANPC_WR_ALERT_NTF_CTNL_PT;
            write_cmd.value.ctnl_pt.cmd_id = CMD_ID_NTF_UNREAD_CAT_STATUS_IMM;
            write_cmd.value.ctnl_pt.cat_id = CAT_ID_ALL_SUPPORTED_CAT;
            APP_AnpcWriteCmd(conhdl, write_cmd);
            break;
        case 'c':
            APP_AnpcReadCmd(conhdl, ANPC_RD_WR_NEW_ALERT_CFG);
            break;
        case 'd':
            APP_AnpcReadCmd(conhdl, ANPC_RD_WR_UNREAD_ALERT_STATUS_CFG);
            break;
        case 'r':
            g_AppEnv.menu_id = menu_main;
            break;
        default:
            break;
    }

    APP_MenuShow();
}
#endif

#if BLE_TIP_CLIENT
const char *const s_appMenuTipc[] = {
    "* BLE TIPC Menu",        "* 0. Select device",        "* 1. Enable",
    "* 2. Read Current Time", "* 3. Start Notification",   "* 4. Stop Notification",
    "* 5. Control Point Get", "* 6. Control Point Cancel",

};

static void APP_MenuShowTipc(void)
{
    APP_MenuShowMenu(s_appMenuTipc, sizeof(s_appMenuTipc) / 4);
}

static void APP_MenuTipcHandler(void)
{
    uint16_t conhdl = CONIDX2CONHDL(g_AppEnv.select_idx);

    switch (g_AppEnv.input[0])
    {
        case '0':
        {
            g_AppEnv.menu_id = menu_tipc_enable;
            APP_MenuShowGapConnRecord(menu_tipc);
            break;
        }

        case '1':
        {
            APP_TipcEnableReq(conhdl);
            break;
        }

        case '2':
        {
            static uint8_t char_code = TIPC_RD_CTS_CURR_TIME_CLI_CFG;
            APP_TipcRdCharReq(conhdl, char_code);
            break;
        }

        case '3':
        {
            uint16_t cfg_val = PRF_CLI_START_NTF;
            APP_TipcCtNtfCfgReq(conhdl, cfg_val);
            break;
        }

        case '4':
        {
            uint16_t cfg_val = PRF_CLI_STOP_NTFIND;
            APP_TipcCtNtfCfgReq(conhdl, cfg_val);
            break;
        }

        case '5':
        {
            uint8_t value = TIPS_TIME_UPD_CTNL_PT_GET;
            APP_TipcWrTimeUpdCtnlPtReq(conhdl, value);
            break;
        }

        case '6':
        {
            uint8_t value = TIPS_TIME_UPD_CTNL_PT_CANCEL;
            APP_TipcWrTimeUpdCtnlPtReq(conhdl, value);
            break;
        }

        case 'r':
            g_AppEnv.menu_id = menu_main;
            break;

        default:
            break;
    }

    APP_MenuShow();
}

#endif

#if BLE_HR_COLLECTOR
const char *const s_appMenuHrpc[] = {
    "* BLE HRPC Menu",           "* 0. Select device",       "* 1. Enable",
    "* 2. Read Sensor Location", "* 3. Write Control Point", "* 4. Start Notification",
    "* 5. Stop Notification",

};

static void APP_MenuShowHrpc(void)
{
    APP_MenuShowMenu(s_appMenuHrpc, sizeof(s_appMenuHrpc) / 4);
}

static void APP_MenuHrpcHandler(void)
{
    uint16_t conhdl = CONIDX2CONHDL(g_AppEnv.select_idx);

    switch (g_AppEnv.input[0])
    {
        case '0':
        {
            g_AppEnv.menu_id = menu_hrpc_enable;
            APP_MenuShowGapConnRecord(menu_hrpc);
            break;
        }

        case '1':
        {
            APP_HrpcEnableReq(conhdl, NULL);
            break;
        }

        case '2':
        {
            APP_HrpcRdCharReq(conhdl, HRPC_RD_HRS_BODY_SENSOR_LOC);
            break;
        }
        case '3':
        {
            uint8_t value = 1;
            APP_HrpcWrCntlPointReq(conhdl, value);
            break;
        }

        case '4':
        {
            uint8_t cfg_val = PRF_CLI_START_NTF;
            APP_HrpcCfgIndntfReq(conhdl, cfg_val);
            break;
        }

        case '5':
        {
            uint8_t cfg_val = PRF_CLI_STOP_NTFIND;
            APP_HrpcCfgIndntfReq(conhdl, cfg_val);
            break;
        }

        case 'r':
            g_AppEnv.menu_id = menu_main;
            break;

        default:
            break;
    }

    APP_MenuShow();
}
#endif

#if BLE_BP_COLLECTOR
const char *const s_appMenuBlpc[] = {
    "* BLE BLPC Menu",          "* 0. Select device",        "* 1. Enable",
    "* 2. Read feature",        "* 3. Enable Indication ",   "* 4. Disable Indication",
    "* 5. Enable Notification", "* 6. Disable Notification",

};

static void APP_MenuShowBlpc(void)
{
    APP_MenuShowMenu(s_appMenuBlpc, sizeof(s_appMenuBlpc) / 4);
}

static void APP_MenuBlpcHandler(void)
{
    uint16_t conhdl = CONIDX2CONHDL(g_AppEnv.select_idx);

    switch (g_AppEnv.input[0])
    {
        case '0':
        {
            g_AppEnv.menu_id = menu_blpc_enable;
            APP_MenuShowGapConnRecord(menu_blpc);
            break;
        }

        case '1':
        {
            APP_BlpcEnableReq(conhdl);
            break;
        }

        case '2':
        {
            static uint8_t char_code = BLPC_RD_BPS_FEATURE;
            APP_BlpcRdCharReq(conhdl, char_code);
            break;
        }

        case '3':
        {
            APP_BlpcCfgIndNtfReq(conhdl, BLPC_CHAR_BP_MEAS, PRF_CLI_START_IND);
            break;
        }

        case '4':
        {
            APP_BlpcCfgIndNtfReq(conhdl, BLPC_CHAR_BP_MEAS, PRF_CLI_STOP_NTFIND);
            break;
        }

        case '5':
        {
            APP_BlpcCfgIndNtfReq(conhdl, BLPC_CHAR_CP_MEAS, PRF_CLI_START_NTF);
            break;
        }

        case '6':
        {
            APP_BlpcCfgIndNtfReq(conhdl, BLPC_CHAR_CP_MEAS, PRF_CLI_START_IND);
            break;
        }

        case 'r':
            g_AppEnv.menu_id = menu_main;
            break;

        default:
            break;
    }

    APP_MenuShow();
}

#endif

#if BLE_RSC_COLLECTOR
const char *const s_appMenuRscpc[] = {
    "* BLE RSCPC Menu",
    "* 0. Select device",
    "* 1. Enable",
    "* 2. Read Feature",
    "* 3. Read Sensor Location",
    "* 4. Read RSC Measurement Client Characteristic Configuration Descriptor",
    "* 5. Read SC Control Point Client Characteristic Configuration Descriptor",
    "* 6. Enable RSC Measurement Notify",
    "* 7. Stop RSC Measurement Notify",
    "* 8. Enable SC Control Point Indicate",
    "* 9. Stop SC Control Point Indicate",
    "* a. Set Cumulative Value",
    "* b. Update Sensor Location",
    "* c. Request Supported Sensor Locations",
};
static void APP_MenuShowRscps(void)
{
    APP_MenuShowMenu(s_appMenuRscpc, sizeof(s_appMenuRscpc) / 4);
}

static void APP_MenuRscpcHandler(void)
{
    uint16_t conhdl = CONIDX2CONHDL(g_AppEnv.select_idx);
    struct rscp_sc_ctnl_pt_req sc_ctnl_pt;
    switch (g_AppEnv.input[0])
    {
        case '0':
            g_AppEnv.menu_id = menu_rscpc_enable;
            APP_MenuShowGapConnRecord(menu_rscpc);
            break;
        case '1':
            APP_RscpcEnableReq(conhdl);
            break;
        case '2':
            APP_RscpcReadCmd(conhdl, RSCPC_READ_OP_CODE, RSCPC_RD_RSC_FEAT);
            break;
        case '3':
            APP_RscpcReadCmd(conhdl, RSCPC_READ_OP_CODE, RSCPC_RD_SENSOR_LOC);
            break;
        case '4':
            APP_RscpcReadCmd(conhdl, RSCPC_READ_OP_CODE, RSCPC_RD_WR_RSC_MEAS_CFG);
            break;
        case '5':
            APP_RscpcReadCmd(conhdl, CSCPC_READ_OP_CODE, RSCPC_RD_WR_SC_CTNL_PT_CFG);
            break;
        case '6':
            APP_RscpcCfgNtfindCmd(conhdl, RSCPC_CFG_NTF_IND_OP_CODE, RSCPC_RD_WR_RSC_MEAS_CFG, PRF_CLI_START_NTF);
            break;
        case '7':
            APP_RscpcCfgNtfindCmd(conhdl, RSCPC_CFG_NTF_IND_OP_CODE, RSCPC_RD_WR_RSC_MEAS_CFG, PRF_CLI_STOP_NTFIND);
            break;
        case '8':
            APP_RscpcCfgNtfindCmd(conhdl, RSCPC_CFG_NTF_IND_OP_CODE, RSCPC_RD_WR_SC_CTNL_PT_CFG, PRF_CLI_START_IND);
            break;
        case '9':
            APP_RscpcCfgNtfindCmd(conhdl, RSCPC_CFG_NTF_IND_OP_CODE, RSCPC_RD_WR_SC_CTNL_PT_CFG, PRF_CLI_STOP_NTFIND);
            break;
        case 'a':
            sc_ctnl_pt.op_code = RSCP_CTNL_PT_OP_SET_CUMUL_VAL;
            sc_ctnl_pt.value.cumul_val = 0;
            APP_RscpcCtnlPtCfgReq(conhdl, RSCPC_CTNL_PT_CFG_WR_OP_CODE, sc_ctnl_pt);
            break;
        case 'b':
            sc_ctnl_pt.op_code = RSCP_CTNL_PT_OP_UPD_LOC;
            sc_ctnl_pt.value.sensor_loc = 4;
            APP_RscpcCtnlPtCfgReq(conhdl, RSCPC_CTNL_PT_CFG_WR_OP_CODE, sc_ctnl_pt);
            break;
        case 'c':
            sc_ctnl_pt.op_code = RSCP_CTNL_PT_OP_REQ_SUPP_LOC;
            APP_RscpcCtnlPtCfgReq(conhdl, RSCPC_CTNL_PT_CFG_WR_OP_CODE, sc_ctnl_pt);
            break;
    }
    APP_MenuShow();
}

#endif

/*!
 * @brief Show the BLE demo application.
 */
static void APP_MenuShow(void)
{
    switch (g_AppEnv.menu_id)
    {
        case menu_main:
            APP_MenuShowMain();
            break;
        case menu_gap:
            app_MenuShowGap();
            break;
        case menu_gatt:
            APP_MenuShowGatt();
            break;
#if BLE_QPP_CLIENT
        case menu_qppc:
            APP_MenuShowQppc();
            break;
#endif
#if BLE_OTA_SERVER
        case menu_otac:
            app_menu_show_otac();
            break;
#endif
#if BLE_DIS_CLIENT
        case menu_disc:
            APP_MenuShowDisc();
            break;
#endif
#if BLE_BATT_CLIENT
        case menu_basc:
            APP_MenuShowBasc();
            break;
#endif
#if BLE_GL_COLLECTOR
        case menu_glpc:
            APP_MenuShowGlpc();
            break;
#endif
#if BLE_PROX_MONITOR
        case menu_proxm:
            APP_MenuShowProxm();
            break;
#endif
#if BLE_FINDME_LOCATOR
        case menu_findl:
            APP_MenuShowFindl();
            break;
#endif
#if BLE_HT_COLLECTOR
        case menu_htpc:
            APP_MenuShowHtpc();
            break;
#endif
#if BLE_CP_COLLECTOR
        case menu_cppc:
            APP_MenuShowCppc();
            break;
#endif
#if BLE_CSC_COLLECTOR
        case menu_cscpc:
            APP_MenuShowCscps();
            break;
#endif

#if BLE_LN_COLLECTOR
        case menu_lanc:
            APP_MenuShowLanc();
            break;
#endif

#if BLE_SP_CLIENT
        case menu_scppc:
            APP_MenuShowScppc();
            break;
#endif
#if BLE_PAS_CLIENT
        case menu_paspc:
            APP_MenuShowPaspc();
            break;
#endif
#if BLE_HID_REPORT_HOST
        case menu_hogph:
            APP_MenuShowHogph();
            break;
#endif
#if BLE_HID_BOOT_HOST
        case menu_hogph:
            APP_MenuShowHogph();
            break;
#endif
#if BLE_AN_CLIENT
        case menu_anpc:
            APP_MenuShowAnpc();
            break;
#endif
#if BLE_TIP_CLIENT
        case menu_tipc:
            APP_MenuShowTipc();
            break;
#endif
#if BLE_HR_COLLECTOR
        case menu_hrpc:
            APP_MenuShowHrpc();
            break;
#endif
#if BLE_BP_COLLECTOR
        case menu_blpc:
            APP_MenuShowBlpc();
            break;
#endif
#if BLE_RSC_COLLECTOR
        case menu_rscpc:
            APP_MenuShowRscps();
            break;
#endif
        default:
            break;
    }
}

static void APP_MenuSmpTkInputHandler(void)
{
    struct gap_sec_key tk = {0};
    uint32_t i = atoi((char *)g_AppEnv.input);
    QPRINTF("The input tk is %d\r\n", i);
    if (i <= 999999)
    {
        tk.key[0] = (uint8_t)((i & 0x000000FF) >> 0); /* LSB */
        tk.key[1] = (uint8_t)((i & 0x0000FF00) >> 8);
        tk.key[2] = (uint8_t)((i & 0x00FF0000) >> 16);
        APP_GapcBondCfm(g_AppEnv.conhdl, GAPC_TK_EXCH, true, &tk);
    }
    else
    {
        QPRINTF("TK is valid\r\n");
        APP_GapcBondCfm(g_AppEnv.conhdl, GAPC_TK_EXCH, false, &tk);
    }
    g_AppEnv.menu_id = menu_gap;
}

static void APP_MenuSmpNcHandler(void)
{
    bool compareResult;
    if (g_AppEnv.input[0] == 'Y' || g_AppEnv.input[0] == 'y')
    {
        compareResult = true;
        APP_GapcBondCfm(g_AppEnv.conhdl, GAPC_NC_EXCH, compareResult, NULL);
    }
    else /* Only 'Y' or 'y' for yes, others for no */
    {
        compareResult = false;
        APP_GapcBondCfm(g_AppEnv.conhdl, GAPC_NC_EXCH, compareResult, NULL);
    }

    g_AppEnv.menu_id = menu_gap;
}

/*!
 * @brief Handler function of the BLE demo application.
 */
static void APP_MenuHandler(void)
{
    switch (g_AppEnv.menu_id)
    {
        case menu_main:
            APP_MenuMainHandler();
            break;
        case menu_gap:
            APP_MenuGapHander();
            break;
        case menu_gatt:
            APP_MenuGattHandler();
            break;
#if BLE_QPP_CLIENT
        case menu_qppc:
            APP_MenuQppcHandler();
            break;
#endif
#if BLE_OTA_SERVER
        case menu_otac:
            app_menu_otac_handler();
            break;
#endif
#if BLE_DIS_CLIENT
        case menu_disc:
            APP_MenuDiscHandler();
            break;
#endif
#if BLE_BATT_CLIENT
        case menu_basc:
            APP_MenuBascHandler();
            break;
#endif
#if BLE_GL_COLLECTOR
        case menu_glpc:
            APP_MenuGlpcHandler();
            break;
#endif
#if BLE_PROX_MONITOR
        case menu_proxm:
            APP_MenuProxmHandler();
            break;
#endif
#if BLE_FINDME_LOCATOR
        case menu_findl:
            APP_MenuFindlHandler();
            break;
#endif
#if BLE_HT_COLLECTOR
        case menu_htpc:
            APP_MenuHtpcHandler();
            break;
#endif
#if BLE_CP_COLLECTOR
        case menu_cppc:
            APP_MenuCppcHandler();
            break;
#endif
#if BLE_CSC_COLLECTOR
        case menu_cscpc:
            APP_MenuCscpcHandler();
            break;
#endif

#if BLE_LN_COLLECTOR
        case menu_lanc:
            APP_MenuLancHandler();
            break;
#endif

#if BLE_SP_CLIENT
        case menu_scppc:
            APP_MenuScppcHandler();
            break;

#endif
#if BLE_PAS_CLIENT
        case menu_paspc:
            APP_MenuPaspcHandler();
            break;
#endif
#if BLE_HID_REPORT_HOST
        case menu_hogph:
            APP_MenuHogphHandler();
            break;
#endif
#if BLE_HID_BOOT_HOST
        case menu_hogph:
            APP_MenuHogphHandler();
            break;
#endif
#if BLE_AN_CLIENT
        case menu_anpc:
            APP_MenuAnpcHandler();
            break;
#endif
#if BLE_TIP_CLIENT
        case menu_tipc:
            APP_MenuTipcHandler();
            break;
#endif
#if BLE_HR_COLLECTOR
        case menu_hrpc:
            APP_MenuHrpcHandler();
            break;
#if BLE_BP_COLLECTOR
        case menu_blpc:
            APP_MenuBlpcHandler();
            break;
#endif
#if BLE_RSC_COLLECTOR
        case menu_rscpc:
            APP_MenuRscpcHandler();
            break;
#endif
#endif

        case menu_gap_pair:
        case menu_gap_encrypt:
        case menu_gap_security:
        case menu_gap_unbond:
        case menu_gap_create_connection:
        case menu_gap_disconnection:
        case menu_gatt_exc_mtu:
        case menu_qppc_enable:
        case menu_otac_enable:
        case menu_disc_enable:
        case menu_basc_enable:
        case menu_glpc_enable:
        case menu_proxm_enable:
        case menu_findl_enable:
        case menu_htpc_enable:
        case menu_cppc_enable:
        case menu_cscpc_enable:
        case menu_lanc_enable:
        case menu_scppc_enable:
        case menu_paspc_enable:
        case menu_hogprh_enable:
        case menu_hogpbh_enable:
        case menu_anpc_enable:
        case menu_tipc_enable:
        case menu_hrpc_enable:
        case menu_blpc_enable:
        case menu_rscpc_enable:
            APP_MenuGapDevSelectHandler();
            break;
        case menu_smp_tk_input:
            APP_MenuSmpTkInputHandler();
            break;
        case menu_smp_nc:
            APP_MenuSmpNcHandler();
            break;
        default:
            break;
    }
}

void APP_MenuShowStart(void)
{
    g_AppEnv.menu_id = menu_main;
    APP_MenuShow();
}

void APP_MenuShowTkInput(void)
{
    g_AppEnv.menu_id = menu_smp_tk_input;
    QPRINTF("Please input the passkey:\r\n");
}

void APP_MenuShowNumericCompare(void)
{
    g_AppEnv.menu_id = menu_smp_nc;
    QPRINTF("Please compare numeric Y or N:\r\n");
}
#endif

/*! @brief @} APP */

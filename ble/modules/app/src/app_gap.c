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

#ifdef MBED_PORT
#include "ke_mem.h"
#include "gap_func.h"
#include "ke_mem.h"
extern void initComplete(void);
extern void connection_start(struct gapc_connection_req_ind const *param);
extern void connection_end(uint16_t conhdl, uint8_t reason);
#endif

#if (BLE_APP_PRESENT)

/*******************************************************************************
 * Variables
 ******************************************************************************/
/*! @brief Default State handlers definition */
const struct ke_msg_handler s_appGapMsgHandlerList[] = {
    /* GAPM */
    {GAPM_CMP_EVT, (ke_msg_func_t)APP_GapmCmpEvtHandler},
    {GAPM_DEVICE_READY_IND, (ke_msg_func_t)APP_GapmDeviceReadyIndHandler},
    {GAPM_DEV_VERSION_IND, (ke_msg_func_t)APP_GapmDevVersionIndHandler},
    {GAPM_DEV_BDADDR_IND, (ke_msg_func_t)APP_GapmDevBdAddrIndHandler},
    {GAPM_DEV_ADV_TX_POWER_IND, (ke_msg_func_t)APP_GapmDevAdvTxPowerIndHandler},
    {GAPM_WHITE_LIST_SIZE_IND, (ke_msg_func_t)APP_GapmWhiteListSizeIndHandler},
#if (BLE_CENTRAL)
    {GAPM_ADV_REPORT_IND, (ke_msg_func_t)APP_GapmAdvReportIndHandler},
#endif
    {GAPM_PEER_NAME_IND, (ke_msg_func_t)APP_GapmPeerNameIndHandler},
    {GAPM_ADDR_SOLVED_IND, (ke_msg_func_t)APP_GapmAddrSolvedIndHandler},
    {GAPM_USE_ENC_BLOCK_IND, (ke_msg_func_t)APP_GapmUseEncBlockIndHandler},
#if (BLE_PROFILES)
    {GAPM_PROFILE_ADDED_IND, (ke_msg_func_t)APP_GapmProfileAddedIndHandler},
#endif
    {GAPM_SUGG_DFLT_DATA_LEN_IND, (ke_msg_func_t)APP_GapmSuggDfltDataLenIndHandler},
    {GAPM_MAX_DATA_LEN_IND, (ke_msg_func_t)APP_GapmMaxDataLenIndHandler},
    {GAPM_RAL_SIZE_IND, (ke_msg_func_t)APP_GapmRalSizeIndHandler},
    {GAPM_RAL_ADDR_IND, (ke_msg_func_t)APP_GapmRalAddrIndHandler},

#if (BLE_CENTRAL || BLE_PERIPHERAL)
    /* GAPC */
    {GAPC_CMP_EVT, (ke_msg_func_t)APP_GapcCmpEvtHandler},
    {GAPC_CONNECTION_REQ_IND, (ke_msg_func_t)APP_GapcConnectionReqIndHandler},
    {GAPC_DISCONNECT_IND, (ke_msg_func_t)APP_GapcDisconnectIndHandler},
    {GAPC_PEER_ATT_INFO_IND, (ke_msg_func_t)APP_GapcPeerAttInfoIndHandler},
    {GAPC_PEER_VERSION_IND, (ke_msg_func_t)APP_GapcPeerVersionIndHandler},
    {GAPC_PEER_FEATURES_IND, (ke_msg_func_t)APP_GapcPeerFeaturesIndHandler},
    {GAPC_CON_RSSI_IND, (ke_msg_func_t)APP_GapcConRssiIndHandler},
    {GAPC_GET_DEV_INFO_REQ_IND, (ke_msg_func_t)APP_GapcGetDevInfoReqIndHandler},
    {GAPC_SET_DEV_INFO_REQ_IND, (ke_msg_func_t)APP_GapcSetDevInfoReqIndHandler},
    {GAPC_PARAM_UPDATE_REQ_IND, (ke_msg_func_t)APP_GapcParamUpdateReqIndHandler},
    {GAPC_PARAM_UPDATED_IND, (ke_msg_func_t)APP_GapcParamUpdatedIndHandler},
    {GAPC_BOND_REQ_IND, (ke_msg_func_t)APP_GapcBondReqIndHandler},
    {GAPC_BOND_IND, (ke_msg_func_t)APP_GapcBondIndHandler},
    {GAPC_ENCRYPT_REQ_IND, (ke_msg_func_t)APP_GapcEncryptReqIndHandler},
    {GAPC_ENCRYPT_IND, (ke_msg_func_t)APP_GapcEncryptIndHandler},
    {GAPC_SECURITY_IND, (ke_msg_func_t)APP_GapcSecurityIndHandler},
    {GAPC_SIGN_COUNTER_IND, (ke_msg_func_t)APP_GapcSignCounterIndHandler},
    {GAPC_CON_CHANNEL_MAP_IND, (ke_msg_func_t)APP_GapcConChannelMapIndHandler},
    {GAPC_LE_PING_TO_VAL_IND, (ke_msg_func_t)APP_GapcLePingToValIndHandler},
    {GAPC_LE_PKT_SIZE_IND, (ke_msg_func_t)APP_GapcLePktSizeIndHandler},
#endif
};

const struct ke_state_handler g_AppGapTableHandler = KE_STATE_HANDLER(s_appGapMsgHandlerList);

/*******************************************************************************
 * Code
 ******************************************************************************/
void APP_GapmResetCmd(void)
{
    struct gapm_reset_cmd *cmd = KE_MSG_ALLOC(GAPM_RESET_CMD, TASK_GAPM, TASK_APP, gapm_reset_cmd);

    cmd->operation = GAPM_RESET;

    APP_MsgSend(cmd);
}

void APP_GapmCancelCmd(void)
{
    struct gapm_cancel_cmd *cmd = KE_MSG_ALLOC(GAPM_CANCEL_CMD, TASK_GAPM, TASK_APP, gapm_cancel_cmd);
    cmd->operation = GAPM_CANCEL;

    APP_MsgSend(cmd);
}

void APP_GapmStopAdvertising(void)
{
    struct gapm_cancel_cmd *cmd = KE_MSG_ALLOC(GAPM_CANCEL_CMD, TASK_GAPM, TASK_APP, gapm_cancel_cmd);
    cmd->operation = GAPM_CANCEL;

    APP_MsgSend(cmd);
}

void APP_GapmStopScanning(void)
{
    struct gapm_cancel_cmd *cmd = KE_MSG_ALLOC(GAPM_CANCEL_CMD, TASK_GAPM, TASK_APP, gapm_cancel_cmd);
    cmd->operation = GAPM_CANCEL;

    APP_MsgSend(cmd);
}

void APP_GapmStopConnecting(void)
{
    struct gapm_cancel_cmd *cmd = KE_MSG_ALLOC(GAPM_CANCEL_CMD, TASK_GAPM, TASK_APP, gapm_cancel_cmd);
    cmd->operation = GAPM_CANCEL;

    APP_MsgSend(cmd);
}

void APP_GapmSetDevConfigCmd(uint8_t role, uint8_t addr_type, bd_addr_t *addr, struct gap_sec_key *irk)
{
    /* set device configuration */
    struct gapm_set_dev_config_cmd *cmd =
        KE_MSG_ALLOC(GAPM_SET_DEV_CONFIG_CMD, TASK_GAPM, TASK_APP, gapm_set_dev_config_cmd);
    cmd->operation = GAPM_SET_DEV_CONFIG;
    cmd->role = role;

    /* Privacy Config */
    cmd->renew_dur = CFG_ADDR_RENEW_DUR;
    cmd->addr_type = addr_type;
    cmd->addr = *addr;
    cmd->irk = *irk;

    /* ATT Database Config: enable appearance and name write permission */
    cmd->att_cfg = (PERM_RIGHT_ENABLE << GAPM_POS_ATT_APPEARENCE_PERM) | (PERM_RIGHT_ENABLE << GAPM_POS_ATT_NAME_PERM);
    /* SC and PCP present */
    if ((role & GAP_ROLE_PERIPHERAL) == GAP_ROLE_PERIPHERAL)
    {
        cmd->att_cfg |= GAPM_MASK_ATT_SLV_PREF_CON_PAR_EN | GAPM_MASK_ATT_SVC_CHG_EN;
    }
    cmd->gap_start_hdl = 0;
    cmd->gatt_start_hdl = 0;
    cmd->max_mtu = CFG_MAX_MTU;
    cmd->max_mps = CFG_MAX_MPS;

    cmd->pairing_mode = CFG_PAIRING_MODE;
    cmd->sugg_max_tx_octets = CFG_SUGG_MAX_TX_SIZE;
    cmd->sugg_max_tx_time = CFG_SUGG_MAX_TX_TIME;

    APP_MsgSend(cmd);
}

void APP_GapmSetChannelMapCmd(le_chnl_map_t *chmap)
{
    struct gapm_set_channel_map_cmd *cmd =
        KE_MSG_ALLOC(GAPM_SET_CHANNEL_MAP_CMD, TASK_GAPM, TASK_APP, gapm_set_channel_map_cmd);
    ASSERT_ERR(chmap != NULL);

    cmd->operation = GAPM_SET_CHANNEL_MAP;
    cmd->chmap = *chmap;

    APP_MsgSend(cmd);
}

void APP_GapmWhiteListMgtCmd(uint8_t operation, uint8_t nb, struct gap_bdaddr *devices)
{
    struct gapm_white_list_mgt_cmd *cmd = KE_MSG_ALLOC_DYN(GAPM_WHITE_LIST_MGT_CMD, TASK_GAPM, TASK_APP,
                                                           gapm_white_list_mgt_cmd, nb * sizeof(struct gap_bdaddr));
    cmd->operation = operation;
    cmd->nb = nb;

    ASSERT_ERR(nb > 0 ? devices != NULL : 1);
    memcpy(cmd->devices, devices, nb * sizeof(struct gap_bdaddr));

    APP_MsgSend(cmd);
}

void APP_GapmRalMgtCmd(uint8_t operation, uint8_t nb, struct gap_ral_dev_info *devices)
{
    struct gapm_ral_mgt_cmd *cmd =
        KE_MSG_ALLOC_DYN(GAPM_RAL_MGT_CMD, TASK_GAPM, TASK_APP, gapm_ral_mgt_cmd, nb * sizeof(struct gap_ral_dev_info));
    cmd->operation = operation;
    cmd->nb = nb;

    ASSERT_ERR(nb > 0 ? devices != NULL : 1);
    memcpy(cmd->devices, devices, nb * sizeof(struct gap_ral_dev_info));

    APP_MsgSend(cmd);
}

void APP_GapmGetDevInfoCmd(uint8_t operation)
{
    struct gapm_get_dev_info_cmd *cmd = KE_MSG_ALLOC(GAPM_GET_DEV_INFO_CMD, TASK_GAPM, TASK_APP, gapm_get_dev_info_cmd);
    cmd->operation = operation;

    APP_MsgSend(cmd);
}

void APP_GapmGetDevVersion(void)
{
    struct gapm_get_dev_info_cmd *cmd = KE_MSG_ALLOC(GAPM_GET_DEV_INFO_CMD, TASK_GAPM, TASK_APP, gapm_get_dev_info_cmd);
    cmd->operation = GAPM_GET_DEV_VERSION;

    APP_MsgSend(cmd);
}

void APP_GapmGetDevBdAddr(void)
{
    struct gapm_get_dev_info_cmd *cmd = KE_MSG_ALLOC(GAPM_GET_DEV_INFO_CMD, TASK_GAPM, TASK_APP, gapm_get_dev_info_cmd);
    cmd->operation = GAPM_GET_DEV_BDADDR;

    APP_MsgSend(cmd);
}

void APP_GapmGetDevAdvTxPower(void)
{
    struct gapm_get_dev_info_cmd *cmd = KE_MSG_ALLOC(GAPM_GET_DEV_INFO_CMD, TASK_GAPM, TASK_APP, gapm_get_dev_info_cmd);
    cmd->operation = GAPM_GET_DEV_ADV_TX_POWER;

    APP_MsgSend(cmd);
}

void APP_GapmGetSuggestedDeflLeDataLen(void)
{
    struct gapm_get_dev_info_cmd *cmd = KE_MSG_ALLOC(GAPM_GET_DEV_INFO_CMD, TASK_GAPM, TASK_APP, gapm_get_dev_info_cmd);
    cmd->operation = GAPM_GET_SUGGESTED_DFLT_LE_DATA_LEN;

    APP_MsgSend(cmd);
}

void APP_GapmGetMaxLeDataLen(void)
{
    struct gapm_get_dev_info_cmd *cmd = KE_MSG_ALLOC(GAPM_GET_DEV_INFO_CMD, TASK_GAPM, TASK_APP, gapm_get_dev_info_cmd);
    cmd->operation = GAPM_GET_MAX_LE_DATA_LEN;

    APP_MsgSend(cmd);
}

void APP_GapmResolvAddrCmd(uint8_t nb_key, bd_addr_t *addr, struct gap_sec_key *irk)
{
    struct gapm_resolv_addr_cmd *cmd = KE_MSG_ALLOC_DYN(GAPM_RESOLV_ADDR_CMD, TASK_GAPM, TASK_APP, gapm_resolv_addr_cmd,
                                                        nb_key * sizeof(struct gap_sec_key));
    ASSERT_ERR(nb_key != 0);
    ASSERT_ERR(addr != NULL);
    ASSERT_ERR(irk != NULL);

    /* Resolve device address */
    cmd->operation = GAPM_RESOLV_ADDR;
    cmd->nb_key = nb_key;
    cmd->addr = *addr;
    memcpy(cmd->irk, irk, nb_key * sizeof(struct gap_sec_key));

    APP_MsgSend(cmd);
}

void APP_GapmGenRandAddrCmd(uint8_t rnd_type)
{
    struct gapm_gen_rand_addr_cmd *cmd =
        KE_MSG_ALLOC(GAPM_GEN_RAND_ADDR_CMD, TASK_GAPM, TASK_APP, gapm_gen_rand_addr_cmd);
    cmd->operation = GAPM_GEN_RAND_ADDR;
    cmd->rnd_type = rnd_type;

    APP_MsgSend(cmd);
}

void APP_GapmUseEncBlockCmd(uint8_t key[16], uint8_t data[16])
{
    struct gapm_use_enc_block_cmd *cmd =
        KE_MSG_ALLOC(GAPM_USE_ENC_BLOCK_CMD, TASK_GAPM, TASK_APP, gapm_use_enc_block_cmd);
    cmd->operation = GAPM_USE_ENC_BLOCK;
    ASSERT_ERR(key != NULL);
    memcpy(&cmd->operand_1[0], key, GAP_KEY_LEN);
    ASSERT_ERR(data != NULL);
    memcpy(&cmd->operand_2[0], data, GAP_KEY_LEN);

    APP_MsgSend(cmd);
}

void APP_GapmSetIrkCmd(struct gap_sec_key *irk)
{
    struct gapm_set_irk_cmd *cmd = KE_MSG_ALLOC(GAPM_SET_IRK_CMD, TASK_GAPM, TASK_APP, gapm_set_irk_cmd);
    cmd->operation = GAPM_SET_IRK;
    ASSERT_ERR(irk != NULL);
    cmd->irk = *irk;

    APP_MsgSend(cmd);
}

#if (BLE_PERIPHERAL || BLE_BROADCASTER)
void APP_GapmStartAdvertiseCmd(uint8_t code,
                               uint8_t addr_src,
                               uint8_t mode,
                               uint16_t intv_min,
                               uint16_t intv_max,
                               uint8_t adv_data_len,
                               uint8_t *adv_data,
                               uint8_t scan_rsp_data_len,
                               uint8_t *scan_rsp_data,
                               struct gap_bdaddr *peer_addr)
{
    struct gapm_start_advertise_cmd *cmd =
        KE_MSG_ALLOC(GAPM_START_ADVERTISE_CMD, TASK_GAPM, TASK_APP, gapm_start_advertise_cmd);

    cmd->op.code = code;
    cmd->op.addr_src = addr_src;

    cmd->intv_min = intv_min;
    cmd->intv_max = intv_max;
    cmd->channel_map = CFG_ADV_CHMAP;

    if ((code == GAPM_ADV_NON_CONN) || (code == GAPM_ADV_UNDIRECT))
    {
        cmd->info.host.mode = mode;
        cmd->info.host.adv_filt_policy = ADV_ALLOW_SCAN_ANY_CON_ANY;
        cmd->info.host.adv_data_len = adv_data_len;
        memcpy(&cmd->info.host.adv_data[0], adv_data, adv_data_len);
        cmd->info.host.scan_rsp_data_len = scan_rsp_data_len;
        memcpy(&cmd->info.host.scan_rsp_data[0], scan_rsp_data, scan_rsp_data_len);
        /* To find out the local IRK from resolving list by a specified peer address */
        cmd->info.host.peer_addr = *peer_addr;
    }
    else
    { /* Direct advertising */
        ASSERT_ERR(peer_addr != NULL);
        cmd->info.direct = *peer_addr;
    }

    g_AppEnv.state = APP_OP_ADVERTISING;
    APP_MsgSend(cmd);
}

void APP_GapmUpdateAdvertiseDataCmd(uint8_t adv_data_len,
                                    uint8_t *adv_data,
                                    uint8_t scan_rsp_data_len,
                                    uint8_t *scan_rsp_data)
{
    struct gapm_update_advertise_data_cmd *cmd =
        KE_MSG_ALLOC(GAPM_UPDATE_ADVERTISE_DATA_CMD, TASK_GAPM, TASK_APP, gapm_update_advertise_data_cmd);

    cmd->operation = GAPM_UPDATE_ADVERTISE_DATA;
    cmd->adv_data_len = adv_data_len;
    ASSERT_ERR(adv_data_len > 0 ? adv_data != NULL : 1);
    memcpy(&cmd->adv_data[0], adv_data, adv_data_len);

    cmd->scan_rsp_data_len = scan_rsp_data_len;
    ASSERT_ERR(scan_rsp_data_len > 0 ? scan_rsp_data != NULL : 1);
    memcpy(&cmd->scan_rsp_data[0], scan_rsp_data, scan_rsp_data_len);

    APP_MsgSend(cmd);
}
#endif

#if (BLE_CENTRAL)
void APP_GapmStartScanCmd(uint8_t code,
                          uint8_t addr_src,
                          uint8_t mode,
                          uint16_t interval,
                          uint16_t window,
                          uint8_t filt_policy,
                          uint8_t filter_duplic)
{
    struct gapm_start_scan_cmd *cmd = KE_MSG_ALLOC(GAPM_START_SCAN_CMD, TASK_GAPM, TASK_APP, gapm_start_scan_cmd);
    cmd->op.code = code;
    cmd->op.addr_src = addr_src;

    cmd->interval = interval;
    cmd->window = window;
    cmd->mode = mode;
    cmd->filt_policy = filt_policy;
    cmd->filter_duplic = filter_duplic;

    g_AppEnv.scan_count = 0;
    g_AppEnv.state = APP_OP_SCANING;

    APP_MsgSend(cmd);
}

void APP_GapmStartConnectionCmd(uint8_t code,
                                uint8_t addr_src,
                                uint16_t con_intv_min,
                                uint16_t con_intv_max,
                                uint16_t con_latency,
                                uint16_t superv_to,
                                uint8_t nb_peers,
                                struct gap_bdaddr *peer)
{
    struct gapm_start_connection_cmd *cmd =
        KE_MSG_ALLOC_DYN(GAPM_START_CONNECTION_CMD, TASK_GAPM, TASK_APP, gapm_start_connection_cmd,
                         nb_peers * sizeof(struct gap_bdaddr));

    cmd->op.code = code;
    cmd->op.addr_src = addr_src;

    /* Scan interval, Value Time = N * 0.625 ms */
    cmd->scan_interval = CFG_SCAN_FAST_INTV;
    /* Scan window size, Value Time = N * 0.625 ms */
    cmd->scan_window = CFG_SCAN_FAST_WIND;
    cmd->con_intv_min = con_intv_min;
    cmd->con_intv_max = con_intv_max;
    cmd->con_latency = con_latency;
    cmd->superv_to = superv_to;
    /* Minimum CE length, Value Time = N * 0.625 ms */
    cmd->ce_len_min = CFG_CONN_MIN_CE;
    /* Maximum CE length, Value Time = N * 0.625 ms */
    cmd->ce_len_max = CFG_CONN_MAX_CE;
    cmd->nb_peers = nb_peers;
    ASSERT_ERR(nb_peers > 0 ? peer != NULL : 1);
    memcpy(cmd->peers, peer, nb_peers * sizeof(struct gap_bdaddr));

    g_AppEnv.state = APP_OP_INITIATING;

    APP_MsgSend(cmd);
}
#endif

void APP_GapmConnectionCfm(
    struct gap_bdaddr *peer, uint16_t con_intv_min, uint16_t con_intv_max, uint16_t con_latency, uint16_t superv_to)
{
    struct gapm_connection_cfm *cfm = KE_MSG_ALLOC(GAPM_CONNECTION_CFM, TASK_GAPM, TASK_APP, gapm_connection_cfm);
    ASSERT_ERR(peer != NULL);

    cfm->addr = peer->addr;
    cfm->addr_type = peer->addr_type;

    cfm->con_intv_min = con_intv_min;
    cfm->con_intv_max = con_intv_max;
    cfm->con_latency = con_latency;
    cfm->superv_to = superv_to;
    cfm->ce_len_min = CFG_CONN_MIN_CE;
    cfm->ce_len_max = CFG_CONN_MAX_CE;

    APP_MsgSend(cfm);
}

#if (BLE_CENTRAL || BLE_PERIPHERAL)
void APP_GapcConnectionCfm(uint16_t conhdl,
                           struct gap_sec_key *lcsrk,
                           uint32_t lsign_counter,
                           struct gap_sec_key *rcsrk,
                           uint32_t rsign_counter,
                           uint8_t auth,
                           uint8_t svc_changed_ind_enable,
                           bool ltk_present)
{
    struct gapc_connection_cfm *cfm =
        KE_MSG_ALLOC(GAPC_CONNECTION_CFM, KE_BUILD_ID(TASK_GAPC, CONHDL2CONIDX(conhdl)), TASK_APP, gapc_connection_cfm);

    cfm->lcsrk = *lcsrk;
    cfm->lsign_counter = lsign_counter;
    cfm->rcsrk = *rcsrk;
    cfm->rsign_counter = rsign_counter;
    cfm->auth = auth;
    cfm->svc_changed_ind_enable = svc_changed_ind_enable;
    cfm->ltk_present = ltk_present;

    APP_MsgSend(cfm);
}

#ifdef MBED_PORT
void APP_GapcDisconnectCmd(uint16_t conhdl, uint8_t reason_code)
{
    struct gapc_disconnect_cmd *cmd =
        KE_MSG_ALLOC(GAPC_DISCONNECT_CMD, KE_BUILD_ID(TASK_GAPC, CONHDL2CONIDX(conhdl)), TASK_APP, gapc_disconnect_cmd);

    cmd->operation = GAPC_DISCONNECT;
    cmd->reason = reason_code; // CO_ERROR_REMOTE_USER_TERM_CON;

    APP_MsgSend(cmd);
}
#else
void APP_GapcDisconnectCmd(uint16_t conhdl)
{
    struct gapc_disconnect_cmd *cmd =
        KE_MSG_ALLOC(GAPC_DISCONNECT_CMD, KE_BUILD_ID(TASK_GAPC, CONHDL2CONIDX(conhdl)), TASK_APP, gapc_disconnect_cmd);

    cmd->operation = GAPC_DISCONNECT;
    cmd->reason = CO_ERROR_REMOTE_USER_TERM_CON;

    APP_MsgSend(cmd);
}
#endif

void APP_GapcGetInfoCmd(uint16_t conhdl, uint8_t operation)
{
    struct gapc_get_info_cmd *cmd =
        KE_MSG_ALLOC(GAPC_GET_INFO_CMD, KE_BUILD_ID(TASK_GAPC, CONHDL2CONIDX(conhdl)), TASK_APP, gapc_get_info_cmd);

    cmd->operation = operation;

    APP_MsgSend(cmd);
}

void APP_GapcGetPeerName(uint16_t conhdl)
{
    struct gapc_get_info_cmd *cmd =
        KE_MSG_ALLOC(GAPC_GET_INFO_CMD, KE_BUILD_ID(TASK_GAPC, CONHDL2CONIDX(conhdl)), TASK_APP, gapc_get_info_cmd);

    cmd->operation = GAPC_GET_PEER_NAME;

    APP_MsgSend(cmd);
}

void APP_GapcGetPeerVersion(uint16_t conhdl)
{
    struct gapc_get_info_cmd *cmd =
        KE_MSG_ALLOC(GAPC_GET_INFO_CMD, KE_BUILD_ID(TASK_GAPC, CONHDL2CONIDX(conhdl)), TASK_APP, gapc_get_info_cmd);

    cmd->operation = GAPC_GET_PEER_VERSION;

    APP_MsgSend(cmd);
}

void APP_GapcGetPeerFeatures(uint16_t conhdl)
{
    struct gapc_get_info_cmd *cmd =
        KE_MSG_ALLOC(GAPC_GET_INFO_CMD, KE_BUILD_ID(TASK_GAPC, CONHDL2CONIDX(conhdl)), TASK_APP, gapc_get_info_cmd);

    cmd->operation = GAPC_GET_PEER_FEATURES;

    APP_MsgSend(cmd);
}

void APP_GapcGetPeerAppearance(uint16_t conhdl)
{
    struct gapc_get_info_cmd *cmd =
        KE_MSG_ALLOC(GAPC_GET_INFO_CMD, KE_BUILD_ID(TASK_GAPC, CONHDL2CONIDX(conhdl)), TASK_APP, gapc_get_info_cmd);

    cmd->operation = GAPC_GET_PEER_APPEARANCE;

    APP_MsgSend(cmd);
}

void APP_GapcGetPeerSlvPrefParams(uint16_t conhdl)
{
    struct gapc_get_info_cmd *cmd =
        KE_MSG_ALLOC(GAPC_GET_INFO_CMD, KE_BUILD_ID(TASK_GAPC, CONHDL2CONIDX(conhdl)), TASK_APP, gapc_get_info_cmd);

    cmd->operation = GAPC_GET_PEER_SLV_PREF_PARAMS;

    APP_MsgSend(cmd);
}

void APP_GapcGetConRssi(uint16_t conhdl)
{
    struct gapc_get_info_cmd *cmd =
        KE_MSG_ALLOC(GAPC_GET_INFO_CMD, KE_BUILD_ID(TASK_GAPC, CONHDL2CONIDX(conhdl)), TASK_APP, gapc_get_info_cmd);

    cmd->operation = GAPC_GET_CON_RSSI;

    APP_MsgSend(cmd);
}

void APP_GapcGetConChannelMap(uint16_t conhdl)
{
    struct gapc_get_info_cmd *cmd =
        KE_MSG_ALLOC(GAPC_GET_INFO_CMD, KE_BUILD_ID(TASK_GAPC, CONHDL2CONIDX(conhdl)), TASK_APP, gapc_get_info_cmd);

    cmd->operation = GAPC_GET_CON_CHANNEL_MAP;

    APP_MsgSend(cmd);
}

void APP_GapcGetLePingTo(uint16_t conhdl)
{
    struct gapc_get_info_cmd *cmd =
        KE_MSG_ALLOC(GAPC_GET_INFO_CMD, KE_BUILD_ID(TASK_GAPC, CONHDL2CONIDX(conhdl)), TASK_APP, gapc_get_info_cmd);

    cmd->operation = GAPC_GET_LE_PING_TO;

    APP_MsgSend(cmd);
}

void APP_GapcSetLePingToCmd(uint16_t conhdl, uint16_t timeout)
{
    struct gapc_set_le_ping_to_cmd *cmd = KE_MSG_ALLOC(
        GAPC_SET_LE_PING_TO_CMD, KE_BUILD_ID(TASK_GAPC, CONHDL2CONIDX(conhdl)), TASK_APP, gapc_set_le_ping_to_cmd);
    cmd->operation = GAPC_SET_LE_PING_TO;
    cmd->timeout = timeout;

    APP_MsgSend(cmd);
}

void APP_GapcSetLePktSizeCmd(uint16_t conhdl, uint8_t tx_octets)
{
    struct gapc_set_le_pkt_size_cmd *cmd = KE_MSG_ALLOC(
        GAPC_SET_LE_PKT_SIZE_CMD, KE_BUILD_ID(TASK_GAPC, CONHDL2CONIDX(conhdl)), TASK_APP, gapc_set_le_pkt_size_cmd);
    cmd->operation = GAPC_SET_LE_PKT_SIZE;
    cmd->tx_octets = tx_octets;
    cmd->tx_time = (tx_octets * 8 + 112);

    APP_MsgSend(cmd);
}

void APP_GapcKeyPressNotifCmd(uint16_t conhdl, uint8_t notification_type)
{
    struct gapc_key_press_notif_cmd *cmd =
        KE_MSG_ALLOC(GAPC_KEY_PRESS_NOTIFICATION_CMD, KE_BUILD_ID(TASK_GAPC, CONHDL2CONIDX(conhdl)), TASK_APP,
                     gapc_key_press_notif_cmd);
    cmd->operation = GAPC_KEY_PRESS_NOTIFICATION;
    cmd->notification_type = notification_type;

    APP_MsgSend(cmd);
}

void APP_GapcParamUpdateCmd(uint16_t conhdl, struct gapc_conn_param *params)
{
    struct gapc_param_update_cmd *cmd = KE_MSG_ALLOC(
        GAPC_PARAM_UPDATE_CMD, KE_BUILD_ID(TASK_GAPC, CONHDL2CONIDX(conhdl)), TASK_APP, gapc_param_update_cmd);

    ASSERT_ERR(params != NULL);

    cmd->operation = GAPC_UPDATE_PARAMS;
    cmd->intv_min = params->intv_min;
    cmd->intv_max = params->intv_max;
    cmd->latency = params->latency;
    cmd->time_out = params->time_out;
    cmd->ce_len_min = CFG_CONN_MIN_CE;
    cmd->ce_len_max = CFG_CONN_MAX_CE;

    APP_MsgSend(cmd);
}

void APP_GapcParamUpdateCfm(uint16_t conhdl, bool accept)
{
    struct gapc_param_update_cfm *cmd = KE_MSG_ALLOC(
        GAPC_PARAM_UPDATE_CFM, KE_BUILD_ID(TASK_GAPC, CONHDL2CONIDX(conhdl)), TASK_APP, gapc_param_update_cfm);
    cmd->accept = accept;

    APP_MsgSend(cmd);
}

void APP_GapcBondCmd(uint16_t conhdl, struct gapc_pairing *pairing)
{
    struct gapc_bond_cmd *cmd =
        KE_MSG_ALLOC(GAPC_BOND_CMD, KE_BUILD_ID(TASK_GAPC, CONHDL2CONIDX(conhdl)), TASK_APP, gapc_bond_cmd);
    ASSERT_ERR(pairing != NULL);

    cmd->operation = GAPC_BOND;
    cmd->pairing = *pairing;

    APP_MsgSend(cmd);
}

void APP_GapcBondCfm(uint16_t conhdl, uint8_t request, uint8_t accept, void *pairing_info)
{
    struct gapc_bond_cfm *cmd =
        KE_MSG_ALLOC(GAPC_BOND_CFM, KE_BUILD_ID(TASK_GAPC, CONHDL2CONIDX(conhdl)), TASK_APP, gapc_bond_cfm);

    if (request != GAPC_NC_EXCH)
    {
        ASSERT_ERR(pairing_info != NULL);
    }

    cmd->request = request;
    cmd->accept = accept;
    switch (request)
    {
        case GAPC_PAIRING_RSP:
            cmd->data.pairing_feat = *(struct gapc_pairing *)pairing_info;
            break;
        case GAPC_TK_EXCH:
            cmd->data.tk = *(struct gap_sec_key *)pairing_info;
            break;
        case GAPC_CSRK_EXCH:
            cmd->data.csrk = *(struct gap_sec_key *)pairing_info;
            break;
        case GAPC_LTK_EXCH:
            cmd->data.ltk = *(struct gapc_ltk *)pairing_info;
            break;
        case GAPC_IRK_EXCH:
            cmd->data.irk = *(struct gapc_irk *)pairing_info;
            break;
        case GAPC_NC_EXCH:
            break;
        default:
            ASSERT_ERR(0);
            break;
    }

    APP_MsgSend(cmd);
}

void APP_GapcEncryptCmd(uint16_t conhdl, struct gapc_ltk *ltk)
{
    struct gapc_encrypt_cmd *cmd =
        KE_MSG_ALLOC(GAPC_ENCRYPT_CMD, KE_BUILD_ID(TASK_GAPC, CONHDL2CONIDX(conhdl)), TASK_APP, gapc_encrypt_cmd);

    ASSERT_ERR(ltk != NULL);

    cmd->operation = GAPC_ENCRYPT;
    cmd->ltk = *ltk;

    APP_MsgSend(cmd);
}

void APP_GapcEncryptCfm(uint16_t conhdl, uint8_t found, struct gapc_ltk *ltk)
{
    struct gapc_encrypt_cfm *cmd =
        KE_MSG_ALLOC(GAPC_ENCRYPT_CFM, KE_BUILD_ID(TASK_GAPC, CONHDL2CONIDX(conhdl)), TASK_APP, gapc_encrypt_cfm);

    cmd->found = found;
    if (found && ltk)
    {
        cmd->ltk = ltk->ltk;
        cmd->key_size = ltk->key_size;
    }

    APP_MsgSend(cmd);
}

void APP_GapcSecurityCmd(uint16_t conhdl, uint8_t auth)
{
    struct gapc_security_cmd *cmd =
        KE_MSG_ALLOC(GAPC_SECURITY_CMD, KE_BUILD_ID(TASK_GAPC, CONHDL2CONIDX(conhdl)), TASK_APP, gapc_security_cmd);

    cmd->operation = GAPC_SECURITY_REQ;
    cmd->auth = auth;

    APP_MsgSend(cmd);
}
#endif

int APP_GapmCmpEvtHandler(ke_msg_id_t const msgid,
                          struct gapm_cmp_evt const *param,
                          ke_task_id_t const dest_id,
                          ke_task_id_t const src_id)
{
    switch (param->operation)
    {
        /* reset completed */
        case GAPM_RESET:
        {
            ASSERT_ERR(param->status == GAP_ERR_NO_ERROR);
            APP_GapmGetDevBdAddr();
        }
        break;

        case GAPM_CANCEL:
        {
            QPRINTF("Cancel result 0x%02x.\r\n", param->status);
        }
        break;

        /* device configuration updated */
        case GAPM_SET_DEV_CONFIG:
        {
            ASSERT_ERR(param->status == GAP_ERR_NO_ERROR);
            if (ke_state_get(dest_id) == APP_STATE_INIT)
            {
#ifndef MBED_PORT
                /* Start to create task for specific profile */
                APP_AddPrfTask();
#else
                initComplete();
#endif
            }
        }
        break;

        case GAPM_GET_DEV_VERSION:
        {
            ASSERT_ERR(param->status == GAP_ERR_NO_ERROR);
        }
        break;

        case GAPM_GET_DEV_BDADDR:
        {
            ASSERT_ERR(param->status == GAP_ERR_NO_ERROR);
#ifndef MBED_PORT
            APP_GapSetDevCfgCallback();
#else
            /* set device configuration */
            APP_GapmSetDevConfigCmd(CFG_GAP_ROLE, GAPM_CFG_ADDR_PUBLIC, NULL, &g_AppEnv.irk);
#endif
        }
        break;

        case GAPM_GET_DEV_ADV_TX_POWER:
        {
            ASSERT_ERR(param->status == GAP_ERR_NO_ERROR);
        }
        break;

        case GAPM_GET_SUGGESTED_DFLT_LE_DATA_LEN:
        {
            ASSERT_ERR(param->status == GAP_ERR_NO_ERROR);
        }
        break;

        case GAPM_GET_MAX_LE_DATA_LEN:
        {
            ASSERT_ERR(param->status == GAP_ERR_NO_ERROR);
        }
        break;

        case GAPM_PROFILE_TASK_ADD:
        {
            ASSERT_ERR(param->status == GAP_ERR_NO_ERROR);
        }
        break;

        /* Advertising */
        case GAPM_ADV_UNDIRECT:
        case GAPM_ADV_DIRECT:
        case GAPM_ADV_DIRECT_LDC:
        case GAPM_ADV_NON_CONN:
        {
            if (g_AppEnv.state != APP_OP_CONNECTING)
                g_AppEnv.state = APP_OP_NOP;
            APP_AdvertisingStopCallback(param->status);
        }
        break;

        case GAPM_UPDATE_ADVERTISE_DATA:
            QPRINTF("Advertising data update status: 0x%02x.\r\n", param->status);
            break;

        case GAPM_SCAN_ACTIVE:
        case GAPM_SCAN_PASSIVE:
        {
            g_AppEnv.state = APP_OP_NOP;
            APP_ScanningStopCallback(param->status);
        }
        break;

        case GAPM_CONNECTION_DIRECT:
        case GAPM_CONNECTION_AUTO:
        case GAPM_CONNECTION_SELECTIVE:
        case GAPM_CONNECTION_GENERAL:
        {
            if (g_AppEnv.state != APP_OP_CONNECTING)
                g_AppEnv.state = APP_OP_NOP;
            APP_InitiatingStopCallback(param->status);
        }
        break;

        case GAPM_CONNECTION_NAME_REQUEST:
        {
            QPRINTF("Name request from peer(0x%02x).\r\n", param->status);
        }
        break;

        case GAPM_RESOLV_ADDR:
#if (BLE_CENTRAL || BLE_PERIPHERAL)
        {
            if (param->status == GAP_ERR_NOT_FOUND)
            {
                if (g_AppEnv.state == APP_OP_CONNECTING)
                {
                    g_AppEnv.state = APP_OP_NOP;
                    /* No bond info found */
                    APP_GapcConnectionCfm(g_AppEnv.conhdl, NULL, 0, NULL, 0, GAP_AUTH_REQ_NO_MITM_NO_BOND, false,
                                          false);
                }
            }
        }
#endif
        break;

        case GAPM_GEN_RAND_ADDR:
        {
            QPRINTF("Random address generation(0x%02x).\r\n", param->status);
        }
        break;

        case GAPM_GET_RAL_SIZE:
        case GAPM_GET_RAL_LOC_ADDR:
        case GAPM_GET_RAL_PEER_ADDR:
        case GAPM_RMV_DEV_FRM_RAL:
        case GAPM_CLEAR_RAL:
            QPRINTF("RAL MGT status(0x%02x).\r\n", param->status);
            break;
        case GAPM_ADD_DEV_IN_RAL:
        {
            if (param->status != GAP_ERR_NO_ERROR)
            {
                QPRINTF("Add device in RAL failed.\r\n");
            }
            else
            {
                QPRINTF("Add device in RAL success.\r\n");
            }
        }
        break;

        case GAPM_GET_WLIST_SIZE:
        case GAPM_ADD_DEV_IN_WLIST:
        case GAPM_RMV_DEV_FRM_WLIST:
        case GAPM_CLEAR_WLIST:
        {
            QPRINTF("White list MGT status(0x%02x).\r\n", param->status);
        }
        break;

        case GAPM_SET_CHANNEL_MAP:
            QPRINTF("Set map status(0x%02x).\r\n", param->status);
            break;

        case GAPM_START_ADVERTISE:
            QPRINTF("Advertising start.\r\n");
            break;

        case GAPM_USE_ENC_BLOCK:
            break;

        default:
        {
            QPRINTF("Unhandle operation(%d)\r\n", param->operation);
            ASSERT_ERR(0);
        }
        break;
    }

    return (KE_MSG_CONSUMED);
}

int APP_GapmDeviceReadyIndHandler(ke_msg_id_t const msgid,
                                  void const *param,
                                  ke_task_id_t const dest_id,
                                  ke_task_id_t const src_id)
{
    if (ke_state_get(dest_id) == APP_STATE_INIT)
    {
        APP_GapmResetCmd();
    }
    else
    {
        /* APP_INIT state is used to wait the GAP_READY_EVT message */
        ASSERT_ERR(0);
    }

    return (KE_MSG_CONSUMED);
}

int APP_GapmWhiteListSizeIndHandler(ke_msg_id_t const msgid,
                                    struct gapm_white_list_size_ind const *param,
                                    ke_task_id_t const dest_id,
                                    ke_task_id_t const src_id)
{
    QPRINTF("White list size: %d.\r\n", param->size);

    return (KE_MSG_CONSUMED);
}

int APP_GapmRalSizeIndHandler(ke_msg_id_t const msgid,
                              struct gapm_ral_size_ind const *param,
                              ke_task_id_t const dest_id,
                              ke_task_id_t const src_id)
{
    QPRINTF("Resolving address list size: %d.\r\n", param->size);

    return (KE_MSG_CONSUMED);
}

int APP_GapmRalAddrIndHandler(ke_msg_id_t const msgid,
                              struct gapm_ral_addr_ind const *param,
                              ke_task_id_t const dest_id,
                              ke_task_id_t const src_id)
{
    QPRINTF("Resolvable address: %02x%02x%02x%02x%02x%02x.\r\n", param->addr.addr.addr[5], param->addr.addr.addr[4],
            param->addr.addr.addr[3], param->addr.addr.addr[2], param->addr.addr.addr[1], param->addr.addr.addr[0]);

    return (KE_MSG_CONSUMED);
}

int APP_GapmDevVersionIndHandler(ke_msg_id_t const msgid,
                                 struct gapm_dev_version_ind const *param,
                                 ke_task_id_t const dest_id,
                                 ke_task_id_t const src_id)
{
    QPRINTF("HCI version: 0x%02x.\r\n", param->hci_ver);
    QPRINTF("LMP version: 0x%02x.\r\n", param->lmp_ver);
    QPRINTF("Host version: 0x%02x.\r\n", param->host_ver);
    QPRINTF("HCI subversion: 0x%02x.\r\n", param->hci_subver);
    QPRINTF("LMP subversion: 0x%02x.\r\n", param->lmp_subver);
    QPRINTF("Host subversion: 0x%02x.\r\n", param->host_subver);
    QPRINTF("Manufacturer name: 0x%02x.\r\n", param->manuf_name);

    return (KE_MSG_CONSUMED);
}

int APP_GapmDevBdAddrIndHandler(ke_msg_id_t const msgid,
                                struct gapm_dev_bdaddr_ind const *param,
                                ke_task_id_t const dest_id,
                                ke_task_id_t const src_id)
{
    QPRINTF("Local BD address type: 0x%02x.\r\n", param->addr.addr_type);
    QPRINTF("Local BD address: %02x%02x%02x%02x%02x%02x.\r\n", param->addr.addr.addr[5], param->addr.addr.addr[4],
            param->addr.addr.addr[3], param->addr.addr.addr[2], param->addr.addr.addr[1], param->addr.addr.addr[0]);
#ifdef MBED_PORT
    func_flag |= BLE_ADDRESS;
    data_ptr = (struct gapm_dev_bdaddr_ind *)ke_malloc(sizeof(struct gapm_dev_bdaddr_ind), KE_MEM_NON_RETENTION);
    memcpy(data_ptr, param, sizeof(struct gapm_dev_bdaddr_ind));
// ble_address = param->addr;
//*(struct gapm_dev_bdaddr_ind *)data_ptr = *param;
// return(KE_MSG_NO_FREE);
#endif

    return (KE_MSG_CONSUMED);
}

int APP_GapmDevAdvTxPowerIndHandler(ke_msg_id_t const msgid,
                                    struct gapm_dev_adv_tx_power_ind const *param,
                                    ke_task_id_t const dest_id,
                                    ke_task_id_t const src_id)
{
    QPRINTF("Advertising channel Tx power level: 0x%02x.\r\n", param->power_lvl);

    return (KE_MSG_CONSUMED);
}

int APP_GapmSuggDfltDataLenIndHandler(ke_msg_id_t const msgid,
                                      struct gapm_sugg_dflt_data_len_ind const *param,
                                      ke_task_id_t const dest_id,
                                      ke_task_id_t const src_id)
{
    QPRINTF("suggted_max_tx_octets: %d.\r\n", param->suggted_max_tx_octets);
    QPRINTF("suggted_max_tx_time: %d.\r\n", param->suggted_max_tx_time);

    return (KE_MSG_CONSUMED);
}

int APP_GapmMaxDataLenIndHandler(ke_msg_id_t const msgid,
                                 struct gapm_max_data_len_ind const *param,
                                 ke_task_id_t const dest_id,
                                 ke_task_id_t const src_id)
{
    QPRINTF("suppted_max_tx_octets: %d.\r\n", param->suppted_max_tx_octets);
    QPRINTF("suppted_max_tx_time: %d.\r\n", param->suppted_max_tx_time);
    QPRINTF("suppted_max_rx_octets: %d.\r\n", param->suppted_max_rx_octets);
    QPRINTF("suppted_max_rx_time: %d.\r\n", param->suppted_max_rx_time);

    return (KE_MSG_CONSUMED);
}

int APP_GapmUseEncBlockIndHandler(ke_msg_id_t const msgid,
                                  struct gapm_use_enc_block_ind const *param,
                                  ke_task_id_t const dest_id,
                                  ke_task_id_t const src_id)
{
    QPRINTF("Result: ");
    for (int i = 0; i < GAP_KEY_LEN; i++)
    {
        QPRINTF(" 0x%02x", param->result[i]);
    }
    QPRINTF("\r\n");

    return (KE_MSG_CONSUMED);
}

int APP_GapmAddrSolvedIndHandler(ke_msg_id_t const msgid,
                                 struct gapm_addr_solved_ind const *param,
                                 ke_task_id_t const dest_id,
                                 ke_task_id_t const src_id)
{
    QPRINTF("Resolvable BD address: %02x%02x%02x%02x%02x%02x.\r\n", param->addr.addr[5], param->addr.addr[4],
            param->addr.addr[3], param->addr.addr[2], param->addr.addr[1], param->addr.addr[0]);

#if (BLE_CENTRAL || BLE_PERIPHERAL)
    if (g_AppEnv.state == APP_OP_CONNECTING)
    { /* To finish the connecting procedure */
        struct app_pairing_keys_info key;
        key.irk.irk = param->irk;
        uint8_t bonded_dev_idx = APP_FindBondedDevByPeerKey(&key, GAP_KDIST_IDKEY);

        APP_GapcConnectionCfm(g_AppEnv.conhdl, &g_AppEnv.bond_dev_list[bonded_dev_idx].bond_info.local_keys.csrk,
                              g_AppEnv.bond_dev_list[bonded_dev_idx].bond_info.local_sign_counter,
                              &g_AppEnv.bond_dev_list[bonded_dev_idx].bond_info.peer_keys.csrk,
                              g_AppEnv.bond_dev_list[bonded_dev_idx].bond_info.peer_sign_counter,
                              g_AppEnv.bond_dev_list[bonded_dev_idx].bond_info.auth,
                              g_AppEnv.bond_dev_list[bonded_dev_idx].bond_info.svc_changed_ind_enable,
                              g_AppEnv.bond_dev_list[bonded_dev_idx].bond_info.ltk_present);

        g_AppEnv.conn_dev_list[g_AppEnv.conhdl].conn_info.bond_index = bonded_dev_idx;

        g_AppEnv.state = APP_OP_NOP;
    }
#endif

    return (KE_MSG_CONSUMED);
}

#if (BLE_CENTRAL)
int APP_GapmAdvReportIndHandler(ke_msg_id_t const msgid,
                                struct gapm_adv_report_ind const *param,
                                ke_task_id_t const dest_id,
                                ke_task_id_t const src_id)
{
    switch (g_AppEnv.state)
    {
        case APP_OP_SCANING:
        {
            bool found = false;
            for (uint8_t i = 0; i < g_AppEnv.scan_count; i++)
            {
                if (true == co_bdaddr_compare((struct bd_addr *)&g_AppEnv.scan_dev_list[i].addr,
                                              (struct bd_addr *)&param->report.adv_addr))
                {
                    found = true;
                    break;
                }
            }
            /* add the device in the address keeper */
            if (!found && (g_AppEnv.scan_count < CFG_SCAN_DEV_MAX))
            {
                g_AppEnv.scan_dev_list[g_AppEnv.scan_count].addr_type = param->report.adv_addr_type;
                memcpy(&g_AppEnv.scan_dev_list[g_AppEnv.scan_count].addr, param->report.adv_addr.addr, BD_ADDR_LEN);
                QPRINTF("Record index: %d.\r\n", g_AppEnv.scan_count);
                g_AppEnv.scan_count++;
            }
        }
        break;

        case APP_OP_INITIATING: /* GAPM_CONNECTION_SELECTIVE */
        {
            /* Sepcify a peer device to connect to */
            struct gap_bdaddr peer_addr;
            /* TODO by user */

            if (true == co_bdaddr_compare((struct bd_addr *)&peer_addr.addr, (struct bd_addr *)&param->report.adv_addr))
            {
                APP_GapmConnectionCfm(&peer_addr, CFG_CONN_MIN_INTV, CFG_CONN_MAX_INTV, CFG_CONN_LATENCY,
                                      CFG_CONN_SUPERV_TIMEOUT);
            }
        }
        break;

        default:
            break;
    }

    APP_AdvReportIndCallback(param);

    return (KE_MSG_CONSUMED);
}
#endif

int APP_GapmPeerNameIndHandler(ke_msg_id_t const msgid,
                               struct gapm_peer_name_ind const *param,
                               ke_task_id_t const dest_id,
                               ke_task_id_t const src_id)
{
    QPRINTF("Peer name: ");
    for (int i = 0; i < param->name_len; i++)
    {
        QPRINTF("%c", param->name[i]);
    }
    QPRINTF(".\r\n");

    return (KE_MSG_CONSUMED);
}

#if (BLE_PROFILES)
int APP_GapmProfileAddedIndHandler(ke_msg_id_t const msgid,
                                   struct gapm_profile_added_ind const *param,
                                   ke_task_id_t const dest_id,
                                   ke_task_id_t const src_id)
{
    if (param->start_hdl)
    {
#if (BLE_BATT_SERVER)
        static uint8_t bas_nb = 0;
        ASSERT_ERR(bas_nb <= CFG_BASS_NB_BAS_INS_MAX);
        if (param->prf_task_id == TASK_ID_BASS)
        {
            g_AppBassEnv.start_hdl[bas_nb++] = param->start_hdl;
        }
#endif
    }

    /* Add the next requested profile */
    APP_AddPrfTask();

    return (KE_MSG_CONSUMED);
}
#endif

#if (BLE_CENTRAL || BLE_PERIPHERAL)
int APP_GapcCmpEvtHandler(ke_msg_id_t const msgid,
                          struct gapc_cmp_evt const *param,
                          ke_task_id_t const dest_id,
                          ke_task_id_t const src_id)
{
    switch (param->operation)
    {
        case GAPC_DISCONNECT:
            break;

        case GAPC_GET_PEER_NAME:
            break;

        case GAPC_GET_PEER_VERSION:
            break;

        case GAPC_GET_PEER_FEATURES:
            break;

        case GAPC_GET_PEER_APPEARANCE:
            break;

        case GAPC_GET_PEER_SLV_PREF_PARAMS:
            break;

        case GAPC_GET_CON_RSSI:
            break;

        case GAPC_GET_CON_CHANNEL_MAP:
            break;

        case GAPC_GET_LE_PING_TO:
            break;

        case GAPC_SET_LE_PING_TO:
        {
            QPRINTF("Set LE ping timeout status: 0x%x.\r\n", param->status);
        }
        break;

        case GAPC_UPDATE_PARAMS:
#ifndef MBED_PORT
            if ((param->status != CO_ERROR_NO_ERROR))
            {
                QPRINTF("Update params failed, status: 0x%x.\r\n", param->status);
            }
            else
            {
                QPRINTF("Update complete\r\n");
            }
            break;
#else
            if ((param->status != CO_ERROR_NO_ERROR))
            {
                QPRINTF("Update params failed, status: 0x%x.\r\n", param->status);
            }
            else
            {
                QPRINTF("Update complete\r\n");
            }
            func_flag |= FB_DEV_UPD_PARAM;
            break;
#endif

        case GAPC_BOND:
            if (param->status == GAP_ERR_NOT_SUPPORTED)
            {
                QPRINTF("Pairirng request supported only by master of the connection.\r\n");
            }
            else if (param->status != CO_ERROR_NO_ERROR)
            {
                QPRINTF("Pairirng request failed, status: 0x%x.\r\n", param->status);
            }
            break;

        case GAPC_ENCRYPT:
            if ((param->status != CO_ERROR_NO_ERROR))
            {
                QPRINTF("Encrypt failed, status: 0x%x.\r\n", param->status);
            }
            else
            {
                QPRINTF("Encrypt complete\r\n");
            }
            break;

        case GAPC_SECURITY_REQ:
            if (param->status == GAP_ERR_NOT_SUPPORTED)
            {
                QPRINTF("Security request supported only by slave of the connection.\r\n");
            }
            break;

        case GAPC_SET_LE_PKT_SIZE:
            if ((param->status != CO_ERROR_NO_ERROR))
            {
                QPRINTF("Set LE packet length failed, status: 0x%x.\r\n", param->status);
            }
            else
            {
                QPRINTF("Set LE packet length complete\r\n");
            }
            break;

        default:
        {
            QPRINTF("Unhandle operation(%d)\r\n", param->operation);
            ASSERT_ERR(0);
        }
        break;
    }

    return (KE_MSG_CONSUMED);
}

int APP_GapcConnectionReqIndHandler(ke_msg_id_t const msgid,
                                    struct gapc_connection_req_ind const *param,
                                    ke_task_id_t const dest_id,
                                    ke_task_id_t const src_id)
{
    uint16_t conhdl = CONIDX2CONHDL(KE_IDX_GET(src_id));

    /* Check if the received Connection Handle was valid */
    if (conhdl != GAP_INVALID_CONHDL)
    {
        if (g_AppEnv.conn_count == 0)
        {
#if (BLE_BATT_SERVER)
            APP_TIMER_SET(APP_MSG_BASS_BATT_LVL_CHK_TIMER, CFG_BASS_BATT_LVL_CHK_DUR, APP_BassBattLvlChkTimerHandler);
#endif
        }

        APP_SetLinkInfoByConhdl(conhdl, param, true);

        uint8_t bonded_dev_idx = APP_FindBondedDevByAddr(&param->peer_addr, param->peer_addr_type);

        /* send connection confirmation */
        if (bonded_dev_idx == APP_INVALID_IDX)
        {
            if ((g_AppEnv.bond_count) && (param->peer_addr_type == ADDR_RAND) &&
                ((param->peer_addr.addr[5] & 0xC0) == GAP_RSLV_ADDR))
            {
                struct gap_sec_key irk_array[CFG_BOND_DEV_MAX];
                for (int i = 0; i < CFG_BOND_DEV_MAX; i++)
                {
                    if (g_AppEnv.bond_dev_list[i].free == false)
                    {
                        memcpy(&irk_array[i], g_AppEnv.bond_dev_list[i].bond_info.peer_keys.irk.irk.key,
                               sizeof(struct gap_sec_key));
                    }
                }

                APP_GapmResolvAddrCmd(g_AppEnv.bond_count, (bd_addr_t *)&param->peer_addr, irk_array);
                g_AppEnv.state = APP_OP_CONNECTING;
                g_AppEnv.conhdl = conhdl;
                /* send connection confirmation later */
            }
            else
            {
                /* No bond info found */
                APP_GapcConnectionCfm(conhdl, NULL, 0, NULL, 0, GAP_AUTH_REQ_NO_MITM_NO_BOND, false, false);
            }
        }
        else
        {
            APP_GapcConnectionCfm(conhdl, &g_AppEnv.bond_dev_list[bonded_dev_idx].bond_info.local_keys.csrk,
                                  g_AppEnv.bond_dev_list[bonded_dev_idx].bond_info.local_sign_counter,
                                  &g_AppEnv.bond_dev_list[bonded_dev_idx].bond_info.peer_keys.csrk,
                                  g_AppEnv.bond_dev_list[bonded_dev_idx].bond_info.peer_sign_counter,
                                  g_AppEnv.bond_dev_list[bonded_dev_idx].bond_info.auth,
                                  g_AppEnv.bond_dev_list[bonded_dev_idx].bond_info.svc_changed_ind_enable,
                                  g_AppEnv.bond_dev_list[bonded_dev_idx].bond_info.ltk_present);

            g_AppEnv.conn_dev_list[conhdl].conn_info.bond_index = bonded_dev_idx;
#if (BLE_BATT_SERVER)
            APP_BassEnableReq(CONIDX2CONHDL(KE_IDX_GET(src_id)),
                              g_AppEnv.bond_dev_list[bonded_dev_idx].bond_info.bass.ntf_cfg,
                              g_AppEnv.bond_dev_list[bonded_dev_idx].bond_info.bass.old_batt_lvl);
#endif
        }

#if defined(CFG_PER_TEST)
        APP_StartStatistic(param->conhdl);
#endif

        APP_ConnectionEstablishedCallback(param);
#ifdef MBED_PORT
        connection_start(param);
#endif
    }
    else
    {
    }

    return (KE_MSG_CONSUMED);
}

int APP_GapcDisconnectIndHandler(ke_msg_id_t const msgid,
                                 struct gapc_disconnect_ind const *param,
                                 ke_task_id_t const dest_id,
                                 ke_task_id_t const src_id)
{
    uint8_t addr_type;
    bd_addr_t peer_addr;

    if (param->conhdl != GAP_INVALID_CONHDL)
    {
        if (APP_GetPeerBdAddrByConhdl(param->conhdl, &peer_addr, &addr_type))
        {
            QPRINTF("Disconnect with %02x%02x%02x%02x%02x%02x, reason is 0x%02x.\r\n", peer_addr.addr[5],
                    peer_addr.addr[4], peer_addr.addr[3], peer_addr.addr[2], peer_addr.addr[1], peer_addr.addr[0],
                    param->reason);

            APP_SetLinkInfoByConhdl(param->conhdl, NULL, false);

#if (BLE_BATT_SERVER)
            uint8_t idx = APP_FindBondedDevByAddr(&peer_addr, addr_type);
            g_AppEnv.bond_dev_list[idx].bond_info.bass.ntf_cfg = g_AppBassEnv.ntf_cfg[KE_IDX_GET(src_id)];

            for (int i = 0; i < BASS_NB_BAS_INSTANCES_MAX; i++)
            {
                g_AppEnv.bond_dev_list[idx].bond_info.bass.old_batt_lvl[i] = g_AppBassEnv.batt_lvl[i];
            }
#endif
        }
        else
        {
            ASSERT_ERR(0);
        }

        if (g_AppEnv.conn_count == 0)
        {
#if (BLE_BATT_SERVER)
            APP_TIMER_CLR(APP_MSG_BASS_BATT_LVL_CHK_TIMER);
#endif
        }

#if defined(CFG_PER_TEST)
        APP_StopStatistic(param->conhdl);
#endif

#ifndef MBED_PORT
        APP_ConnectionTerminatedCallback(param);
#else
        connection_end(param->conhdl, param->reason);
#endif
        ke_state_set(dest_id, APP_STATE_IDLE);
    }

    return (KE_MSG_CONSUMED);
}

int APP_GapcPeerAttInfoIndHandler(ke_msg_id_t const msgid,
                                  struct gapc_peer_att_info_ind const *param,
                                  ke_task_id_t const dest_id,
                                  ke_task_id_t const src_id)
{
    switch (param->req)
    {
        case GAPC_DEV_NAME:
            QPRINTF("Peer device name: ");
            for (int i = 0; i < param->info.name.length; i++)
            {
                QPRINTF("%c", param->info.name.value[i]);
            }
            QPRINTF(".\r\n");
            break;

        case GAPC_DEV_APPEARANCE:
            QPRINTF("Peer device appearance icon: 0x%04x.\r\n", param->info.appearance);
            break;

        case GAPC_DEV_SLV_PREF_PARAMS:
            QPRINTF(
                "Peer device slave preferred parameters: con_intv_min=0x%04x, con_intv_max=0x%04x, "
                "slave_latency=0x%04x, conn_timeout=0x%04x.\r\n",
                param->info.slv_params.con_intv_min, param->info.slv_params.con_intv_max,
                param->info.slv_params.slave_latency, param->info.slv_params.conn_timeout);
            break;

        case GAPC_DEV_CTL_ADDR_RESOL:
            QPRINTF("Peer device Central address resolution: 0x%02x.\r\n", param->info.cnt_addr_resol);
            break;
        default:
            break;
    }

    return (KE_MSG_CONSUMED);
}

int APP_GapcPeerVersionIndHandler(ke_msg_id_t const msgid,
                                  struct gapc_peer_version_ind const *param,
                                  ke_task_id_t const dest_id,
                                  ke_task_id_t const src_id)
{
    QPRINTF("Peer device company id: 0x%04x.\r\n", param->compid);
    QPRINTF("Peer device LMP version: 0x%02x, subversion: 0x%04x.\r\n", param->lmp_vers, param->lmp_subvers);

    return (KE_MSG_CONSUMED);
}

int APP_GapcPeerFeaturesIndHandler(ke_msg_id_t const msgid,
                                   struct gapc_peer_features_ind const *param,
                                   ke_task_id_t const dest_id,
                                   ke_task_id_t const src_id)
{
    QPRINTF("Peer device features:");
    for (int i = 0; i < LE_FEATS_LEN; i++)
    {
        QPRINTF(" %02x", param->features[i]);
    }
    QPRINTF("\r\n");

    return (KE_MSG_CONSUMED);
}

int APP_GapcConRssiIndHandler(ke_msg_id_t const msgid,
                              struct gapc_con_rssi_ind const *param,
                              ke_task_id_t const dest_id,
                              ke_task_id_t const src_id)
{
    QPRINTF("RSSI value: 0x%02x.\r\n", param->rssi);

    return (KE_MSG_CONSUMED);
}

int APP_GapcSignCounterIndHandler(ke_msg_id_t const msgid,
                                  struct gapc_sign_counter_ind const *param,
                                  ke_task_id_t const dest_id,
                                  ke_task_id_t const src_id)
{
    uint16_t conhdl = CONIDX2CONHDL(KE_IDX_GET(src_id));

    /* Only bonded deivce to store sign counter */
    if (g_AppEnv.conn_dev_list[conhdl].conn_info.bond_index != APP_INVALID_IDX)
    {
        g_AppEnv.bond_dev_list[g_AppEnv.conn_dev_list[conhdl].conn_info.bond_index].bond_info.local_sign_counter =
            param->local_sign_counter;
        g_AppEnv.bond_dev_list[g_AppEnv.conn_dev_list[conhdl].conn_info.bond_index].bond_info.local_sign_counter =
            param->peer_sign_counter;
    }

    QPRINTF("Local sign counter: %d.\r\n", param->local_sign_counter);
    QPRINTF("Peer sign counter: %d.\r\n", param->peer_sign_counter);

    return (KE_MSG_CONSUMED);
}

int APP_GapcConChannelMapIndHandler(ke_msg_id_t const msgid,
                                    struct gapc_con_channel_map_ind const *param,
                                    ke_task_id_t const dest_id,
                                    ke_task_id_t const src_id)
{
    QPRINTF("Channel map:");
    for (int i = 0; i < LE_CHNL_MAP_LEN; i++)
    {
        QPRINTF(" %02x", param->ch_map.map[i]);
    }
    QPRINTF("\r\n");

    return (KE_MSG_CONSUMED);
}

int APP_GapcLePingToValIndHandler(ke_msg_id_t const msgid,
                                  struct gapc_le_ping_to_val_ind const *param,
                                  ke_task_id_t const dest_id,
                                  ke_task_id_t const src_id)
{
    QPRINTF("LE ping timeout value: 0x%04x.\r\n", param->timeout);

    return (KE_MSG_CONSUMED);
}

int APP_GapcGetDevInfoReqIndHandler(ke_msg_id_t const msgid,
                                    struct gapc_get_dev_info_req_ind const *param,
                                    ke_task_id_t const dest_id,
                                    ke_task_id_t const src_id)
{
    struct gapc_get_dev_info_cfm *cfm = NULL;

    switch (param->req)
    {
        case GAPC_DEV_NAME:
        {
            QPRINTF("Requested device name\r\n");
            /* Get device name from NVDS */
            uint8_t dev_name[NVDS_LEN_DEVICE_NAME];
            uint8_t dev_name_len = NVDS_LEN_DEVICE_NAME;
            if (nvds_get(NVDS_TAG_DEVICE_NAME, (nvds_tag_len_t *)&dev_name_len, dev_name) != NVDS_OK)
            {
                dev_name_len = sizeof(GAP_DEV_NAME);
                memcpy(dev_name, GAP_DEV_NAME, dev_name_len);
            }

            cfm = KE_MSG_ALLOC_DYN(GAPC_GET_DEV_INFO_CFM, KE_BUILD_ID(TASK_GAPC, KE_IDX_GET(src_id)), TASK_APP,
                                   gapc_get_dev_info_cfm, ((dev_name_len > 6) ? (dev_name_len - 6) : 0));
            cfm->info.name.length = dev_name_len;
            memcpy(cfm->info.name.value, dev_name, dev_name_len);
        }
        break;

        case GAPC_DEV_APPEARANCE:
            QPRINTF("Requested device appearance.\r\n");
            cfm = KE_MSG_ALLOC(GAPC_GET_DEV_INFO_CFM, KE_BUILD_ID(TASK_GAPC, KE_IDX_GET(src_id)), TASK_APP,
                               gapc_get_dev_info_cfm);

            cfm->info.appearance = CFG_DEV_APPEARANCE;
            break;

        case GAPC_DEV_SLV_PREF_PARAMS:
            QPRINTF("Requested slave preferred parameters.\r\n");
            cfm = KE_MSG_ALLOC(GAPC_GET_DEV_INFO_CFM, KE_BUILD_ID(TASK_GAPC, KE_IDX_GET(src_id)), TASK_APP,
                               gapc_get_dev_info_cfm);

            cfm->info.slv_params.conn_timeout = CFG_CONN_SUPERV_TIMEOUT;
            cfm->info.slv_params.con_intv_max = CFG_CONN_MAX_INTV;
            cfm->info.slv_params.con_intv_min = CFG_CONN_MIN_INTV;
            cfm->info.slv_params.slave_latency = CFG_CONN_LATENCY;
            break;

        case GAPC_DEV_CTL_ADDR_RESOL:
            QPRINTF("Requested device Central address resolution.\r\n");
            cfm = KE_MSG_ALLOC(GAPC_GET_DEV_INFO_CFM, KE_BUILD_ID(TASK_GAPC, KE_IDX_GET(src_id)), TASK_APP,
                               gapc_get_dev_info_cfm);

            cfm->info.cnt_addr_resol = 0x01;
            break;

        default:
            break;
    }

    cfm->req = param->req;
    APP_MsgSend(cfm);

    return (KE_MSG_CONSUMED);
}

int APP_GapcSetDevInfoReqIndHandler(ke_msg_id_t const msgid,
                                    struct gapc_set_dev_info_req_ind const *param,
                                    ke_task_id_t const dest_id,
                                    ke_task_id_t const src_id)
{
    /* Set Device configuration */
    struct gapc_set_dev_info_cfm *cfm = KE_MSG_ALLOC(GAPC_SET_DEV_INFO_CFM, src_id, dest_id, gapc_set_dev_info_cfm);

    /* Allow to change parameters */
    cfm->status = GAP_ERR_NO_ERROR;
    cfm->req = param->req;

    switch (param->req)
    {
        case GAPC_DEV_NAME:
            if (nvds_put(NVDS_TAG_DEVICE_NAME, param->info.name.length, (uint8_t *)param->info.name.value) != NVDS_OK)
            {
                QPRINTF("Set device name failed.\r\n");
                /* Reject to change parameters */
                cfm->status = GAP_ERR_REJECTED;
            }
            else
            {
                QPRINTF("Set device name seccessful.\r\n");
            }
            break;

        case GAPC_DEV_APPEARANCE:
            QPRINTF("Set device appearance by peer.\r\n");
            break;

        default:
            break;
    }

    /* Send message */
    APP_MsgSend(cfm);

    return (KE_MSG_CONSUMED);
}

int APP_GapcLePktSizeIndHandler(ke_msg_id_t const msgid,
                                struct gapc_le_pkt_size_ind const *param,
                                ke_task_id_t const dest_id,
                                ke_task_id_t const src_id)
{
    QPRINTF("max_tx_octets: %d.\r\n", param->max_tx_octets);
    QPRINTF("max_tx_time: %d.\r\n", param->max_tx_time);
    QPRINTF("max_rx_octets: %d.\r\n", param->max_rx_octets);
    QPRINTF("max_rx_time: %d.\r\n", param->max_rx_time);

    return (KE_MSG_CONSUMED);
}

int APP_GapcParamUpdateReqIndHandler(ke_msg_id_t const msgid,
                                     struct gapc_param_update_req_ind const *param,
                                     ke_task_id_t const dest_id,
                                     ke_task_id_t const src_id)
{
    uint16_t conhdl = CONIDX2CONHDL(KE_IDX_GET(src_id));
    APP_GapcParamUpdateCfm(conhdl, true);

    return (KE_MSG_CONSUMED);
}

int APP_GapcParamUpdatedIndHandler(ke_msg_id_t const msgid,
                                   struct gapc_param_updated_ind const *param,
                                   ke_task_id_t const dest_id,
                                   ke_task_id_t const src_id)
{
    QPRINTF("Connection(%d) update parameter complete, interval: 0x%04x, latency: 0x%04x, sup to: 0x%04x.\r\n",
            CONIDX2CONHDL(KE_IDX_GET(src_id)), param->con_interval, param->con_latency, param->sup_to);

    return (KE_MSG_CONSUMED);
}

int APP_GapcBondReqIndHandler(ke_msg_id_t const msgid,
                              struct gapc_bond_req_ind const *param,
                              ke_task_id_t const dest_id,
                              ke_task_id_t const src_id)
{
    uint16_t conhdl = CONIDX2CONHDL(KE_IDX_GET(src_id));

    switch (param->request)
    {
        /* Bond Pairing request */
        case GAPC_PAIRING_REQ:
        {
            /* Check the authentication level to decide to accept the pairing or not */
            APP_GapcBondCfm(conhdl, GAPC_PAIRING_RSP, true, (void *)&s_appPairingParameters);
        }
        break;

        /* Used to retrieve pairing Temporary Key */
        case GAPC_TK_EXCH:
        {
            uint32_t pin_code = 0;
            /* TK shall be displayed by local device */
            if (param->data.tk_type == GAP_TK_DISPLAY)
            {
                /* TK generated */
                pin_code = APP_SecGenTk();

                struct gap_sec_key tk;
                memset(tk.key, 0, GAP_KEY_LEN);
                tk.key[0] = (uint8_t)((pin_code & 0x000000FF) >> 0); /* LSB */
                tk.key[1] = (uint8_t)((pin_code & 0x0000FF00) >> 8);
                tk.key[2] = (uint8_t)((pin_code & 0x00FF0000) >> 16);
                APP_GapcBondCfm(conhdl, GAPC_TK_EXCH, true, &tk);
            }
            APP_TkExchangeCallback(conhdl, param->data.tk_type, pin_code);
            QPRINTF("Send TK.\r\n");
        }
        break;

        case GAPC_CSRK_EXCH:
        {
            /* Generate random CSRK */
            APP_SecGenKey(g_AppEnv.conn_dev_list[conhdl].conn_info.local_keys.csrk.key, GAP_KEY_LEN);
            APP_GapcBondCfm(conhdl, GAPC_CSRK_EXCH, true, &g_AppEnv.conn_dev_list[conhdl].conn_info.local_keys.csrk);
            QPRINTF("Send CSRK.\r\n");
        }
        break;

        case GAPC_LTK_EXCH:
        {
            /* Generate random LTK */
            APP_SecGenLtk(conhdl, param->data.key_size);
            APP_GapcBondCfm(conhdl, GAPC_LTK_EXCH, true, &g_AppEnv.conn_dev_list[conhdl].conn_info.local_keys.ltk);
            QPRINTF("Send LTK.\r\n");
        }
        break;

        case GAPC_IRK_EXCH:
        {
            /* Used the initail IRK in g_AppEnv */
            APP_GapcBondCfm(conhdl, GAPC_IRK_EXCH, true, &g_AppEnv.irk);
            QPRINTF("Send IRK.\r\n");
        }
        break;

        case GAPC_NC_EXCH:
        {
            uint32_t nc_number = co_read32p(param->data.nc_data.value);
            APP_NcExchangeCallback(conhdl, nc_number);
        }
        break;

        default:
            QPRINTF("Unhandle bond request %d.\r\n", param->request);
            break;
    }

    return (KE_MSG_CONSUMED);
}

int APP_GapcBondIndHandler(ke_msg_id_t const msgid,
                           struct gapc_bond_ind const *param,
                           ke_task_id_t const dest_id,
                           ke_task_id_t const src_id)
{
    uint16_t conhdl = CONIDX2CONHDL(KE_IDX_GET(src_id));

    switch (param->info)
    {
        case GAPC_PAIRING_SUCCEED:
            if (param->data.auth.info & GAP_AUTH_BOND)
            {
                uint8_t bonded_dev_idx = APP_AddBondedDev(&g_AppEnv.conn_dev_list[conhdl].conn_info);
                if (bonded_dev_idx == APP_INVALID_IDX)
                {
                    QPRINTF("Bond info store failed!\r\n");
                }
                else
                {
                    g_AppEnv.conn_dev_list[conhdl].conn_info.bond_index = bonded_dev_idx;

                    g_AppEnv.bond_dev_list[bonded_dev_idx].bond_info.auth = param->data.auth.info;
                    g_AppEnv.bond_dev_list[bonded_dev_idx].bond_info.ltk_present = param->data.auth.ltk_present;
                    if (param->data.auth.info & GAP_AUTH_SEC_CON)
                    {
                        g_AppEnv.bond_dev_list[bonded_dev_idx].bond_info.local_keys.ltk =
                            g_AppEnv.bond_dev_list[bonded_dev_idx].bond_info.peer_keys.ltk;
                    }
                    QPRINTF("Bond info store success.\r\n");
                }
            }
            APP_BondSuccessCallback(conhdl, param);
            QPRINTF("Bond success, conhdl: 0x%02x.\r\n", conhdl);
            break;

        case GAPC_PAIRING_FAILED:
            QPRINTF("Bond failed, conhdl: 0x%02x, status: 0x%02x.\r\n", conhdl, param->data.reason);
            break;

        /* Received Identity Resolving Key */
        case GAPC_IRK_EXCH:
            g_AppEnv.conn_dev_list[conhdl].conn_info.peer_keys.irk = param->data.irk;
            QPRINTF("Recieve IRK.\r\n");
            break;

        /* Received Connection Signature Resolving Key */
        case GAPC_CSRK_EXCH:
            g_AppEnv.conn_dev_list[conhdl].conn_info.peer_keys.csrk = param->data.csrk;
            QPRINTF("Recieve CSRK.\r\n");
            break;

        /* Received Long Term Key */
        case GAPC_LTK_EXCH:
            g_AppEnv.conn_dev_list[conhdl].conn_info.peer_keys.ltk = param->data.ltk;
            QPRINTF("Recieve LTK.\r\n");
            break;

        default:
            QPRINTF("Unhandle bond info %d\r\n", param->info);
            break;
    }

    return (KE_MSG_CONSUMED);
}

int APP_GapcEncryptReqIndHandler(ke_msg_id_t const msgid,
                                 struct gapc_encrypt_req_ind const *param,
                                 ke_task_id_t const dest_id,
                                 ke_task_id_t const src_id)
{
    uint16_t conhdl = CONIDX2CONHDL(KE_IDX_GET(src_id));

    /* Try to find out LTK */
    struct app_pairing_keys_info key;
    key.ltk.ediv = param->ediv;
    key.ltk.randnb = param->rand_nb;

    uint8_t bonded_dev_idx = APP_FindBondedDevByLocalKey(conhdl, &key, GAP_KDIST_ENCKEY);
    if (bonded_dev_idx == APP_INVALID_IDX)
    {
        /* Cannot find the LTK */
        APP_GapcEncryptCfm(conhdl, false, NULL);
    }
    else
    {
        APP_GapcConnectionCfm(conhdl, &g_AppEnv.bond_dev_list[bonded_dev_idx].bond_info.local_keys.csrk,
                              g_AppEnv.bond_dev_list[bonded_dev_idx].bond_info.local_sign_counter,
                              &g_AppEnv.bond_dev_list[bonded_dev_idx].bond_info.peer_keys.csrk,
                              g_AppEnv.bond_dev_list[bonded_dev_idx].bond_info.peer_sign_counter,
                              g_AppEnv.bond_dev_list[bonded_dev_idx].bond_info.auth,
                              g_AppEnv.bond_dev_list[bonded_dev_idx].bond_info.svc_changed_ind_enable,
                              g_AppEnv.bond_dev_list[bonded_dev_idx].bond_info.ltk_present);

        APP_GapcEncryptCfm(conhdl, true, &g_AppEnv.bond_dev_list[bonded_dev_idx].bond_info.local_keys.ltk);
    }

    return (KE_MSG_CONSUMED);
}

int APP_GapcEncryptIndHandler(ke_msg_id_t const msgid,
                              struct gapc_encrypt_ind const *param,
                              ke_task_id_t const dest_id,
                              ke_task_id_t const src_id)
{
    QPRINTF("Connection(%d) encrypt success, auth level %d\r\n", CONIDX2CONHDL(KE_IDX_GET(src_id)), param->auth);

    return (KE_MSG_CONSUMED);
}

int APP_GapcSecurityIndHandler(ke_msg_id_t const msgid,
                               struct gapc_security_ind const *param,
                               ke_task_id_t const dest_id,
                               ke_task_id_t const src_id)
{
    /* Master devcie receives a security request command, it may encrypt the link, or initiate the pairing procedure, or
     */
    /* reject the request. */
    uint16_t conhdl = CONIDX2CONHDL(KE_IDX_GET(src_id));
    uint8_t bonded_dev_idx = g_AppEnv.conn_dev_list[conhdl].conn_info.bond_index;

    if ((bonded_dev_idx != APP_INVALID_IDX) &&
        (g_AppEnv.bond_dev_list[bonded_dev_idx].bond_info.ltk_present) /* LTK is present */
        && (param->auth <= g_AppEnv.bond_dev_list[bonded_dev_idx].bond_info.auth))
    {
        APP_GapcEncryptCmd(conhdl, &g_AppEnv.bond_dev_list[bonded_dev_idx].bond_info.peer_keys.ltk);
    }
    else
    { /* No bond info found, need pairing */
        struct gapc_pairing pairing_param = s_appPairingParameters;
        pairing_param.auth = param->auth;
        APP_GapcBondCmd(conhdl, &pairing_param);
    }

    return (KE_MSG_CONSUMED);
}
#endif
#endif /* BLE_APP_PRESENT */

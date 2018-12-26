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

#ifndef _APP_GAP_H_
#define _APP_GAP_H_

/*!
 * @addtogroup APP_GAP_API
 * @{
 */

#include "gap.h"
#include "gapm_task.h"
#include "gapc_task.h"

/*******************************************************************************
 * Variables
 ******************************************************************************/
/*! @brief Table of message handlers */
extern const struct ke_state_handler g_AppGapTableHandler;

/*******************************************************************************
 * API
 ******************************************************************************/
/*!
 * @brief Reset the link layer and the host.
 *
 * @response GAPM_CMP_EVT
 *
 * @description
 * This will initialize the BLE Host stack - rearrange to default settings the ATT, GAP,
 * GATT, L2CAP and SMP blocks. Furthermore, this will cause the host to send a reset command
 * down to the link layer part.
 */
void APP_GapmResetCmd(void);

/*!
 * @brief Cancel an ongoing air operation.
 *
 * @response GAPM_CMP_EVT
 *
 * @description
 * Cancel an ongoing air operation such as scanning, advertising or connecting. It has no
 * impact on other commands.
 */
void APP_GapmCancelCmd(void);

/*!
 * @brief Stop advertising.
 *
 * @response GAPM_CMP_EVT
 *
 * @description
 * Stop advertising.
 */
void APP_GapmStopAdvertising(void);

/*!
 * @brief Stop scanning.
 *
 * @response GAPM_CMP_EVT
 *
 * @description
 * Stop scanning.
 */
void APP_GapmStopScanning(void);

/*!
 * @brief Stop connecting.
 *
 * @response GAPM_CMP_EVT
 *
 * @description
 * Stop connecting.
 */
void APP_GapmStopConnecting(void);

/*!
 * @brief Set device configuration command.
 *
 * @param[in] role Device role:
 * - Central(GAP_ROLE_CENTRAL)
 * - Peripheral(GAP_ROLE_PERIPHERAL)
 * - Observer(GAP_ROLE_OBSERVER)
 * - Broadcaster(GAP_ROLE_BROADCASTER)
 * - All(GAP_ROLE_ALL)
 * @param[in] addr_type Device address type
 * - GAPM_CFG_ADDR_PUBLIC
 * - GAPM_CFG_ADDR_PRIVATE
 * - GAPM_CFG_ADDR_HOST_PRIVACY
 * - GAPM_CFG_ADDR_CTNL_PRIVACY
 * @param[in] addr Provided own static random address (addr_type=GAPM_CFG_ADDR_PRIVATE)
 * @param[in] irk Local device IRK used for resolvable random BD address generation (LSB first)
 * @response GAPM_CMP_EVT
 *
 * @description
 * Set the device configuration such as:
 * - Device role
 * - Manage device address type: Public, Static random or Generated for Privacy
 * - Internal IRK used to generate resolvable random address
 * - Set Internal GAP / GATT service start
 * - Set specific write permissions on the appearance and name attributes in internal GAP database.
 * - Manage presence of some attribute.
 * Since system does not support dynamic role switching, this command is allowed only when no link is
 * established.
 */
void APP_GapmSetDevConfigCmd(uint8_t role, uint8_t addr_type, bd_addr_t *addr, struct gap_sec_key *irk);

/*!
 * @brief Set device channel map.
 *
 * @param[in] chmap     Channel map
 * @response  GAPM_CMP_EVT
 *
 * @description
 * Set the channel map of the device.
 *
 * @note
 * The Channel map can be modified only if device is Central.
 */
void APP_GapmSetChannelMapCmd(le_chnl_map_t *chmap);

/*!
 * @brief White list management command.
 *
 * @param[in] operation GAPM requested operation:
 * - GAPM_GET_WLIST_SIZE
 * - GAPM_ADD_DEV_IN_WLIST
 * - GAPM_RMV_DEV_FRM_WLIST
 * - GAPM_CLEAR_WLIST
 * @param[in] nb Number of device information present in command
 * @param[in] devices Device address information that can be used to add or remove element in device list
 * @response
 * - GAPM_WHITE_LIST_SIZE_IND
 * - GAPM_CMP_EVT
 *
 * @description
 * Command used to manage the lower layer BLE White list:
 * - GAPM_GET_WLIST_SIZE: Get White List Size. Array of devices is ignored. Triggers a
 *   GAPM_WHITE_LIST_SIZE_IND that contains size of internal white list.
 * - GAPM_ADD_DEV_IN_WLIST: Add devices in white list, array of devices must be filled.
 * - GAPM_RMV_DEV_FRM_WLIST: Remove devices form white list, array of devices must be filled.
 * - GAPM_CLEAR_WLIST: Clear all devices from white list. Array of devices is ignored.
 *
 * @note
 * White list can be modified by automatic or selective connections modes. This message API should be
 * used only for advertising or scanning Air Operations.
 */
void APP_GapmWhiteListMgtCmd(uint8_t operation, uint8_t nb, struct gap_bdaddr *devices);

/*!
 * @brief Resolving address list management command.
 *
 * @param[in] operation GAPM requested operation:
 * - GAPM_GET_RAL_SIZE
 * - GAPM_GET_RAL_LOC_ADDR
 * - GAPM_GET_RAL_PEER_ADDR
 * - GAPM_ADD_DEV_IN_RAL
 * - GAPM_RMV_DEV_FRM_RAL
 * - GAPM_CLEAR_RAL
 * @param[in] nb Number of device information present in command
 * @param[in] devices Resolving list device information
 * @response
 * - GAPM_CMP_EVT
 * - GAPM_RAL_SIZE_IND
 * - GAPM_RAL_ADDR_IND
 *
 * @description
 * Command used to manage the lower layer BLE resolving address list:
 * - GAPM_GET_RAL_SIZE: Gets resolving address list size. Array of devices is ignored. Triggers a
 *   GAPM_RAL_SIZE_IND that contains size of internal RAL.
 * - GAPM_ADD_DEV_IN_RAL: Add devices in resolving address list, array of devices must be filled.
 * - GAPM_GET_RAL_LOC_ADDR: Gets current local resolvable address. Triggers a
 *   GAPM_RAL_ADDR_IND including the requested address.
 * - GAPM_GET_RAL_PEER_ADDR: Gets current peer resolvable address. Triggers a
 *   GAPM_RAL_ADDR_IND including the requested address.
 * - GAPM_RMV_DEV_FRM_RAL: Remove devices form resolving address list, array of devices must be filled.
 * - GAPM_CLEAR_RAL: Clear all devices from resolving address list. Array of devices is ignored.
 */
void app_GapmRalMgtCmd(uint8_t operation, uint8_t nb, struct gap_ral_dev_info *devices);

/*!
 * @brief Get local divice information command.
 *
 * @param[in] operation GAPM requested operation:
 * - GAPM_GET_DEV_VERSION
 * - GAPM_GET_DEV_BDADDR
 * - GAPM_GET_DEV_ADV_TX_POWER
 * - GAPM_GET_SUGGESTED_DFLT_LE_DATA_LEN
 * - GAPM_GET_MAX_LE_DATA_LEN
 * @response
 * - GAPM_DEV_VERSION_IND
 * - GAPM_DEV_BDADDR_IND
 * - GAPM_DEV_ADV_TX_POWER_IND
 * - GAPM_SUGG_DFLT_DATA_LEN_IND
 * - GAPM_MAX_DATA_LEN_IND
 * - GAPM_CMP_EVT
 *
 * @description
 * Get the local information such as device name, device version, device public BD address, data length extension
 *parameters.
 */
void app_GapmGetDevInfoCmd(uint8_t operation);

/*!
 * @brief Get local divice version.
 *
 * @response
 * - GAPM_DEV_VERSION_IND
 * - GAPM_CMP_EVT
 *
 * @description
 * Get the local device version.
 */
void app_GapmGetDevVersion(void);

/*!
 * @brief Get local divice address.
 *
 * @response
 * - GAPM_DEV_BDADDR_IND
 * - GAPM_CMP_EVT
 *
 * @description
 * Get the local device address.
 */
void APP_GapmGetDevBdAddr(void);

/*!
 * @brief Get local divice advertising TX power.
 *
 * @response
 * - GAPM_DEV_ADV_TX_POWER_IND
 * - GAPM_CMP_EVT
 *
 * @description
 * Get the local device advertising TX power.
 */
void app_GapmGetDevAdvTxPower(void);

/*!
 * @brief Get local divice suggested default LE data length.
 *
 * @response
 * - GAPM_SUGG_DFLT_DATA_LEN_IND
 * - GAPM_CMP_EVT
 *
 * @description
 * Get the local device suggested default LE data length.
 */
void app_GapmGetSuggestedDeflLeDataLen(void);

/*!
 * @brief Get local divice maximum data length.
 *
 * @response
 * - GAPM_MAX_DATA_LEN_IND
 * - GAPM_CMP_EVT
 *
 * @description
 * Get the local divice maximum data length.
 */
void app_GapmGetMaxLeDataLen(void);

/*!
 * @brief Resolve address Command.
 *
 * @param[in] nb_key Number of provided IRK (shall be > 0)
 * @param[in] addr Resolvable random address to solve
 * @param[in] irk Array of IRK used for address resolution (LSB -> MSB)
 * @response
 * - GAPM_ADDR_SOLVED_IND
 * - GAPM_CMP_EVT
 *
 * @description
 * Resolve provided random address using array of Identity Resolution Key (IRK) exchanged
 * and bonded with devices during pairing operations (See GAPC Pairing).
 * Operation will complete successfully if address has been correctly resolved and GAPM_ADDR_SOLVED_IND
 * message will be triggered to inform which key has been used to perform resolution,
 * else operation complete with GAP_ERR_NOT_FOUND error status code.
 */
void APP_GapmResolvAddrCmd(uint8_t nb_key, bd_addr_t *addr, struct gap_sec_key *irk);

/*!
 * @brief Generate a random address Command.
 *
 * @param[in] rnd_type Random address type:
 * - GAP_STATIC_ADDR
 * - GAP_NON_RSLV_ADDR
 * - GAP_RSLV_ADDR
 * @response
 * - GAPM_DEV_BDADDR_IND
 * - GAPM_CMP_EVT
 *
 * @description
 * Generate a random device address without starting any air operation. This can be useful
 * for privacy in order to generate the reconnection address on demand.
 */
void APP_GapmGenRandAddrCmd(uint8_t rnd_type);

/*!
 * @brief Use the AES-128 block in the controller.
 *
 * @param[in] key 128 bits key
 * @param[in] data 128 bits data
 * @response
 * - GAPM_USE_ENC_BLOCK_IND
 * - GAPM_CMP_EVT
 *
 * @description
 * Security toolbox message used to perform an AES-128 calculation operation. This can be used to generate
 * encryption keys.
 */
void APP_GapmUseEncBlockCmd(uint8_t key[16], uint8_t data[16]);

/*!
 * @brief Set new IRK.
 *
 * @param[in] irk New IRK to be set
 * @response
 * - GAPM_CMP_EVT
 *
 * @description
 * Command to change the current IRK for a renewed one, it can be used every time no air operation is being
 * performed.
 */
void APP_GapmSetIrkCmd(struct gap_sec_key *irk);

/*!
 * @brief Set advertising mode Command.
 *
 * @param[in] code Operation code:
 * - GAPM_ADV_NON_CONN
 * - GAPM_ADV_UNDIRECT
 * - GAPM_ADV_DIRECT
 * - GAPM_ADV_DIRECT_LDC
 * @param[in] addr_src Own BD address source of the device:
 * - GAPM_STATIC_ADDR
 * - GAPM_GEN_RSLV_ADDR
 * - GAPM_GEN_NON_RSLV_ADDR
 * @param[in] mode Advertising mode:
 * - GAP_NON_DISCOVERABLE
 * - GAP_GEN_DISCOVERABLE
 * - GAP_LIM_DISCOVERABLE
 * - GAP_BROADCASTER_MODE
 * @param[in] intv_min Minimum interval N for advertising, Value Time = N * 0.625 ms
 * @param[in] intv_max Maximum interval N for advertising, Value Time = N * 0.625 ms
 * @param[in] adv_data_len Advertising data length - maximum 28 bytes, 3 bytes are reserved
 * to set AD type flags in low layer
 * @param[in] adv_data Advertising data,
 * @param[in] scan_rsp_data_len Scan response data length- maximum 31 bytes
 * @param[in] scan_rsp_data Scan response data, if operation code is GAPM_ADV_NON_CONN,
 * and scan response data was set, then the advertising PUD ADV_SCAN_IND will be send
 * @param[in] peer Peer device address information
 * @response
 * - GAPC_CONNECTION_REQ_IND
 * - GAPM_CMP_EVT
 *
 * @description
 * Start Advertising command. This air operation is allowed only for device that support broadcaster or
 * peripheral role.
 * Excepting the direct advertising mode, this air operation is not time-limited. If no connection is established,
 * advertising will continue until application requests to cancel it using GAPM_CANCEL_CMD command.
 *
 * @note
 * Direct advertising package doesn't contain user data, just includes local and peer device address,
 * and the advertising procedure of high duty cycle mode will automatically stopped after 1.28 s.
 */
void APP_GapmStartAdvertiseCmd(uint8_t code,
                               uint8_t addr_src,
                               uint8_t mode,
                               uint16_t intv_min,
                               uint16_t intv_max,
                               uint8_t adv_data_len,
                               uint8_t *adv_data,
                               uint8_t scan_rsp_data_len,
                               uint8_t *scan_rsp_data,
                               struct gap_bdaddr *peer_addr);

/*!
 * @brief Update Advertising Data Command - On fly update when device is advertising.
 *
 * @param[in] adv_data_len Advertising data length - maximum 28 bytes, 3 bytes are reserved to set
 * Advertising AD type flags, shall not be set in advertising data
 * @param[in] adv_data Advertising data
 * @param[in] scan_rsp_data_len Scan response data length- maximum 31 bytes
 * @param[in] scan_rsp_data Scan response data
 * @response GAPM_CMP_EVT
 *
 * @description
 * Update advertising and scan response Data on the fly when device is advertising.
 * If device not in advertise mode, command is rejected.
 */
void APP_GapmUpdateAdvertiseDataCmd(uint8_t adv_data_len,
                                    uint8_t *adv_data,
                                    uint8_t scan_rsp_data_len,
                                    uint8_t *scan_rsp_data);

/*!
 * @brief Set Scan mode Command.
 *
 * @param[in] code Operation code:
 *  - GAPM_SCAN_ACTIVE
 *  - GAPM_SCAN_PASSIVE
 * @param[in] addr_src Own BD address source of the device:
 * - GAPM_STATIC_ADDR
 * - GAPM_GEN_RSLV_ADDR
 * - GAPM_GEN_NON_RSLV_ADDR
 * @param[in] mode Scanning mode:
 * - GAP_GEN_DISCOVERY
 * - GAP_LIM_DISCOVERY
 * - GAP_OBSERVER_MODE
 * @param[in] interval Scan interval N for scanning, Value Time = N * 0.625 ms
 * @param[in] window Scan window size N for scanning, Value Time = N * 0.625 ms
 * @param[in] filt_policy Scan filter policy:
 * - SCAN_ALLOW_ADV_ALL
 * - SCAN_ALLOW_ADV_WLST
 * @param[in] filter_duplic Scan duplicate filtering policy:
 * - SCAN_FILT_DUPLIC_DIS
 * - SCAN_FILT_DUPLIC_EN
 * @response
 * - GAPM_ADV_REPORT_IND
 * - GAPM_CMP_EVT
 *
 * @description
 * Start scanning command. This command is allowed only for device that support observer or central role.
 * When information of a peer device is received, an advertising report event is triggered (see
 * GAPM_ADV_REPORT_IND).
 */
void APP_GapmStartScanCmd(uint8_t code,
                          uint8_t addr_src,
                          uint8_t mode,
                          uint16_t interval,
                          uint16_t window,
                          uint8_t filt_policy,
                          uint8_t filter_duplic);

/*!
 * @brief Set connection initialization Command.
 *
 * @param[in] code Operation code:
 * - GAPM_CONNECTION_DIRECT
 * - GAPM_CONNECTION_AUTO
 * - GAPM_CONNECTION_SELECTIVE
 * - GAPM_CONNECTION_NAME_REQUEST
 * - GAPM_CONNECTION_GENERAL
 * @param[in] addr_src Own BD address source of the device:
 * - GAPM_STATIC_ADDR
 * - GAPM_GEN_RSLV_ADDR
 * - GAPM_GEN_NON_RSLV_ADDR
 * @param[in] con_intv_min Minimum connection interval N, Value Time = N * 1.25 ms
 * @param[in] con_intv_max Maximum of connection interval N, Value Time = N * 1.25 ms
 * @param[in] con_latency Connection latency (number of events)
 * @param[in] superv_to Link supervision timeout N, Value Time = N * 10 ms
 * @param[in] nb_peers Number of peer device information present in message:
 * - 1 for GAPM_CONNECTION_DIRECT or GAPM_CONNECTION_NAME_REQUEST operations
 * - greater than 1 for other operations
 * @param[in] peer Peer device address information
 * @response
 * - GAPM_PEER_NAME_IND
 * - GAPC_CONNECTION_REQ_IND
 * - GAPM_CMP_EVT
 *
 * @description
 * Start connection establishment command. This command is allowed only for device that support central role.
 */
void APP_GapmStartConnectionCmd(uint8_t code,
                                uint8_t addr_src,
                                uint16_t con_intv_min,
                                uint16_t con_intv_max,
                                uint16_t con_latency,
                                uint16_t superv_to,
                                uint8_t nb_peers,
                                struct gap_bdaddr *peer);

/*!
 * @brief Confirm connection to a specific device (Connection Operation in Selective mode).
 *
 * @param[in] peer Peer device address information
 * @param[in] con_intv_min Minimum connection interval N, Value Time = N * 1.25 ms
 * @param[in] con_intv_max Maximum of connection interval N, Value Time = N * 1.25 ms
 * @param[in] con_latency Connection latency (number of events)
 * @param[in] superv_to Link supervision timeout N, Value Time = N * 10 ms
 * @response  GAPC_CONNECTION_REQ_IND
 *
 * @description
 * This message shall be send by application during a selective connection establishment
 * (GAPM_CONNECTION_SELECTIVE). This message is used to confirm which peer device that device should
 * establish a connection. It should be send by application after receiving some advertising report indication of
 * visible devices (GAPM_ADV_REPORT_IND).
 * This message shall contain connection parameters used for selective connection establishment.
 */
void APP_GapmConnectionCfm(
    struct gap_bdaddr *peer, uint16_t con_intv_min, uint16_t con_intv_max, uint16_t con_latency, uint16_t superv_to);

/*!
 * @brief Set specific link data configuration.
 *
 * @param[in] conhdl Connection handle
 * @param[in] lcsrk Local CSRK value
 * @param[in] lsign_counter Local signature counter value
 * @param[in] rcsrk Remote CSRK value
 * @param[in] rsign_counter Remote signature counter value
 * @param[in] auth Authentication:
 * - GAP_AUTH_REQ_NO_MITM_NO_BOND
 * - GAP_AUTH_REQ_NO_MITM_BOND
 * - GAP_AUTH_REQ_MITM_NO_BOND
 * - GAP_AUTH_REQ_MITM_BOND
 * @param[in] svc_changed_ind_enable Service Changed Indication enabled
 * @param[in] ltk_present LTK exchanged during pairing
 * @response  None
 *
 * @description
 * Set specific link security configuration and bonding data:
 * - Set connection bond data
 * - Authentication and authorization link configuration
 * This confirmation message shall be sent by application after receiving a GAPC_CONNECTION_REQ_IND in
 * order to enables local attribute tasks and security manager for the connection.
 * It can be resent later if peer device information is retrieved later (for instance when a master initiates an
 * encryption, information of the LTK can be used to identify peer device). In fact, when encryption is initiated
 * by master device, it uses a couple of encryption diversifier (ediv) and random number (rand_nb) that can be
 * used to retrieve corresponding encryption Long Term Key (LTK) that has been exchanged during a previous
 * connection. By retrieving the LTK, we retrieve a known device and in that case before terminating encryption
 * procedure, application shall update connection parameters.
 *
 * @note
 * If authentication parameter is marked has "Not Bonded", other parameters are ignored and peer
 * device is considered as an unknown device.
 */
void APP_GapcConnectionCfm(uint16_t conhdl,
                           struct gap_sec_key *lcsrk,
                           uint32_t lsign_counter,
                           struct gap_sec_key *rcsrk,
                           uint32_t rsign_counter,
                           uint8_t auth,
                           uint8_t svc_changed_ind_enable,
                           bool ltk_present);

/*!
 * @brief Request disconnection of current link command.
 *
 * @param[in] conhdl Connection handle
 * @response
 * - GAPC_DISCONNECT_IND
 * - GAPC_CMP_EVT
 *
 * @description
 * Request disconnection of link. This can be requested by master or slave of the connection.
 * Reason of disconnection shall be a valid disconnection reason.
 */
#ifdef MBED_PORT
void APP_GapcDisconnectCmd(uint16_t conhdl, uint8_t reason_code);
#else
void APP_GapcDisconnectCmd(uint16_t conhdl);
#endif

/*!
 * @brief Retrieve information command.
 *
 * @param[in] conhdl Connection handle
 * @param[in] operation Requested operation:
 * - GAPC_GET_PEER_NAME
 * - GAPC_GET_PEER_VERSION
 * - GAPC_GET_PEER_FEATURES
 * - GAPC_GET_PEER_APPEARANCE
 * - GAPC_GET_PEER_SLV_PREF_PARAMS
 * - GAPC_GET_CON_RSSI
 * - GAPC_GET_CON_CHANNEL_MAP
 * - GAPC_GET_LE_PING_TO
 * @response
 * - GAPC_PEER_ATT_INFO_IND
 * - GAPC_PEER_VERSION_IND
 * - GAPC_PEER_FEATURES_IND
 * - GAPC_CON_RSSI_IND
 * - GAPC_CON_CHANNEL_MAP_IND
 * - GAPC_LE_PING_TO_VAL_IND
 * - GAPC_CMP_EVT
 *
 * @description
 * Retrieve information about peer device or about the current active link.
 */
void APP_GapcGetInfoCmd(uint16_t conhdl, uint8_t operation);

/*!
 * @brief Retrieve name of peer device.
 *
 * @param[in] conhdl Connection handle
 * @response
 * - GAPC_PEER_ATT_INFO_IND
 * - GAPC_CMP_EVT
 *
 * @description
 * Retrieve name of peer device.
 */
void APP_GapcGetPeerName(uint16_t conhdl);

/*!
 * @brief Retrieve peer device version info.
 *
 * @param[in] conhdl Connection handle
 * @response
 * - GAPC_PEER_VERSION_IND
 * - GAPC_CMP_EVT
 *
 * @description
 * Retrieve peer device version info.
 */
void APP_GapcGetPeerVersion(uint16_t conhdl);

/*!
 * @brief Retrieve peer device features.
 *
 * @param[in] conhdl Connection handle
 * @response
 * - GAPC_PEER_FEATURES_IND
 * - GAPC_CMP_EVT
 *
 * @description
 * Retrieve peer device features.
 */
void APP_GapcGetPeerFeatures(uint16_t conhdl);

/*!
 * @brief Get peer device appearance.
 *
 * @param[in] conhdl Connection handle
 * @response
 * - GAPC_PEER_ATT_INFO_IND
 * - GAPC_CMP_EVT
 *
 * @description
 * Get peer device appearance.
 */
void APP_GapcGetPeerAppearance(uint16_t conhdl);

/*!
 * @brief Get peer device slaved preferred parameters.
 *
 * @param[in] conhdl Connection handle
 * @response
 * - GAPC_PEER_ATT_INFO_IND
 * - GAPC_CMP_EVT
 *
 * @description
 * Get peer device slaved preferred parameters.
 */
void APP_GapcGetPeerSlvPrefParams(uint16_t conhdl);

/*!
 * @brief Retrieve connection RSSI information.
 *
 * @param[in] conhdl Connection handle
 * @response
 * - GAPC_CON_RSSI_IND
 * - GAPC_CMP_EVT
 *
 * @description
 * Retrieve connection RSSI information from peer device.
 */
void APP_GapcGetConRssi(uint16_t conhdl);

/*!
 * @brief Retrieve conection channel map.
 *
 * @param[in] conhdl Connection handle
 * @response
 * - GAPC_CON_CHANNEL_MAP_IND
 * - GAPC_CMP_EVT
 *
 * @description
 * Retrieve conection channel map from peer device.
 */
void APP_GapcGetConChannelMap(uint16_t conhdl);

/*!
 * @brief Retrieve LE ping timeout.
 *
 * @param[in] conhdl Connection handle
 * @response
 * - GAPC_LE_PING_TO_VAL_IND
 * - GAPC_CMP_EVT
 *
 * @description
 * Retrieve LE ping timeout from peer device.
 */
void APP_GapcGetLePingTo(uint16_t conhdl);

/*!
 * @brief Update LE ping timeout value.
 *
 * @param[in] conhdl Connection handle
 * @param[in] timeout Authenticated payload timeout value N, Value Time = N * 10 ms
 * @response GAPC_CMP_EVT
 *
 * @description
 * Change the LE ping authenticated payload timeout value in lower layers for current link.
 */
void APP_GapcSetLePingToCmd(uint16_t conhdl, uint16_t timeout);

/*!
 * @brief LE set data length command.
 *
 * @param[in] conhdl Connection handle
 * @param[in] tx_octets Preferred maximum number of payload octets that the local Controller
 *  should include in a single Link Layer Data Channel PDU
 * @response
 * - GAPC_LE_PKT_SIZE_IND
 * - GAPC_CMP_EVT
 *
 * @description
 * Command used to change current data length extension values in controller.
 */
void APP_GapcSetLePktSizeCmd(uint16_t conhdl, uint8_t tx_octets);

/*!
 * @brief Request to inform the remote device when keys have been entered or erased.
 *
 * @param[in] conhdl Connection handle
 * @param[in] notification_type Key press notification type
 * @response
 * - GAPC_CMP_EVT
 *
 * @description
 * Send a keypress notification to the peer when digit is entered or erased to prevent a timeout.
 */
void APP_GapcKeyPressNotifCmd(uint16_t conhdl, uint8_t notification_type);

/*!
 * @brief Perform update of connection parameters command.
 *
 * @param[in] conhdl Connection handle
 * @param[in] params Connection parameters
 * @response
 * - GAPC_PARAM_UPDATED_IND
 * - GAPC_CMP_EVT
 *
 * @description
 * Connection parameter update command can be used by both master and slave of the connection.
 * For master of the connection, new connection parameters will be applied immediately.
 * For slave of the connection, a connection update message request will be send to master over L2CAP. Then
 * master will be able to accept or refuse those parameters. If master accept them, it will be in charge of
 * applying them.
 *
 * @note
 * If slave of connection request update of connection parameters, a 30s timer will be started in order to
 * let master to reply. If timer ends without response from master, link is automatically disconnected.
 */
void APP_GapcParamUpdateCmd(uint16_t conhdl, struct gapc_conn_param *params);

/*!
 * @brief Master confirm that parameters proposed by slave are accepted or not.
 *
 * @param[in] conhdl Connection handle
 * @param[in] accept accept(0x01), reject(0x00)
 * @response  None
 *
 * @description
 * Used by master of the connection in order to accept or refuse connection parameters proposed by peer
 * slave device.
 */
void APP_GapcParamUpdateCfm(uint16_t conhdl, bool accept);

/*!
 * @brief Start bonding command procedure.
 *
 * @param[in] conhdl Connection handle
 * @param[in] pairing Pairing information
 * @response
 * - GAPC_BOND_REQ_IND
 * - GAPC_BOND_IND
 * - GAPC_CMP_EVT
 *
 * @description
 * This operation can be requested only by master of the link in order to initiate the bond procedure. It
 * contains pairing requirement of initiator.
 */
void APP_GapcBondCmd(uint16_t conhdl, struct gapc_pairing *pairing);

/*!
 * @brief Confirm requested bond information.
 *
 * @param[in] conhdl Connection handle
 * @param[in] request Bond request type (@see gapc_bond)
 * @param[in] accept Request accepted or not
 * @param[in] pairing_info Bond procedure information data (@see gapc_bond_cfm_data)
 * @response  None
 *
 * @description
 * Confirmation message to send after receiving a GAPC_BOND_REQ_IND message
 * This message can contain:
 * - Slave pairing information
 * - Pairing temporary key (TK)
 * - Key to provide to peer device during key exchange.
 */
void APP_GapcBondCfm(uint16_t conhdl, uint8_t request, uint8_t accept, void *pairing_info);

/*!
 * @brief Start encryption command procedure
 *
 * @param[in] conhdl Connection handle
 * @param[in] ltk Long term key information
 * @response
 * - GAPC_ENCRYPT_IND
 * - GAPC_CMP_EVT
 *
 * @description
 * This operation can be requested only by master of the link in order to initiate encryption procedure. It
 * contains Long Term Key that should be used during the encryption.
 */
void APP_GapcEncryptCmd(uint16_t conhdl, struct gapc_ltk *ltk);

/*!
 * @brief Confirm requested encryption information.
 *
 * @param[in] conhdl Connection handle
 * @param[in] found Indicate if a LTK has been found for the peer device
 * @param[in] key_size LTK Key Size
 * @param[in] ltk Long term key information
 * @response None
 *
 * @description
 * Confirmation message to send after receiving a GAPC_ENCRYPT_REQ_IND message
 * This message can is used to inform if encryption key has been found, if yes found Long Term Key and its size
 * shall be provided.
 */
void APP_GapcEncryptCfm(uint16_t conhdl, uint8_t found, struct gapc_ltk *ltk);

/*!
 * @brief Initiate the security request procedure by slave.
 *
 * @param[in] conhdl Connection handle
 * @param[in] auth Authentification level:
 * - GAP_AUTH_REQ_NO_MITM_NO_BOND
 * - GAP_AUTH_REQ_NO_MITM_BOND
 * - GAP_AUTH_REQ_MITM_NO_BOND
 * - GAP_AUTH_REQ_MITM_BOND
 * @response
 * - GAPC_CMP_EVT
 *
 * @description
 * This operation can be requested only by slave of the link in order to initiate security request procedure. It
 * contains authentication level requested by current device.
 */
void APP_GapcSecurityCmd(uint16_t conhdl, uint8_t auth);

/*!
 * @brief GPAM command complete event handler.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAPM).
 * @param[in] src_id    ID of the sending task instance.
 * @return If the message was consumed or not.
 *
 * @description
 * This is the generic complete event for GAP operations. All operations trigger this
 * event when operation is finished
 */
int APP_GapmCmpEvtHandler(ke_msg_id_t const msgid,
                          struct gapm_cmp_evt const *param,
                          ke_task_id_t const dest_id,
                          ke_task_id_t const src_id);

/*!
 * @brief Lower layers are ready event handler.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAPM).
 * @param[in] src_id    ID of the sending task instance.
 * @return If the message was consumed or not.
 *
 * @description
 * Event triggered at system power-up in order to inform that BLE Lower Layers are ready.
 */
int APP_GapmDeviceReadyIndHandler(ke_msg_id_t const msgid,
                                  void const *param,
                                  ke_task_id_t const dest_id,
                                  ke_task_id_t const src_id);

/*!
 * @brief White list size indication event handler.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAPM).
 * @param[in] src_id    ID of the sending task instance.
 * @return If the message was consumed or not.
 *
 * @description
 * Event triggered when size of white list is requested. Inform application about size of lower layer white list.
 */
int APP_GapmWhiteListSizeIndHandler(ke_msg_id_t const msgid,
                                    struct gapm_white_list_size_ind const *param,
                                    ke_task_id_t const dest_id,
                                    ke_task_id_t const src_id);

/*!
 * @brief Resolving address list size indication event handler.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAPM).
 * @param[in] src_id    ID of the sending task instance.
 * @return If the message was consumed or not.
 *
 * @description
 * Event triggered when size of resolving list is requested. Inform the application about the lower layer
 * resolving list size.
 */
int APP_GapmRalSizeIndHandler(ke_msg_id_t const msgid,
                              struct gapm_ral_size_ind const *param,
                              ke_task_id_t const dest_id,
                              ke_task_id_t const src_id);

/*!
 * @brief Resolving Address indication event handler.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAPM).
 * @param[in] src_id    ID of the sending task instance.
 * @return If the message was consumed or not.
 *
 * @description
 * Event triggered when local or peer resolvable address is requested.
 */
int APP_GapmRalAddrIndHandler(ke_msg_id_t const msgid,
                              struct gapm_ral_addr_ind const *param,
                              ke_task_id_t const dest_id,
                              ke_task_id_t const src_id);

/*!
 * @brief Local device version indication event handler.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAPM).
 * @param[in] src_id    ID of the sending task instance.
 * @return If the message was consumed or not.
 *
 * @description
 * Event containing local device version and manufacturer name.
 */
int APP_GapmDevVersionIndHandler(ke_msg_id_t const msgid,
                                 struct gapm_dev_version_ind const *param,
                                 ke_task_id_t const dest_id,
                                 ke_task_id_t const src_id);

/*!
 * @brief Local device BD address indication event handler
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAPM).
 * @param[in] src_id    ID of the sending task instance.
 * @return If the message was consumed or not.
 *
 * @description
 * This event can be triggered when reading local BD Address, but also when starting an air operation
 * (advertising, connecting, scanning) in order to inform application about the used random address.
 *
 * This event is also triggered when generating a random address using security toolbox (see
 * GAPM_GEN_RAND_ADDR_CMD)
 */
int APP_GapmDevBdAddrIndHandler(ke_msg_id_t const msgid,
                                struct gapm_dev_bdaddr_ind const *param,
                                ke_task_id_t const dest_id,
                                ke_task_id_t const src_id);

/*!
 * @brief Advertising channel Tx power level indication event handler.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAPM).
 * @param[in] src_id    ID of the sending task instance.
 * @return If the message was consumed or not.
 *
 * @description
 * Event triggered when application request advertising TX power level.
 */
int APP_GapmDevAdvTxPowerIndHandler(ke_msg_id_t const msgid,
                                    struct gapm_dev_adv_tx_power_ind const *param,
                                    ke_task_id_t const dest_id,
                                    ke_task_id_t const src_id);

/*!
 * @brief Host's suggested default data length indication event handler.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAPM).
 * @param[in] src_id    ID of the sending task instance.
 * @return If the message was consumed or not.
 *
 * @description
 * Event triggered when application requests suggested data length values.
 */
int APP_GapmSuggDfltDataLenIndHandler(ke_msg_id_t const msgid,
                                      struct gapm_sugg_dflt_data_len_ind const *param,
                                      ke_task_id_t const dest_id,
                                      ke_task_id_t const src_id);

/*!
 * @brief Maximum data length indication event handler.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAPM).
 * @param[in] src_id    ID of the sending task instance.
 * @return If the message was consumed or not.
 *
 * @description
 * Event triggered when application requests the maximum data length supported by controller.
 */
int APP_GapmMaxDataLenIndHandler(ke_msg_id_t const msgid,
                                 struct gapm_max_data_len_ind const *param,
                                 ke_task_id_t const dest_id,
                                 ke_task_id_t const src_id);

/*!
 * @brief AES-128 block result indication event handler.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAPM).
 * @param[in] src_id    ID of the sending task instance.
 * @return If the message was consumed or not.
 *
 * @description
 * Event triggered when AES-128 encryption calculation has been performed.
 */
int APP_GapmUseEncBlockIndHandler(ke_msg_id_t const msgid,
                                  struct gapm_use_enc_block_ind const *param,
                                  ke_task_id_t const dest_id,
                                  ke_task_id_t const src_id);

/*!
 * @brief Resolvable random address has been solved indication event handler.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAPM).
 * @param[in] src_id    ID of the sending task instance.
 * @return If the message was consumed or not.
 *
 * @description
 * Triggered if provided BD address has been successfully resolved. It indicates which key has been used to
 * resolve the address and index of the key in the provided array of keys.
 */
int APP_GapmAddrSolvedIndHandler(ke_msg_id_t const msgid,
                                 struct gapm_addr_solved_ind const *param,
                                 ke_task_id_t const dest_id,
                                 ke_task_id_t const src_id);

#if (BLE_CENTRAL)
/*!
 * @brief Advertising or scanning report information event handler.
 *
 * @param[in] msgid     GAPM_ADV_REPORT_IND
 * @param[in] param     Pointer to struct gapm_adv_report_ind
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAPM).
 * @param[in] src_id    ID of the sending task instance.
 * @return If the message was consumed or not.
 *
 * @description
 * Event triggered when scanning operation of selective connection establishment procedure receive
 * advertising report information.
 *
 * @note
 * Several advertising report can be received for a peer device if scan response is available or if
 * scan duplicate filter policy is disabled
 * According to executed procedure, some advertising report can be filtered by GAP Manager.
 */
int APP_GapmAdvReportIndHandler(ke_msg_id_t const msgid,
                                struct gapm_adv_report_ind const *param,
                                ke_task_id_t const dest_id,
                                ke_task_id_t const src_id);
#endif

/*!
 * @brief Name of peer device indication event handler.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAPM).
 * @param[in] src_id    ID of the sending task instance.
 * @return If the message was consumed or not.
 *
 * @description
 * This message is triggered during a Name Request operation (GAPM_CONNECTION_NAME_REQUEST). This
 * signal contains name of peer device and device Bluetooth address information.
 */
int APP_GapmPeerNameIndHandler(ke_msg_id_t const msgid,
                               struct gapm_peer_name_ind const *param,
                               ke_task_id_t const dest_id,
                               ke_task_id_t const src_id);

#if (BLE_PROFILES)
/*!
 * @brief Inform that profile task has been added.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAPM).
 * @param[in] src_id    ID of the sending task instance.
 * @return If the message was consumed or not.
 *
 * @description
 * Event triggered when a profile task is added. Used to know task number allocated for profile.
 */
int APP_GapmProfileAddedIndHandler(ke_msg_id_t const msgid,
                                   struct gapm_profile_added_ind const *param,
                                   ke_task_id_t const dest_id,
                                   ke_task_id_t const src_id);
#endif

/*!
 * @brief GAPC command complete event handler.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAPC).
 * @param[in] src_id    ID of the sending task instance.
 * @return If the message was consumed or not.
 *
 * @description
 * Complete event for GAP operation. This is the generic complete event for GAP operations. All operation
 * triggers this event when operation is finished.
 */
int APP_GapcCmpEvtHandler(ke_msg_id_t const msgid,
                          struct gapc_cmp_evt const *param,
                          ke_task_id_t const dest_id,
                          ke_task_id_t const src_id);

/*!
 * @brief Indicate that a connection has been established.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAPC).
 * @param[in] src_id    ID of the sending task instance.
 * @return If the message was consumed or not.
 *
 * @description
 * Inform that a connection has been established with a peer device. This message is a request because it is
 * waiting for GAPC_CONNECTION_CFM message in order to:
 * - Set connection bond data
 * - Authentication and authorization link configuration
 * The confirmation message will then enable the attribute database and security manager in order to process
 * requests from peer device.
 * Before sending confirmation message, application can perform address resolution in order to retrieve if it's a
 * known device and also start some services.
 * When a link is established, a corresponding task instance is created for all connection related tasks (GATTC, L2CC).
 */
int APP_GapcConnectionReqIndHandler(ke_msg_id_t const msgid,
                                    struct gapc_connection_req_ind const *param,
                                    ke_task_id_t const dest_id,
                                    ke_task_id_t const src_id);

/*!
 * @brief Indicate that a link has been disconnected.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAPC).
 * @param[in] src_id    ID of the sending task instance.
 * @return If the message was consumed or not.
 *
 * @description
 * Event sent to application task in order to inform that link has been disconnected. Receiving this message also
 * means that task instances related to the link are cleaned-up and corresponding task instances cannot be
 * used anymore until new connection is established.
 */
int APP_GapcDisconnectIndHandler(ke_msg_id_t const msgid,
                                 struct gapc_disconnect_ind const *param,
                                 ke_task_id_t const dest_id,
                                 ke_task_id_t const src_id);

/*!
 * @brief Peer device attribute DB info indication.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAPC).
 * @param[in] src_id    ID of the sending task instance.
 * @return If the message was consumed or not.
 *
 * @description
 * Event triggered when requesting peer device attribute DB info such as device name, appearance, slave
 * preferred parameters or device central address resolution.
 */
int APP_GapcPeerAttInfoIndHandler(ke_msg_id_t const msgid,
                                  struct gapc_peer_att_info_ind const *param,
                                  ke_task_id_t const dest_id,
                                  ke_task_id_t const src_id);

/*!
 * @brief Peer version info indication event handler.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAPC).
 * @param[in] src_id    ID of the sending task instance.
 * @return If the message was consumed or not.
 *
 * @description
 * Event triggered when peer device version is requested.
 */
int APP_GapcPeerVersionIndHandler(ke_msg_id_t const msgid,
                                  struct gapc_peer_version_ind const *param,
                                  ke_task_id_t const dest_id,
                                  ke_task_id_t const src_id);

/*!
 * @brief Peer features info indication event handler.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAPC).
 * @param[in] src_id    ID of the sending task instance.
 * @return If the message was consumed or not.
 *
 * @description
 * Event triggered when peer device features are requested.
 */
int APP_GapcPeerFeaturesIndHandler(ke_msg_id_t const msgid,
                                   struct gapc_peer_features_ind const *param,
                                   ke_task_id_t const dest_id,
                                   ke_task_id_t const src_id);

/*!
 * @brief Ongoing connection RSSI indication event handler.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAPC).
 * @param[in] src_id    ID of the sending task instance.
 * @return If the message was consumed or not.
 *
 * @description
 * Event triggered when connection RSSI is requested.
 */
int APP_GapcConRssiIndHandler(ke_msg_id_t const msgid,
                              struct gapc_con_rssi_ind const *param,
                              ke_task_id_t const dest_id,
                              ke_task_id_t const src_id);

/*!
 * @brief The updated sign counters indication event handler.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAPC).
 * @param[in] src_id    ID of the sending task instance.
 * @return If the message was consumed or not.
 *
 * @description
 * Event triggered when one of the sign counter has been updated.
 */
int APP_GapcSignCounterIndHandler(ke_msg_id_t const msgid,
                                  struct gapc_sign_counter_ind const *param,
                                  ke_task_id_t const dest_id,
                                  ke_task_id_t const src_id);

/*!
 * @brief Ongoing connection channel map indication event handler.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAPC).
 * @param[in] src_id    ID of the sending task instance.
 * @return If the message was consumed or not.
 *
 * @description
 * Event triggered when connection channel map is requested.
 */
int APP_GapcConChannelMapIndHandler(ke_msg_id_t const msgid,
                                    struct gapc_con_channel_map_ind const *param,
                                    ke_task_id_t const dest_id,
                                    ke_task_id_t const src_id);

/*!
 * @brief LE ping indication event handler.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAPC).
 * @param[in] src_id    ID of the sending task instance.
 * @return If the message was consumed or not.
 *
 * @description
 * Indication of LE Ping timeout value.
 */
int APP_GapcLePingToValIndHandler(ke_msg_id_t const msgid,
                                  struct gapc_le_ping_to_val_ind const *param,
                                  ke_task_id_t const dest_id,
                                  ke_task_id_t const src_id);

/*!
 * @brief Peer device request local device info.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAPC).
 * @param[in] src_id    ID of the sending task instance.
 * @return If the message was consumed or not.
 *
 * @description
 * Event triggered when peer device requests local device info such as name, appearance, slave preferred
 * parameters or device Central address resolution. Application should answer with GAPC_GET_DEV_INFO_CFM message.
 * This value is not present in host stack and should be managed by application to reduce size of GAP attribute
 * database.
 */
int APP_GapcGetDevInfoReqIndHandler(ke_msg_id_t const msgid,
                                    struct gapc_get_dev_info_req_ind const *param,
                                    ke_task_id_t const dest_id,
                                    ke_task_id_t const src_id);

/*!
 * @brief Handles GAPC_SET_DEV_INFO_REQ_IND message.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 * @return If the message was consumed or not.
 *
 * @description
 * Peer device request to modify local device info such as name or appearance
 */
int APP_GapcSetDevInfoReqIndHandler(ke_msg_id_t const msgid,
                                    struct gapc_set_dev_info_req_ind const *param,
                                    ke_task_id_t const dest_id,
                                    ke_task_id_t const src_id);

/*!
 * @brief LE set data length indication event handler.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAPC).
 * @param[in] src_id    ID of the sending task instance.
 * @return If the message was consumed or not.
 *
 * @description
 * Event triggered when local data length extension parameters are modified either using
 * GAPM_SET_DEV_CONFIG_CMD to define new suggested values or GAPC_SET_LE_PKT_SIZE_CMD to define
 * the preferred packet length to be used by the controller.
 */
int APP_GapcLePktSizeIndHandler(ke_msg_id_t const msgid,
                                struct gapc_le_pkt_size_ind const *param,
                                ke_task_id_t const dest_id,
                                ke_task_id_t const src_id);

/*!
 * @brief Request of updating connection parameters indication event handler.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAPC).
 * @param[in] src_id    ID of the sending task instance.
 * @return If the message was consumed or not.
 *
 * @description
 * This message event is triggered on master when slave of the connection requests to update connection
 * parameters.
 * This message shall be followed by GAPC_PARAM_UPDATE_CFM message to accept or not new connection
 * parameters.
 */
int APP_GapcParamUpdateReqIndHandler(ke_msg_id_t const msgid,
                                     struct gapc_param_update_req_ind const *param,
                                     ke_task_id_t const dest_id,
                                     ke_task_id_t const src_id);

/*!
 * @brief Connection parameters updated indication event handler.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAPC).
 * @param[in] src_id    ID of the sending task instance.
 * @return If the message was consumed or not.
 *
 * @description
 * Event triggered when parameters of the connection have been updated.
 */
int APP_GapcParamUpdatedIndHandler(ke_msg_id_t const msgid,
                                   struct gapc_param_updated_ind const *param,
                                   ke_task_id_t const dest_id,
                                   ke_task_id_t const src_id);

/*!
 * @brief Bonding requested by peer device indication event handler.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAPC).
 * @param[in] src_id    ID of the sending task instance.
 * @return If the message was consumed or not.
 *
 * @description
 * Event Triggered during a bonding procedure in order to get:
 * - Slave pairing information
 * - Pairing temporary key (TK)
 * - Key to provide to peer device during key exchange.
 * This event shall be followed by a GAPC_BOND_CFM message with same request code value.
 */
int APP_GapcBondReqIndHandler(ke_msg_id_t const msgid,
                              struct gapc_bond_req_ind const *param,
                              ke_task_id_t const dest_id,
                              ke_task_id_t const src_id);

/*!
 * @brief Bonding information indication event handler.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAPC).
 * @param[in] src_id    ID of the sending task instance.
 * @return If the message was consumed or not.
 *
 * @description
 * Event triggered when bonding information is available such as:
 * - Status of the pairing (succeed or failed)
 * - Key exchanged by peer device.
 */
int APP_GapcBondIndHandler(ke_msg_id_t const msgid,
                           struct gapc_bond_ind const *param,
                           ke_task_id_t const dest_id,
                           ke_task_id_t const src_id);

/*!
 * @brief Encryption requested by peer device indication event handler.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAPC).
 * @param[in] src_id    ID of the sending task instance.
 * @return If the message was consumed or not.
 *
 * @description
 * Event Triggered during encryption procedure on slave device in order to retrieve LTK according to random
 * number and encryption diversifier value.
 * This event shall be followed by a GAPC_ENCRYPT_CFM message.
 */
int APP_GapcEncryptReqIndHandler(ke_msg_id_t const msgid,
                                 struct gapc_encrypt_req_ind const *param,
                                 ke_task_id_t const dest_id,
                                 ke_task_id_t const src_id);

/*!
 * @brief Encryption information indication event handler.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAPC).
 * @param[in] src_id    ID of the sending task instance.
 * @return If the message was consumed or not.
 *
 * @description
 * Event triggered when encryption procedure succeed, it contains the link authentication level provided
 * during connection confirmation (see GAPC_CONNECTION_CFM)
 */
int APP_GapcEncryptIndHandler(ke_msg_id_t const msgid,
                              struct gapc_encrypt_ind const *param,
                              ke_task_id_t const dest_id,
                              ke_task_id_t const src_id);

/*!
 * @brief Security requested by peer device indication event handler.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAPC).
 * @param[in] src_id    ID of the sending task instance.
 * @return If the message was consumed or not.
 *
 * @description
 * Event Triggered on master side when slave request to have a certain level of authentication.
 */
int APP_GapcSecurityIndHandler(ke_msg_id_t const msgid,
                               struct gapc_security_ind const *param,
                               ke_task_id_t const dest_id,
                               ke_task_id_t const src_id);

/*! @brief @} APP_GAP_API */

#endif /* _APP_GAP_H_ */

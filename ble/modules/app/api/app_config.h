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
#ifndef _APP_CONFIG_H_
#define _APP_CONFIG_H_

#include "system_config.h"
#include "ble_config.h"
#include "profile_config.h"
#include "fsl_debug_console.h"

#include "rwip_config.h"
#include "gapm_task.h" /* GAP Manager Task API */
#include "gapc_task.h" /* GAP Controller Task API */
#include "smpc_api.h"
#include "gattc_task.h"
#include "gattm_task.h"
#include "gapm_util.h"
#include "l2cc_task.h"
#include "rwip.h"
#include "ke_timer.h"

#if defined(CFG_DBG_PRINT)
#define QPRINTF PRINTF
#else
#define QPRINTF(fmt, ...)
#endif

/*! @brief GAP role for code reduction */
#if (BLE_CENTRAL && BLE_PERIPHERAL && BLE_BROADCASTER && BLE_OBSERVER)
#define CFG_GAP_ROLE GAP_ROLE_ALL
#elif(BLE_PERIPHERAL)
#define CFG_GAP_ROLE GAP_ROLE_PERIPHERAL
#elif(BLE_CENTRAL)
#define CFG_GAP_ROLE GAP_ROLE_CENTRAL
#elif(BLE_BROADCASTER)
#define CFG_GAP_ROLE GAP_ROLE_BROADCASTER
#elif(BLE_OBSERVER)
#define CFG_GAP_ROLE GAP_ROLE_OBSERVER
#endif

#if defined(__ICCARM__)

__weak void APP_ButtonDownCallback(uint32_t pin_mask);

__weak void APP_GapSetDevCfgCallback(void);

__weak void APP_BleReadyCallback(void);

__weak void APP_AdvertisingStopCallback(uint8_t status);

__weak void APP_ScanningStopCallback(uint8_t status);

__weak void APP_InitiatingStopCallback(uint8_t status);

__weak void APP_AdvReportIndCallback(struct gapm_adv_report_ind const *param);

__weak void APP_ConnectionEstablishedCallback(struct gapc_connection_req_ind const *param);

__weak void APP_ConnectionTerminatedCallback(struct gapc_disconnect_ind const *param);

__weak void APP_TkExchangeCallback(uint16_t conhdl, uint8_t tk_type, uint32_t pin_code);

__weak void APP_NcExchangeCallback(uint16_t conhdl, uint32_t nc_value);

__weak void APP_BondSuccessCallback(uint16_t conhdl, struct gapc_bond_ind const *param);

__weak void APP_ProxrAlertCallback(uint8_t char_code, uint8_t alert_lvl);

__weak void APP_FindtAlertCallback(uint8_t alert_lvl);

__weak void APP_QppcReceivedDataCallback(uint16_t conhdl, uint8_t *data, uint16_t length);

__weak void APP_QppcSendDataRspCallback(uint16_t conhdl, uint8_t status);

__weak void APP_QppsReceivedDataCallback(uint16_t conhdl, uint8_t *data, uint16_t length);

__weak void APP_QppsCfgIndntfIndCallback(uint16_t conhdl, uint8_t status);

__weak void APP_QppsSendDataRspCallback(uint16_t conhdl, uint8_t status);

__weak void APP_CppsCtnlPtReqIndCallback(void *param);

__weak void APP_CppsGetAdvDataRspCallback(uint8_t data_len, uint8_t *adv_data);

__weak void APP_LansLnCtnlPtReqIndCallback(void *param);

__weak uint8_t APP_RscpsCntlPtStartCalibCallback(void);

__weak void APP_PaspsSetRingerModeCallback(uint8_t mode);

__weak void APP_HogpdCtnlPtIndCallback(uint8_t hid_idx, uint8_t hid_ctnl_pt);

__weak void APP_HogpdReportReqIndCallback(uint8_t operation, void *report);

__weak void APP_TipsRdReqIndCallback(uint8_t char_code);

__weak void APP_TipsTimeUpdCtnlPtIndCallback(uint8_t value);

__weak void APP_AnpsNtfImmediateReqIndCallback(uint8_t alert_type, uint16_t cat_ntf_cfg);

#elif defined(__CC_ARM)

__weak void APP_ButtonDownCallback(uint32_t pin_mask) __attribute__((used));

__weak void APP_GapSetDevCfgCallback(void) __attribute__((used));

__weak void APP_BleReadyCallback(void) __attribute__((used));

__weak void APP_AdvertisingStopCallback(uint8_t status) __attribute__((used));

__weak void APP_ScanningStopCallback(uint8_t status) __attribute__((used));

__weak void APP_InitiatingStopCallback(uint8_t status) __attribute__((used));

__weak void APP_AdvReportIndCallback(struct gapm_adv_report_ind const *param) __attribute__((used));

__weak void APP_ConnectionEstablishedCallback(struct gapc_connection_req_ind const *param) __attribute__((used));

__weak void APP_ConnectionTerminatedCallback(struct gapc_disconnect_ind const *param) __attribute__((used));

__weak void APP_NcExchangeCallback(uint16_t conhdl, uint32_t nc_value) __attribute__((used));

__weak void APP_TkExchangeCallback(uint16_t conhdl, uint8_t tk_type, uint32_t pin_code) __attribute__((used));

__weak void APP_BondSuccessCallback(uint16_t conhdl, struct gapc_bond_ind const *param) __attribute__((used));

__weak void APP_ProxrAlertCallback(uint8_t char_code, uint8_t alert_lvl) __attribute__((used));

__weak void APP_FindtAlertCallback(uint8_t alert_lvl) __attribute__((used));

__weak void APP_QppcReceivedDataCallback(uint16_t conhdl, uint8_t *data, uint16_t length) __attribute__((used));

__weak void APP_QppcSendDataRspCallback(uint16_t conhdl, uint8_t status) __attribute__((used));

__weak void APP_QppsReceivedDataCallback(uint16_t conhdl, uint8_t *data, uint16_t length) __attribute__((used));

__weak void APP_QppsCfgIndntfIndCallback(uint16_t conhdl, uint8_t status) __attribute__((used));

__weak void APP_QppsSendDataRspCallback(uint16_t conhdl, uint8_t status) __attribute__((used));

__weak void APP_CppsCtnlPtReqIndCallback(void *param) __attribute__((used));

__weak void APP_CppsGetAdvDataRspCallback(uint8_t data_len, uint8_t *adv_data) __attribute__((used));

__weak void APP_LansLnCtnlPtReqIndCallback(void *param) __attribute__((used));

__weak uint8_t APP_RscpsCntlPtStartCalibCallback(void) __attribute__((used));

__weak void APP_PaspsSetRingerModeCallback(uint8_t mode) __attribute__((used));

__weak void APP_HogpdCtnlPtIndCallback(uint8_t hid_idx, uint8_t hid_ctnl_pt) __attribute__((used));

__weak void APP_HogpdReportReqIndCallback(uint8_t operation, void *report) __attribute__((used));

__weak void APP_TipsRdReqIndCallback(uint8_t char_code) __attribute__((used));

__weak void APP_TipsTimeUpdCtnlPtIndCallback(uint8_t value) __attribute__((used));

__weak void APP_AnpsNtfImmediateReqIndCallback(uint8_t alert_type, uint16_t cat_ntf_cfg) __attribute__((used));

#else /* __GUNC__ */

void APP_ButtonDownCallback(uint32_t pin_mask) __attribute__((weak));

void APP_GapSetDevCfgCallback(void) __attribute__((weak));

void APP_BleReadyCallback(void) __attribute__((weak));

void APP_AdvertisingStopCallback(uint8_t status) __attribute__((weak));

void APP_ScanningStopCallback(uint8_t status) __attribute__((weak));

void APP_InitiatingStopCallback(uint8_t status) __attribute__((weak));

void APP_AdvReportIndCallback(struct gapm_adv_report_ind const *param) __attribute__((weak));

void APP_ConnectionEstablishedCallback(struct gapc_connection_req_ind const *param) __attribute__((weak));

void APP_ConnectionTerminatedCallback(struct gapc_disconnect_ind const *param) __attribute__((weak));

void APP_TkExchangeCallback(uint16_t conhdl, uint8_t tk_type, uint32_t pin_code) __attribute__((weak));

void APP_NcExchangeCallback(uint16_t conhdl, uint32_t nc_value) __attribute__((weak));

void APP_BondSuccessCallback(uint16_t conhdl, struct gapc_bond_ind const *param) __attribute__((weak));

void APP_ProxrAlertCallback(uint8_t char_code, uint8_t alert_lvl) __attribute__((weak));

void APP_FindtAlertCallback(uint8_t alert_lvl) __attribute__((weak));

void APP_QppcReceivedDataCallback(uint16_t conhdl, uint8_t *data, uint16_t length) __attribute__((weak));

void APP_QppcSendDataRspCallback(uint16_t conhdl, uint8_t status) __attribute__((weak));

void APP_QppsReceivedDataCallback(uint16_t conhdl, uint8_t *data, uint16_t length) __attribute__((weak));

void APP_QppsCfgIndntfIndCallback(uint16_t conhdl, uint8_t status) __attribute__((weak));

void APP_QppsSendDataRspCallback(uint16_t conhdl, uint8_t status) __attribute__((weak));

void APP_CppsCtnlPtReqIndCallback(void *param) __attribute__((weak));

void APP_CppsGetAdvDataRspCallback(uint8_t data_len, uint8_t *adv_data) __attribute__((weak));

void APP_LansLnCtnlPtReqIndCallback(void *param) __attribute__((weak));

uint8_t APP_RscpsCntlPtStartCalibCallback(void) __attribute__((weak));

void APP_PaspsSetRingerModeCallback(uint8_t mode) __attribute__((weak));

void APP_HogpdCtnlPtIndCallback(uint8_t hid_idx, uint8_t hid_ctnl_pt) __attribute__((weak));

void APP_HogpdReportReqIndCallback(uint8_t operation, void *report) __attribute__((weak));

void APP_TipsRdReqIndCallback(uint8_t char_code) __attribute__((weak));

void APP_TipsTimeUpdCtnlPtIndCallback(uint8_t value) __attribute__((weak));

void APP_AnpsNtfImmediateReqIndCallback(uint8_t alert_type, uint16_t cat_ntf_cfg) __attribute__((weak));

#endif

#endif /* _APP_CONFIG_H_ */

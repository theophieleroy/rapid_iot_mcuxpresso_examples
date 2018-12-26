/* mbed Microcontroller Library
 * Copyright (c) 2006-2013 ARM Limited
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef __NXP_GAP_H__
#define __NXP_GAP_H__

//#ifdef YOTTA_CFG_MBED_OS
//    #include "mbed-drivers/mbed.h"
//#else
//    #include "mbed.h"
//#endif
#ifndef YOTTA_CFG_WHITELIST_MAX_SIZE
#define YOTTA_CFG_WHITELIST_MAX_SIZE BLE_GAP_WHITELIST_ADDR_MAX_COUNT
#elif YOTTA_CFG_WHITELIST_MAX_SIZE > BLE_GAP_WHITELIST_ADDR_MAX_COUNT
#undef YOTTA_CFG_WHITELIST_MAX_SIZE
#define YOTTA_CFG_WHITELIST_MAX_SIZE BLE_GAP_WHITELIST_ADDR_MAX_COUNT
#endif
#ifndef YOTTA_CFG_IRK_TABLE_MAX_SIZE
#define YOTTA_CFG_IRK_TABLE_MAX_SIZE BLE_GAP_WHITELIST_IRK_MAX_COUNT
#elif YOTTA_CFG_IRK_TABLE_MAX_SIZE > BLE_GAP_WHITELIST_IRK_MAX_COUNT
#undef YOTTA_CFG_IRK_TABLE_MAX_SIZE
#define YOTTA_CFG_IRK_TABLE_MAX_SIZE BLE_GAP_WHITELIST_IRK_MAX_COUNT
#endif
#include "blecommon.h"
#include "GapAdvertisingParams.h"
#include "GapAdvertisingData.h"
#include "mbedGap.h"
#include "GapScanningParams.h"

extern "C" {
#include "ble_general.h"
#include "gatt_db_app_interface.h"
#include "gatt_server_interface.h"
#include "gap_interface.h"
#include "gatt_db_dynamic.h"
#include "mbed_gatt_database_dynamic.h"
#include "timer_manager.h"
#include "ble_conn_manager.h"
}

/**************************************************************************/
/*!
    \brief

*/
/**************************************************************************/
class nxpGap : public Gap
{
   public:
#define CFG_WHITELIST_MAX_SIZE 20
    typedef struct
    {
        bool_t advOn;
        bool_t advFast;
    } advState_t;

    typedef struct
    {
        bleIdentityAddress_t address;
        gapRole_t role;
    } deviceRole_t;
    /* Functions that must be implemented from Gap */
    virtual ble_error_t setAddress(AddressType_t type, const Address_t address);
    virtual ble_error_t getAddress(AddressType_t *typeP, Address_t address);
    virtual ble_error_t setAdvertisingData(const GapAdvertisingData &, const GapAdvertisingData &);

    virtual uint16_t getMinAdvertisingInterval(void) const
    {
        return GapAdvertisingParams::ADVERTISEMENT_DURATION_UNITS_TO_MS(gGapAdvertisingIntervalRangeMinimum_c);
    }
    virtual uint16_t getMinNonConnectableAdvertisingInterval(void) const
    {
        return GapAdvertisingParams::ADVERTISEMENT_DURATION_UNITS_TO_MS(gGapAdvertisingIntervalRangeMinimum_c);
    }
    virtual uint16_t getMaxAdvertisingInterval(void) const
    {
        return GapAdvertisingParams::ADVERTISEMENT_DURATION_UNITS_TO_MS(gGapAdvertisingIntervalRangeMaximum_c);
    }

    virtual ble_error_t startAdvertising(const GapAdvertisingParams &);
    virtual ble_error_t stopAdvertising(void);
    virtual ble_error_t connect(const BLEProtocol::AddressBytes_t peerAddr,
                                BLEProtocol::AddressType_t peerAddrType,
                                const ConnectionParams_t *connectionParams,
                                const GapScanningParams *scanParams);
    virtual ble_error_t disconnect(Handle_t connectionHandle, DisconnectionReason_t reason);
    virtual ble_error_t disconnect(DisconnectionReason_t reason);

    virtual ble_error_t setDeviceName(const uint8_t *deviceName);
    virtual ble_error_t getDeviceName(uint8_t *deviceName, unsigned *lengthP);
    virtual ble_error_t setAppearance(GapAdvertisingData::Appearance appearance);
    virtual ble_error_t getAppearance(GapAdvertisingData::Appearance *appearanceP);
    //    virtual ble_error_t setTxPower(int8_t txPower);
    //    virtual void        getPermittedTxPowerValues(const int8_t **valueArrayPP, size_t *countP);

    void setConnectionHandle(uint16_t con_handle);
    uint16_t getConnectionHandle(void);

    virtual ble_error_t getPreferredConnectionParams(ConnectionParams_t *params);
    virtual ble_error_t setPreferredConnectionParams(const ConnectionParams_t *params);
    virtual ble_error_t updateConnectionParams(Handle_t handle, const ConnectionParams_t *params);

    virtual ble_error_t reset(void);

    /*
     * The following functions are part of the whitelisting experimental API.
     * Therefore, this functionality can change in the near future.
     */
    virtual uint8_t getMaxWhitelistSize(void) const;
    virtual ble_error_t getWhitelist(Gap::Whitelist_t &whitelistOut) const;
    virtual ble_error_t setWhitelist(const Gap::Whitelist_t &whitelistIn);

    virtual ble_error_t setAdvertisingPolicyMode(AdvertisingPolicyMode_t mode);
    virtual ble_error_t setScanningPolicyMode(ScanningPolicyMode_t mode);
    virtual ble_error_t setInitiatorPolicyMode(InitiatorPolicyMode_t mode);
    virtual Gap::AdvertisingPolicyMode_t getAdvertisingPolicyMode(void) const;
    virtual Gap::ScanningPolicyMode_t getScanningPolicyMode(void) const;
    virtual Gap::InitiatorPolicyMode_t getInitiatorPolicyMode(void) const;

    GapScanningParams &getScanningParams(void)
    {
        return _scanningParams;
    }
    static void ScanningCallback(gapScanningEvent_t *pScanningEvent);
    static void ScanningTimerCallback(void *pParam);
    static void ConnectionCallback(deviceId_t peerDeviceId, gapConnectionEvent_t *pConnectionEvent);
    static void AdvertisingCallback(gapAdvertisingEvent_t *pAdvertisingEvent);
    static void AdvertisingTimerCallback(void *pParam);

    void AdvertisingHandler(gapAdvertisingEvent_t *pAdvertisingEvent);
    void ScanningEventHandler(gapScanningEvent_t *pScanningEvent);
    void ConnectionEventHandler(deviceId_t peerDeviceId, gapConnectionEvent_t *pConnectionEvent);
    void AdvertisingTimerHandler(void *pParam);
    void ScanningTimerHandler(void *pParam);
    void gapInit(gapServiceHandles_t handles);
    void advDataSetupComplete(void);
    void advParamSetupComplete(void);
    void readBleAddressComplete(uint8_t *addr);
    void setRandomAddressComplete(void);
    void clrWhiteListComplete(void);
    void AddWhiteListComplete(void);
    gapRole_t getDeviceRole(Gap::Handle_t connectionHandle);

    /*   virtual ble_error_t initRadioNotification(void) {
           return BLE_ERROR_UNSPECIFIED;
       }*/

    virtual ble_error_t startRadioScan(const GapScanningParams &scanningParams);
    virtual ble_error_t stopScan(void);

    nxpGap()
        : advertisingPolicyMode(Gap::ADV_POLICY_IGNORE_WHITELIST),
          scanningPolicyMode(Gap::SCAN_POLICY_IGNORE_WHITELIST),
          initiatorPolicyMode(Gap::INIT_POLICY_IGNORE_WHITELIST),
          whitelistAddressesSize(0)
    {
        connectInitiated = FALSE;
        gapReqCompleteFlags = 0;
        advState.advOn = FALSE;
        advState.advFast = TRUE;
        restartAdv = FALSE;
        scanningOn = FALSE;
        for (uint8_t i = 0; i < 16; i++)
        {
            deviceRole[i].role = (gapRole_t)0xff;
        }
    }

   private:
    /*
     * Whitelisting API related structures and helper functions.
     */

    /* Policy modes set by the user. By default these are set to ignore the whitelist */
    Gap::AdvertisingPolicyMode_t advertisingPolicyMode;
    Gap::ScanningPolicyMode_t scanningPolicyMode;
    Gap::InitiatorPolicyMode_t initiatorPolicyMode;

    /* Internal representation of a whitelist */
    uint8_t whitelistAddressesSize;
    bleIdentityAddress_t whitelistAddresses[CFG_WHITELIST_MAX_SIZE];

    /*
     * An internal function used to populate the ble_gap_whitelist_t that will be used by
     * the SoftDevice for filtering requests. This function is needed because for the BLE
     * API the whitelist is just a collection of keys, but for the stack it also includes
     * the IRK table.
     */
    // ble_error_t generateStackWhitelist(ble_gap_whitelist_t &whitelist);

   private:
    gapServiceHandles_t gapHandles;
    deviceRole_t deviceRole[16]; /* Supporting upto 16 simultaneous connections */
    bool_t connectInitiated;
    bleDeviceAddress_t peerAddress;
    gapAdvertisingParameters_t advParams;
    advState_t advState;
    bool_t restartAdv;
    uint32_t advTimeout;
    bool_t scanningOn;
    tm_timer_id_t advTimerId;
    tm_timer_id_t scanTimerId;
    uint32_t gapReqCompleteFlags;

    bleDeviceAddress_t blePubAddr;

    nxpGap(nxpGap const &);
    void operator=(nxpGap const &);
};

#endif // ifndef __NXP_GAP_H__

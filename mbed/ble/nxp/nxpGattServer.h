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

#ifndef __NXP_GATT_SERVER_H__
#define __NXP_GATT_SERVER_H__

#include <stddef.h>

#include "blecommon.h"
#include "mbedGap.h"
#include "GattServer.h"
#include "GattService.h"

extern "C" {
#include "ble_general.h"
#include "gatt_db_app_interface.h"
#include "gatt_server_interface.h"
#include "gap_interface.h"
#include "gatt_db_dynamic.h"
#include "mbed_gatt_database_dynamic.h"
#include "MemManager.h"
#include "gatt_client_interface.h"
#include "ApplMain.h"
};

class nxpGattServer : public GattServer
{
   public:
    typedef enum
    {
        ATTR_CHAR = 0,
        ATTR_DESC,
        ATTR_INVALID = 0xFF
    } attType_t;

    /* Service, Characteristics and Descriptors used to add a service */

    typedef struct descriptorInfo_tag
    {
        uint16_t handle;
        uint16_t maxValueLength;
        uint16_t isCCCD;
        uint16_t properties;
    } descriptorInfo_t;

    typedef struct characteristicInfo_tag
    {
        uint16_t handle;
        uint16_t maxValueLength;
        uint16_t properties;
        uint16_t nbOfDescriptors;
        descriptorInfo_t *pDescriptorInfo;
    } characteristicInfo_t;

    typedef struct serviceInfo_tag
    {
        uint16_t handle;
        uint16_t nbOfCharacteristics;
        gapSecurityRequirements_t serviceSecurityRequirements;
        characteristicInfo_t *pCharacteristicInfo;
        struct serviceInfo_tag *next;
    } serviceInfo_t;

    /* Functions that must be implemented from GattServer */
    virtual ble_error_t addService(GattService &);
    virtual ble_error_t read(GattAttribute::Handle_t attributeHandle, uint8_t buffer[], uint16_t *lengthP);
    virtual ble_error_t read(Gap::Handle_t connectionHandle,
                             GattAttribute::Handle_t attributeHandle,
                             uint8_t buffer[],
                             uint16_t *lengthP);
    virtual ble_error_t write(GattAttribute::Handle_t, const uint8_t[], uint16_t, bool localOnly = false);
    virtual ble_error_t write(
        Gap::Handle_t connectionHandle, GattAttribute::Handle_t, const uint8_t[], uint16_t, bool localOnly = false);
    virtual ble_error_t areUpdatesEnabled(const GattCharacteristic &characteristic, bool *enabledP);
    virtual ble_error_t areUpdatesEnabled(Gap::Handle_t connectionHandle,
                                          const GattCharacteristic &characteristic,
                                          bool *enabledP);
    virtual ble_error_t reset(void);
    nxpGattServer();
    void connectionEvent(deviceId_t peerDeviceId);
    void disconnectionEvent(deviceId_t peerDeviceId);
    void BleApp_GattServerCallback(deviceId_t deviceId, gattServerEvent_t *pServerEvent);

   private:
    serviceInfo_t *headServiceInfo;
    serviceInfo_t *tailServiceInfo;
    uint16_t descriptorCount;
    uint32_t connectionStatus;
    uint16_t lastIndicateHandle;

   private:
    nxpGattServer(const nxpGattServer &);
    const nxpGattServer &operator=(const nxpGattServer &);
    uint32_t calcServiceInfoSize(GattService &service);
    void registerCallbacks(void);
    void registerDeviceSecurity(void);
    attType_t getAttributeInfo(GattAttribute::Handle_t attributeHandle, void **attInfo);
};

#endif // ifndef __NXP_GATT_SERVER_H__

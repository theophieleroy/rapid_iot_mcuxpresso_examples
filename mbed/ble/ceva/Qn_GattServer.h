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

#ifndef __QN_GATT_SERVER_H__
#define __QN_GATT_SERVER_H__

#include <stddef.h>

#include "blecommon.h"
#include "mbedGap.h"
#include "GattServer.h"
#include "GattService.h"

extern "C" {
#include "rwip_config.h"
#include "rwip_task.h"
#include "prf_types.h"
#include "prf.h"
#include "attm.h"
#include "atts.h"
#include "attm_db.h"
#include "ke_task.h"
#include "ke_msg.h"
#include "gattc_task.h"
#include "ke_mem.h"
};

class QN_GattServer : public GattServer
{
public:
    
    typedef enum
    {
        ATTR_CHAR = 0,
        ATTR_DESC,
        ATTR_INVALID = 0xFF
    }attrType;
    
    typedef enum
    {
        UUID_16BIT = 0,
        UUID_32BIT,
        UUID_128BIT
    }uuidType;
    
    typedef struct
    {
        uint32_t type;
        union
        {
            uint16_t uuid16;
            uint32_t uuid32;
            uint8_t  uuid128[16];
        }uuid;
    }uuid_t;
    
    typedef struct 
    {
        uuid_t              uuid;
        uint16_t            len;
        uint16_t            lenMax;
        uint16_t            handle;
        uint8_t             descriptor_count;
        uint8_t             properties;
        uint8_t             data[1];
    }characteristic_t;
    
    typedef struct 
    {
        uuid_t              uuid;
        uint16_t            len;
        uint16_t            lenMax;
        uint16_t            handle;
        uint16_t            unused;
        uint8_t             data[1];
    }descriptor_t;
    
    struct ServiceData
    {
        //GattService          *service;
        struct ServiceData  *next;
        uint16_t            start_handle;
        uint16_t            end_handle;
        uuid_t              uuid;
        uint32_t            characteristic_count;
    };
    
    struct ServiceData *findServiceWithAttr(GattAttribute::Handle_t attributeHandle);
    characteristic_t * findCharacteristicWithAttr(GattAttribute::Handle_t attributeHandle, struct ServiceData *service);
    attrType findAttrByHandle(GattAttribute::Handle_t attributeHandle, void **attr);

    
    int gatts_write_req_ind_handler(ke_msg_id_t const msgid, struct gattc_write_req_ind *param,
                                      ke_task_id_t const dest_id, ke_task_id_t const src_id);
    
    int gatts_read_req_ind_handler(ke_msg_id_t const msgid, struct gattc_read_req_ind *param,
                                      ke_task_id_t const dest_id, ke_task_id_t const src_id);
    
    int gatts_cmp_evt_handler(ke_msg_id_t const msgid, struct gattc_cmp_evt *param,
                                      ke_task_id_t const dest_id, ke_task_id_t const src_id);
    
    static int write_handler(ke_msg_id_t const msgid, struct gattc_write_req_ind *param,
                                      ke_task_id_t const dest_id, ke_task_id_t const src_id);
    
    static int read_handler(ke_msg_id_t const msgid, struct gattc_read_req_ind *param,
                                      ke_task_id_t const dest_id, ke_task_id_t const src_id);
    
    static int complete_event_handler(ke_msg_id_t const msgid, struct gattc_cmp_evt *param,
                                      ke_task_id_t const dest_id, ke_task_id_t const src_id);
    
    struct ke_msg_handler default_msg_handler[3];
    
    struct ke_state_handler default_service_handler;

    /* Functions that must be implemented from GattServer */
    virtual ble_error_t addService(GattService &);
    virtual ble_error_t read(GattAttribute::Handle_t attributeHandle, uint8_t buffer[], uint16_t *lengthP);
    virtual ble_error_t read(Gap::Handle_t connectionHandle, GattAttribute::Handle_t attributeHandle, uint8_t buffer[], uint16_t *lengthP);
    virtual ble_error_t write(GattAttribute::Handle_t, const uint8_t[], uint16_t, bool localOnly = false);
    virtual ble_error_t write(Gap::Handle_t connectionHandle, GattAttribute::Handle_t, const uint8_t[], uint16_t, bool localOnly = false);
    virtual ble_error_t areUpdatesEnabled(const GattCharacteristic &characteristic, bool *enabledP);
    virtual ble_error_t areUpdatesEnabled(Gap::Handle_t connectionHandle, const GattCharacteristic &characteristic, bool *enabledP);
    virtual ble_error_t reset(void);
    QN_GattServer();
    void connectionEvent(uint16_t conhdl);
    void disconnectionEvent(uint16_t conhdl);
    
private:
    struct ServiceData        *headServiceList;
    struct ServiceData        *tailServiceList;
    struct prf_task_env       *task_env;
    uint32_t                   connectionStatus;


private:
    QN_GattServer(const QN_GattServer &);
    const QN_GattServer& operator=(const QN_GattServer &);
    void setData(attrType type, void *attribute, uint16_t length, uint16_t offset, const uint8_t *data_ptr);
    void getData(attrType type, void *attribute, uint16_t *length, uint8_t *data_ptr);
    bool IsCCCDEnabled(characteristic_t *attribute, uint8_t conidx);
};

#endif // ifndef __QN_GATT_SERVER_H__

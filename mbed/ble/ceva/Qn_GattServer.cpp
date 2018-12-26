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

#include "GattServer.h"
#include "Qn_Instance.h"
#include "Qn_GattServer.h"


QN_GattServer::ServiceData * QN_GattServer::findServiceWithAttr(GattAttribute::Handle_t attributeHandle)
{
    struct ServiceData  *currServiceList = headServiceList;
    
    while(currServiceList)
    {
        if((attributeHandle > currServiceList->start_handle) && (attributeHandle <= currServiceList->end_handle))
        {
            return currServiceList;
        }
        else
        {
            currServiceList = currServiceList->next; 
        }
    }
    
    return NULL;
}

QN_GattServer::characteristic_t * QN_GattServer::findCharacteristicWithAttr(GattAttribute::Handle_t attributeHandle, struct ServiceData *service)
{
    characteristic_t *curr_ptr = NULL;
    uint16_t desc_count;
    if(service)
    {
        curr_ptr = (characteristic_t *)((uint32_t)(&(service->characteristic_count)) + sizeof(service->characteristic_count));
    }
    for(uint8_t i = 0; service && (i < service->characteristic_count); i++)
    {
        if(attributeHandle == curr_ptr->handle) 
        {
            return curr_ptr;
        }
        /* Move to next characteristic */
        desc_count = curr_ptr->descriptor_count;
        curr_ptr = (characteristic_t *)(curr_ptr->data + curr_ptr->lenMax);
        for(uint8_t j = 0; j < desc_count; j++)
        {
            curr_ptr = (characteristic_t *)(curr_ptr->data + curr_ptr->lenMax);
        }
    }
    return NULL;
}

QN_GattServer::attrType QN_GattServer::findAttrByHandle(GattAttribute::Handle_t attributeHandle, void **attr)
{
    characteristic_t *curr_ptr = NULL;
    uint16_t desc_count;
    
    *attr = NULL;
    
    struct ServiceData *service = findServiceWithAttr(attributeHandle);
    if(service)
    {
        curr_ptr = (characteristic_t *)((uint32_t)(&(service->characteristic_count)) + sizeof(service->characteristic_count));
        for(uint8_t i = 0; i < service->characteristic_count; i++)
        {
            if((attributeHandle >= curr_ptr->handle) && 
                (attributeHandle <= curr_ptr->handle + curr_ptr->descriptor_count))
            {
                uint8_t index = attributeHandle - curr_ptr->handle;
                if(index == 0)
                {
                    *attr = curr_ptr;
                    return ATTR_CHAR;
                }
                else
                {
                    /* Move to the correct descriptor based on index value */
                    for(uint8_t j = 0; j < index; j++)
                    {
                        curr_ptr = (characteristic_t *)(curr_ptr->data + curr_ptr->lenMax);
                    }
                    *attr = curr_ptr;
                    return ATTR_DESC;
                }
            }
            /* Move to next characteristic */
            desc_count = curr_ptr->descriptor_count;
            curr_ptr = (characteristic_t *)(curr_ptr->data + curr_ptr->lenMax);
            for(uint8_t j = 0; j < desc_count; j++)
            {
                curr_ptr = (characteristic_t *)(curr_ptr->data + curr_ptr->lenMax);
            }
        }
    }
    
    return ATTR_INVALID;
}

void QN_GattServer::setData(attrType type, void *attribute, uint16_t length, uint16_t offset, const uint8_t *data_ptr)
{
    if(type == ATTR_CHAR)
    {
        ((characteristic_t *)attribute)->len = length + offset;
        memcpy(&(((characteristic_t *)attribute)->data[offset]), data_ptr, length);
        
    }
    else if(type == ATTR_DESC)
    {
        ((descriptor_t *)attribute)->len = length + offset;
        memcpy(&(((descriptor_t *)attribute)->data[offset]), data_ptr, length);
    }
}

void QN_GattServer::getData(attrType type, void *attribute, uint16_t *length, uint8_t *data_ptr)
{
    if(type == ATTR_CHAR)
    {
        *length = ((characteristic_t *)attribute)->len;
        memcpy(data_ptr, ((characteristic_t *)attribute)->data, *length);
        
    }
    else if(type == ATTR_DESC)
    {
        *length = ((descriptor_t *)attribute)->len;
        memcpy(data_ptr, ((descriptor_t *)attribute)->data, *length);
    }
}
bool QN_GattServer::IsCCCDEnabled(characteristic_t *attribute, uint8_t conidx)
{
    descriptor_t *curr_ptr = (descriptor_t *)(attribute->data + attribute->lenMax);
    bool enabled = false;
                
    /* Check if updates are enabled for the characteristic */
    for(uint8_t i = 0; i < attribute->descriptor_count; i++)
    {
        if(curr_ptr->uuid.uuid.uuid16 == ATT_DESC_CLIENT_CHAR_CFG)
        {
            if((curr_ptr->len == sizeof(uint16_t)) && 
                (((*((uint16_t *)curr_ptr->data) >> (conidx << 1)) & BLE_HVX_NOTIFICATION) || ((*((uint16_t *)curr_ptr->data) >> (conidx << 1)) & BLE_HVX_INDICATION)))
            {
                enabled = true;
            }
            break;
        }
        curr_ptr = (descriptor_t *)(curr_ptr->data + curr_ptr->lenMax);
    }
    return enabled;
}

int QN_GattServer::gatts_write_req_ind_handler(ke_msg_id_t const msgid, struct gattc_write_req_ind *param, ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    uint8_t conidx = KE_IDX_GET(src_id);
    struct gattc_write_cfm * cfm;
    void *attribute;
    uint16_t cccd_len;
    uint16_t cccd_val;
    attrType            type;
    uint8_t             status = PRF_ERR_INVALID_PARAM;
    
    type = findAttrByHandle(param->handle, &attribute);
    if(type == ATTR_CHAR)
    {
        if (param->offset + param->length <= ((characteristic_t *)attribute)->lenMax)
        {
            setData(type, attribute, param->length, param->offset, &param->value[0]);
            
            GattWriteCallbackParams params;
            params.connHandle = CONIDX2CONHDL(conidx);
            params.handle = param->handle;
            params.writeOp = GattWriteCallbackParams::OP_WRITE_REQ;
            params.offset = param->offset;
            params.len = param->length;
            params.data = &param->value[0];
            handleDataWrittenEvent(&params);
            
            status = ATT_ERR_NO_ERROR;
        }
        else
        {
            status = ATT_ERR_INVALID_ATTRIBUTE_VAL_LEN;
        }
    }
    else if(type == ATTR_DESC)
    {
        if (param->offset + param->length <= ((descriptor_t *)attribute)->lenMax)
        {
            switch(((descriptor_t *)attribute)->uuid.uuid.uuid16)
            {
                case ATT_DESC_CLIENT_CHAR_CFG:
                    switch(co_read16p(param->value))
                    {
                        case PRF_CLI_STOP_NTFIND:
                            /* For CCCD data update the value based on the connection index */
                            getData(type, attribute, &cccd_len, (uint8_t *)&cccd_val);
                            cccd_val = (cccd_val & ~(0x3U << (conidx << 1))) | ((param->value[0] & 0x3U) << (conidx << 1));
                            setData(type, attribute, param->length, param->offset, (const uint8_t *)&cccd_val);
                            handleEvent(GattServerEvents::GATT_EVENT_UPDATES_DISABLED, param->handle);
                            status = ATT_ERR_NO_ERROR;
                            break;
                        case PRF_CLI_START_NTF:
                        case PRF_CLI_START_IND:
                            /* For CCCD data update the value based on the connection index */
                            getData(type, attribute, &cccd_len, (uint8_t *)&cccd_val);
                            cccd_val = (cccd_val & ~(0x3 << (conidx << 1))) | ((param->value[0] & 0x3) << (conidx << 1));
                            setData(type, attribute, param->length, param->offset, (const uint8_t *)&cccd_val);
                            handleEvent(GattServerEvents::GATT_EVENT_UPDATES_ENABLED, param->handle);
                            status = ATT_ERR_NO_ERROR;
                            break;
                        default:
                            status = PRF_APP_ERROR;
                            break;
                    }
                    break;
                case ATT_DESC_SERVER_CHAR_CFG:
                    //FIXME - To Do for later
                    setData(type, attribute, param->length, param->offset, param->value);
                    status = ATT_ERR_NO_ERROR;
                    break; 
                default:
                    status = PRF_ERR_INVALID_PARAM;
                    break;
            }
        }
        else
        {
            status = ATT_ERR_INVALID_ATTRIBUTE_VAL_LEN;
        }
    }
    else
    {
        status = PRF_ERR_INVALID_PARAM;
    }
    
    //Send write response
    cfm = KE_MSG_ALLOC(GATTC_WRITE_CFM, src_id, dest_id, gattc_write_cfm);
    cfm->handle = param->handle;
    cfm->status = status;
    ke_msg_send(cfm);

    return (KE_MSG_CONSUMED);
}

int QN_GattServer::gatts_read_req_ind_handler(ke_msg_id_t const msgid, struct gattc_read_req_ind *param, ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    void *attribute;
    attrType type;
    struct gattc_read_cfm *msg;
    GattReadCallbackParams cb_param;
    uint8_t conidx = KE_IDX_GET(src_id);
    
    type = findAttrByHandle(param->handle, &attribute);
    
    if(type == ATTR_CHAR)
    {
        msg = KE_MSG_ALLOC_DYN(GATTC_READ_CFM, src_id, dest_id, gattc_read_cfm, ((characteristic_t *)attribute)->lenMax);
        getData(type, attribute, &(msg->length), msg->value);
        msg->status = ATT_ERR_NO_ERROR;
    }
    else if(type == ATTR_DESC)
    {
        msg = KE_MSG_ALLOC_DYN(GATTC_READ_CFM, src_id, dest_id, gattc_read_cfm, ((descriptor_t *)attribute)->lenMax);
        getData(type, attribute, &(msg->length), msg->value);
        if(((descriptor_t *)attribute)->uuid.uuid.uuid16 == ATT_DESC_CLIENT_CHAR_CFG)
        {
            *((uint16_t *)msg->value) = (*((uint16_t *)msg->value) >> (conidx << 1)) & 0x3;
        }
        msg->status = ATT_ERR_NO_ERROR;
    }
    else
    {
        msg->length = 0;
        msg->status = ATT_ERR_INVALID_HANDLE;
    }
    msg->handle = param->handle;
    
    cb_param.connHandle = CONIDX2CONHDL(conidx);
    cb_param.handle = param->handle;
    cb_param.offset = 0;
    cb_param.len = msg->length;
    cb_param.data = msg->value;
    handleDataReadEvent(&cb_param);
    
    ke_msg_send(msg);
    
    return (KE_MSG_CONSUMED);
}

int QN_GattServer::gatts_cmp_evt_handler(ke_msg_id_t const msgid, struct gattc_cmp_evt *param, ke_task_id_t const dest_id, ke_task_id_t const src_id)
{   
    if (param->operation == GATTC_NOTIFY)
    {
        if(param->status == GAP_ERR_NO_ERROR)
        {
            handleDataSentEvent(1);
        }
    }
    else if(param->operation == GATTC_INDICATE)
    {
        if(param->status == GAP_ERR_NO_ERROR)
        {
            handleEvent(GattServerEvents::GATT_EVENT_CONFIRMATION_RECEIVED, param->seq_num);
        }
    }

    return (KE_MSG_CONSUMED);
}

int QN_GattServer::write_handler(ke_msg_id_t const msgid, struct gattc_write_req_ind *param, ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    QN_BLEInstance& bleInstance = QN_BLEInstance::Instance(BLE::DEFAULT_INSTANCE);
    return (((QN_GattServer&)(bleInstance.getGattServer())).gatts_write_req_ind_handler(msgid, param, dest_id, src_id));
}

int QN_GattServer::read_handler(ke_msg_id_t const msgid, struct gattc_read_req_ind *param, ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    QN_BLEInstance& bleInstance = QN_BLEInstance::Instance(BLE::DEFAULT_INSTANCE);
    return (((QN_GattServer&)(bleInstance.getGattServer())).gatts_read_req_ind_handler(msgid, param, dest_id, src_id));
}

int QN_GattServer::complete_event_handler(ke_msg_id_t const msgid, struct gattc_cmp_evt *param, ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    QN_BLEInstance& bleInstance = QN_BLEInstance::Instance(BLE::DEFAULT_INSTANCE);
    return (((QN_GattServer&)(bleInstance.getGattServer())).gatts_cmp_evt_handler(msgid, param, dest_id, src_id));
}

void QN_GattServer::connectionEvent(uint16_t conhdl)
{
    struct ServiceData  *currServiceList = headServiceList;
    characteristic_t * curr_ptr = NULL;
    descriptor_t * desc_ptr = NULL;
    
    connectionStatus |= (1 << conhdl);
    
    while(currServiceList)
    {
        curr_ptr = (characteristic_t *)((uint32_t)(&(currServiceList->characteristic_count)) + sizeof(currServiceList->characteristic_count));
        for(uint8_t i = 0; i < currServiceList->characteristic_count; i++) {
            if(curr_ptr->properties & (GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_INDICATE | GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_NOTIFY)) {
                desc_ptr = (descriptor_t *)(curr_ptr->data + curr_ptr->lenMax);
                for(uint8_t j = 0; j < curr_ptr->descriptor_count; j++) {
                    if(desc_ptr->uuid.uuid.uuid16 == ATT_DESC_CLIENT_CHAR_CFG) {
                        desc_ptr->len = 2;
                        *((uint16_t *)desc_ptr->data) &= ~(0x3 << (conhdl << 1));
                        /* There is no break statement on purpose */
                    }
                    desc_ptr = (descriptor_t *)(desc_ptr->data + desc_ptr->lenMax);
                }
            }
            /* Move to next characteristic */
            curr_ptr = (characteristic_t *)desc_ptr;
        }
        currServiceList = currServiceList->next;
    }
}

void QN_GattServer::disconnectionEvent(uint16_t conhdl)
{
    /* Do same thing as we do for connection i.e. clear notification information */
    connectionEvent(conhdl);
    connectionStatus &= ~(1 << conhdl);
}

QN_GattServer::QN_GattServer() : GattServer() 
{
    headServiceList = NULL;
    tailServiceList = NULL;
    connectionStatus = 0;
    task_env = &(prf_env.prf[TASK_GATT_SERVER - TASK_GAPC -1]);
    /* Initialize profile environment */
    task_env->env              = (prf_env_t *) ke_malloc(sizeof(prf_env_t), KE_MEM_ATT_DB);
    task_env->env->app_task    = TASK_NONE;
    task_env->env->prf_task    = task_env->task | PERM(PRF_MI, DISABLE);

    /* initialize task descriptor */
    task_env->desc.idx_max           = 1;
    task_env->desc.default_handler   = &default_service_handler;

    default_msg_handler[0].id = GATTC_WRITE_REQ_IND;
    default_msg_handler[0].func = (ke_msg_func_t)(&write_handler);
    default_msg_handler[1].id = GATTC_CMP_EVT;
    default_msg_handler[1].func = (ke_msg_func_t)(&complete_event_handler);
    default_msg_handler[2].id = GATTC_READ_REQ_IND;
    default_msg_handler[2].func = (ke_msg_func_t)(&read_handler);
    default_service_handler.msg_table = default_msg_handler;
    default_service_handler.msg_cnt = 3;
}

/**************************************************************************/
/*!
    @brief  Adds a new service to the GATT table on the peripheral

    @returns    ble_error_t

    @retval     BLE_ERROR_NONE
                Everything executed properly

    @section EXAMPLE

    @code

    @endcode
*/
/**************************************************************************/
ble_error_t QN_GattServer::addService(GattService &service)
{
    ble_error_t error = BLE_ERROR_NONE;
    uint8_t index = 0;
    void *curr_ptr;
    uint8_t *extra_desc_count;
    uint8_t essential_desc;
    /* Calculate the total number of attributes to add to the database */
    uint16_t att_count = service.getCharacteristicCount()*2 + 1;
    uint16_t local_db_size = sizeof(struct ServiceData);
    extra_desc_count = (uint8_t *)ke_malloc(service.getCharacteristicCount(), KE_MEM_NON_RETENTION);
    
    for (uint8_t i = 0; i < service.getCharacteristicCount(); i++)
    {
        att_count += service.getCharacteristic(i)->getDescriptorCount();
        local_db_size += (sizeof(characteristic_t) - 1);
        local_db_size += ((service.getCharacteristic(i)->getValueAttribute().getMaxLength() + 3) & ~0x3);
        essential_desc = service.getCharacteristic(i)->getProperties() & 0xB1;
        extra_desc_count[i] = 0;
        for(uint8_t j = 0; j < service.getCharacteristic(i)->getDescriptorCount(); j++)
        {
            local_db_size += (sizeof(descriptor_t) - 1);
            local_db_size += ((service.getCharacteristic(i)->getDescriptor(j)->getMaxLength() + 3) & ~0x3);
            switch(service.getCharacteristic(i)->getDescriptor(j)->getUUID().getShortUUID())
            {
                case ATT_DESC_CHAR_EXT_PROPERTIES:
                    essential_desc &= ~GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_EXTENDED_PROPERTIES;
                    break;
                case ATT_DESC_CLIENT_CHAR_CFG:
                    essential_desc &= ~(GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_NOTIFY | GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_INDICATE);
                    break;
                case ATT_DESC_SERVER_CHAR_CFG:
                    essential_desc &= ~GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_BROADCAST;
                    break;
            }
        }
        for(uint8_t j = 0x80; essential_desc && j; j >>= 1)
        {
            if(essential_desc & j)
            {
                local_db_size += (sizeof(descriptor_t) + 3);
                essential_desc &= ~j;
                att_count++;
                extra_desc_count[i]++;
            }
        }
    }
    
    /* Add the service related info into a service list */
    if(headServiceList == NULL)
    {
        headServiceList = (struct ServiceData * )ke_malloc(local_db_size, KE_MEM_ATT_DB);
        headServiceList->next = NULL;
        tailServiceList = headServiceList;
    }
    else
    {
        tailServiceList->next = (struct ServiceData * )ke_malloc(local_db_size, KE_MEM_ATT_DB);
        tailServiceList = tailServiceList->next;
        tailServiceList->next = NULL;
    }
    
    /*Adding to attribute database varies based on the UUID length of the service */
    if(service.getUUID().shortOrLong() == UUID::UUID_TYPE_LONG)
    {
        struct attm_desc_128 *att_db;
        att_db = (struct attm_desc_128 *)ke_malloc(att_count * sizeof(attm_desc_128), KE_MEM_NON_RETENTION);
        index = 0;
        /* Service Declaration Attribute */
        att_db[index].uuid[0] = 0x0;
        att_db[index].uuid[1] = 0x28;
        att_db[index].perm = PERM(RD, ENABLE);
        att_db[index].ext_perm = (PERM_UUID_128 <<PERM_POS_SVC_UUID_LEN);
        att_db[index].max_size = 0;
        index++;
        
        /* Update the local service list database */
        memcpy(&(tailServiceList->uuid.uuid.uuid128[0]), service.getUUID().getBaseUUID(), ATT_UUID_128_LEN);
        tailServiceList->uuid.type = UUID_128BIT;
        tailServiceList->characteristic_count = service.getCharacteristicCount();
        curr_ptr = (uint8_t *)(&(tailServiceList->characteristic_count)) + sizeof(tailServiceList->characteristic_count);
        
        for(uint8_t i = 0; i < service.getCharacteristicCount(); i++)
        {
            /* Update the local service list database */
            memcpy(&(((characteristic_t *)curr_ptr)->uuid.uuid.uuid128[0]), service.getCharacteristic(i)->getValueAttribute().getUUID().getBaseUUID(), ATT_UUID_128_LEN);
            ((characteristic_t *)curr_ptr)->uuid.type = UUID_128BIT;
            ((characteristic_t *)curr_ptr)->len = service.getCharacteristic(i)->getValueAttribute().getLength();
            ((characteristic_t *)curr_ptr)->lenMax = (service.getCharacteristic(i)->getValueAttribute().getMaxLength() + 3) & ~0x03;
            ((characteristic_t *)curr_ptr)->descriptor_count = service.getCharacteristic(i)->getDescriptorCount() + extra_desc_count[i];
            ((characteristic_t *)curr_ptr)->properties = service.getCharacteristic(i)->getProperties();
            memcpy(((characteristic_t *)curr_ptr)->data, service.getCharacteristic(i)->getValueAttribute().getValuePtr(), ((characteristic_t *)curr_ptr)->len);
            curr_ptr = ((characteristic_t *)curr_ptr)->data + ((characteristic_t *)curr_ptr)->lenMax;
            
            /* Characteristic Declaration Attribute */
            att_db[index].uuid[0] = 0x03;
            att_db[index].uuid[1] = 0x28;
            att_db[index].perm = PERM(RD, ENABLE);
            att_db[index].ext_perm = 0;
            att_db[index].max_size = 0;
            index++;
            
            /* Characteristic Value Attribute */
            memcpy(&(att_db[index].uuid), service.getCharacteristic(i)->getValueAttribute().getUUID().getBaseUUID(), ATT_UUID_128_LEN);
            
            att_db[index].perm = (service.getCharacteristic(i)->getProperties() & 0xff) << 8;
            att_db[index].ext_perm = PERM(RI, ENABLE)|(PERM_UUID_128 <<PERM_POS_UUID_LEN);
            att_db[index].max_size = service.getCharacteristic(i)->getValueAttribute().getMaxLength();
            index++;
            
            essential_desc = service.getCharacteristic(i)->getProperties() & 0xB1;
            /* Characteristic Descriptor Attributes */
            for(uint8_t j = 0; j < service.getCharacteristic(i)->getDescriptorCount(); j++)
            {
                /* Update the local service list database */
                memcpy(&(((descriptor_t *)curr_ptr)->uuid.uuid.uuid16), service.getCharacteristic(i)->getDescriptor(j)->getUUID().getBaseUUID(), ATT_UUID_16_LEN);
                ((descriptor_t *)curr_ptr)->uuid.type = UUID_16BIT;
                ((descriptor_t *)curr_ptr)->len = service.getCharacteristic(i)->getDescriptor(j)->getLength();
                ((descriptor_t *)curr_ptr)->lenMax = (service.getCharacteristic(i)->getDescriptor(j)->getMaxLength() + 3) & ~0x3;
                memcpy(((descriptor_t *)curr_ptr)->data, service.getCharacteristic(i)->getDescriptor(j)->getValuePtr(), ((descriptor_t *)curr_ptr)->len);
                curr_ptr = ((descriptor_t *)curr_ptr)->data + ((descriptor_t *)curr_ptr)->lenMax;
                    
                memcpy(&(att_db[index].uuid), service.getCharacteristic(i)->getDescriptor(j)->getUUID().getBaseUUID(), ATT_UUID_16_LEN);
                //FIXME - Need to add all descriptors
                switch(service.getCharacteristic(i)->getDescriptor(j)->getUUID().getShortUUID())
                {
                    case ATT_DESC_CHAR_EXT_PROPERTIES:
                        essential_desc &= ~GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_EXTENDED_PROPERTIES;
                    case ATT_DESC_CHAR_USER_DESCRIPTION:
                    case ATT_DESC_CHAR_PRES_FORMAT:
                        att_db[index].perm = PERM(RD, ENABLE);
                        att_db[index].ext_perm = PERM(RI, ENABLE); //FIXME - How to extract this permission based on the descriptor type?
                        break;
                    case ATT_DESC_CLIENT_CHAR_CFG:
                        essential_desc &= ~(GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_NOTIFY | GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_INDICATE);
                        att_db[index].perm = PERM(WRITE_REQ, ENABLE) | PERM(RD, ENABLE);
                        att_db[index].ext_perm = PERM(RI, ENABLE);
                        break;
                    case ATT_DESC_SERVER_CHAR_CFG:
                        essential_desc &= ~GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_BROADCAST;
                        att_db[index].perm = PERM(WRITE_REQ, ENABLE) | PERM(RD, ENABLE);
                        att_db[index].ext_perm = PERM(RI, ENABLE);
                        break;
                }
                att_db[index].max_size = service.getCharacteristic(i)->getDescriptor(j)->getMaxLength();
                index++;
            }
            /* Add mandatory descriptors that were not provided in the GattService */
            for(uint8_t j = 0x80; essential_desc && j; j >>= 1)
            {
                switch(essential_desc & j)
                {
                    case GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_BROADCAST:
                        ((descriptor_t *)curr_ptr)->uuid.uuid.uuid16 = *(uint16_t *)(att_db[index].uuid) = ATT_DESC_SERVER_CHAR_CFG;
                        ((descriptor_t *)curr_ptr)->uuid.type = UUID_16BIT;
                        ((descriptor_t *)curr_ptr)->len = 2;
                        ((descriptor_t *)curr_ptr)->lenMax = 4;
                        ((descriptor_t *)curr_ptr)->data[0] = 0;
                        ((descriptor_t *)curr_ptr)->data[1] = 0;
                        curr_ptr = ((descriptor_t *)curr_ptr)->data + ((descriptor_t *)curr_ptr)->lenMax;
                        essential_desc &= ~GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_BROADCAST;
                        att_db[index].perm = PERM(WRITE_REQ, ENABLE) | PERM(RD, ENABLE);
                        att_db[index].ext_perm = PERM(RI, ENABLE);
                        att_db[index].max_size = 2;
                        index++;
                        break;
                    case GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_NOTIFY:
                    case GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_INDICATE:
                        ((descriptor_t *)curr_ptr)->uuid.uuid.uuid16 = *(uint16_t *)(att_db[index].uuid) = ATT_DESC_CLIENT_CHAR_CFG;
                        ((descriptor_t *)curr_ptr)->uuid.type = UUID_16BIT;
                        ((descriptor_t *)curr_ptr)->len = 2;
                        ((descriptor_t *)curr_ptr)->lenMax = 4;
                        ((descriptor_t *)curr_ptr)->data[0] = 0;
                        ((descriptor_t *)curr_ptr)->data[1] = 0;
                        curr_ptr = ((descriptor_t *)curr_ptr)->data + ((descriptor_t *)curr_ptr)->lenMax;
                        essential_desc &= ~(GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_NOTIFY | GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_INDICATE);
                        att_db[index].perm = PERM(WRITE_REQ, ENABLE) | PERM(RD, ENABLE);
                        att_db[index].ext_perm = PERM(RI, ENABLE);
                        att_db[index].max_size = 2;
                        index++;
                        break;
                    case GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_EXTENDED_PROPERTIES:
                        ((descriptor_t *)curr_ptr)->uuid.uuid.uuid16 = *(uint16_t *)(att_db[index].uuid) = ATT_DESC_CHAR_EXT_PROPERTIES;
                        ((descriptor_t *)curr_ptr)->uuid.type = UUID_16BIT;
                        ((descriptor_t *)curr_ptr)->len = 2;
                        ((descriptor_t *)curr_ptr)->lenMax = 4;
                        ((descriptor_t *)curr_ptr)->data[0] = 0;
                        ((descriptor_t *)curr_ptr)->data[1] = 0;
                        curr_ptr = ((descriptor_t *)curr_ptr)->data + ((descriptor_t *)curr_ptr)->lenMax;
                        essential_desc &= ~GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_EXTENDED_PROPERTIES;
                        att_db[index].perm = PERM(RD, ENABLE);
                        att_db[index].ext_perm = PERM(RI, ENABLE);
                        att_db[index].max_size = 2;
                        index++;
                        break;
                    default:
                        break;
                }
            }
            
        }
        characteristicCount += service.getCharacteristicCount();
        
        /* Now add to database */
        /* Service configuration flag */
        uint32_t cfg_flag = (1UL << att_count) - 1;
        /* DB Creation Status */
        uint8_t status = ATT_ERR_NO_ERROR;
        /* Start Handle for service */
        uint16_t start_hdl = 0;

        /* Add to database */
        status = attm_svc_create_db_128(&start_hdl, service.getUUID().getBaseUUID(), (uint8_t *)&cfg_flag,
                 att_count, NULL, task_env->task, &att_db[0], PERM_UUID_128 << PERM_POS_SVC_UUID_LEN);

        if (status == ATT_ERR_NO_ERROR)
        {
            service.setHandle(start_hdl);
            tailServiceList->start_handle = start_hdl;
            tailServiceList->end_handle = start_hdl + att_count - 1;
            curr_ptr = (uint8_t *)(&(tailServiceList->characteristic_count)) + sizeof(tailServiceList->characteristic_count);
            
            /* Set Attribute Handles */
            for(uint8_t i = 0, offset = 2; i < service.getCharacteristicCount(); i++)
            {
                service.getCharacteristic(i)->getValueAttribute().setHandle(start_hdl + offset);
                ((characteristic_t *)curr_ptr)->handle = start_hdl + offset++;
                curr_ptr = ((characteristic_t *)curr_ptr)->data + ((characteristic_t *)curr_ptr)->lenMax;
                
                for(uint8_t j = 0; j < service.getCharacteristic(i)->getDescriptorCount(); j++)
                {
                    service.getCharacteristic(i)->getDescriptor(j)->setHandle(start_hdl + offset);
                    ((descriptor_t *)curr_ptr)->handle = start_hdl + offset++;
                    curr_ptr = ((descriptor_t *)curr_ptr)->data + ((descriptor_t *)curr_ptr)->lenMax;
                }
                for(uint8_t j = 0; j < extra_desc_count[i]; j++)
                {
                    ((descriptor_t *)curr_ptr)->handle = start_hdl + offset++;
                    curr_ptr = ((descriptor_t *)curr_ptr)->data + ((descriptor_t *)curr_ptr)->lenMax;
                }
                offset++;
                
            }
        }
        else
        {
            //FIXME - Need to map error codes correctly
            error = (ble_error_t) status;
        }
        
        ke_free(att_db);
    }
    /* Service with 16 bit UUID */
    else 
    {
        struct attm_desc *att_db;
        att_db = (struct attm_desc *)ke_malloc(att_count * sizeof(attm_desc), KE_MEM_NON_RETENTION);
        index = 0;
        /* Service Declaration Attribute */
        att_db[index].uuid = 0x2800;
        att_db[index].perm = PERM(RD, ENABLE);
        att_db[index].ext_perm = 0;
        att_db[index].max_size = 0;
        index++;
        
        /* Update the local service list database */
        memcpy(&(tailServiceList->uuid.uuid.uuid16), service.getUUID().getBaseUUID(), ATT_UUID_16_LEN);
        tailServiceList->uuid.type = UUID_16BIT;
        tailServiceList->characteristic_count = service.getCharacteristicCount();
        curr_ptr = (uint8_t *)(&(tailServiceList->characteristic_count)) + sizeof(tailServiceList->characteristic_count);
        
        for(uint8_t i = 0; i < service.getCharacteristicCount(); i++)
        {
            /* Update the local service list database */
            memcpy(&(((characteristic_t *)curr_ptr)->uuid.uuid.uuid16), service.getCharacteristic(i)->getValueAttribute().getUUID().getBaseUUID(), ATT_UUID_16_LEN);
            ((characteristic_t *)curr_ptr)->uuid.type = UUID_16BIT;
            ((characteristic_t *)curr_ptr)->len = service.getCharacteristic(i)->getValueAttribute().getLength();
            ((characteristic_t *)curr_ptr)->lenMax = (service.getCharacteristic(i)->getValueAttribute().getMaxLength() + 3) & ~0x3;
            ((characteristic_t *)curr_ptr)->descriptor_count = service.getCharacteristic(i)->getDescriptorCount() + extra_desc_count[i];
            ((characteristic_t *)curr_ptr)->properties = service.getCharacteristic(i)->getProperties();
            memcpy(((characteristic_t *)curr_ptr)->data, service.getCharacteristic(i)->getValueAttribute().getValuePtr(), ((characteristic_t *)curr_ptr)->lenMax);
            curr_ptr = ((characteristic_t *)curr_ptr)->data + ((characteristic_t *)curr_ptr)->lenMax;
            
            /* Characteristic Declaration Attribute */
            att_db[index].uuid = 0x2803;
            att_db[index].perm = PERM(RD, ENABLE);
            att_db[index].ext_perm = 0;
            att_db[index].max_size = 0;
            index++;
            
            /* Characteristic Value Attribute */
            memcpy(&(att_db[index].uuid), service.getCharacteristic(i)->getValueAttribute().getUUID().getBaseUUID(), ATT_UUID_16_LEN);
            att_db[index].perm = (service.getCharacteristic(i)->getProperties() & 0xff) << 8;
            att_db[index].ext_perm = PERM(RI, ENABLE);
            att_db[index].max_size = service.getCharacteristic(i)->getValueAttribute().getMaxLength();
            index++;
            
            essential_desc = service.getCharacteristic(i)->getProperties() & 0xB1;
            /* Characteristic Descriptors Attribute */
            for(uint8_t j = 0; j < service.getCharacteristic(i)->getDescriptorCount(); j++)
            {
                /* Update the local service list database */
                memcpy(&(((descriptor_t *)curr_ptr)->uuid.uuid.uuid16), service.getCharacteristic(i)->getDescriptor(j)->getUUID().getBaseUUID(), ATT_UUID_16_LEN);
                ((descriptor_t *)curr_ptr)->uuid.type = UUID_16BIT;
                ((descriptor_t *)curr_ptr)->len = service.getCharacteristic(i)->getDescriptor(j)->getLength();
                ((descriptor_t *)curr_ptr)->lenMax = (service.getCharacteristic(i)->getDescriptor(j)->getMaxLength() + 3) & ~0x3;
                memcpy(((descriptor_t *)curr_ptr)->data, service.getCharacteristic(i)->getDescriptor(j)->getValuePtr(), ((descriptor_t *)curr_ptr)->lenMax);
                curr_ptr = ((descriptor_t *)curr_ptr)->data + ((descriptor_t *)curr_ptr)->lenMax;
                
                memcpy(&(att_db[index].uuid), service.getCharacteristic(i)->getDescriptor(j)->getUUID().getBaseUUID(), ATT_UUID_16_LEN);
                //FIXME - Need to add all descriptors
                switch(service.getCharacteristic(i)->getDescriptor(j)->getUUID().getShortUUID())
                {
                    case ATT_DESC_CHAR_EXT_PROPERTIES:
                        essential_desc &= ~GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_EXTENDED_PROPERTIES;
                    case ATT_DESC_CHAR_USER_DESCRIPTION:
                    case ATT_DESC_CHAR_PRES_FORMAT:
                        att_db[index].perm = PERM(RD, ENABLE);
                        att_db[index].ext_perm = PERM(RI, ENABLE); //FIXME - How to extract this permission based on the descriptor type?
                        break;
                    case ATT_DESC_CLIENT_CHAR_CFG:
                        essential_desc &= ~(GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_NOTIFY | GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_INDICATE);
                        att_db[index].perm = PERM(WRITE_REQ, ENABLE) | PERM(RD, ENABLE);
                        att_db[index].ext_perm = PERM(RI, ENABLE);
                        break;
                    case ATT_DESC_SERVER_CHAR_CFG:
                        essential_desc &= ~GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_BROADCAST;
                        att_db[index].perm = PERM(WRITE_REQ, ENABLE) | PERM(RD, ENABLE);
                        att_db[index].ext_perm = PERM(RI, ENABLE);
                        break;
                }
                att_db[index].max_size = service.getCharacteristic(i)->getDescriptor(j)->getMaxLength();
                index++;
            }
            /* Add mandatory descriptors that were not provided in the GattService */
            for(uint8_t j = 0x80; essential_desc && j; j >>= 1)
            {
                switch(essential_desc & j)
                {
                    case GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_BROADCAST:
                        ((descriptor_t *)curr_ptr)->uuid.uuid.uuid16 = att_db[index].uuid = ATT_DESC_SERVER_CHAR_CFG;
                        ((descriptor_t *)curr_ptr)->uuid.type = UUID_16BIT;
                        ((descriptor_t *)curr_ptr)->len = 2;
                        ((descriptor_t *)curr_ptr)->lenMax = 4;
                        ((descriptor_t *)curr_ptr)->data[0] = 0;
                        ((descriptor_t *)curr_ptr)->data[1] = 0;
                        curr_ptr = ((descriptor_t *)curr_ptr)->data + ((descriptor_t *)curr_ptr)->lenMax;
                        essential_desc &= ~GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_BROADCAST;
                        att_db[index].perm = PERM(WRITE_REQ, ENABLE) | PERM(RD, ENABLE);
                        att_db[index].ext_perm = PERM(RI, ENABLE);
                        att_db[index].max_size = 2;
                        index++;
                        break;
                    case GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_NOTIFY:
                    case GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_INDICATE:
                        ((descriptor_t *)curr_ptr)->uuid.uuid.uuid16 = att_db[index].uuid = ATT_DESC_CLIENT_CHAR_CFG;
                        ((descriptor_t *)curr_ptr)->uuid.type = UUID_16BIT;
                        ((descriptor_t *)curr_ptr)->len = 2;
                        ((descriptor_t *)curr_ptr)->lenMax = 4;
                        ((descriptor_t *)curr_ptr)->data[0] = 0;
                        ((descriptor_t *)curr_ptr)->data[1] = 0;
                        curr_ptr = ((descriptor_t *)curr_ptr)->data + ((descriptor_t *)curr_ptr)->lenMax;
                        essential_desc &= ~(GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_NOTIFY | GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_INDICATE);
                        att_db[index].perm = PERM(WRITE_REQ, ENABLE) | PERM(RD, ENABLE);
                        att_db[index].ext_perm = PERM(RI, ENABLE);
                        att_db[index].max_size = 2;
                        index++;
                        break;
                    case GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_EXTENDED_PROPERTIES:
                        ((descriptor_t *)curr_ptr)->uuid.uuid.uuid16 = *(uint16_t *)(att_db[index].uuid) = ATT_DESC_CHAR_EXT_PROPERTIES;
                        ((descriptor_t *)curr_ptr)->uuid.type = UUID_16BIT;
                        ((descriptor_t *)curr_ptr)->len = 2;
                        ((descriptor_t *)curr_ptr)->lenMax = 4;
                        ((descriptor_t *)curr_ptr)->data[0] = 0;
                        ((descriptor_t *)curr_ptr)->data[1] = 0;
                        curr_ptr = ((descriptor_t *)curr_ptr)->data + ((descriptor_t *)curr_ptr)->lenMax;
                        essential_desc &= ~GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_EXTENDED_PROPERTIES;
                        att_db[index].perm = PERM(RD, ENABLE);
                        att_db[index].ext_perm = PERM(RI, ENABLE);
                        att_db[index].max_size = 2;
                        index++;
                        break;
                    default:
                        break;
                }
            }
            
        }
        characteristicCount += service.getCharacteristicCount();
        
         /* Now add to database */
        /* Service configuration flag */
        uint32_t cfg_flag = (1UL << att_count) - 1;
        /* DB Creation Status */
        uint8_t status = ATT_ERR_NO_ERROR;
        /* Start Handle for service */
        uint16_t start_hdl = 0;

        /* Add to database */
        status = attm_svc_create_db(&start_hdl, service.getUUID().getShortUUID(), (uint8_t *)&cfg_flag,
                 att_count, NULL, task_env->task, &att_db[0], 0);

        if (status == ATT_ERR_NO_ERROR)
        {
            service.setHandle(start_hdl);
            tailServiceList->start_handle = start_hdl;
            tailServiceList->end_handle = start_hdl + att_count - 1;
            curr_ptr = (uint8_t *)(&(tailServiceList->characteristic_count)) + sizeof(tailServiceList->characteristic_count);
            
            /* Set Attribute Handles */
            for(uint8_t i = 0, offset = 2; i < service.getCharacteristicCount(); i++)
            {
                service.getCharacteristic(i)->getValueAttribute().setHandle(start_hdl + offset);
                ((characteristic_t *)curr_ptr)->handle = start_hdl + offset++;
                curr_ptr = ((characteristic_t *)curr_ptr)->data + ((characteristic_t *)curr_ptr)->lenMax;
                
                for(uint8_t j = 0; j < service.getCharacteristic(i)->getDescriptorCount(); j++)
                {
                    service.getCharacteristic(i)->getDescriptor(j)->setHandle(start_hdl + offset);
                    ((descriptor_t *)curr_ptr)->handle = start_hdl + offset++;
                    curr_ptr = ((descriptor_t *)curr_ptr)->data + ((descriptor_t *)curr_ptr)->lenMax;
                }
                for(uint8_t j = 0; j < extra_desc_count[i]; j++)
                {
                    ((descriptor_t *)curr_ptr)->handle = start_hdl + offset++;
                    curr_ptr = ((descriptor_t *)curr_ptr)->data + ((descriptor_t *)curr_ptr)->lenMax;
                }
                offset++;
                
            }
        }
        else
        {
            //FIXME - Need to map error codes correctly
            error = (ble_error_t) status;
        }
        
        ke_free(att_db);
    }
    
    ke_free(extra_desc_count);
        
    serviceCount++;

    return error;
}

/**************************************************************************/
/*!
    @brief  Reads the value of a characteristic, based on the service
            and characteristic index fields

    @param[in]  attributeHandle
                The handle of the GattCharacteristic to read from
    @param[in]  buffer
                Buffer to hold the the characteristic's value
                (raw byte array in LSB format)
    @param[in/out] len
                input:  Length in bytes to be read.
                output: Total length of attribute value upon successful return.

    @returns    ble_error_t

    @retval     BLE_ERROR_NONE
                Everything executed properly
*/
/**************************************************************************/
ble_error_t QN_GattServer::read(GattAttribute::Handle_t attributeHandle, uint8_t buffer[], uint16_t *lengthP)
{
    return read(0, attributeHandle, buffer, lengthP);
}

ble_error_t QN_GattServer::read(Gap::Handle_t connectionHandle, GattAttribute::Handle_t attributeHandle, uint8_t buffer[], uint16_t *lengthP)
{
    void *attribute;   
    attrType type;
    
    type = findAttrByHandle(attributeHandle, &attribute);
    
    if(attribute)
    {
        getData(type, attribute, lengthP, buffer);
        if((type == ATTR_DESC) && (((descriptor_t *)attribute)->uuid.uuid.uuid16 == ATT_DESC_CLIENT_CHAR_CFG))
        {
            *((uint16_t *)buffer) = (*((uint16_t *)buffer) >> (connectionHandle << 1)) & 0x3;
        }
        return BLE_ERROR_NONE;
    }
    else
    {
        return BLE_ERROR_INVALID_PARAM;
    }
    
}

/**************************************************************************/
/*!
    @brief  Updates the value of a characteristic, based on the service
            and characteristic index fields

    @param[in]  charHandle
                The handle of the GattCharacteristic to write to
    @param[in]  buffer
                Data to use when updating the characteristic's value
                (raw byte array in LSB format)
    @param[in]  len
                The number of bytes in buffer

    @returns    ble_error_t

    @retval     BLE_ERROR_NONE
                Everything executed properly
*/
/**************************************************************************/
ble_error_t QN_GattServer::write(GattAttribute::Handle_t attributeHandle, const uint8_t buffer[], uint16_t len, bool localOnly)
{
    ble_error_t error = BLE_ERROR_NONE;
    for(uint8_t i = 0; (i < BLE_CONNECTION_MAX) && ((error == BLE_ERROR_NONE) || (error == BLE_ERROR_OPERATION_NOT_PERMITTED)) ; i++)
    {
        if(connectionStatus & (1 << i))
        {
            error = write(i, attributeHandle, buffer, len, localOnly);
        }
    }
    return error;
}

ble_error_t QN_GattServer::write(Gap::Handle_t connectionHandle, GattAttribute::Handle_t attributeHandle, const uint8_t buffer[], uint16_t len, bool localOnly)
{
    ble_error_t returnValue = BLE_ERROR_NONE;
    void *attribute;   
    attrType type;
    
    type = findAttrByHandle(attributeHandle, &attribute);
    
    if(type == ATTR_CHAR)
    {
        if(len <= ((characteristic_t *)attribute)->lenMax)
        {
            setData(type, attribute, len, 0, buffer);
        }
        else
        {
            returnValue = BLE_ERROR_PARAM_OUT_OF_RANGE;
        }
    }
    else if(type == ATTR_DESC)
    {
        if(len <= ((descriptor_t *)attribute)->lenMax)
        {
            setData(type, attribute, len, 0, buffer);
        }
        else
        {
            returnValue = BLE_ERROR_PARAM_OUT_OF_RANGE;
        }
    }
    else
    {
        returnValue = BLE_ERROR_INVALID_PARAM;
    }
    
    if ((returnValue == BLE_ERROR_NONE) && (!localOnly))
    {
        if(type == ATTR_CHAR)
        {
            uint8_t conidx = CONHDL2CONIDX(connectionHandle);
            
            if(IsCCCDEnabled((characteristic_t *)attribute, conidx))
            {
                struct gattc_send_evt_cmd *msg = KE_MSG_ALLOC_DYN(GATTC_SEND_EVT_CMD,
                                                                            KE_BUILD_ID(TASK_GATTC, conidx),
                                                                            KE_BUILD_ID(TASK_GATT_SERVER, conidx),
                                                                            gattc_send_evt_cmd,
                                                                            len);
                if(((characteristic_t *)attribute)->properties & GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_INDICATE)
                {
                    msg->operation = GATTC_INDICATE;
                }
                else
                {
                    msg->operation = GATTC_NOTIFY;
                }
                msg->handle = attributeHandle;
                //FIXME - A hack for now to send handle in seq number
                msg->seq_num = attributeHandle;
                msg->length = len;
                memcpy(&msg->value[0], buffer, len);

                ke_msg_send(msg);
                returnValue = BLE_ERROR_NONE;
            }
            else 
            {
                handleDataSentEvent(0);
                returnValue = BLE_ERROR_OPERATION_NOT_PERMITTED;
            }
        }
        else
        {
            /* Descriptors are notified */
            returnValue = BLE_ERROR_INVALID_PARAM;
        }
    }
    
    return returnValue;
}

ble_error_t QN_GattServer::areUpdatesEnabled(const GattCharacteristic &characteristic, bool *enabledP)
{
    /* Forward the call with the default connection handle. */
    return areUpdatesEnabled(0, characteristic, enabledP);
}

ble_error_t QN_GattServer::areUpdatesEnabled(Gap::Handle_t connectionHandle, const GattCharacteristic &characteristic, bool *enabledP)
{    
    characteristic_t *attribute = findCharacteristicWithAttr(characteristic.getValueHandle(), findServiceWithAttr(characteristic.getValueHandle()));
    *enabledP = false;
    if(attribute)
    {
        *enabledP = IsCCCDEnabled(attribute, (uint8_t)connectionHandle);
        return BLE_ERROR_NONE;
    }
    return BLE_ERROR_INVALID_PARAM;
}

/**************************************************************************/
/*!
    @brief  Clear QN_GattServer state.

    @returns    ble_error_t

    @retval     BLE_ERROR_NONE
                Everything executed properly
*/
/**************************************************************************/
ble_error_t QN_GattServer::reset(void)
{
    struct ServiceData        *temp;
    /* Clear all state that is from the parent, including private members */
    if (GattServer::reset() != BLE_ERROR_NONE) {
        return BLE_ERROR_INVALID_STATE;
    }

    /* Clear derived class members */
    while(headServiceList)
    {
        temp = headServiceList;
        //ke_free(headServiceList->task_env->env);
        headServiceList = headServiceList->next;
        ke_free(temp);
    }
    tailServiceList = headServiceList;
    connectionStatus = 0;

    return BLE_ERROR_NONE;
}

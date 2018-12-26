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
#include "nxpInstance.h"
#include "nxpGattServer.h"
#include "nxpSecurityManager.h"
#define ESSENTIAL_DESC_MASK                                                                                        \
    (GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_EXTENDED_PROPERTIES |                                            \
     GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_INDICATE | GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_NOTIFY | \
     GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_BROADCAST)

extern gapPairingParameters_t gPeripheralPairingParameters;

void mbedBleApp_GattServerCallback(deviceId_t deviceId, gattServerEvent_t *pServerEvent)
{
    nxpInstance &bleInstance = nxpInstance::Instance(BLE::DEFAULT_INSTANCE);
    (((nxpGattServer &)(bleInstance.getGattServer())).BleApp_GattServerCallback(deviceId, pServerEvent));
}
void nxpGattServer::BleApp_GattServerCallback(deviceId_t deviceId, gattServerEvent_t *pServerEvent)
{
    switch (pServerEvent->eventType)
    {
        case gEvtAttributeWritten_c:
        {
            GattWriteCallbackParams params;
            params.connHandle = deviceId;
            params.handle = pServerEvent->eventData.attributeWrittenEvent.handle;
            params.writeOp = GattWriteCallbackParams::OP_WRITE_REQ;
            params.offset = 0;
            params.len = pServerEvent->eventData.attributeWrittenEvent.cValueLength;
            params.data = pServerEvent->eventData.attributeWrittenEvent.aValue;
            handleDataWrittenEvent(&params);
            GattDb_WriteAttribute(params.handle, params.len, (uint8_t *)params.data);
            GattServer_SendAttributeWrittenStatus(deviceId, params.handle, gAttErrCodeNoError_c);
        }
        break;
        case gEvtAttributeWrittenWithoutResponse_c:
        {
            GattWriteCallbackParams params;
            params.connHandle = deviceId;
            params.handle = pServerEvent->eventData.attributeWrittenEvent.handle;
            params.writeOp = GattWriteCallbackParams::OP_WRITE_CMD;
            params.offset = 0;
            params.len = pServerEvent->eventData.attributeWrittenEvent.cValueLength;
            params.data = pServerEvent->eventData.attributeWrittenEvent.aValue;
            handleDataWrittenEvent(&params);
        }
        break;
        case gEvtAttributeRead_c:
        {
            GattReadCallbackParams params;
            params.connHandle = deviceId;
            params.handle = pServerEvent->eventData.attributeReadEvent.handle;
            params.offset = 0;
            params.len = 0;
            params.data = 0;
            handleDataReadEvent(&params);
            GattServer_SendAttributeReadStatus(deviceId, params.handle, gAttErrCodeNoError_c);
        }
        break;
        case gEvtHandleValueConfirmation_c:
        {
            handleEvent(GattServerEvents::GATT_EVENT_CONFIRMATION_RECEIVED, lastIndicateHandle);
        }
        break;
        case gEvtCharacteristicCccdWritten_c:
        {
            switch (pServerEvent->eventData.charCccdWrittenEvent.newCccd)
            {
                case gCccdNotification_c:
                case gCccdIndication_c:
                    handleEvent(GattServerEvents::GATT_EVENT_UPDATES_ENABLED,
                                pServerEvent->eventData.charCccdWrittenEvent.handle);
                    break;
                case gCccdEmpty_c:
                    handleEvent(GattServerEvents::GATT_EVENT_UPDATES_DISABLED,
                                pServerEvent->eventData.charCccdWrittenEvent.handle);
                    break;
                default:
                    break;
            }
        }
        break;
        default:
            break;
    }
}

void nxpGattServer::connectionEvent(deviceId_t peerDeviceId)
{
    serviceInfo_t *currServiceInfo = headServiceInfo;

    connectionStatus |= (1 << peerDeviceId);

    while (currServiceInfo)
    {
        for (uint8_t i = 0; i < currServiceInfo->nbOfCharacteristics; i++)
        {
            for (uint8_t j = 0; j < currServiceInfo->pCharacteristicInfo[i].nbOfDescriptors; j++)
            {
                if (currServiceInfo->pCharacteristicInfo[i].pDescriptorInfo[j].isCCCD)
                {
                    Gap_SaveCccd(peerDeviceId, currServiceInfo->pCharacteristicInfo[i].pDescriptorInfo[j].handle,
                                 gCccdEmpty_c);
                }
            }
        }
        currServiceInfo = currServiceInfo->next;
    }
}

void nxpGattServer::disconnectionEvent(deviceId_t peerDeviceId)
{
    /* Do same thing as we do for connection i.e. clear notification information */
    connectionEvent(peerDeviceId);
    connectionStatus &= ~(1 << peerDeviceId);
}

nxpGattServer::nxpGattServer() : GattServer()
{
    descriptorCount = 0;
    connectionStatus = 0;
    lastIndicateHandle = 0;
    headServiceInfo = NULL;
    tailServiceInfo = NULL;
}

nxpGattServer::attType_t nxpGattServer::getAttributeInfo(GattAttribute::Handle_t attributeHandle, void **attInfo)
{
    serviceInfo_t *currServiceInfo = headServiceInfo;
    *attInfo = NULL;
    while (currServiceInfo)
    {
        if ((attributeHandle >= currServiceInfo->handle) &&
            (attributeHandle < (currServiceInfo->next ? currServiceInfo->next->handle : 0xFFFF)))
        {
            for (uint8_t i = 0; i < currServiceInfo->nbOfCharacteristics; i++)
            {
                if ((attributeHandle >= currServiceInfo->pCharacteristicInfo[i].handle) &&
                    (attributeHandle < ((i == (currServiceInfo->nbOfCharacteristics - 1)) ?
                                            0xFFFF :
                                            currServiceInfo->pCharacteristicInfo[i + 1].handle)))
                {
                    if (attributeHandle == currServiceInfo->pCharacteristicInfo[i].handle)
                    {
                        *attInfo = &(currServiceInfo->pCharacteristicInfo[i]);
                        return ATTR_CHAR;
                    }

                    for (uint8_t j = 0; j < currServiceInfo->pCharacteristicInfo[i].nbOfDescriptors; j++)
                    {
                        if (attributeHandle == currServiceInfo->pCharacteristicInfo[i].pDescriptorInfo[j].handle)
                        {
                            *attInfo = &(currServiceInfo->pCharacteristicInfo[i].pDescriptorInfo[j]);
                            return ATTR_DESC;
                        }
                    }
                    break;
                }
            }
            break;
        }
        currServiceInfo = currServiceInfo->next;
    }
    return ATTR_INVALID;
}

uint32_t nxpGattServer::calcServiceInfoSize(GattService &service)
{
    uint32_t nodeSize = sizeof(serviceInfo_t);
    uint8_t essential_desc;

    for (uint8_t i = 0; i < service.getCharacteristicCount(); i++)
    {
        nodeSize += sizeof(characteristicInfo_t);
        essential_desc = service.getCharacteristic(i)->getProperties() & ESSENTIAL_DESC_MASK;
        for (uint8_t j = 0; j < service.getCharacteristic(i)->getDescriptorCount(); j++)
        {
            nodeSize += sizeof(descriptorInfo_t);
            switch (service.getCharacteristic(i)->getDescriptor(j)->getUUID().getShortUUID())
            {
                case BLE_UUID_DESCRIPTOR_CHAR_EXT_PROP:
                    essential_desc &= ~GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_EXTENDED_PROPERTIES;
                    break;
                case BLE_UUID_DESCRIPTOR_CLIENT_CHAR_CONFIG:
                    essential_desc &= ~(GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_NOTIFY |
                                        GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_INDICATE);
                    break;
                case BLE_UUID_DESCRIPTOR_SERVER_CHAR_CONFIG:
                    essential_desc &= ~GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_BROADCAST;
                    break;
            }
        }
        for (uint8_t j = 0x80; essential_desc && j; j >>= 1)
        {
            if (essential_desc & j)
            {
                nodeSize += sizeof(descriptorInfo_t);
                essential_desc &= ~j;
            }
        }
    }
    return nodeSize;
}

/* Register device security settings */
void nxpGattServer::registerDeviceSecurity(void)
{
    BLE &ble = BLE::Instance(BLE::DEFAULT_INSTANCE);
    serviceInfo_t *currServiceInfo = headServiceInfo;

    gapDeviceSecurityRequirements_t deviceSecurityRequirements;

    deviceSecurityRequirements.aServiceSecurityRequirements = NULL;
    deviceSecurityRequirements.cNumServices = 0;
    deviceSecurityRequirements.pMasterSecurityRequirements =
        ((nxpSecurityManager &)(ble.securityManager())).getMasterSecurityReq();

    while (currServiceInfo)
    { /* Set the master service requirements to the maximum security level among all services */
        if (currServiceInfo->serviceSecurityRequirements.securityModeLevel >
            deviceSecurityRequirements.pMasterSecurityRequirements->securityModeLevel)
        {
            deviceSecurityRequirements.pMasterSecurityRequirements->securityModeLevel =
                currServiceInfo->serviceSecurityRequirements.securityModeLevel;
            gPeripheralPairingParameters.securityModeAndLevel =
                deviceSecurityRequirements.pMasterSecurityRequirements->securityModeLevel;
        }

        currServiceInfo = currServiceInfo->next;
    }

    /* Register security requirements */
    Gap_RegisterDeviceSecurityRequirements(&deviceSecurityRequirements);
}

void nxpGattServer::registerCallbacks(void)
{
    serviceInfo_t *currServiceInfo = headServiceInfo;
    uint16_t *writeAttHandles;
    uint16_t writeCount = 0;
    uint16_t *readAttHandles;
    uint16_t readCount = 0;
    writeAttHandles = (uint16_t *)MEM_BufferAlloc(descriptorCount + characteristicCount);
    readAttHandles = (uint16_t *)MEM_BufferAlloc(descriptorCount + characteristicCount);
    while (currServiceInfo)
    {
        for (uint8_t i = 0; i < currServiceInfo->nbOfCharacteristics; i++)
        {
            if (currServiceInfo->pCharacteristicInfo[i].properties & GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_WRITE)
            {
                writeAttHandles[writeCount++] = currServiceInfo->pCharacteristicInfo[i].handle;
            }
            if (currServiceInfo->pCharacteristicInfo[i].properties & GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_READ)
            {
                readAttHandles[readCount++] = currServiceInfo->pCharacteristicInfo[i].handle;
            }

            for (uint8_t j = 0; j < currServiceInfo->pCharacteristicInfo[i].nbOfDescriptors; j++)
            {
                if (currServiceInfo->pCharacteristicInfo[i].pDescriptorInfo[j].isCCCD == 0)
                {
                    if (currServiceInfo->pCharacteristicInfo[i].pDescriptorInfo[j].properties &
                        GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_WRITE)
                    {
                        writeAttHandles[writeCount++] =
                            currServiceInfo->pCharacteristicInfo[i].pDescriptorInfo[j].handle;
                    }
                    if (currServiceInfo->pCharacteristicInfo[i].pDescriptorInfo[j].properties &
                        GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_READ)
                    {
                        readAttHandles[readCount++] = currServiceInfo->pCharacteristicInfo[i].pDescriptorInfo[j].handle;
                    }
                }
            }
        }
        currServiceInfo = currServiceInfo->next;
    }

    /* Register for callbacks*/
    GattServer_RegisterHandlesForWriteNotifications(writeCount, writeAttHandles);
    GattServer_RegisterHandlesForReadNotifications(readCount, readAttHandles);
    App_RegisterGattServerCallback(mbedBleApp_GattServerCallback);

    MEM_BufferFree(writeAttHandles);
    MEM_BufferFree(readAttHandles);
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
ble_error_t nxpGattServer::addService(GattService &service)
{
    // ble_error_t error = BLE_ERROR_NONE;
    uint16_t handle;
    bleResult_t result;
    uint32_t nodeSize = calcServiceInfoSize(service);
    uint8_t requiredServiceSecurity;
    uint8_t prevCharSecurity = SecurityManager::SECURITY_MODE_ENCRYPTION_OPEN_LINK;

    /* Add the service related info into a service list */
    if (headServiceInfo == NULL)
    {
        headServiceInfo = (serviceInfo_t *)MEM_BufferAlloc(nodeSize);
        headServiceInfo->next = NULL;
        tailServiceInfo = headServiceInfo;
    }
    else
    {
        tailServiceInfo->next = (serviceInfo_t *)MEM_BufferAlloc(nodeSize);
        tailServiceInfo = tailServiceInfo->next;
        tailServiceInfo->next = NULL;
    }

    if (service.getUUID().shortOrLong())
    {
        /* Add service */
        result = GattDbDynamic_AddPrimaryServiceDeclaration(gBleUuidType128_c,
                                                            (bleUuid_t *)(service.getUUID().getBaseUUID()), &handle);
    }
    else
    {
        /* Add service */
        result = GattDbDynamic_AddPrimaryServiceDeclaration(gBleUuidType16_c,
                                                            (bleUuid_t *)(service.getUUID().getBaseUUID()), &handle);
    }

    if (result != gBleSuccess_c)
    {
        // FIXME - Map error codes correctly
        return (ble_error_t)(result & 0xff);
    }
    service.setHandle(handle);
    /* Update local service info */
    tailServiceInfo->handle = handle;
    tailServiceInfo->nbOfCharacteristics = service.getCharacteristicCount();
    tailServiceInfo->pCharacteristicInfo = (characteristicInfo_t *)((uint32_t)tailServiceInfo + sizeof(serviceInfo_t));

    /* Add characteristics */
    for (uint8_t i = 0; i < service.getCharacteristicCount(); i++)
    {
        GattCharacteristic *pCharacteristicInfo = service.getCharacteristic(i);
        uint8_t essential_desc = pCharacteristicInfo->getProperties() & ESSENTIAL_DESC_MASK;

        uint8_t permission = 0;
        if (pCharacteristicInfo->getProperties() & GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_READ)
        {
            permission |= gPermissionFlagReadable_c;
        }
        if ((pCharacteristicInfo->getProperties() & GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_WRITE) ||
            (pCharacteristicInfo->getProperties() &
             GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_WRITE_WITHOUT_RESPONSE))
        {
            permission |= gPermissionFlagWritable_c;
        }
        if (pCharacteristicInfo->getValueAttribute().getUUID().shortOrLong())
        {
            /* Add characteristic */
            result = GattDbDynamic_AddCharacteristicDeclarationAndValue(
                gBleUuidType128_c, (bleUuid_t *)pCharacteristicInfo->getValueAttribute().getUUID().getBaseUUID(),
                (gattCharacteristicPropertiesBitFields_t)pCharacteristicInfo->getProperties(),
                pCharacteristicInfo->getValueAttribute().getMaxLength(),
                pCharacteristicInfo->getValueAttribute().getLength(),
                pCharacteristicInfo->getValueAttribute().getValuePtr(), (gattAttributePermissionsBitFields_t)permission,
                &handle);
        }
        else
        {
            /* Add characteristic */
            result = GattDbDynamic_AddCharacteristicDeclarationAndValue(
                gBleUuidType16_c, (bleUuid_t *)pCharacteristicInfo->getValueAttribute().getUUID().getBaseUUID(),
                (gattCharacteristicPropertiesBitFields_t)pCharacteristicInfo->getProperties(),
                pCharacteristicInfo->getValueAttribute().getMaxLength(),
                pCharacteristicInfo->getValueAttribute().getLength(),
                pCharacteristicInfo->getValueAttribute().getValuePtr(), (gattAttributePermissionsBitFields_t)permission,
                &handle);
        }

        if (result != gBleSuccess_c)
        {
            // FIXME - Map error codes correctly
            return (ble_error_t)(result & 0xff);
        }

        service.getCharacteristic(i)->getValueAttribute().setHandle(handle + 1);
        /* Update local characteristic info */
        tailServiceInfo->pCharacteristicInfo[i].handle = handle + 1;
        tailServiceInfo->pCharacteristicInfo[i].maxValueLength =
            pCharacteristicInfo->getValueAttribute().getMaxLength();
        tailServiceInfo->pCharacteristicInfo[i].properties = pCharacteristicInfo->getProperties();
        tailServiceInfo->pCharacteristicInfo[i].nbOfDescriptors = pCharacteristicInfo->getDescriptorCount();

        /* Store security setting of characteristic to temporary variable */
        requiredServiceSecurity = pCharacteristicInfo->getRequiredSecurity();

        /* Check if the security level of this characteristic is greater than previous characteristic. This is done to
          set the security level of the service to the maximum security level of all characteristics in that service */
        if (requiredServiceSecurity >= prevCharSecurity)
        {
            switch (requiredServiceSecurity)
            {
                case SecurityManager::SECURITY_MODE_NO_ACCESS:
                case SecurityManager::SECURITY_MODE_ENCRYPTION_OPEN_LINK:
                {
                    tailServiceInfo->serviceSecurityRequirements.securityModeLevel = gSecurityMode_1_Level_1_c;
                }
                break;
                case SecurityManager::SECURITY_MODE_ENCRYPTION_NO_MITM:
                {
                    tailServiceInfo->serviceSecurityRequirements.securityModeLevel = gSecurityMode_1_Level_2_c;
                }
                break;
                case SecurityManager::SECURITY_MODE_ENCRYPTION_WITH_MITM:
                {
                    tailServiceInfo->serviceSecurityRequirements.securityModeLevel = gSecurityMode_1_Level_3_c;
                }
                break;
                case SecurityManager::SECURITY_MODE_SIGNED_NO_MITM:
                {
                    tailServiceInfo->serviceSecurityRequirements.securityModeLevel = gSecurityMode_2_Level_1_c;
                }
                break;
                case SecurityManager::SECURITY_MODE_SIGNED_WITH_MITM:
                {
                    tailServiceInfo->serviceSecurityRequirements.securityModeLevel = gSecurityMode_2_Level_2_c;
                }
                break;

                default:
                    break;
            } /* end switch*/
            prevCharSecurity = requiredServiceSecurity;
        }

        /* Set the authorization and minimum encryption key size  security settings for the service*/
        tailServiceInfo->serviceSecurityRequirements.authorization = false;
        tailServiceInfo->serviceSecurityRequirements.minimumEncryptionKeySize = gEncryptionKeySize_d;

        if (i == 0)
        {
            tailServiceInfo->pCharacteristicInfo[i].pDescriptorInfo =
                (descriptorInfo_t *)&(tailServiceInfo->pCharacteristicInfo[tailServiceInfo->nbOfCharacteristics]);
        }
        else
        {
            tailServiceInfo->pCharacteristicInfo[i].pDescriptorInfo =
                &(tailServiceInfo->pCharacteristicInfo[i - 1]
                      .pDescriptorInfo[tailServiceInfo->pCharacteristicInfo[i - 1].nbOfDescriptors]);
        }

        /* Add descriptors */
        for (uint8_t j = 0; j < pCharacteristicInfo->getDescriptorCount(); j++)
        {
            GattAttribute *pDescriptorInfo = pCharacteristicInfo->getDescriptor(j);
            tailServiceInfo->pCharacteristicInfo[i].pDescriptorInfo[j].isCCCD = 0;
            if (pCharacteristicInfo->getValueAttribute().getUUID().shortOrLong())
            {
                /* Add descriptor */
                result = GattDbDynamic_AddCharacteristicDescriptor(
                    gBleUuidType128_c, (bleUuid_t *)pDescriptorInfo->getUUID().getBaseUUID(),
                    pDescriptorInfo->getLength(), pDescriptorInfo->getValuePtr(), gPermissionNone_c, &handle);
            }
            else
            {
                /* Add descriptor */
                result = GattDbDynamic_AddCharacteristicDescriptor(
                    gBleUuidType16_c, (bleUuid_t *)pDescriptorInfo->getUUID().getBaseUUID(),
                    pDescriptorInfo->getLength(), pDescriptorInfo->getValuePtr(), gPermissionNone_c, &handle);
                switch (pDescriptorInfo->getUUID().getShortUUID())
                {
                    case BLE_UUID_DESCRIPTOR_CHAR_EXT_PROP:
                        essential_desc &= ~GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_EXTENDED_PROPERTIES;
                        break;
                    case BLE_UUID_DESCRIPTOR_CLIENT_CHAR_CONFIG:
                        essential_desc &= ~(GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_NOTIFY |
                                            GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_INDICATE);
                        if (tailServiceInfo->pCharacteristicInfo[i].properties &
                            GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_NOTIFY)
                        {
                            tailServiceInfo->pCharacteristicInfo[i].pDescriptorInfo[j].isCCCD = BLE_HVX_NOTIFICATION;
                        }
                        else if (tailServiceInfo->pCharacteristicInfo[i].properties &
                                 GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_INDICATE)
                        {
                            tailServiceInfo->pCharacteristicInfo[i].pDescriptorInfo[j].isCCCD = BLE_HVX_INDICATION;
                        }
                        break;
                    case BLE_UUID_DESCRIPTOR_SERVER_CHAR_CONFIG:
                        essential_desc &= ~GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_BROADCAST;
                        break;
                }
            }
            if (result != gBleSuccess_c)
            {
                // FIXME - Map error codes correctly
                return (ble_error_t)(result & 0xff);
            }

            service.getCharacteristic(i)->getDescriptor(j)->setHandle(handle);
            /* Update local descriptor info */
            tailServiceInfo->pCharacteristicInfo[i].pDescriptorInfo[j].handle = handle;
            tailServiceInfo->pCharacteristicInfo[i].pDescriptorInfo[j].maxValueLength = pDescriptorInfo->getLength();
            tailServiceInfo->pCharacteristicInfo[i].pDescriptorInfo[j].properties =
                GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_WRITE | GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_READ;
        }
        descriptorCount += pCharacteristicInfo->getDescriptorCount();

        /* Add CCCD */

        for (uint8_t j = 0x80; essential_desc && j; j >>= 1)
        {
            if ((essential_desc & j) && ((j == GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_NOTIFY) ||
                                         (j == GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_INDICATE)))
            {
                essential_desc &= ~j;
                /* Essential descriptor values are always 16 bit so add two extra bytes */
                result = GattDbDynamic_AddCccd(&handle);

                if (result != gBleSuccess_c)
                {
                    // FIXME - Map error codes correctly
                    return (ble_error_t)(result & 0xff);
                }
                /* Update local descriptor info for forced CCCD */
                tailServiceInfo->pCharacteristicInfo[i]
                    .pDescriptorInfo[tailServiceInfo->pCharacteristicInfo[i].nbOfDescriptors]
                    .handle = handle;
                tailServiceInfo->pCharacteristicInfo[i]
                    .pDescriptorInfo[tailServiceInfo->pCharacteristicInfo[i].nbOfDescriptors]
                    .maxValueLength = 2;

                tailServiceInfo->pCharacteristicInfo[i]
                    .pDescriptorInfo[tailServiceInfo->pCharacteristicInfo[i].nbOfDescriptors]
                    .isCCCD = 0;
                if (tailServiceInfo->pCharacteristicInfo[i].properties &
                    GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_NOTIFY)
                {
                    tailServiceInfo->pCharacteristicInfo[i]
                        .pDescriptorInfo[tailServiceInfo->pCharacteristicInfo[i].nbOfDescriptors]
                        .isCCCD = BLE_HVX_NOTIFICATION;
                }
                else if (tailServiceInfo->pCharacteristicInfo[i].properties &
                         GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_INDICATE)
                {
                    tailServiceInfo->pCharacteristicInfo[i]
                        .pDescriptorInfo[tailServiceInfo->pCharacteristicInfo[i].nbOfDescriptors]
                        .isCCCD = BLE_HVX_INDICATION;
                }

                tailServiceInfo->pCharacteristicInfo[i]
                    .pDescriptorInfo[tailServiceInfo->pCharacteristicInfo[i].nbOfDescriptors]
                    .properties = GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_WRITE |
                                  GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_READ;
                tailServiceInfo->pCharacteristicInfo[i].nbOfDescriptors++;
                descriptorCount++;
            }
        }
    }

    registerCallbacks();
    characteristicCount += service.getCharacteristicCount();
    serviceCount++;

    /* Register Device security if Security is initialized*/
    if (((nxpSecurityManager &)(BLE::Instance(BLE::DEFAULT_INSTANCE).securityManager())).hasInitialized())
    {
        registerDeviceSecurity();
    }
    return BLE_ERROR_NONE;
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
ble_error_t nxpGattServer::read(GattAttribute::Handle_t attributeHandle, uint8_t buffer[], uint16_t *lengthP)
{
    return read(0, attributeHandle, buffer, lengthP);
}

ble_error_t nxpGattServer::read(Gap::Handle_t connectionHandle,
                                GattAttribute::Handle_t attributeHandle,
                                uint8_t buffer[],
                                uint16_t *lengthP)
{
    void *attInfo;
    attType_t attType;
    attType = getAttributeInfo(attributeHandle, &attInfo);
    if (attType == ATTR_CHAR)
    {
        return (ble_error_t)(
            GattDb_ReadAttribute(attributeHandle, ((characteristicInfo_t *)attInfo)->maxValueLength, buffer, lengthP) &
            0xff);
    }
    else if (attType == ATTR_DESC)
    {
        if (((descriptorInfo_t *)attInfo)->isCCCD == BLE_HVX_NOTIFICATION)
        {
            Gap_CheckNotificationStatus(connectionHandle, attributeHandle, buffer);
            buffer[1] = 0;
            *lengthP = 2;
        }
        else if (((descriptorInfo_t *)attInfo)->isCCCD == BLE_HVX_INDICATION)
        {
            Gap_CheckIndicationStatus(connectionHandle, attributeHandle, buffer);
            buffer[0] <<= 1;
            buffer[1] = 0;
            *lengthP = 2;
        }
        else
        {
            return (ble_error_t)(
                GattDb_ReadAttribute(attributeHandle, ((descriptorInfo_t *)attInfo)->maxValueLength, buffer, lengthP) &
                0xff);
        }
    }
    return BLE_ERROR_INVALID_PARAM;
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
ble_error_t nxpGattServer::write(GattAttribute::Handle_t attributeHandle,
                                 const uint8_t buffer[],
                                 uint16_t len,
                                 bool localOnly)
{
    ble_error_t error = BLE_ERROR_NONE;
    for (uint8_t i = 0; (i < 32) && ((error == BLE_ERROR_NONE) || (error == BLE_ERROR_OPERATION_NOT_PERMITTED)); i++)
    {
        if (connectionStatus & (1 << i))
        {
            error = write(i, attributeHandle, buffer, len, localOnly);
        }
    }
    return error;
}

ble_error_t nxpGattServer::write(Gap::Handle_t connectionHandle,
                                 GattAttribute::Handle_t attributeHandle,
                                 const uint8_t buffer[],
                                 uint16_t len,
                                 bool localOnly)
{
    bleResult_t result;
    uint16_t cccdHandle;
    bool_t notifyActive;
    result = (bleResult_t)(GattDb_WriteAttribute(attributeHandle, len, (uint8_t *)buffer) & 0xff);
    if ((result == gBleSuccess_c) && (!localOnly))
    {
        result = GattDb_FindCccdHandleForCharValueHandle(attributeHandle, &cccdHandle);
        if (!result)
        {
            void *attInfo;
            getAttributeInfo(cccdHandle, &attInfo);
            if (attInfo)
            {
                if (((descriptorInfo_t *)attInfo)->isCCCD == BLE_HVX_NOTIFICATION)
                {
                    if ((gBleSuccess_c == Gap_CheckNotificationStatus(connectionHandle, cccdHandle, &notifyActive)) &&
                        (TRUE == notifyActive))
                    {
                        if (gBleSuccess_c == GattServer_SendNotification(connectionHandle, attributeHandle))
                        {
                            handleDataSentEvent(1);
                        }
                        else
                        {
                            return BLE_ERROR_OPERATION_NOT_PERMITTED;
                        }
                    }
                    else
                    {
                        // handleDataSentEvent(0);
                        return BLE_ERROR_OPERATION_NOT_PERMITTED;
                    }
                }
                else if (((descriptorInfo_t *)attInfo)->isCCCD == BLE_HVX_INDICATION)
                {
                    if ((gBleSuccess_c == Gap_CheckIndicationStatus(connectionHandle, cccdHandle, &notifyActive)) &&
                        (TRUE == notifyActive))
                    {
                        lastIndicateHandle = attributeHandle;
                        if (gBleSuccess_c == GattServer_SendIndication(connectionHandle, attributeHandle))
                        {
                            handleDataSentEvent(1);
                        }
                        else
                        {
                            return BLE_ERROR_OPERATION_NOT_PERMITTED;
                        }
                    }
                    else
                    {
                        // handleDataSentEvent(0);
                        return BLE_ERROR_OPERATION_NOT_PERMITTED;
                    }
                }
            }
        }
        else
        {
            /* Descriptors are not notified */
            return BLE_ERROR_INVALID_PARAM;
        }
    }

    return (ble_error_t)(result & 0xff);
}

ble_error_t nxpGattServer::areUpdatesEnabled(const GattCharacteristic &characteristic, bool *enabledP)
{
    /* Forward the call with the default connection handle. */
    return areUpdatesEnabled(0, characteristic, enabledP);
}

ble_error_t nxpGattServer::areUpdatesEnabled(Gap::Handle_t connectionHandle,
                                             const GattCharacteristic &characteristic,
                                             bool *enabledP)
{
    bleResult_t result;
    uint16_t cccdHandle;

    result = GattDb_FindCccdHandleForCharValueHandle(characteristic.getValueHandle(), &cccdHandle);
    if (!result)
    {
        void *attInfo;
        getAttributeInfo(cccdHandle, &attInfo);
        if (attInfo)
        {
            if (((descriptorInfo_t *)attInfo)->isCCCD == BLE_HVX_NOTIFICATION)
            {
                if (gBleSuccess_c == Gap_CheckNotificationStatus(connectionHandle, cccdHandle, (bool_t *)enabledP))
                {
                    return BLE_ERROR_NONE;
                }
            }
            else if (((descriptorInfo_t *)attInfo)->isCCCD == BLE_HVX_INDICATION)
            {
                if (gBleSuccess_c == Gap_CheckIndicationStatus(connectionHandle, cccdHandle, (bool_t *)enabledP))
                {
                    return BLE_ERROR_NONE;
                }
            }
        }
    }
    return BLE_ERROR_INVALID_PARAM;
}

/**************************************************************************/
/*!
    @brief  Clear nxpGattServer state.

    @returns    ble_error_t

    @retval     BLE_ERROR_NONE
                Everything executed properly
*/
/**************************************************************************/
ble_error_t nxpGattServer::reset(void)
{
    serviceInfo_t *temp;
    /* Clear all state that is from the parent, including private members */
    if (GattServer::reset() != BLE_ERROR_NONE)
    {
        return BLE_ERROR_INVALID_STATE;
    }

    /* Clear derived class members */
    while (headServiceInfo)
    {
        temp = headServiceInfo;
        headServiceInfo = headServiceInfo->next;
        MEM_BufferFree(temp);
    }
    tailServiceInfo = headServiceInfo;
    descriptorCount = 0;
    connectionStatus = 0;
    lastIndicateHandle = 0;

    return BLE_ERROR_NONE;
}

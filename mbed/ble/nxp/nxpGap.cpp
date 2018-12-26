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

//#include "Instance.h"
//#ifdef YOTTA_CFG_MBED_OS
//    #include "mbed-drivers/mbed.h"
//#else
//    #include "mbed.h"
//#endif
#include "BLE.h"
#include "mbedGap.h"
#include "nxpGap.h"
#include "nxpInstance.h"
#include "appble.h"

extern "C" {
gapAdvertisingFilterPolicy_t mbedGetAdvFilterPolicy(void)
{
    return (gapAdvertisingFilterPolicy_t)BLE::Instance(BLE::DEFAULT_INSTANCE).gap().getAdvertisingPolicyMode();
}

uint16_t mbedGetAdvTimeout(void)
{
    return BLE::Instance(BLE::DEFAULT_INSTANCE).gap().getAdvertisingParams().getTimeout();
}

uint16_t mbedGetScanTimeout(void)
{
    return ((nxpGap &)(BLE::Instance(BLE::DEFAULT_INSTANCE).gap())).getScanningParams().getTimeout();
}

void readPubBleAddrDone(uint8_t *addr)
{
    ((nxpGap &)(BLE::Instance(BLE::DEFAULT_INSTANCE).gap())).readBleAddressComplete(addr);
}

void setRandBleAddrDone(void)
{
    ((nxpGap &)(BLE::Instance(BLE::DEFAULT_INSTANCE).gap())).setRandomAddressComplete();
}

void ClrWhiteListDone(void)
{
    ((nxpGap &)(BLE::Instance(BLE::DEFAULT_INSTANCE).gap())).clrWhiteListComplete();
}
void AddWhiteListDone(void)
{
    ((nxpGap &)(BLE::Instance(BLE::DEFAULT_INSTANCE).gap())).AddWhiteListComplete();
}

bleResult_t App_StartAdvertising(gapAdvertisingCallback_t advertisingCallback,
                                 gapConnectionCallback_t connectionCallback);

void initComplete(gapServiceHandles_t handles);
void BleConnManager_GapCommonConfig(void);
extern uint8_t keysNvmWritten;
}

#define gSetAdvDataReq (1UL << 0)
#define gSetAdvParamReq (1UL << 1)
#define gStartAdvReq (1UL << 2)
#define gReadBleAddrReq (1UL << 3)
#define gSetBleAddrReq (1UL << 4)
#define gClrWhiteListReq (1UL << 5)
#define gAddWhiteListReq (1UL << 6)
#define gStopScanReq (1UL << 7)

void BleApp_Init(void)
{
}

void BleApp_GenericCallback(gapGenericEvent_t *pGenericEvent)
{
    bleResult_t result = gBleSuccess_c;
    gapServiceHandles_t gapServiceHandles;
    /* Call BLE Conn Manager */
    BleConnManager_GenericEvent(pGenericEvent);

    switch (pGenericEvent->eventType)
    {
        case gInitializationComplete_c:
        {
            result = GattDbDynamic_AddGattService(NULL);

            if (gBleSuccess_c == result)
            {
                result = GattDbDynamic_AddGapService(&gapServiceHandles);
            }
            initComplete(gapServiceHandles);
        }
        break;

        case gAdvertisingDataSetupComplete_c:
        {
            ((nxpGap &)(BLE::Instance(BLE::DEFAULT_INSTANCE).gap())).advDataSetupComplete();
        }
        break;

        case gAdvertisingParametersSetupComplete_c:
        {
            ((nxpGap &)(BLE::Instance(BLE::DEFAULT_INSTANCE).gap())).advParamSetupComplete();
        }
        break;

        default:
            break;
    }
}

void nxpGap::gapInit(gapServiceHandles_t handles)
{
    gapIdentityInformation_t bondedDevices[gMaxBondedDevices_c];
    gapIdentityInformation_t privateDevices[gcGapControllerResolvingListSize_c];
    uint8_t deviceCount = 0, privateIndex = 0;
    bleResult_t result;

    advTimerId = TM_AllocateTimer();
    scanTimerId = TM_AllocateTimer();

    gapHandles.serviceHandle = handles.serviceHandle;
    gapHandles.charDeviceNameHandle = handles.charDeviceNameHandle;
    gapHandles.charAppearanceHandle = handles.charAppearanceHandle;
    gapHandles.charPpcpHandle = handles.charPpcpHandle;

    result = Gap_GetBondedDevicesIdentityInformation(bondedDevices, gMaxBondedDevices_c, &deviceCount);

    if ((gBleSuccess_c == result) && (deviceCount > 0))
    {
        /* Clear White List first */
        Gap_ClearWhiteList();
        gapReqCompleteFlags |= gClrWhiteListReq;
        do
        {
            OSA_Start();
        } while (gapReqCompleteFlags & gClrWhiteListReq);

        whitelistAddressesSize = 0;
        for (uint8_t i = 0; i < deviceCount; i++)
        {
            memcpy(whitelistAddresses[whitelistAddressesSize].idAddress, bondedDevices[i].identityAddress.idAddress,
                   gcBleDeviceAddressSize_c);
            whitelistAddresses[whitelistAddressesSize].idAddressType = bondedDevices[i].identityAddress.idAddressType;
            Gap_AddDeviceToWhiteList(whitelistAddresses[whitelistAddressesSize].idAddressType,
                                     whitelistAddresses[whitelistAddressesSize].idAddress);
            gapReqCompleteFlags |= gAddWhiteListReq;
            do
            {
                OSA_Start();
            } while (gapReqCompleteFlags & gAddWhiteListReq);
            whitelistAddressesSize++;

            if ((bondedDevices[i].identityAddress.idAddressType == gBleAddrTypeRandom_c) &&
                (privateIndex < gcGapControllerResolvingListSize_c))
            {
                memcpy(&privateDevices[privateIndex], &bondedDevices[i], sizeof(gapIdentityInformation_t));
                privateIndex++;
            }
        }
        if (privateIndex > 0)
        {
            Gap_EnableControllerPrivacy(TRUE, gSmpKeys.aIrk, privateIndex, privateDevices);
        }
    }
}

void nxpGap::advDataSetupComplete(void)
{
    gapReqCompleteFlags &= ~gSetAdvDataReq;
}

void nxpGap::advParamSetupComplete(void)
{
    // gapReqCompleteFlags &= ~gSetAdvDataReq;
    /* Clear request completion flag */
    if (gapReqCompleteFlags & gSetAdvParamReq)
    {
        /* Clear request completion flag only if request is set */
        gapReqCompleteFlags &= ~gSetAdvParamReq;
    }
    else
    {
        /* If there is no request set, then this is due to change from fast to slow advertising */
        App_StartAdvertising(nxpGap::AdvertisingCallback, nxpGap::ConnectionCallback);
    }
}

void nxpGap::readBleAddressComplete(uint8_t *addr)
{
    memcpy(blePubAddr, addr, gcBleDeviceAddressSize_c);
    gapReqCompleteFlags &= ~gReadBleAddrReq;
}

void nxpGap::setRandomAddressComplete(void)
{
    gapReqCompleteFlags &= ~gSetBleAddrReq;
}

void nxpGap::clrWhiteListComplete(void)
{
    gapReqCompleteFlags &= ~gClrWhiteListReq;
}

void nxpGap::AddWhiteListComplete(void)
{
    gapReqCompleteFlags &= ~gAddWhiteListReq;
}

gapRole_t nxpGap::getDeviceRole(Gap::Handle_t connectionHandle)
{
    return deviceRole[connectionHandle].role;
}

void nxpGap::AdvertisingCallback(gapAdvertisingEvent_t *pAdvertisingEvent)
{
    ((nxpGap &)(BLE::Instance(BLE::DEFAULT_INSTANCE).gap())).AdvertisingHandler(pAdvertisingEvent);
}
void nxpGap::AdvertisingHandler(gapAdvertisingEvent_t *pAdvertisingEvent)
{
    switch (pAdvertisingEvent->eventType)
    {
        case gAdvertisingStateChanged_c:
        {
            advState.advOn = !advState.advOn;

            if (advState.advOn)
            {
                TM_SetTimer(advTimerId, kTM_SingleShotTimer, TA_SECOND(advTimeout), nxpGap::AdvertisingTimerCallback,
                            NULL);
                /* Clear request completion flag */
                gapReqCompleteFlags &= ~gStartAdvReq;
            }
            else if (restartAdv)
            {
                restartAdv = FALSE;
                if (!advState.advFast)
                {
                    advParams.minInterval = gReducedPowerMinAdvInterval_c;
                    advParams.maxInterval = gReducedPowerMinAdvInterval_c;
                    advParams.filterPolicy = (gapAdvertisingFilterPolicy_t)advertisingPolicyMode;
                    advTimeout = getAdvertisingParams().getTimeout();
                    if (advTimeout == 0)
                    {
                        advTimeout = gReducedPowerAdvTime_c;
                    }
                }

                /* Set advertising parameters*/
                Gap_SetAdvertisingParameters(&advParams);
            }
            else
            {
                /*Advertising stopped, so reset to fast advertising */
                advState.advFast = TRUE;
                TM_ClearTimer(advTimerId);
            }
        }
        break;

        case gAdvertisingCommandFailed_c:
            // FIXME - Handle error
            break;
        default:
            break;
    }
}

void nxpGap::AdvertisingTimerCallback(void *pParam)
{
    ((nxpGap &)(BLE::Instance(BLE::DEFAULT_INSTANCE).gap())).AdvertisingTimerHandler(pParam);
}

void nxpGap::AdvertisingTimerHandler(void *pParam)
{
    /* Stop and restart advertising with new parameters */
    stopAdvertising();

    if (advState.advFast)
    {
        advState.advFast = FALSE;
        restartAdv = TRUE;
    }
    else if (getAdvertisingParams().getTimeout() == 0)
    {
        restartAdv = TRUE;
    }
    else
    {
        restartAdv = FALSE;
    }
}

void nxpGap::ScanningCallback(gapScanningEvent_t *pScanningEvent)
{
    ((nxpGap &)(BLE::Instance(BLE::DEFAULT_INSTANCE).gap())).ScanningEventHandler(pScanningEvent);
}

void nxpGap::ScanningEventHandler(gapScanningEvent_t *pScanningEvent)
{
    switch (pScanningEvent->eventType)
    {
        case gDeviceScanned_c:
        {
            GapAdvertisingParams::AdvertisingType_t type;
            switch (pScanningEvent->eventData.scannedDevice.advEventType)
            {
                case gBleAdvRepAdvInd_c:
                    type = GapAdvertisingParams::ADV_CONNECTABLE_UNDIRECTED;
                    break;
                case gBleAdvRepAdvDirectInd_c:
                    type = GapAdvertisingParams::ADV_CONNECTABLE_DIRECTED;
                    break;
                case gBleAdvRepAdvScanInd_c:
                    type = GapAdvertisingParams::ADV_SCANNABLE_UNDIRECTED;
                    break;
                case gBleAdvRepAdvNonconnInd_c:
                    type = GapAdvertisingParams::ADV_NON_CONNECTABLE_UNDIRECTED;
                    break;
                default:
                    type = GapAdvertisingParams::ADV_CONNECTABLE_UNDIRECTED;
                    break;
            }
            // FIXME - Should we renew Scan timer here??

            processAdvertisementReport(
                pScanningEvent->eventData.scannedDevice.aAddress, pScanningEvent->eventData.scannedDevice.rssi,
                (pScanningEvent->eventData.scannedDevice.advEventType == gBleAdvRepScanRsp_c), type,
                pScanningEvent->eventData.scannedDevice.dataLength, pScanningEvent->eventData.scannedDevice.data);
        }
        break;
        case gScanStateChanged_c:
        {
            scanningOn = !scanningOn;
            if (scanningOn)
            {
                TM_SetTimer(scanTimerId, kTM_SingleShotTimer, TA_SECOND(getScanningParams().getTimeout()),
                            nxpGap::ScanningTimerCallback, NULL);
            }
            else
            {
                TM_ClearTimer(scanTimerId);
                gapReqCompleteFlags &= ~gStopScanReq;
            }
        }
        break;
        case gScanCommandFailed_c:
        {
        }
        default:
            break;
    }
}

void nxpGap::ScanningTimerCallback(void *pParam)
{
    ((nxpGap &)(BLE::Instance(BLE::DEFAULT_INSTANCE).gap())).ScanningTimerHandler(pParam);
}

void nxpGap::ScanningTimerHandler(void *pParam)
{
    /* Stop Scanning */
    stopScan();
}

void nxpGap::ConnectionCallback(deviceId_t peerDeviceId, gapConnectionEvent_t *pConnectionEvent)
{
    ((nxpGap &)(BLE::Instance(BLE::DEFAULT_INSTANCE).gap())).ConnectionEventHandler(peerDeviceId, pConnectionEvent);
}

void nxpGap::ConnectionEventHandler(deviceId_t peerDeviceId, gapConnectionEvent_t *pConnectionEvent)
{
    switch (pConnectionEvent->eventType)
    {
        case gConnEvtConnected_c:
        {
            bool_t isBonded = FALSE;

            if (connectInitiated && (memcmp(peerAddress, pConnectionEvent->eventData.connectedEvent.peerAddress,
                                            gcBleDeviceAddressSize_c) == 0))
            {
                connectInitiated = FALSE;
                deviceRole[peerDeviceId].role = gGapCentral_c;
                deviceRole[peerDeviceId].address.idAddressType =
                    pConnectionEvent->eventData.connectedEvent.peerAddressType;
                memcpy(deviceRole[peerDeviceId].address.idAddress,
                       pConnectionEvent->eventData.connectedEvent.peerAddress, gcBleDeviceAddressSize_c);

                Gap_CheckIfBonded(peerDeviceId, &isBonded);
                if (isBonded)
                {
                    /* Restored custom connection information. Encrypt link */
                    Gap_EncryptLink(peerDeviceId);
                }
            }
            else
            {
                /* This is connection event for the peripheral role */
                deviceRole[peerDeviceId].role = gGapPeripheral_c;
                deviceRole[peerDeviceId].address.idAddressType =
                    pConnectionEvent->eventData.connectedEvent.peerAddressType;
                memcpy(deviceRole[peerDeviceId].address.idAddress,
                       pConnectionEvent->eventData.connectedEvent.peerAddress, gcBleDeviceAddressSize_c);
                /* Advertising stops when connected, so reset flags */
                advState.advOn = FALSE;
                advState.advFast = TRUE;
                restartAdv = FALSE;
                TM_ClearTimer(advTimerId);

                ((nxpGattServer &)(nxpInstance::Instance(BLE::DEFAULT_INSTANCE).getGattServer()))
                    .connectionEvent(peerDeviceId);
                if ((gBleSuccess_c == Gap_CheckIfBonded(peerDeviceId, &isBonded)) && (FALSE == isBonded))
                {
                    // FIXME - Bonding and security mode should be obtained from Security Manager
                    //                    SecurityManager::SecurityMode_t securityMode;
                    //                    switch (gPeripheralPairingParameters.securityModeAndLevel)
                    //                    {
                    //                        case gSecurityMode_1_Level_1_c:
                    //                            securityMode = SecurityManager::SECURITY_MODE_ENCRYPTION_OPEN_LINK;
                    //                            break;
                    //                        case gSecurityMode_1_Level_2_c:
                    //                            securityMode = SecurityManager::SECURITY_MODE_ENCRYPTION_NO_MITM;
                    //                            break;
                    //                        case gSecurityMode_1_Level_3_c:
                    //                            securityMode = SecurityManager::SECURITY_MODE_ENCRYPTION_WITH_MITM;
                    //                            break;
                    //                        case gSecurityMode_2_Level_1_c:
                    //                            securityMode = SecurityManager::SECURITY_MODE_SIGNED_NO_MITM;
                    //                            break;
                    //                        case gSecurityMode_2_Level_2_c:
                    //                            securityMode = SecurityManager::SECURITY_MODE_SIGNED_WITH_MITM;
                    //                            break;
                    //                    }

                    //                    (BLE::Instance(BLE::DEFAULT_INSTANCE).securityManager()).setLinkSecurity((Gap::Handle_t)peerDeviceId,
                    //                    securityMode);
                }
            }

            Gap_EnableUpdateConnectionParameters(peerDeviceId, TRUE);

            Gap::ConnectionParams_t connection_param;
            AddressType_t ownType;
            Address_t ownAddress;

            connection_param.minConnectionInterval =
                pConnectionEvent->eventData.connectedEvent.connParameters.connInterval;
            connection_param.maxConnectionInterval =
                pConnectionEvent->eventData.connectedEvent.connParameters.connInterval;
            connection_param.slaveLatency = pConnectionEvent->eventData.connectedEvent.connParameters.connLatency;
            connection_param.connectionSupervisionTimeout =
                pConnectionEvent->eventData.connectedEvent.connParameters.supervisionTimeout;

            getAddress(&ownType, ownAddress);
            /* Call the application callbacks */
            processConnectionEvent(
                peerDeviceId, Gap::PERIPHERAL,
                (BLEProtocol::AddressType_t)pConnectionEvent->eventData.connectedEvent.peerAddressType,
                pConnectionEvent->eventData.connectedEvent.peerAddress, ownType, ownAddress, &connection_param);
        }
        break;

        case gConnEvtDisconnected_c:
        {
            if (deviceRole[peerDeviceId].role == gGapPeripheral_c)
            {
                ((nxpGattServer &)(nxpInstance::Instance(BLE::DEFAULT_INSTANCE).getGattServer()))
                    .disconnectionEvent(peerDeviceId);
            }
            deviceRole[peerDeviceId].role = (gapRole_t)0xFF;
            processDisconnectionEvent(peerDeviceId,
                                      (Gap::DisconnectionReason_t)pConnectionEvent->eventData.disconnectedEvent.reason);
        }
        break;
        case gConnEvtPairingComplete_c:
            if (keysNvmWritten)
            {
                /*Call to indicate that device context is stored persistently */
                (BLE::Instance(BLE::DEFAULT_INSTANCE).securityManager())
                    .processSecurityContextStoredEvent((Gap::Handle_t)peerDeviceId);
            }
            if (pConnectionEvent->eventData.pairingCompleteEvent.pairingSuccessful &&
                pConnectionEvent->eventData.pairingCompleteEvent.pairingCompleteData.withBonding)
            {
                /* Update Link security state of a connection handle */
                ((nxpSecurityManager &)(BLE::Instance(BLE::DEFAULT_INSTANCE).securityManager()))
                    .setLinkSecurityState(peerDeviceId, nxpSecurityManager::kEncrypted);

                /* If a bond is created, write device address in controller’s White List */
                Gap_AddDeviceToWhiteList(deviceRole[peerDeviceId].address.idAddressType,
                                         deviceRole[peerDeviceId].address.idAddress);
                gapReqCompleteFlags |= gAddWhiteListReq;
                do
                {
                    OSA_Start();
                } while (gapReqCompleteFlags & gAddWhiteListReq);

                whitelistAddresses[whitelistAddressesSize].idAddressType =
                    deviceRole[peerDeviceId].address.idAddressType;
                memcpy(whitelistAddresses[whitelistAddressesSize].idAddress, deviceRole[peerDeviceId].address.idAddress,
                       gcBleDeviceAddressSize_c);
                whitelistAddressesSize++;
                if (deviceRole[peerDeviceId].address.idAddressType == gBleAddrTypeRandom_c)
                {
                    gapIdentityInformation_t aOutIdentityAddresses[gcGapControllerResolvingListSize_c];
                    uint8_t identitiesCount = 0;
                    bleResult_t result = Gap_GetBondedDevicesIdentityInformation(
                        aOutIdentityAddresses, gcGapControllerResolvingListSize_c, &identitiesCount);
                    if (gBleSuccess_c == result && (identitiesCount <= gcGapControllerResolvingListSize_c))
                    {
                        if (identitiesCount > 1)
                        {
                            /* Disable Privacy first if it was enabled earlier. */
                            Gap_EnableControllerPrivacy(FALSE, NULL, 0, NULL);
                        }
                        /* (Re)enable Controller Privacy. */
                        Gap_EnableControllerPrivacy(TRUE, gSmpKeys.aIrk, identitiesCount, aOutIdentityAddresses);
                    }
                }
            }
            ((nxpSecurityManager &)(BLE::Instance(BLE::DEFAULT_INSTANCE).securityManager()))
                .ProcessSecSetupCompletedEvent(peerDeviceId, pConnectionEvent);
            break;
        case gConnEvtPairingRequest_c:
        case gConnEvtEncryptionChanged_c:
        case gConnEvtPasskeyDisplay_c:
        case gConnEvtPasskeyRequest_c:
            ((nxpSecurityManager &)(BLE::Instance(BLE::DEFAULT_INSTANCE).securityManager()))
                .ProcessSecurityEvent(peerDeviceId, pConnectionEvent);
            break;
        default:
            if (deviceRole[peerDeviceId].role == gGapCentral_c)
            {
                BleConnManager_GapCentralEvent(peerDeviceId, pConnectionEvent);
            }
            else
            {
                /* Connection Manager to handle Host Stack interactions */
                BleConnManager_GapPeripheralEvent(peerDeviceId, pConnectionEvent);
            }
            break;
    }
}

/**************************************************************************/
/*!
    @brief  Sets the advertising parameters and payload for the device

    @param[in]  params
                Basic advertising details, including the advertising
                delay, timeout and how the device should be advertised
    @params[in] advData
                The primary advertising data payload
    @params[in] scanResponse
                The optional Scan Response payload if the advertising
                type is set to \ref GapAdvertisingParams::ADV_SCANNABLE_UNDIRECTED
                in \ref GapAdveritinngParams

    @returns    \ref ble_error_t

    @retval     BLE_ERROR_NONE
                Everything executed properly

    @retval     BLE_ERROR_BUFFER_OVERFLOW
                The proposed action would cause a buffer overflow.  All
                advertising payloads must be <= 31 bytes, for example.

    @retval     BLE_ERROR_NOT_IMPLEMENTED
                A feature was requested that is not yet supported in the
                NXP firmware or hardware.

    @retval     BLE_ERROR_PARAM_OUT_OF_RANGE
                One of the proposed values is outside the valid range.

    @section EXAMPLE

    @code

    @endcode
*/

/**************************************************************************/
ble_error_t nxpGap::setAdvertisingData(const GapAdvertisingData &advData, const GapAdvertisingData &scanResponse)
{
    uint8_t i;
    uint8_t len = 0;
    const uint8_t *payloadptr;

    /* Make sure we don't exceed the advertising payload length */
    if (advData.getPayloadLen() > GAP_ADVERTISING_DATA_MAX_PAYLOAD)
    {
        return BLE_ERROR_BUFFER_OVERFLOW;
    }

    /* Make sure we have a payload! */
    if (advData.getPayloadLen() == 0)
    {
        return BLE_ERROR_PARAM_OUT_OF_RANGE;
    }

    gapAdStructure_t gapAdvDataAdStruct[10];
    gapAdvertisingData_t gAppAdvertisingTempData;
    payloadptr = advData.getPayload();

    for (i = 0; len < advData.getPayloadLen(); i++)
    {
        if (*(payloadptr + len) == 0)
            break;
        gapAdvDataAdStruct[i].length = *(payloadptr + len);
        gapAdvDataAdStruct[i].adType = (gapAdType_t) * (payloadptr + len + 1);
        gapAdvDataAdStruct[i].aData = (uint8_t *)(payloadptr + len + 2);
        len += gapAdvDataAdStruct[i].length + 1;
    }
    gAppAdvertisingTempData.aAdStructures = &gapAdvDataAdStruct[0];
    gAppAdvertisingTempData.cNumAdStructures = i;

    len = 0;

    payloadptr = scanResponse.getPayload();
    gapAdStructure_t gapScanRespAdStruct[10];
    gapScanResponseData_t gAppScanResponseTempData;

    for (i = 0; len < scanResponse.getPayloadLen(); i++)
    {
        if (*(payloadptr + len) == 0)
            break;
        gapScanRespAdStruct[i].length = *(payloadptr + len);
        gapScanRespAdStruct[i].adType = (gapAdType_t) * (payloadptr + len + 1);
        gapScanRespAdStruct[i].aData = (uint8_t *)(payloadptr + len + 2);
        len += gapScanRespAdStruct[i].length + 1;
    }
    gAppScanResponseTempData.aAdStructures = &gapScanRespAdStruct[0];
    gAppScanResponseTempData.cNumAdStructures = i;

    Gap_SetAdvertisingData(&gAppAdvertisingTempData, &gAppScanResponseTempData);

    gapReqCompleteFlags |= gSetAdvDataReq;
    do
    {
        /* Call scheduler to get the response completion indication */
        OSA_Start();
    } while (gapReqCompleteFlags & gSetAdvDataReq);

    return BLE_ERROR_NONE;
}

/**************************************************************************/
/*!
    @brief  Starts the BLE HW, initialising any services that were
            added before this function was called.

    @note   All services must be added before calling this function!

    @returns    ble_error_t

    @retval     BLE_ERROR_NONE
                Everything executed properly

    @section EXAMPLE

    @code

    @endcode
*/
/**************************************************************************/
ble_error_t nxpGap::startAdvertising(const GapAdvertisingParams &params)
{
    /* Make sure we support the advertising type */
    if (params.getAdvertisingType() == GapAdvertisingParams::ADV_CONNECTABLE_DIRECTED)
    {
        /* ToDo: This requires a propery security implementation, etc. */
        return BLE_ERROR_NOT_IMPLEMENTED;
    }

    /* Check interval range */
    if (params.getAdvertisingType() == GapAdvertisingParams::ADV_NON_CONNECTABLE_UNDIRECTED)
    {
        /* Min delay is slightly longer for unconnectable devices */
        if ((params.getIntervalInADVUnits() < GapAdvertisingParams::GAP_ADV_PARAMS_INTERVAL_MIN_NONCON) ||
            (params.getIntervalInADVUnits() > GapAdvertisingParams::GAP_ADV_PARAMS_INTERVAL_MAX))
        {
            return BLE_ERROR_PARAM_OUT_OF_RANGE;
        }
    }
    else
    {
        if ((params.getIntervalInADVUnits() < GapAdvertisingParams::GAP_ADV_PARAMS_INTERVAL_MIN) ||
            (params.getIntervalInADVUnits() > GapAdvertisingParams::GAP_ADV_PARAMS_INTERVAL_MAX))
        {
            return BLE_ERROR_PARAM_OUT_OF_RANGE;
        }
    }

    /* Check timeout is zero for Connectable Directed */
    if ((params.getAdvertisingType() == GapAdvertisingParams::ADV_CONNECTABLE_DIRECTED) && (params.getTimeout() != 0))
    {
        /* Timeout must be 0 with this type, although we'll never get here */
        /* since this isn't implemented yet anyway */
        return BLE_ERROR_PARAM_OUT_OF_RANGE;
    }

    /* Check timeout for other advertising types */
    if ((params.getAdvertisingType() != GapAdvertisingParams::ADV_CONNECTABLE_DIRECTED) &&
        (params.getTimeout() > GapAdvertisingParams::GAP_ADV_PARAMS_TIMEOUT_MAX))
    {
        return BLE_ERROR_PARAM_OUT_OF_RANGE;
    }

    switch (params.getAdvertisingType())
    {
        case GapAdvertisingParams::ADV_CONNECTABLE_UNDIRECTED:
            advParams.advertisingType = gAdvConnectableUndirected_c;
            break;
        case GapAdvertisingParams::ADV_CONNECTABLE_DIRECTED:
            advParams.advertisingType = gAdvDirectedHighDutyCycle_c;
            break;
        case GapAdvertisingParams::ADV_SCANNABLE_UNDIRECTED:
            advParams.advertisingType = gAdvScannable_c;
            break;
        case GapAdvertisingParams::ADV_NON_CONNECTABLE_UNDIRECTED:
            advParams.advertisingType = gAdvNonConnectable_c;
            break;
        default:
            break;
    }

    advParams.ownAddressType = gBleAddrTypePublic_c;
    advParams.peerAddressType = gBleAddrTypePublic_c;
    memset(advParams.peerAddress, 0, 6);
    advParams.channelMap = (gapAdvertisingChannelMapFlags_t)(gGapAdvertisingChannelMapDefault_c);
    advParams.filterPolicy = (gapAdvertisingFilterPolicy_t)advertisingPolicyMode;

    advParams.minInterval = params.getInterval();
    advParams.maxInterval = params.getInterval();
    advTimeout = gFastConnAdvTime_c;
    Gap_SetAdvertisingParameters(&advParams);

    gapReqCompleteFlags |= gSetAdvParamReq;
    do
    {
        /* Run Scheduler to get the response */
        OSA_Start();
    } while (gapReqCompleteFlags & gSetAdvParamReq);

    App_StartAdvertising(nxpGap::AdvertisingCallback, nxpGap::ConnectionCallback);
    gapReqCompleteFlags |= gStartAdvReq;
    do
    {
        /* Run Scheduler to get the response */
        OSA_Start();
    } while (gapReqCompleteFlags & gStartAdvReq);

    return BLE_ERROR_NONE;
}

ble_error_t nxpGap::startRadioScan(const GapScanningParams &scanningParams)
{
    gapScanningParameters_t scanParams;

    if (scanningParams.getActiveScanning())
    {
        scanParams.type = gScanTypeActive_c;
    }
    else
    {
        scanParams.type = gScanTypePassive_c;
    }
    scanParams.interval = scanningParams.getInterval();
    scanParams.window = scanningParams.getWindow();
    scanParams.filterPolicy = (bleScanningFilterPolicy_t)scanningPolicyMode;
    scanParams.ownAddressType = gBleAddrTypePublic_c;

    App_StartScanning(&scanParams, ScanningCallback);
    return BLE_ERROR_NONE;
}

/************************************************
 *  Stop scan function
 ***********************************************/

ble_error_t nxpGap::stopScan(void)
{
    Gap_StopScanning();
    gapReqCompleteFlags |= gStopScanReq;
    do
    {
        /* Run Scheduler to get the response */
        OSA_Start();
    } while (gapReqCompleteFlags & gStopScanReq);
    return BLE_ERROR_NONE;
}

/**************************************************************************/
/*!
    @brief  Stops the BLE HW and disconnects from any devices

    @returns    ble_error_t

    @retval     BLE_ERROR_NONE
                Everything executed properly

    @section EXAMPLE

    @code

    @endcode
*/
/**************************************************************************/
ble_error_t nxpGap::stopAdvertising(void)
{
    /* Stop Advertising */
    // FIXME - Map error codes
    Gap_StopAdvertising();

    return BLE_ERROR_NONE;
}

/*********************************************************************
 *
 *********************************************************************/
ble_error_t nxpGap::connect(const BLEProtocol::AddressBytes_t peerAddr,
                            BLEProtocol::AddressType_t peerAddrType,
                            const ConnectionParams_t *connectionParams,
                            const GapScanningParams *scanParamsIn)
{
    gapConnectionRequestParameters_t connParams;
    connParams.connIntervalMin = connectionParams->minConnectionInterval;
    connParams.connIntervalMax = connectionParams->maxConnectionInterval;
    connParams.connLatency = connectionParams->slaveLatency;
    connParams.supervisionTimeout = connectionParams->connectionSupervisionTimeout;
    connParams.connEventLengthMin = gGapConnEventLengthMin_d;
    connParams.connEventLengthMax = gGapConnEventLengthMax_d;

    connParams.filterPolicy = (bleInitiatorFilterPolicy_t)initiatorPolicyMode;
    memcpy(connParams.peerAddress, peerAddr, gcBleDeviceAddressSize_c);
    connParams.peerAddressType = (bleAddressType_t)peerAddrType;
    connParams.ownAddressType = gBleAddrTypePublic_c;

    connParams.scanInterval = scanParamsIn->getInterval();
    connParams.scanWindow = scanParamsIn->getWindow();
    connParams.usePeerIdentityAddress = FALSE;

    memcpy(peerAddress, peerAddr, gcBleDeviceAddressSize_c);
    connectInitiated = TRUE;

    App_Connect(&connParams, ConnectionCallback);

    return BLE_ERROR_NONE;
}

ble_error_t nxpGap::disconnect(Handle_t connectionHandle, DisconnectionReason_t reason)
{
    Gap_Disconnect(connectionHandle);
    return BLE_ERROR_NONE;
}

/*!
    @brief  Disconnects if we are connected to a central device

    @returns    ble_error_t

    @retval     BLE_ERROR_NONE
                Everything executed properly
*/
ble_error_t nxpGap::disconnect(DisconnectionReason_t reason)
{
    Gap_Disconnect(0);
    return BLE_ERROR_NONE;
}

ble_error_t nxpGap::getPreferredConnectionParams(ConnectionParams_t *params)
{
    bleResult_t result;
    uint16_t length;
    if ((BLE::Instance(BLE::DEFAULT_INSTANCE)).hasInitialized() == false)
    {
        return BLE_ERROR_INITIALIZATION_INCOMPLETE;
    }
    result = GattDb_ReadAttribute(gapHandles.charPpcpHandle, sizeof(ConnectionParams_t), (uint8_t *)params, &length);
    if (length != sizeof(ConnectionParams_t))
    {
        return BLE_ERROR_INTERNAL_STACK_FAILURE;
    }
    // FIXME - Map Error codes, this is a general FIXME to do
    return (ble_error_t)result;
}

ble_error_t nxpGap::setPreferredConnectionParams(const ConnectionParams_t *params)
{
    bleResult_t result;
    if ((BLE::Instance(BLE::DEFAULT_INSTANCE)).hasInitialized() == false)
    {
        return BLE_ERROR_INITIALIZATION_INCOMPLETE;
    }
    result = GattDb_WriteAttribute(gapHandles.charPpcpHandle, sizeof(ConnectionParams_t), (uint8_t *)params);
    return (ble_error_t)result;
}

ble_error_t nxpGap::updateConnectionParams(Handle_t handle, const ConnectionParams_t *newParams)
{
    bleResult_t result;
    if (newParams)
    {
        result = Gap_UpdateConnectionParameters(
            handle, newParams->minConnectionInterval, newParams->maxConnectionInterval, newParams->slaveLatency,
            newParams->connectionSupervisionTimeout, gConnUpdateIntervalMin_d, gConnUpdateIntervalMax_d);
    }
    else
    {
        ConnectionParams_t params;
        uint16_t length;
        result =
            GattDb_ReadAttribute(gapHandles.charPpcpHandle, sizeof(ConnectionParams_t), (uint8_t *)&params, &length);
        if ((result == gBleSuccess_c) && (length == sizeof(ConnectionParams_t)))
        {
            result = Gap_UpdateConnectionParameters(handle, params.minConnectionInterval, params.maxConnectionInterval,
                                                    params.slaveLatency, params.connectionSupervisionTimeout,
                                                    gConnUpdateIntervalMin_d, gConnUpdateIntervalMax_d);
        }
    }
    return (ble_error_t)result;
}

/**************************************************************************/
/*!
    @brief  Clear nxpGap's state.

    @returns    ble_error_t

    @retval     BLE_ERROR_NONE
                Everything executed properly
*/
/**************************************************************************/
ble_error_t nxpGap::reset(void)
{
    /* Clear all state that is from the parent, including private members */
    if (Gap::reset() != BLE_ERROR_NONE)
    {
        return BLE_ERROR_INVALID_STATE;
    }

    /* Clear derived class members */
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

    /* Set the whitelist policy filter modes to IGNORE_WHITELIST */
    advertisingPolicyMode = Gap::ADV_POLICY_IGNORE_WHITELIST;
    scanningPolicyMode = Gap::SCAN_POLICY_IGNORE_WHITELIST;
    initiatorPolicyMode = Gap::INIT_POLICY_IGNORE_WHITELIST;

    /* Clear the internal whitelist */
    whitelistAddressesSize = 0;

    return BLE_ERROR_NONE;
}

/**************************************************************************/
/*!
    @brief      Sets the BLE device address

    @returns    ble_error_t

    @section EXAMPLE

    @code

    uint8_t device_address[6] = { 0xca, 0xfe, 0xf0, 0xf0, 0xf0, 0xf0 };
    nxpGap.getGap().setAddress(Gap::BLEProtocol::AddressType::RANDOM_STATIC, device_address);

    @endcode
*/
/**************************************************************************/
ble_error_t nxpGap::setAddress(AddressType_t type, const Address_t address)
{
    bleDeviceAddress_t addr;
    if (type == BLEProtocol::AddressType::PUBLIC)
    {
        return BLE_ERROR_OPERATION_NOT_PERMITTED;
    }
    memcpy(addr, address, gcBleDeviceAddressSize_c);
    switch (type)
    {
        case BLEProtocol::AddressType::RANDOM_STATIC:
            addr[5] |= 0xC0;
            break;
        case BLEProtocol::AddressType::RANDOM_PRIVATE_NON_RESOLVABLE:
            addr[5] &= ~0xC0;
            break;
        case BLEProtocol::AddressType::RANDOM_PRIVATE_RESOLVABLE:
            addr[5] &= ~0xC0;
            addr[5] |= 0x40;
            break;
    }
    Gap_SetRandomAddress(addr);
    gapReqCompleteFlags |= gSetBleAddrReq;
    do
    {
        OSA_Start();
    } while (gapReqCompleteFlags & gSetBleAddrReq);
    Gap_EnableHostPrivacy(TRUE, gSmpKeys.aIrk);
    return BLE_ERROR_NONE;
}

/*
*  Device Address in from the stack come in the handler so can not get it immediately
*/
ble_error_t nxpGap::getAddress(AddressType_t *typeP, Address_t address)
{
    Gap_ReadPublicDeviceAddress();
    gapReqCompleteFlags |= gReadBleAddrReq;
    do
    {
        OSA_Start();
    } while (gapReqCompleteFlags & gReadBleAddrReq);
    memcpy(address, blePubAddr, gcBleDeviceAddressSize_c);
    *typeP = BLEProtocol::AddressType::PUBLIC;
    return BLE_ERROR_NONE;
}

ble_error_t nxpGap::setDeviceName(const uint8_t *deviceName)
{
    bleResult_t result;
    uint8_t length = strlen((const char *)deviceName);
    if (length > 20)
    {
        return BLE_ERROR_PARAM_OUT_OF_RANGE;
    }
    if ((BLE::Instance(BLE::DEFAULT_INSTANCE)).hasInitialized() == false)
    {
        return BLE_ERROR_INITIALIZATION_INCOMPLETE;
    }
    result = GattDb_WriteAttribute(gapHandles.charDeviceNameHandle, length, (uint8_t *)deviceName);
    return (ble_error_t)result;
}

ble_error_t nxpGap::getDeviceName(uint8_t *deviceName, unsigned *lengthP)
{
    bleResult_t result;
    if ((BLE::Instance(BLE::DEFAULT_INSTANCE)).hasInitialized() == false)
    {
        return BLE_ERROR_INITIALIZATION_INCOMPLETE;
    }
    result = GattDb_ReadAttribute(gapHandles.charDeviceNameHandle, 20, deviceName, (uint16_t *)lengthP);
    return (ble_error_t)result;
}

ble_error_t nxpGap::setAppearance(GapAdvertisingData::Appearance appearance)
{
    bleResult_t result;
    if ((BLE::Instance(BLE::DEFAULT_INSTANCE)).hasInitialized() == false)
    {
        return BLE_ERROR_INITIALIZATION_INCOMPLETE;
    }
    result = GattDb_WriteAttribute(gapHandles.charAppearanceHandle, 2, (uint8_t *)&appearance);
    return (ble_error_t)result;
}

ble_error_t nxpGap::getAppearance(GapAdvertisingData::Appearance *appearanceP)
{
    bleResult_t result;
    uint16_t length;
    if ((BLE::Instance(BLE::DEFAULT_INSTANCE)).hasInitialized() == false)
    {
        return BLE_ERROR_INITIALIZATION_INCOMPLETE;
    }
    result = GattDb_ReadAttribute(gapHandles.charAppearanceHandle, 2, (uint8_t *)appearanceP, &length);
    if (length != 2)
    {
        return BLE_ERROR_INTERNAL_STACK_FAILURE;
    }
    return (ble_error_t)result;
}

// ble_error_t nxpGap::setTxPower(int8_t txPower)
//{
//    return BLE_ERROR_NONE;
//}

// void nxpGap::getPermittedTxPowerValues(const int8_t **valueArrayPP, size_t *countP)
//{
//
//}

/**************************************************************************/
/*!
    @brief  Get the capacity of the internal whitelist maintained by this
            implementation.

    @returns    The capacity of the internal whitelist.

    @section EXAMPLE

    @code

    @endcode
*/
/**************************************************************************/

uint8_t nxpGap::getMaxWhitelistSize(void) const
{
    return CFG_WHITELIST_MAX_SIZE;
}

/**************************************************************************/
/*!
    @brief  Get a copy of the implementation's internal whitelist.

    @param[out] whitelistOut
                A \ref Gap::Whitelist_t structure containing a copy of the
                addresses in the implemenetation's internal whitelist.

    @returns    \ref ble_errror_t

    @retval     BLE_ERROR_NONE
                Everything executed properly.

    @section EXAMPLE

    @code

    @endcode
*/
/**************************************************************************/
ble_error_t nxpGap::getWhitelist(Gap::Whitelist_t &whitelistOut) const
{
    uint8_t i;
    for (i = 0; i < whitelistAddressesSize && i < whitelistOut.capacity; ++i)
    {
        memcpy(&whitelistOut.addresses[i], &whitelistAddresses[i], sizeof(BLEProtocol::Address_t));
    }
    whitelistOut.size = i;

    return BLE_ERROR_NONE;
}
/**************************************************************************/
/*!
    @brief  Set the whitelist that will be used in the next call to
            startAdvertising().

    @param[in]  whitelistIn
                A reference to a \ref Gap::Whitelist_t structure
                representing a whitelist containing all the white listed
                BLE addresses.

    @returns    \ref ble_errror_t

    @retval     BLE_ERROR_NONE
                Everything executed properly.

                BLE_ERROR_INVALID_PARAM
                The supplied whitelist contains a private non-resolvable
                address

                BLE_ERROR_PARAM_OUT_OF_RANGE
                The size of the supplied whitelist exceeds the maximum
                capacity of the implementation's internal whitelist.

    @section EXAMPLE

    @code

    @endcode
*/
/**************************************************************************/
ble_error_t nxpGap::setWhitelist(const Gap::Whitelist_t &whitelistIn)
{
    if (whitelistIn.size > getMaxWhitelistSize())
    {
        return BLE_ERROR_PARAM_OUT_OF_RANGE;
    }

    /* Test for invalid parameters before we change the internal state */
    for (uint8_t i = 0; i < whitelistIn.size; ++i)
    {
        if (whitelistIn.addresses[i].type == BLEProtocol::AddressType::RANDOM_PRIVATE_NON_RESOLVABLE)
        {
            /* This is not allowed because it is completely meaningless */
            return BLE_ERROR_INVALID_PARAM;
        }
    }

    Gap_ClearWhiteList();
    gapReqCompleteFlags |= gClrWhiteListReq;
    do
    {
        OSA_Start();
    } while (gapReqCompleteFlags & gClrWhiteListReq);

    whitelistAddressesSize = 0;
    for (uint8_t i = 0; i < whitelistIn.size; ++i)
    {
        memcpy(&whitelistAddresses[whitelistAddressesSize], &whitelistIn.addresses[i], sizeof(BLEProtocol::Address_t));
        Gap_AddDeviceToWhiteList(whitelistAddresses[whitelistAddressesSize].idAddressType,
                                 whitelistAddresses[whitelistAddressesSize].idAddress);
        gapReqCompleteFlags |= gAddWhiteListReq;
        do
        {
            OSA_Start();
        } while (gapReqCompleteFlags & gAddWhiteListReq);
        whitelistAddressesSize++;
    }

    return BLE_ERROR_NONE;
}

/**************************************************************************/
/*!
    @brief  Set the advertising policy filter mode that will be used in
            the next call to startAdvertising().

    @returns    \ref ble_errror_t

    @retval     BLE_ERROR_NONE
                Everything executed properly.

                BLE_ERROR_NOT_IMPLEMENTED
                This feature is currently note implemented.

    @section EXAMPLE

    @code

    @endcode
*/
/**************************************************************************/
ble_error_t nxpGap::setAdvertisingPolicyMode(Gap::AdvertisingPolicyMode_t mode)
{
    advertisingPolicyMode = mode;
    return BLE_ERROR_NONE;
}

/**************************************************************************/
/*!
    @brief  Set the scanning policy filter mode that will be used in
            the next call to startAdvertising().

    @returns    \ref ble_errror_t

    @retval     BLE_ERROR_NONE
                Everything executed properly.

                BLE_ERROR_NOT_IMPLEMENTED
                This feature is currently note implemented.

    @section EXAMPLE

    @code

    @endcode
*/
/**************************************************************************/
ble_error_t nxpGap::setScanningPolicyMode(Gap::ScanningPolicyMode_t mode)
{
    scanningPolicyMode = mode;
    return BLE_ERROR_NONE;
}

/**************************************************************************/
/*!
    @brief  Set the initiator policy filter mode that will be used in
            the next call to startAdvertising()

    @returns    \ref ble_errror_t

    @retval     BLE_ERROR_NONE
                Everything executed properly.

                BLE_ERROR_NOT_IMPLEMENTED
                This feature is currently note implemented.

    @section EXAMPLE

    @code

    @endcode
*/
/**************************************************************************/
ble_error_t nxpGap::setInitiatorPolicyMode(Gap::InitiatorPolicyMode_t mode)
{
    initiatorPolicyMode = mode;
    return BLE_ERROR_NOT_IMPLEMENTED;
}

/**************************************************************************/
/*!
    @brief  Get the current advertising policy filter mode.

    @returns    The advertising policy filter mode.

    @section EXAMPLE

    @code

    @endcode
*/
/**************************************************************************/
Gap::AdvertisingPolicyMode_t nxpGap::getAdvertisingPolicyMode(void) const
{
    return advertisingPolicyMode;
}

/**************************************************************************/
/*!
    @brief  Get the current scanning policy filter mode.

    @returns    The scanning policy filter mode.

    @section EXAMPLE

    @code

    @endcode
*/
/**************************************************************************/
Gap::ScanningPolicyMode_t nxpGap::getScanningPolicyMode(void) const
{
    return scanningPolicyMode;
}

/**************************************************************************/
/*!
    @brief  Get the current initiator policy filter mode.

    @returns    The initiator policy filter mode.

    @note   Currently initiator filtering using the whitelist is not
            implemented in this module.

    @section EXAMPLE

    @code

    @endcode
*/
/**************************************************************************/
Gap::InitiatorPolicyMode_t nxpGap::getInitiatorPolicyMode(void) const
{
    return initiatorPolicyMode;
}

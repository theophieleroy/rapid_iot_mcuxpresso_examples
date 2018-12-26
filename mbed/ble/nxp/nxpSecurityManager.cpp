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

#include "nxpSecurityManager.h"
#include "gap_types.h"
#include "BLEProtocol.h"
#include "nxpGap.h"
#include "nxpInstance.h"

extern "C" {
#include "appble.h"
#include "gap_interface.h"
}

/*******************************************************************************
 * Variables
 ******************************************************************************/
uint32_t mPasskeyValue = 0;                                 /*static passkey*/
extern gapPairingParameters_t gPeripheralPairingParameters; /* Peripheral Pairing Parameters*/
static bool initialized = false;                            /* To indicate initialization of security*/
uint8_t keysNvmWritten = 0;                                 /* Flag for indicating keys written to NVM*/
/*******************************************************************************
 * Code
 ******************************************************************************/

/*!
 * @brief			Initialize security.
 * @param[in]  enableBonding : Allow for bonding.
 * @param[in]  requireMITM   : Require protection for man-in-the-middle attacks.
 * @param[in]  iocaps        : To specify the I/O capabilities of this peripheral,
 *                           	such as availability of a display or keyboard, to
 *                           	support out-of-band exchanges of security data.
 * @param[in]  passkey       : To specify a static passkey.
 *
 * @return BLE_ERROR_NONE on success.
 */
ble_error_t nxpSecurityManager::init(bool enableBonding,
                                     bool requireMITM,
                                     SecurityIOCapabilities_t iocaps,
                                     const Passkey_t passkey)
{
    int i;

    if (initialized)
    {
        return BLE_ERROR_NONE;
    }
    /* Update Pairing parameters IO caps */
    gPeripheralPairingParameters.localIoCapabilities = (gapIoCapabilities_t)iocaps;

    /* Update Pairing Parameter bonding */
    gPeripheralPairingParameters.withBonding = enableBonding;

    if (enableBonding)
    {
        if (requireMITM)
        {
            if (gPeripheralPairingParameters.leSecureConnectionSupported) /* Bonding :true , MITM:true , SC: true*/
            {
                gPeripheralPairingParameters.securityModeAndLevel =
                    gSecurityMode_1_Level_4_c; /*!< Mode 1 Level 4 - Encryption
                                        with LE Secure Connections
                                        pairing (BLE 4.2 only). */
            }
            else
            { /* Bonding : true,  MITM: true */
                gPeripheralPairingParameters.securityModeAndLevel =
                    gSecurityMode_1_Level_3_c; /*!< Mode 1 Level 3 - Encryption with authentication. */
            }
        }
        else /*  Bonding : true, MITM: false */
        {
            gPeripheralPairingParameters.securityModeAndLevel =
                gSecurityMode_1_Level_2_c; /*!< Mode 1 Level 2 - Encryption without authentication. */
        }
    }
    else /*  Bonding : false, MITM: false */
    {
        gPeripheralPairingParameters.securityModeAndLevel =
            gSecurityMode_1_Level_1_c; /*!< Mode 1 Level 1 - No Security. */
    }

    /* Register Initial Security */
    registerInitialSecurity();

    if (passkey != NULL)
    {
        for (i = 0; i < PASSKEY_LEN; i++)
        {
            mPasskeyValue = 10 * mPasskeyValue + passkey[i];
        }
    }

    /*Set the local passkey*/
    Gap_SetLocalPasskey(mPasskeyValue);

    initialized = true;

    return BLE_ERROR_NONE;
}

/*!
 * @brief			Get pointer to master security requirements
 */
gapSecurityRequirements_t *nxpSecurityManager::getMasterSecurityReq(void)
{
    return &masterSecurityReq;
}

/*!
 * @brief			Register Initial Device security.
 */
void nxpSecurityManager::registerInitialSecurity()
{
    /* Device Security Requirements */
    gapDeviceSecurityRequirements_t deviceSecurityRequirements = {
        .pMasterSecurityRequirements = NULL, .cNumServices = 0, .aServiceSecurityRequirements = NULL};

    nxpSecurityManager::masterSecurityReq.securityModeLevel = gPeripheralPairingParameters.securityModeAndLevel;
    nxpSecurityManager::masterSecurityReq.authorization = FALSE;
    nxpSecurityManager::masterSecurityReq.minimumEncryptionKeySize = gEncryptionKeySize_d;

    deviceSecurityRequirements.pMasterSecurityRequirements = nxpSecurityManager::getMasterSecurityReq();

    /* Register Device security requirements if pairing is used */
    Gap_RegisterDeviceSecurityRequirements(&deviceSecurityRequirements);
}

bool nxpSecurityManager::hasInitialized(void) const
{
    return initialized;
}

/*!
 * @brief Set the security mode on a connection. Useful for elevating the security mode
 * 				once certain conditions are met, e.g., a particular service is found.
 *
 * @param[in]  connectionHandle :  Handle to identify the connection.
 * @param[in]  securityMode     :  Requested security mode.
 *
 * @return BLE_ERROR_NONE or appropriate error code indicating the failure reason.
 */
ble_error_t nxpSecurityManager::setLinkSecurity(Gap::Handle_t connectionHandle, SecurityMode_t securityMode)
{
    /** FIXME: */
    bleResult_t result;

    switch (securityMode)
    {
        case SecurityManager::SECURITY_MODE_ENCRYPTION_OPEN_LINK:
            /**< Require no protection, open link. */
            gPeripheralPairingParameters.withBonding = false;
            gPeripheralPairingParameters.securityModeAndLevel = gSecurityMode_1_Level_1_c;
            break;

        case SecurityManager::SECURITY_MODE_ENCRYPTION_NO_MITM:
            /**< Require encryption, but no MITM protection. */
            gPeripheralPairingParameters.withBonding = true;
            gPeripheralPairingParameters.securityModeAndLevel = gSecurityMode_1_Level_2_c;
            break;

        case SecurityManager::SECURITY_MODE_ENCRYPTION_WITH_MITM:
            /**< Require encryption and MITM protection. */
            gPeripheralPairingParameters.withBonding = true;
            gPeripheralPairingParameters.securityModeAndLevel = gSecurityMode_1_Level_3_c;
            break;

        case SecurityManager::SECURITY_MODE_SIGNED_NO_MITM:
            /**< Require signing or encryption, but no MITM protection. */
            gPeripheralPairingParameters.withBonding = true;
            gPeripheralPairingParameters.securityModeAndLevel = gSecurityMode_2_Level_1_c;
            break;
        case SecurityManager::SECURITY_MODE_SIGNED_WITH_MITM:
            /**< Require signing or encryption, and MITM protection. */
            gPeripheralPairingParameters.withBonding = true;
            gPeripheralPairingParameters.securityModeAndLevel = gSecurityMode_2_Level_2_c;
            break;
        default:
            return BLE_ERROR_NOT_IMPLEMENTED;
    }

    if (((nxpGap &)(BLE::Instance(BLE::DEFAULT_INSTANCE).gap())).getDeviceRole(connectionHandle) == gGapPeripheral_c)
    {
        result = Gap_SendSlaveSecurityRequest(connectionHandle, gPeripheralPairingParameters.withBonding,
                                              gPeripheralPairingParameters.securityModeAndLevel);
    }

    if (result == gBleSuccess_c)
    {
        return BLE_ERROR_NONE;
    }
    else
    {
        return BLE_ERROR_UNSPECIFIED;
    }
}

/*!
 * @brief Get the security status of a connection.
 *
 * @param[in]  connectionHandle  : Handle to identify the connection.
 * @param[out] securityStatusP   : Security status.
 *
 * @return BLE_ERROR_NONE or appropriate error code indicating the failure reason.
 */

ble_error_t nxpSecurityManager::getLinkSecurity(Gap::Handle_t connectionHandle, LinkSecurityStatus_t *securityStatusP)
{
    if (mLinkSecurityState[connectionHandle] == kEncrypted)
    {
        *securityStatusP = ENCRYPTED;
        return BLE_ERROR_NONE;
    }
    else if (mLinkSecurityState[connectionHandle] == kNotEncrypted)
    {
        *securityStatusP = NOT_ENCRYPTED;
        return BLE_ERROR_NONE;
    }
    else if (mLinkSecurityState[connectionHandle] == kEncryptionInProgress)
    {
        *securityStatusP = ENCRYPTION_IN_PROGRESS;
        return BLE_ERROR_NONE;
    }

    return BLE_ERROR_INVALID_PARAM;
}

/*!
 * @brief		Get a list of addresses from all peers in the bond table.
 *
 * @param[in,out]   : addresses
 *                  	(on input) addresses.capacity contains the maximum
 *                  	number of addresses to be returned.
 *                  	(on output) The populated table with copies of the
 *                  	addresses in the implementation's whitelist.
 *
 * @retval BLE_ERROR_NONE           :  On success, else an error code indicating reason for failure.
 * @retval BLE_ERROR_INVALID_STATE  :  If the API is called without module initialization or
 *                                     application registration.
 *
 * @experimental
 */
ble_error_t nxpSecurityManager::getAddressesFromBondTable(Gap::Whitelist_t &addresses) const
{
    gapIdentityInformation_t aOutBondedAddresses[gMaxBondedDevices_c];
    uint8_t bondedDevicesCount = 0;
    /** FIXME: */
    bleResult_t result =
        Gap_GetBondedDevicesIdentityInformation(aOutBondedAddresses, addresses.capacity, &bondedDevicesCount);

    if (result == gBleSuccess_c)
    {
        addresses.size = bondedDevicesCount;

        for (uint8_t i = 0; i < bondedDevicesCount; i++)
        {
            addresses.addresses[i].type =
                (BLEProtocol::AddressType_t)aOutBondedAddresses[i].identityAddress.idAddressType;
            memcpy(&addresses.addresses[i].address, &aOutBondedAddresses[i].identityAddress.idAddress,
                   sizeof(BLEProtocol::AddressBytes_t));
        }
        return BLE_ERROR_NONE;
    }
    else
    {
        return BLE_ERROR_INVALID_STATE;
    }
}

/*!
 * @brief  Delete all peer device context and all related bonding information from
 * 				 the database within the security manager.
 *
 * @retval BLE_ERROR_NONE            : On success, else an error code indicating reason for failure.
 * @retval BLE_ERROR_INVALID_STATE   : If the API is called without module initialization or
 *                                     application registration.
 */
ble_error_t nxpSecurityManager::purgeAllBondingState(void)
{
    /*FIXME */
    bleResult_t result;

    result = Gap_RemoveAllBonds();

    if (result == gBleSuccess_c)
        return BLE_ERROR_NONE;
    else
        return BLE_ERROR_INVALID_STATE;
}

ble_error_t nxpSecurityManager::reset(void)
{
    bleResult_t result;

    if (SecurityManager::reset() != BLE_ERROR_NONE)
    {
        return BLE_ERROR_INVALID_STATE;
    }

    result = Gap_ClearWhiteList();

    if (result == gBleSuccess_c)
        return BLE_ERROR_NONE;
    else
        return BLE_ERROR_INVALID_STATE;
}

extern "C" void ProcessSecSetupInitiatedEvent(deviceId_t peerDeviceId,
                                              gapPairingParameters_t gPeripheralPairingParameters)
{
    bool allowBonding;
    bool requireMITM;
    allowBonding = gPeripheralPairingParameters.withBonding;

    switch (gPeripheralPairingParameters.securityModeAndLevel)
    {
        case gSecurityMode_1_Level_1_c:
        case gSecurityMode_1_Level_2_c:
        case gSecurityMode_2_Level_1_c:
        {
            requireMITM = false;
        }
        break;
        case gSecurityMode_1_Level_3_c:
        case gSecurityMode_1_Level_4_c:
        case gSecurityMode_2_Level_2_c:
        {
            requireMITM = true;
        }
        break;
    }

    (BLE::Instance(BLE::DEFAULT_INSTANCE).securityManager())
        .processSecuritySetupInitiatedEvent(
            (Gap::Handle_t)peerDeviceId, allowBonding, requireMITM,
            SecurityManager::SecurityIOCapabilities_t(gPeripheralPairingParameters.localIoCapabilities));
}

void ProcessLinkSecureEvent(deviceId_t peerDeviceId, gapSecurityModeAndLevel_t securityModeAndLevel)
{
    SecurityManager::SecurityMode_t securityMode;

    switch (securityModeAndLevel)
    {
        case gSecurityMode_1_Level_1_c:
            securityMode = SecurityManager::SECURITY_MODE_ENCRYPTION_OPEN_LINK;
            break;
        case gSecurityMode_1_Level_2_c:
            securityMode = SecurityManager::SECURITY_MODE_ENCRYPTION_NO_MITM;
            break;
        case gSecurityMode_1_Level_3_c:
            securityMode = SecurityManager::SECURITY_MODE_ENCRYPTION_WITH_MITM;
            break;
        case gSecurityMode_2_Level_1_c:
            securityMode = SecurityManager::SECURITY_MODE_SIGNED_NO_MITM;
            break;
        case gSecurityMode_2_Level_2_c:
            securityMode = SecurityManager::SECURITY_MODE_SIGNED_WITH_MITM;
            break;
    }

    (BLE::Instance(BLE::DEFAULT_INSTANCE).securityManager())
        .processLinkSecuredEvent((Gap::Handle_t)peerDeviceId, securityMode);
}

void nxpSecurityManager::ProcessPasskeyDispEvent(deviceId_t peerDeviceId, uint32_t passkey)
{
    SecurityManager::Passkey_t Passkey;
    uint32_t tempPassKey = passkey;
    /* Coverting integer to an array*/
    for (uint8_t i = 0; i < SecurityManager::PASSKEY_LEN; i++)
    {
        Passkey[i] = tempPassKey % 10;
        tempPassKey = tempPassKey / 10;
    }

    (BLE::Instance(BLE::DEFAULT_INSTANCE).securityManager())
        .processPasskeyDisplayEvent((Gap::Handle_t)peerDeviceId, Passkey);
}

extern "C" void ProcessSecContextStoredEvent(deviceId_t peerDeviceId)
{
    (BLE::Instance(BLE::DEFAULT_INSTANCE).securityManager())
        .processSecurityContextStoredEvent((Gap::Handle_t)peerDeviceId);
}

void nxpSecurityManager::ProcessSecSetupCompletedEvent(deviceId_t peerDeviceId, gapConnectionEvent_t *pConnectionEvent)
{
    SecurityCompletionStatus_t status = SEC_STATUS_SUCCESS;

    switch (pConnectionEvent->eventType)
    {
        case gConnEvtAuthenticationRejected_c:
        {
            switch (pConnectionEvent->eventData.authenticationRejectedEvent.rejectReason)
            {
                case gLinkEncryptionFailed_c:
                {
                    status = SEC_STATUS_UNSPECIFIED;
                }
                break;

                case gOobNotAvailable_c:
                {
                    status = SEC_STATUS_OOB_NOT_AVAILABLE;
                }
                break;

                case gIncompatibleIoCapabilities_c:
                {
                    status = SEC_STATUS_AUTH_REQ;
                }
                break;

                case gPairingNotSupported_c:
                {
                    status = SEC_STATUS_PAIRING_NOT_SUPP;
                }
                break;

                case gLowEncryptionKeySize_c:
                {
                    status = SEC_STATUS_ENC_KEY_SIZE;
                }
                break;

                case gRepeatedAttempts_c:
                {
                    status = SEC_STATUS_REPEATED_ATTEMPTS;
                }
                break;

                case gUnspecifiedReason_c:
                {
                    status = SEC_STATUS_UNSPECIFIED;
                }
                break;

                case gBleSuccess_c:
                {
                    status = SEC_STATUS_SUCCESS;
                }
                break;

                default:
                {
                    status = SEC_STATUS_UNSPECIFIED;
                }
                break;
            } /*end inner switch case */
        }
        break;

        case gConnEvtPairingComplete_c:
        {
            switch (pConnectionEvent->eventData.pairingCompleteEvent.pairingCompleteData.failReason)
            {
                case gSmSuccess_c:
                // FIXME - Should be removed after the stack is fixed
                case gSmNullCBFunction_c:
                {
                    status = SEC_STATUS_SUCCESS;
                }
                break;

                case gBleInvalidParameter_c:
                case gBleUnavailable_c:
                case gSmInvalidCommandParameter_c:
                case gSmInvalidDeviceId_c:
                case gSmInvalidConnectionHandle_c:
                case gSmUnknownSmpPacketType_c:
                case gSmInvalidSmpPacketLength_c:
                case gSmInvalidSmpPacketParameter_c:
                case gSmInvalidHciEventParameter_c:
                case gSmPairingErrorInvalidParameters_c:
                {
                    status = SEC_STATUS_INVALID_PARAMS;
                }
                break;

                case gBleFeatureNotSupported_c:
                case gSmCommandNotSupported_c:
                case gSmUnexpectedCommand_c:
                case gSmInvalidCommandCode_c:
                case gSmInvalidCommandLength_c:
                case gSmPairingErrorCommandNotSupported_c:
                {
                    status = SEC_STATUS_SMP_CMD_UNSUPPORTED;
                }
                break;

                case gSmInproperKeyDistributionField_c:
                case gSmUnexpectedKeyType_c:
                case gSmUnexpectedKeyset_c:
                case gSmUnattainableLocalDeviceSecRequirements_c:
                case gSmUnattainableSlaveSecReqRequirements_c:
                {
                    status = SEC_STATUS_AUTH_REQ;
                }
                break;

                case gSmSmpTimeoutOccurred_c:
                case gSmSmpPacketReceivedAfterTimeoutOccurred_c:
                {
                    status = SEC_STATUS_TIMEOUT;
                }
                break;

                case gSmOobDataAddressMismatch_c:
                {
                    status = SEC_STATUS_OOB_NOT_AVAILABLE;
                }
                break;

                case gSmUnattainableLocalDeviceMinKeySize_c:
                {
                    status = SEC_STATUS_ENC_KEY_SIZE;
                }
                break;

                case gSmPairingErrorPasskeyEntryFailed_c:
                {
                    status = SEC_STATUS_PASSKEY_ENTRY_FAILED;
                }
                break;

                case gSmPairingErrorConfirmValueFailed_c:
                {
                    status = SEC_STATUS_CONFIRM_VALUE;
                }
                break;

                default:
                {
                    status = SEC_STATUS_UNSPECIFIED;
                }
                break;
            } /*end inner switch case */
        }
        break;

    } /* end outer switch case */

    (BLE::Instance(BLE::DEFAULT_INSTANCE).securityManager())
        .processSecuritySetupCompletedEvent((Gap::Handle_t)peerDeviceId, status);
}

void nxpSecurityManager::ProcessSecurityEvent(deviceId_t peerDeviceId, gapConnectionEvent_t *pConnectionEvent)
{
    switch (pConnectionEvent->eventType)
    {
        case gConnEvtPairingRequest_c:
        {
            /*If pairing enabled*/
            if (gPeripheralPairingParameters.securityModeAndLevel != gSecurityMode_1_Level_1_c)
            {
                gPeripheralPairingParameters.centralKeys = pConnectionEvent->eventData.pairingEvent.centralKeys;
                Gap_AcceptPairingRequest(peerDeviceId, &gPeripheralPairingParameters);

                /* Set the keysNvmWritten flag to 0*/
                keysNvmWritten = 0;
                /* Call  ProcessSecurityInitiatedEvent */
                ProcessSecSetupInitiatedEvent(peerDeviceId, gPeripheralPairingParameters);
                /* Update Link security state of a connection handle */
                mLinkSecurityState[peerDeviceId] = kEncryptionInProgress;
            }
            else
            {
                Gap_RejectPairing(peerDeviceId, gPairingNotSupported_c);
            }
        }
        break;

        case gConnEvtEncryptionChanged_c:
        {
            if (pConnectionEvent->eventData.encryptionChangedEvent.newEncryptionState)
            {
                mLinkSecurityState[peerDeviceId] = kEncrypted; /* Update Link security state of a connection handle */
                ProcessLinkSecureEvent(peerDeviceId, gPeripheralPairingParameters.securityModeAndLevel);
            }
            else
            {
                mLinkSecurityState[peerDeviceId] =
                    kNotEncrypted; /* Update Link security state of a connection handle */
            }
        }
        break;

        case gConnEvtPasskeyDisplay_c:
        {
            ProcessPasskeyDispEvent(peerDeviceId, pConnectionEvent->eventData.passkeyForDisplay);
        }
        break;
    }
}

nxpSecurityManager::LinkSecurityState_t nxpSecurityManager::getLinkSecurityState(deviceId_t peerDeviceId)
{
    return mLinkSecurityState[peerDeviceId];
}

void nxpSecurityManager::setLinkSecurityState(deviceId_t peerDeviceId,
                                              nxpSecurityManager::LinkSecurityState_t linkSecState)
{
    mLinkSecurityState[peerDeviceId] = linkSecState;
}

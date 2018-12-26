/*!
 * Copyright 2017 NXP
 *
 * \file
 *
 * This file is the source file for the GATTDB Shell module
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
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

/************************************************************************************
*************************************************************************************
* Include
*************************************************************************************
************************************************************************************/
#include "FunctionLib.h"
#include "FsciInterface.h"

#include "shell.h"
#include "cmd_ble.h"
#include "ble_sig_defines.h"
#include "ble_shell.h"
#include "ble_otap_client.h"

#include "shell_gap.h"
#include "shell_gatt.h"
#include "GUI.h"

#include "otap_interface.h"

// GAP extern
extern uint16_t                                     gLatestHandle;
extern uint16_t                                     gLatestUuid;
extern char                                         gDeviceName[];
extern GAPScanningEventDeviceScannedIndication_t    gScannedDevices[];
extern uint8_t                                      gScannedDevicesCount;
extern deviceId_t                                   gPeerDeviceId;
extern GAPPairRequest_t                             gPairingParameters;
extern GAPAddDeviceToWhiteListRequest_AddressType_t mPeerDeviceAddressType;
extern uint8_t maPeerDeviceAddress[6];

// GATT extern
extern uint16_t gCccdHandle;
extern bool_t gIsNotificationActive;
extern bool_t gIsIndicationActive;

// RPK extern
extern enum RpkDemoState_t gRpkDemoState;

// OTAP extern
extern enum otapState_t gOtapState;

// GATT static
static uint8_t mCurrentServiceInDiscoveryIndex = 0;
static uint8_t mCurrentCharInDiscoveryIndex = 0;
static GATTClientProcedureDiscoverAllPrimaryServicesIndication_t mAllPrimaryServices;
static GATTClientProcedureDiscoverAllCharacteristicsIndication_t mAllChars;

uint8_t bleAddress[6] = {0x00};

// internals
bool_t mSuppressBleEventPrint = FALSE;

/*!*************************************************************************************************
\fn     static void SHELL_PrintBuff(uint8_t *buff, uint32_t length)
\brief  Hexlify a byte array

\param  [in]    buff      data
\param  [in]    length    length of data
***************************************************************************************************/
static void SHELL_PrintBuff
(
    uint8_t *buff,
    uint32_t length
)
{
    uint32_t i;

    for (i = 0; i < length; i++)
    {
        shell_printf("%02X", buff[i]);
    }
}

/*!*************************************************************************************************
\fn     static void SHELL_PrintBuffReversed(uint8_t *buff, uint32_t length)
\brief  Hexlify a byte array, reversed

\param  [in]    buff      data
\param  [in]    length    length of data
***************************************************************************************************/
static void SHELL_PrintBuffReversed
(
    uint8_t *buff,
    uint32_t length
)
{
    for (int i = length - 1; i >= 0; i--)
    {
        shell_printf("%02X", buff[i]);
    }
}


static void SHELL_GattDb_PrintPrimaryService
(
    GATTDBDynamicAddPrimaryServiceDeclarationIndication_t *evt
)
{
    shell_printf("\r\n\tService Handle --> %d\r\n", evt->ServiceHandle);
}

static void SHELL_GattDb_PrintReadAttr
(
    GATTDBReadAttributeIndication_t *evt
)
{
    shell_printf("\r\n\tRead Attribute Indication --> 0x");
    SHELL_PrintBuffReversed(evt->Value, evt->ValueLength);
    SHELL_NEWLINE();

    if (gLatestUuid == gBleSig_GapDeviceName_d)
    {
        FLib_MemCpy(gDeviceName, evt->Value, evt->ValueLength);
        gDeviceName[evt->ValueLength] = '\0';
    }

    MEM_BufferFree(evt->Value);
}

static void SHELL_GattDb_PrintChar
(
    GATTDBDynamicAddCharacteristicDeclarationAndValueIndication_t *evt
)
{
    shell_printf("\r\n\tCharacteristic Handle --> %d\r\n", evt->CharacteristicHandle);
}

static void SHELL_GattDb_PrintCharDesc
(
    GATTDBDynamicAddCharacteristicDescriptorIndication_t *evt
)
{
    shell_printf("\r\n\tCharacteristic Descriptor Handle --> %d\r\n", evt->DescriptorHandle);
}

static void SHELL_GattDb_PrintCCCD
(
    GATTDBDynamicAddCccdIndication_t *evt
)
{
    shell_printf("\r\n\tCCCD Handle --> %d\r\n", evt->CCCDHandle);
}

static void SHELL_GapPrintPublicAddr
(
    GAPGenericEventPublicAddressReadIndication_t *evt
)
{
    shell_printf("\r\n\tPublic Address --> 0x");
    SHELL_PrintBuffReversed(evt->Address, sizeof(evt->Address));
    SHELL_NEWLINE();
    FLib_MemCpy(bleAddress, evt->Address, sizeof(evt->Address));
}

static void SHELL_GapPrintDeviceScannedIndication
(
    GAPScanningEventDeviceScannedIndication_t *evt
)
{
    shell_printf("\r\nScanned Device ID --> %d; use this ID as argument to gap connect", gScannedDevicesCount);
    shell_printf("\r\nAddress --> 0x");
    SHELL_PrintBuff(evt->Address, sizeof(evt->Address));
    shell_printf("\r\nRSSI --> %d", evt->Rssi);
    shell_printf("\r\nData --> 0x");
    SHELL_PrintBuff(evt->Data, evt->DataLength);
    shell_printf("\r\nDirect RPA Used --> %d", evt->DirectRpaUsed);

    if (evt->DirectRpaUsed)
    {
        shell_printf("\r\nDirect RPA --> 0x");
        SHELL_PrintBuff(evt->DirectRpa, sizeof(evt->DirectRpa));
    }

    shell_printf("\r\nAdvertising Address Resolved --> %d", evt->advertisingAddressResolved);
    SHELL_NEWLINE();

    if (gScannedDevicesCount == mShellGapMaxScannedDevicesCount_c)
    {
        shell_write("\r\nNo more memory to store another scanned device!");
    }
    else
    {
        FLib_MemCpy(&gScannedDevices[gScannedDevicesCount++], evt, sizeof(GAPScanningEventDeviceScannedIndication_t));
    }

    MEM_BufferFree(evt->Data);
}

#include "TimersManager.h"
static tmrTimerID_t     mDelayTimerID = gTmrInvalidTimerID_c;

static void SHELL_GapAcceptPairingRequest
(
    void *param  //GAPConnectionEventPairingRequestIndication_t *evt
)
{
    uint16_t DeviceId = (uint16_t)((uint32_t)param);
    GAPAcceptPairingRequestRequest_t req;

    req.DeviceId = DeviceId;
    FLib_MemCpy(&req.PairingParameters, &gPairingParameters.PairingParameters, sizeof(req.PairingParameters));

    GAPAcceptPairingRequestRequest(&req, BLE_FSCI_IF);
}


static void SHELL_GapDiscoverAllCharsOfService
(
    void *param
)
{
    uint16_t uuid16;

    SHELL_NEWLINE();

    if (mCurrentServiceInDiscoveryIndex < mAllPrimaryServices.NbOfDiscoveredServices)
    {
        uuid16 = mAllPrimaryServices.DiscoveredServices[mCurrentServiceInDiscoveryIndex].Uuid.Uuid16Bits[0] +
                 (mAllPrimaryServices.DiscoveredServices[mCurrentServiceInDiscoveryIndex].Uuid.Uuid16Bits[1] << 8);

        shell_printf("\n\r--> %s Start Handle: %d End Handle: %d\n\r",
                     ShellGatt_GetServiceName(uuid16),
                     mAllPrimaryServices.DiscoveredServices[mCurrentServiceInDiscoveryIndex].StartHandle,
                     mAllPrimaryServices.DiscoveredServices[mCurrentServiceInDiscoveryIndex].EndHandle);

        GATTClientDiscoverAllCharacteristicsOfServiceRequest_t req = { 0 };
        req.DeviceId = gPeerDeviceId;
        req.Service.StartHandle = mAllPrimaryServices.DiscoveredServices[mCurrentServiceInDiscoveryIndex].StartHandle;
        req.Service.EndHandle = mAllPrimaryServices.DiscoveredServices[mCurrentServiceInDiscoveryIndex].EndHandle;
        req.Service.UuidType = Uuid16Bits;
        req.Service.Uuid.Uuid16Bits[0] = mAllPrimaryServices.DiscoveredServices[mCurrentServiceInDiscoveryIndex].Uuid.Uuid16Bits[0];
        req.Service.Uuid.Uuid16Bits[1] = mAllPrimaryServices.DiscoveredServices[mCurrentServiceInDiscoveryIndex].Uuid.Uuid16Bits[1];
        req.MaxNbOfCharacteristics = mMaxServiceCharCount_d;
        GATTClientDiscoverAllCharacteristicsOfServiceRequest(&req, BLE_FSCI_IF);

        mCurrentServiceInDiscoveryIndex++;
    }
    else
    {
        mCurrentServiceInDiscoveryIndex = 0;
        MEM_BufferFree(mAllPrimaryServices.DiscoveredServices);
        shell_cmd_finished();
    }
}

static void SHELL_DiscoverAllDescriptorsOfChar
(
    void *param
)
{
    uint16_t uuid16;

    if (mCurrentCharInDiscoveryIndex < mAllChars.Service.NbOfCharacteristics)
    {
        uuid16 = mAllChars.Service.Characteristics[mCurrentCharInDiscoveryIndex].Value.Uuid.Uuid16Bits[0] +
                 (mAllChars.Service.Characteristics[mCurrentCharInDiscoveryIndex].Value.Uuid.Uuid16Bits[1] << 8);

        shell_printf("\n\r- %s Value Handle: %d\n\r",
                     ShellGatt_GetCharacteristicName(uuid16),
                     mAllChars.Service.Characteristics[mCurrentCharInDiscoveryIndex].Value.Handle);

        // Value not used here
        MEM_BufferFree(mAllChars.Service.Characteristics[mCurrentCharInDiscoveryIndex].Value.Value);

        if (mAllChars.Service.Characteristics[mCurrentCharInDiscoveryIndex].Value.Handle < mAllChars.Service.EndHandle)
        {
            GATTClientDiscoverAllCharacteristicDescriptorsRequest_t req = { 0 };

            req.DeviceId = gPeerDeviceId;
            req.Characteristic.Properties = mAllChars.Service.Characteristics[mCurrentCharInDiscoveryIndex].Properties;

            req.Characteristic.Value.Handle = mAllChars.Service.Characteristics[mCurrentCharInDiscoveryIndex].Value.Handle;
            req.Characteristic.Value.UuidType = mAllChars.Service.Characteristics[mCurrentCharInDiscoveryIndex].Value.UuidType;
            req.Characteristic.Value.Uuid.Uuid16Bits[0] = mAllChars.Service.Characteristics[mCurrentCharInDiscoveryIndex].Value.Uuid.Uuid16Bits[0];
            req.Characteristic.Value.Uuid.Uuid16Bits[1] = mAllChars.Service.Characteristics[mCurrentCharInDiscoveryIndex].Value.Uuid.Uuid16Bits[1];
            req.Characteristic.Value.MaxValueLength = mAllChars.Service.Characteristics[mCurrentCharInDiscoveryIndex].Value.MaxValueLength;

            req.EndingHandle = mAllChars.Service.EndHandle;
            req.MaxNbOfDescriptors = mMaxCharDescriptorsCount_d;

            GATTClientDiscoverAllCharacteristicDescriptorsRequest(&req, BLE_FSCI_IF);
            mCurrentCharInDiscoveryIndex++;
        }
        else
        {
            mCurrentCharInDiscoveryIndex++;
            TMR_StartSingleShotTimer(mDelayTimerID, 100, SHELL_DiscoverAllDescriptorsOfChar, NULL);
        }
    }
    else
    {
        mCurrentCharInDiscoveryIndex = 0;
        MEM_BufferFree(mAllChars.Service.Characteristics);
        MEM_BufferFree(mAllChars.Service.IncludedServices);
        TMR_StartSingleShotTimer(mDelayTimerID, 100, SHELL_GapDiscoverAllCharsOfService, NULL);
    }
}

static void SHELL_PrintDiscoveredDescriptors
(
    GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication_t *evt
)
{
    uint16_t uuid16;
    SHELL_NEWLINE();

    for (uint8_t k = 0; k < evt->Characteristic.NbOfDescriptors; k++)
    {
        uuid16 = evt->Characteristic.Descriptors[k].Uuid.Uuid16Bits[0] +
                 (evt->Characteristic.Descriptors[k].Uuid.Uuid16Bits[1] << 8);

        shell_printf("\n\r- %s Descriptor Handle: %d\n\r",
                     ShellGatt_GetDescriptorName(uuid16),
                     evt->Characteristic.Descriptors[k].Handle);

        MEM_BufferFree(evt->Characteristic.Descriptors[k].Value);
    }

    MEM_BufferFree(evt->Characteristic.Descriptors);
    MEM_BufferFree(evt->Characteristic.Value.Value);

    // keep the loop going
    TMR_StartSingleShotTimer(mDelayTimerID, 100, SHELL_DiscoverAllDescriptorsOfChar, NULL);
}

static void SHELL_PrintDiscoveredChars
(
    GATTClientProcedureDiscoverAllCharacteristicsIndication_t *evt
)
{
    SHELL_NEWLINE();

    // save for later use
    FLib_MemCpy(&mAllChars, evt, sizeof(GATTClientProcedureDiscoverAllCharacteristicsIndication_t));

    // keep the loop going
    TMR_StartSingleShotTimer(mDelayTimerID, 100, SHELL_DiscoverAllDescriptorsOfChar, NULL);
}

static void SHELL_PrintDiscoveredPrimaryServices
(
    GATTClientProcedureDiscoverAllPrimaryServicesIndication_t *evt
)
{
    SHELL_NEWLINE();
    shell_printf("--> Discovered primary services: %d\n\r", evt->NbOfDiscoveredServices);

    // save for later use
    FLib_MemCpy(&mAllPrimaryServices, evt, sizeof(GATTClientProcedureDiscoverAllPrimaryServicesIndication_t));

    if (mDelayTimerID == gTmrInvalidTimerID_c)
    {
        mDelayTimerID = TMR_AllocateTimer();
    }

    // keep the loop going
    TMR_StartSingleShotTimer(mDelayTimerID, 100, SHELL_GapDiscoverAllCharsOfService, NULL);
}


static char *const gFSCISuccess_c = "gFsciSuccess_c";
static char *const gFSCISAPDisabled_c = "gFSCISAPDisabled_c";
static char *const gFSCIAppMsgTooBig_c = "gFSCIAppMsgTooBig_c";
static char *const gFSCIOutOfMessages_c = "gFSCIOutOfMessages_c";
static char *const gFSCIUnknownOpcodeGroup_c = "gFSCIUnknownOpcodeGroup_c";
static char *const gFSCIOpcodeGroupIsDisabled_c = "gFSCIOpcodeGroupIsDisabled_c";
static char *const gFSCIUnknownOpcode_c = "gFSCIUnknownOpcode_c";
static char *const gFSCITooBig_c = "gFSCITooBig_c";
static char *const gFSCIError_c = "gFSCIError_c";
static char *const gSuccess = "gSuccess";
static char *const gError = "gError";
static char *const gBleSuccess_c = "gBleSuccess_c";
static char *const gBleInvalidParameter_c = "gBleInvalidParameter_c";
static char *const gBleOverflow_c = "gBleOverflow_c";
static char *const gBleUnavailable_c = "gBleUnavailable_c";
static char *const gBleFeatureNotSupported_c = "gBleFeatureNotSupported_c";
static char *const gBleOutOfMemory_c = "gBleOutOfMemory_c";
static char *const gBleAlreadyInitialized_c = "gBleAlreadyInitialized_c";
static char *const gBleOsError_c = "gBleOsError_c";
static char *const gBleUnexpectedError_c = "gBleUnexpectedError_c";
static char *const gBleInvalidState_c = "gBleInvalidState_c";
static char *const gHciUnknownHciCommand_c = "gHciUnknownHciCommand_c";
static char *const gHciUnknownConnectionIdentifier_c = "gHciUnknownConnectionIdentifier_c";
static char *const gHciHardwareFailure_c = "gHciHardwareFailure_c";
static char *const gHciPageTimeout_c = "gHciPageTimeout_c";
static char *const gHciAuthenticationFailure_c = "gHciAuthenticationFailure_c";
static char *const gHciPinOrKeyMissing_c = "gHciPinOrKeyMissing_c";
static char *const gHciMemoryCapacityExceeded_c = "gHciMemoryCapacityExceeded_c";
static char *const gHciConnectionTimeout_c = "gHciConnectionTimeout_c";
static char *const gHciConnectionLimitExceeded_c = "gHciConnectionLimitExceeded_c";
static char *const gHciSynchronousConnectionLimitToADeviceExceeded_c = "gHciSynchronousConnectionLimitToADeviceExceeded_c";
static char *const gHciAclConnectionAlreadyExists_c = "gHciAclConnectionAlreadyExists_c";
static char *const gHciCommandDisallowed_c = "gHciCommandDisallowed_c";
static char *const gHciConnectionRejectedDueToLimitedResources_c = "gHciConnectionRejectedDueToLimitedResources_c";
static char *const gHciConnectionRejectedDueToSecurityReasons_c = "gHciConnectionRejectedDueToSecurityReasons_c";
static char *const gHciConnectionRejectedDueToUnacceptableBdAddr_c = "gHciConnectionRejectedDueToUnacceptableBdAddr_c";
static char *const gHciConnectionAcceptTimeoutExceeded_c = "gHciConnectionAcceptTimeoutExceeded_c";
static char *const gHciUnsupportedFeatureOrParameterValue_c = "gHciUnsupportedFeatureOrParameterValue_c";
static char *const gHciInvalidHciCommandParameters_c = "gHciInvalidHciCommandParameters_c";
static char *const gHciRemoteUserTerminatedConnection_c = "gHciRemoteUserTerminatedConnection_c";
static char *const gHciRemoteDeviceTerminatedConnectionLowResources_c = "gHciRemoteDeviceTerminatedConnectionLowResources_c";
static char *const gHciRemoteDeviceTerminatedConnectionPowerOff_c = "gHciRemoteDeviceTerminatedConnectionPowerOff_c";
static char *const gHciConnectionTerminatedByLocalHost_c = "gHciConnectionTerminatedByLocalHost_c";
static char *const gHciRepeatedAttempts_c = "gHciRepeatedAttempts_c";
static char *const gHciPairingNotAllowed_c = "gHciPairingNotAllowed_c";
static char *const gHciUnknownLpmPdu_c = "gHciUnknownLpmPdu_c";
static char *const gHciUnsupportedRemoteFeature_c = "gHciUnsupportedRemoteFeature_c";
static char *const gHciScoOffsetRejected_c = "gHciScoOffsetRejected_c";
static char *const gHciScoIntervalRejected_c = "gHciScoIntervalRejected_c";
static char *const gHciScoAirModeRejected_c = "gHciScoAirModeRejected_c";
static char *const gHciInvalidLpmParameters_c = "gHciInvalidLpmParameters_c";
static char *const gHciUnspecifiedError_c = "gHciUnspecifiedError_c";
static char *const gHciUnsupportedLpmParameterValue_c = "gHciUnsupportedLpmParameterValue_c";
static char *const gHciRoleChangeNotAllowed_c = "gHciRoleChangeNotAllowed_c";
static char *const gHciLLResponseTimeout_c = "gHciLLResponseTimeout_c";
static char *const gHciLmpErrorTransactionCollision_c = "gHciLmpErrorTransactionCollision_c";
static char *const gHciLmpPduNotAllowed_c = "gHciLmpPduNotAllowed_c";
static char *const gHciEncryptionModeNotAcceptable_c = "gHciEncryptionModeNotAcceptable_c";
static char *const gHciLinkKeyCannotBeChanged_c = "gHciLinkKeyCannotBeChanged_c";
static char *const gHciRequestedQosNotSupported_c = "gHciRequestedQosNotSupported_c";
static char *const gHciInstantPassed_c = "gHciInstantPassed_c";
static char *const gHciPairingWithUnitKeyNotSupported_c = "gHciPairingWithUnitKeyNotSupported_c";
static char *const gHciDifferentTransactionCollision_c = "gHciDifferentTransactionCollision_c";
static char *const gHciReserved_0x2B_c = "gHciReserved_0x2B_c";
static char *const gHciQosNotAcceptableParameter_c = "gHciQosNotAcceptableParameter_c";
static char *const gHciQosRejected_c = "gHciQosRejected_c";
static char *const gHciChannelClassificationNotSupported_c = "gHciChannelClassificationNotSupported_c";
static char *const gHciInsufficientSecurity_c = "gHciInsufficientSecurity_c";
static char *const gHciParameterOutOfMandatoryRange_c = "gHciParameterOutOfMandatoryRange_c";
static char *const gHciReserved_0x31_c = "gHciReserved_0x31_c";
static char *const gHciRoleSwitchPending_c = "gHciRoleSwitchPending_c";
static char *const gHciReserved_0x33_c = "gHciReserved_0x33_c";
static char *const gHciReservedSlotViolation_c = "gHciReservedSlotViolation_c";
static char *const gHciRoleSwitchFailed_c = "gHciRoleSwitchFailed_c";
static char *const gHciExtendedInquiryResponseTooLarge_c = "gHciExtendedInquiryResponseTooLarge_c";
static char *const gHciSecureSimplePairingNotSupportedByHost_c = "gHciSecureSimplePairingNotSupportedByHost_c";
static char *const gHciHostBusyPairing_c = "gHciHostBusyPairing_c";
static char *const gHciConnectionRejectedDueToNoSuitableChannelFound_c = "gHciConnectionRejectedDueToNoSuitableChannelFound_c";
static char *const gHciControllerBusy_c = "gHciControllerBusy_c";
static char *const gHciUnacceptableConnectionParameters_c = "gHciUnacceptableConnectionParameters_c";
static char *const gHciDirectedAdvertisingTimeout_c = "gHciDirectedAdvertisingTimeout_c";
static char *const gHciConnectionTerminatedDueToMicFailure_c = "gHciConnectionTerminatedDueToMicFailure_c";
static char *const gHciConnectionFailedToBeEstablished_c = "gHciConnectionFailedToBeEstablished_c";
static char *const gHciMacConnectionFailed_c = "gHciMacConnectionFailed_c";
static char *const gHciCoarseClockAdjustmentRejected_c = "gHciCoarseClockAdjustmentRejected_c";
static char *const gHciAlreadyInit_c = "gHciAlreadyInit_c";
static char *const gHciInvalidParameter_c = "gHciInvalidParameter_c";
static char *const gHciCallbackNotInstalled_c = "gHciCallbackNotInstalled_c";
static char *const gHciCallbackAlreadyInstalled_c = "gHciCallbackAlreadyInstalled_c";
static char *const gHciCommandNotSupported_c = "gHciCommandNotSupported_c";
static char *const gHciEventNotSupported_c = "gHciEventNotSupported_c";
static char *const gHciTransportError_c = "gHciTransportError_c";
static char *const gL2caAlreadyInit_c = "gL2caAlreadyInit_c";
static char *const gL2caInsufficientResources_c = "gL2caInsufficientResources_c";
static char *const gL2caCallbackNotInstalled_c = "gL2caCallbackNotInstalled_c";
static char *const gL2caCallbackAlreadyInstalled_c = "gL2caCallbackAlreadyInstalled_c";
static char *const gL2caLePsmInvalid_c = "gL2caLePsmInvalid_c";
static char *const gL2caLePsmAlreadyRegistered_c = "gL2caLePsmAlreadyRegistered_c";
static char *const gL2caLePsmNotRegistered_c = "gL2caLePsmNotRegistered_c";
static char *const gL2caLePsmInsufficientResources_c = "gL2caLePsmInsufficientResources_c";
static char *const gL2caChannelInvalid_c = "gL2caChannelInvalid_c";
static char *const gL2caChannelClosed_c = "gL2caChannelClosed_c";
static char *const gL2caChannelAlreadyConnected_c = "gL2caChannelAlreadyConnected_c";
static char *const gL2caConnectionParametersRejected_c = "gL2caConnectionParametersRejected_c";
static char *const gL2caChannelBusy_c = "gL2caChannelBusy_c";
static char *const gL2caInvalidParameter_c = "gL2caInvalidParameter_c";
static char *const gL2caError_c = "gL2caError_c";
static char *const gSmNullCBFunction_c = "gSmNullCBFunction_c";
static char *const gSmCommandNotSupported_c = "gSmCommandNotSupported_c";
static char *const gSmUnexpectedCommand_c = "gSmUnexpectedCommand_c";
static char *const gSmInvalidCommandCode_c = "gSmInvalidCommandCode_c";
static char *const gSmInvalidCommandLength_c = "gSmInvalidCommandLength_c";
static char *const gSmInvalidCommandParameter_c = "gSmInvalidCommandParameter_c";
static char *const gSmInvalidDeviceId_c = "gSmInvalidDeviceId_c";
static char *const gSmInvalidInternalOperation_c = "gSmInvalidInternalOperation_c";
static char *const gSmInvalidConnectionHandle_c = "gSmInvalidConnectionHandle_c";
static char *const gSmInproperKeyDistributionField_c = "gSmInproperKeyDistributionField_c";
static char *const gSmUnexpectedKeyType_c = "gSmUnexpectedKeyType_c";
static char *const gSmUnexpectedPairingTerminationReason_c = "gSmUnexpectedPairingTerminationReason_c";
static char *const gSmUnexpectedKeyset_c = "gSmUnexpectedKeyset_c";
static char *const gSmSmpTimeoutOccurred_c = "gSmSmpTimeoutOccurred_c";
static char *const gSmUnknownSmpPacketType_c = "gSmUnknownSmpPacketType_c";
static char *const gSmInvalidSmpPacketLength_c = "gSmInvalidSmpPacketLength_c";
static char *const gSmInvalidSmpPacketParameter_c = "gSmInvalidSmpPacketParameter_c";
static char *const gSmReceivedUnexpectedSmpPacket_c = "gSmReceivedUnexpectedSmpPacket_c";
static char *const gSmReceivedSmpPacketFromUnknownDevice_c = "gSmReceivedSmpPacketFromUnknownDevice_c";
static char *const gSmReceivedUnexpectedHciEvent_c = "gSmReceivedUnexpectedHciEvent_c";
static char *const gSmReceivedHciEventFromUnknownDevice_c = "gSmReceivedHciEventFromUnknownDevice_c";
static char *const gSmInvalidHciEventParameter_c = "gSmInvalidHciEventParameter_c";
static char *const gSmLlConnectionEncryptionInProgress_c = "gSmLlConnectionEncryptionInProgress_c";
static char *const gSmLlConnectionEncryptionFailure_c = "gSmLlConnectionEncryptionFailure_c";
static char *const gSmInsufficientResources_c = "gSmInsufficientResources_c";
static char *const gSmOobDataAddressMismatch_c = "gSmOobDataAddressMismatch_c";
// static char *const gSmSmpPacketReceivedAfterTimeoutOccurred_c = "gSmSmpPacketReceivedAfterTimeoutOccurred_c";
static char *const gSmPairingErrorPasskeyEntryFailed_c = "gSmPairingErrorPasskeyEntryFailed_c";
static char *const gSmPairingErrorConfirmValueFailed_c = "gSmPairingErrorConfirmValueFailed_c";
static char *const gSmPairingErrorCommandNotSupported_c = "gSmPairingErrorCommandNotSupported_c";
static char *const gSmPairingErrorInvalidParameters_c = "gSmPairingErrorInvalidParameters_c";
static char *const gSmPairingErrorUnknownReason_c = "gSmPairingErrorUnknownReason_c";
static char *const gSmTbResolvableAddressDoesNotMatchIrk_c = "gSmTbResolvableAddressDoesNotMatchIrk_c";
static char *const gSmTbInvalidDataSignature_c = "gSmTbInvalidDataSignature_c";
static char *const gAttInvalidHandle_c = "gAttInvalidHandle_c";
static char *const gAttReadNotPermitted_c = "gAttReadNotPermitted_c";
static char *const gAttWriteNotPermitted_c = "gAttWriteNotPermitted_c";
static char *const gAttInvalidPdu_c = "gAttInvalidPdu_c";
static char *const gAttInsufficientAuthentication_c = "gAttInsufficientAuthentication_c";
static char *const gAttRequestNotSupported_c = "gAttRequestNotSupported_c";
static char *const gAttInvalidOffset_c = "gAttInvalidOffset_c";
static char *const gAttInsufficientAuthorization_c = "gAttInsufficientAuthorization_c";
static char *const gAttPrepareQueueFull_c = "gAttPrepareQueueFull_c";
static char *const gAttAttributeNotFound_c = "gAttAttributeNotFound_c";
static char *const gAttAttributeNotLong_c = "gAttAttributeNotLong_c";
static char *const gAttInsufficientEncryptionKeySize_c = "gAttInsufficientEncryptionKeySize_c";
static char *const gAttInvalidAttributeValueLength_c = "gAttInvalidAttributeValueLength_c";
static char *const gAttUnlikelyor_c = "gAttUnlikelyor_c";
static char *const gAttInsufficientEncryption_c = "gAttInsufficientEncryption_c";
static char *const gAttUnsupportedGroupType_c = "gAttUnsupportedGroupType_c";
static char *const gAttInsufficientResources_c = "gAttInsufficientResources_c";
static char *const gGattAnotherProcedureInProgress_c = "gGattAnotherProcedureInProgress_c";
static char *const gGattLongAttributePacketsCorrupted_c = "gGattLongAttributePacketsCorrupted_c";
static char *const gGattMultipleAttributesOverflow_c = "gGattMultipleAttributesOverflow_c";
static char *const gGattUnexpectedReadMultipleResponseLength_c = "gGattUnexpectedReadMultipleResponseLength_c";
static char *const gGattInvalidValueLength_c = "gGattInvalidValueLength_c";
static char *const gGattServerTimeout_c = "gGattServerTimeout_c";
static char *const gGattIndicationAlreadyInProgress_c = "gGattIndicationAlreadyInProgress_c";
static char *const gGattClientConfirmationTimeout_c = "gGattClientConfirmationTimeout_c";
static char *const gGapAdvDataTooLong_c = "gGapAdvDataTooLong_c";
static char *const gGapScanRspDataTooLong_c = "gGapScanRspDataTooLong_c";
static char *const gGapDeviceNotBonded_c = "gGapDeviceNotBonded_c";
static char *const gDevDbCccdLimitReached_c = "gDevDbCccdLimitReached_c";
static char *const gDevDbCccdNotFound_c = "gDevDbCccdNotFound_c";
static char *const gGattDbInvalidHandle_c = "gGattDbInvalidHandle_c";
static char *const gGattDbCharacteristicNotFound_c = "gGattDbCharacteristicNotFound_c";
static char *const gGattDbCccdNotFound_c = "gGattDbCccdNotFound_c";
static char *const gGattDbServiceNotFound_c = "gGattDbServiceNotFound_c";
static char *const gGattDbDescriptorNotFound_c = "gGattDbDescriptorNotFound_c";
static char *const gGattProcSuccess_c = "gGattProcSuccess_c";
static char *const gProcedureError_c = "gProcedureError_c";
static char *const gBleOsc = "gBleOsc";
static char *const gBleUnexpectedc = "gBleUnexpectedc";
static char *const gHciUnspecifiedc = "gHciUnspecifiedc";
static char *const gHciTransportc = "gHciTransportc";
static char *const gL2cac = "gL2cac";
static char *const gHciCommandStatus_c = "gHciCommandStatus_c";
static char *const gCheckPrivateResolvableAddress_c = "gCheckPrivateResolvableAddress_c";
static char *const gVerifySignature_c = "gVerifySignature_c";
static char *const gAddNewConnection_c = "gAddNewConnection_c";
static char *const gResetController_c = "gResetController_c";
static char *const gSetEventMask_c = "gSetEventMask_c";
static char *const gReadLeBufferSize_c = "gReadLeBufferSize_c";
static char *const gSetLeEventMask_c = "gSetLeEventMask_c";
static char *const gReadDeviceAddress_c = "gReadDeviceAddress_c";
static char *const gReadLocalSupportedFeatures_c = "gReadLocalSupportedFeatures_c";
static char *const gReadWhiteListSize_c = "gReadWhiteListSize_c";
static char *const gClearWhiteList_c = "gClearWhiteList_c";
static char *const gAddDeviceToWhiteList_c = "gAddDeviceToWhiteList_c";
static char *const gRemoveDeviceFromWhiteList_c = "gRemoveDeviceFromWhiteList_c";
static char *const gCancelCreateConnection_c = "gCancelCreateConnection_c";
static char *const gReadRadioPower_c = "gReadRadioPower_c";
static char *const gSetRandomAddress_c = "gSetRandomAddress_c";
static char *const gCreateRandomAddress_c = "gCreateRandomAddress_c";
static char *const gEncryptLink_c = "gEncryptLink_c";
static char *const gProvideLongTermKey_c = "gProvideLongTermKey_c";
static char *const gDenyLongTermKey_c = "gDenyLongTermKey_c";
static char *const gConnect_c = "gConnect_c";
static char *const gDisconnect_c = "gDisconnect_c";
static char *const gTerminatePairing_c = "gTerminatePairing_c";
static char *const gSendSlaveSecurityRequest_c = "gSendSlaveSecurityRequest_c";
static char *const gEnterPasskey_c = "gEnterPasskey_c";
static char *const gProvideOob_c = "gProvideOob_c";
static char *const gSendSmpKeys_c = "gSendSmpKeys_c";
static char *const gWriteSuggestedDefaultDataLength_c = "gWriteSuggestedDefaultDataLength_c";
static char *const gReadSuggestedDefaultDataLength_c = "gReadSuggestedDefaultDataLength_c";
static char *const gUpdateLeDataLength_c = "gUpdateLeDataLength_c";
static char *const gEnableControllerPrivacy_c = "gEnableControllerPrivacy_c";
static char *const gLeScSendKeypressNotification_c = "gLeScSendKeypressNotification_c";
static char *const gLeScSetPeerOobData_c = "gLeScSetPeerOobData_c";
static char *const gLeScGetLocalOobData_c = "gLeScGetLocalOobData_c";
static char *const gLeScValidateNumericValue_c = "gLeScValidateNumericValue_c";
static char *const gLeScRegeneratePublicKey_c = "gLeScRegeneratePublicKey_c";
static char *const gLeSetResolvablePrivateAddressTimeout_c = "gLeSetResolvablePrivateAddressTimeout_c";
static char *const gPublic_c = "gPublic_c";
static char *const gRandom_c = "gRandom_c";
static char *const gBleAdvRepAdvInd_c = "gBleAdvRepAdvInd_c";
static char *const gBleAdvRepAdvDirectInd_c = "gBleAdvRepAdvDirectInd_c";
static char *const gBleAdvRepAdvScanInd_c = "gBleAdvRepAdvScanInd_c";
static char *const gBleAdvRepAdvNonconnInd_c = "gBleAdvRepAdvNonconnInd_c";
static char *const gBleAdvRepScanRsp_c = "gBleAdvRepScanRsp_c";
static char *const gOobNotAvailable_c = "gOobNotAvailable_c";
static char *const gIncompatibleIoCapabilities_c = "gIncompatibleIoCapabilities_c";
static char *const gPairingNotSupported_c = "gPairingNotSupported_c";
static char *const gLowEncryptionKeySize_c = "gLowEncryptionKeySize_c";
static char *const gUnspecifiedReason_c = "gUnspecifiedReason_c";
static char *const gRepeatedAttempts_c = "gRepeatedAttempts_c";
static char *const gLinkEncryptionFailed_c = "gLinkEncryptionFailed_c";
static char *const gNoKeys_c = "gNoKeys_c";
static char *const gLtk_c = "gLtk_c";
static char *const gIrk_c = "gIrk_c";
static char *const gCsrk_c = "gCsrk_c";
static char *const gPairingSuccessful_c = "gPairingSuccessful_c";
static char *const gPairingFailed_c = "gPairingFailed_c";
static char *const gSmUnexpectedPairingTermination_c = "gSmUnexpectedPairingTermination_c";
static char *const gSmPairingErrorUnknown_c = "gSmPairingErrorUnknown_c";

/*!*************************************************************************************************
\fn     void SHELL_BleEventNotify(void *param)
\brief  Print to host's shell information about BLE events coming from black-box

\param  [in]    param       the received event
***************************************************************************************************/
void SHELL_BleEventNotify(void *param)
{
    bleEvtContainer_t *container = (bleEvtContainer_t *)param;

    if (!mSuppressBleEventPrint)
    {
        SHELL_NEWLINE();
    }

    switch (container->id)
    {
        case 0xA4FE:
            shell_write("\r\nFSCIErrorIndication");
            shell_write(" -> ");

            switch (container->Data.FSCIErrorIndication.Status)
            {
                case FSCIErrorIndication_Status_gFsciSuccess_c:
                    shell_write(gFSCISuccess_c);
                    break;

                case FSCIErrorIndication_Status_gFsciSAPDisabled_c:
                    shell_write(gFSCISAPDisabled_c);
                    break;

                case FSCIErrorIndication_Status_gFsciAppMsgTooBig_c:
                    shell_write(gFSCIAppMsgTooBig_c);
                    break;

                case FSCIErrorIndication_Status_gFsciOutOfMessages_c:
                    shell_write(gFSCIOutOfMessages_c);
                    break;

                case FSCIErrorIndication_Status_gFsciUnknownOpcodeGroup_c:
                    shell_write(gFSCIUnknownOpcodeGroup_c);
                    break;

                case FSCIErrorIndication_Status_gFsciOpcodeGroupIsDisabled_c:
                    shell_write(gFSCIOpcodeGroupIsDisabled_c);
                    break;

                case FSCIErrorIndication_Status_gFsciUnknownOpcode_c:
                    shell_write(gFSCIUnknownOpcode_c);
                    break;

                case FSCIErrorIndication_Status_gFsciTooBig_c:
                    shell_write(gFSCITooBig_c);
                    break;

                case FSCIErrorIndication_Status_gFsciError_c:
                    shell_write(gFSCIError_c);
                    break;

                default:
                    shell_printf("Unrecognized status 0x%02X", container->Data.FSCIErrorIndication.Status);
                    break;
            }

            break;

        case 0xA470:
            shell_write("\r\nFSCIAllowDeviceToSleepConfirm");
            shell_write(" -> ");

            switch (container->Data.FSCIAllowDeviceToSleepConfirm.Status)
            {
                case FSCIAllowDeviceToSleepConfirm_Status_gSuccess:
                    shell_write(gSuccess);
                    break;

                case FSCIAllowDeviceToSleepConfirm_Status_gError:
                    shell_write(gError);
                    break;

                default:
                    shell_printf("Unrecognized status 0x%02X", container->Data.FSCIAllowDeviceToSleepConfirm.Status);
                    break;
            }

            break;

        case 0xA471:
            shell_write("\r\nFSCIWakeUpIndication");
            break;

        case 0xA472:
            shell_write("\r\nFSCIGetWakeupReasonResponse");
            break;

        case 0x4180:

            if (mSuppressBleEventPrint)
            {
                break;
            }

            shell_write("\r\nL2CAPConfirm");
            shell_write(" -> ");

            switch (container->Data.L2CAPConfirm.Status)
            {
                case L2CAPConfirm_Status_gBleSuccess_c:
                    shell_write(gBleSuccess_c);
                    break;

                case L2CAPConfirm_Status_gBleInvalidParameter_c:
                    shell_write(gBleInvalidParameter_c);
                    break;

                case L2CAPConfirm_Status_gBleOverflow_c:
                    shell_write(gBleOverflow_c);
                    break;

                case L2CAPConfirm_Status_gBleUnavailable_c:
                    shell_write(gBleUnavailable_c);
                    break;

                case L2CAPConfirm_Status_gBleFeatureNotSupported_c:
                    shell_write(gBleFeatureNotSupported_c);
                    break;

                case L2CAPConfirm_Status_gBleOutOfMemory_c:
                    shell_write(gBleOutOfMemory_c);
                    break;

                case L2CAPConfirm_Status_gBleAlreadyInitialized_c:
                    shell_write(gBleAlreadyInitialized_c);
                    break;

                case L2CAPConfirm_Status_gBleOsError_c:
                    shell_write(gBleOsError_c);
                    break;

                case L2CAPConfirm_Status_gBleUnexpectedError_c:
                    shell_write(gBleUnexpectedError_c);
                    break;

                case L2CAPConfirm_Status_gBleInvalidState_c:
                    shell_write(gBleInvalidState_c);
                    break;

                case L2CAPConfirm_Status_gHciUnknownHciCommand_c:
                    shell_write(gHciUnknownHciCommand_c);
                    break;

                case L2CAPConfirm_Status_gHciUnknownConnectionIdentifier_c:
                    shell_write(gHciUnknownConnectionIdentifier_c);
                    break;

                case L2CAPConfirm_Status_gHciHardwareFailure_c:
                    shell_write(gHciHardwareFailure_c);
                    break;

                case L2CAPConfirm_Status_gHciPageTimeout_c:
                    shell_write(gHciPageTimeout_c);
                    break;

                case L2CAPConfirm_Status_gHciAuthenticationFailure_c:
                    shell_write(gHciAuthenticationFailure_c);
                    break;

                case L2CAPConfirm_Status_gHciPinOrKeyMissing_c:
                    shell_write(gHciPinOrKeyMissing_c);
                    break;

                case L2CAPConfirm_Status_gHciMemoryCapacityExceeded_c:
                    shell_write(gHciMemoryCapacityExceeded_c);
                    break;

                case L2CAPConfirm_Status_gHciConnectionTimeout_c:
                    shell_write(gHciConnectionTimeout_c);
                    break;

                case L2CAPConfirm_Status_gHciConnectionLimitExceeded_c:
                    shell_write(gHciConnectionLimitExceeded_c);
                    break;

                case L2CAPConfirm_Status_gHciSynchronousConnectionLimitToADeviceExceeded_c:
                    shell_write(gHciSynchronousConnectionLimitToADeviceExceeded_c);
                    break;

                case L2CAPConfirm_Status_gHciAclConnectionAlreadyExists_c:
                    shell_write(gHciAclConnectionAlreadyExists_c);
                    break;

                case L2CAPConfirm_Status_gHciCommandDisallowed_c:
                    shell_write(gHciCommandDisallowed_c);
                    break;

                case L2CAPConfirm_Status_gHciConnectionRejectedDueToLimitedResources_c:
                    shell_write(gHciConnectionRejectedDueToLimitedResources_c);
                    break;

                case L2CAPConfirm_Status_gHciConnectionRejectedDueToSecurityReasons_c:
                    shell_write(gHciConnectionRejectedDueToSecurityReasons_c);
                    break;

                case L2CAPConfirm_Status_gHciConnectionRejectedDueToUnacceptableBdAddr_c:
                    shell_write(gHciConnectionRejectedDueToUnacceptableBdAddr_c);
                    break;

                case L2CAPConfirm_Status_gHciConnectionAcceptTimeoutExceeded_c:
                    shell_write(gHciConnectionAcceptTimeoutExceeded_c);
                    break;

                case L2CAPConfirm_Status_gHciUnsupportedFeatureOrParameterValue_c:
                    shell_write(gHciUnsupportedFeatureOrParameterValue_c);
                    break;

                case L2CAPConfirm_Status_gHciInvalidHciCommandParameters_c:
                    shell_write(gHciInvalidHciCommandParameters_c);
                    break;

                case L2CAPConfirm_Status_gHciRemoteUserTerminatedConnection_c:
                    shell_write(gHciRemoteUserTerminatedConnection_c);
                    break;

                case L2CAPConfirm_Status_gHciRemoteDeviceTerminatedConnectionLowResources_c:
                    shell_write(gHciRemoteDeviceTerminatedConnectionLowResources_c);
                    break;

                case L2CAPConfirm_Status_gHciRemoteDeviceTerminatedConnectionPowerOff_c:
                    shell_write(gHciRemoteDeviceTerminatedConnectionPowerOff_c);
                    break;

                case L2CAPConfirm_Status_gHciConnectionTerminatedByLocalHost_c:
                    shell_write(gHciConnectionTerminatedByLocalHost_c);
                    break;

                case L2CAPConfirm_Status_gHciRepeatedAttempts_c:
                    shell_write(gHciRepeatedAttempts_c);
                    break;

                case L2CAPConfirm_Status_gHciPairingNotAllowed_c:
                    shell_write(gHciPairingNotAllowed_c);
                    break;

                case L2CAPConfirm_Status_gHciUnknownLpmPdu_c:
                    shell_write(gHciUnknownLpmPdu_c);
                    break;

                case L2CAPConfirm_Status_gHciUnsupportedRemoteFeature_c:
                    shell_write(gHciUnsupportedRemoteFeature_c);
                    break;

                case L2CAPConfirm_Status_gHciScoOffsetRejected_c:
                    shell_write(gHciScoOffsetRejected_c);
                    break;

                case L2CAPConfirm_Status_gHciScoIntervalRejected_c:
                    shell_write(gHciScoIntervalRejected_c);
                    break;

                case L2CAPConfirm_Status_gHciScoAirModeRejected_c:
                    shell_write(gHciScoAirModeRejected_c);
                    break;

                case L2CAPConfirm_Status_gHciInvalidLpmParameters_c:
                    shell_write(gHciInvalidLpmParameters_c);
                    break;

                case L2CAPConfirm_Status_gHciUnspecifiedError_c:
                    shell_write(gHciUnspecifiedError_c);
                    break;

                case L2CAPConfirm_Status_gHciUnsupportedLpmParameterValue_c:
                    shell_write(gHciUnsupportedLpmParameterValue_c);
                    break;

                case L2CAPConfirm_Status_gHciRoleChangeNotAllowed_c:
                    shell_write(gHciRoleChangeNotAllowed_c);
                    break;

                case L2CAPConfirm_Status_gHciLLResponseTimeout_c:
                    shell_write(gHciLLResponseTimeout_c);
                    break;

                case L2CAPConfirm_Status_gHciLmpErrorTransactionCollision_c:
                    shell_write(gHciLmpErrorTransactionCollision_c);
                    break;

                case L2CAPConfirm_Status_gHciLmpPduNotAllowed_c:
                    shell_write(gHciLmpPduNotAllowed_c);
                    break;

                case L2CAPConfirm_Status_gHciEncryptionModeNotAcceptable_c:
                    shell_write(gHciEncryptionModeNotAcceptable_c);
                    break;

                case L2CAPConfirm_Status_gHciLinkKeyCannotBeChanged_c:
                    shell_write(gHciLinkKeyCannotBeChanged_c);
                    break;

                case L2CAPConfirm_Status_gHciRequestedQosNotSupported_c:
                    shell_write(gHciRequestedQosNotSupported_c);
                    break;

                case L2CAPConfirm_Status_gHciInstantPassed_c:
                    shell_write(gHciInstantPassed_c);
                    break;

                case L2CAPConfirm_Status_gHciPairingWithUnitKeyNotSupported_c:
                    shell_write(gHciPairingWithUnitKeyNotSupported_c);
                    break;

                case L2CAPConfirm_Status_gHciDifferentTransactionCollision_c:
                    shell_write(gHciDifferentTransactionCollision_c);
                    break;

                case L2CAPConfirm_Status_gHciReserved_0x2B_c:
                    shell_write(gHciReserved_0x2B_c);
                    break;

                case L2CAPConfirm_Status_gHciQosNotAcceptableParameter_c:
                    shell_write(gHciQosNotAcceptableParameter_c);
                    break;

                case L2CAPConfirm_Status_gHciQosRejected_c:
                    shell_write(gHciQosRejected_c);
                    break;

                case L2CAPConfirm_Status_gHciChannelClassificationNotSupported_c:
                    shell_write(gHciChannelClassificationNotSupported_c);
                    break;

                case L2CAPConfirm_Status_gHciInsufficientSecurity_c:
                    shell_write(gHciInsufficientSecurity_c);
                    break;

                case L2CAPConfirm_Status_gHciParameterOutOfMandatoryRange_c:
                    shell_write(gHciParameterOutOfMandatoryRange_c);
                    break;

                case L2CAPConfirm_Status_gHciReserved_0x31_c:
                    shell_write(gHciReserved_0x31_c);
                    break;

                case L2CAPConfirm_Status_gHciRoleSwitchPending_c:
                    shell_write(gHciRoleSwitchPending_c);
                    break;

                case L2CAPConfirm_Status_gHciReserved_0x33_c:
                    shell_write(gHciReserved_0x33_c);
                    break;

                case L2CAPConfirm_Status_gHciReservedSlotViolation_c:
                    shell_write(gHciReservedSlotViolation_c);
                    break;

                case L2CAPConfirm_Status_gHciRoleSwitchFailed_c:
                    shell_write(gHciRoleSwitchFailed_c);
                    break;

                case L2CAPConfirm_Status_gHciExtendedInquiryResponseTooLarge_c:
                    shell_write(gHciExtendedInquiryResponseTooLarge_c);
                    break;

                case L2CAPConfirm_Status_gHciSecureSimplePairingNotSupportedByHost_c:
                    shell_write(gHciSecureSimplePairingNotSupportedByHost_c);
                    break;

                case L2CAPConfirm_Status_gHciHostBusyPairing_c:
                    shell_write(gHciHostBusyPairing_c);
                    break;

                case L2CAPConfirm_Status_gHciConnectionRejectedDueToNoSuitableChannelFound_c:
                    shell_write(gHciConnectionRejectedDueToNoSuitableChannelFound_c);
                    break;

                case L2CAPConfirm_Status_gHciControllerBusy_c:
                    shell_write(gHciControllerBusy_c);
                    break;

                case L2CAPConfirm_Status_gHciUnacceptableConnectionParameters_c:
                    shell_write(gHciUnacceptableConnectionParameters_c);
                    break;

                case L2CAPConfirm_Status_gHciDirectedAdvertisingTimeout_c:
                    shell_write(gHciDirectedAdvertisingTimeout_c);
                    break;

                case L2CAPConfirm_Status_gHciConnectionTerminatedDueToMicFailure_c:
                    shell_write(gHciConnectionTerminatedDueToMicFailure_c);
                    break;

                case L2CAPConfirm_Status_gHciConnectionFailedToBeEstablished_c:
                    shell_write(gHciConnectionFailedToBeEstablished_c);
                    break;

                case L2CAPConfirm_Status_gHciMacConnectionFailed_c:
                    shell_write(gHciMacConnectionFailed_c);
                    break;

                case L2CAPConfirm_Status_gHciCoarseClockAdjustmentRejected_c:
                    shell_write(gHciCoarseClockAdjustmentRejected_c);
                    break;

                case L2CAPConfirm_Status_gHciAlreadyInit_c:
                    shell_write(gHciAlreadyInit_c);
                    break;

                case L2CAPConfirm_Status_gHciInvalidParameter_c:
                    shell_write(gHciInvalidParameter_c);
                    break;

                case L2CAPConfirm_Status_gHciCallbackNotInstalled_c:
                    shell_write(gHciCallbackNotInstalled_c);
                    break;

                case L2CAPConfirm_Status_gHciCallbackAlreadyInstalled_c:
                    shell_write(gHciCallbackAlreadyInstalled_c);
                    break;

                case L2CAPConfirm_Status_gHciCommandNotSupported_c:
                    shell_write(gHciCommandNotSupported_c);
                    break;

                case L2CAPConfirm_Status_gHciEventNotSupported_c:
                    shell_write(gHciEventNotSupported_c);
                    break;

                case L2CAPConfirm_Status_gHciTransportError_c:
                    shell_write(gHciTransportError_c);
                    break;

                case L2CAPConfirm_Status_gL2caAlreadyInit_c:
                    shell_write(gL2caAlreadyInit_c);
                    break;

                case L2CAPConfirm_Status_gL2caInsufficientResources_c:
                    shell_write(gL2caInsufficientResources_c);
                    break;

                case L2CAPConfirm_Status_gL2caCallbackNotInstalled_c:
                    shell_write(gL2caCallbackNotInstalled_c);
                    break;

                case L2CAPConfirm_Status_gL2caCallbackAlreadyInstalled_c:
                    shell_write(gL2caCallbackAlreadyInstalled_c);
                    break;

                case L2CAPConfirm_Status_gL2caLePsmInvalid_c:
                    shell_write(gL2caLePsmInvalid_c);
                    break;

                case L2CAPConfirm_Status_gL2caLePsmAlreadyRegistered_c:
                    shell_write(gL2caLePsmAlreadyRegistered_c);
                    break;

                case L2CAPConfirm_Status_gL2caLePsmNotRegistered_c:
                    shell_write(gL2caLePsmNotRegistered_c);
                    break;

                case L2CAPConfirm_Status_gL2caLePsmInsufficientResources_c:
                    shell_write(gL2caLePsmInsufficientResources_c);
                    break;

                case L2CAPConfirm_Status_gL2caChannelInvalid_c:
                    shell_write(gL2caChannelInvalid_c);
                    break;

                case L2CAPConfirm_Status_gL2caChannelClosed_c:
                    shell_write(gL2caChannelClosed_c);
                    break;

                case L2CAPConfirm_Status_gL2caChannelAlreadyConnected_c:
                    shell_write(gL2caChannelAlreadyConnected_c);
                    break;

                case L2CAPConfirm_Status_gL2caConnectionParametersRejected_c:
                    shell_write(gL2caConnectionParametersRejected_c);
                    break;

                case L2CAPConfirm_Status_gL2caChannelBusy_c:
                    shell_write(gL2caChannelBusy_c);
                    break;

                case L2CAPConfirm_Status_gL2caInvalidParameter_c:
                    shell_write(gL2caInvalidParameter_c);
                    break;

                case L2CAPConfirm_Status_gL2caError_c:
                    shell_write(gL2caError_c);
                    break;

                case L2CAPConfirm_Status_gSmNullCBFunction_c:
                    shell_write(gSmNullCBFunction_c);
                    break;

                case L2CAPConfirm_Status_gSmCommandNotSupported_c:
                    shell_write(gSmCommandNotSupported_c);
                    break;

                case L2CAPConfirm_Status_gSmUnexpectedCommand_c:
                    shell_write(gSmUnexpectedCommand_c);
                    break;

                case L2CAPConfirm_Status_gSmInvalidCommandCode_c:
                    shell_write(gSmInvalidCommandCode_c);
                    break;

                case L2CAPConfirm_Status_gSmInvalidCommandLength_c:
                    shell_write(gSmInvalidCommandLength_c);
                    break;

                case L2CAPConfirm_Status_gSmInvalidCommandParameter_c:
                    shell_write(gSmInvalidCommandParameter_c);
                    break;

                case L2CAPConfirm_Status_gSmInvalidDeviceId_c:
                    shell_write(gSmInvalidDeviceId_c);
                    break;

                case L2CAPConfirm_Status_gSmInvalidInternalOperation_c:
                    shell_write(gSmInvalidInternalOperation_c);
                    break;

                case L2CAPConfirm_Status_gSmInvalidConnectionHandle_c:
                    shell_write(gSmInvalidConnectionHandle_c);
                    break;

                case L2CAPConfirm_Status_gSmInproperKeyDistributionField_c:
                    shell_write(gSmInproperKeyDistributionField_c);
                    break;

                case L2CAPConfirm_Status_gSmUnexpectedKeyType_c:
                    shell_write(gSmUnexpectedKeyType_c);
                    break;

                case L2CAPConfirm_Status_gSmUnexpectedPairingTerminationReason_c:
                    shell_write(gSmUnexpectedPairingTerminationReason_c);
                    break;

                case L2CAPConfirm_Status_gSmUnexpectedKeyset_c:
                    shell_write(gSmUnexpectedKeyset_c);
                    break;

                case L2CAPConfirm_Status_gSmSmpTimeoutOccurred_c:
                    shell_write(gSmSmpTimeoutOccurred_c);
                    break;

                case L2CAPConfirm_Status_gSmUnknownSmpPacketType_c:
                    shell_write(gSmUnknownSmpPacketType_c);
                    break;

                case L2CAPConfirm_Status_gSmInvalidSmpPacketLength_c:
                    shell_write(gSmInvalidSmpPacketLength_c);
                    break;

                case L2CAPConfirm_Status_gSmInvalidSmpPacketParameter_c:
                    shell_write(gSmInvalidSmpPacketParameter_c);
                    break;

                case L2CAPConfirm_Status_gSmReceivedUnexpectedSmpPacket_c:
                    shell_write(gSmReceivedUnexpectedSmpPacket_c);
                    break;

                case L2CAPConfirm_Status_gSmReceivedSmpPacketFromUnknownDevice_c:
                    shell_write(gSmReceivedSmpPacketFromUnknownDevice_c);
                    break;

                case L2CAPConfirm_Status_gSmReceivedUnexpectedHciEvent_c:
                    shell_write(gSmReceivedUnexpectedHciEvent_c);
                    break;

                case L2CAPConfirm_Status_gSmReceivedHciEventFromUnknownDevice_c:
                    shell_write(gSmReceivedHciEventFromUnknownDevice_c);
                    break;

                case L2CAPConfirm_Status_gSmInvalidHciEventParameter_c:
                    shell_write(gSmInvalidHciEventParameter_c);
                    break;

                case L2CAPConfirm_Status_gSmLlConnectionEncryptionInProgress_c:
                    shell_write(gSmLlConnectionEncryptionInProgress_c);
                    break;

                case L2CAPConfirm_Status_gSmLlConnectionEncryptionFailure_c:
                    shell_write(gSmLlConnectionEncryptionFailure_c);
                    break;

                case L2CAPConfirm_Status_gSmInsufficientResources_c:
                    shell_write(gSmInsufficientResources_c);
                    break;

                case L2CAPConfirm_Status_gSmOobDataAddressMismatch_c:
                    shell_write(gSmOobDataAddressMismatch_c);
                    break;

//                case L2CAPConfirm_Status_gSmSmpPacketReceivedAfterTimeoutOccurred_c:
//                    shell_write(gSmSmpPacketReceivedAfterTimeoutOccurred_c);
//                    break;

                case L2CAPConfirm_Status_gSmPairingErrorPasskeyEntryFailed_c:
                    shell_write(gSmPairingErrorPasskeyEntryFailed_c);
                    break;

                case L2CAPConfirm_Status_gSmPairingErrorConfirmValueFailed_c:
                    shell_write(gSmPairingErrorConfirmValueFailed_c);
                    break;

                case L2CAPConfirm_Status_gSmPairingErrorCommandNotSupported_c:
                    shell_write(gSmPairingErrorCommandNotSupported_c);
                    break;

                case L2CAPConfirm_Status_gSmPairingErrorInvalidParameters_c:
                    shell_write(gSmPairingErrorInvalidParameters_c);
                    break;

                case L2CAPConfirm_Status_gSmPairingErrorUnknownReason_c:
                    shell_write(gSmPairingErrorUnknownReason_c);
                    break;

                case L2CAPConfirm_Status_gSmTbResolvableAddressDoesNotMatchIrk_c:
                    shell_write(gSmTbResolvableAddressDoesNotMatchIrk_c);
                    break;

                case L2CAPConfirm_Status_gSmTbInvalidDataSignature_c:
                    shell_write(gSmTbInvalidDataSignature_c);
                    break;

                case L2CAPConfirm_Status_gAttInvalidHandle_c:
                    shell_write(gAttInvalidHandle_c);
                    break;

                case L2CAPConfirm_Status_gAttReadNotPermitted_c:
                    shell_write(gAttReadNotPermitted_c);
                    break;

                case L2CAPConfirm_Status_gAttWriteNotPermitted_c:
                    shell_write(gAttWriteNotPermitted_c);
                    break;

                case L2CAPConfirm_Status_gAttInvalidPdu_c:
                    shell_write(gAttInvalidPdu_c);
                    break;

                case L2CAPConfirm_Status_gAttInsufficientAuthentication_c:
                    shell_write(gAttInsufficientAuthentication_c);
                    break;

                case L2CAPConfirm_Status_gAttRequestNotSupported_c:
                    shell_write(gAttRequestNotSupported_c);
                    break;

                case L2CAPConfirm_Status_gAttInvalidOffset_c:
                    shell_write(gAttInvalidOffset_c);
                    break;

                case L2CAPConfirm_Status_gAttInsufficientAuthorization_c:
                    shell_write(gAttInsufficientAuthorization_c);
                    break;

                case L2CAPConfirm_Status_gAttPrepareQueueFull_c:
                    shell_write(gAttPrepareQueueFull_c);
                    break;

                case L2CAPConfirm_Status_gAttAttributeNotFound_c:
                    shell_write(gAttAttributeNotFound_c);
                    break;

                case L2CAPConfirm_Status_gAttAttributeNotLong_c:
                    shell_write(gAttAttributeNotLong_c);
                    break;

                case L2CAPConfirm_Status_gAttInsufficientEncryptionKeySize_c:
                    shell_write(gAttInsufficientEncryptionKeySize_c);
                    break;

                case L2CAPConfirm_Status_gAttInvalidAttributeValueLength_c:
                    shell_write(gAttInvalidAttributeValueLength_c);
                    break;

                case L2CAPConfirm_Status_gAttUnlikelyor_c:
                    shell_write(gAttUnlikelyor_c);
                    break;

                case L2CAPConfirm_Status_gAttInsufficientEncryption_c:
                    shell_write(gAttInsufficientEncryption_c);
                    break;

                case L2CAPConfirm_Status_gAttUnsupportedGroupType_c:
                    shell_write(gAttUnsupportedGroupType_c);
                    break;

                case L2CAPConfirm_Status_gAttInsufficientResources_c:
                    shell_write(gAttInsufficientResources_c);
                    break;

                case L2CAPConfirm_Status_gGattAnotherProcedureInProgress_c:
                    shell_write(gGattAnotherProcedureInProgress_c);
                    break;

                case L2CAPConfirm_Status_gGattLongAttributePacketsCorrupted_c:
                    shell_write(gGattLongAttributePacketsCorrupted_c);
                    break;

                case L2CAPConfirm_Status_gGattMultipleAttributesOverflow_c:
                    shell_write(gGattMultipleAttributesOverflow_c);
                    break;

                case L2CAPConfirm_Status_gGattUnexpectedReadMultipleResponseLength_c:
                    shell_write(gGattUnexpectedReadMultipleResponseLength_c);
                    break;

                case L2CAPConfirm_Status_gGattInvalidValueLength_c:
                    shell_write(gGattInvalidValueLength_c);
                    break;

                case L2CAPConfirm_Status_gGattServerTimeout_c:
                    shell_write(gGattServerTimeout_c);
                    break;

                case L2CAPConfirm_Status_gGattIndicationAlreadyInProgress_c:
                    shell_write(gGattIndicationAlreadyInProgress_c);
                    break;

                case L2CAPConfirm_Status_gGattClientConfirmationTimeout_c:
                    shell_write(gGattClientConfirmationTimeout_c);
                    break;

                case L2CAPConfirm_Status_gGapAdvDataTooLong_c:
                    shell_write(gGapAdvDataTooLong_c);
                    break;

                case L2CAPConfirm_Status_gGapScanRspDataTooLong_c:
                    shell_write(gGapScanRspDataTooLong_c);
                    break;

                case L2CAPConfirm_Status_gGapDeviceNotBonded_c:
                    shell_write(gGapDeviceNotBonded_c);
                    break;

                case L2CAPConfirm_Status_gDevDbCccdLimitReached_c:
                    shell_write(gDevDbCccdLimitReached_c);
                    break;

                case L2CAPConfirm_Status_gDevDbCccdNotFound_c:
                    shell_write(gDevDbCccdNotFound_c);
                    break;

                case L2CAPConfirm_Status_gGattDbInvalidHandle_c:
                    shell_write(gGattDbInvalidHandle_c);
                    break;

                case L2CAPConfirm_Status_gGattDbCharacteristicNotFound_c:
                    shell_write(gGattDbCharacteristicNotFound_c);
                    break;

                case L2CAPConfirm_Status_gGattDbCccdNotFound_c:
                    shell_write(gGattDbCccdNotFound_c);
                    break;

                case L2CAPConfirm_Status_gGattDbServiceNotFound_c:
                    shell_write(gGattDbServiceNotFound_c);
                    break;

                case L2CAPConfirm_Status_gGattDbDescriptorNotFound_c:
                    shell_write(gGattDbDescriptorNotFound_c);
                    break;

                default:
                    shell_printf("Unrecognized status 0x%04X", container->Data.L2CAPConfirm.Status);
                    break;
            }

            break;

        case 0x4183:
            shell_write("\r\nL2CAPLePsmConnectionRequestIndication");
            break;

        case 0x4184:
            shell_write("\r\nL2CAPLePsmConnectionCompleteIndication");
            break;

        case 0x4185:
            shell_write("\r\nL2CAPLePsmDisconnectNotificationIndication");
            break;

        case 0x4186:
            shell_write("\r\nL2CAPNoPeerCreditsIndication");
            break;

        case 0x4187:
            shell_write("\r\nL2CAPLocalCreditsNotificationIndication");
            break;

        case 0x4188:
            shell_write("\r\nL2CAPLeCbDataIndication");
            break;

        case 0x4480:

            if (mSuppressBleEventPrint)
            {
                break;
            }

            shell_write("\r\nGATTConfirm");
            shell_write(" -> ");

            switch (container->Data.GATTConfirm.Status)
            {
                case GATTConfirm_Status_gBleSuccess_c:
                    shell_write(gBleSuccess_c);
                    break;

                case GATTConfirm_Status_gBleInvalidParameter_c:
                    shell_write(gBleInvalidParameter_c);
                    break;

                case GATTConfirm_Status_gBleOverflow_c:
                    shell_write(gBleOverflow_c);
                    break;

                case GATTConfirm_Status_gBleUnavailable_c:
                    shell_write(gBleUnavailable_c);
                    break;

                case GATTConfirm_Status_gBleFeatureNotSupported_c:
                    shell_write(gBleFeatureNotSupported_c);
                    break;

                case GATTConfirm_Status_gBleOutOfMemory_c:
                    shell_write(gBleOutOfMemory_c);
                    break;

                case GATTConfirm_Status_gBleAlreadyInitialized_c:
                    shell_write(gBleAlreadyInitialized_c);
                    break;

                case GATTConfirm_Status_gBleOsError_c:
                    shell_write(gBleOsError_c);
                    break;

                case GATTConfirm_Status_gBleUnexpectedError_c:
                    shell_write(gBleUnexpectedError_c);
                    break;

                case GATTConfirm_Status_gBleInvalidState_c:
                    shell_write(gBleInvalidState_c);
                    break;

                case GATTConfirm_Status_gHciUnknownHciCommand_c:
                    shell_write(gHciUnknownHciCommand_c);
                    break;

                case GATTConfirm_Status_gHciUnknownConnectionIdentifier_c:
                    shell_write(gHciUnknownConnectionIdentifier_c);
                    break;

                case GATTConfirm_Status_gHciHardwareFailure_c:
                    shell_write(gHciHardwareFailure_c);
                    break;

                case GATTConfirm_Status_gHciPageTimeout_c:
                    shell_write(gHciPageTimeout_c);
                    break;

                case GATTConfirm_Status_gHciAuthenticationFailure_c:
                    shell_write(gHciAuthenticationFailure_c);
                    break;

                case GATTConfirm_Status_gHciPinOrKeyMissing_c:
                    shell_write(gHciPinOrKeyMissing_c);
                    break;

                case GATTConfirm_Status_gHciMemoryCapacityExceeded_c:
                    shell_write(gHciMemoryCapacityExceeded_c);
                    break;

                case GATTConfirm_Status_gHciConnectionTimeout_c:
                    shell_write(gHciConnectionTimeout_c);
                    break;

                case GATTConfirm_Status_gHciConnectionLimitExceeded_c:
                    shell_write(gHciConnectionLimitExceeded_c);
                    break;

                case GATTConfirm_Status_gHciSynchronousConnectionLimitToADeviceExceeded_c:
                    shell_write(gHciSynchronousConnectionLimitToADeviceExceeded_c);
                    break;

                case GATTConfirm_Status_gHciAclConnectionAlreadyExists_c:
                    shell_write(gHciAclConnectionAlreadyExists_c);
                    break;

                case GATTConfirm_Status_gHciCommandDisallowed_c:
                    shell_write(gHciCommandDisallowed_c);
                    break;

                case GATTConfirm_Status_gHciConnectionRejectedDueToLimitedResources_c:
                    shell_write(gHciConnectionRejectedDueToLimitedResources_c);
                    break;

                case GATTConfirm_Status_gHciConnectionRejectedDueToSecurityReasons_c:
                    shell_write(gHciConnectionRejectedDueToSecurityReasons_c);
                    break;

                case GATTConfirm_Status_gHciConnectionRejectedDueToUnacceptableBdAddr_c:
                    shell_write(gHciConnectionRejectedDueToUnacceptableBdAddr_c);
                    break;

                case GATTConfirm_Status_gHciConnectionAcceptTimeoutExceeded_c:
                    shell_write(gHciConnectionAcceptTimeoutExceeded_c);
                    break;

                case GATTConfirm_Status_gHciUnsupportedFeatureOrParameterValue_c:
                    shell_write(gHciUnsupportedFeatureOrParameterValue_c);
                    break;

                case GATTConfirm_Status_gHciInvalidHciCommandParameters_c:
                    shell_write(gHciInvalidHciCommandParameters_c);
                    break;

                case GATTConfirm_Status_gHciRemoteUserTerminatedConnection_c:
                    shell_write(gHciRemoteUserTerminatedConnection_c);
                    break;

                case GATTConfirm_Status_gHciRemoteDeviceTerminatedConnectionLowResources_c:
                    shell_write(gHciRemoteDeviceTerminatedConnectionLowResources_c);
                    break;

                case GATTConfirm_Status_gHciRemoteDeviceTerminatedConnectionPowerOff_c:
                    shell_write(gHciRemoteDeviceTerminatedConnectionPowerOff_c);
                    break;

                case GATTConfirm_Status_gHciConnectionTerminatedByLocalHost_c:
                    shell_write(gHciConnectionTerminatedByLocalHost_c);
                    break;

                case GATTConfirm_Status_gHciRepeatedAttempts_c:
                    shell_write(gHciRepeatedAttempts_c);
                    break;

                case GATTConfirm_Status_gHciPairingNotAllowed_c:
                    shell_write(gHciPairingNotAllowed_c);
                    break;

                case GATTConfirm_Status_gHciUnknownLpmPdu_c:
                    shell_write(gHciUnknownLpmPdu_c);
                    break;

                case GATTConfirm_Status_gHciUnsupportedRemoteFeature_c:
                    shell_write(gHciUnsupportedRemoteFeature_c);
                    break;

                case GATTConfirm_Status_gHciScoOffsetRejected_c:
                    shell_write(gHciScoOffsetRejected_c);
                    break;

                case GATTConfirm_Status_gHciScoIntervalRejected_c:
                    shell_write(gHciScoIntervalRejected_c);
                    break;

                case GATTConfirm_Status_gHciScoAirModeRejected_c:
                    shell_write(gHciScoAirModeRejected_c);
                    break;

                case GATTConfirm_Status_gHciInvalidLpmParameters_c:
                    shell_write(gHciInvalidLpmParameters_c);
                    break;

                case GATTConfirm_Status_gHciUnspecifiedError_c:
                    shell_write(gHciUnspecifiedError_c);
                    break;

                case GATTConfirm_Status_gHciUnsupportedLpmParameterValue_c:
                    shell_write(gHciUnsupportedLpmParameterValue_c);
                    break;

                case GATTConfirm_Status_gHciRoleChangeNotAllowed_c:
                    shell_write(gHciRoleChangeNotAllowed_c);
                    break;

                case GATTConfirm_Status_gHciLLResponseTimeout_c:
                    shell_write(gHciLLResponseTimeout_c);
                    break;

                case GATTConfirm_Status_gHciLmpErrorTransactionCollision_c:
                    shell_write(gHciLmpErrorTransactionCollision_c);
                    break;

                case GATTConfirm_Status_gHciLmpPduNotAllowed_c:
                    shell_write(gHciLmpPduNotAllowed_c);
                    break;

                case GATTConfirm_Status_gHciEncryptionModeNotAcceptable_c:
                    shell_write(gHciEncryptionModeNotAcceptable_c);
                    break;

                case GATTConfirm_Status_gHciLinkKeyCannotBeChanged_c:
                    shell_write(gHciLinkKeyCannotBeChanged_c);
                    break;

                case GATTConfirm_Status_gHciRequestedQosNotSupported_c:
                    shell_write(gHciRequestedQosNotSupported_c);
                    break;

                case GATTConfirm_Status_gHciInstantPassed_c:
                    shell_write(gHciInstantPassed_c);
                    break;

                case GATTConfirm_Status_gHciPairingWithUnitKeyNotSupported_c:
                    shell_write(gHciPairingWithUnitKeyNotSupported_c);
                    break;

                case GATTConfirm_Status_gHciDifferentTransactionCollision_c:
                    shell_write(gHciDifferentTransactionCollision_c);
                    break;

                case GATTConfirm_Status_gHciReserved_0x2B_c:
                    shell_write(gHciReserved_0x2B_c);
                    break;

                case GATTConfirm_Status_gHciQosNotAcceptableParameter_c:
                    shell_write(gHciQosNotAcceptableParameter_c);
                    break;

                case GATTConfirm_Status_gHciQosRejected_c:
                    shell_write(gHciQosRejected_c);
                    break;

                case GATTConfirm_Status_gHciChannelClassificationNotSupported_c:
                    shell_write(gHciChannelClassificationNotSupported_c);
                    break;

                case GATTConfirm_Status_gHciInsufficientSecurity_c:
                    shell_write(gHciInsufficientSecurity_c);
                    break;

                case GATTConfirm_Status_gHciParameterOutOfMandatoryRange_c:
                    shell_write(gHciParameterOutOfMandatoryRange_c);
                    break;

                case GATTConfirm_Status_gHciReserved_0x31_c:
                    shell_write(gHciReserved_0x31_c);
                    break;

                case GATTConfirm_Status_gHciRoleSwitchPending_c:
                    shell_write(gHciRoleSwitchPending_c);
                    break;

                case GATTConfirm_Status_gHciReserved_0x33_c:
                    shell_write(gHciReserved_0x33_c);
                    break;

                case GATTConfirm_Status_gHciReservedSlotViolation_c:
                    shell_write(gHciReservedSlotViolation_c);
                    break;

                case GATTConfirm_Status_gHciRoleSwitchFailed_c:
                    shell_write(gHciRoleSwitchFailed_c);
                    break;

                case GATTConfirm_Status_gHciExtendedInquiryResponseTooLarge_c:
                    shell_write(gHciExtendedInquiryResponseTooLarge_c);
                    break;

                case GATTConfirm_Status_gHciSecureSimplePairingNotSupportedByHost_c:
                    shell_write(gHciSecureSimplePairingNotSupportedByHost_c);
                    break;

                case GATTConfirm_Status_gHciHostBusyPairing_c:
                    shell_write(gHciHostBusyPairing_c);
                    break;

                case GATTConfirm_Status_gHciConnectionRejectedDueToNoSuitableChannelFound_c:
                    shell_write(gHciConnectionRejectedDueToNoSuitableChannelFound_c);
                    break;

                case GATTConfirm_Status_gHciControllerBusy_c:
                    shell_write(gHciControllerBusy_c);
                    break;

                case GATTConfirm_Status_gHciUnacceptableConnectionParameters_c:
                    shell_write(gHciUnacceptableConnectionParameters_c);
                    break;

                case GATTConfirm_Status_gHciDirectedAdvertisingTimeout_c:
                    shell_write(gHciDirectedAdvertisingTimeout_c);
                    break;

                case GATTConfirm_Status_gHciConnectionTerminatedDueToMicFailure_c:
                    shell_write(gHciConnectionTerminatedDueToMicFailure_c);
                    break;

                case GATTConfirm_Status_gHciConnectionFailedToBeEstablished_c:
                    shell_write(gHciConnectionFailedToBeEstablished_c);
                    break;

                case GATTConfirm_Status_gHciMacConnectionFailed_c:
                    shell_write(gHciMacConnectionFailed_c);
                    break;

                case GATTConfirm_Status_gHciCoarseClockAdjustmentRejected_c:
                    shell_write(gHciCoarseClockAdjustmentRejected_c);
                    break;

                case GATTConfirm_Status_gHciAlreadyInit_c:
                    shell_write(gHciAlreadyInit_c);
                    break;

                case GATTConfirm_Status_gHciInvalidParameter_c:
                    shell_write(gHciInvalidParameter_c);
                    break;

                case GATTConfirm_Status_gHciCallbackNotInstalled_c:
                    shell_write(gHciCallbackNotInstalled_c);
                    break;

                case GATTConfirm_Status_gHciCallbackAlreadyInstalled_c:
                    shell_write(gHciCallbackAlreadyInstalled_c);
                    break;

                case GATTConfirm_Status_gHciCommandNotSupported_c:
                    shell_write(gHciCommandNotSupported_c);
                    break;

                case GATTConfirm_Status_gHciEventNotSupported_c:
                    shell_write(gHciEventNotSupported_c);
                    break;

                case GATTConfirm_Status_gHciTransportError_c:
                    shell_write(gHciTransportError_c);
                    break;

                case GATTConfirm_Status_gL2caAlreadyInit_c:
                    shell_write(gL2caAlreadyInit_c);
                    break;

                case GATTConfirm_Status_gL2caInsufficientResources_c:
                    shell_write(gL2caInsufficientResources_c);
                    break;

                case GATTConfirm_Status_gL2caCallbackNotInstalled_c:
                    shell_write(gL2caCallbackNotInstalled_c);
                    break;

                case GATTConfirm_Status_gL2caCallbackAlreadyInstalled_c:
                    shell_write(gL2caCallbackAlreadyInstalled_c);
                    break;

                case GATTConfirm_Status_gL2caLePsmInvalid_c:
                    shell_write(gL2caLePsmInvalid_c);
                    break;

                case GATTConfirm_Status_gL2caLePsmAlreadyRegistered_c:
                    shell_write(gL2caLePsmAlreadyRegistered_c);
                    break;

                case GATTConfirm_Status_gL2caLePsmNotRegistered_c:
                    shell_write(gL2caLePsmNotRegistered_c);
                    break;

                case GATTConfirm_Status_gL2caLePsmInsufficientResources_c:
                    shell_write(gL2caLePsmInsufficientResources_c);
                    break;

                case GATTConfirm_Status_gL2caChannelInvalid_c:
                    shell_write(gL2caChannelInvalid_c);
                    break;

                case GATTConfirm_Status_gL2caChannelClosed_c:
                    shell_write(gL2caChannelClosed_c);
                    break;

                case GATTConfirm_Status_gL2caChannelAlreadyConnected_c:
                    shell_write(gL2caChannelAlreadyConnected_c);
                    break;

                case GATTConfirm_Status_gL2caConnectionParametersRejected_c:
                    shell_write(gL2caConnectionParametersRejected_c);
                    break;

                case GATTConfirm_Status_gL2caChannelBusy_c:
                    shell_write(gL2caChannelBusy_c);
                    break;

                case GATTConfirm_Status_gL2caInvalidParameter_c:
                    shell_write(gL2caInvalidParameter_c);
                    break;

                case GATTConfirm_Status_gL2caError_c:
                    shell_write(gL2caError_c);
                    break;

                case GATTConfirm_Status_gSmNullCBFunction_c:
                    shell_write(gSmNullCBFunction_c);
                    break;

                case GATTConfirm_Status_gSmCommandNotSupported_c:
                    shell_write(gSmCommandNotSupported_c);
                    break;

                case GATTConfirm_Status_gSmUnexpectedCommand_c:
                    shell_write(gSmUnexpectedCommand_c);
                    break;

                case GATTConfirm_Status_gSmInvalidCommandCode_c:
                    shell_write(gSmInvalidCommandCode_c);
                    break;

                case GATTConfirm_Status_gSmInvalidCommandLength_c:
                    shell_write(gSmInvalidCommandLength_c);
                    break;

                case GATTConfirm_Status_gSmInvalidCommandParameter_c:
                    shell_write(gSmInvalidCommandParameter_c);
                    break;

                case GATTConfirm_Status_gSmInvalidDeviceId_c:
                    shell_write(gSmInvalidDeviceId_c);
                    break;

                case GATTConfirm_Status_gSmInvalidInternalOperation_c:
                    shell_write(gSmInvalidInternalOperation_c);
                    break;

                case GATTConfirm_Status_gSmInvalidConnectionHandle_c:
                    shell_write(gSmInvalidConnectionHandle_c);
                    break;

                case GATTConfirm_Status_gSmInproperKeyDistributionField_c:
                    shell_write(gSmInproperKeyDistributionField_c);
                    break;

                case GATTConfirm_Status_gSmUnexpectedKeyType_c:
                    shell_write(gSmUnexpectedKeyType_c);
                    break;

                case GATTConfirm_Status_gSmUnexpectedPairingTerminationReason_c:
                    shell_write(gSmUnexpectedPairingTerminationReason_c);
                    break;

                case GATTConfirm_Status_gSmUnexpectedKeyset_c:
                    shell_write(gSmUnexpectedKeyset_c);
                    break;

                case GATTConfirm_Status_gSmSmpTimeoutOccurred_c:
                    shell_write(gSmSmpTimeoutOccurred_c);
                    break;

                case GATTConfirm_Status_gSmUnknownSmpPacketType_c:
                    shell_write(gSmUnknownSmpPacketType_c);
                    break;

                case GATTConfirm_Status_gSmInvalidSmpPacketLength_c:
                    shell_write(gSmInvalidSmpPacketLength_c);
                    break;

                case GATTConfirm_Status_gSmInvalidSmpPacketParameter_c:
                    shell_write(gSmInvalidSmpPacketParameter_c);
                    break;

                case GATTConfirm_Status_gSmReceivedUnexpectedSmpPacket_c:
                    shell_write(gSmReceivedUnexpectedSmpPacket_c);
                    break;

                case GATTConfirm_Status_gSmReceivedSmpPacketFromUnknownDevice_c:
                    shell_write(gSmReceivedSmpPacketFromUnknownDevice_c);
                    break;

                case GATTConfirm_Status_gSmReceivedUnexpectedHciEvent_c:
                    shell_write(gSmReceivedUnexpectedHciEvent_c);
                    break;

                case GATTConfirm_Status_gSmReceivedHciEventFromUnknownDevice_c:
                    shell_write(gSmReceivedHciEventFromUnknownDevice_c);
                    break;

                case GATTConfirm_Status_gSmInvalidHciEventParameter_c:
                    shell_write(gSmInvalidHciEventParameter_c);
                    break;

                case GATTConfirm_Status_gSmLlConnectionEncryptionInProgress_c:
                    shell_write(gSmLlConnectionEncryptionInProgress_c);
                    break;

                case GATTConfirm_Status_gSmLlConnectionEncryptionFailure_c:
                    shell_write(gSmLlConnectionEncryptionFailure_c);
                    break;

                case GATTConfirm_Status_gSmInsufficientResources_c:
                    shell_write(gSmInsufficientResources_c);
                    break;

                case GATTConfirm_Status_gSmOobDataAddressMismatch_c:
                    shell_write(gSmOobDataAddressMismatch_c);
                    break;

//                case GATTConfirm_Status_gSmSmpPacketReceivedAfterTimeoutOccurred_c:
//                    shell_write(gSmSmpPacketReceivedAfterTimeoutOccurred_c);
//                    break;

                case GATTConfirm_Status_gSmPairingErrorPasskeyEntryFailed_c:
                    shell_write(gSmPairingErrorPasskeyEntryFailed_c);
                    break;

                case GATTConfirm_Status_gSmPairingErrorConfirmValueFailed_c:
                    shell_write(gSmPairingErrorConfirmValueFailed_c);
                    break;

                case GATTConfirm_Status_gSmPairingErrorCommandNotSupported_c:
                    shell_write(gSmPairingErrorCommandNotSupported_c);
                    break;

                case GATTConfirm_Status_gSmPairingErrorInvalidParameters_c:
                    shell_write(gSmPairingErrorInvalidParameters_c);
                    break;

                case GATTConfirm_Status_gSmPairingErrorUnknownReason_c:
                    shell_write(gSmPairingErrorUnknownReason_c);
                    break;

                case GATTConfirm_Status_gSmTbResolvableAddressDoesNotMatchIrk_c:
                    shell_write(gSmTbResolvableAddressDoesNotMatchIrk_c);
                    break;

                case GATTConfirm_Status_gSmTbInvalidDataSignature_c:
                    shell_write(gSmTbInvalidDataSignature_c);
                    break;

                case GATTConfirm_Status_gAttInvalidHandle_c:
                    shell_write(gAttInvalidHandle_c);
                    break;

                case GATTConfirm_Status_gAttReadNotPermitted_c:
                    shell_write(gAttReadNotPermitted_c);
                    break;

                case GATTConfirm_Status_gAttWriteNotPermitted_c:
                    shell_write(gAttWriteNotPermitted_c);
                    break;

                case GATTConfirm_Status_gAttInvalidPdu_c:
                    shell_write(gAttInvalidPdu_c);
                    break;

                case GATTConfirm_Status_gAttInsufficientAuthentication_c:
                    shell_write(gAttInsufficientAuthentication_c);
                    break;

                case GATTConfirm_Status_gAttRequestNotSupported_c:
                    shell_write(gAttRequestNotSupported_c);
                    break;

                case GATTConfirm_Status_gAttInvalidOffset_c:
                    shell_write(gAttInvalidOffset_c);
                    break;

                case GATTConfirm_Status_gAttInsufficientAuthorization_c:
                    shell_write(gAttInsufficientAuthorization_c);
                    break;

                case GATTConfirm_Status_gAttPrepareQueueFull_c:
                    shell_write(gAttPrepareQueueFull_c);
                    break;

                case GATTConfirm_Status_gAttAttributeNotFound_c:
                    shell_write(gAttAttributeNotFound_c);
                    break;

                case GATTConfirm_Status_gAttAttributeNotLong_c:
                    shell_write(gAttAttributeNotLong_c);
                    break;

                case GATTConfirm_Status_gAttInsufficientEncryptionKeySize_c:
                    shell_write(gAttInsufficientEncryptionKeySize_c);
                    break;

                case GATTConfirm_Status_gAttInvalidAttributeValueLength_c:
                    shell_write(gAttInvalidAttributeValueLength_c);
                    break;

                case GATTConfirm_Status_gAttUnlikelyor_c:
                    shell_write(gAttUnlikelyor_c);
                    break;

                case GATTConfirm_Status_gAttInsufficientEncryption_c:
                    shell_write(gAttInsufficientEncryption_c);
                    break;

                case GATTConfirm_Status_gAttUnsupportedGroupType_c:
                    shell_write(gAttUnsupportedGroupType_c);
                    break;

                case GATTConfirm_Status_gAttInsufficientResources_c:
                    shell_write(gAttInsufficientResources_c);
                    break;

                case GATTConfirm_Status_gGattAnotherProcedureInProgress_c:
                    shell_write(gGattAnotherProcedureInProgress_c);
                    break;

                case GATTConfirm_Status_gGattLongAttributePacketsCorrupted_c:
                    shell_write(gGattLongAttributePacketsCorrupted_c);
                    break;

                case GATTConfirm_Status_gGattMultipleAttributesOverflow_c:
                    shell_write(gGattMultipleAttributesOverflow_c);
                    break;

                case GATTConfirm_Status_gGattUnexpectedReadMultipleResponseLength_c:
                    shell_write(gGattUnexpectedReadMultipleResponseLength_c);
                    break;

                case GATTConfirm_Status_gGattInvalidValueLength_c:
                    shell_write(gGattInvalidValueLength_c);
                    break;

                case GATTConfirm_Status_gGattServerTimeout_c:
                    shell_write(gGattServerTimeout_c);
                    break;

                case GATTConfirm_Status_gGattIndicationAlreadyInProgress_c:
                    shell_write(gGattIndicationAlreadyInProgress_c);
                    break;

                case GATTConfirm_Status_gGattClientConfirmationTimeout_c:
                    shell_write(gGattClientConfirmationTimeout_c);
                    break;

                case GATTConfirm_Status_gGapAdvDataTooLong_c:
                    shell_write(gGapAdvDataTooLong_c);
                    break;

                case GATTConfirm_Status_gGapScanRspDataTooLong_c:
                    shell_write(gGapScanRspDataTooLong_c);
                    break;

                case GATTConfirm_Status_gGapDeviceNotBonded_c:
                    shell_write(gGapDeviceNotBonded_c);
                    break;

                case GATTConfirm_Status_gDevDbCccdLimitReached_c:
                    shell_write(gDevDbCccdLimitReached_c);
                    break;

                case GATTConfirm_Status_gDevDbCccdNotFound_c:
                    shell_write(gDevDbCccdNotFound_c);
                    break;

                case GATTConfirm_Status_gGattDbInvalidHandle_c:
                    shell_write(gGattDbInvalidHandle_c);
                    break;

                case GATTConfirm_Status_gGattDbCharacteristicNotFound_c:
                    shell_write(gGattDbCharacteristicNotFound_c);
                    break;

                case GATTConfirm_Status_gGattDbCccdNotFound_c:
                    shell_write(gGattDbCccdNotFound_c);
                    break;

                case GATTConfirm_Status_gGattDbServiceNotFound_c:
                    shell_write(gGattDbServiceNotFound_c);
                    break;

                case GATTConfirm_Status_gGattDbDescriptorNotFound_c:
                    shell_write(gGattDbDescriptorNotFound_c);
                    break;

                default:
                    shell_printf("Unrecognized status 0x%04X", container->Data.GATTConfirm.Status);
                    break;
            }

            break;

        case 0x4481:
            shell_write("\r\nGATTGetMtuIndication");
            break;

        case 0x4482:
            shell_write("\r\nGATTClientProcedureExchangeMtuIndication");
            shell_write(" -> ");

            switch (container->Data.GATTClientProcedureExchangeMtuIndication.ProcedureResult)
            {
                case GATTClientProcedureExchangeMtuIndication_ProcedureResult_gGattProcSuccess_c:
                    shell_write(gGattProcSuccess_c);
                    break;

                case GATTClientProcedureExchangeMtuIndication_ProcedureResult_gProcedureError_c:
                    shell_write(gProcedureError_c);
                    break;

                default:
                    shell_printf("Unrecognized status 0x%02X", container->Data.GATTClientProcedureExchangeMtuIndication.ProcedureResult);
                    break;
            }

            shell_write(" -> ");

            switch (container->Data.GATTClientProcedureExchangeMtuIndication.Error)
            {
                case GATTClientProcedureExchangeMtuIndication_Error_gBleSuccess_c:
                    shell_write(gBleSuccess_c);
                    break;

                case GATTClientProcedureExchangeMtuIndication_Error_gBleInvalidParameter_c:
                    shell_write(gBleInvalidParameter_c);
                    break;

                case GATTClientProcedureExchangeMtuIndication_Error_gBleOverflow_c:
                    shell_write(gBleOverflow_c);
                    break;

                case GATTClientProcedureExchangeMtuIndication_Error_gBleUnavailable_c:
                    shell_write(gBleUnavailable_c);
                    break;

                case GATTClientProcedureExchangeMtuIndication_Error_gBleFeatureNotSupported_c:
                    shell_write(gBleFeatureNotSupported_c);
                    break;

                case GATTClientProcedureExchangeMtuIndication_Error_gBleOutOfMemory_c:
                    shell_write(gBleOutOfMemory_c);
                    break;

                case GATTClientProcedureExchangeMtuIndication_Error_gBleAlreadyInitialized_c:
                    shell_write(gBleAlreadyInitialized_c);
                    break;

                case GATTClientProcedureExchangeMtuIndication_Error_gBleOsError_c:
                    shell_write(gBleOsc);
                    break;

                case GATTClientProcedureExchangeMtuIndication_Error_gBleUnexpectedError_c:
                    shell_write(gBleUnexpectedc);
                    break;

                case GATTClientProcedureExchangeMtuIndication_Error_gBleInvalidState_c:
                    shell_write(gBleInvalidState_c);
                    break;

                case GATTClientProcedureExchangeMtuIndication_Error_gHciUnknownHciCommand_c:
                    shell_write(gHciUnknownHciCommand_c);
                    break;

                case GATTClientProcedureExchangeMtuIndication_Error_gHciUnknownConnectionIdentifier_c:
                    shell_write(gHciUnknownConnectionIdentifier_c);
                    break;

                case GATTClientProcedureExchangeMtuIndication_Error_gHciHardwareFailure_c:
                    shell_write(gHciHardwareFailure_c);
                    break;

                case GATTClientProcedureExchangeMtuIndication_Error_gHciPageTimeout_c:
                    shell_write(gHciPageTimeout_c);
                    break;

                case GATTClientProcedureExchangeMtuIndication_Error_gHciAuthenticationFailure_c:
                    shell_write(gHciAuthenticationFailure_c);
                    break;

                case GATTClientProcedureExchangeMtuIndication_Error_gHciPinOrKeyMissing_c:
                    shell_write(gHciPinOrKeyMissing_c);
                    break;

                case GATTClientProcedureExchangeMtuIndication_Error_gHciMemoryCapacityExceeded_c:
                    shell_write(gHciMemoryCapacityExceeded_c);
                    break;

                case GATTClientProcedureExchangeMtuIndication_Error_gHciConnectionTimeout_c:
                    shell_write(gHciConnectionTimeout_c);
                    break;

                case GATTClientProcedureExchangeMtuIndication_Error_gHciConnectionLimitExceeded_c:
                    shell_write(gHciConnectionLimitExceeded_c);
                    break;

                case GATTClientProcedureExchangeMtuIndication_Error_gHciSynchronousConnectionLimitToADeviceExceeded_c:
                    shell_write(gHciSynchronousConnectionLimitToADeviceExceeded_c);
                    break;

                case GATTClientProcedureExchangeMtuIndication_Error_gHciAclConnectionAlreadyExists_c:
                    shell_write(gHciAclConnectionAlreadyExists_c);
                    break;

                case GATTClientProcedureExchangeMtuIndication_Error_gHciCommandDisallowed_c:
                    shell_write(gHciCommandDisallowed_c);
                    break;

                case GATTClientProcedureExchangeMtuIndication_Error_gHciConnectionRejectedDueToLimitedResources_c:
                    shell_write(gHciConnectionRejectedDueToLimitedResources_c);
                    break;

                case GATTClientProcedureExchangeMtuIndication_Error_gHciConnectionRejectedDueToSecurityReasons_c:
                    shell_write(gHciConnectionRejectedDueToSecurityReasons_c);
                    break;

                case GATTClientProcedureExchangeMtuIndication_Error_gHciConnectionRejectedDueToUnacceptableBdAddr_c:
                    shell_write(gHciConnectionRejectedDueToUnacceptableBdAddr_c);
                    break;

                case GATTClientProcedureExchangeMtuIndication_Error_gHciConnectionAcceptTimeoutExceeded_c:
                    shell_write(gHciConnectionAcceptTimeoutExceeded_c);
                    break;

                case GATTClientProcedureExchangeMtuIndication_Error_gHciUnsupportedFeatureOrParameterValue_c:
                    shell_write(gHciUnsupportedFeatureOrParameterValue_c);
                    break;

                case GATTClientProcedureExchangeMtuIndication_Error_gHciInvalidHciCommandParameters_c:
                    shell_write(gHciInvalidHciCommandParameters_c);
                    break;

                case GATTClientProcedureExchangeMtuIndication_Error_gHciRemoteUserTerminatedConnection_c:
                    shell_write(gHciRemoteUserTerminatedConnection_c);
                    break;

                case GATTClientProcedureExchangeMtuIndication_Error_gHciRemoteDeviceTerminatedConnectionLowResources_c:
                    shell_write(gHciRemoteDeviceTerminatedConnectionLowResources_c);
                    break;

                case GATTClientProcedureExchangeMtuIndication_Error_gHciRemoteDeviceTerminatedConnectionPowerOff_c:
                    shell_write(gHciRemoteDeviceTerminatedConnectionPowerOff_c);
                    break;

                case GATTClientProcedureExchangeMtuIndication_Error_gHciConnectionTerminatedByLocalHost_c:
                    shell_write(gHciConnectionTerminatedByLocalHost_c);
                    break;

                case GATTClientProcedureExchangeMtuIndication_Error_gHciRepeatedAttempts_c:
                    shell_write(gHciRepeatedAttempts_c);
                    break;

                case GATTClientProcedureExchangeMtuIndication_Error_gHciPairingNotAllowed_c:
                    shell_write(gHciPairingNotAllowed_c);
                    break;

                case GATTClientProcedureExchangeMtuIndication_Error_gHciUnknownLpmPdu_c:
                    shell_write(gHciUnknownLpmPdu_c);
                    break;

                case GATTClientProcedureExchangeMtuIndication_Error_gHciUnsupportedRemoteFeature_c:
                    shell_write(gHciUnsupportedRemoteFeature_c);
                    break;

                case GATTClientProcedureExchangeMtuIndication_Error_gHciScoOffsetRejected_c:
                    shell_write(gHciScoOffsetRejected_c);
                    break;

                case GATTClientProcedureExchangeMtuIndication_Error_gHciScoIntervalRejected_c:
                    shell_write(gHciScoIntervalRejected_c);
                    break;

                case GATTClientProcedureExchangeMtuIndication_Error_gHciScoAirModeRejected_c:
                    shell_write(gHciScoAirModeRejected_c);
                    break;

                case GATTClientProcedureExchangeMtuIndication_Error_gHciInvalidLpmParameters_c:
                    shell_write(gHciInvalidLpmParameters_c);
                    break;

                case GATTClientProcedureExchangeMtuIndication_Error_gHciUnspecifiedError_c:
                    shell_write(gHciUnspecifiedc);
                    break;

                case GATTClientProcedureExchangeMtuIndication_Error_gHciUnsupportedLpmParameterValue_c:
                    shell_write(gHciUnsupportedLpmParameterValue_c);
                    break;

                case GATTClientProcedureExchangeMtuIndication_Error_gHciRoleChangeNotAllowed_c:
                    shell_write(gHciRoleChangeNotAllowed_c);
                    break;

                case GATTClientProcedureExchangeMtuIndication_Error_gHciLLResponseTimeout_c:
                    shell_write(gHciLLResponseTimeout_c);
                    break;

                case GATTClientProcedureExchangeMtuIndication_Error_gHciLmpErrorTransactionCollision_c:
                    shell_write(gHciLmpErrorTransactionCollision_c);
                    break;

                case GATTClientProcedureExchangeMtuIndication_Error_gHciLmpPduNotAllowed_c:
                    shell_write(gHciLmpPduNotAllowed_c);
                    break;

                case GATTClientProcedureExchangeMtuIndication_Error_gHciEncryptionModeNotAcceptable_c:
                    shell_write(gHciEncryptionModeNotAcceptable_c);
                    break;

                case GATTClientProcedureExchangeMtuIndication_Error_gHciLinkKeyCannotBeChanged_c:
                    shell_write(gHciLinkKeyCannotBeChanged_c);
                    break;

                case GATTClientProcedureExchangeMtuIndication_Error_gHciRequestedQosNotSupported_c:
                    shell_write(gHciRequestedQosNotSupported_c);
                    break;

                case GATTClientProcedureExchangeMtuIndication_Error_gHciInstantPassed_c:
                    shell_write(gHciInstantPassed_c);
                    break;

                case GATTClientProcedureExchangeMtuIndication_Error_gHciPairingWithUnitKeyNotSupported_c:
                    shell_write(gHciPairingWithUnitKeyNotSupported_c);
                    break;

                case GATTClientProcedureExchangeMtuIndication_Error_gHciDifferentTransactionCollision_c:
                    shell_write(gHciDifferentTransactionCollision_c);
                    break;

                case GATTClientProcedureExchangeMtuIndication_Error_gHciReserved_0x2B_c:
                    shell_write(gHciReserved_0x2B_c);
                    break;

                case GATTClientProcedureExchangeMtuIndication_Error_gHciQosNotAcceptableParameter_c:
                    shell_write(gHciQosNotAcceptableParameter_c);
                    break;

                case GATTClientProcedureExchangeMtuIndication_Error_gHciQosRejected_c:
                    shell_write(gHciQosRejected_c);
                    break;

                case GATTClientProcedureExchangeMtuIndication_Error_gHciChannelClassificationNotSupported_c:
                    shell_write(gHciChannelClassificationNotSupported_c);
                    break;

                case GATTClientProcedureExchangeMtuIndication_Error_gHciInsufficientSecurity_c:
                    shell_write(gHciInsufficientSecurity_c);
                    break;

                case GATTClientProcedureExchangeMtuIndication_Error_gHciParameterOutOfMandatoryRange_c:
                    shell_write(gHciParameterOutOfMandatoryRange_c);
                    break;

                case GATTClientProcedureExchangeMtuIndication_Error_gHciReserved_0x31_c:
                    shell_write(gHciReserved_0x31_c);
                    break;

                case GATTClientProcedureExchangeMtuIndication_Error_gHciRoleSwitchPending_c:
                    shell_write(gHciRoleSwitchPending_c);
                    break;

                case GATTClientProcedureExchangeMtuIndication_Error_gHciReserved_0x33_c:
                    shell_write(gHciReserved_0x33_c);
                    break;

                case GATTClientProcedureExchangeMtuIndication_Error_gHciReservedSlotViolation_c:
                    shell_write(gHciReservedSlotViolation_c);
                    break;

                case GATTClientProcedureExchangeMtuIndication_Error_gHciRoleSwitchFailed_c:
                    shell_write(gHciRoleSwitchFailed_c);
                    break;

                case GATTClientProcedureExchangeMtuIndication_Error_gHciExtendedInquiryResponseTooLarge_c:
                    shell_write(gHciExtendedInquiryResponseTooLarge_c);
                    break;

                case GATTClientProcedureExchangeMtuIndication_Error_gHciSecureSimplePairingNotSupportedByHost_c:
                    shell_write(gHciSecureSimplePairingNotSupportedByHost_c);
                    break;

                case GATTClientProcedureExchangeMtuIndication_Error_gHciHostBusyPairing_c:
                    shell_write(gHciHostBusyPairing_c);
                    break;

                case GATTClientProcedureExchangeMtuIndication_Error_gHciConnectionRejectedDueToNoSuitableChannelFound_c:
                    shell_write(gHciConnectionRejectedDueToNoSuitableChannelFound_c);
                    break;

                case GATTClientProcedureExchangeMtuIndication_Error_gHciControllerBusy_c:
                    shell_write(gHciControllerBusy_c);
                    break;

                case GATTClientProcedureExchangeMtuIndication_Error_gHciUnacceptableConnectionParameters_c:
                    shell_write(gHciUnacceptableConnectionParameters_c);
                    break;

                case GATTClientProcedureExchangeMtuIndication_Error_gHciDirectedAdvertisingTimeout_c:
                    shell_write(gHciDirectedAdvertisingTimeout_c);
                    break;

                case GATTClientProcedureExchangeMtuIndication_Error_gHciConnectionTerminatedDueToMicFailure_c:
                    shell_write(gHciConnectionTerminatedDueToMicFailure_c);
                    break;

                case GATTClientProcedureExchangeMtuIndication_Error_gHciConnectionFailedToBeEstablished_c:
                    shell_write(gHciConnectionFailedToBeEstablished_c);
                    break;

                case GATTClientProcedureExchangeMtuIndication_Error_gHciMacConnectionFailed_c:
                    shell_write(gHciMacConnectionFailed_c);
                    break;

                case GATTClientProcedureExchangeMtuIndication_Error_gHciCoarseClockAdjustmentRejected_c:
                    shell_write(gHciCoarseClockAdjustmentRejected_c);
                    break;

                case GATTClientProcedureExchangeMtuIndication_Error_gHciAlreadyInit_c:
                    shell_write(gHciAlreadyInit_c);
                    break;

                case GATTClientProcedureExchangeMtuIndication_Error_gHciInvalidParameter_c:
                    shell_write(gHciInvalidParameter_c);
                    break;

                case GATTClientProcedureExchangeMtuIndication_Error_gHciCallbackNotInstalled_c:
                    shell_write(gHciCallbackNotInstalled_c);
                    break;

                case GATTClientProcedureExchangeMtuIndication_Error_gHciCallbackAlreadyInstalled_c:
                    shell_write(gHciCallbackAlreadyInstalled_c);
                    break;

                case GATTClientProcedureExchangeMtuIndication_Error_gHciCommandNotSupported_c:
                    shell_write(gHciCommandNotSupported_c);
                    break;

                case GATTClientProcedureExchangeMtuIndication_Error_gHciEventNotSupported_c:
                    shell_write(gHciEventNotSupported_c);
                    break;

                case GATTClientProcedureExchangeMtuIndication_Error_gHciTransportError_c:
                    shell_write(gHciTransportc);
                    break;

                case GATTClientProcedureExchangeMtuIndication_Error_gL2caAlreadyInit_c:
                    shell_write(gL2caAlreadyInit_c);
                    break;

                case GATTClientProcedureExchangeMtuIndication_Error_gL2caInsufficientResources_c:
                    shell_write(gL2caInsufficientResources_c);
                    break;

                case GATTClientProcedureExchangeMtuIndication_Error_gL2caCallbackNotInstalled_c:
                    shell_write(gL2caCallbackNotInstalled_c);
                    break;

                case GATTClientProcedureExchangeMtuIndication_Error_gL2caCallbackAlreadyInstalled_c:
                    shell_write(gL2caCallbackAlreadyInstalled_c);
                    break;

                case GATTClientProcedureExchangeMtuIndication_Error_gL2caLePsmInvalid_c:
                    shell_write(gL2caLePsmInvalid_c);
                    break;

                case GATTClientProcedureExchangeMtuIndication_Error_gL2caLePsmAlreadyRegistered_c:
                    shell_write(gL2caLePsmAlreadyRegistered_c);
                    break;

                case GATTClientProcedureExchangeMtuIndication_Error_gL2caLePsmNotRegistered_c:
                    shell_write(gL2caLePsmNotRegistered_c);
                    break;

                case GATTClientProcedureExchangeMtuIndication_Error_gL2caLePsmInsufficientResources_c:
                    shell_write(gL2caLePsmInsufficientResources_c);
                    break;

                case GATTClientProcedureExchangeMtuIndication_Error_gL2caChannelInvalid_c:
                    shell_write(gL2caChannelInvalid_c);
                    break;

                case GATTClientProcedureExchangeMtuIndication_Error_gL2caChannelClosed_c:
                    shell_write(gL2caChannelClosed_c);
                    break;

                case GATTClientProcedureExchangeMtuIndication_Error_gL2caChannelAlreadyConnected_c:
                    shell_write(gL2caChannelAlreadyConnected_c);
                    break;

                case GATTClientProcedureExchangeMtuIndication_Error_gL2caConnectionParametersRejected_c:
                    shell_write(gL2caConnectionParametersRejected_c);
                    break;

                case GATTClientProcedureExchangeMtuIndication_Error_gL2caChannelBusy_c:
                    shell_write(gL2caChannelBusy_c);
                    break;

                case GATTClientProcedureExchangeMtuIndication_Error_gL2caInvalidParameter_c:
                    shell_write(gL2caInvalidParameter_c);
                    break;

                case GATTClientProcedureExchangeMtuIndication_Error_gL2caError_c:
                    shell_write(gL2cac);
                    break;

                case GATTClientProcedureExchangeMtuIndication_Error_gSmNullCBFunction_c:
                    shell_write(gSmNullCBFunction_c);
                    break;

                case GATTClientProcedureExchangeMtuIndication_Error_gSmCommandNotSupported_c:
                    shell_write(gSmCommandNotSupported_c);
                    break;

                case GATTClientProcedureExchangeMtuIndication_Error_gSmUnexpectedCommand_c:
                    shell_write(gSmUnexpectedCommand_c);
                    break;

                case GATTClientProcedureExchangeMtuIndication_Error_gSmInvalidCommandCode_c:
                    shell_write(gSmInvalidCommandCode_c);
                    break;

                case GATTClientProcedureExchangeMtuIndication_Error_gSmInvalidCommandLength_c:
                    shell_write(gSmInvalidCommandLength_c);
                    break;

                case GATTClientProcedureExchangeMtuIndication_Error_gSmInvalidCommandParameter_c:
                    shell_write(gSmInvalidCommandParameter_c);
                    break;

                case GATTClientProcedureExchangeMtuIndication_Error_gSmInvalidDeviceId_c:
                    shell_write(gSmInvalidDeviceId_c);
                    break;

                case GATTClientProcedureExchangeMtuIndication_Error_gSmInvalidInternalOperation_c:
                    shell_write(gSmInvalidInternalOperation_c);
                    break;

                case GATTClientProcedureExchangeMtuIndication_Error_gSmInvalidConnectionHandle_c:
                    shell_write(gSmInvalidConnectionHandle_c);
                    break;

                case GATTClientProcedureExchangeMtuIndication_Error_gSmInproperKeyDistributionField_c:
                    shell_write(gSmInproperKeyDistributionField_c);
                    break;

                case GATTClientProcedureExchangeMtuIndication_Error_gSmUnexpectedKeyType_c:
                    shell_write(gSmUnexpectedKeyType_c);
                    break;

                case GATTClientProcedureExchangeMtuIndication_Error_gSmUnexpectedPairingTerminationReason_c:
                    shell_write(gSmUnexpectedPairingTerminationReason_c);
                    break;

                case GATTClientProcedureExchangeMtuIndication_Error_gSmUnexpectedKeyset_c:
                    shell_write(gSmUnexpectedKeyset_c);
                    break;

                case GATTClientProcedureExchangeMtuIndication_Error_gSmSmpTimeoutOccurred_c:
                    shell_write(gSmSmpTimeoutOccurred_c);
                    break;

                case GATTClientProcedureExchangeMtuIndication_Error_gSmUnknownSmpPacketType_c:
                    shell_write(gSmUnknownSmpPacketType_c);
                    break;

                case GATTClientProcedureExchangeMtuIndication_Error_gSmInvalidSmpPacketLength_c:
                    shell_write(gSmInvalidSmpPacketLength_c);
                    break;

                case GATTClientProcedureExchangeMtuIndication_Error_gSmInvalidSmpPacketParameter_c:
                    shell_write(gSmInvalidSmpPacketParameter_c);
                    break;

                case GATTClientProcedureExchangeMtuIndication_Error_gSmReceivedUnexpectedSmpPacket_c:
                    shell_write(gSmReceivedUnexpectedSmpPacket_c);
                    break;

                case GATTClientProcedureExchangeMtuIndication_Error_gSmReceivedSmpPacketFromUnknownDevice_c:
                    shell_write(gSmReceivedSmpPacketFromUnknownDevice_c);
                    break;

                case GATTClientProcedureExchangeMtuIndication_Error_gSmReceivedUnexpectedHciEvent_c:
                    shell_write(gSmReceivedUnexpectedHciEvent_c);
                    break;

                case GATTClientProcedureExchangeMtuIndication_Error_gSmReceivedHciEventFromUnknownDevice_c:
                    shell_write(gSmReceivedHciEventFromUnknownDevice_c);
                    break;

                case GATTClientProcedureExchangeMtuIndication_Error_gSmInvalidHciEventParameter_c:
                    shell_write(gSmInvalidHciEventParameter_c);
                    break;

                case GATTClientProcedureExchangeMtuIndication_Error_gSmLlConnectionEncryptionInProgress_c:
                    shell_write(gSmLlConnectionEncryptionInProgress_c);
                    break;

                case GATTClientProcedureExchangeMtuIndication_Error_gSmLlConnectionEncryptionFailure_c:
                    shell_write(gSmLlConnectionEncryptionFailure_c);
                    break;

                case GATTClientProcedureExchangeMtuIndication_Error_gSmInsufficientResources_c:
                    shell_write(gSmInsufficientResources_c);
                    break;

                case GATTClientProcedureExchangeMtuIndication_Error_gSmOobDataAddressMismatch_c:
                    shell_write(gSmOobDataAddressMismatch_c);
                    break;

//                case GATTClientProcedureExchangeMtuIndication_Error_gSmSmpPacketReceivedAfterTimeoutOccurred_c:
//                    shell_write(gSmSmpPacketReceivedAfterTimeoutOccurred_c);
//                    break;

                case GATTClientProcedureExchangeMtuIndication_Error_gSmPairingErrorPasskeyEntryFailed_c:
                    shell_write(gSmPairingErrorPasskeyEntryFailed_c);
                    break;

                case GATTClientProcedureExchangeMtuIndication_Error_gSmPairingErrorConfirmValueFailed_c:
                    shell_write(gSmPairingErrorConfirmValueFailed_c);
                    break;

                case GATTClientProcedureExchangeMtuIndication_Error_gSmPairingErrorCommandNotSupported_c:
                    shell_write(gSmPairingErrorCommandNotSupported_c);
                    break;

                case GATTClientProcedureExchangeMtuIndication_Error_gSmPairingErrorInvalidParameters_c:
                    shell_write(gSmPairingErrorInvalidParameters_c);
                    break;

                case GATTClientProcedureExchangeMtuIndication_Error_gSmPairingErrorUnknownReason_c:
                    shell_write(gSmPairingErrorUnknownReason_c);
                    break;

                case GATTClientProcedureExchangeMtuIndication_Error_gSmTbResolvableAddressDoesNotMatchIrk_c:
                    shell_write(gSmTbResolvableAddressDoesNotMatchIrk_c);
                    break;

                case GATTClientProcedureExchangeMtuIndication_Error_gSmTbInvalidDataSignature_c:
                    shell_write(gSmTbInvalidDataSignature_c);
                    break;

                case GATTClientProcedureExchangeMtuIndication_Error_gAttInvalidHandle_c:
                    shell_write(gAttInvalidHandle_c);
                    break;

                case GATTClientProcedureExchangeMtuIndication_Error_gAttReadNotPermitted_c:
                    shell_write(gAttReadNotPermitted_c);
                    break;

                case GATTClientProcedureExchangeMtuIndication_Error_gAttWriteNotPermitted_c:
                    shell_write(gAttWriteNotPermitted_c);
                    break;

                case GATTClientProcedureExchangeMtuIndication_Error_gAttInvalidPdu_c:
                    shell_write(gAttInvalidPdu_c);
                    break;

                case GATTClientProcedureExchangeMtuIndication_Error_gAttInsufficientAuthentication_c:
                    shell_write(gAttInsufficientAuthentication_c);
                    break;

                case GATTClientProcedureExchangeMtuIndication_Error_gAttRequestNotSupported_c:
                    shell_write(gAttRequestNotSupported_c);
                    break;

                case GATTClientProcedureExchangeMtuIndication_Error_gAttInvalidOffset_c:
                    shell_write(gAttInvalidOffset_c);
                    break;

                case GATTClientProcedureExchangeMtuIndication_Error_gAttInsufficientAuthorization_c:
                    shell_write(gAttInsufficientAuthorization_c);
                    break;

                case GATTClientProcedureExchangeMtuIndication_Error_gAttPrepareQueueFull_c:
                    shell_write(gAttPrepareQueueFull_c);
                    break;

                case GATTClientProcedureExchangeMtuIndication_Error_gAttAttributeNotFound_c:
                    shell_write(gAttAttributeNotFound_c);
                    break;

                case GATTClientProcedureExchangeMtuIndication_Error_gAttAttributeNotLong_c:
                    shell_write(gAttAttributeNotLong_c);
                    break;

                case GATTClientProcedureExchangeMtuIndication_Error_gAttInsufficientEncryptionKeySize_c:
                    shell_write(gAttInsufficientEncryptionKeySize_c);
                    break;

                case GATTClientProcedureExchangeMtuIndication_Error_gAttInvalidAttributeValueLength_c:
                    shell_write(gAttInvalidAttributeValueLength_c);
                    break;

                case GATTClientProcedureExchangeMtuIndication_Error_gAttUnlikelyor_c:
                    shell_write(gAttUnlikelyor_c);
                    break;

                case GATTClientProcedureExchangeMtuIndication_Error_gAttInsufficientEncryption_c:
                    shell_write(gAttInsufficientEncryption_c);
                    break;

                case GATTClientProcedureExchangeMtuIndication_Error_gAttUnsupportedGroupType_c:
                    shell_write(gAttUnsupportedGroupType_c);
                    break;

                case GATTClientProcedureExchangeMtuIndication_Error_gAttInsufficientResources_c:
                    shell_write(gAttInsufficientResources_c);
                    break;

                case GATTClientProcedureExchangeMtuIndication_Error_gGattAnotherProcedureInProgress_c:
                    shell_write(gGattAnotherProcedureInProgress_c);
                    break;

                case GATTClientProcedureExchangeMtuIndication_Error_gGattLongAttributePacketsCorrupted_c:
                    shell_write(gGattLongAttributePacketsCorrupted_c);
                    break;

                case GATTClientProcedureExchangeMtuIndication_Error_gGattMultipleAttributesOverflow_c:
                    shell_write(gGattMultipleAttributesOverflow_c);
                    break;

                case GATTClientProcedureExchangeMtuIndication_Error_gGattUnexpectedReadMultipleResponseLength_c:
                    shell_write(gGattUnexpectedReadMultipleResponseLength_c);
                    break;

                case GATTClientProcedureExchangeMtuIndication_Error_gGattInvalidValueLength_c:
                    shell_write(gGattInvalidValueLength_c);
                    break;

                case GATTClientProcedureExchangeMtuIndication_Error_gGattServerTimeout_c:
                    shell_write(gGattServerTimeout_c);
                    break;

                case GATTClientProcedureExchangeMtuIndication_Error_gGattIndicationAlreadyInProgress_c:
                    shell_write(gGattIndicationAlreadyInProgress_c);
                    break;

                case GATTClientProcedureExchangeMtuIndication_Error_gGattClientConfirmationTimeout_c:
                    shell_write(gGattClientConfirmationTimeout_c);
                    break;

                case GATTClientProcedureExchangeMtuIndication_Error_gGapAdvDataTooLong_c:
                    shell_write(gGapAdvDataTooLong_c);
                    break;

                case GATTClientProcedureExchangeMtuIndication_Error_gGapScanRspDataTooLong_c:
                    shell_write(gGapScanRspDataTooLong_c);
                    break;

                case GATTClientProcedureExchangeMtuIndication_Error_gGapDeviceNotBonded_c:
                    shell_write(gGapDeviceNotBonded_c);
                    break;

                case GATTClientProcedureExchangeMtuIndication_Error_gDevDbCccdLimitReached_c:
                    shell_write(gDevDbCccdLimitReached_c);
                    break;

                case GATTClientProcedureExchangeMtuIndication_Error_gDevDbCccdNotFound_c:
                    shell_write(gDevDbCccdNotFound_c);
                    break;

                case GATTClientProcedureExchangeMtuIndication_Error_gGattDbInvalidHandle_c:
                    shell_write(gGattDbInvalidHandle_c);
                    break;

                case GATTClientProcedureExchangeMtuIndication_Error_gGattDbCharacteristicNotFound_c:
                    shell_write(gGattDbCharacteristicNotFound_c);
                    break;

                case GATTClientProcedureExchangeMtuIndication_Error_gGattDbCccdNotFound_c:
                    shell_write(gGattDbCccdNotFound_c);
                    break;

                case GATTClientProcedureExchangeMtuIndication_Error_gGattDbServiceNotFound_c:
                    shell_write(gGattDbServiceNotFound_c);
                    break;

                case GATTClientProcedureExchangeMtuIndication_Error_gGattDbDescriptorNotFound_c:
                    shell_write(gGattDbDescriptorNotFound_c);
                    break;

                default:
                    shell_printf("Unrecognized status 0x%04X", container->Data.GATTClientProcedureExchangeMtuIndication.Error);
                    break;
            }

            break;

        case 0x4483:
            shell_write("\r\nGATTClientProcedureDiscoverAllPrimaryServicesIndication");
            shell_write(" -> ");

            switch (container->Data.GATTClientProcedureDiscoverAllPrimaryServicesIndication.ProcedureResult)
            {
                case GATTClientProcedureDiscoverAllPrimaryServicesIndication_ProcedureResult_gGattProcSuccess_c:
                    shell_write(gGattProcSuccess_c);
                    break;

                case GATTClientProcedureDiscoverAllPrimaryServicesIndication_ProcedureResult_gProcedureError_c:
                    shell_write(gProcedureError_c);
                    break;

                default:
                    shell_printf("Unrecognized status 0x%02X", container->Data.GATTClientProcedureDiscoverAllPrimaryServicesIndication.ProcedureResult);
                    break;
            }

            shell_write(" -> ");

            switch (container->Data.GATTClientProcedureDiscoverAllPrimaryServicesIndication.Error)
            {
                case GATTClientProcedureDiscoverAllPrimaryServicesIndication_Error_gBleSuccess_c:
                    shell_write(gBleSuccess_c);
                    SHELL_PrintDiscoveredPrimaryServices(&(container->Data.GATTClientProcedureDiscoverAllPrimaryServicesIndication));
                    break;

                case GATTClientProcedureDiscoverAllPrimaryServicesIndication_Error_gBleInvalidParameter_c:
                    shell_write(gBleInvalidParameter_c);
                    break;

                case GATTClientProcedureDiscoverAllPrimaryServicesIndication_Error_gBleOverflow_c:
                    shell_write(gBleOverflow_c);
                    break;

                case GATTClientProcedureDiscoverAllPrimaryServicesIndication_Error_gBleUnavailable_c:
                    shell_write(gBleUnavailable_c);
                    break;

                case GATTClientProcedureDiscoverAllPrimaryServicesIndication_Error_gBleFeatureNotSupported_c:
                    shell_write(gBleFeatureNotSupported_c);
                    break;

                case GATTClientProcedureDiscoverAllPrimaryServicesIndication_Error_gBleOutOfMemory_c:
                    shell_write(gBleOutOfMemory_c);
                    break;

                case GATTClientProcedureDiscoverAllPrimaryServicesIndication_Error_gBleAlreadyInitialized_c:
                    shell_write(gBleAlreadyInitialized_c);
                    break;

                case GATTClientProcedureDiscoverAllPrimaryServicesIndication_Error_gBleOsError_c:
                    shell_write(gBleOsc);
                    break;

                case GATTClientProcedureDiscoverAllPrimaryServicesIndication_Error_gBleUnexpectedError_c:
                    shell_write(gBleUnexpectedc);
                    break;

                case GATTClientProcedureDiscoverAllPrimaryServicesIndication_Error_gBleInvalidState_c:
                    shell_write(gBleInvalidState_c);
                    break;

                case GATTClientProcedureDiscoverAllPrimaryServicesIndication_Error_gHciUnknownHciCommand_c:
                    shell_write(gHciUnknownHciCommand_c);
                    break;

                case GATTClientProcedureDiscoverAllPrimaryServicesIndication_Error_gHciUnknownConnectionIdentifier_c:
                    shell_write(gHciUnknownConnectionIdentifier_c);
                    break;

                case GATTClientProcedureDiscoverAllPrimaryServicesIndication_Error_gHciHardwareFailure_c:
                    shell_write(gHciHardwareFailure_c);
                    break;

                case GATTClientProcedureDiscoverAllPrimaryServicesIndication_Error_gHciPageTimeout_c:
                    shell_write(gHciPageTimeout_c);
                    break;

                case GATTClientProcedureDiscoverAllPrimaryServicesIndication_Error_gHciAuthenticationFailure_c:
                    shell_write(gHciAuthenticationFailure_c);
                    break;

                case GATTClientProcedureDiscoverAllPrimaryServicesIndication_Error_gHciPinOrKeyMissing_c:
                    shell_write(gHciPinOrKeyMissing_c);
                    break;

                case GATTClientProcedureDiscoverAllPrimaryServicesIndication_Error_gHciMemoryCapacityExceeded_c:
                    shell_write(gHciMemoryCapacityExceeded_c);
                    break;

                case GATTClientProcedureDiscoverAllPrimaryServicesIndication_Error_gHciConnectionTimeout_c:
                    shell_write(gHciConnectionTimeout_c);
                    break;

                case GATTClientProcedureDiscoverAllPrimaryServicesIndication_Error_gHciConnectionLimitExceeded_c:
                    shell_write(gHciConnectionLimitExceeded_c);
                    break;

                case GATTClientProcedureDiscoverAllPrimaryServicesIndication_Error_gHciSynchronousConnectionLimitToADeviceExceeded_c:
                    shell_write(gHciSynchronousConnectionLimitToADeviceExceeded_c);
                    break;

                case GATTClientProcedureDiscoverAllPrimaryServicesIndication_Error_gHciAclConnectionAlreadyExists_c:
                    shell_write(gHciAclConnectionAlreadyExists_c);
                    break;

                case GATTClientProcedureDiscoverAllPrimaryServicesIndication_Error_gHciCommandDisallowed_c:
                    shell_write(gHciCommandDisallowed_c);
                    break;

                case GATTClientProcedureDiscoverAllPrimaryServicesIndication_Error_gHciConnectionRejectedDueToLimitedResources_c:
                    shell_write(gHciConnectionRejectedDueToLimitedResources_c);
                    break;

                case GATTClientProcedureDiscoverAllPrimaryServicesIndication_Error_gHciConnectionRejectedDueToSecurityReasons_c:
                    shell_write(gHciConnectionRejectedDueToSecurityReasons_c);
                    break;

                case GATTClientProcedureDiscoverAllPrimaryServicesIndication_Error_gHciConnectionRejectedDueToUnacceptableBdAddr_c:
                    shell_write(gHciConnectionRejectedDueToUnacceptableBdAddr_c);
                    break;

                case GATTClientProcedureDiscoverAllPrimaryServicesIndication_Error_gHciConnectionAcceptTimeoutExceeded_c:
                    shell_write(gHciConnectionAcceptTimeoutExceeded_c);
                    break;

                case GATTClientProcedureDiscoverAllPrimaryServicesIndication_Error_gHciUnsupportedFeatureOrParameterValue_c:
                    shell_write(gHciUnsupportedFeatureOrParameterValue_c);
                    break;

                case GATTClientProcedureDiscoverAllPrimaryServicesIndication_Error_gHciInvalidHciCommandParameters_c:
                    shell_write(gHciInvalidHciCommandParameters_c);
                    break;

                case GATTClientProcedureDiscoverAllPrimaryServicesIndication_Error_gHciRemoteUserTerminatedConnection_c:
                    shell_write(gHciRemoteUserTerminatedConnection_c);
                    break;

                case GATTClientProcedureDiscoverAllPrimaryServicesIndication_Error_gHciRemoteDeviceTerminatedConnectionLowResources_c:
                    shell_write(gHciRemoteDeviceTerminatedConnectionLowResources_c);
                    break;

                case GATTClientProcedureDiscoverAllPrimaryServicesIndication_Error_gHciRemoteDeviceTerminatedConnectionPowerOff_c:
                    shell_write(gHciRemoteDeviceTerminatedConnectionPowerOff_c);
                    break;

                case GATTClientProcedureDiscoverAllPrimaryServicesIndication_Error_gHciConnectionTerminatedByLocalHost_c:
                    shell_write(gHciConnectionTerminatedByLocalHost_c);
                    break;

                case GATTClientProcedureDiscoverAllPrimaryServicesIndication_Error_gHciRepeatedAttempts_c:
                    shell_write(gHciRepeatedAttempts_c);
                    break;

                case GATTClientProcedureDiscoverAllPrimaryServicesIndication_Error_gHciPairingNotAllowed_c:
                    shell_write(gHciPairingNotAllowed_c);
                    break;

                case GATTClientProcedureDiscoverAllPrimaryServicesIndication_Error_gHciUnknownLpmPdu_c:
                    shell_write(gHciUnknownLpmPdu_c);
                    break;

                case GATTClientProcedureDiscoverAllPrimaryServicesIndication_Error_gHciUnsupportedRemoteFeature_c:
                    shell_write(gHciUnsupportedRemoteFeature_c);
                    break;

                case GATTClientProcedureDiscoverAllPrimaryServicesIndication_Error_gHciScoOffsetRejected_c:
                    shell_write(gHciScoOffsetRejected_c);
                    break;

                case GATTClientProcedureDiscoverAllPrimaryServicesIndication_Error_gHciScoIntervalRejected_c:
                    shell_write(gHciScoIntervalRejected_c);
                    break;

                case GATTClientProcedureDiscoverAllPrimaryServicesIndication_Error_gHciScoAirModeRejected_c:
                    shell_write(gHciScoAirModeRejected_c);
                    break;

                case GATTClientProcedureDiscoverAllPrimaryServicesIndication_Error_gHciInvalidLpmParameters_c:
                    shell_write(gHciInvalidLpmParameters_c);
                    break;

                case GATTClientProcedureDiscoverAllPrimaryServicesIndication_Error_gHciUnspecifiedError_c:
                    shell_write(gHciUnspecifiedc);
                    break;

                case GATTClientProcedureDiscoverAllPrimaryServicesIndication_Error_gHciUnsupportedLpmParameterValue_c:
                    shell_write(gHciUnsupportedLpmParameterValue_c);
                    break;

                case GATTClientProcedureDiscoverAllPrimaryServicesIndication_Error_gHciRoleChangeNotAllowed_c:
                    shell_write(gHciRoleChangeNotAllowed_c);
                    break;

                case GATTClientProcedureDiscoverAllPrimaryServicesIndication_Error_gHciLLResponseTimeout_c:
                    shell_write(gHciLLResponseTimeout_c);
                    break;

                case GATTClientProcedureDiscoverAllPrimaryServicesIndication_Error_gHciLmpErrorTransactionCollision_c:
                    shell_write(gHciLmpErrorTransactionCollision_c);
                    break;

                case GATTClientProcedureDiscoverAllPrimaryServicesIndication_Error_gHciLmpPduNotAllowed_c:
                    shell_write(gHciLmpPduNotAllowed_c);
                    break;

                case GATTClientProcedureDiscoverAllPrimaryServicesIndication_Error_gHciEncryptionModeNotAcceptable_c:
                    shell_write(gHciEncryptionModeNotAcceptable_c);
                    break;

                case GATTClientProcedureDiscoverAllPrimaryServicesIndication_Error_gHciLinkKeyCannotBeChanged_c:
                    shell_write(gHciLinkKeyCannotBeChanged_c);
                    break;

                case GATTClientProcedureDiscoverAllPrimaryServicesIndication_Error_gHciRequestedQosNotSupported_c:
                    shell_write(gHciRequestedQosNotSupported_c);
                    break;

                case GATTClientProcedureDiscoverAllPrimaryServicesIndication_Error_gHciInstantPassed_c:
                    shell_write(gHciInstantPassed_c);
                    break;

                case GATTClientProcedureDiscoverAllPrimaryServicesIndication_Error_gHciPairingWithUnitKeyNotSupported_c:
                    shell_write(gHciPairingWithUnitKeyNotSupported_c);
                    break;

                case GATTClientProcedureDiscoverAllPrimaryServicesIndication_Error_gHciDifferentTransactionCollision_c:
                    shell_write(gHciDifferentTransactionCollision_c);
                    break;

                case GATTClientProcedureDiscoverAllPrimaryServicesIndication_Error_gHciReserved_0x2B_c:
                    shell_write(gHciReserved_0x2B_c);
                    break;

                case GATTClientProcedureDiscoverAllPrimaryServicesIndication_Error_gHciQosNotAcceptableParameter_c:
                    shell_write(gHciQosNotAcceptableParameter_c);
                    break;

                case GATTClientProcedureDiscoverAllPrimaryServicesIndication_Error_gHciQosRejected_c:
                    shell_write(gHciQosRejected_c);
                    break;

                case GATTClientProcedureDiscoverAllPrimaryServicesIndication_Error_gHciChannelClassificationNotSupported_c:
                    shell_write(gHciChannelClassificationNotSupported_c);
                    break;

                case GATTClientProcedureDiscoverAllPrimaryServicesIndication_Error_gHciInsufficientSecurity_c:
                    shell_write(gHciInsufficientSecurity_c);
                    break;

                case GATTClientProcedureDiscoverAllPrimaryServicesIndication_Error_gHciParameterOutOfMandatoryRange_c:
                    shell_write(gHciParameterOutOfMandatoryRange_c);
                    break;

                case GATTClientProcedureDiscoverAllPrimaryServicesIndication_Error_gHciReserved_0x31_c:
                    shell_write(gHciReserved_0x31_c);
                    break;

                case GATTClientProcedureDiscoverAllPrimaryServicesIndication_Error_gHciRoleSwitchPending_c:
                    shell_write(gHciRoleSwitchPending_c);
                    break;

                case GATTClientProcedureDiscoverAllPrimaryServicesIndication_Error_gHciReserved_0x33_c:
                    shell_write(gHciReserved_0x33_c);
                    break;

                case GATTClientProcedureDiscoverAllPrimaryServicesIndication_Error_gHciReservedSlotViolation_c:
                    shell_write(gHciReservedSlotViolation_c);
                    break;

                case GATTClientProcedureDiscoverAllPrimaryServicesIndication_Error_gHciRoleSwitchFailed_c:
                    shell_write(gHciRoleSwitchFailed_c);
                    break;

                case GATTClientProcedureDiscoverAllPrimaryServicesIndication_Error_gHciExtendedInquiryResponseTooLarge_c:
                    shell_write(gHciExtendedInquiryResponseTooLarge_c);
                    break;

                case GATTClientProcedureDiscoverAllPrimaryServicesIndication_Error_gHciSecureSimplePairingNotSupportedByHost_c:
                    shell_write(gHciSecureSimplePairingNotSupportedByHost_c);
                    break;

                case GATTClientProcedureDiscoverAllPrimaryServicesIndication_Error_gHciHostBusyPairing_c:
                    shell_write(gHciHostBusyPairing_c);
                    break;

                case GATTClientProcedureDiscoverAllPrimaryServicesIndication_Error_gHciConnectionRejectedDueToNoSuitableChannelFound_c:
                    shell_write(gHciConnectionRejectedDueToNoSuitableChannelFound_c);
                    break;

                case GATTClientProcedureDiscoverAllPrimaryServicesIndication_Error_gHciControllerBusy_c:
                    shell_write(gHciControllerBusy_c);
                    break;

                case GATTClientProcedureDiscoverAllPrimaryServicesIndication_Error_gHciUnacceptableConnectionParameters_c:
                    shell_write(gHciUnacceptableConnectionParameters_c);
                    break;

                case GATTClientProcedureDiscoverAllPrimaryServicesIndication_Error_gHciDirectedAdvertisingTimeout_c:
                    shell_write(gHciDirectedAdvertisingTimeout_c);
                    break;

                case GATTClientProcedureDiscoverAllPrimaryServicesIndication_Error_gHciConnectionTerminatedDueToMicFailure_c:
                    shell_write(gHciConnectionTerminatedDueToMicFailure_c);
                    break;

                case GATTClientProcedureDiscoverAllPrimaryServicesIndication_Error_gHciConnectionFailedToBeEstablished_c:
                    shell_write(gHciConnectionFailedToBeEstablished_c);
                    break;

                case GATTClientProcedureDiscoverAllPrimaryServicesIndication_Error_gHciMacConnectionFailed_c:
                    shell_write(gHciMacConnectionFailed_c);
                    break;

                case GATTClientProcedureDiscoverAllPrimaryServicesIndication_Error_gHciCoarseClockAdjustmentRejected_c:
                    shell_write(gHciCoarseClockAdjustmentRejected_c);
                    break;

                case GATTClientProcedureDiscoverAllPrimaryServicesIndication_Error_gHciAlreadyInit_c:
                    shell_write(gHciAlreadyInit_c);
                    break;

                case GATTClientProcedureDiscoverAllPrimaryServicesIndication_Error_gHciInvalidParameter_c:
                    shell_write(gHciInvalidParameter_c);
                    break;

                case GATTClientProcedureDiscoverAllPrimaryServicesIndication_Error_gHciCallbackNotInstalled_c:
                    shell_write(gHciCallbackNotInstalled_c);
                    break;

                case GATTClientProcedureDiscoverAllPrimaryServicesIndication_Error_gHciCallbackAlreadyInstalled_c:
                    shell_write(gHciCallbackAlreadyInstalled_c);
                    break;

                case GATTClientProcedureDiscoverAllPrimaryServicesIndication_Error_gHciCommandNotSupported_c:
                    shell_write(gHciCommandNotSupported_c);
                    break;

                case GATTClientProcedureDiscoverAllPrimaryServicesIndication_Error_gHciEventNotSupported_c:
                    shell_write(gHciEventNotSupported_c);
                    break;

                case GATTClientProcedureDiscoverAllPrimaryServicesIndication_Error_gHciTransportError_c:
                    shell_write(gHciTransportc);
                    break;

                case GATTClientProcedureDiscoverAllPrimaryServicesIndication_Error_gL2caAlreadyInit_c:
                    shell_write(gL2caAlreadyInit_c);
                    break;

                case GATTClientProcedureDiscoverAllPrimaryServicesIndication_Error_gL2caInsufficientResources_c:
                    shell_write(gL2caInsufficientResources_c);
                    break;

                case GATTClientProcedureDiscoverAllPrimaryServicesIndication_Error_gL2caCallbackNotInstalled_c:
                    shell_write(gL2caCallbackNotInstalled_c);
                    break;

                case GATTClientProcedureDiscoverAllPrimaryServicesIndication_Error_gL2caCallbackAlreadyInstalled_c:
                    shell_write(gL2caCallbackAlreadyInstalled_c);
                    break;

                case GATTClientProcedureDiscoverAllPrimaryServicesIndication_Error_gL2caLePsmInvalid_c:
                    shell_write(gL2caLePsmInvalid_c);
                    break;

                case GATTClientProcedureDiscoverAllPrimaryServicesIndication_Error_gL2caLePsmAlreadyRegistered_c:
                    shell_write(gL2caLePsmAlreadyRegistered_c);
                    break;

                case GATTClientProcedureDiscoverAllPrimaryServicesIndication_Error_gL2caLePsmNotRegistered_c:
                    shell_write(gL2caLePsmNotRegistered_c);
                    break;

                case GATTClientProcedureDiscoverAllPrimaryServicesIndication_Error_gL2caLePsmInsufficientResources_c:
                    shell_write(gL2caLePsmInsufficientResources_c);
                    break;

                case GATTClientProcedureDiscoverAllPrimaryServicesIndication_Error_gL2caChannelInvalid_c:
                    shell_write(gL2caChannelInvalid_c);
                    break;

                case GATTClientProcedureDiscoverAllPrimaryServicesIndication_Error_gL2caChannelClosed_c:
                    shell_write(gL2caChannelClosed_c);
                    break;

                case GATTClientProcedureDiscoverAllPrimaryServicesIndication_Error_gL2caChannelAlreadyConnected_c:
                    shell_write(gL2caChannelAlreadyConnected_c);
                    break;

                case GATTClientProcedureDiscoverAllPrimaryServicesIndication_Error_gL2caConnectionParametersRejected_c:
                    shell_write(gL2caConnectionParametersRejected_c);
                    break;

                case GATTClientProcedureDiscoverAllPrimaryServicesIndication_Error_gL2caChannelBusy_c:
                    shell_write(gL2caChannelBusy_c);
                    break;

                case GATTClientProcedureDiscoverAllPrimaryServicesIndication_Error_gL2caInvalidParameter_c:
                    shell_write(gL2caInvalidParameter_c);
                    break;

                case GATTClientProcedureDiscoverAllPrimaryServicesIndication_Error_gL2caError_c:
                    shell_write(gL2cac);
                    break;

                case GATTClientProcedureDiscoverAllPrimaryServicesIndication_Error_gSmNullCBFunction_c:
                    shell_write(gSmNullCBFunction_c);
                    break;

                case GATTClientProcedureDiscoverAllPrimaryServicesIndication_Error_gSmCommandNotSupported_c:
                    shell_write(gSmCommandNotSupported_c);
                    break;

                case GATTClientProcedureDiscoverAllPrimaryServicesIndication_Error_gSmUnexpectedCommand_c:
                    shell_write(gSmUnexpectedCommand_c);
                    break;

                case GATTClientProcedureDiscoverAllPrimaryServicesIndication_Error_gSmInvalidCommandCode_c:
                    shell_write(gSmInvalidCommandCode_c);
                    break;

                case GATTClientProcedureDiscoverAllPrimaryServicesIndication_Error_gSmInvalidCommandLength_c:
                    shell_write(gSmInvalidCommandLength_c);
                    break;

                case GATTClientProcedureDiscoverAllPrimaryServicesIndication_Error_gSmInvalidCommandParameter_c:
                    shell_write(gSmInvalidCommandParameter_c);
                    break;

                case GATTClientProcedureDiscoverAllPrimaryServicesIndication_Error_gSmInvalidDeviceId_c:
                    shell_write(gSmInvalidDeviceId_c);
                    break;

                case GATTClientProcedureDiscoverAllPrimaryServicesIndication_Error_gSmInvalidInternalOperation_c:
                    shell_write(gSmInvalidInternalOperation_c);
                    break;

                case GATTClientProcedureDiscoverAllPrimaryServicesIndication_Error_gSmInvalidConnectionHandle_c:
                    shell_write(gSmInvalidConnectionHandle_c);
                    break;

                case GATTClientProcedureDiscoverAllPrimaryServicesIndication_Error_gSmInproperKeyDistributionField_c:
                    shell_write(gSmInproperKeyDistributionField_c);
                    break;

                case GATTClientProcedureDiscoverAllPrimaryServicesIndication_Error_gSmUnexpectedKeyType_c:
                    shell_write(gSmUnexpectedKeyType_c);
                    break;

                case GATTClientProcedureDiscoverAllPrimaryServicesIndication_Error_gSmUnexpectedPairingTerminationReason_c:
                    shell_write(gSmUnexpectedPairingTerminationReason_c);
                    break;

                case GATTClientProcedureDiscoverAllPrimaryServicesIndication_Error_gSmUnexpectedKeyset_c:
                    shell_write(gSmUnexpectedKeyset_c);
                    break;

                case GATTClientProcedureDiscoverAllPrimaryServicesIndication_Error_gSmSmpTimeoutOccurred_c:
                    shell_write(gSmSmpTimeoutOccurred_c);
                    break;

                case GATTClientProcedureDiscoverAllPrimaryServicesIndication_Error_gSmUnknownSmpPacketType_c:
                    shell_write(gSmUnknownSmpPacketType_c);
                    break;

                case GATTClientProcedureDiscoverAllPrimaryServicesIndication_Error_gSmInvalidSmpPacketLength_c:
                    shell_write(gSmInvalidSmpPacketLength_c);
                    break;

                case GATTClientProcedureDiscoverAllPrimaryServicesIndication_Error_gSmInvalidSmpPacketParameter_c:
                    shell_write(gSmInvalidSmpPacketParameter_c);
                    break;

                case GATTClientProcedureDiscoverAllPrimaryServicesIndication_Error_gSmReceivedUnexpectedSmpPacket_c:
                    shell_write(gSmReceivedUnexpectedSmpPacket_c);
                    break;

                case GATTClientProcedureDiscoverAllPrimaryServicesIndication_Error_gSmReceivedSmpPacketFromUnknownDevice_c:
                    shell_write(gSmReceivedSmpPacketFromUnknownDevice_c);
                    break;

                case GATTClientProcedureDiscoverAllPrimaryServicesIndication_Error_gSmReceivedUnexpectedHciEvent_c:
                    shell_write(gSmReceivedUnexpectedHciEvent_c);
                    break;

                case GATTClientProcedureDiscoverAllPrimaryServicesIndication_Error_gSmReceivedHciEventFromUnknownDevice_c:
                    shell_write(gSmReceivedHciEventFromUnknownDevice_c);
                    break;

                case GATTClientProcedureDiscoverAllPrimaryServicesIndication_Error_gSmInvalidHciEventParameter_c:
                    shell_write(gSmInvalidHciEventParameter_c);
                    break;

                case GATTClientProcedureDiscoverAllPrimaryServicesIndication_Error_gSmLlConnectionEncryptionInProgress_c:
                    shell_write(gSmLlConnectionEncryptionInProgress_c);
                    break;

                case GATTClientProcedureDiscoverAllPrimaryServicesIndication_Error_gSmLlConnectionEncryptionFailure_c:
                    shell_write(gSmLlConnectionEncryptionFailure_c);
                    break;

                case GATTClientProcedureDiscoverAllPrimaryServicesIndication_Error_gSmInsufficientResources_c:
                    shell_write(gSmInsufficientResources_c);
                    break;

                case GATTClientProcedureDiscoverAllPrimaryServicesIndication_Error_gSmOobDataAddressMismatch_c:
                    shell_write(gSmOobDataAddressMismatch_c);
                    break;

//                case GATTClientProcedureDiscoverAllPrimaryServicesIndication_Error_gSmSmpPacketReceivedAfterTimeoutOccurred_c:
//                    shell_write(gSmSmpPacketReceivedAfterTimeoutOccurred_c);
//                    break;

                case GATTClientProcedureDiscoverAllPrimaryServicesIndication_Error_gSmPairingErrorPasskeyEntryFailed_c:
                    shell_write(gSmPairingErrorPasskeyEntryFailed_c);
                    break;

                case GATTClientProcedureDiscoverAllPrimaryServicesIndication_Error_gSmPairingErrorConfirmValueFailed_c:
                    shell_write(gSmPairingErrorConfirmValueFailed_c);
                    break;

                case GATTClientProcedureDiscoverAllPrimaryServicesIndication_Error_gSmPairingErrorCommandNotSupported_c:
                    shell_write(gSmPairingErrorCommandNotSupported_c);
                    break;

                case GATTClientProcedureDiscoverAllPrimaryServicesIndication_Error_gSmPairingErrorInvalidParameters_c:
                    shell_write(gSmPairingErrorInvalidParameters_c);
                    break;

                case GATTClientProcedureDiscoverAllPrimaryServicesIndication_Error_gSmPairingErrorUnknownReason_c:
                    shell_write(gSmPairingErrorUnknownReason_c);
                    break;

                case GATTClientProcedureDiscoverAllPrimaryServicesIndication_Error_gSmTbResolvableAddressDoesNotMatchIrk_c:
                    shell_write(gSmTbResolvableAddressDoesNotMatchIrk_c);
                    break;

                case GATTClientProcedureDiscoverAllPrimaryServicesIndication_Error_gSmTbInvalidDataSignature_c:
                    shell_write(gSmTbInvalidDataSignature_c);
                    break;

                case GATTClientProcedureDiscoverAllPrimaryServicesIndication_Error_gAttInvalidHandle_c:
                    shell_write(gAttInvalidHandle_c);
                    break;

                case GATTClientProcedureDiscoverAllPrimaryServicesIndication_Error_gAttReadNotPermitted_c:
                    shell_write(gAttReadNotPermitted_c);
                    break;

                case GATTClientProcedureDiscoverAllPrimaryServicesIndication_Error_gAttWriteNotPermitted_c:
                    shell_write(gAttWriteNotPermitted_c);
                    break;

                case GATTClientProcedureDiscoverAllPrimaryServicesIndication_Error_gAttInvalidPdu_c:
                    shell_write(gAttInvalidPdu_c);
                    break;

                case GATTClientProcedureDiscoverAllPrimaryServicesIndication_Error_gAttInsufficientAuthentication_c:
                    shell_write(gAttInsufficientAuthentication_c);
                    break;

                case GATTClientProcedureDiscoverAllPrimaryServicesIndication_Error_gAttRequestNotSupported_c:
                    shell_write(gAttRequestNotSupported_c);
                    break;

                case GATTClientProcedureDiscoverAllPrimaryServicesIndication_Error_gAttInvalidOffset_c:
                    shell_write(gAttInvalidOffset_c);
                    break;

                case GATTClientProcedureDiscoverAllPrimaryServicesIndication_Error_gAttInsufficientAuthorization_c:
                    shell_write(gAttInsufficientAuthorization_c);
                    break;

                case GATTClientProcedureDiscoverAllPrimaryServicesIndication_Error_gAttPrepareQueueFull_c:
                    shell_write(gAttPrepareQueueFull_c);
                    break;

                case GATTClientProcedureDiscoverAllPrimaryServicesIndication_Error_gAttAttributeNotFound_c:
                    shell_write(gAttAttributeNotFound_c);
                    break;

                case GATTClientProcedureDiscoverAllPrimaryServicesIndication_Error_gAttAttributeNotLong_c:
                    shell_write(gAttAttributeNotLong_c);
                    break;

                case GATTClientProcedureDiscoverAllPrimaryServicesIndication_Error_gAttInsufficientEncryptionKeySize_c:
                    shell_write(gAttInsufficientEncryptionKeySize_c);
                    break;

                case GATTClientProcedureDiscoverAllPrimaryServicesIndication_Error_gAttInvalidAttributeValueLength_c:
                    shell_write(gAttInvalidAttributeValueLength_c);
                    break;

                case GATTClientProcedureDiscoverAllPrimaryServicesIndication_Error_gAttUnlikelyor_c:
                    shell_write(gAttUnlikelyor_c);
                    break;

                case GATTClientProcedureDiscoverAllPrimaryServicesIndication_Error_gAttInsufficientEncryption_c:
                    shell_write(gAttInsufficientEncryption_c);
                    break;

                case GATTClientProcedureDiscoverAllPrimaryServicesIndication_Error_gAttUnsupportedGroupType_c:
                    shell_write(gAttUnsupportedGroupType_c);
                    break;

                case GATTClientProcedureDiscoverAllPrimaryServicesIndication_Error_gAttInsufficientResources_c:
                    shell_write(gAttInsufficientResources_c);
                    break;

                case GATTClientProcedureDiscoverAllPrimaryServicesIndication_Error_gGattAnotherProcedureInProgress_c:
                    shell_write(gGattAnotherProcedureInProgress_c);
                    break;

                case GATTClientProcedureDiscoverAllPrimaryServicesIndication_Error_gGattLongAttributePacketsCorrupted_c:
                    shell_write(gGattLongAttributePacketsCorrupted_c);
                    break;

                case GATTClientProcedureDiscoverAllPrimaryServicesIndication_Error_gGattMultipleAttributesOverflow_c:
                    shell_write(gGattMultipleAttributesOverflow_c);
                    break;

                case GATTClientProcedureDiscoverAllPrimaryServicesIndication_Error_gGattUnexpectedReadMultipleResponseLength_c:
                    shell_write(gGattUnexpectedReadMultipleResponseLength_c);
                    break;

                case GATTClientProcedureDiscoverAllPrimaryServicesIndication_Error_gGattInvalidValueLength_c:
                    shell_write(gGattInvalidValueLength_c);
                    break;

                case GATTClientProcedureDiscoverAllPrimaryServicesIndication_Error_gGattServerTimeout_c:
                    shell_write(gGattServerTimeout_c);
                    break;

                case GATTClientProcedureDiscoverAllPrimaryServicesIndication_Error_gGattIndicationAlreadyInProgress_c:
                    shell_write(gGattIndicationAlreadyInProgress_c);
                    break;

                case GATTClientProcedureDiscoverAllPrimaryServicesIndication_Error_gGattClientConfirmationTimeout_c:
                    shell_write(gGattClientConfirmationTimeout_c);
                    break;

                case GATTClientProcedureDiscoverAllPrimaryServicesIndication_Error_gGapAdvDataTooLong_c:
                    shell_write(gGapAdvDataTooLong_c);
                    break;

                case GATTClientProcedureDiscoverAllPrimaryServicesIndication_Error_gGapScanRspDataTooLong_c:
                    shell_write(gGapScanRspDataTooLong_c);
                    break;

                case GATTClientProcedureDiscoverAllPrimaryServicesIndication_Error_gGapDeviceNotBonded_c:
                    shell_write(gGapDeviceNotBonded_c);
                    break;

                case GATTClientProcedureDiscoverAllPrimaryServicesIndication_Error_gDevDbCccdLimitReached_c:
                    shell_write(gDevDbCccdLimitReached_c);
                    break;

                case GATTClientProcedureDiscoverAllPrimaryServicesIndication_Error_gDevDbCccdNotFound_c:
                    shell_write(gDevDbCccdNotFound_c);
                    break;

                case GATTClientProcedureDiscoverAllPrimaryServicesIndication_Error_gGattDbInvalidHandle_c:
                    shell_write(gGattDbInvalidHandle_c);
                    break;

                case GATTClientProcedureDiscoverAllPrimaryServicesIndication_Error_gGattDbCharacteristicNotFound_c:
                    shell_write(gGattDbCharacteristicNotFound_c);
                    break;

                case GATTClientProcedureDiscoverAllPrimaryServicesIndication_Error_gGattDbCccdNotFound_c:
                    shell_write(gGattDbCccdNotFound_c);
                    break;

                case GATTClientProcedureDiscoverAllPrimaryServicesIndication_Error_gGattDbServiceNotFound_c:
                    shell_write(gGattDbServiceNotFound_c);
                    break;

                case GATTClientProcedureDiscoverAllPrimaryServicesIndication_Error_gGattDbDescriptorNotFound_c:
                    shell_write(gGattDbDescriptorNotFound_c);
                    break;

                default:
                    shell_printf("Unrecognized status 0x%04X", container->Data.GATTClientProcedureDiscoverAllPrimaryServicesIndication.Error);
                    break;
            }

            break;

        case 0x4484:
            shell_write("\r\nGATTClientProcedureDiscoverPrimaryServicesByUuidIndication");
            shell_write(" -> ");

            switch (container->Data.GATTClientProcedureDiscoverPrimaryServicesByUuidIndication.ProcedureResult)
            {
                case GATTClientProcedureDiscoverPrimaryServicesByUuidIndication_ProcedureResult_gGattProcSuccess_c:
                    shell_write(gGattProcSuccess_c);
                    break;

                case GATTClientProcedureDiscoverPrimaryServicesByUuidIndication_ProcedureResult_gProcedureError_c:
                    shell_write(gProcedureError_c);
                    break;

                default:
                    shell_printf("Unrecognized status 0x%02X", container->Data.GATTClientProcedureDiscoverPrimaryServicesByUuidIndication.ProcedureResult);
                    break;
            }

            shell_write(" -> ");

            switch (container->Data.GATTClientProcedureDiscoverPrimaryServicesByUuidIndication.Error)
            {
                case GATTClientProcedureDiscoverPrimaryServicesByUuidIndication_Error_gBleSuccess_c:
                    shell_write(gBleSuccess_c);
                    SHELL_PrintDiscoveredPrimaryServices((GATTClientProcedureDiscoverAllPrimaryServicesIndication_t *) & (container->Data.GATTClientProcedureDiscoverPrimaryServicesByUuidIndication));
                    break;

                case GATTClientProcedureDiscoverPrimaryServicesByUuidIndication_Error_gBleInvalidParameter_c:
                    shell_write(gBleInvalidParameter_c);
                    break;

                case GATTClientProcedureDiscoverPrimaryServicesByUuidIndication_Error_gBleOverflow_c:
                    shell_write(gBleOverflow_c);
                    break;

                case GATTClientProcedureDiscoverPrimaryServicesByUuidIndication_Error_gBleUnavailable_c:
                    shell_write(gBleUnavailable_c);
                    break;

                case GATTClientProcedureDiscoverPrimaryServicesByUuidIndication_Error_gBleFeatureNotSupported_c:
                    shell_write(gBleFeatureNotSupported_c);
                    break;

                case GATTClientProcedureDiscoverPrimaryServicesByUuidIndication_Error_gBleOutOfMemory_c:
                    shell_write(gBleOutOfMemory_c);
                    break;

                case GATTClientProcedureDiscoverPrimaryServicesByUuidIndication_Error_gBleAlreadyInitialized_c:
                    shell_write(gBleAlreadyInitialized_c);
                    break;

                case GATTClientProcedureDiscoverPrimaryServicesByUuidIndication_Error_gBleOsError_c:
                    shell_write(gBleOsc);
                    break;

                case GATTClientProcedureDiscoverPrimaryServicesByUuidIndication_Error_gBleUnexpectedError_c:
                    shell_write(gBleUnexpectedc);
                    break;

                case GATTClientProcedureDiscoverPrimaryServicesByUuidIndication_Error_gBleInvalidState_c:
                    shell_write(gBleInvalidState_c);
                    break;

                case GATTClientProcedureDiscoverPrimaryServicesByUuidIndication_Error_gHciUnknownHciCommand_c:
                    shell_write(gHciUnknownHciCommand_c);
                    break;

                case GATTClientProcedureDiscoverPrimaryServicesByUuidIndication_Error_gHciUnknownConnectionIdentifier_c:
                    shell_write(gHciUnknownConnectionIdentifier_c);
                    break;

                case GATTClientProcedureDiscoverPrimaryServicesByUuidIndication_Error_gHciHardwareFailure_c:
                    shell_write(gHciHardwareFailure_c);
                    break;

                case GATTClientProcedureDiscoverPrimaryServicesByUuidIndication_Error_gHciPageTimeout_c:
                    shell_write(gHciPageTimeout_c);
                    break;

                case GATTClientProcedureDiscoverPrimaryServicesByUuidIndication_Error_gHciAuthenticationFailure_c:
                    shell_write(gHciAuthenticationFailure_c);
                    break;

                case GATTClientProcedureDiscoverPrimaryServicesByUuidIndication_Error_gHciPinOrKeyMissing_c:
                    shell_write(gHciPinOrKeyMissing_c);
                    break;

                case GATTClientProcedureDiscoverPrimaryServicesByUuidIndication_Error_gHciMemoryCapacityExceeded_c:
                    shell_write(gHciMemoryCapacityExceeded_c);
                    break;

                case GATTClientProcedureDiscoverPrimaryServicesByUuidIndication_Error_gHciConnectionTimeout_c:
                    shell_write(gHciConnectionTimeout_c);
                    break;

                case GATTClientProcedureDiscoverPrimaryServicesByUuidIndication_Error_gHciConnectionLimitExceeded_c:
                    shell_write(gHciConnectionLimitExceeded_c);
                    break;

                case GATTClientProcedureDiscoverPrimaryServicesByUuidIndication_Error_gHciSynchronousConnectionLimitToADeviceExceeded_c:
                    shell_write(gHciSynchronousConnectionLimitToADeviceExceeded_c);
                    break;

                case GATTClientProcedureDiscoverPrimaryServicesByUuidIndication_Error_gHciAclConnectionAlreadyExists_c:
                    shell_write(gHciAclConnectionAlreadyExists_c);
                    break;

                case GATTClientProcedureDiscoverPrimaryServicesByUuidIndication_Error_gHciCommandDisallowed_c:
                    shell_write(gHciCommandDisallowed_c);
                    break;

                case GATTClientProcedureDiscoverPrimaryServicesByUuidIndication_Error_gHciConnectionRejectedDueToLimitedResources_c:
                    shell_write(gHciConnectionRejectedDueToLimitedResources_c);
                    break;

                case GATTClientProcedureDiscoverPrimaryServicesByUuidIndication_Error_gHciConnectionRejectedDueToSecurityReasons_c:
                    shell_write(gHciConnectionRejectedDueToSecurityReasons_c);
                    break;

                case GATTClientProcedureDiscoverPrimaryServicesByUuidIndication_Error_gHciConnectionRejectedDueToUnacceptableBdAddr_c:
                    shell_write(gHciConnectionRejectedDueToUnacceptableBdAddr_c);
                    break;

                case GATTClientProcedureDiscoverPrimaryServicesByUuidIndication_Error_gHciConnectionAcceptTimeoutExceeded_c:
                    shell_write(gHciConnectionAcceptTimeoutExceeded_c);
                    break;

                case GATTClientProcedureDiscoverPrimaryServicesByUuidIndication_Error_gHciUnsupportedFeatureOrParameterValue_c:
                    shell_write(gHciUnsupportedFeatureOrParameterValue_c);
                    break;

                case GATTClientProcedureDiscoverPrimaryServicesByUuidIndication_Error_gHciInvalidHciCommandParameters_c:
                    shell_write(gHciInvalidHciCommandParameters_c);
                    break;

                case GATTClientProcedureDiscoverPrimaryServicesByUuidIndication_Error_gHciRemoteUserTerminatedConnection_c:
                    shell_write(gHciRemoteUserTerminatedConnection_c);
                    break;

                case GATTClientProcedureDiscoverPrimaryServicesByUuidIndication_Error_gHciRemoteDeviceTerminatedConnectionLowResources_c:
                    shell_write(gHciRemoteDeviceTerminatedConnectionLowResources_c);
                    break;

                case GATTClientProcedureDiscoverPrimaryServicesByUuidIndication_Error_gHciRemoteDeviceTerminatedConnectionPowerOff_c:
                    shell_write(gHciRemoteDeviceTerminatedConnectionPowerOff_c);
                    break;

                case GATTClientProcedureDiscoverPrimaryServicesByUuidIndication_Error_gHciConnectionTerminatedByLocalHost_c:
                    shell_write(gHciConnectionTerminatedByLocalHost_c);
                    break;

                case GATTClientProcedureDiscoverPrimaryServicesByUuidIndication_Error_gHciRepeatedAttempts_c:
                    shell_write(gHciRepeatedAttempts_c);
                    break;

                case GATTClientProcedureDiscoverPrimaryServicesByUuidIndication_Error_gHciPairingNotAllowed_c:
                    shell_write(gHciPairingNotAllowed_c);
                    break;

                case GATTClientProcedureDiscoverPrimaryServicesByUuidIndication_Error_gHciUnknownLpmPdu_c:
                    shell_write(gHciUnknownLpmPdu_c);
                    break;

                case GATTClientProcedureDiscoverPrimaryServicesByUuidIndication_Error_gHciUnsupportedRemoteFeature_c:
                    shell_write(gHciUnsupportedRemoteFeature_c);
                    break;

                case GATTClientProcedureDiscoverPrimaryServicesByUuidIndication_Error_gHciScoOffsetRejected_c:
                    shell_write(gHciScoOffsetRejected_c);
                    break;

                case GATTClientProcedureDiscoverPrimaryServicesByUuidIndication_Error_gHciScoIntervalRejected_c:
                    shell_write(gHciScoIntervalRejected_c);
                    break;

                case GATTClientProcedureDiscoverPrimaryServicesByUuidIndication_Error_gHciScoAirModeRejected_c:
                    shell_write(gHciScoAirModeRejected_c);
                    break;

                case GATTClientProcedureDiscoverPrimaryServicesByUuidIndication_Error_gHciInvalidLpmParameters_c:
                    shell_write(gHciInvalidLpmParameters_c);
                    break;

                case GATTClientProcedureDiscoverPrimaryServicesByUuidIndication_Error_gHciUnspecifiedError_c:
                    shell_write(gHciUnspecifiedc);
                    break;

                case GATTClientProcedureDiscoverPrimaryServicesByUuidIndication_Error_gHciUnsupportedLpmParameterValue_c:
                    shell_write(gHciUnsupportedLpmParameterValue_c);
                    break;

                case GATTClientProcedureDiscoverPrimaryServicesByUuidIndication_Error_gHciRoleChangeNotAllowed_c:
                    shell_write(gHciRoleChangeNotAllowed_c);
                    break;

                case GATTClientProcedureDiscoverPrimaryServicesByUuidIndication_Error_gHciLLResponseTimeout_c:
                    shell_write(gHciLLResponseTimeout_c);
                    break;

                case GATTClientProcedureDiscoverPrimaryServicesByUuidIndication_Error_gHciLmpErrorTransactionCollision_c:
                    shell_write(gHciLmpErrorTransactionCollision_c);
                    break;

                case GATTClientProcedureDiscoverPrimaryServicesByUuidIndication_Error_gHciLmpPduNotAllowed_c:
                    shell_write(gHciLmpPduNotAllowed_c);
                    break;

                case GATTClientProcedureDiscoverPrimaryServicesByUuidIndication_Error_gHciEncryptionModeNotAcceptable_c:
                    shell_write(gHciEncryptionModeNotAcceptable_c);
                    break;

                case GATTClientProcedureDiscoverPrimaryServicesByUuidIndication_Error_gHciLinkKeyCannotBeChanged_c:
                    shell_write(gHciLinkKeyCannotBeChanged_c);
                    break;

                case GATTClientProcedureDiscoverPrimaryServicesByUuidIndication_Error_gHciRequestedQosNotSupported_c:
                    shell_write(gHciRequestedQosNotSupported_c);
                    break;

                case GATTClientProcedureDiscoverPrimaryServicesByUuidIndication_Error_gHciInstantPassed_c:
                    shell_write(gHciInstantPassed_c);
                    break;

                case GATTClientProcedureDiscoverPrimaryServicesByUuidIndication_Error_gHciPairingWithUnitKeyNotSupported_c:
                    shell_write(gHciPairingWithUnitKeyNotSupported_c);
                    break;

                case GATTClientProcedureDiscoverPrimaryServicesByUuidIndication_Error_gHciDifferentTransactionCollision_c:
                    shell_write(gHciDifferentTransactionCollision_c);
                    break;

                case GATTClientProcedureDiscoverPrimaryServicesByUuidIndication_Error_gHciReserved_0x2B_c:
                    shell_write(gHciReserved_0x2B_c);
                    break;

                case GATTClientProcedureDiscoverPrimaryServicesByUuidIndication_Error_gHciQosNotAcceptableParameter_c:
                    shell_write(gHciQosNotAcceptableParameter_c);
                    break;

                case GATTClientProcedureDiscoverPrimaryServicesByUuidIndication_Error_gHciQosRejected_c:
                    shell_write(gHciQosRejected_c);
                    break;

                case GATTClientProcedureDiscoverPrimaryServicesByUuidIndication_Error_gHciChannelClassificationNotSupported_c:
                    shell_write(gHciChannelClassificationNotSupported_c);
                    break;

                case GATTClientProcedureDiscoverPrimaryServicesByUuidIndication_Error_gHciInsufficientSecurity_c:
                    shell_write(gHciInsufficientSecurity_c);
                    break;

                case GATTClientProcedureDiscoverPrimaryServicesByUuidIndication_Error_gHciParameterOutOfMandatoryRange_c:
                    shell_write(gHciParameterOutOfMandatoryRange_c);
                    break;

                case GATTClientProcedureDiscoverPrimaryServicesByUuidIndication_Error_gHciReserved_0x31_c:
                    shell_write(gHciReserved_0x31_c);
                    break;

                case GATTClientProcedureDiscoverPrimaryServicesByUuidIndication_Error_gHciRoleSwitchPending_c:
                    shell_write(gHciRoleSwitchPending_c);
                    break;

                case GATTClientProcedureDiscoverPrimaryServicesByUuidIndication_Error_gHciReserved_0x33_c:
                    shell_write(gHciReserved_0x33_c);
                    break;

                case GATTClientProcedureDiscoverPrimaryServicesByUuidIndication_Error_gHciReservedSlotViolation_c:
                    shell_write(gHciReservedSlotViolation_c);
                    break;

                case GATTClientProcedureDiscoverPrimaryServicesByUuidIndication_Error_gHciRoleSwitchFailed_c:
                    shell_write(gHciRoleSwitchFailed_c);
                    break;

                case GATTClientProcedureDiscoverPrimaryServicesByUuidIndication_Error_gHciExtendedInquiryResponseTooLarge_c:
                    shell_write(gHciExtendedInquiryResponseTooLarge_c);
                    break;

                case GATTClientProcedureDiscoverPrimaryServicesByUuidIndication_Error_gHciSecureSimplePairingNotSupportedByHost_c:
                    shell_write(gHciSecureSimplePairingNotSupportedByHost_c);
                    break;

                case GATTClientProcedureDiscoverPrimaryServicesByUuidIndication_Error_gHciHostBusyPairing_c:
                    shell_write(gHciHostBusyPairing_c);
                    break;

                case GATTClientProcedureDiscoverPrimaryServicesByUuidIndication_Error_gHciConnectionRejectedDueToNoSuitableChannelFound_c:
                    shell_write(gHciConnectionRejectedDueToNoSuitableChannelFound_c);
                    break;

                case GATTClientProcedureDiscoverPrimaryServicesByUuidIndication_Error_gHciControllerBusy_c:
                    shell_write(gHciControllerBusy_c);
                    break;

                case GATTClientProcedureDiscoverPrimaryServicesByUuidIndication_Error_gHciUnacceptableConnectionParameters_c:
                    shell_write(gHciUnacceptableConnectionParameters_c);
                    break;

                case GATTClientProcedureDiscoverPrimaryServicesByUuidIndication_Error_gHciDirectedAdvertisingTimeout_c:
                    shell_write(gHciDirectedAdvertisingTimeout_c);
                    break;

                case GATTClientProcedureDiscoverPrimaryServicesByUuidIndication_Error_gHciConnectionTerminatedDueToMicFailure_c:
                    shell_write(gHciConnectionTerminatedDueToMicFailure_c);
                    break;

                case GATTClientProcedureDiscoverPrimaryServicesByUuidIndication_Error_gHciConnectionFailedToBeEstablished_c:
                    shell_write(gHciConnectionFailedToBeEstablished_c);
                    break;

                case GATTClientProcedureDiscoverPrimaryServicesByUuidIndication_Error_gHciMacConnectionFailed_c:
                    shell_write(gHciMacConnectionFailed_c);
                    break;

                case GATTClientProcedureDiscoverPrimaryServicesByUuidIndication_Error_gHciCoarseClockAdjustmentRejected_c:
                    shell_write(gHciCoarseClockAdjustmentRejected_c);
                    break;

                case GATTClientProcedureDiscoverPrimaryServicesByUuidIndication_Error_gHciAlreadyInit_c:
                    shell_write(gHciAlreadyInit_c);
                    break;

                case GATTClientProcedureDiscoverPrimaryServicesByUuidIndication_Error_gHciInvalidParameter_c:
                    shell_write(gHciInvalidParameter_c);
                    break;

                case GATTClientProcedureDiscoverPrimaryServicesByUuidIndication_Error_gHciCallbackNotInstalled_c:
                    shell_write(gHciCallbackNotInstalled_c);
                    break;

                case GATTClientProcedureDiscoverPrimaryServicesByUuidIndication_Error_gHciCallbackAlreadyInstalled_c:
                    shell_write(gHciCallbackAlreadyInstalled_c);
                    break;

                case GATTClientProcedureDiscoverPrimaryServicesByUuidIndication_Error_gHciCommandNotSupported_c:
                    shell_write(gHciCommandNotSupported_c);
                    break;

                case GATTClientProcedureDiscoverPrimaryServicesByUuidIndication_Error_gHciEventNotSupported_c:
                    shell_write(gHciEventNotSupported_c);
                    break;

                case GATTClientProcedureDiscoverPrimaryServicesByUuidIndication_Error_gHciTransportError_c:
                    shell_write(gHciTransportc);
                    break;

                case GATTClientProcedureDiscoverPrimaryServicesByUuidIndication_Error_gL2caAlreadyInit_c:
                    shell_write(gL2caAlreadyInit_c);
                    break;

                case GATTClientProcedureDiscoverPrimaryServicesByUuidIndication_Error_gL2caInsufficientResources_c:
                    shell_write(gL2caInsufficientResources_c);
                    break;

                case GATTClientProcedureDiscoverPrimaryServicesByUuidIndication_Error_gL2caCallbackNotInstalled_c:
                    shell_write(gL2caCallbackNotInstalled_c);
                    break;

                case GATTClientProcedureDiscoverPrimaryServicesByUuidIndication_Error_gL2caCallbackAlreadyInstalled_c:
                    shell_write(gL2caCallbackAlreadyInstalled_c);
                    break;

                case GATTClientProcedureDiscoverPrimaryServicesByUuidIndication_Error_gL2caLePsmInvalid_c:
                    shell_write(gL2caLePsmInvalid_c);
                    break;

                case GATTClientProcedureDiscoverPrimaryServicesByUuidIndication_Error_gL2caLePsmAlreadyRegistered_c:
                    shell_write(gL2caLePsmAlreadyRegistered_c);
                    break;

                case GATTClientProcedureDiscoverPrimaryServicesByUuidIndication_Error_gL2caLePsmNotRegistered_c:
                    shell_write(gL2caLePsmNotRegistered_c);
                    break;

                case GATTClientProcedureDiscoverPrimaryServicesByUuidIndication_Error_gL2caLePsmInsufficientResources_c:
                    shell_write(gL2caLePsmInsufficientResources_c);
                    break;

                case GATTClientProcedureDiscoverPrimaryServicesByUuidIndication_Error_gL2caChannelInvalid_c:
                    shell_write(gL2caChannelInvalid_c);
                    break;

                case GATTClientProcedureDiscoverPrimaryServicesByUuidIndication_Error_gL2caChannelClosed_c:
                    shell_write(gL2caChannelClosed_c);
                    break;

                case GATTClientProcedureDiscoverPrimaryServicesByUuidIndication_Error_gL2caChannelAlreadyConnected_c:
                    shell_write(gL2caChannelAlreadyConnected_c);
                    break;

                case GATTClientProcedureDiscoverPrimaryServicesByUuidIndication_Error_gL2caConnectionParametersRejected_c:
                    shell_write(gL2caConnectionParametersRejected_c);
                    break;

                case GATTClientProcedureDiscoverPrimaryServicesByUuidIndication_Error_gL2caChannelBusy_c:
                    shell_write(gL2caChannelBusy_c);
                    break;

                case GATTClientProcedureDiscoverPrimaryServicesByUuidIndication_Error_gL2caInvalidParameter_c:
                    shell_write(gL2caInvalidParameter_c);
                    break;

                case GATTClientProcedureDiscoverPrimaryServicesByUuidIndication_Error_gL2caError_c:
                    shell_write(gL2cac);
                    break;

                case GATTClientProcedureDiscoverPrimaryServicesByUuidIndication_Error_gSmNullCBFunction_c:
                    shell_write(gSmNullCBFunction_c);
                    break;

                case GATTClientProcedureDiscoverPrimaryServicesByUuidIndication_Error_gSmCommandNotSupported_c:
                    shell_write(gSmCommandNotSupported_c);
                    break;

                case GATTClientProcedureDiscoverPrimaryServicesByUuidIndication_Error_gSmUnexpectedCommand_c:
                    shell_write(gSmUnexpectedCommand_c);
                    break;

                case GATTClientProcedureDiscoverPrimaryServicesByUuidIndication_Error_gSmInvalidCommandCode_c:
                    shell_write(gSmInvalidCommandCode_c);
                    break;

                case GATTClientProcedureDiscoverPrimaryServicesByUuidIndication_Error_gSmInvalidCommandLength_c:
                    shell_write(gSmInvalidCommandLength_c);
                    break;

                case GATTClientProcedureDiscoverPrimaryServicesByUuidIndication_Error_gSmInvalidCommandParameter_c:
                    shell_write(gSmInvalidCommandParameter_c);
                    break;

                case GATTClientProcedureDiscoverPrimaryServicesByUuidIndication_Error_gSmInvalidDeviceId_c:
                    shell_write(gSmInvalidDeviceId_c);
                    break;

                case GATTClientProcedureDiscoverPrimaryServicesByUuidIndication_Error_gSmInvalidInternalOperation_c:
                    shell_write(gSmInvalidInternalOperation_c);
                    break;

                case GATTClientProcedureDiscoverPrimaryServicesByUuidIndication_Error_gSmInvalidConnectionHandle_c:
                    shell_write(gSmInvalidConnectionHandle_c);
                    break;

                case GATTClientProcedureDiscoverPrimaryServicesByUuidIndication_Error_gSmInproperKeyDistributionField_c:
                    shell_write(gSmInproperKeyDistributionField_c);
                    break;

                case GATTClientProcedureDiscoverPrimaryServicesByUuidIndication_Error_gSmUnexpectedKeyType_c:
                    shell_write(gSmUnexpectedKeyType_c);
                    break;

                case GATTClientProcedureDiscoverPrimaryServicesByUuidIndication_Error_gSmUnexpectedPairingTerminationReason_c:
                    shell_write(gSmUnexpectedPairingTerminationReason_c);
                    break;

                case GATTClientProcedureDiscoverPrimaryServicesByUuidIndication_Error_gSmUnexpectedKeyset_c:
                    shell_write(gSmUnexpectedKeyset_c);
                    break;

                case GATTClientProcedureDiscoverPrimaryServicesByUuidIndication_Error_gSmSmpTimeoutOccurred_c:
                    shell_write(gSmSmpTimeoutOccurred_c);
                    break;

                case GATTClientProcedureDiscoverPrimaryServicesByUuidIndication_Error_gSmUnknownSmpPacketType_c:
                    shell_write(gSmUnknownSmpPacketType_c);
                    break;

                case GATTClientProcedureDiscoverPrimaryServicesByUuidIndication_Error_gSmInvalidSmpPacketLength_c:
                    shell_write(gSmInvalidSmpPacketLength_c);
                    break;

                case GATTClientProcedureDiscoverPrimaryServicesByUuidIndication_Error_gSmInvalidSmpPacketParameter_c:
                    shell_write(gSmInvalidSmpPacketParameter_c);
                    break;

                case GATTClientProcedureDiscoverPrimaryServicesByUuidIndication_Error_gSmReceivedUnexpectedSmpPacket_c:
                    shell_write(gSmReceivedUnexpectedSmpPacket_c);
                    break;

                case GATTClientProcedureDiscoverPrimaryServicesByUuidIndication_Error_gSmReceivedSmpPacketFromUnknownDevice_c:
                    shell_write(gSmReceivedSmpPacketFromUnknownDevice_c);
                    break;

                case GATTClientProcedureDiscoverPrimaryServicesByUuidIndication_Error_gSmReceivedUnexpectedHciEvent_c:
                    shell_write(gSmReceivedUnexpectedHciEvent_c);
                    break;

                case GATTClientProcedureDiscoverPrimaryServicesByUuidIndication_Error_gSmReceivedHciEventFromUnknownDevice_c:
                    shell_write(gSmReceivedHciEventFromUnknownDevice_c);
                    break;

                case GATTClientProcedureDiscoverPrimaryServicesByUuidIndication_Error_gSmInvalidHciEventParameter_c:
                    shell_write(gSmInvalidHciEventParameter_c);
                    break;

                case GATTClientProcedureDiscoverPrimaryServicesByUuidIndication_Error_gSmLlConnectionEncryptionInProgress_c:
                    shell_write(gSmLlConnectionEncryptionInProgress_c);
                    break;

                case GATTClientProcedureDiscoverPrimaryServicesByUuidIndication_Error_gSmLlConnectionEncryptionFailure_c:
                    shell_write(gSmLlConnectionEncryptionFailure_c);
                    break;

                case GATTClientProcedureDiscoverPrimaryServicesByUuidIndication_Error_gSmInsufficientResources_c:
                    shell_write(gSmInsufficientResources_c);
                    break;

                case GATTClientProcedureDiscoverPrimaryServicesByUuidIndication_Error_gSmOobDataAddressMismatch_c:
                    shell_write(gSmOobDataAddressMismatch_c);
                    break;

//                case GATTClientProcedureDiscoverPrimaryServicesByUuidIndication_Error_gSmSmpPacketReceivedAfterTimeoutOccurred_c:
//                    shell_write(gSmSmpPacketReceivedAfterTimeoutOccurred_c);
//                    break;

                case GATTClientProcedureDiscoverPrimaryServicesByUuidIndication_Error_gSmPairingErrorPasskeyEntryFailed_c:
                    shell_write(gSmPairingErrorPasskeyEntryFailed_c);
                    break;

                case GATTClientProcedureDiscoverPrimaryServicesByUuidIndication_Error_gSmPairingErrorConfirmValueFailed_c:
                    shell_write(gSmPairingErrorConfirmValueFailed_c);
                    break;

                case GATTClientProcedureDiscoverPrimaryServicesByUuidIndication_Error_gSmPairingErrorCommandNotSupported_c:
                    shell_write(gSmPairingErrorCommandNotSupported_c);
                    break;

                case GATTClientProcedureDiscoverPrimaryServicesByUuidIndication_Error_gSmPairingErrorInvalidParameters_c:
                    shell_write(gSmPairingErrorInvalidParameters_c);
                    break;

                case GATTClientProcedureDiscoverPrimaryServicesByUuidIndication_Error_gSmPairingErrorUnknownReason_c:
                    shell_write(gSmPairingErrorUnknownReason_c);
                    break;

                case GATTClientProcedureDiscoverPrimaryServicesByUuidIndication_Error_gSmTbResolvableAddressDoesNotMatchIrk_c:
                    shell_write(gSmTbResolvableAddressDoesNotMatchIrk_c);
                    break;

                case GATTClientProcedureDiscoverPrimaryServicesByUuidIndication_Error_gSmTbInvalidDataSignature_c:
                    shell_write(gSmTbInvalidDataSignature_c);
                    break;

                case GATTClientProcedureDiscoverPrimaryServicesByUuidIndication_Error_gAttInvalidHandle_c:
                    shell_write(gAttInvalidHandle_c);
                    break;

                case GATTClientProcedureDiscoverPrimaryServicesByUuidIndication_Error_gAttReadNotPermitted_c:
                    shell_write(gAttReadNotPermitted_c);
                    break;

                case GATTClientProcedureDiscoverPrimaryServicesByUuidIndication_Error_gAttWriteNotPermitted_c:
                    shell_write(gAttWriteNotPermitted_c);
                    break;

                case GATTClientProcedureDiscoverPrimaryServicesByUuidIndication_Error_gAttInvalidPdu_c:
                    shell_write(gAttInvalidPdu_c);
                    break;

                case GATTClientProcedureDiscoverPrimaryServicesByUuidIndication_Error_gAttInsufficientAuthentication_c:
                    shell_write(gAttInsufficientAuthentication_c);
                    break;

                case GATTClientProcedureDiscoverPrimaryServicesByUuidIndication_Error_gAttRequestNotSupported_c:
                    shell_write(gAttRequestNotSupported_c);
                    break;

                case GATTClientProcedureDiscoverPrimaryServicesByUuidIndication_Error_gAttInvalidOffset_c:
                    shell_write(gAttInvalidOffset_c);
                    break;

                case GATTClientProcedureDiscoverPrimaryServicesByUuidIndication_Error_gAttInsufficientAuthorization_c:
                    shell_write(gAttInsufficientAuthorization_c);
                    break;

                case GATTClientProcedureDiscoverPrimaryServicesByUuidIndication_Error_gAttPrepareQueueFull_c:
                    shell_write(gAttPrepareQueueFull_c);
                    break;

                case GATTClientProcedureDiscoverPrimaryServicesByUuidIndication_Error_gAttAttributeNotFound_c:
                    shell_write(gAttAttributeNotFound_c);
                    break;

                case GATTClientProcedureDiscoverPrimaryServicesByUuidIndication_Error_gAttAttributeNotLong_c:
                    shell_write(gAttAttributeNotLong_c);
                    break;

                case GATTClientProcedureDiscoverPrimaryServicesByUuidIndication_Error_gAttInsufficientEncryptionKeySize_c:
                    shell_write(gAttInsufficientEncryptionKeySize_c);
                    break;

                case GATTClientProcedureDiscoverPrimaryServicesByUuidIndication_Error_gAttInvalidAttributeValueLength_c:
                    shell_write(gAttInvalidAttributeValueLength_c);
                    break;

                case GATTClientProcedureDiscoverPrimaryServicesByUuidIndication_Error_gAttUnlikelyor_c:
                    shell_write(gAttUnlikelyor_c);
                    break;

                case GATTClientProcedureDiscoverPrimaryServicesByUuidIndication_Error_gAttInsufficientEncryption_c:
                    shell_write(gAttInsufficientEncryption_c);
                    break;

                case GATTClientProcedureDiscoverPrimaryServicesByUuidIndication_Error_gAttUnsupportedGroupType_c:
                    shell_write(gAttUnsupportedGroupType_c);
                    break;

                case GATTClientProcedureDiscoverPrimaryServicesByUuidIndication_Error_gAttInsufficientResources_c:
                    shell_write(gAttInsufficientResources_c);
                    break;

                case GATTClientProcedureDiscoverPrimaryServicesByUuidIndication_Error_gGattAnotherProcedureInProgress_c:
                    shell_write(gGattAnotherProcedureInProgress_c);
                    break;

                case GATTClientProcedureDiscoverPrimaryServicesByUuidIndication_Error_gGattLongAttributePacketsCorrupted_c:
                    shell_write(gGattLongAttributePacketsCorrupted_c);
                    break;

                case GATTClientProcedureDiscoverPrimaryServicesByUuidIndication_Error_gGattMultipleAttributesOverflow_c:
                    shell_write(gGattMultipleAttributesOverflow_c);
                    break;

                case GATTClientProcedureDiscoverPrimaryServicesByUuidIndication_Error_gGattUnexpectedReadMultipleResponseLength_c:
                    shell_write(gGattUnexpectedReadMultipleResponseLength_c);
                    break;

                case GATTClientProcedureDiscoverPrimaryServicesByUuidIndication_Error_gGattInvalidValueLength_c:
                    shell_write(gGattInvalidValueLength_c);
                    break;

                case GATTClientProcedureDiscoverPrimaryServicesByUuidIndication_Error_gGattServerTimeout_c:
                    shell_write(gGattServerTimeout_c);
                    break;

                case GATTClientProcedureDiscoverPrimaryServicesByUuidIndication_Error_gGattIndicationAlreadyInProgress_c:
                    shell_write(gGattIndicationAlreadyInProgress_c);
                    break;

                case GATTClientProcedureDiscoverPrimaryServicesByUuidIndication_Error_gGattClientConfirmationTimeout_c:
                    shell_write(gGattClientConfirmationTimeout_c);
                    break;

                case GATTClientProcedureDiscoverPrimaryServicesByUuidIndication_Error_gGapAdvDataTooLong_c:
                    shell_write(gGapAdvDataTooLong_c);
                    break;

                case GATTClientProcedureDiscoverPrimaryServicesByUuidIndication_Error_gGapScanRspDataTooLong_c:
                    shell_write(gGapScanRspDataTooLong_c);
                    break;

                case GATTClientProcedureDiscoverPrimaryServicesByUuidIndication_Error_gGapDeviceNotBonded_c:
                    shell_write(gGapDeviceNotBonded_c);
                    break;

                case GATTClientProcedureDiscoverPrimaryServicesByUuidIndication_Error_gDevDbCccdLimitReached_c:
                    shell_write(gDevDbCccdLimitReached_c);
                    break;

                case GATTClientProcedureDiscoverPrimaryServicesByUuidIndication_Error_gDevDbCccdNotFound_c:
                    shell_write(gDevDbCccdNotFound_c);
                    break;

                case GATTClientProcedureDiscoverPrimaryServicesByUuidIndication_Error_gGattDbInvalidHandle_c:
                    shell_write(gGattDbInvalidHandle_c);
                    break;

                case GATTClientProcedureDiscoverPrimaryServicesByUuidIndication_Error_gGattDbCharacteristicNotFound_c:
                    shell_write(gGattDbCharacteristicNotFound_c);
                    break;

                case GATTClientProcedureDiscoverPrimaryServicesByUuidIndication_Error_gGattDbCccdNotFound_c:
                    shell_write(gGattDbCccdNotFound_c);
                    break;

                case GATTClientProcedureDiscoverPrimaryServicesByUuidIndication_Error_gGattDbServiceNotFound_c:
                    shell_write(gGattDbServiceNotFound_c);
                    break;

                case GATTClientProcedureDiscoverPrimaryServicesByUuidIndication_Error_gGattDbDescriptorNotFound_c:
                    shell_write(gGattDbDescriptorNotFound_c);
                    break;

                default:
                    shell_printf("Unrecognized status 0x%04X", container->Data.GATTClientProcedureDiscoverPrimaryServicesByUuidIndication.Error);
                    break;
            }

            break;

        case 0x4485:
            shell_write("\r\nGATTClientProcedureFindIncludedServicesIndication");
            shell_write(" -> ");

            switch (container->Data.GATTClientProcedureFindIncludedServicesIndication.ProcedureResult)
            {
                case GATTClientProcedureFindIncludedServicesIndication_ProcedureResult_gGattProcSuccess_c:
                    shell_write(gGattProcSuccess_c);
                    break;

                case GATTClientProcedureFindIncludedServicesIndication_ProcedureResult_gProcedureError_c:
                    shell_write(gProcedureError_c);
                    break;

                default:
                    shell_printf("Unrecognized status 0x%02X", container->Data.GATTClientProcedureFindIncludedServicesIndication.ProcedureResult);
                    break;
            }

            shell_write(" -> ");

            switch (container->Data.GATTClientProcedureFindIncludedServicesIndication.Error)
            {
                case GATTClientProcedureFindIncludedServicesIndication_Error_gBleSuccess_c:
                    shell_write(gBleSuccess_c);
                    break;

                case GATTClientProcedureFindIncludedServicesIndication_Error_gBleInvalidParameter_c:
                    shell_write(gBleInvalidParameter_c);
                    break;

                case GATTClientProcedureFindIncludedServicesIndication_Error_gBleOverflow_c:
                    shell_write(gBleOverflow_c);
                    break;

                case GATTClientProcedureFindIncludedServicesIndication_Error_gBleUnavailable_c:
                    shell_write(gBleUnavailable_c);
                    break;

                case GATTClientProcedureFindIncludedServicesIndication_Error_gBleFeatureNotSupported_c:
                    shell_write(gBleFeatureNotSupported_c);
                    break;

                case GATTClientProcedureFindIncludedServicesIndication_Error_gBleOutOfMemory_c:
                    shell_write(gBleOutOfMemory_c);
                    break;

                case GATTClientProcedureFindIncludedServicesIndication_Error_gBleAlreadyInitialized_c:
                    shell_write(gBleAlreadyInitialized_c);
                    break;

                case GATTClientProcedureFindIncludedServicesIndication_Error_gBleOsError_c:
                    shell_write(gBleOsc);
                    break;

                case GATTClientProcedureFindIncludedServicesIndication_Error_gBleUnexpectedError_c:
                    shell_write(gBleUnexpectedc);
                    break;

                case GATTClientProcedureFindIncludedServicesIndication_Error_gBleInvalidState_c:
                    shell_write(gBleInvalidState_c);
                    break;

                case GATTClientProcedureFindIncludedServicesIndication_Error_gHciUnknownHciCommand_c:
                    shell_write(gHciUnknownHciCommand_c);
                    break;

                case GATTClientProcedureFindIncludedServicesIndication_Error_gHciUnknownConnectionIdentifier_c:
                    shell_write(gHciUnknownConnectionIdentifier_c);
                    break;

                case GATTClientProcedureFindIncludedServicesIndication_Error_gHciHardwareFailure_c:
                    shell_write(gHciHardwareFailure_c);
                    break;

                case GATTClientProcedureFindIncludedServicesIndication_Error_gHciPageTimeout_c:
                    shell_write(gHciPageTimeout_c);
                    break;

                case GATTClientProcedureFindIncludedServicesIndication_Error_gHciAuthenticationFailure_c:
                    shell_write(gHciAuthenticationFailure_c);
                    break;

                case GATTClientProcedureFindIncludedServicesIndication_Error_gHciPinOrKeyMissing_c:
                    shell_write(gHciPinOrKeyMissing_c);
                    break;

                case GATTClientProcedureFindIncludedServicesIndication_Error_gHciMemoryCapacityExceeded_c:
                    shell_write(gHciMemoryCapacityExceeded_c);
                    break;

                case GATTClientProcedureFindIncludedServicesIndication_Error_gHciConnectionTimeout_c:
                    shell_write(gHciConnectionTimeout_c);
                    break;

                case GATTClientProcedureFindIncludedServicesIndication_Error_gHciConnectionLimitExceeded_c:
                    shell_write(gHciConnectionLimitExceeded_c);
                    break;

                case GATTClientProcedureFindIncludedServicesIndication_Error_gHciSynchronousConnectionLimitToADeviceExceeded_c:
                    shell_write(gHciSynchronousConnectionLimitToADeviceExceeded_c);
                    break;

                case GATTClientProcedureFindIncludedServicesIndication_Error_gHciAclConnectionAlreadyExists_c:
                    shell_write(gHciAclConnectionAlreadyExists_c);
                    break;

                case GATTClientProcedureFindIncludedServicesIndication_Error_gHciCommandDisallowed_c:
                    shell_write(gHciCommandDisallowed_c);
                    break;

                case GATTClientProcedureFindIncludedServicesIndication_Error_gHciConnectionRejectedDueToLimitedResources_c:
                    shell_write(gHciConnectionRejectedDueToLimitedResources_c);
                    break;

                case GATTClientProcedureFindIncludedServicesIndication_Error_gHciConnectionRejectedDueToSecurityReasons_c:
                    shell_write(gHciConnectionRejectedDueToSecurityReasons_c);
                    break;

                case GATTClientProcedureFindIncludedServicesIndication_Error_gHciConnectionRejectedDueToUnacceptableBdAddr_c:
                    shell_write(gHciConnectionRejectedDueToUnacceptableBdAddr_c);
                    break;

                case GATTClientProcedureFindIncludedServicesIndication_Error_gHciConnectionAcceptTimeoutExceeded_c:
                    shell_write(gHciConnectionAcceptTimeoutExceeded_c);
                    break;

                case GATTClientProcedureFindIncludedServicesIndication_Error_gHciUnsupportedFeatureOrParameterValue_c:
                    shell_write(gHciUnsupportedFeatureOrParameterValue_c);
                    break;

                case GATTClientProcedureFindIncludedServicesIndication_Error_gHciInvalidHciCommandParameters_c:
                    shell_write(gHciInvalidHciCommandParameters_c);
                    break;

                case GATTClientProcedureFindIncludedServicesIndication_Error_gHciRemoteUserTerminatedConnection_c:
                    shell_write(gHciRemoteUserTerminatedConnection_c);
                    break;

                case GATTClientProcedureFindIncludedServicesIndication_Error_gHciRemoteDeviceTerminatedConnectionLowResources_c:
                    shell_write(gHciRemoteDeviceTerminatedConnectionLowResources_c);
                    break;

                case GATTClientProcedureFindIncludedServicesIndication_Error_gHciRemoteDeviceTerminatedConnectionPowerOff_c:
                    shell_write(gHciRemoteDeviceTerminatedConnectionPowerOff_c);
                    break;

                case GATTClientProcedureFindIncludedServicesIndication_Error_gHciConnectionTerminatedByLocalHost_c:
                    shell_write(gHciConnectionTerminatedByLocalHost_c);
                    break;

                case GATTClientProcedureFindIncludedServicesIndication_Error_gHciRepeatedAttempts_c:
                    shell_write(gHciRepeatedAttempts_c);
                    break;

                case GATTClientProcedureFindIncludedServicesIndication_Error_gHciPairingNotAllowed_c:
                    shell_write(gHciPairingNotAllowed_c);
                    break;

                case GATTClientProcedureFindIncludedServicesIndication_Error_gHciUnknownLpmPdu_c:
                    shell_write(gHciUnknownLpmPdu_c);
                    break;

                case GATTClientProcedureFindIncludedServicesIndication_Error_gHciUnsupportedRemoteFeature_c:
                    shell_write(gHciUnsupportedRemoteFeature_c);
                    break;

                case GATTClientProcedureFindIncludedServicesIndication_Error_gHciScoOffsetRejected_c:
                    shell_write(gHciScoOffsetRejected_c);
                    break;

                case GATTClientProcedureFindIncludedServicesIndication_Error_gHciScoIntervalRejected_c:
                    shell_write(gHciScoIntervalRejected_c);
                    break;

                case GATTClientProcedureFindIncludedServicesIndication_Error_gHciScoAirModeRejected_c:
                    shell_write(gHciScoAirModeRejected_c);
                    break;

                case GATTClientProcedureFindIncludedServicesIndication_Error_gHciInvalidLpmParameters_c:
                    shell_write(gHciInvalidLpmParameters_c);
                    break;

                case GATTClientProcedureFindIncludedServicesIndication_Error_gHciUnspecifiedError_c:
                    shell_write(gHciUnspecifiedc);
                    break;

                case GATTClientProcedureFindIncludedServicesIndication_Error_gHciUnsupportedLpmParameterValue_c:
                    shell_write(gHciUnsupportedLpmParameterValue_c);
                    break;

                case GATTClientProcedureFindIncludedServicesIndication_Error_gHciRoleChangeNotAllowed_c:
                    shell_write(gHciRoleChangeNotAllowed_c);
                    break;

                case GATTClientProcedureFindIncludedServicesIndication_Error_gHciLLResponseTimeout_c:
                    shell_write(gHciLLResponseTimeout_c);
                    break;

                case GATTClientProcedureFindIncludedServicesIndication_Error_gHciLmpErrorTransactionCollision_c:
                    shell_write(gHciLmpErrorTransactionCollision_c);
                    break;

                case GATTClientProcedureFindIncludedServicesIndication_Error_gHciLmpPduNotAllowed_c:
                    shell_write(gHciLmpPduNotAllowed_c);
                    break;

                case GATTClientProcedureFindIncludedServicesIndication_Error_gHciEncryptionModeNotAcceptable_c:
                    shell_write(gHciEncryptionModeNotAcceptable_c);
                    break;

                case GATTClientProcedureFindIncludedServicesIndication_Error_gHciLinkKeyCannotBeChanged_c:
                    shell_write(gHciLinkKeyCannotBeChanged_c);
                    break;

                case GATTClientProcedureFindIncludedServicesIndication_Error_gHciRequestedQosNotSupported_c:
                    shell_write(gHciRequestedQosNotSupported_c);
                    break;

                case GATTClientProcedureFindIncludedServicesIndication_Error_gHciInstantPassed_c:
                    shell_write(gHciInstantPassed_c);
                    break;

                case GATTClientProcedureFindIncludedServicesIndication_Error_gHciPairingWithUnitKeyNotSupported_c:
                    shell_write(gHciPairingWithUnitKeyNotSupported_c);
                    break;

                case GATTClientProcedureFindIncludedServicesIndication_Error_gHciDifferentTransactionCollision_c:
                    shell_write(gHciDifferentTransactionCollision_c);
                    break;

                case GATTClientProcedureFindIncludedServicesIndication_Error_gHciReserved_0x2B_c:
                    shell_write(gHciReserved_0x2B_c);
                    break;

                case GATTClientProcedureFindIncludedServicesIndication_Error_gHciQosNotAcceptableParameter_c:
                    shell_write(gHciQosNotAcceptableParameter_c);
                    break;

                case GATTClientProcedureFindIncludedServicesIndication_Error_gHciQosRejected_c:
                    shell_write(gHciQosRejected_c);
                    break;

                case GATTClientProcedureFindIncludedServicesIndication_Error_gHciChannelClassificationNotSupported_c:
                    shell_write(gHciChannelClassificationNotSupported_c);
                    break;

                case GATTClientProcedureFindIncludedServicesIndication_Error_gHciInsufficientSecurity_c:
                    shell_write(gHciInsufficientSecurity_c);
                    break;

                case GATTClientProcedureFindIncludedServicesIndication_Error_gHciParameterOutOfMandatoryRange_c:
                    shell_write(gHciParameterOutOfMandatoryRange_c);
                    break;

                case GATTClientProcedureFindIncludedServicesIndication_Error_gHciReserved_0x31_c:
                    shell_write(gHciReserved_0x31_c);
                    break;

                case GATTClientProcedureFindIncludedServicesIndication_Error_gHciRoleSwitchPending_c:
                    shell_write(gHciRoleSwitchPending_c);
                    break;

                case GATTClientProcedureFindIncludedServicesIndication_Error_gHciReserved_0x33_c:
                    shell_write(gHciReserved_0x33_c);
                    break;

                case GATTClientProcedureFindIncludedServicesIndication_Error_gHciReservedSlotViolation_c:
                    shell_write(gHciReservedSlotViolation_c);
                    break;

                case GATTClientProcedureFindIncludedServicesIndication_Error_gHciRoleSwitchFailed_c:
                    shell_write(gHciRoleSwitchFailed_c);
                    break;

                case GATTClientProcedureFindIncludedServicesIndication_Error_gHciExtendedInquiryResponseTooLarge_c:
                    shell_write(gHciExtendedInquiryResponseTooLarge_c);
                    break;

                case GATTClientProcedureFindIncludedServicesIndication_Error_gHciSecureSimplePairingNotSupportedByHost_c:
                    shell_write(gHciSecureSimplePairingNotSupportedByHost_c);
                    break;

                case GATTClientProcedureFindIncludedServicesIndication_Error_gHciHostBusyPairing_c:
                    shell_write(gHciHostBusyPairing_c);
                    break;

                case GATTClientProcedureFindIncludedServicesIndication_Error_gHciConnectionRejectedDueToNoSuitableChannelFound_c:
                    shell_write(gHciConnectionRejectedDueToNoSuitableChannelFound_c);
                    break;

                case GATTClientProcedureFindIncludedServicesIndication_Error_gHciControllerBusy_c:
                    shell_write(gHciControllerBusy_c);
                    break;

                case GATTClientProcedureFindIncludedServicesIndication_Error_gHciUnacceptableConnectionParameters_c:
                    shell_write(gHciUnacceptableConnectionParameters_c);
                    break;

                case GATTClientProcedureFindIncludedServicesIndication_Error_gHciDirectedAdvertisingTimeout_c:
                    shell_write(gHciDirectedAdvertisingTimeout_c);
                    break;

                case GATTClientProcedureFindIncludedServicesIndication_Error_gHciConnectionTerminatedDueToMicFailure_c:
                    shell_write(gHciConnectionTerminatedDueToMicFailure_c);
                    break;

                case GATTClientProcedureFindIncludedServicesIndication_Error_gHciConnectionFailedToBeEstablished_c:
                    shell_write(gHciConnectionFailedToBeEstablished_c);
                    break;

                case GATTClientProcedureFindIncludedServicesIndication_Error_gHciMacConnectionFailed_c:
                    shell_write(gHciMacConnectionFailed_c);
                    break;

                case GATTClientProcedureFindIncludedServicesIndication_Error_gHciCoarseClockAdjustmentRejected_c:
                    shell_write(gHciCoarseClockAdjustmentRejected_c);
                    break;

                case GATTClientProcedureFindIncludedServicesIndication_Error_gHciAlreadyInit_c:
                    shell_write(gHciAlreadyInit_c);
                    break;

                case GATTClientProcedureFindIncludedServicesIndication_Error_gHciInvalidParameter_c:
                    shell_write(gHciInvalidParameter_c);
                    break;

                case GATTClientProcedureFindIncludedServicesIndication_Error_gHciCallbackNotInstalled_c:
                    shell_write(gHciCallbackNotInstalled_c);
                    break;

                case GATTClientProcedureFindIncludedServicesIndication_Error_gHciCallbackAlreadyInstalled_c:
                    shell_write(gHciCallbackAlreadyInstalled_c);
                    break;

                case GATTClientProcedureFindIncludedServicesIndication_Error_gHciCommandNotSupported_c:
                    shell_write(gHciCommandNotSupported_c);
                    break;

                case GATTClientProcedureFindIncludedServicesIndication_Error_gHciEventNotSupported_c:
                    shell_write(gHciEventNotSupported_c);
                    break;

                case GATTClientProcedureFindIncludedServicesIndication_Error_gHciTransportError_c:
                    shell_write(gHciTransportc);
                    break;

                case GATTClientProcedureFindIncludedServicesIndication_Error_gL2caAlreadyInit_c:
                    shell_write(gL2caAlreadyInit_c);
                    break;

                case GATTClientProcedureFindIncludedServicesIndication_Error_gL2caInsufficientResources_c:
                    shell_write(gL2caInsufficientResources_c);
                    break;

                case GATTClientProcedureFindIncludedServicesIndication_Error_gL2caCallbackNotInstalled_c:
                    shell_write(gL2caCallbackNotInstalled_c);
                    break;

                case GATTClientProcedureFindIncludedServicesIndication_Error_gL2caCallbackAlreadyInstalled_c:
                    shell_write(gL2caCallbackAlreadyInstalled_c);
                    break;

                case GATTClientProcedureFindIncludedServicesIndication_Error_gL2caLePsmInvalid_c:
                    shell_write(gL2caLePsmInvalid_c);
                    break;

                case GATTClientProcedureFindIncludedServicesIndication_Error_gL2caLePsmAlreadyRegistered_c:
                    shell_write(gL2caLePsmAlreadyRegistered_c);
                    break;

                case GATTClientProcedureFindIncludedServicesIndication_Error_gL2caLePsmNotRegistered_c:
                    shell_write(gL2caLePsmNotRegistered_c);
                    break;

                case GATTClientProcedureFindIncludedServicesIndication_Error_gL2caLePsmInsufficientResources_c:
                    shell_write(gL2caLePsmInsufficientResources_c);
                    break;

                case GATTClientProcedureFindIncludedServicesIndication_Error_gL2caChannelInvalid_c:
                    shell_write(gL2caChannelInvalid_c);
                    break;

                case GATTClientProcedureFindIncludedServicesIndication_Error_gL2caChannelClosed_c:
                    shell_write(gL2caChannelClosed_c);
                    break;

                case GATTClientProcedureFindIncludedServicesIndication_Error_gL2caChannelAlreadyConnected_c:
                    shell_write(gL2caChannelAlreadyConnected_c);
                    break;

                case GATTClientProcedureFindIncludedServicesIndication_Error_gL2caConnectionParametersRejected_c:
                    shell_write(gL2caConnectionParametersRejected_c);
                    break;

                case GATTClientProcedureFindIncludedServicesIndication_Error_gL2caChannelBusy_c:
                    shell_write(gL2caChannelBusy_c);
                    break;

                case GATTClientProcedureFindIncludedServicesIndication_Error_gL2caInvalidParameter_c:
                    shell_write(gL2caInvalidParameter_c);
                    break;

                case GATTClientProcedureFindIncludedServicesIndication_Error_gL2caError_c:
                    shell_write(gL2cac);
                    break;

                case GATTClientProcedureFindIncludedServicesIndication_Error_gSmNullCBFunction_c:
                    shell_write(gSmNullCBFunction_c);
                    break;

                case GATTClientProcedureFindIncludedServicesIndication_Error_gSmCommandNotSupported_c:
                    shell_write(gSmCommandNotSupported_c);
                    break;

                case GATTClientProcedureFindIncludedServicesIndication_Error_gSmUnexpectedCommand_c:
                    shell_write(gSmUnexpectedCommand_c);
                    break;

                case GATTClientProcedureFindIncludedServicesIndication_Error_gSmInvalidCommandCode_c:
                    shell_write(gSmInvalidCommandCode_c);
                    break;

                case GATTClientProcedureFindIncludedServicesIndication_Error_gSmInvalidCommandLength_c:
                    shell_write(gSmInvalidCommandLength_c);
                    break;

                case GATTClientProcedureFindIncludedServicesIndication_Error_gSmInvalidCommandParameter_c:
                    shell_write(gSmInvalidCommandParameter_c);
                    break;

                case GATTClientProcedureFindIncludedServicesIndication_Error_gSmInvalidDeviceId_c:
                    shell_write(gSmInvalidDeviceId_c);
                    break;

                case GATTClientProcedureFindIncludedServicesIndication_Error_gSmInvalidInternalOperation_c:
                    shell_write(gSmInvalidInternalOperation_c);
                    break;

                case GATTClientProcedureFindIncludedServicesIndication_Error_gSmInvalidConnectionHandle_c:
                    shell_write(gSmInvalidConnectionHandle_c);
                    break;

                case GATTClientProcedureFindIncludedServicesIndication_Error_gSmInproperKeyDistributionField_c:
                    shell_write(gSmInproperKeyDistributionField_c);
                    break;

                case GATTClientProcedureFindIncludedServicesIndication_Error_gSmUnexpectedKeyType_c:
                    shell_write(gSmUnexpectedKeyType_c);
                    break;

                case GATTClientProcedureFindIncludedServicesIndication_Error_gSmUnexpectedPairingTerminationReason_c:
                    shell_write(gSmUnexpectedPairingTerminationReason_c);
                    break;

                case GATTClientProcedureFindIncludedServicesIndication_Error_gSmUnexpectedKeyset_c:
                    shell_write(gSmUnexpectedKeyset_c);
                    break;

                case GATTClientProcedureFindIncludedServicesIndication_Error_gSmSmpTimeoutOccurred_c:
                    shell_write(gSmSmpTimeoutOccurred_c);
                    break;

                case GATTClientProcedureFindIncludedServicesIndication_Error_gSmUnknownSmpPacketType_c:
                    shell_write(gSmUnknownSmpPacketType_c);
                    break;

                case GATTClientProcedureFindIncludedServicesIndication_Error_gSmInvalidSmpPacketLength_c:
                    shell_write(gSmInvalidSmpPacketLength_c);
                    break;

                case GATTClientProcedureFindIncludedServicesIndication_Error_gSmInvalidSmpPacketParameter_c:
                    shell_write(gSmInvalidSmpPacketParameter_c);
                    break;

                case GATTClientProcedureFindIncludedServicesIndication_Error_gSmReceivedUnexpectedSmpPacket_c:
                    shell_write(gSmReceivedUnexpectedSmpPacket_c);
                    break;

                case GATTClientProcedureFindIncludedServicesIndication_Error_gSmReceivedSmpPacketFromUnknownDevice_c:
                    shell_write(gSmReceivedSmpPacketFromUnknownDevice_c);
                    break;

                case GATTClientProcedureFindIncludedServicesIndication_Error_gSmReceivedUnexpectedHciEvent_c:
                    shell_write(gSmReceivedUnexpectedHciEvent_c);
                    break;

                case GATTClientProcedureFindIncludedServicesIndication_Error_gSmReceivedHciEventFromUnknownDevice_c:
                    shell_write(gSmReceivedHciEventFromUnknownDevice_c);
                    break;

                case GATTClientProcedureFindIncludedServicesIndication_Error_gSmInvalidHciEventParameter_c:
                    shell_write(gSmInvalidHciEventParameter_c);
                    break;

                case GATTClientProcedureFindIncludedServicesIndication_Error_gSmLlConnectionEncryptionInProgress_c:
                    shell_write(gSmLlConnectionEncryptionInProgress_c);
                    break;

                case GATTClientProcedureFindIncludedServicesIndication_Error_gSmLlConnectionEncryptionFailure_c:
                    shell_write(gSmLlConnectionEncryptionFailure_c);
                    break;

                case GATTClientProcedureFindIncludedServicesIndication_Error_gSmInsufficientResources_c:
                    shell_write(gSmInsufficientResources_c);
                    break;

                case GATTClientProcedureFindIncludedServicesIndication_Error_gSmOobDataAddressMismatch_c:
                    shell_write(gSmOobDataAddressMismatch_c);
                    break;

//                case GATTClientProcedureFindIncludedServicesIndication_Error_gSmSmpPacketReceivedAfterTimeoutOccurred_c:
//                    shell_write(gSmSmpPacketReceivedAfterTimeoutOccurred_c);
//                    break;

                case GATTClientProcedureFindIncludedServicesIndication_Error_gSmPairingErrorPasskeyEntryFailed_c:
                    shell_write(gSmPairingErrorPasskeyEntryFailed_c);
                    break;

                case GATTClientProcedureFindIncludedServicesIndication_Error_gSmPairingErrorConfirmValueFailed_c:
                    shell_write(gSmPairingErrorConfirmValueFailed_c);
                    break;

                case GATTClientProcedureFindIncludedServicesIndication_Error_gSmPairingErrorCommandNotSupported_c:
                    shell_write(gSmPairingErrorCommandNotSupported_c);
                    break;

                case GATTClientProcedureFindIncludedServicesIndication_Error_gSmPairingErrorInvalidParameters_c:
                    shell_write(gSmPairingErrorInvalidParameters_c);
                    break;

                case GATTClientProcedureFindIncludedServicesIndication_Error_gSmPairingErrorUnknownReason_c:
                    shell_write(gSmPairingErrorUnknownReason_c);
                    break;

                case GATTClientProcedureFindIncludedServicesIndication_Error_gSmTbResolvableAddressDoesNotMatchIrk_c:
                    shell_write(gSmTbResolvableAddressDoesNotMatchIrk_c);
                    break;

                case GATTClientProcedureFindIncludedServicesIndication_Error_gSmTbInvalidDataSignature_c:
                    shell_write(gSmTbInvalidDataSignature_c);
                    break;

                case GATTClientProcedureFindIncludedServicesIndication_Error_gAttInvalidHandle_c:
                    shell_write(gAttInvalidHandle_c);
                    break;

                case GATTClientProcedureFindIncludedServicesIndication_Error_gAttReadNotPermitted_c:
                    shell_write(gAttReadNotPermitted_c);
                    break;

                case GATTClientProcedureFindIncludedServicesIndication_Error_gAttWriteNotPermitted_c:
                    shell_write(gAttWriteNotPermitted_c);
                    break;

                case GATTClientProcedureFindIncludedServicesIndication_Error_gAttInvalidPdu_c:
                    shell_write(gAttInvalidPdu_c);
                    break;

                case GATTClientProcedureFindIncludedServicesIndication_Error_gAttInsufficientAuthentication_c:
                    shell_write(gAttInsufficientAuthentication_c);
                    break;

                case GATTClientProcedureFindIncludedServicesIndication_Error_gAttRequestNotSupported_c:
                    shell_write(gAttRequestNotSupported_c);
                    break;

                case GATTClientProcedureFindIncludedServicesIndication_Error_gAttInvalidOffset_c:
                    shell_write(gAttInvalidOffset_c);
                    break;

                case GATTClientProcedureFindIncludedServicesIndication_Error_gAttInsufficientAuthorization_c:
                    shell_write(gAttInsufficientAuthorization_c);
                    break;

                case GATTClientProcedureFindIncludedServicesIndication_Error_gAttPrepareQueueFull_c:
                    shell_write(gAttPrepareQueueFull_c);
                    break;

                case GATTClientProcedureFindIncludedServicesIndication_Error_gAttAttributeNotFound_c:
                    shell_write(gAttAttributeNotFound_c);
                    break;

                case GATTClientProcedureFindIncludedServicesIndication_Error_gAttAttributeNotLong_c:
                    shell_write(gAttAttributeNotLong_c);
                    break;

                case GATTClientProcedureFindIncludedServicesIndication_Error_gAttInsufficientEncryptionKeySize_c:
                    shell_write(gAttInsufficientEncryptionKeySize_c);
                    break;

                case GATTClientProcedureFindIncludedServicesIndication_Error_gAttInvalidAttributeValueLength_c:
                    shell_write(gAttInvalidAttributeValueLength_c);
                    break;

                case GATTClientProcedureFindIncludedServicesIndication_Error_gAttUnlikelyor_c:
                    shell_write(gAttUnlikelyor_c);
                    break;

                case GATTClientProcedureFindIncludedServicesIndication_Error_gAttInsufficientEncryption_c:
                    shell_write(gAttInsufficientEncryption_c);
                    break;

                case GATTClientProcedureFindIncludedServicesIndication_Error_gAttUnsupportedGroupType_c:
                    shell_write(gAttUnsupportedGroupType_c);
                    break;

                case GATTClientProcedureFindIncludedServicesIndication_Error_gAttInsufficientResources_c:
                    shell_write(gAttInsufficientResources_c);
                    break;

                case GATTClientProcedureFindIncludedServicesIndication_Error_gGattAnotherProcedureInProgress_c:
                    shell_write(gGattAnotherProcedureInProgress_c);
                    break;

                case GATTClientProcedureFindIncludedServicesIndication_Error_gGattLongAttributePacketsCorrupted_c:
                    shell_write(gGattLongAttributePacketsCorrupted_c);
                    break;

                case GATTClientProcedureFindIncludedServicesIndication_Error_gGattMultipleAttributesOverflow_c:
                    shell_write(gGattMultipleAttributesOverflow_c);
                    break;

                case GATTClientProcedureFindIncludedServicesIndication_Error_gGattUnexpectedReadMultipleResponseLength_c:
                    shell_write(gGattUnexpectedReadMultipleResponseLength_c);
                    break;

                case GATTClientProcedureFindIncludedServicesIndication_Error_gGattInvalidValueLength_c:
                    shell_write(gGattInvalidValueLength_c);
                    break;

                case GATTClientProcedureFindIncludedServicesIndication_Error_gGattServerTimeout_c:
                    shell_write(gGattServerTimeout_c);
                    break;

                case GATTClientProcedureFindIncludedServicesIndication_Error_gGattIndicationAlreadyInProgress_c:
                    shell_write(gGattIndicationAlreadyInProgress_c);
                    break;

                case GATTClientProcedureFindIncludedServicesIndication_Error_gGattClientConfirmationTimeout_c:
                    shell_write(gGattClientConfirmationTimeout_c);
                    break;

                case GATTClientProcedureFindIncludedServicesIndication_Error_gGapAdvDataTooLong_c:
                    shell_write(gGapAdvDataTooLong_c);
                    break;

                case GATTClientProcedureFindIncludedServicesIndication_Error_gGapScanRspDataTooLong_c:
                    shell_write(gGapScanRspDataTooLong_c);
                    break;

                case GATTClientProcedureFindIncludedServicesIndication_Error_gGapDeviceNotBonded_c:
                    shell_write(gGapDeviceNotBonded_c);
                    break;

                case GATTClientProcedureFindIncludedServicesIndication_Error_gDevDbCccdLimitReached_c:
                    shell_write(gDevDbCccdLimitReached_c);
                    break;

                case GATTClientProcedureFindIncludedServicesIndication_Error_gDevDbCccdNotFound_c:
                    shell_write(gDevDbCccdNotFound_c);
                    break;

                case GATTClientProcedureFindIncludedServicesIndication_Error_gGattDbInvalidHandle_c:
                    shell_write(gGattDbInvalidHandle_c);
                    break;

                case GATTClientProcedureFindIncludedServicesIndication_Error_gGattDbCharacteristicNotFound_c:
                    shell_write(gGattDbCharacteristicNotFound_c);
                    break;

                case GATTClientProcedureFindIncludedServicesIndication_Error_gGattDbCccdNotFound_c:
                    shell_write(gGattDbCccdNotFound_c);
                    break;

                case GATTClientProcedureFindIncludedServicesIndication_Error_gGattDbServiceNotFound_c:
                    shell_write(gGattDbServiceNotFound_c);
                    break;

                case GATTClientProcedureFindIncludedServicesIndication_Error_gGattDbDescriptorNotFound_c:
                    shell_write(gGattDbDescriptorNotFound_c);
                    break;

                default:
                    shell_printf("Unrecognized status 0x%04X", container->Data.GATTClientProcedureFindIncludedServicesIndication.Error);
                    break;
            }

            break;

        case 0x4486:
            shell_write("\r\nGATTClientProcedureDiscoverAllCharacteristicsIndication");
            shell_write(" -> ");

            switch (container->Data.GATTClientProcedureDiscoverAllCharacteristicsIndication.ProcedureResult)
            {
                case GATTClientProcedureDiscoverAllCharacteristicsIndication_ProcedureResult_gGattProcSuccess_c:
                    shell_write(gGattProcSuccess_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicsIndication_ProcedureResult_gProcedureError_c:
                    shell_write(gProcedureError_c);
                    break;

                default:
                    shell_printf("Unrecognized status 0x%02X", container->Data.GATTClientProcedureDiscoverAllCharacteristicsIndication.ProcedureResult);
                    break;
            }

            shell_write(" -> ");

            switch (container->Data.GATTClientProcedureDiscoverAllCharacteristicsIndication.Error)
            {
                case GATTClientProcedureDiscoverAllCharacteristicsIndication_Error_gBleSuccess_c:
                    shell_write(gBleSuccess_c);
                    SHELL_PrintDiscoveredChars(&(container->Data.GATTClientProcedureDiscoverAllCharacteristicsIndication));
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicsIndication_Error_gBleInvalidParameter_c:
                    shell_write(gBleInvalidParameter_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicsIndication_Error_gBleOverflow_c:
                    shell_write(gBleOverflow_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicsIndication_Error_gBleUnavailable_c:
                    shell_write(gBleUnavailable_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicsIndication_Error_gBleFeatureNotSupported_c:
                    shell_write(gBleFeatureNotSupported_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicsIndication_Error_gBleOutOfMemory_c:
                    shell_write(gBleOutOfMemory_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicsIndication_Error_gBleAlreadyInitialized_c:
                    shell_write(gBleAlreadyInitialized_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicsIndication_Error_gBleOsError_c:
                    shell_write(gBleOsc);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicsIndication_Error_gBleUnexpectedError_c:
                    shell_write(gBleUnexpectedc);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicsIndication_Error_gBleInvalidState_c:
                    shell_write(gBleInvalidState_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicsIndication_Error_gHciUnknownHciCommand_c:
                    shell_write(gHciUnknownHciCommand_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicsIndication_Error_gHciUnknownConnectionIdentifier_c:
                    shell_write(gHciUnknownConnectionIdentifier_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicsIndication_Error_gHciHardwareFailure_c:
                    shell_write(gHciHardwareFailure_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicsIndication_Error_gHciPageTimeout_c:
                    shell_write(gHciPageTimeout_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicsIndication_Error_gHciAuthenticationFailure_c:
                    shell_write(gHciAuthenticationFailure_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicsIndication_Error_gHciPinOrKeyMissing_c:
                    shell_write(gHciPinOrKeyMissing_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicsIndication_Error_gHciMemoryCapacityExceeded_c:
                    shell_write(gHciMemoryCapacityExceeded_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicsIndication_Error_gHciConnectionTimeout_c:
                    shell_write(gHciConnectionTimeout_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicsIndication_Error_gHciConnectionLimitExceeded_c:
                    shell_write(gHciConnectionLimitExceeded_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicsIndication_Error_gHciSynchronousConnectionLimitToADeviceExceeded_c:
                    shell_write(gHciSynchronousConnectionLimitToADeviceExceeded_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicsIndication_Error_gHciAclConnectionAlreadyExists_c:
                    shell_write(gHciAclConnectionAlreadyExists_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicsIndication_Error_gHciCommandDisallowed_c:
                    shell_write(gHciCommandDisallowed_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicsIndication_Error_gHciConnectionRejectedDueToLimitedResources_c:
                    shell_write(gHciConnectionRejectedDueToLimitedResources_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicsIndication_Error_gHciConnectionRejectedDueToSecurityReasons_c:
                    shell_write(gHciConnectionRejectedDueToSecurityReasons_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicsIndication_Error_gHciConnectionRejectedDueToUnacceptableBdAddr_c:
                    shell_write(gHciConnectionRejectedDueToUnacceptableBdAddr_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicsIndication_Error_gHciConnectionAcceptTimeoutExceeded_c:
                    shell_write(gHciConnectionAcceptTimeoutExceeded_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicsIndication_Error_gHciUnsupportedFeatureOrParameterValue_c:
                    shell_write(gHciUnsupportedFeatureOrParameterValue_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicsIndication_Error_gHciInvalidHciCommandParameters_c:
                    shell_write(gHciInvalidHciCommandParameters_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicsIndication_Error_gHciRemoteUserTerminatedConnection_c:
                    shell_write(gHciRemoteUserTerminatedConnection_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicsIndication_Error_gHciRemoteDeviceTerminatedConnectionLowResources_c:
                    shell_write(gHciRemoteDeviceTerminatedConnectionLowResources_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicsIndication_Error_gHciRemoteDeviceTerminatedConnectionPowerOff_c:
                    shell_write(gHciRemoteDeviceTerminatedConnectionPowerOff_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicsIndication_Error_gHciConnectionTerminatedByLocalHost_c:
                    shell_write(gHciConnectionTerminatedByLocalHost_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicsIndication_Error_gHciRepeatedAttempts_c:
                    shell_write(gHciRepeatedAttempts_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicsIndication_Error_gHciPairingNotAllowed_c:
                    shell_write(gHciPairingNotAllowed_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicsIndication_Error_gHciUnknownLpmPdu_c:
                    shell_write(gHciUnknownLpmPdu_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicsIndication_Error_gHciUnsupportedRemoteFeature_c:
                    shell_write(gHciUnsupportedRemoteFeature_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicsIndication_Error_gHciScoOffsetRejected_c:
                    shell_write(gHciScoOffsetRejected_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicsIndication_Error_gHciScoIntervalRejected_c:
                    shell_write(gHciScoIntervalRejected_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicsIndication_Error_gHciScoAirModeRejected_c:
                    shell_write(gHciScoAirModeRejected_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicsIndication_Error_gHciInvalidLpmParameters_c:
                    shell_write(gHciInvalidLpmParameters_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicsIndication_Error_gHciUnspecifiedError_c:
                    shell_write(gHciUnspecifiedc);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicsIndication_Error_gHciUnsupportedLpmParameterValue_c:
                    shell_write(gHciUnsupportedLpmParameterValue_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicsIndication_Error_gHciRoleChangeNotAllowed_c:
                    shell_write(gHciRoleChangeNotAllowed_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicsIndication_Error_gHciLLResponseTimeout_c:
                    shell_write(gHciLLResponseTimeout_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicsIndication_Error_gHciLmpErrorTransactionCollision_c:
                    shell_write(gHciLmpErrorTransactionCollision_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicsIndication_Error_gHciLmpPduNotAllowed_c:
                    shell_write(gHciLmpPduNotAllowed_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicsIndication_Error_gHciEncryptionModeNotAcceptable_c:
                    shell_write(gHciEncryptionModeNotAcceptable_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicsIndication_Error_gHciLinkKeyCannotBeChanged_c:
                    shell_write(gHciLinkKeyCannotBeChanged_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicsIndication_Error_gHciRequestedQosNotSupported_c:
                    shell_write(gHciRequestedQosNotSupported_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicsIndication_Error_gHciInstantPassed_c:
                    shell_write(gHciInstantPassed_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicsIndication_Error_gHciPairingWithUnitKeyNotSupported_c:
                    shell_write(gHciPairingWithUnitKeyNotSupported_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicsIndication_Error_gHciDifferentTransactionCollision_c:
                    shell_write(gHciDifferentTransactionCollision_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicsIndication_Error_gHciReserved_0x2B_c:
                    shell_write(gHciReserved_0x2B_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicsIndication_Error_gHciQosNotAcceptableParameter_c:
                    shell_write(gHciQosNotAcceptableParameter_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicsIndication_Error_gHciQosRejected_c:
                    shell_write(gHciQosRejected_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicsIndication_Error_gHciChannelClassificationNotSupported_c:
                    shell_write(gHciChannelClassificationNotSupported_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicsIndication_Error_gHciInsufficientSecurity_c:
                    shell_write(gHciInsufficientSecurity_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicsIndication_Error_gHciParameterOutOfMandatoryRange_c:
                    shell_write(gHciParameterOutOfMandatoryRange_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicsIndication_Error_gHciReserved_0x31_c:
                    shell_write(gHciReserved_0x31_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicsIndication_Error_gHciRoleSwitchPending_c:
                    shell_write(gHciRoleSwitchPending_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicsIndication_Error_gHciReserved_0x33_c:
                    shell_write(gHciReserved_0x33_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicsIndication_Error_gHciReservedSlotViolation_c:
                    shell_write(gHciReservedSlotViolation_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicsIndication_Error_gHciRoleSwitchFailed_c:
                    shell_write(gHciRoleSwitchFailed_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicsIndication_Error_gHciExtendedInquiryResponseTooLarge_c:
                    shell_write(gHciExtendedInquiryResponseTooLarge_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicsIndication_Error_gHciSecureSimplePairingNotSupportedByHost_c:
                    shell_write(gHciSecureSimplePairingNotSupportedByHost_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicsIndication_Error_gHciHostBusyPairing_c:
                    shell_write(gHciHostBusyPairing_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicsIndication_Error_gHciConnectionRejectedDueToNoSuitableChannelFound_c:
                    shell_write(gHciConnectionRejectedDueToNoSuitableChannelFound_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicsIndication_Error_gHciControllerBusy_c:
                    shell_write(gHciControllerBusy_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicsIndication_Error_gHciUnacceptableConnectionParameters_c:
                    shell_write(gHciUnacceptableConnectionParameters_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicsIndication_Error_gHciDirectedAdvertisingTimeout_c:
                    shell_write(gHciDirectedAdvertisingTimeout_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicsIndication_Error_gHciConnectionTerminatedDueToMicFailure_c:
                    shell_write(gHciConnectionTerminatedDueToMicFailure_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicsIndication_Error_gHciConnectionFailedToBeEstablished_c:
                    shell_write(gHciConnectionFailedToBeEstablished_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicsIndication_Error_gHciMacConnectionFailed_c:
                    shell_write(gHciMacConnectionFailed_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicsIndication_Error_gHciCoarseClockAdjustmentRejected_c:
                    shell_write(gHciCoarseClockAdjustmentRejected_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicsIndication_Error_gHciAlreadyInit_c:
                    shell_write(gHciAlreadyInit_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicsIndication_Error_gHciInvalidParameter_c:
                    shell_write(gHciInvalidParameter_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicsIndication_Error_gHciCallbackNotInstalled_c:
                    shell_write(gHciCallbackNotInstalled_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicsIndication_Error_gHciCallbackAlreadyInstalled_c:
                    shell_write(gHciCallbackAlreadyInstalled_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicsIndication_Error_gHciCommandNotSupported_c:
                    shell_write(gHciCommandNotSupported_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicsIndication_Error_gHciEventNotSupported_c:
                    shell_write(gHciEventNotSupported_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicsIndication_Error_gHciTransportError_c:
                    shell_write(gHciTransportc);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicsIndication_Error_gL2caAlreadyInit_c:
                    shell_write(gL2caAlreadyInit_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicsIndication_Error_gL2caInsufficientResources_c:
                    shell_write(gL2caInsufficientResources_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicsIndication_Error_gL2caCallbackNotInstalled_c:
                    shell_write(gL2caCallbackNotInstalled_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicsIndication_Error_gL2caCallbackAlreadyInstalled_c:
                    shell_write(gL2caCallbackAlreadyInstalled_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicsIndication_Error_gL2caLePsmInvalid_c:
                    shell_write(gL2caLePsmInvalid_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicsIndication_Error_gL2caLePsmAlreadyRegistered_c:
                    shell_write(gL2caLePsmAlreadyRegistered_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicsIndication_Error_gL2caLePsmNotRegistered_c:
                    shell_write(gL2caLePsmNotRegistered_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicsIndication_Error_gL2caLePsmInsufficientResources_c:
                    shell_write(gL2caLePsmInsufficientResources_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicsIndication_Error_gL2caChannelInvalid_c:
                    shell_write(gL2caChannelInvalid_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicsIndication_Error_gL2caChannelClosed_c:
                    shell_write(gL2caChannelClosed_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicsIndication_Error_gL2caChannelAlreadyConnected_c:
                    shell_write(gL2caChannelAlreadyConnected_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicsIndication_Error_gL2caConnectionParametersRejected_c:
                    shell_write(gL2caConnectionParametersRejected_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicsIndication_Error_gL2caChannelBusy_c:
                    shell_write(gL2caChannelBusy_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicsIndication_Error_gL2caInvalidParameter_c:
                    shell_write(gL2caInvalidParameter_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicsIndication_Error_gL2caError_c:
                    shell_write(gL2cac);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicsIndication_Error_gSmNullCBFunction_c:
                    shell_write(gSmNullCBFunction_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicsIndication_Error_gSmCommandNotSupported_c:
                    shell_write(gSmCommandNotSupported_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicsIndication_Error_gSmUnexpectedCommand_c:
                    shell_write(gSmUnexpectedCommand_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicsIndication_Error_gSmInvalidCommandCode_c:
                    shell_write(gSmInvalidCommandCode_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicsIndication_Error_gSmInvalidCommandLength_c:
                    shell_write(gSmInvalidCommandLength_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicsIndication_Error_gSmInvalidCommandParameter_c:
                    shell_write(gSmInvalidCommandParameter_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicsIndication_Error_gSmInvalidDeviceId_c:
                    shell_write(gSmInvalidDeviceId_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicsIndication_Error_gSmInvalidInternalOperation_c:
                    shell_write(gSmInvalidInternalOperation_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicsIndication_Error_gSmInvalidConnectionHandle_c:
                    shell_write(gSmInvalidConnectionHandle_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicsIndication_Error_gSmInproperKeyDistributionField_c:
                    shell_write(gSmInproperKeyDistributionField_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicsIndication_Error_gSmUnexpectedKeyType_c:
                    shell_write(gSmUnexpectedKeyType_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicsIndication_Error_gSmUnexpectedPairingTerminationReason_c:
                    shell_write(gSmUnexpectedPairingTerminationReason_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicsIndication_Error_gSmUnexpectedKeyset_c:
                    shell_write(gSmUnexpectedKeyset_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicsIndication_Error_gSmSmpTimeoutOccurred_c:
                    shell_write(gSmSmpTimeoutOccurred_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicsIndication_Error_gSmUnknownSmpPacketType_c:
                    shell_write(gSmUnknownSmpPacketType_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicsIndication_Error_gSmInvalidSmpPacketLength_c:
                    shell_write(gSmInvalidSmpPacketLength_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicsIndication_Error_gSmInvalidSmpPacketParameter_c:
                    shell_write(gSmInvalidSmpPacketParameter_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicsIndication_Error_gSmReceivedUnexpectedSmpPacket_c:
                    shell_write(gSmReceivedUnexpectedSmpPacket_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicsIndication_Error_gSmReceivedSmpPacketFromUnknownDevice_c:
                    shell_write(gSmReceivedSmpPacketFromUnknownDevice_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicsIndication_Error_gSmReceivedUnexpectedHciEvent_c:
                    shell_write(gSmReceivedUnexpectedHciEvent_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicsIndication_Error_gSmReceivedHciEventFromUnknownDevice_c:
                    shell_write(gSmReceivedHciEventFromUnknownDevice_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicsIndication_Error_gSmInvalidHciEventParameter_c:
                    shell_write(gSmInvalidHciEventParameter_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicsIndication_Error_gSmLlConnectionEncryptionInProgress_c:
                    shell_write(gSmLlConnectionEncryptionInProgress_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicsIndication_Error_gSmLlConnectionEncryptionFailure_c:
                    shell_write(gSmLlConnectionEncryptionFailure_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicsIndication_Error_gSmInsufficientResources_c:
                    shell_write(gSmInsufficientResources_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicsIndication_Error_gSmOobDataAddressMismatch_c:
                    shell_write(gSmOobDataAddressMismatch_c);
                    break;

//                case GATTClientProcedureDiscoverAllCharacteristicsIndication_Error_gSmSmpPacketReceivedAfterTimeoutOccurred_c:
//                    shell_write(gSmSmpPacketReceivedAfterTimeoutOccurred_c);
//                    break;

                case GATTClientProcedureDiscoverAllCharacteristicsIndication_Error_gSmPairingErrorPasskeyEntryFailed_c:
                    shell_write(gSmPairingErrorPasskeyEntryFailed_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicsIndication_Error_gSmPairingErrorConfirmValueFailed_c:
                    shell_write(gSmPairingErrorConfirmValueFailed_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicsIndication_Error_gSmPairingErrorCommandNotSupported_c:
                    shell_write(gSmPairingErrorCommandNotSupported_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicsIndication_Error_gSmPairingErrorInvalidParameters_c:
                    shell_write(gSmPairingErrorInvalidParameters_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicsIndication_Error_gSmPairingErrorUnknownReason_c:
                    shell_write(gSmPairingErrorUnknownReason_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicsIndication_Error_gSmTbResolvableAddressDoesNotMatchIrk_c:
                    shell_write(gSmTbResolvableAddressDoesNotMatchIrk_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicsIndication_Error_gSmTbInvalidDataSignature_c:
                    shell_write(gSmTbInvalidDataSignature_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicsIndication_Error_gAttInvalidHandle_c:
                    shell_write(gAttInvalidHandle_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicsIndication_Error_gAttReadNotPermitted_c:
                    shell_write(gAttReadNotPermitted_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicsIndication_Error_gAttWriteNotPermitted_c:
                    shell_write(gAttWriteNotPermitted_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicsIndication_Error_gAttInvalidPdu_c:
                    shell_write(gAttInvalidPdu_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicsIndication_Error_gAttInsufficientAuthentication_c:
                    shell_write(gAttInsufficientAuthentication_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicsIndication_Error_gAttRequestNotSupported_c:
                    shell_write(gAttRequestNotSupported_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicsIndication_Error_gAttInvalidOffset_c:
                    shell_write(gAttInvalidOffset_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicsIndication_Error_gAttInsufficientAuthorization_c:
                    shell_write(gAttInsufficientAuthorization_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicsIndication_Error_gAttPrepareQueueFull_c:
                    shell_write(gAttPrepareQueueFull_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicsIndication_Error_gAttAttributeNotFound_c:
                    shell_write(gAttAttributeNotFound_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicsIndication_Error_gAttAttributeNotLong_c:
                    shell_write(gAttAttributeNotLong_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicsIndication_Error_gAttInsufficientEncryptionKeySize_c:
                    shell_write(gAttInsufficientEncryptionKeySize_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicsIndication_Error_gAttInvalidAttributeValueLength_c:
                    shell_write(gAttInvalidAttributeValueLength_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicsIndication_Error_gAttUnlikelyor_c:
                    shell_write(gAttUnlikelyor_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicsIndication_Error_gAttInsufficientEncryption_c:
                    shell_write(gAttInsufficientEncryption_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicsIndication_Error_gAttUnsupportedGroupType_c:
                    shell_write(gAttUnsupportedGroupType_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicsIndication_Error_gAttInsufficientResources_c:
                    shell_write(gAttInsufficientResources_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicsIndication_Error_gGattAnotherProcedureInProgress_c:
                    shell_write(gGattAnotherProcedureInProgress_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicsIndication_Error_gGattLongAttributePacketsCorrupted_c:
                    shell_write(gGattLongAttributePacketsCorrupted_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicsIndication_Error_gGattMultipleAttributesOverflow_c:
                    shell_write(gGattMultipleAttributesOverflow_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicsIndication_Error_gGattUnexpectedReadMultipleResponseLength_c:
                    shell_write(gGattUnexpectedReadMultipleResponseLength_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicsIndication_Error_gGattInvalidValueLength_c:
                    shell_write(gGattInvalidValueLength_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicsIndication_Error_gGattServerTimeout_c:
                    shell_write(gGattServerTimeout_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicsIndication_Error_gGattIndicationAlreadyInProgress_c:
                    shell_write(gGattIndicationAlreadyInProgress_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicsIndication_Error_gGattClientConfirmationTimeout_c:
                    shell_write(gGattClientConfirmationTimeout_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicsIndication_Error_gGapAdvDataTooLong_c:
                    shell_write(gGapAdvDataTooLong_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicsIndication_Error_gGapScanRspDataTooLong_c:
                    shell_write(gGapScanRspDataTooLong_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicsIndication_Error_gGapDeviceNotBonded_c:
                    shell_write(gGapDeviceNotBonded_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicsIndication_Error_gDevDbCccdLimitReached_c:
                    shell_write(gDevDbCccdLimitReached_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicsIndication_Error_gDevDbCccdNotFound_c:
                    shell_write(gDevDbCccdNotFound_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicsIndication_Error_gGattDbInvalidHandle_c:
                    shell_write(gGattDbInvalidHandle_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicsIndication_Error_gGattDbCharacteristicNotFound_c:
                    shell_write(gGattDbCharacteristicNotFound_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicsIndication_Error_gGattDbCccdNotFound_c:
                    shell_write(gGattDbCccdNotFound_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicsIndication_Error_gGattDbServiceNotFound_c:
                    shell_write(gGattDbServiceNotFound_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicsIndication_Error_gGattDbDescriptorNotFound_c:
                    shell_write(gGattDbDescriptorNotFound_c);
                    break;

                default:
                    shell_printf("Unrecognized status 0x%04X", container->Data.GATTClientProcedureDiscoverAllCharacteristicsIndication.Error);
                    break;
            }

            break;

        case 0x4487:
            shell_write("\r\nGATTClientProcedureDiscoverCharacteristicByUuidIndication");
            shell_write(" -> ");

            switch (container->Data.GATTClientProcedureDiscoverCharacteristicByUuidIndication.ProcedureResult)
            {
                case GATTClientProcedureDiscoverCharacteristicByUuidIndication_ProcedureResult_gGattProcSuccess_c:
                    shell_write(gGattProcSuccess_c);
                    break;

                case GATTClientProcedureDiscoverCharacteristicByUuidIndication_ProcedureResult_gProcedureError_c:
                    shell_write(gProcedureError_c);
                    break;

                default:
                    shell_printf("Unrecognized status 0x%02X", container->Data.GATTClientProcedureDiscoverCharacteristicByUuidIndication.ProcedureResult);
                    break;
            }

            shell_write(" -> ");

            switch (container->Data.GATTClientProcedureDiscoverCharacteristicByUuidIndication.Error)
            {
                case GATTClientProcedureDiscoverCharacteristicByUuidIndication_Error_gBleSuccess_c:
                    shell_write(gBleSuccess_c);
                    break;

                case GATTClientProcedureDiscoverCharacteristicByUuidIndication_Error_gBleInvalidParameter_c:
                    shell_write(gBleInvalidParameter_c);
                    break;

                case GATTClientProcedureDiscoverCharacteristicByUuidIndication_Error_gBleOverflow_c:
                    shell_write(gBleOverflow_c);
                    break;

                case GATTClientProcedureDiscoverCharacteristicByUuidIndication_Error_gBleUnavailable_c:
                    shell_write(gBleUnavailable_c);
                    break;

                case GATTClientProcedureDiscoverCharacteristicByUuidIndication_Error_gBleFeatureNotSupported_c:
                    shell_write(gBleFeatureNotSupported_c);
                    break;

                case GATTClientProcedureDiscoverCharacteristicByUuidIndication_Error_gBleOutOfMemory_c:
                    shell_write(gBleOutOfMemory_c);
                    break;

                case GATTClientProcedureDiscoverCharacteristicByUuidIndication_Error_gBleAlreadyInitialized_c:
                    shell_write(gBleAlreadyInitialized_c);
                    break;

                case GATTClientProcedureDiscoverCharacteristicByUuidIndication_Error_gBleOsError_c:
                    shell_write(gBleOsc);
                    break;

                case GATTClientProcedureDiscoverCharacteristicByUuidIndication_Error_gBleUnexpectedError_c:
                    shell_write(gBleUnexpectedc);
                    break;

                case GATTClientProcedureDiscoverCharacteristicByUuidIndication_Error_gBleInvalidState_c:
                    shell_write(gBleInvalidState_c);
                    break;

                case GATTClientProcedureDiscoverCharacteristicByUuidIndication_Error_gHciUnknownHciCommand_c:
                    shell_write(gHciUnknownHciCommand_c);
                    break;

                case GATTClientProcedureDiscoverCharacteristicByUuidIndication_Error_gHciUnknownConnectionIdentifier_c:
                    shell_write(gHciUnknownConnectionIdentifier_c);
                    break;

                case GATTClientProcedureDiscoverCharacteristicByUuidIndication_Error_gHciHardwareFailure_c:
                    shell_write(gHciHardwareFailure_c);
                    break;

                case GATTClientProcedureDiscoverCharacteristicByUuidIndication_Error_gHciPageTimeout_c:
                    shell_write(gHciPageTimeout_c);
                    break;

                case GATTClientProcedureDiscoverCharacteristicByUuidIndication_Error_gHciAuthenticationFailure_c:
                    shell_write(gHciAuthenticationFailure_c);
                    break;

                case GATTClientProcedureDiscoverCharacteristicByUuidIndication_Error_gHciPinOrKeyMissing_c:
                    shell_write(gHciPinOrKeyMissing_c);
                    break;

                case GATTClientProcedureDiscoverCharacteristicByUuidIndication_Error_gHciMemoryCapacityExceeded_c:
                    shell_write(gHciMemoryCapacityExceeded_c);
                    break;

                case GATTClientProcedureDiscoverCharacteristicByUuidIndication_Error_gHciConnectionTimeout_c:
                    shell_write(gHciConnectionTimeout_c);
                    break;

                case GATTClientProcedureDiscoverCharacteristicByUuidIndication_Error_gHciConnectionLimitExceeded_c:
                    shell_write(gHciConnectionLimitExceeded_c);
                    break;

                case GATTClientProcedureDiscoverCharacteristicByUuidIndication_Error_gHciSynchronousConnectionLimitToADeviceExceeded_c:
                    shell_write(gHciSynchronousConnectionLimitToADeviceExceeded_c);
                    break;

                case GATTClientProcedureDiscoverCharacteristicByUuidIndication_Error_gHciAclConnectionAlreadyExists_c:
                    shell_write(gHciAclConnectionAlreadyExists_c);
                    break;

                case GATTClientProcedureDiscoverCharacteristicByUuidIndication_Error_gHciCommandDisallowed_c:
                    shell_write(gHciCommandDisallowed_c);
                    break;

                case GATTClientProcedureDiscoverCharacteristicByUuidIndication_Error_gHciConnectionRejectedDueToLimitedResources_c:
                    shell_write(gHciConnectionRejectedDueToLimitedResources_c);
                    break;

                case GATTClientProcedureDiscoverCharacteristicByUuidIndication_Error_gHciConnectionRejectedDueToSecurityReasons_c:
                    shell_write(gHciConnectionRejectedDueToSecurityReasons_c);
                    break;

                case GATTClientProcedureDiscoverCharacteristicByUuidIndication_Error_gHciConnectionRejectedDueToUnacceptableBdAddr_c:
                    shell_write(gHciConnectionRejectedDueToUnacceptableBdAddr_c);
                    break;

                case GATTClientProcedureDiscoverCharacteristicByUuidIndication_Error_gHciConnectionAcceptTimeoutExceeded_c:
                    shell_write(gHciConnectionAcceptTimeoutExceeded_c);
                    break;

                case GATTClientProcedureDiscoverCharacteristicByUuidIndication_Error_gHciUnsupportedFeatureOrParameterValue_c:
                    shell_write(gHciUnsupportedFeatureOrParameterValue_c);
                    break;

                case GATTClientProcedureDiscoverCharacteristicByUuidIndication_Error_gHciInvalidHciCommandParameters_c:
                    shell_write(gHciInvalidHciCommandParameters_c);
                    break;

                case GATTClientProcedureDiscoverCharacteristicByUuidIndication_Error_gHciRemoteUserTerminatedConnection_c:
                    shell_write(gHciRemoteUserTerminatedConnection_c);
                    break;

                case GATTClientProcedureDiscoverCharacteristicByUuidIndication_Error_gHciRemoteDeviceTerminatedConnectionLowResources_c:
                    shell_write(gHciRemoteDeviceTerminatedConnectionLowResources_c);
                    break;

                case GATTClientProcedureDiscoverCharacteristicByUuidIndication_Error_gHciRemoteDeviceTerminatedConnectionPowerOff_c:
                    shell_write(gHciRemoteDeviceTerminatedConnectionPowerOff_c);
                    break;

                case GATTClientProcedureDiscoverCharacteristicByUuidIndication_Error_gHciConnectionTerminatedByLocalHost_c:
                    shell_write(gHciConnectionTerminatedByLocalHost_c);
                    break;

                case GATTClientProcedureDiscoverCharacteristicByUuidIndication_Error_gHciRepeatedAttempts_c:
                    shell_write(gHciRepeatedAttempts_c);
                    break;

                case GATTClientProcedureDiscoverCharacteristicByUuidIndication_Error_gHciPairingNotAllowed_c:
                    shell_write(gHciPairingNotAllowed_c);
                    break;

                case GATTClientProcedureDiscoverCharacteristicByUuidIndication_Error_gHciUnknownLpmPdu_c:
                    shell_write(gHciUnknownLpmPdu_c);
                    break;

                case GATTClientProcedureDiscoverCharacteristicByUuidIndication_Error_gHciUnsupportedRemoteFeature_c:
                    shell_write(gHciUnsupportedRemoteFeature_c);
                    break;

                case GATTClientProcedureDiscoverCharacteristicByUuidIndication_Error_gHciScoOffsetRejected_c:
                    shell_write(gHciScoOffsetRejected_c);
                    break;

                case GATTClientProcedureDiscoverCharacteristicByUuidIndication_Error_gHciScoIntervalRejected_c:
                    shell_write(gHciScoIntervalRejected_c);
                    break;

                case GATTClientProcedureDiscoverCharacteristicByUuidIndication_Error_gHciScoAirModeRejected_c:
                    shell_write(gHciScoAirModeRejected_c);
                    break;

                case GATTClientProcedureDiscoverCharacteristicByUuidIndication_Error_gHciInvalidLpmParameters_c:
                    shell_write(gHciInvalidLpmParameters_c);
                    break;

                case GATTClientProcedureDiscoverCharacteristicByUuidIndication_Error_gHciUnspecifiedError_c:
                    shell_write(gHciUnspecifiedc);
                    break;

                case GATTClientProcedureDiscoverCharacteristicByUuidIndication_Error_gHciUnsupportedLpmParameterValue_c:
                    shell_write(gHciUnsupportedLpmParameterValue_c);
                    break;

                case GATTClientProcedureDiscoverCharacteristicByUuidIndication_Error_gHciRoleChangeNotAllowed_c:
                    shell_write(gHciRoleChangeNotAllowed_c);
                    break;

                case GATTClientProcedureDiscoverCharacteristicByUuidIndication_Error_gHciLLResponseTimeout_c:
                    shell_write(gHciLLResponseTimeout_c);
                    break;

                case GATTClientProcedureDiscoverCharacteristicByUuidIndication_Error_gHciLmpErrorTransactionCollision_c:
                    shell_write(gHciLmpErrorTransactionCollision_c);
                    break;

                case GATTClientProcedureDiscoverCharacteristicByUuidIndication_Error_gHciLmpPduNotAllowed_c:
                    shell_write(gHciLmpPduNotAllowed_c);
                    break;

                case GATTClientProcedureDiscoverCharacteristicByUuidIndication_Error_gHciEncryptionModeNotAcceptable_c:
                    shell_write(gHciEncryptionModeNotAcceptable_c);
                    break;

                case GATTClientProcedureDiscoverCharacteristicByUuidIndication_Error_gHciLinkKeyCannotBeChanged_c:
                    shell_write(gHciLinkKeyCannotBeChanged_c);
                    break;

                case GATTClientProcedureDiscoverCharacteristicByUuidIndication_Error_gHciRequestedQosNotSupported_c:
                    shell_write(gHciRequestedQosNotSupported_c);
                    break;

                case GATTClientProcedureDiscoverCharacteristicByUuidIndication_Error_gHciInstantPassed_c:
                    shell_write(gHciInstantPassed_c);
                    break;

                case GATTClientProcedureDiscoverCharacteristicByUuidIndication_Error_gHciPairingWithUnitKeyNotSupported_c:
                    shell_write(gHciPairingWithUnitKeyNotSupported_c);
                    break;

                case GATTClientProcedureDiscoverCharacteristicByUuidIndication_Error_gHciDifferentTransactionCollision_c:
                    shell_write(gHciDifferentTransactionCollision_c);
                    break;

                case GATTClientProcedureDiscoverCharacteristicByUuidIndication_Error_gHciReserved_0x2B_c:
                    shell_write(gHciReserved_0x2B_c);
                    break;

                case GATTClientProcedureDiscoverCharacteristicByUuidIndication_Error_gHciQosNotAcceptableParameter_c:
                    shell_write(gHciQosNotAcceptableParameter_c);
                    break;

                case GATTClientProcedureDiscoverCharacteristicByUuidIndication_Error_gHciQosRejected_c:
                    shell_write(gHciQosRejected_c);
                    break;

                case GATTClientProcedureDiscoverCharacteristicByUuidIndication_Error_gHciChannelClassificationNotSupported_c:
                    shell_write(gHciChannelClassificationNotSupported_c);
                    break;

                case GATTClientProcedureDiscoverCharacteristicByUuidIndication_Error_gHciInsufficientSecurity_c:
                    shell_write(gHciInsufficientSecurity_c);
                    break;

                case GATTClientProcedureDiscoverCharacteristicByUuidIndication_Error_gHciParameterOutOfMandatoryRange_c:
                    shell_write(gHciParameterOutOfMandatoryRange_c);
                    break;

                case GATTClientProcedureDiscoverCharacteristicByUuidIndication_Error_gHciReserved_0x31_c:
                    shell_write(gHciReserved_0x31_c);
                    break;

                case GATTClientProcedureDiscoverCharacteristicByUuidIndication_Error_gHciRoleSwitchPending_c:
                    shell_write(gHciRoleSwitchPending_c);
                    break;

                case GATTClientProcedureDiscoverCharacteristicByUuidIndication_Error_gHciReserved_0x33_c:
                    shell_write(gHciReserved_0x33_c);
                    break;

                case GATTClientProcedureDiscoverCharacteristicByUuidIndication_Error_gHciReservedSlotViolation_c:
                    shell_write(gHciReservedSlotViolation_c);
                    break;

                case GATTClientProcedureDiscoverCharacteristicByUuidIndication_Error_gHciRoleSwitchFailed_c:
                    shell_write(gHciRoleSwitchFailed_c);
                    break;

                case GATTClientProcedureDiscoverCharacteristicByUuidIndication_Error_gHciExtendedInquiryResponseTooLarge_c:
                    shell_write(gHciExtendedInquiryResponseTooLarge_c);
                    break;

                case GATTClientProcedureDiscoverCharacteristicByUuidIndication_Error_gHciSecureSimplePairingNotSupportedByHost_c:
                    shell_write(gHciSecureSimplePairingNotSupportedByHost_c);
                    break;

                case GATTClientProcedureDiscoverCharacteristicByUuidIndication_Error_gHciHostBusyPairing_c:
                    shell_write(gHciHostBusyPairing_c);
                    break;

                case GATTClientProcedureDiscoverCharacteristicByUuidIndication_Error_gHciConnectionRejectedDueToNoSuitableChannelFound_c:
                    shell_write(gHciConnectionRejectedDueToNoSuitableChannelFound_c);
                    break;

                case GATTClientProcedureDiscoverCharacteristicByUuidIndication_Error_gHciControllerBusy_c:
                    shell_write(gHciControllerBusy_c);
                    break;

                case GATTClientProcedureDiscoverCharacteristicByUuidIndication_Error_gHciUnacceptableConnectionParameters_c:
                    shell_write(gHciUnacceptableConnectionParameters_c);
                    break;

                case GATTClientProcedureDiscoverCharacteristicByUuidIndication_Error_gHciDirectedAdvertisingTimeout_c:
                    shell_write(gHciDirectedAdvertisingTimeout_c);
                    break;

                case GATTClientProcedureDiscoverCharacteristicByUuidIndication_Error_gHciConnectionTerminatedDueToMicFailure_c:
                    shell_write(gHciConnectionTerminatedDueToMicFailure_c);
                    break;

                case GATTClientProcedureDiscoverCharacteristicByUuidIndication_Error_gHciConnectionFailedToBeEstablished_c:
                    shell_write(gHciConnectionFailedToBeEstablished_c);
                    break;

                case GATTClientProcedureDiscoverCharacteristicByUuidIndication_Error_gHciMacConnectionFailed_c:
                    shell_write(gHciMacConnectionFailed_c);
                    break;

                case GATTClientProcedureDiscoverCharacteristicByUuidIndication_Error_gHciCoarseClockAdjustmentRejected_c:
                    shell_write(gHciCoarseClockAdjustmentRejected_c);
                    break;

                case GATTClientProcedureDiscoverCharacteristicByUuidIndication_Error_gHciAlreadyInit_c:
                    shell_write(gHciAlreadyInit_c);
                    break;

                case GATTClientProcedureDiscoverCharacteristicByUuidIndication_Error_gHciInvalidParameter_c:
                    shell_write(gHciInvalidParameter_c);
                    break;

                case GATTClientProcedureDiscoverCharacteristicByUuidIndication_Error_gHciCallbackNotInstalled_c:
                    shell_write(gHciCallbackNotInstalled_c);
                    break;

                case GATTClientProcedureDiscoverCharacteristicByUuidIndication_Error_gHciCallbackAlreadyInstalled_c:
                    shell_write(gHciCallbackAlreadyInstalled_c);
                    break;

                case GATTClientProcedureDiscoverCharacteristicByUuidIndication_Error_gHciCommandNotSupported_c:
                    shell_write(gHciCommandNotSupported_c);
                    break;

                case GATTClientProcedureDiscoverCharacteristicByUuidIndication_Error_gHciEventNotSupported_c:
                    shell_write(gHciEventNotSupported_c);
                    break;

                case GATTClientProcedureDiscoverCharacteristicByUuidIndication_Error_gHciTransportError_c:
                    shell_write(gHciTransportc);
                    break;

                case GATTClientProcedureDiscoverCharacteristicByUuidIndication_Error_gL2caAlreadyInit_c:
                    shell_write(gL2caAlreadyInit_c);
                    break;

                case GATTClientProcedureDiscoverCharacteristicByUuidIndication_Error_gL2caInsufficientResources_c:
                    shell_write(gL2caInsufficientResources_c);
                    break;

                case GATTClientProcedureDiscoverCharacteristicByUuidIndication_Error_gL2caCallbackNotInstalled_c:
                    shell_write(gL2caCallbackNotInstalled_c);
                    break;

                case GATTClientProcedureDiscoverCharacteristicByUuidIndication_Error_gL2caCallbackAlreadyInstalled_c:
                    shell_write(gL2caCallbackAlreadyInstalled_c);
                    break;

                case GATTClientProcedureDiscoverCharacteristicByUuidIndication_Error_gL2caLePsmInvalid_c:
                    shell_write(gL2caLePsmInvalid_c);
                    break;

                case GATTClientProcedureDiscoverCharacteristicByUuidIndication_Error_gL2caLePsmAlreadyRegistered_c:
                    shell_write(gL2caLePsmAlreadyRegistered_c);
                    break;

                case GATTClientProcedureDiscoverCharacteristicByUuidIndication_Error_gL2caLePsmNotRegistered_c:
                    shell_write(gL2caLePsmNotRegistered_c);
                    break;

                case GATTClientProcedureDiscoverCharacteristicByUuidIndication_Error_gL2caLePsmInsufficientResources_c:
                    shell_write(gL2caLePsmInsufficientResources_c);
                    break;

                case GATTClientProcedureDiscoverCharacteristicByUuidIndication_Error_gL2caChannelInvalid_c:
                    shell_write(gL2caChannelInvalid_c);
                    break;

                case GATTClientProcedureDiscoverCharacteristicByUuidIndication_Error_gL2caChannelClosed_c:
                    shell_write(gL2caChannelClosed_c);
                    break;

                case GATTClientProcedureDiscoverCharacteristicByUuidIndication_Error_gL2caChannelAlreadyConnected_c:
                    shell_write(gL2caChannelAlreadyConnected_c);
                    break;

                case GATTClientProcedureDiscoverCharacteristicByUuidIndication_Error_gL2caConnectionParametersRejected_c:
                    shell_write(gL2caConnectionParametersRejected_c);
                    break;

                case GATTClientProcedureDiscoverCharacteristicByUuidIndication_Error_gL2caChannelBusy_c:
                    shell_write(gL2caChannelBusy_c);
                    break;

                case GATTClientProcedureDiscoverCharacteristicByUuidIndication_Error_gL2caInvalidParameter_c:
                    shell_write(gL2caInvalidParameter_c);
                    break;

                case GATTClientProcedureDiscoverCharacteristicByUuidIndication_Error_gL2caError_c:
                    shell_write(gL2cac);
                    break;

                case GATTClientProcedureDiscoverCharacteristicByUuidIndication_Error_gSmNullCBFunction_c:
                    shell_write(gSmNullCBFunction_c);
                    break;

                case GATTClientProcedureDiscoverCharacteristicByUuidIndication_Error_gSmCommandNotSupported_c:
                    shell_write(gSmCommandNotSupported_c);
                    break;

                case GATTClientProcedureDiscoverCharacteristicByUuidIndication_Error_gSmUnexpectedCommand_c:
                    shell_write(gSmUnexpectedCommand_c);
                    break;

                case GATTClientProcedureDiscoverCharacteristicByUuidIndication_Error_gSmInvalidCommandCode_c:
                    shell_write(gSmInvalidCommandCode_c);
                    break;

                case GATTClientProcedureDiscoverCharacteristicByUuidIndication_Error_gSmInvalidCommandLength_c:
                    shell_write(gSmInvalidCommandLength_c);
                    break;

                case GATTClientProcedureDiscoverCharacteristicByUuidIndication_Error_gSmInvalidCommandParameter_c:
                    shell_write(gSmInvalidCommandParameter_c);
                    break;

                case GATTClientProcedureDiscoverCharacteristicByUuidIndication_Error_gSmInvalidDeviceId_c:
                    shell_write(gSmInvalidDeviceId_c);
                    break;

                case GATTClientProcedureDiscoverCharacteristicByUuidIndication_Error_gSmInvalidInternalOperation_c:
                    shell_write(gSmInvalidInternalOperation_c);
                    break;

                case GATTClientProcedureDiscoverCharacteristicByUuidIndication_Error_gSmInvalidConnectionHandle_c:
                    shell_write(gSmInvalidConnectionHandle_c);
                    break;

                case GATTClientProcedureDiscoverCharacteristicByUuidIndication_Error_gSmInproperKeyDistributionField_c:
                    shell_write(gSmInproperKeyDistributionField_c);
                    break;

                case GATTClientProcedureDiscoverCharacteristicByUuidIndication_Error_gSmUnexpectedKeyType_c:
                    shell_write(gSmUnexpectedKeyType_c);
                    break;

                case GATTClientProcedureDiscoverCharacteristicByUuidIndication_Error_gSmUnexpectedPairingTerminationReason_c:
                    shell_write(gSmUnexpectedPairingTerminationReason_c);
                    break;

                case GATTClientProcedureDiscoverCharacteristicByUuidIndication_Error_gSmUnexpectedKeyset_c:
                    shell_write(gSmUnexpectedKeyset_c);
                    break;

                case GATTClientProcedureDiscoverCharacteristicByUuidIndication_Error_gSmSmpTimeoutOccurred_c:
                    shell_write(gSmSmpTimeoutOccurred_c);
                    break;

                case GATTClientProcedureDiscoverCharacteristicByUuidIndication_Error_gSmUnknownSmpPacketType_c:
                    shell_write(gSmUnknownSmpPacketType_c);
                    break;

                case GATTClientProcedureDiscoverCharacteristicByUuidIndication_Error_gSmInvalidSmpPacketLength_c:
                    shell_write(gSmInvalidSmpPacketLength_c);
                    break;

                case GATTClientProcedureDiscoverCharacteristicByUuidIndication_Error_gSmInvalidSmpPacketParameter_c:
                    shell_write(gSmInvalidSmpPacketParameter_c);
                    break;

                case GATTClientProcedureDiscoverCharacteristicByUuidIndication_Error_gSmReceivedUnexpectedSmpPacket_c:
                    shell_write(gSmReceivedUnexpectedSmpPacket_c);
                    break;

                case GATTClientProcedureDiscoverCharacteristicByUuidIndication_Error_gSmReceivedSmpPacketFromUnknownDevice_c:
                    shell_write(gSmReceivedSmpPacketFromUnknownDevice_c);
                    break;

                case GATTClientProcedureDiscoverCharacteristicByUuidIndication_Error_gSmReceivedUnexpectedHciEvent_c:
                    shell_write(gSmReceivedUnexpectedHciEvent_c);
                    break;

                case GATTClientProcedureDiscoverCharacteristicByUuidIndication_Error_gSmReceivedHciEventFromUnknownDevice_c:
                    shell_write(gSmReceivedHciEventFromUnknownDevice_c);
                    break;

                case GATTClientProcedureDiscoverCharacteristicByUuidIndication_Error_gSmInvalidHciEventParameter_c:
                    shell_write(gSmInvalidHciEventParameter_c);
                    break;

                case GATTClientProcedureDiscoverCharacteristicByUuidIndication_Error_gSmLlConnectionEncryptionInProgress_c:
                    shell_write(gSmLlConnectionEncryptionInProgress_c);
                    break;

                case GATTClientProcedureDiscoverCharacteristicByUuidIndication_Error_gSmLlConnectionEncryptionFailure_c:
                    shell_write(gSmLlConnectionEncryptionFailure_c);
                    break;

                case GATTClientProcedureDiscoverCharacteristicByUuidIndication_Error_gSmInsufficientResources_c:
                    shell_write(gSmInsufficientResources_c);
                    break;

                case GATTClientProcedureDiscoverCharacteristicByUuidIndication_Error_gSmOobDataAddressMismatch_c:
                    shell_write(gSmOobDataAddressMismatch_c);
                    break;

//                case GATTClientProcedureDiscoverCharacteristicByUuidIndication_Error_gSmSmpPacketReceivedAfterTimeoutOccurred_c:
//                    shell_write(gSmSmpPacketReceivedAfterTimeoutOccurred_c);
//                    break;

                case GATTClientProcedureDiscoverCharacteristicByUuidIndication_Error_gSmPairingErrorPasskeyEntryFailed_c:
                    shell_write(gSmPairingErrorPasskeyEntryFailed_c);
                    break;

                case GATTClientProcedureDiscoverCharacteristicByUuidIndication_Error_gSmPairingErrorConfirmValueFailed_c:
                    shell_write(gSmPairingErrorConfirmValueFailed_c);
                    break;

                case GATTClientProcedureDiscoverCharacteristicByUuidIndication_Error_gSmPairingErrorCommandNotSupported_c:
                    shell_write(gSmPairingErrorCommandNotSupported_c);
                    break;

                case GATTClientProcedureDiscoverCharacteristicByUuidIndication_Error_gSmPairingErrorInvalidParameters_c:
                    shell_write(gSmPairingErrorInvalidParameters_c);
                    break;

                case GATTClientProcedureDiscoverCharacteristicByUuidIndication_Error_gSmPairingErrorUnknownReason_c:
                    shell_write(gSmPairingErrorUnknownReason_c);
                    break;

                case GATTClientProcedureDiscoverCharacteristicByUuidIndication_Error_gSmTbResolvableAddressDoesNotMatchIrk_c:
                    shell_write(gSmTbResolvableAddressDoesNotMatchIrk_c);
                    break;

                case GATTClientProcedureDiscoverCharacteristicByUuidIndication_Error_gSmTbInvalidDataSignature_c:
                    shell_write(gSmTbInvalidDataSignature_c);
                    break;

                case GATTClientProcedureDiscoverCharacteristicByUuidIndication_Error_gAttInvalidHandle_c:
                    shell_write(gAttInvalidHandle_c);
                    break;

                case GATTClientProcedureDiscoverCharacteristicByUuidIndication_Error_gAttReadNotPermitted_c:
                    shell_write(gAttReadNotPermitted_c);
                    break;

                case GATTClientProcedureDiscoverCharacteristicByUuidIndication_Error_gAttWriteNotPermitted_c:
                    shell_write(gAttWriteNotPermitted_c);
                    break;

                case GATTClientProcedureDiscoverCharacteristicByUuidIndication_Error_gAttInvalidPdu_c:
                    shell_write(gAttInvalidPdu_c);
                    break;

                case GATTClientProcedureDiscoverCharacteristicByUuidIndication_Error_gAttInsufficientAuthentication_c:
                    shell_write(gAttInsufficientAuthentication_c);
                    break;

                case GATTClientProcedureDiscoverCharacteristicByUuidIndication_Error_gAttRequestNotSupported_c:
                    shell_write(gAttRequestNotSupported_c);
                    break;

                case GATTClientProcedureDiscoverCharacteristicByUuidIndication_Error_gAttInvalidOffset_c:
                    shell_write(gAttInvalidOffset_c);
                    break;

                case GATTClientProcedureDiscoverCharacteristicByUuidIndication_Error_gAttInsufficientAuthorization_c:
                    shell_write(gAttInsufficientAuthorization_c);
                    break;

                case GATTClientProcedureDiscoverCharacteristicByUuidIndication_Error_gAttPrepareQueueFull_c:
                    shell_write(gAttPrepareQueueFull_c);
                    break;

                case GATTClientProcedureDiscoverCharacteristicByUuidIndication_Error_gAttAttributeNotFound_c:
                    shell_write(gAttAttributeNotFound_c);
                    break;

                case GATTClientProcedureDiscoverCharacteristicByUuidIndication_Error_gAttAttributeNotLong_c:
                    shell_write(gAttAttributeNotLong_c);
                    break;

                case GATTClientProcedureDiscoverCharacteristicByUuidIndication_Error_gAttInsufficientEncryptionKeySize_c:
                    shell_write(gAttInsufficientEncryptionKeySize_c);
                    break;

                case GATTClientProcedureDiscoverCharacteristicByUuidIndication_Error_gAttInvalidAttributeValueLength_c:
                    shell_write(gAttInvalidAttributeValueLength_c);
                    break;

                case GATTClientProcedureDiscoverCharacteristicByUuidIndication_Error_gAttUnlikelyor_c:
                    shell_write(gAttUnlikelyor_c);
                    break;

                case GATTClientProcedureDiscoverCharacteristicByUuidIndication_Error_gAttInsufficientEncryption_c:
                    shell_write(gAttInsufficientEncryption_c);
                    break;

                case GATTClientProcedureDiscoverCharacteristicByUuidIndication_Error_gAttUnsupportedGroupType_c:
                    shell_write(gAttUnsupportedGroupType_c);
                    break;

                case GATTClientProcedureDiscoverCharacteristicByUuidIndication_Error_gAttInsufficientResources_c:
                    shell_write(gAttInsufficientResources_c);
                    break;

                case GATTClientProcedureDiscoverCharacteristicByUuidIndication_Error_gGattAnotherProcedureInProgress_c:
                    shell_write(gGattAnotherProcedureInProgress_c);
                    break;

                case GATTClientProcedureDiscoverCharacteristicByUuidIndication_Error_gGattLongAttributePacketsCorrupted_c:
                    shell_write(gGattLongAttributePacketsCorrupted_c);
                    break;

                case GATTClientProcedureDiscoverCharacteristicByUuidIndication_Error_gGattMultipleAttributesOverflow_c:
                    shell_write(gGattMultipleAttributesOverflow_c);
                    break;

                case GATTClientProcedureDiscoverCharacteristicByUuidIndication_Error_gGattUnexpectedReadMultipleResponseLength_c:
                    shell_write(gGattUnexpectedReadMultipleResponseLength_c);
                    break;

                case GATTClientProcedureDiscoverCharacteristicByUuidIndication_Error_gGattInvalidValueLength_c:
                    shell_write(gGattInvalidValueLength_c);
                    break;

                case GATTClientProcedureDiscoverCharacteristicByUuidIndication_Error_gGattServerTimeout_c:
                    shell_write(gGattServerTimeout_c);
                    break;

                case GATTClientProcedureDiscoverCharacteristicByUuidIndication_Error_gGattIndicationAlreadyInProgress_c:
                    shell_write(gGattIndicationAlreadyInProgress_c);
                    break;

                case GATTClientProcedureDiscoverCharacteristicByUuidIndication_Error_gGattClientConfirmationTimeout_c:
                    shell_write(gGattClientConfirmationTimeout_c);
                    break;

                case GATTClientProcedureDiscoverCharacteristicByUuidIndication_Error_gGapAdvDataTooLong_c:
                    shell_write(gGapAdvDataTooLong_c);
                    break;

                case GATTClientProcedureDiscoverCharacteristicByUuidIndication_Error_gGapScanRspDataTooLong_c:
                    shell_write(gGapScanRspDataTooLong_c);
                    break;

                case GATTClientProcedureDiscoverCharacteristicByUuidIndication_Error_gGapDeviceNotBonded_c:
                    shell_write(gGapDeviceNotBonded_c);
                    break;

                case GATTClientProcedureDiscoverCharacteristicByUuidIndication_Error_gDevDbCccdLimitReached_c:
                    shell_write(gDevDbCccdLimitReached_c);
                    break;

                case GATTClientProcedureDiscoverCharacteristicByUuidIndication_Error_gDevDbCccdNotFound_c:
                    shell_write(gDevDbCccdNotFound_c);
                    break;

                case GATTClientProcedureDiscoverCharacteristicByUuidIndication_Error_gGattDbInvalidHandle_c:
                    shell_write(gGattDbInvalidHandle_c);
                    break;

                case GATTClientProcedureDiscoverCharacteristicByUuidIndication_Error_gGattDbCharacteristicNotFound_c:
                    shell_write(gGattDbCharacteristicNotFound_c);
                    break;

                case GATTClientProcedureDiscoverCharacteristicByUuidIndication_Error_gGattDbCccdNotFound_c:
                    shell_write(gGattDbCccdNotFound_c);
                    break;

                case GATTClientProcedureDiscoverCharacteristicByUuidIndication_Error_gGattDbServiceNotFound_c:
                    shell_write(gGattDbServiceNotFound_c);
                    break;

                case GATTClientProcedureDiscoverCharacteristicByUuidIndication_Error_gGattDbDescriptorNotFound_c:
                    shell_write(gGattDbDescriptorNotFound_c);
                    break;

                default:
                    shell_printf("Unrecognized status 0x%04X", container->Data.GATTClientProcedureDiscoverCharacteristicByUuidIndication.Error);
                    break;
            }

            break;

        case 0x4488:
            shell_write("\r\nGATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication");
            shell_write(" -> ");

            switch (container->Data.GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication.ProcedureResult)
            {
                case GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication_ProcedureResult_gGattProcSuccess_c:
                    shell_write(gGattProcSuccess_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication_ProcedureResult_gProcedureError_c:
                    shell_write(gProcedureError_c);
                    break;

                default:
                    shell_printf("Unrecognized status 0x%02X", container->Data.GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication.ProcedureResult);
                    break;
            }

            shell_write(" -> ");

            switch (container->Data.GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication.Error)
            {
                case GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication_Error_gBleSuccess_c:
                    shell_write(gBleSuccess_c);
                    SHELL_PrintDiscoveredDescriptors(&(container->Data.GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication));
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication_Error_gBleInvalidParameter_c:
                    shell_write(gBleInvalidParameter_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication_Error_gBleOverflow_c:
                    shell_write(gBleOverflow_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication_Error_gBleUnavailable_c:
                    shell_write(gBleUnavailable_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication_Error_gBleFeatureNotSupported_c:
                    shell_write(gBleFeatureNotSupported_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication_Error_gBleOutOfMemory_c:
                    shell_write(gBleOutOfMemory_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication_Error_gBleAlreadyInitialized_c:
                    shell_write(gBleAlreadyInitialized_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication_Error_gBleOsError_c:
                    shell_write(gBleOsc);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication_Error_gBleUnexpectedError_c:
                    shell_write(gBleUnexpectedc);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication_Error_gBleInvalidState_c:
                    shell_write(gBleInvalidState_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication_Error_gHciUnknownHciCommand_c:
                    shell_write(gHciUnknownHciCommand_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication_Error_gHciUnknownConnectionIdentifier_c:
                    shell_write(gHciUnknownConnectionIdentifier_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication_Error_gHciHardwareFailure_c:
                    shell_write(gHciHardwareFailure_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication_Error_gHciPageTimeout_c:
                    shell_write(gHciPageTimeout_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication_Error_gHciAuthenticationFailure_c:
                    shell_write(gHciAuthenticationFailure_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication_Error_gHciPinOrKeyMissing_c:
                    shell_write(gHciPinOrKeyMissing_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication_Error_gHciMemoryCapacityExceeded_c:
                    shell_write(gHciMemoryCapacityExceeded_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication_Error_gHciConnectionTimeout_c:
                    shell_write(gHciConnectionTimeout_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication_Error_gHciConnectionLimitExceeded_c:
                    shell_write(gHciConnectionLimitExceeded_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication_Error_gHciSynchronousConnectionLimitToADeviceExceeded_c:
                    shell_write(gHciSynchronousConnectionLimitToADeviceExceeded_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication_Error_gHciAclConnectionAlreadyExists_c:
                    shell_write(gHciAclConnectionAlreadyExists_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication_Error_gHciCommandDisallowed_c:
                    shell_write(gHciCommandDisallowed_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication_Error_gHciConnectionRejectedDueToLimitedResources_c:
                    shell_write(gHciConnectionRejectedDueToLimitedResources_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication_Error_gHciConnectionRejectedDueToSecurityReasons_c:
                    shell_write(gHciConnectionRejectedDueToSecurityReasons_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication_Error_gHciConnectionRejectedDueToUnacceptableBdAddr_c:
                    shell_write(gHciConnectionRejectedDueToUnacceptableBdAddr_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication_Error_gHciConnectionAcceptTimeoutExceeded_c:
                    shell_write(gHciConnectionAcceptTimeoutExceeded_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication_Error_gHciUnsupportedFeatureOrParameterValue_c:
                    shell_write(gHciUnsupportedFeatureOrParameterValue_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication_Error_gHciInvalidHciCommandParameters_c:
                    shell_write(gHciInvalidHciCommandParameters_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication_Error_gHciRemoteUserTerminatedConnection_c:
                    shell_write(gHciRemoteUserTerminatedConnection_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication_Error_gHciRemoteDeviceTerminatedConnectionLowResources_c:
                    shell_write(gHciRemoteDeviceTerminatedConnectionLowResources_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication_Error_gHciRemoteDeviceTerminatedConnectionPowerOff_c:
                    shell_write(gHciRemoteDeviceTerminatedConnectionPowerOff_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication_Error_gHciConnectionTerminatedByLocalHost_c:
                    shell_write(gHciConnectionTerminatedByLocalHost_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication_Error_gHciRepeatedAttempts_c:
                    shell_write(gHciRepeatedAttempts_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication_Error_gHciPairingNotAllowed_c:
                    shell_write(gHciPairingNotAllowed_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication_Error_gHciUnknownLpmPdu_c:
                    shell_write(gHciUnknownLpmPdu_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication_Error_gHciUnsupportedRemoteFeature_c:
                    shell_write(gHciUnsupportedRemoteFeature_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication_Error_gHciScoOffsetRejected_c:
                    shell_write(gHciScoOffsetRejected_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication_Error_gHciScoIntervalRejected_c:
                    shell_write(gHciScoIntervalRejected_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication_Error_gHciScoAirModeRejected_c:
                    shell_write(gHciScoAirModeRejected_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication_Error_gHciInvalidLpmParameters_c:
                    shell_write(gHciInvalidLpmParameters_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication_Error_gHciUnspecifiedError_c:
                    shell_write(gHciUnspecifiedc);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication_Error_gHciUnsupportedLpmParameterValue_c:
                    shell_write(gHciUnsupportedLpmParameterValue_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication_Error_gHciRoleChangeNotAllowed_c:
                    shell_write(gHciRoleChangeNotAllowed_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication_Error_gHciLLResponseTimeout_c:
                    shell_write(gHciLLResponseTimeout_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication_Error_gHciLmpErrorTransactionCollision_c:
                    shell_write(gHciLmpErrorTransactionCollision_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication_Error_gHciLmpPduNotAllowed_c:
                    shell_write(gHciLmpPduNotAllowed_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication_Error_gHciEncryptionModeNotAcceptable_c:
                    shell_write(gHciEncryptionModeNotAcceptable_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication_Error_gHciLinkKeyCannotBeChanged_c:
                    shell_write(gHciLinkKeyCannotBeChanged_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication_Error_gHciRequestedQosNotSupported_c:
                    shell_write(gHciRequestedQosNotSupported_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication_Error_gHciInstantPassed_c:
                    shell_write(gHciInstantPassed_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication_Error_gHciPairingWithUnitKeyNotSupported_c:
                    shell_write(gHciPairingWithUnitKeyNotSupported_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication_Error_gHciDifferentTransactionCollision_c:
                    shell_write(gHciDifferentTransactionCollision_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication_Error_gHciReserved_0x2B_c:
                    shell_write(gHciReserved_0x2B_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication_Error_gHciQosNotAcceptableParameter_c:
                    shell_write(gHciQosNotAcceptableParameter_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication_Error_gHciQosRejected_c:
                    shell_write(gHciQosRejected_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication_Error_gHciChannelClassificationNotSupported_c:
                    shell_write(gHciChannelClassificationNotSupported_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication_Error_gHciInsufficientSecurity_c:
                    shell_write(gHciInsufficientSecurity_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication_Error_gHciParameterOutOfMandatoryRange_c:
                    shell_write(gHciParameterOutOfMandatoryRange_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication_Error_gHciReserved_0x31_c:
                    shell_write(gHciReserved_0x31_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication_Error_gHciRoleSwitchPending_c:
                    shell_write(gHciRoleSwitchPending_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication_Error_gHciReserved_0x33_c:
                    shell_write(gHciReserved_0x33_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication_Error_gHciReservedSlotViolation_c:
                    shell_write(gHciReservedSlotViolation_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication_Error_gHciRoleSwitchFailed_c:
                    shell_write(gHciRoleSwitchFailed_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication_Error_gHciExtendedInquiryResponseTooLarge_c:
                    shell_write(gHciExtendedInquiryResponseTooLarge_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication_Error_gHciSecureSimplePairingNotSupportedByHost_c:
                    shell_write(gHciSecureSimplePairingNotSupportedByHost_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication_Error_gHciHostBusyPairing_c:
                    shell_write(gHciHostBusyPairing_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication_Error_gHciConnectionRejectedDueToNoSuitableChannelFound_c:
                    shell_write(gHciConnectionRejectedDueToNoSuitableChannelFound_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication_Error_gHciControllerBusy_c:
                    shell_write(gHciControllerBusy_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication_Error_gHciUnacceptableConnectionParameters_c:
                    shell_write(gHciUnacceptableConnectionParameters_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication_Error_gHciDirectedAdvertisingTimeout_c:
                    shell_write(gHciDirectedAdvertisingTimeout_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication_Error_gHciConnectionTerminatedDueToMicFailure_c:
                    shell_write(gHciConnectionTerminatedDueToMicFailure_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication_Error_gHciConnectionFailedToBeEstablished_c:
                    shell_write(gHciConnectionFailedToBeEstablished_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication_Error_gHciMacConnectionFailed_c:
                    shell_write(gHciMacConnectionFailed_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication_Error_gHciCoarseClockAdjustmentRejected_c:
                    shell_write(gHciCoarseClockAdjustmentRejected_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication_Error_gHciAlreadyInit_c:
                    shell_write(gHciAlreadyInit_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication_Error_gHciInvalidParameter_c:
                    shell_write(gHciInvalidParameter_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication_Error_gHciCallbackNotInstalled_c:
                    shell_write(gHciCallbackNotInstalled_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication_Error_gHciCallbackAlreadyInstalled_c:
                    shell_write(gHciCallbackAlreadyInstalled_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication_Error_gHciCommandNotSupported_c:
                    shell_write(gHciCommandNotSupported_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication_Error_gHciEventNotSupported_c:
                    shell_write(gHciEventNotSupported_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication_Error_gHciTransportError_c:
                    shell_write(gHciTransportc);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication_Error_gL2caAlreadyInit_c:
                    shell_write(gL2caAlreadyInit_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication_Error_gL2caInsufficientResources_c:
                    shell_write(gL2caInsufficientResources_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication_Error_gL2caCallbackNotInstalled_c:
                    shell_write(gL2caCallbackNotInstalled_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication_Error_gL2caCallbackAlreadyInstalled_c:
                    shell_write(gL2caCallbackAlreadyInstalled_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication_Error_gL2caLePsmInvalid_c:
                    shell_write(gL2caLePsmInvalid_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication_Error_gL2caLePsmAlreadyRegistered_c:
                    shell_write(gL2caLePsmAlreadyRegistered_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication_Error_gL2caLePsmNotRegistered_c:
                    shell_write(gL2caLePsmNotRegistered_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication_Error_gL2caLePsmInsufficientResources_c:
                    shell_write(gL2caLePsmInsufficientResources_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication_Error_gL2caChannelInvalid_c:
                    shell_write(gL2caChannelInvalid_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication_Error_gL2caChannelClosed_c:
                    shell_write(gL2caChannelClosed_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication_Error_gL2caChannelAlreadyConnected_c:
                    shell_write(gL2caChannelAlreadyConnected_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication_Error_gL2caConnectionParametersRejected_c:
                    shell_write(gL2caConnectionParametersRejected_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication_Error_gL2caChannelBusy_c:
                    shell_write(gL2caChannelBusy_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication_Error_gL2caInvalidParameter_c:
                    shell_write(gL2caInvalidParameter_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication_Error_gL2caError_c:
                    shell_write(gL2cac);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication_Error_gSmNullCBFunction_c:
                    shell_write(gSmNullCBFunction_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication_Error_gSmCommandNotSupported_c:
                    shell_write(gSmCommandNotSupported_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication_Error_gSmUnexpectedCommand_c:
                    shell_write(gSmUnexpectedCommand_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication_Error_gSmInvalidCommandCode_c:
                    shell_write(gSmInvalidCommandCode_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication_Error_gSmInvalidCommandLength_c:
                    shell_write(gSmInvalidCommandLength_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication_Error_gSmInvalidCommandParameter_c:
                    shell_write(gSmInvalidCommandParameter_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication_Error_gSmInvalidDeviceId_c:
                    shell_write(gSmInvalidDeviceId_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication_Error_gSmInvalidInternalOperation_c:
                    shell_write(gSmInvalidInternalOperation_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication_Error_gSmInvalidConnectionHandle_c:
                    shell_write(gSmInvalidConnectionHandle_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication_Error_gSmInproperKeyDistributionField_c:
                    shell_write(gSmInproperKeyDistributionField_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication_Error_gSmUnexpectedKeyType_c:
                    shell_write(gSmUnexpectedKeyType_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication_Error_gSmUnexpectedPairingTerminationReason_c:
                    shell_write(gSmUnexpectedPairingTerminationReason_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication_Error_gSmUnexpectedKeyset_c:
                    shell_write(gSmUnexpectedKeyset_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication_Error_gSmSmpTimeoutOccurred_c:
                    shell_write(gSmSmpTimeoutOccurred_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication_Error_gSmUnknownSmpPacketType_c:
                    shell_write(gSmUnknownSmpPacketType_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication_Error_gSmInvalidSmpPacketLength_c:
                    shell_write(gSmInvalidSmpPacketLength_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication_Error_gSmInvalidSmpPacketParameter_c:
                    shell_write(gSmInvalidSmpPacketParameter_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication_Error_gSmReceivedUnexpectedSmpPacket_c:
                    shell_write(gSmReceivedUnexpectedSmpPacket_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication_Error_gSmReceivedSmpPacketFromUnknownDevice_c:
                    shell_write(gSmReceivedSmpPacketFromUnknownDevice_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication_Error_gSmReceivedUnexpectedHciEvent_c:
                    shell_write(gSmReceivedUnexpectedHciEvent_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication_Error_gSmReceivedHciEventFromUnknownDevice_c:
                    shell_write(gSmReceivedHciEventFromUnknownDevice_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication_Error_gSmInvalidHciEventParameter_c:
                    shell_write(gSmInvalidHciEventParameter_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication_Error_gSmLlConnectionEncryptionInProgress_c:
                    shell_write(gSmLlConnectionEncryptionInProgress_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication_Error_gSmLlConnectionEncryptionFailure_c:
                    shell_write(gSmLlConnectionEncryptionFailure_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication_Error_gSmInsufficientResources_c:
                    shell_write(gSmInsufficientResources_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication_Error_gSmOobDataAddressMismatch_c:
                    shell_write(gSmOobDataAddressMismatch_c);
                    break;

//                case GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication_Error_gSmSmpPacketReceivedAfterTimeoutOccurred_c:
//                    shell_write(gSmSmpPacketReceivedAfterTimeoutOccurred_c);
//                    break;

                case GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication_Error_gSmPairingErrorPasskeyEntryFailed_c:
                    shell_write(gSmPairingErrorPasskeyEntryFailed_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication_Error_gSmPairingErrorConfirmValueFailed_c:
                    shell_write(gSmPairingErrorConfirmValueFailed_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication_Error_gSmPairingErrorCommandNotSupported_c:
                    shell_write(gSmPairingErrorCommandNotSupported_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication_Error_gSmPairingErrorInvalidParameters_c:
                    shell_write(gSmPairingErrorInvalidParameters_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication_Error_gSmPairingErrorUnknownReason_c:
                    shell_write(gSmPairingErrorUnknownReason_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication_Error_gSmTbResolvableAddressDoesNotMatchIrk_c:
                    shell_write(gSmTbResolvableAddressDoesNotMatchIrk_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication_Error_gSmTbInvalidDataSignature_c:
                    shell_write(gSmTbInvalidDataSignature_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication_Error_gAttInvalidHandle_c:
                    shell_write(gAttInvalidHandle_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication_Error_gAttReadNotPermitted_c:
                    shell_write(gAttReadNotPermitted_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication_Error_gAttWriteNotPermitted_c:
                    shell_write(gAttWriteNotPermitted_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication_Error_gAttInvalidPdu_c:
                    shell_write(gAttInvalidPdu_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication_Error_gAttInsufficientAuthentication_c:
                    shell_write(gAttInsufficientAuthentication_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication_Error_gAttRequestNotSupported_c:
                    shell_write(gAttRequestNotSupported_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication_Error_gAttInvalidOffset_c:
                    shell_write(gAttInvalidOffset_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication_Error_gAttInsufficientAuthorization_c:
                    shell_write(gAttInsufficientAuthorization_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication_Error_gAttPrepareQueueFull_c:
                    shell_write(gAttPrepareQueueFull_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication_Error_gAttAttributeNotFound_c:
                    shell_write(gAttAttributeNotFound_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication_Error_gAttAttributeNotLong_c:
                    shell_write(gAttAttributeNotLong_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication_Error_gAttInsufficientEncryptionKeySize_c:
                    shell_write(gAttInsufficientEncryptionKeySize_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication_Error_gAttInvalidAttributeValueLength_c:
                    shell_write(gAttInvalidAttributeValueLength_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication_Error_gAttUnlikelyor_c:
                    shell_write(gAttUnlikelyor_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication_Error_gAttInsufficientEncryption_c:
                    shell_write(gAttInsufficientEncryption_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication_Error_gAttUnsupportedGroupType_c:
                    shell_write(gAttUnsupportedGroupType_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication_Error_gAttInsufficientResources_c:
                    shell_write(gAttInsufficientResources_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication_Error_gGattAnotherProcedureInProgress_c:
                    shell_write(gGattAnotherProcedureInProgress_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication_Error_gGattLongAttributePacketsCorrupted_c:
                    shell_write(gGattLongAttributePacketsCorrupted_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication_Error_gGattMultipleAttributesOverflow_c:
                    shell_write(gGattMultipleAttributesOverflow_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication_Error_gGattUnexpectedReadMultipleResponseLength_c:
                    shell_write(gGattUnexpectedReadMultipleResponseLength_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication_Error_gGattInvalidValueLength_c:
                    shell_write(gGattInvalidValueLength_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication_Error_gGattServerTimeout_c:
                    shell_write(gGattServerTimeout_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication_Error_gGattIndicationAlreadyInProgress_c:
                    shell_write(gGattIndicationAlreadyInProgress_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication_Error_gGattClientConfirmationTimeout_c:
                    shell_write(gGattClientConfirmationTimeout_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication_Error_gGapAdvDataTooLong_c:
                    shell_write(gGapAdvDataTooLong_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication_Error_gGapScanRspDataTooLong_c:
                    shell_write(gGapScanRspDataTooLong_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication_Error_gGapDeviceNotBonded_c:
                    shell_write(gGapDeviceNotBonded_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication_Error_gDevDbCccdLimitReached_c:
                    shell_write(gDevDbCccdLimitReached_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication_Error_gDevDbCccdNotFound_c:
                    shell_write(gDevDbCccdNotFound_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication_Error_gGattDbInvalidHandle_c:
                    shell_write(gGattDbInvalidHandle_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication_Error_gGattDbCharacteristicNotFound_c:
                    shell_write(gGattDbCharacteristicNotFound_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication_Error_gGattDbCccdNotFound_c:
                    shell_write(gGattDbCccdNotFound_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication_Error_gGattDbServiceNotFound_c:
                    shell_write(gGattDbServiceNotFound_c);
                    break;

                case GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication_Error_gGattDbDescriptorNotFound_c:
                    shell_write(gGattDbDescriptorNotFound_c);
                    break;

                default:
                    shell_printf("Unrecognized status 0x%04X", container->Data.GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication.Error);
                    break;
            }

            break;

        case 0x4489:
            shell_write("\r\nGATTClientProcedureReadCharacteristicValueIndication");
            shell_write(" -> ");

            switch (container->Data.GATTClientProcedureReadCharacteristicValueIndication.ProcedureResult)
            {
                case GATTClientProcedureReadCharacteristicValueIndication_ProcedureResult_gGattProcSuccess_c:
                    shell_write(gGattProcSuccess_c);
                    break;

                case GATTClientProcedureReadCharacteristicValueIndication_ProcedureResult_gProcedureError_c:
                    shell_write(gProcedureError_c);
                    break;

                default:
                    shell_printf("Unrecognized status 0x%02X", container->Data.GATTClientProcedureReadCharacteristicValueIndication.ProcedureResult);
                    break;
            }

            shell_write(" -> ");

            switch (container->Data.GATTClientProcedureReadCharacteristicValueIndication.Error)
            {
                case GATTClientProcedureReadCharacteristicValueIndication_Error_gBleSuccess_c:
                    shell_write(gBleSuccess_c);
                    shell_write(" -> ");
                    SHELL_PrintBuff(
                        container->Data.GATTClientProcedureReadCharacteristicValueIndication.Characteristic.Value.Value,
                        container->Data.GATTClientProcedureReadCharacteristicValueIndication.Characteristic.Value.ValueLength
                    );
                    MEM_BufferFree(container->Data.GATTClientProcedureReadCharacteristicValueIndication.Characteristic.Value.Value);
                    break;

                case GATTClientProcedureReadCharacteristicValueIndication_Error_gBleInvalidParameter_c:
                    shell_write(gBleInvalidParameter_c);
                    break;

                case GATTClientProcedureReadCharacteristicValueIndication_Error_gBleOverflow_c:
                    shell_write(gBleOverflow_c);
                    break;

                case GATTClientProcedureReadCharacteristicValueIndication_Error_gBleUnavailable_c:
                    shell_write(gBleUnavailable_c);
                    break;

                case GATTClientProcedureReadCharacteristicValueIndication_Error_gBleFeatureNotSupported_c:
                    shell_write(gBleFeatureNotSupported_c);
                    break;

                case GATTClientProcedureReadCharacteristicValueIndication_Error_gBleOutOfMemory_c:
                    shell_write(gBleOutOfMemory_c);
                    break;

                case GATTClientProcedureReadCharacteristicValueIndication_Error_gBleAlreadyInitialized_c:
                    shell_write(gBleAlreadyInitialized_c);
                    break;

                case GATTClientProcedureReadCharacteristicValueIndication_Error_gBleOsError_c:
                    shell_write(gBleOsc);
                    break;

                case GATTClientProcedureReadCharacteristicValueIndication_Error_gBleUnexpectedError_c:
                    shell_write(gBleUnexpectedc);
                    break;

                case GATTClientProcedureReadCharacteristicValueIndication_Error_gBleInvalidState_c:
                    shell_write(gBleInvalidState_c);
                    break;

                case GATTClientProcedureReadCharacteristicValueIndication_Error_gHciUnknownHciCommand_c:
                    shell_write(gHciUnknownHciCommand_c);
                    break;

                case GATTClientProcedureReadCharacteristicValueIndication_Error_gHciUnknownConnectionIdentifier_c:
                    shell_write(gHciUnknownConnectionIdentifier_c);
                    break;

                case GATTClientProcedureReadCharacteristicValueIndication_Error_gHciHardwareFailure_c:
                    shell_write(gHciHardwareFailure_c);
                    break;

                case GATTClientProcedureReadCharacteristicValueIndication_Error_gHciPageTimeout_c:
                    shell_write(gHciPageTimeout_c);
                    break;

                case GATTClientProcedureReadCharacteristicValueIndication_Error_gHciAuthenticationFailure_c:
                    shell_write(gHciAuthenticationFailure_c);
                    break;

                case GATTClientProcedureReadCharacteristicValueIndication_Error_gHciPinOrKeyMissing_c:
                    shell_write(gHciPinOrKeyMissing_c);
                    break;

                case GATTClientProcedureReadCharacteristicValueIndication_Error_gHciMemoryCapacityExceeded_c:
                    shell_write(gHciMemoryCapacityExceeded_c);
                    break;

                case GATTClientProcedureReadCharacteristicValueIndication_Error_gHciConnectionTimeout_c:
                    shell_write(gHciConnectionTimeout_c);
                    break;

                case GATTClientProcedureReadCharacteristicValueIndication_Error_gHciConnectionLimitExceeded_c:
                    shell_write(gHciConnectionLimitExceeded_c);
                    break;

                case GATTClientProcedureReadCharacteristicValueIndication_Error_gHciSynchronousConnectionLimitToADeviceExceeded_c:
                    shell_write(gHciSynchronousConnectionLimitToADeviceExceeded_c);
                    break;

                case GATTClientProcedureReadCharacteristicValueIndication_Error_gHciAclConnectionAlreadyExists_c:
                    shell_write(gHciAclConnectionAlreadyExists_c);
                    break;

                case GATTClientProcedureReadCharacteristicValueIndication_Error_gHciCommandDisallowed_c:
                    shell_write(gHciCommandDisallowed_c);
                    break;

                case GATTClientProcedureReadCharacteristicValueIndication_Error_gHciConnectionRejectedDueToLimitedResources_c:
                    shell_write(gHciConnectionRejectedDueToLimitedResources_c);
                    break;

                case GATTClientProcedureReadCharacteristicValueIndication_Error_gHciConnectionRejectedDueToSecurityReasons_c:
                    shell_write(gHciConnectionRejectedDueToSecurityReasons_c);
                    break;

                case GATTClientProcedureReadCharacteristicValueIndication_Error_gHciConnectionRejectedDueToUnacceptableBdAddr_c:
                    shell_write(gHciConnectionRejectedDueToUnacceptableBdAddr_c);
                    break;

                case GATTClientProcedureReadCharacteristicValueIndication_Error_gHciConnectionAcceptTimeoutExceeded_c:
                    shell_write(gHciConnectionAcceptTimeoutExceeded_c);
                    break;

                case GATTClientProcedureReadCharacteristicValueIndication_Error_gHciUnsupportedFeatureOrParameterValue_c:
                    shell_write(gHciUnsupportedFeatureOrParameterValue_c);
                    break;

                case GATTClientProcedureReadCharacteristicValueIndication_Error_gHciInvalidHciCommandParameters_c:
                    shell_write(gHciInvalidHciCommandParameters_c);
                    break;

                case GATTClientProcedureReadCharacteristicValueIndication_Error_gHciRemoteUserTerminatedConnection_c:
                    shell_write(gHciRemoteUserTerminatedConnection_c);
                    break;

                case GATTClientProcedureReadCharacteristicValueIndication_Error_gHciRemoteDeviceTerminatedConnectionLowResources_c:
                    shell_write(gHciRemoteDeviceTerminatedConnectionLowResources_c);
                    break;

                case GATTClientProcedureReadCharacteristicValueIndication_Error_gHciRemoteDeviceTerminatedConnectionPowerOff_c:
                    shell_write(gHciRemoteDeviceTerminatedConnectionPowerOff_c);
                    break;

                case GATTClientProcedureReadCharacteristicValueIndication_Error_gHciConnectionTerminatedByLocalHost_c:
                    shell_write(gHciConnectionTerminatedByLocalHost_c);
                    break;

                case GATTClientProcedureReadCharacteristicValueIndication_Error_gHciRepeatedAttempts_c:
                    shell_write(gHciRepeatedAttempts_c);
                    break;

                case GATTClientProcedureReadCharacteristicValueIndication_Error_gHciPairingNotAllowed_c:
                    shell_write(gHciPairingNotAllowed_c);
                    break;

                case GATTClientProcedureReadCharacteristicValueIndication_Error_gHciUnknownLpmPdu_c:
                    shell_write(gHciUnknownLpmPdu_c);
                    break;

                case GATTClientProcedureReadCharacteristicValueIndication_Error_gHciUnsupportedRemoteFeature_c:
                    shell_write(gHciUnsupportedRemoteFeature_c);
                    break;

                case GATTClientProcedureReadCharacteristicValueIndication_Error_gHciScoOffsetRejected_c:
                    shell_write(gHciScoOffsetRejected_c);
                    break;

                case GATTClientProcedureReadCharacteristicValueIndication_Error_gHciScoIntervalRejected_c:
                    shell_write(gHciScoIntervalRejected_c);
                    break;

                case GATTClientProcedureReadCharacteristicValueIndication_Error_gHciScoAirModeRejected_c:
                    shell_write(gHciScoAirModeRejected_c);
                    break;

                case GATTClientProcedureReadCharacteristicValueIndication_Error_gHciInvalidLpmParameters_c:
                    shell_write(gHciInvalidLpmParameters_c);
                    break;

                case GATTClientProcedureReadCharacteristicValueIndication_Error_gHciUnspecifiedError_c:
                    shell_write(gHciUnspecifiedc);
                    break;

                case GATTClientProcedureReadCharacteristicValueIndication_Error_gHciUnsupportedLpmParameterValue_c:
                    shell_write(gHciUnsupportedLpmParameterValue_c);
                    break;

                case GATTClientProcedureReadCharacteristicValueIndication_Error_gHciRoleChangeNotAllowed_c:
                    shell_write(gHciRoleChangeNotAllowed_c);
                    break;

                case GATTClientProcedureReadCharacteristicValueIndication_Error_gHciLLResponseTimeout_c:
                    shell_write(gHciLLResponseTimeout_c);
                    break;

                case GATTClientProcedureReadCharacteristicValueIndication_Error_gHciLmpErrorTransactionCollision_c:
                    shell_write(gHciLmpErrorTransactionCollision_c);
                    break;

                case GATTClientProcedureReadCharacteristicValueIndication_Error_gHciLmpPduNotAllowed_c:
                    shell_write(gHciLmpPduNotAllowed_c);
                    break;

                case GATTClientProcedureReadCharacteristicValueIndication_Error_gHciEncryptionModeNotAcceptable_c:
                    shell_write(gHciEncryptionModeNotAcceptable_c);
                    break;

                case GATTClientProcedureReadCharacteristicValueIndication_Error_gHciLinkKeyCannotBeChanged_c:
                    shell_write(gHciLinkKeyCannotBeChanged_c);
                    break;

                case GATTClientProcedureReadCharacteristicValueIndication_Error_gHciRequestedQosNotSupported_c:
                    shell_write(gHciRequestedQosNotSupported_c);
                    break;

                case GATTClientProcedureReadCharacteristicValueIndication_Error_gHciInstantPassed_c:
                    shell_write(gHciInstantPassed_c);
                    break;

                case GATTClientProcedureReadCharacteristicValueIndication_Error_gHciPairingWithUnitKeyNotSupported_c:
                    shell_write(gHciPairingWithUnitKeyNotSupported_c);
                    break;

                case GATTClientProcedureReadCharacteristicValueIndication_Error_gHciDifferentTransactionCollision_c:
                    shell_write(gHciDifferentTransactionCollision_c);
                    break;

                case GATTClientProcedureReadCharacteristicValueIndication_Error_gHciReserved_0x2B_c:
                    shell_write(gHciReserved_0x2B_c);
                    break;

                case GATTClientProcedureReadCharacteristicValueIndication_Error_gHciQosNotAcceptableParameter_c:
                    shell_write(gHciQosNotAcceptableParameter_c);
                    break;

                case GATTClientProcedureReadCharacteristicValueIndication_Error_gHciQosRejected_c:
                    shell_write(gHciQosRejected_c);
                    break;

                case GATTClientProcedureReadCharacteristicValueIndication_Error_gHciChannelClassificationNotSupported_c:
                    shell_write(gHciChannelClassificationNotSupported_c);
                    break;

                case GATTClientProcedureReadCharacteristicValueIndication_Error_gHciInsufficientSecurity_c:
                    shell_write(gHciInsufficientSecurity_c);
                    break;

                case GATTClientProcedureReadCharacteristicValueIndication_Error_gHciParameterOutOfMandatoryRange_c:
                    shell_write(gHciParameterOutOfMandatoryRange_c);
                    break;

                case GATTClientProcedureReadCharacteristicValueIndication_Error_gHciReserved_0x31_c:
                    shell_write(gHciReserved_0x31_c);
                    break;

                case GATTClientProcedureReadCharacteristicValueIndication_Error_gHciRoleSwitchPending_c:
                    shell_write(gHciRoleSwitchPending_c);
                    break;

                case GATTClientProcedureReadCharacteristicValueIndication_Error_gHciReserved_0x33_c:
                    shell_write(gHciReserved_0x33_c);
                    break;

                case GATTClientProcedureReadCharacteristicValueIndication_Error_gHciReservedSlotViolation_c:
                    shell_write(gHciReservedSlotViolation_c);
                    break;

                case GATTClientProcedureReadCharacteristicValueIndication_Error_gHciRoleSwitchFailed_c:
                    shell_write(gHciRoleSwitchFailed_c);
                    break;

                case GATTClientProcedureReadCharacteristicValueIndication_Error_gHciExtendedInquiryResponseTooLarge_c:
                    shell_write(gHciExtendedInquiryResponseTooLarge_c);
                    break;

                case GATTClientProcedureReadCharacteristicValueIndication_Error_gHciSecureSimplePairingNotSupportedByHost_c:
                    shell_write(gHciSecureSimplePairingNotSupportedByHost_c);
                    break;

                case GATTClientProcedureReadCharacteristicValueIndication_Error_gHciHostBusyPairing_c:
                    shell_write(gHciHostBusyPairing_c);
                    break;

                case GATTClientProcedureReadCharacteristicValueIndication_Error_gHciConnectionRejectedDueToNoSuitableChannelFound_c:
                    shell_write(gHciConnectionRejectedDueToNoSuitableChannelFound_c);
                    break;

                case GATTClientProcedureReadCharacteristicValueIndication_Error_gHciControllerBusy_c:
                    shell_write(gHciControllerBusy_c);
                    break;

                case GATTClientProcedureReadCharacteristicValueIndication_Error_gHciUnacceptableConnectionParameters_c:
                    shell_write(gHciUnacceptableConnectionParameters_c);
                    break;

                case GATTClientProcedureReadCharacteristicValueIndication_Error_gHciDirectedAdvertisingTimeout_c:
                    shell_write(gHciDirectedAdvertisingTimeout_c);
                    break;

                case GATTClientProcedureReadCharacteristicValueIndication_Error_gHciConnectionTerminatedDueToMicFailure_c:
                    shell_write(gHciConnectionTerminatedDueToMicFailure_c);
                    break;

                case GATTClientProcedureReadCharacteristicValueIndication_Error_gHciConnectionFailedToBeEstablished_c:
                    shell_write(gHciConnectionFailedToBeEstablished_c);
                    break;

                case GATTClientProcedureReadCharacteristicValueIndication_Error_gHciMacConnectionFailed_c:
                    shell_write(gHciMacConnectionFailed_c);
                    break;

                case GATTClientProcedureReadCharacteristicValueIndication_Error_gHciCoarseClockAdjustmentRejected_c:
                    shell_write(gHciCoarseClockAdjustmentRejected_c);
                    break;

                case GATTClientProcedureReadCharacteristicValueIndication_Error_gHciAlreadyInit_c:
                    shell_write(gHciAlreadyInit_c);
                    break;

                case GATTClientProcedureReadCharacteristicValueIndication_Error_gHciInvalidParameter_c:
                    shell_write(gHciInvalidParameter_c);
                    break;

                case GATTClientProcedureReadCharacteristicValueIndication_Error_gHciCallbackNotInstalled_c:
                    shell_write(gHciCallbackNotInstalled_c);
                    break;

                case GATTClientProcedureReadCharacteristicValueIndication_Error_gHciCallbackAlreadyInstalled_c:
                    shell_write(gHciCallbackAlreadyInstalled_c);
                    break;

                case GATTClientProcedureReadCharacteristicValueIndication_Error_gHciCommandNotSupported_c:
                    shell_write(gHciCommandNotSupported_c);
                    break;

                case GATTClientProcedureReadCharacteristicValueIndication_Error_gHciEventNotSupported_c:
                    shell_write(gHciEventNotSupported_c);
                    break;

                case GATTClientProcedureReadCharacteristicValueIndication_Error_gHciTransportError_c:
                    shell_write(gHciTransportc);
                    break;

                case GATTClientProcedureReadCharacteristicValueIndication_Error_gL2caAlreadyInit_c:
                    shell_write(gL2caAlreadyInit_c);
                    break;

                case GATTClientProcedureReadCharacteristicValueIndication_Error_gL2caInsufficientResources_c:
                    shell_write(gL2caInsufficientResources_c);
                    break;

                case GATTClientProcedureReadCharacteristicValueIndication_Error_gL2caCallbackNotInstalled_c:
                    shell_write(gL2caCallbackNotInstalled_c);
                    break;

                case GATTClientProcedureReadCharacteristicValueIndication_Error_gL2caCallbackAlreadyInstalled_c:
                    shell_write(gL2caCallbackAlreadyInstalled_c);
                    break;

                case GATTClientProcedureReadCharacteristicValueIndication_Error_gL2caLePsmInvalid_c:
                    shell_write(gL2caLePsmInvalid_c);
                    break;

                case GATTClientProcedureReadCharacteristicValueIndication_Error_gL2caLePsmAlreadyRegistered_c:
                    shell_write(gL2caLePsmAlreadyRegistered_c);
                    break;

                case GATTClientProcedureReadCharacteristicValueIndication_Error_gL2caLePsmNotRegistered_c:
                    shell_write(gL2caLePsmNotRegistered_c);
                    break;

                case GATTClientProcedureReadCharacteristicValueIndication_Error_gL2caLePsmInsufficientResources_c:
                    shell_write(gL2caLePsmInsufficientResources_c);
                    break;

                case GATTClientProcedureReadCharacteristicValueIndication_Error_gL2caChannelInvalid_c:
                    shell_write(gL2caChannelInvalid_c);
                    break;

                case GATTClientProcedureReadCharacteristicValueIndication_Error_gL2caChannelClosed_c:
                    shell_write(gL2caChannelClosed_c);
                    break;

                case GATTClientProcedureReadCharacteristicValueIndication_Error_gL2caChannelAlreadyConnected_c:
                    shell_write(gL2caChannelAlreadyConnected_c);
                    break;

                case GATTClientProcedureReadCharacteristicValueIndication_Error_gL2caConnectionParametersRejected_c:
                    shell_write(gL2caConnectionParametersRejected_c);
                    break;

                case GATTClientProcedureReadCharacteristicValueIndication_Error_gL2caChannelBusy_c:
                    shell_write(gL2caChannelBusy_c);
                    break;

                case GATTClientProcedureReadCharacteristicValueIndication_Error_gL2caInvalidParameter_c:
                    shell_write(gL2caInvalidParameter_c);
                    break;

                case GATTClientProcedureReadCharacteristicValueIndication_Error_gL2caError_c:
                    shell_write(gL2cac);
                    break;

                case GATTClientProcedureReadCharacteristicValueIndication_Error_gSmNullCBFunction_c:
                    shell_write(gSmNullCBFunction_c);
                    break;

                case GATTClientProcedureReadCharacteristicValueIndication_Error_gSmCommandNotSupported_c:
                    shell_write(gSmCommandNotSupported_c);
                    break;

                case GATTClientProcedureReadCharacteristicValueIndication_Error_gSmUnexpectedCommand_c:
                    shell_write(gSmUnexpectedCommand_c);
                    break;

                case GATTClientProcedureReadCharacteristicValueIndication_Error_gSmInvalidCommandCode_c:
                    shell_write(gSmInvalidCommandCode_c);
                    break;

                case GATTClientProcedureReadCharacteristicValueIndication_Error_gSmInvalidCommandLength_c:
                    shell_write(gSmInvalidCommandLength_c);
                    break;

                case GATTClientProcedureReadCharacteristicValueIndication_Error_gSmInvalidCommandParameter_c:
                    shell_write(gSmInvalidCommandParameter_c);
                    break;

                case GATTClientProcedureReadCharacteristicValueIndication_Error_gSmInvalidDeviceId_c:
                    shell_write(gSmInvalidDeviceId_c);
                    break;

                case GATTClientProcedureReadCharacteristicValueIndication_Error_gSmInvalidInternalOperation_c:
                    shell_write(gSmInvalidInternalOperation_c);
                    break;

                case GATTClientProcedureReadCharacteristicValueIndication_Error_gSmInvalidConnectionHandle_c:
                    shell_write(gSmInvalidConnectionHandle_c);
                    break;

                case GATTClientProcedureReadCharacteristicValueIndication_Error_gSmInproperKeyDistributionField_c:
                    shell_write(gSmInproperKeyDistributionField_c);
                    break;

                case GATTClientProcedureReadCharacteristicValueIndication_Error_gSmUnexpectedKeyType_c:
                    shell_write(gSmUnexpectedKeyType_c);
                    break;

                case GATTClientProcedureReadCharacteristicValueIndication_Error_gSmUnexpectedPairingTerminationReason_c:
                    shell_write(gSmUnexpectedPairingTerminationReason_c);
                    break;

                case GATTClientProcedureReadCharacteristicValueIndication_Error_gSmUnexpectedKeyset_c:
                    shell_write(gSmUnexpectedKeyset_c);
                    break;

                case GATTClientProcedureReadCharacteristicValueIndication_Error_gSmSmpTimeoutOccurred_c:
                    shell_write(gSmSmpTimeoutOccurred_c);
                    break;

                case GATTClientProcedureReadCharacteristicValueIndication_Error_gSmUnknownSmpPacketType_c:
                    shell_write(gSmUnknownSmpPacketType_c);
                    break;

                case GATTClientProcedureReadCharacteristicValueIndication_Error_gSmInvalidSmpPacketLength_c:
                    shell_write(gSmInvalidSmpPacketLength_c);
                    break;

                case GATTClientProcedureReadCharacteristicValueIndication_Error_gSmInvalidSmpPacketParameter_c:
                    shell_write(gSmInvalidSmpPacketParameter_c);
                    break;

                case GATTClientProcedureReadCharacteristicValueIndication_Error_gSmReceivedUnexpectedSmpPacket_c:
                    shell_write(gSmReceivedUnexpectedSmpPacket_c);
                    break;

                case GATTClientProcedureReadCharacteristicValueIndication_Error_gSmReceivedSmpPacketFromUnknownDevice_c:
                    shell_write(gSmReceivedSmpPacketFromUnknownDevice_c);
                    break;

                case GATTClientProcedureReadCharacteristicValueIndication_Error_gSmReceivedUnexpectedHciEvent_c:
                    shell_write(gSmReceivedUnexpectedHciEvent_c);
                    break;

                case GATTClientProcedureReadCharacteristicValueIndication_Error_gSmReceivedHciEventFromUnknownDevice_c:
                    shell_write(gSmReceivedHciEventFromUnknownDevice_c);
                    break;

                case GATTClientProcedureReadCharacteristicValueIndication_Error_gSmInvalidHciEventParameter_c:
                    shell_write(gSmInvalidHciEventParameter_c);
                    break;

                case GATTClientProcedureReadCharacteristicValueIndication_Error_gSmLlConnectionEncryptionInProgress_c:
                    shell_write(gSmLlConnectionEncryptionInProgress_c);
                    break;

                case GATTClientProcedureReadCharacteristicValueIndication_Error_gSmLlConnectionEncryptionFailure_c:
                    shell_write(gSmLlConnectionEncryptionFailure_c);
                    break;

                case GATTClientProcedureReadCharacteristicValueIndication_Error_gSmInsufficientResources_c:
                    shell_write(gSmInsufficientResources_c);
                    break;

                case GATTClientProcedureReadCharacteristicValueIndication_Error_gSmOobDataAddressMismatch_c:
                    shell_write(gSmOobDataAddressMismatch_c);
                    break;

//                case GATTClientProcedureReadCharacteristicValueIndication_Error_gSmSmpPacketReceivedAfterTimeoutOccurred_c:
//                    shell_write(gSmSmpPacketReceivedAfterTimeoutOccurred_c);
//                    break;

                case GATTClientProcedureReadCharacteristicValueIndication_Error_gSmPairingErrorPasskeyEntryFailed_c:
                    shell_write(gSmPairingErrorPasskeyEntryFailed_c);
                    break;

                case GATTClientProcedureReadCharacteristicValueIndication_Error_gSmPairingErrorConfirmValueFailed_c:
                    shell_write(gSmPairingErrorConfirmValueFailed_c);
                    break;

                case GATTClientProcedureReadCharacteristicValueIndication_Error_gSmPairingErrorCommandNotSupported_c:
                    shell_write(gSmPairingErrorCommandNotSupported_c);
                    break;

                case GATTClientProcedureReadCharacteristicValueIndication_Error_gSmPairingErrorInvalidParameters_c:
                    shell_write(gSmPairingErrorInvalidParameters_c);
                    break;

                case GATTClientProcedureReadCharacteristicValueIndication_Error_gSmPairingErrorUnknownReason_c:
                    shell_write(gSmPairingErrorUnknownReason_c);
                    break;

                case GATTClientProcedureReadCharacteristicValueIndication_Error_gSmTbResolvableAddressDoesNotMatchIrk_c:
                    shell_write(gSmTbResolvableAddressDoesNotMatchIrk_c);
                    break;

                case GATTClientProcedureReadCharacteristicValueIndication_Error_gSmTbInvalidDataSignature_c:
                    shell_write(gSmTbInvalidDataSignature_c);
                    break;

                case GATTClientProcedureReadCharacteristicValueIndication_Error_gAttInvalidHandle_c:
                    shell_write(gAttInvalidHandle_c);
                    break;

                case GATTClientProcedureReadCharacteristicValueIndication_Error_gAttReadNotPermitted_c:
                    shell_write(gAttReadNotPermitted_c);
                    break;

                case GATTClientProcedureReadCharacteristicValueIndication_Error_gAttWriteNotPermitted_c:
                    shell_write(gAttWriteNotPermitted_c);
                    break;

                case GATTClientProcedureReadCharacteristicValueIndication_Error_gAttInvalidPdu_c:
                    shell_write(gAttInvalidPdu_c);
                    break;

                case GATTClientProcedureReadCharacteristicValueIndication_Error_gAttInsufficientAuthentication_c:
                    shell_write(gAttInsufficientAuthentication_c);
                    break;

                case GATTClientProcedureReadCharacteristicValueIndication_Error_gAttRequestNotSupported_c:
                    shell_write(gAttRequestNotSupported_c);
                    break;

                case GATTClientProcedureReadCharacteristicValueIndication_Error_gAttInvalidOffset_c:
                    shell_write(gAttInvalidOffset_c);
                    break;

                case GATTClientProcedureReadCharacteristicValueIndication_Error_gAttInsufficientAuthorization_c:
                    shell_write(gAttInsufficientAuthorization_c);
                    break;

                case GATTClientProcedureReadCharacteristicValueIndication_Error_gAttPrepareQueueFull_c:
                    shell_write(gAttPrepareQueueFull_c);
                    break;

                case GATTClientProcedureReadCharacteristicValueIndication_Error_gAttAttributeNotFound_c:
                    shell_write(gAttAttributeNotFound_c);
                    break;

                case GATTClientProcedureReadCharacteristicValueIndication_Error_gAttAttributeNotLong_c:
                    shell_write(gAttAttributeNotLong_c);
                    break;

                case GATTClientProcedureReadCharacteristicValueIndication_Error_gAttInsufficientEncryptionKeySize_c:
                    shell_write(gAttInsufficientEncryptionKeySize_c);
                    break;

                case GATTClientProcedureReadCharacteristicValueIndication_Error_gAttInvalidAttributeValueLength_c:
                    shell_write(gAttInvalidAttributeValueLength_c);
                    break;

                case GATTClientProcedureReadCharacteristicValueIndication_Error_gAttUnlikelyor_c:
                    shell_write(gAttUnlikelyor_c);
                    break;

                case GATTClientProcedureReadCharacteristicValueIndication_Error_gAttInsufficientEncryption_c:
                    shell_write(gAttInsufficientEncryption_c);
                    break;

                case GATTClientProcedureReadCharacteristicValueIndication_Error_gAttUnsupportedGroupType_c:
                    shell_write(gAttUnsupportedGroupType_c);
                    break;

                case GATTClientProcedureReadCharacteristicValueIndication_Error_gAttInsufficientResources_c:
                    shell_write(gAttInsufficientResources_c);
                    break;

                case GATTClientProcedureReadCharacteristicValueIndication_Error_gGattAnotherProcedureInProgress_c:
                    shell_write(gGattAnotherProcedureInProgress_c);
                    break;

                case GATTClientProcedureReadCharacteristicValueIndication_Error_gGattLongAttributePacketsCorrupted_c:
                    shell_write(gGattLongAttributePacketsCorrupted_c);
                    break;

                case GATTClientProcedureReadCharacteristicValueIndication_Error_gGattMultipleAttributesOverflow_c:
                    shell_write(gGattMultipleAttributesOverflow_c);
                    break;

                case GATTClientProcedureReadCharacteristicValueIndication_Error_gGattUnexpectedReadMultipleResponseLength_c:
                    shell_write(gGattUnexpectedReadMultipleResponseLength_c);
                    break;

                case GATTClientProcedureReadCharacteristicValueIndication_Error_gGattInvalidValueLength_c:
                    shell_write(gGattInvalidValueLength_c);
                    break;

                case GATTClientProcedureReadCharacteristicValueIndication_Error_gGattServerTimeout_c:
                    shell_write(gGattServerTimeout_c);
                    break;

                case GATTClientProcedureReadCharacteristicValueIndication_Error_gGattIndicationAlreadyInProgress_c:
                    shell_write(gGattIndicationAlreadyInProgress_c);
                    break;

                case GATTClientProcedureReadCharacteristicValueIndication_Error_gGattClientConfirmationTimeout_c:
                    shell_write(gGattClientConfirmationTimeout_c);
                    break;

                case GATTClientProcedureReadCharacteristicValueIndication_Error_gGapAdvDataTooLong_c:
                    shell_write(gGapAdvDataTooLong_c);
                    break;

                case GATTClientProcedureReadCharacteristicValueIndication_Error_gGapScanRspDataTooLong_c:
                    shell_write(gGapScanRspDataTooLong_c);
                    break;

                case GATTClientProcedureReadCharacteristicValueIndication_Error_gGapDeviceNotBonded_c:
                    shell_write(gGapDeviceNotBonded_c);
                    break;

                case GATTClientProcedureReadCharacteristicValueIndication_Error_gDevDbCccdLimitReached_c:
                    shell_write(gDevDbCccdLimitReached_c);
                    break;

                case GATTClientProcedureReadCharacteristicValueIndication_Error_gDevDbCccdNotFound_c:
                    shell_write(gDevDbCccdNotFound_c);
                    break;

                case GATTClientProcedureReadCharacteristicValueIndication_Error_gGattDbInvalidHandle_c:
                    shell_write(gGattDbInvalidHandle_c);
                    break;

                case GATTClientProcedureReadCharacteristicValueIndication_Error_gGattDbCharacteristicNotFound_c:
                    shell_write(gGattDbCharacteristicNotFound_c);
                    break;

                case GATTClientProcedureReadCharacteristicValueIndication_Error_gGattDbCccdNotFound_c:
                    shell_write(gGattDbCccdNotFound_c);
                    break;

                case GATTClientProcedureReadCharacteristicValueIndication_Error_gGattDbServiceNotFound_c:
                    shell_write(gGattDbServiceNotFound_c);
                    break;

                case GATTClientProcedureReadCharacteristicValueIndication_Error_gGattDbDescriptorNotFound_c:
                    shell_write(gGattDbDescriptorNotFound_c);
                    break;

                default:
                    shell_printf("Unrecognized status 0x%04X", container->Data.GATTClientProcedureReadCharacteristicValueIndication.Error);
                    break;
            }

            break;

        case 0x448A:
            shell_write("\r\nGATTClientProcedureReadUsingCharacteristicUuidIndication");
            shell_write(" -> ");

            switch (container->Data.GATTClientProcedureReadUsingCharacteristicUuidIndication.ProcedureResult)
            {
                case GATTClientProcedureReadUsingCharacteristicUuidIndication_ProcedureResult_gGattProcSuccess_c:
                    shell_write(gGattProcSuccess_c);
                    break;

                case GATTClientProcedureReadUsingCharacteristicUuidIndication_ProcedureResult_gProcedureError_c:
                    shell_write(gProcedureError_c);
                    break;

                default:
                    shell_printf("Unrecognized status 0x%02X", container->Data.GATTClientProcedureReadUsingCharacteristicUuidIndication.ProcedureResult);
                    break;
            }

            shell_write(" -> ");

            switch (container->Data.GATTClientProcedureReadUsingCharacteristicUuidIndication.Error)
            {
                case GATTClientProcedureReadUsingCharacteristicUuidIndication_Error_gBleSuccess_c:
                    shell_write(gBleSuccess_c);
                    break;

                case GATTClientProcedureReadUsingCharacteristicUuidIndication_Error_gBleInvalidParameter_c:
                    shell_write(gBleInvalidParameter_c);
                    break;

                case GATTClientProcedureReadUsingCharacteristicUuidIndication_Error_gBleOverflow_c:
                    shell_write(gBleOverflow_c);
                    break;

                case GATTClientProcedureReadUsingCharacteristicUuidIndication_Error_gBleUnavailable_c:
                    shell_write(gBleUnavailable_c);
                    break;

                case GATTClientProcedureReadUsingCharacteristicUuidIndication_Error_gBleFeatureNotSupported_c:
                    shell_write(gBleFeatureNotSupported_c);
                    break;

                case GATTClientProcedureReadUsingCharacteristicUuidIndication_Error_gBleOutOfMemory_c:
                    shell_write(gBleOutOfMemory_c);
                    break;

                case GATTClientProcedureReadUsingCharacteristicUuidIndication_Error_gBleAlreadyInitialized_c:
                    shell_write(gBleAlreadyInitialized_c);
                    break;

                case GATTClientProcedureReadUsingCharacteristicUuidIndication_Error_gBleOsError_c:
                    shell_write(gBleOsc);
                    break;

                case GATTClientProcedureReadUsingCharacteristicUuidIndication_Error_gBleUnexpectedError_c:
                    shell_write(gBleUnexpectedc);
                    break;

                case GATTClientProcedureReadUsingCharacteristicUuidIndication_Error_gBleInvalidState_c:
                    shell_write(gBleInvalidState_c);
                    break;

                case GATTClientProcedureReadUsingCharacteristicUuidIndication_Error_gHciUnknownHciCommand_c:
                    shell_write(gHciUnknownHciCommand_c);
                    break;

                case GATTClientProcedureReadUsingCharacteristicUuidIndication_Error_gHciUnknownConnectionIdentifier_c:
                    shell_write(gHciUnknownConnectionIdentifier_c);
                    break;

                case GATTClientProcedureReadUsingCharacteristicUuidIndication_Error_gHciHardwareFailure_c:
                    shell_write(gHciHardwareFailure_c);
                    break;

                case GATTClientProcedureReadUsingCharacteristicUuidIndication_Error_gHciPageTimeout_c:
                    shell_write(gHciPageTimeout_c);
                    break;

                case GATTClientProcedureReadUsingCharacteristicUuidIndication_Error_gHciAuthenticationFailure_c:
                    shell_write(gHciAuthenticationFailure_c);
                    break;

                case GATTClientProcedureReadUsingCharacteristicUuidIndication_Error_gHciPinOrKeyMissing_c:
                    shell_write(gHciPinOrKeyMissing_c);
                    break;

                case GATTClientProcedureReadUsingCharacteristicUuidIndication_Error_gHciMemoryCapacityExceeded_c:
                    shell_write(gHciMemoryCapacityExceeded_c);
                    break;

                case GATTClientProcedureReadUsingCharacteristicUuidIndication_Error_gHciConnectionTimeout_c:
                    shell_write(gHciConnectionTimeout_c);
                    break;

                case GATTClientProcedureReadUsingCharacteristicUuidIndication_Error_gHciConnectionLimitExceeded_c:
                    shell_write(gHciConnectionLimitExceeded_c);
                    break;

                case GATTClientProcedureReadUsingCharacteristicUuidIndication_Error_gHciSynchronousConnectionLimitToADeviceExceeded_c:
                    shell_write(gHciSynchronousConnectionLimitToADeviceExceeded_c);
                    break;

                case GATTClientProcedureReadUsingCharacteristicUuidIndication_Error_gHciAclConnectionAlreadyExists_c:
                    shell_write(gHciAclConnectionAlreadyExists_c);
                    break;

                case GATTClientProcedureReadUsingCharacteristicUuidIndication_Error_gHciCommandDisallowed_c:
                    shell_write(gHciCommandDisallowed_c);
                    break;

                case GATTClientProcedureReadUsingCharacteristicUuidIndication_Error_gHciConnectionRejectedDueToLimitedResources_c:
                    shell_write(gHciConnectionRejectedDueToLimitedResources_c);
                    break;

                case GATTClientProcedureReadUsingCharacteristicUuidIndication_Error_gHciConnectionRejectedDueToSecurityReasons_c:
                    shell_write(gHciConnectionRejectedDueToSecurityReasons_c);
                    break;

                case GATTClientProcedureReadUsingCharacteristicUuidIndication_Error_gHciConnectionRejectedDueToUnacceptableBdAddr_c:
                    shell_write(gHciConnectionRejectedDueToUnacceptableBdAddr_c);
                    break;

                case GATTClientProcedureReadUsingCharacteristicUuidIndication_Error_gHciConnectionAcceptTimeoutExceeded_c:
                    shell_write(gHciConnectionAcceptTimeoutExceeded_c);
                    break;

                case GATTClientProcedureReadUsingCharacteristicUuidIndication_Error_gHciUnsupportedFeatureOrParameterValue_c:
                    shell_write(gHciUnsupportedFeatureOrParameterValue_c);
                    break;

                case GATTClientProcedureReadUsingCharacteristicUuidIndication_Error_gHciInvalidHciCommandParameters_c:
                    shell_write(gHciInvalidHciCommandParameters_c);
                    break;

                case GATTClientProcedureReadUsingCharacteristicUuidIndication_Error_gHciRemoteUserTerminatedConnection_c:
                    shell_write(gHciRemoteUserTerminatedConnection_c);
                    break;

                case GATTClientProcedureReadUsingCharacteristicUuidIndication_Error_gHciRemoteDeviceTerminatedConnectionLowResources_c:
                    shell_write(gHciRemoteDeviceTerminatedConnectionLowResources_c);
                    break;

                case GATTClientProcedureReadUsingCharacteristicUuidIndication_Error_gHciRemoteDeviceTerminatedConnectionPowerOff_c:
                    shell_write(gHciRemoteDeviceTerminatedConnectionPowerOff_c);
                    break;

                case GATTClientProcedureReadUsingCharacteristicUuidIndication_Error_gHciConnectionTerminatedByLocalHost_c:
                    shell_write(gHciConnectionTerminatedByLocalHost_c);
                    break;

                case GATTClientProcedureReadUsingCharacteristicUuidIndication_Error_gHciRepeatedAttempts_c:
                    shell_write(gHciRepeatedAttempts_c);
                    break;

                case GATTClientProcedureReadUsingCharacteristicUuidIndication_Error_gHciPairingNotAllowed_c:
                    shell_write(gHciPairingNotAllowed_c);
                    break;

                case GATTClientProcedureReadUsingCharacteristicUuidIndication_Error_gHciUnknownLpmPdu_c:
                    shell_write(gHciUnknownLpmPdu_c);
                    break;

                case GATTClientProcedureReadUsingCharacteristicUuidIndication_Error_gHciUnsupportedRemoteFeature_c:
                    shell_write(gHciUnsupportedRemoteFeature_c);
                    break;

                case GATTClientProcedureReadUsingCharacteristicUuidIndication_Error_gHciScoOffsetRejected_c:
                    shell_write(gHciScoOffsetRejected_c);
                    break;

                case GATTClientProcedureReadUsingCharacteristicUuidIndication_Error_gHciScoIntervalRejected_c:
                    shell_write(gHciScoIntervalRejected_c);
                    break;

                case GATTClientProcedureReadUsingCharacteristicUuidIndication_Error_gHciScoAirModeRejected_c:
                    shell_write(gHciScoAirModeRejected_c);
                    break;

                case GATTClientProcedureReadUsingCharacteristicUuidIndication_Error_gHciInvalidLpmParameters_c:
                    shell_write(gHciInvalidLpmParameters_c);
                    break;

                case GATTClientProcedureReadUsingCharacteristicUuidIndication_Error_gHciUnspecifiedError_c:
                    shell_write(gHciUnspecifiedc);
                    break;

                case GATTClientProcedureReadUsingCharacteristicUuidIndication_Error_gHciUnsupportedLpmParameterValue_c:
                    shell_write(gHciUnsupportedLpmParameterValue_c);
                    break;

                case GATTClientProcedureReadUsingCharacteristicUuidIndication_Error_gHciRoleChangeNotAllowed_c:
                    shell_write(gHciRoleChangeNotAllowed_c);
                    break;

                case GATTClientProcedureReadUsingCharacteristicUuidIndication_Error_gHciLLResponseTimeout_c:
                    shell_write(gHciLLResponseTimeout_c);
                    break;

                case GATTClientProcedureReadUsingCharacteristicUuidIndication_Error_gHciLmpErrorTransactionCollision_c:
                    shell_write(gHciLmpErrorTransactionCollision_c);
                    break;

                case GATTClientProcedureReadUsingCharacteristicUuidIndication_Error_gHciLmpPduNotAllowed_c:
                    shell_write(gHciLmpPduNotAllowed_c);
                    break;

                case GATTClientProcedureReadUsingCharacteristicUuidIndication_Error_gHciEncryptionModeNotAcceptable_c:
                    shell_write(gHciEncryptionModeNotAcceptable_c);
                    break;

                case GATTClientProcedureReadUsingCharacteristicUuidIndication_Error_gHciLinkKeyCannotBeChanged_c:
                    shell_write(gHciLinkKeyCannotBeChanged_c);
                    break;

                case GATTClientProcedureReadUsingCharacteristicUuidIndication_Error_gHciRequestedQosNotSupported_c:
                    shell_write(gHciRequestedQosNotSupported_c);
                    break;

                case GATTClientProcedureReadUsingCharacteristicUuidIndication_Error_gHciInstantPassed_c:
                    shell_write(gHciInstantPassed_c);
                    break;

                case GATTClientProcedureReadUsingCharacteristicUuidIndication_Error_gHciPairingWithUnitKeyNotSupported_c:
                    shell_write(gHciPairingWithUnitKeyNotSupported_c);
                    break;

                case GATTClientProcedureReadUsingCharacteristicUuidIndication_Error_gHciDifferentTransactionCollision_c:
                    shell_write(gHciDifferentTransactionCollision_c);
                    break;

                case GATTClientProcedureReadUsingCharacteristicUuidIndication_Error_gHciReserved_0x2B_c:
                    shell_write(gHciReserved_0x2B_c);
                    break;

                case GATTClientProcedureReadUsingCharacteristicUuidIndication_Error_gHciQosNotAcceptableParameter_c:
                    shell_write(gHciQosNotAcceptableParameter_c);
                    break;

                case GATTClientProcedureReadUsingCharacteristicUuidIndication_Error_gHciQosRejected_c:
                    shell_write(gHciQosRejected_c);
                    break;

                case GATTClientProcedureReadUsingCharacteristicUuidIndication_Error_gHciChannelClassificationNotSupported_c:
                    shell_write(gHciChannelClassificationNotSupported_c);
                    break;

                case GATTClientProcedureReadUsingCharacteristicUuidIndication_Error_gHciInsufficientSecurity_c:
                    shell_write(gHciInsufficientSecurity_c);
                    break;

                case GATTClientProcedureReadUsingCharacteristicUuidIndication_Error_gHciParameterOutOfMandatoryRange_c:
                    shell_write(gHciParameterOutOfMandatoryRange_c);
                    break;

                case GATTClientProcedureReadUsingCharacteristicUuidIndication_Error_gHciReserved_0x31_c:
                    shell_write(gHciReserved_0x31_c);
                    break;

                case GATTClientProcedureReadUsingCharacteristicUuidIndication_Error_gHciRoleSwitchPending_c:
                    shell_write(gHciRoleSwitchPending_c);
                    break;

                case GATTClientProcedureReadUsingCharacteristicUuidIndication_Error_gHciReserved_0x33_c:
                    shell_write(gHciReserved_0x33_c);
                    break;

                case GATTClientProcedureReadUsingCharacteristicUuidIndication_Error_gHciReservedSlotViolation_c:
                    shell_write(gHciReservedSlotViolation_c);
                    break;

                case GATTClientProcedureReadUsingCharacteristicUuidIndication_Error_gHciRoleSwitchFailed_c:
                    shell_write(gHciRoleSwitchFailed_c);
                    break;

                case GATTClientProcedureReadUsingCharacteristicUuidIndication_Error_gHciExtendedInquiryResponseTooLarge_c:
                    shell_write(gHciExtendedInquiryResponseTooLarge_c);
                    break;

                case GATTClientProcedureReadUsingCharacteristicUuidIndication_Error_gHciSecureSimplePairingNotSupportedByHost_c:
                    shell_write(gHciSecureSimplePairingNotSupportedByHost_c);
                    break;

                case GATTClientProcedureReadUsingCharacteristicUuidIndication_Error_gHciHostBusyPairing_c:
                    shell_write(gHciHostBusyPairing_c);
                    break;

                case GATTClientProcedureReadUsingCharacteristicUuidIndication_Error_gHciConnectionRejectedDueToNoSuitableChannelFound_c:
                    shell_write(gHciConnectionRejectedDueToNoSuitableChannelFound_c);
                    break;

                case GATTClientProcedureReadUsingCharacteristicUuidIndication_Error_gHciControllerBusy_c:
                    shell_write(gHciControllerBusy_c);
                    break;

                case GATTClientProcedureReadUsingCharacteristicUuidIndication_Error_gHciUnacceptableConnectionParameters_c:
                    shell_write(gHciUnacceptableConnectionParameters_c);
                    break;

                case GATTClientProcedureReadUsingCharacteristicUuidIndication_Error_gHciDirectedAdvertisingTimeout_c:
                    shell_write(gHciDirectedAdvertisingTimeout_c);
                    break;

                case GATTClientProcedureReadUsingCharacteristicUuidIndication_Error_gHciConnectionTerminatedDueToMicFailure_c:
                    shell_write(gHciConnectionTerminatedDueToMicFailure_c);
                    break;

                case GATTClientProcedureReadUsingCharacteristicUuidIndication_Error_gHciConnectionFailedToBeEstablished_c:
                    shell_write(gHciConnectionFailedToBeEstablished_c);
                    break;

                case GATTClientProcedureReadUsingCharacteristicUuidIndication_Error_gHciMacConnectionFailed_c:
                    shell_write(gHciMacConnectionFailed_c);
                    break;

                case GATTClientProcedureReadUsingCharacteristicUuidIndication_Error_gHciCoarseClockAdjustmentRejected_c:
                    shell_write(gHciCoarseClockAdjustmentRejected_c);
                    break;

                case GATTClientProcedureReadUsingCharacteristicUuidIndication_Error_gHciAlreadyInit_c:
                    shell_write(gHciAlreadyInit_c);
                    break;

                case GATTClientProcedureReadUsingCharacteristicUuidIndication_Error_gHciInvalidParameter_c:
                    shell_write(gHciInvalidParameter_c);
                    break;

                case GATTClientProcedureReadUsingCharacteristicUuidIndication_Error_gHciCallbackNotInstalled_c:
                    shell_write(gHciCallbackNotInstalled_c);
                    break;

                case GATTClientProcedureReadUsingCharacteristicUuidIndication_Error_gHciCallbackAlreadyInstalled_c:
                    shell_write(gHciCallbackAlreadyInstalled_c);
                    break;

                case GATTClientProcedureReadUsingCharacteristicUuidIndication_Error_gHciCommandNotSupported_c:
                    shell_write(gHciCommandNotSupported_c);
                    break;

                case GATTClientProcedureReadUsingCharacteristicUuidIndication_Error_gHciEventNotSupported_c:
                    shell_write(gHciEventNotSupported_c);
                    break;

                case GATTClientProcedureReadUsingCharacteristicUuidIndication_Error_gHciTransportError_c:
                    shell_write(gHciTransportc);
                    break;

                case GATTClientProcedureReadUsingCharacteristicUuidIndication_Error_gL2caAlreadyInit_c:
                    shell_write(gL2caAlreadyInit_c);
                    break;

                case GATTClientProcedureReadUsingCharacteristicUuidIndication_Error_gL2caInsufficientResources_c:
                    shell_write(gL2caInsufficientResources_c);
                    break;

                case GATTClientProcedureReadUsingCharacteristicUuidIndication_Error_gL2caCallbackNotInstalled_c:
                    shell_write(gL2caCallbackNotInstalled_c);
                    break;

                case GATTClientProcedureReadUsingCharacteristicUuidIndication_Error_gL2caCallbackAlreadyInstalled_c:
                    shell_write(gL2caCallbackAlreadyInstalled_c);
                    break;

                case GATTClientProcedureReadUsingCharacteristicUuidIndication_Error_gL2caLePsmInvalid_c:
                    shell_write(gL2caLePsmInvalid_c);
                    break;

                case GATTClientProcedureReadUsingCharacteristicUuidIndication_Error_gL2caLePsmAlreadyRegistered_c:
                    shell_write(gL2caLePsmAlreadyRegistered_c);
                    break;

                case GATTClientProcedureReadUsingCharacteristicUuidIndication_Error_gL2caLePsmNotRegistered_c:
                    shell_write(gL2caLePsmNotRegistered_c);
                    break;

                case GATTClientProcedureReadUsingCharacteristicUuidIndication_Error_gL2caLePsmInsufficientResources_c:
                    shell_write(gL2caLePsmInsufficientResources_c);
                    break;

                case GATTClientProcedureReadUsingCharacteristicUuidIndication_Error_gL2caChannelInvalid_c:
                    shell_write(gL2caChannelInvalid_c);
                    break;

                case GATTClientProcedureReadUsingCharacteristicUuidIndication_Error_gL2caChannelClosed_c:
                    shell_write(gL2caChannelClosed_c);
                    break;

                case GATTClientProcedureReadUsingCharacteristicUuidIndication_Error_gL2caChannelAlreadyConnected_c:
                    shell_write(gL2caChannelAlreadyConnected_c);
                    break;

                case GATTClientProcedureReadUsingCharacteristicUuidIndication_Error_gL2caConnectionParametersRejected_c:
                    shell_write(gL2caConnectionParametersRejected_c);
                    break;

                case GATTClientProcedureReadUsingCharacteristicUuidIndication_Error_gL2caChannelBusy_c:
                    shell_write(gL2caChannelBusy_c);
                    break;

                case GATTClientProcedureReadUsingCharacteristicUuidIndication_Error_gL2caInvalidParameter_c:
                    shell_write(gL2caInvalidParameter_c);
                    break;

                case GATTClientProcedureReadUsingCharacteristicUuidIndication_Error_gL2caError_c:
                    shell_write(gL2cac);
                    break;

                case GATTClientProcedureReadUsingCharacteristicUuidIndication_Error_gSmNullCBFunction_c:
                    shell_write(gSmNullCBFunction_c);
                    break;

                case GATTClientProcedureReadUsingCharacteristicUuidIndication_Error_gSmCommandNotSupported_c:
                    shell_write(gSmCommandNotSupported_c);
                    break;

                case GATTClientProcedureReadUsingCharacteristicUuidIndication_Error_gSmUnexpectedCommand_c:
                    shell_write(gSmUnexpectedCommand_c);
                    break;

                case GATTClientProcedureReadUsingCharacteristicUuidIndication_Error_gSmInvalidCommandCode_c:
                    shell_write(gSmInvalidCommandCode_c);
                    break;

                case GATTClientProcedureReadUsingCharacteristicUuidIndication_Error_gSmInvalidCommandLength_c:
                    shell_write(gSmInvalidCommandLength_c);
                    break;

                case GATTClientProcedureReadUsingCharacteristicUuidIndication_Error_gSmInvalidCommandParameter_c:
                    shell_write(gSmInvalidCommandParameter_c);
                    break;

                case GATTClientProcedureReadUsingCharacteristicUuidIndication_Error_gSmInvalidDeviceId_c:
                    shell_write(gSmInvalidDeviceId_c);
                    break;

                case GATTClientProcedureReadUsingCharacteristicUuidIndication_Error_gSmInvalidInternalOperation_c:
                    shell_write(gSmInvalidInternalOperation_c);
                    break;

                case GATTClientProcedureReadUsingCharacteristicUuidIndication_Error_gSmInvalidConnectionHandle_c:
                    shell_write(gSmInvalidConnectionHandle_c);
                    break;

                case GATTClientProcedureReadUsingCharacteristicUuidIndication_Error_gSmInproperKeyDistributionField_c:
                    shell_write(gSmInproperKeyDistributionField_c);
                    break;

                case GATTClientProcedureReadUsingCharacteristicUuidIndication_Error_gSmUnexpectedKeyType_c:
                    shell_write(gSmUnexpectedKeyType_c);
                    break;

                case GATTClientProcedureReadUsingCharacteristicUuidIndication_Error_gSmUnexpectedPairingTerminationReason_c:
                    shell_write(gSmUnexpectedPairingTerminationReason_c);
                    break;

                case GATTClientProcedureReadUsingCharacteristicUuidIndication_Error_gSmUnexpectedKeyset_c:
                    shell_write(gSmUnexpectedKeyset_c);
                    break;

                case GATTClientProcedureReadUsingCharacteristicUuidIndication_Error_gSmSmpTimeoutOccurred_c:
                    shell_write(gSmSmpTimeoutOccurred_c);
                    break;

                case GATTClientProcedureReadUsingCharacteristicUuidIndication_Error_gSmUnknownSmpPacketType_c:
                    shell_write(gSmUnknownSmpPacketType_c);
                    break;

                case GATTClientProcedureReadUsingCharacteristicUuidIndication_Error_gSmInvalidSmpPacketLength_c:
                    shell_write(gSmInvalidSmpPacketLength_c);
                    break;

                case GATTClientProcedureReadUsingCharacteristicUuidIndication_Error_gSmInvalidSmpPacketParameter_c:
                    shell_write(gSmInvalidSmpPacketParameter_c);
                    break;

                case GATTClientProcedureReadUsingCharacteristicUuidIndication_Error_gSmReceivedUnexpectedSmpPacket_c:
                    shell_write(gSmReceivedUnexpectedSmpPacket_c);
                    break;

                case GATTClientProcedureReadUsingCharacteristicUuidIndication_Error_gSmReceivedSmpPacketFromUnknownDevice_c:
                    shell_write(gSmReceivedSmpPacketFromUnknownDevice_c);
                    break;

                case GATTClientProcedureReadUsingCharacteristicUuidIndication_Error_gSmReceivedUnexpectedHciEvent_c:
                    shell_write(gSmReceivedUnexpectedHciEvent_c);
                    break;

                case GATTClientProcedureReadUsingCharacteristicUuidIndication_Error_gSmReceivedHciEventFromUnknownDevice_c:
                    shell_write(gSmReceivedHciEventFromUnknownDevice_c);
                    break;

                case GATTClientProcedureReadUsingCharacteristicUuidIndication_Error_gSmInvalidHciEventParameter_c:
                    shell_write(gSmInvalidHciEventParameter_c);
                    break;

                case GATTClientProcedureReadUsingCharacteristicUuidIndication_Error_gSmLlConnectionEncryptionInProgress_c:
                    shell_write(gSmLlConnectionEncryptionInProgress_c);
                    break;

                case GATTClientProcedureReadUsingCharacteristicUuidIndication_Error_gSmLlConnectionEncryptionFailure_c:
                    shell_write(gSmLlConnectionEncryptionFailure_c);
                    break;

                case GATTClientProcedureReadUsingCharacteristicUuidIndication_Error_gSmInsufficientResources_c:
                    shell_write(gSmInsufficientResources_c);
                    break;

                case GATTClientProcedureReadUsingCharacteristicUuidIndication_Error_gSmOobDataAddressMismatch_c:
                    shell_write(gSmOobDataAddressMismatch_c);
                    break;

//                case GATTClientProcedureReadUsingCharacteristicUuidIndication_Error_gSmSmpPacketReceivedAfterTimeoutOccurred_c:
//                    shell_write(gSmSmpPacketReceivedAfterTimeoutOccurred_c);
//                    break;

                case GATTClientProcedureReadUsingCharacteristicUuidIndication_Error_gSmPairingErrorPasskeyEntryFailed_c:
                    shell_write(gSmPairingErrorPasskeyEntryFailed_c);
                    break;

                case GATTClientProcedureReadUsingCharacteristicUuidIndication_Error_gSmPairingErrorConfirmValueFailed_c:
                    shell_write(gSmPairingErrorConfirmValueFailed_c);
                    break;

                case GATTClientProcedureReadUsingCharacteristicUuidIndication_Error_gSmPairingErrorCommandNotSupported_c:
                    shell_write(gSmPairingErrorCommandNotSupported_c);
                    break;

                case GATTClientProcedureReadUsingCharacteristicUuidIndication_Error_gSmPairingErrorInvalidParameters_c:
                    shell_write(gSmPairingErrorInvalidParameters_c);
                    break;

                case GATTClientProcedureReadUsingCharacteristicUuidIndication_Error_gSmPairingErrorUnknownReason_c:
                    shell_write(gSmPairingErrorUnknownReason_c);
                    break;

                case GATTClientProcedureReadUsingCharacteristicUuidIndication_Error_gSmTbResolvableAddressDoesNotMatchIrk_c:
                    shell_write(gSmTbResolvableAddressDoesNotMatchIrk_c);
                    break;

                case GATTClientProcedureReadUsingCharacteristicUuidIndication_Error_gSmTbInvalidDataSignature_c:
                    shell_write(gSmTbInvalidDataSignature_c);
                    break;

                case GATTClientProcedureReadUsingCharacteristicUuidIndication_Error_gAttInvalidHandle_c:
                    shell_write(gAttInvalidHandle_c);
                    break;

                case GATTClientProcedureReadUsingCharacteristicUuidIndication_Error_gAttReadNotPermitted_c:
                    shell_write(gAttReadNotPermitted_c);
                    break;

                case GATTClientProcedureReadUsingCharacteristicUuidIndication_Error_gAttWriteNotPermitted_c:
                    shell_write(gAttWriteNotPermitted_c);
                    break;

                case GATTClientProcedureReadUsingCharacteristicUuidIndication_Error_gAttInvalidPdu_c:
                    shell_write(gAttInvalidPdu_c);
                    break;

                case GATTClientProcedureReadUsingCharacteristicUuidIndication_Error_gAttInsufficientAuthentication_c:
                    shell_write(gAttInsufficientAuthentication_c);
                    break;

                case GATTClientProcedureReadUsingCharacteristicUuidIndication_Error_gAttRequestNotSupported_c:
                    shell_write(gAttRequestNotSupported_c);
                    break;

                case GATTClientProcedureReadUsingCharacteristicUuidIndication_Error_gAttInvalidOffset_c:
                    shell_write(gAttInvalidOffset_c);
                    break;

                case GATTClientProcedureReadUsingCharacteristicUuidIndication_Error_gAttInsufficientAuthorization_c:
                    shell_write(gAttInsufficientAuthorization_c);
                    break;

                case GATTClientProcedureReadUsingCharacteristicUuidIndication_Error_gAttPrepareQueueFull_c:
                    shell_write(gAttPrepareQueueFull_c);
                    break;

                case GATTClientProcedureReadUsingCharacteristicUuidIndication_Error_gAttAttributeNotFound_c:
                    shell_write(gAttAttributeNotFound_c);
                    break;

                case GATTClientProcedureReadUsingCharacteristicUuidIndication_Error_gAttAttributeNotLong_c:
                    shell_write(gAttAttributeNotLong_c);
                    break;

                case GATTClientProcedureReadUsingCharacteristicUuidIndication_Error_gAttInsufficientEncryptionKeySize_c:
                    shell_write(gAttInsufficientEncryptionKeySize_c);
                    break;

                case GATTClientProcedureReadUsingCharacteristicUuidIndication_Error_gAttInvalidAttributeValueLength_c:
                    shell_write(gAttInvalidAttributeValueLength_c);
                    break;

                case GATTClientProcedureReadUsingCharacteristicUuidIndication_Error_gAttUnlikelyor_c:
                    shell_write(gAttUnlikelyor_c);
                    break;

                case GATTClientProcedureReadUsingCharacteristicUuidIndication_Error_gAttInsufficientEncryption_c:
                    shell_write(gAttInsufficientEncryption_c);
                    break;

                case GATTClientProcedureReadUsingCharacteristicUuidIndication_Error_gAttUnsupportedGroupType_c:
                    shell_write(gAttUnsupportedGroupType_c);
                    break;

                case GATTClientProcedureReadUsingCharacteristicUuidIndication_Error_gAttInsufficientResources_c:
                    shell_write(gAttInsufficientResources_c);
                    break;

                case GATTClientProcedureReadUsingCharacteristicUuidIndication_Error_gGattAnotherProcedureInProgress_c:
                    shell_write(gGattAnotherProcedureInProgress_c);
                    break;

                case GATTClientProcedureReadUsingCharacteristicUuidIndication_Error_gGattLongAttributePacketsCorrupted_c:
                    shell_write(gGattLongAttributePacketsCorrupted_c);
                    break;

                case GATTClientProcedureReadUsingCharacteristicUuidIndication_Error_gGattMultipleAttributesOverflow_c:
                    shell_write(gGattMultipleAttributesOverflow_c);
                    break;

                case GATTClientProcedureReadUsingCharacteristicUuidIndication_Error_gGattUnexpectedReadMultipleResponseLength_c:
                    shell_write(gGattUnexpectedReadMultipleResponseLength_c);
                    break;

                case GATTClientProcedureReadUsingCharacteristicUuidIndication_Error_gGattInvalidValueLength_c:
                    shell_write(gGattInvalidValueLength_c);
                    break;

                case GATTClientProcedureReadUsingCharacteristicUuidIndication_Error_gGattServerTimeout_c:
                    shell_write(gGattServerTimeout_c);
                    break;

                case GATTClientProcedureReadUsingCharacteristicUuidIndication_Error_gGattIndicationAlreadyInProgress_c:
                    shell_write(gGattIndicationAlreadyInProgress_c);
                    break;

                case GATTClientProcedureReadUsingCharacteristicUuidIndication_Error_gGattClientConfirmationTimeout_c:
                    shell_write(gGattClientConfirmationTimeout_c);
                    break;

                case GATTClientProcedureReadUsingCharacteristicUuidIndication_Error_gGapAdvDataTooLong_c:
                    shell_write(gGapAdvDataTooLong_c);
                    break;

                case GATTClientProcedureReadUsingCharacteristicUuidIndication_Error_gGapScanRspDataTooLong_c:
                    shell_write(gGapScanRspDataTooLong_c);
                    break;

                case GATTClientProcedureReadUsingCharacteristicUuidIndication_Error_gGapDeviceNotBonded_c:
                    shell_write(gGapDeviceNotBonded_c);
                    break;

                case GATTClientProcedureReadUsingCharacteristicUuidIndication_Error_gDevDbCccdLimitReached_c:
                    shell_write(gDevDbCccdLimitReached_c);
                    break;

                case GATTClientProcedureReadUsingCharacteristicUuidIndication_Error_gDevDbCccdNotFound_c:
                    shell_write(gDevDbCccdNotFound_c);
                    break;

                case GATTClientProcedureReadUsingCharacteristicUuidIndication_Error_gGattDbInvalidHandle_c:
                    shell_write(gGattDbInvalidHandle_c);
                    break;

                case GATTClientProcedureReadUsingCharacteristicUuidIndication_Error_gGattDbCharacteristicNotFound_c:
                    shell_write(gGattDbCharacteristicNotFound_c);
                    break;

                case GATTClientProcedureReadUsingCharacteristicUuidIndication_Error_gGattDbCccdNotFound_c:
                    shell_write(gGattDbCccdNotFound_c);
                    break;

                case GATTClientProcedureReadUsingCharacteristicUuidIndication_Error_gGattDbServiceNotFound_c:
                    shell_write(gGattDbServiceNotFound_c);
                    break;

                case GATTClientProcedureReadUsingCharacteristicUuidIndication_Error_gGattDbDescriptorNotFound_c:
                    shell_write(gGattDbDescriptorNotFound_c);
                    break;

                default:
                    shell_printf("Unrecognized status 0x%04X", container->Data.GATTClientProcedureReadUsingCharacteristicUuidIndication.Error);
                    break;
            }

            break;

        case 0x448B:
            shell_write("\r\nGATTClientProcedureReadMultipleCharacteristicValuesIndication");
            shell_write(" -> ");

            switch (container->Data.GATTClientProcedureReadMultipleCharacteristicValuesIndication.ProcedureResult)
            {
                case GATTClientProcedureReadMultipleCharacteristicValuesIndication_ProcedureResult_gGattProcSuccess_c:
                    shell_write(gGattProcSuccess_c);
                    break;

                case GATTClientProcedureReadMultipleCharacteristicValuesIndication_ProcedureResult_gProcedureError_c:
                    shell_write(gProcedureError_c);
                    break;

                default:
                    shell_printf("Unrecognized status 0x%02X", container->Data.GATTClientProcedureReadMultipleCharacteristicValuesIndication.ProcedureResult);
                    break;
            }

            shell_write(" -> ");

            switch (container->Data.GATTClientProcedureReadMultipleCharacteristicValuesIndication.Error)
            {
                case GATTClientProcedureReadMultipleCharacteristicValuesIndication_Error_gBleSuccess_c:
                    shell_write(gBleSuccess_c);
                    break;

                case GATTClientProcedureReadMultipleCharacteristicValuesIndication_Error_gBleInvalidParameter_c:
                    shell_write(gBleInvalidParameter_c);
                    break;

                case GATTClientProcedureReadMultipleCharacteristicValuesIndication_Error_gBleOverflow_c:
                    shell_write(gBleOverflow_c);
                    break;

                case GATTClientProcedureReadMultipleCharacteristicValuesIndication_Error_gBleUnavailable_c:
                    shell_write(gBleUnavailable_c);
                    break;

                case GATTClientProcedureReadMultipleCharacteristicValuesIndication_Error_gBleFeatureNotSupported_c:
                    shell_write(gBleFeatureNotSupported_c);
                    break;

                case GATTClientProcedureReadMultipleCharacteristicValuesIndication_Error_gBleOutOfMemory_c:
                    shell_write(gBleOutOfMemory_c);
                    break;

                case GATTClientProcedureReadMultipleCharacteristicValuesIndication_Error_gBleAlreadyInitialized_c:
                    shell_write(gBleAlreadyInitialized_c);
                    break;

                case GATTClientProcedureReadMultipleCharacteristicValuesIndication_Error_gBleOsError_c:
                    shell_write(gBleOsc);
                    break;

                case GATTClientProcedureReadMultipleCharacteristicValuesIndication_Error_gBleUnexpectedError_c:
                    shell_write(gBleUnexpectedc);
                    break;

                case GATTClientProcedureReadMultipleCharacteristicValuesIndication_Error_gBleInvalidState_c:
                    shell_write(gBleInvalidState_c);
                    break;

                case GATTClientProcedureReadMultipleCharacteristicValuesIndication_Error_gHciUnknownHciCommand_c:
                    shell_write(gHciUnknownHciCommand_c);
                    break;

                case GATTClientProcedureReadMultipleCharacteristicValuesIndication_Error_gHciUnknownConnectionIdentifier_c:
                    shell_write(gHciUnknownConnectionIdentifier_c);
                    break;

                case GATTClientProcedureReadMultipleCharacteristicValuesIndication_Error_gHciHardwareFailure_c:
                    shell_write(gHciHardwareFailure_c);
                    break;

                case GATTClientProcedureReadMultipleCharacteristicValuesIndication_Error_gHciPageTimeout_c:
                    shell_write(gHciPageTimeout_c);
                    break;

                case GATTClientProcedureReadMultipleCharacteristicValuesIndication_Error_gHciAuthenticationFailure_c:
                    shell_write(gHciAuthenticationFailure_c);
                    break;

                case GATTClientProcedureReadMultipleCharacteristicValuesIndication_Error_gHciPinOrKeyMissing_c:
                    shell_write(gHciPinOrKeyMissing_c);
                    break;

                case GATTClientProcedureReadMultipleCharacteristicValuesIndication_Error_gHciMemoryCapacityExceeded_c:
                    shell_write(gHciMemoryCapacityExceeded_c);
                    break;

                case GATTClientProcedureReadMultipleCharacteristicValuesIndication_Error_gHciConnectionTimeout_c:
                    shell_write(gHciConnectionTimeout_c);
                    break;

                case GATTClientProcedureReadMultipleCharacteristicValuesIndication_Error_gHciConnectionLimitExceeded_c:
                    shell_write(gHciConnectionLimitExceeded_c);
                    break;

                case GATTClientProcedureReadMultipleCharacteristicValuesIndication_Error_gHciSynchronousConnectionLimitToADeviceExceeded_c:
                    shell_write(gHciSynchronousConnectionLimitToADeviceExceeded_c);
                    break;

                case GATTClientProcedureReadMultipleCharacteristicValuesIndication_Error_gHciAclConnectionAlreadyExists_c:
                    shell_write(gHciAclConnectionAlreadyExists_c);
                    break;

                case GATTClientProcedureReadMultipleCharacteristicValuesIndication_Error_gHciCommandDisallowed_c:
                    shell_write(gHciCommandDisallowed_c);
                    break;

                case GATTClientProcedureReadMultipleCharacteristicValuesIndication_Error_gHciConnectionRejectedDueToLimitedResources_c:
                    shell_write(gHciConnectionRejectedDueToLimitedResources_c);
                    break;

                case GATTClientProcedureReadMultipleCharacteristicValuesIndication_Error_gHciConnectionRejectedDueToSecurityReasons_c:
                    shell_write(gHciConnectionRejectedDueToSecurityReasons_c);
                    break;

                case GATTClientProcedureReadMultipleCharacteristicValuesIndication_Error_gHciConnectionRejectedDueToUnacceptableBdAddr_c:
                    shell_write(gHciConnectionRejectedDueToUnacceptableBdAddr_c);
                    break;

                case GATTClientProcedureReadMultipleCharacteristicValuesIndication_Error_gHciConnectionAcceptTimeoutExceeded_c:
                    shell_write(gHciConnectionAcceptTimeoutExceeded_c);
                    break;

                case GATTClientProcedureReadMultipleCharacteristicValuesIndication_Error_gHciUnsupportedFeatureOrParameterValue_c:
                    shell_write(gHciUnsupportedFeatureOrParameterValue_c);
                    break;

                case GATTClientProcedureReadMultipleCharacteristicValuesIndication_Error_gHciInvalidHciCommandParameters_c:
                    shell_write(gHciInvalidHciCommandParameters_c);
                    break;

                case GATTClientProcedureReadMultipleCharacteristicValuesIndication_Error_gHciRemoteUserTerminatedConnection_c:
                    shell_write(gHciRemoteUserTerminatedConnection_c);
                    break;

                case GATTClientProcedureReadMultipleCharacteristicValuesIndication_Error_gHciRemoteDeviceTerminatedConnectionLowResources_c:
                    shell_write(gHciRemoteDeviceTerminatedConnectionLowResources_c);
                    break;

                case GATTClientProcedureReadMultipleCharacteristicValuesIndication_Error_gHciRemoteDeviceTerminatedConnectionPowerOff_c:
                    shell_write(gHciRemoteDeviceTerminatedConnectionPowerOff_c);
                    break;

                case GATTClientProcedureReadMultipleCharacteristicValuesIndication_Error_gHciConnectionTerminatedByLocalHost_c:
                    shell_write(gHciConnectionTerminatedByLocalHost_c);
                    break;

                case GATTClientProcedureReadMultipleCharacteristicValuesIndication_Error_gHciRepeatedAttempts_c:
                    shell_write(gHciRepeatedAttempts_c);
                    break;

                case GATTClientProcedureReadMultipleCharacteristicValuesIndication_Error_gHciPairingNotAllowed_c:
                    shell_write(gHciPairingNotAllowed_c);
                    break;

                case GATTClientProcedureReadMultipleCharacteristicValuesIndication_Error_gHciUnknownLpmPdu_c:
                    shell_write(gHciUnknownLpmPdu_c);
                    break;

                case GATTClientProcedureReadMultipleCharacteristicValuesIndication_Error_gHciUnsupportedRemoteFeature_c:
                    shell_write(gHciUnsupportedRemoteFeature_c);
                    break;

                case GATTClientProcedureReadMultipleCharacteristicValuesIndication_Error_gHciScoOffsetRejected_c:
                    shell_write(gHciScoOffsetRejected_c);
                    break;

                case GATTClientProcedureReadMultipleCharacteristicValuesIndication_Error_gHciScoIntervalRejected_c:
                    shell_write(gHciScoIntervalRejected_c);
                    break;

                case GATTClientProcedureReadMultipleCharacteristicValuesIndication_Error_gHciScoAirModeRejected_c:
                    shell_write(gHciScoAirModeRejected_c);
                    break;

                case GATTClientProcedureReadMultipleCharacteristicValuesIndication_Error_gHciInvalidLpmParameters_c:
                    shell_write(gHciInvalidLpmParameters_c);
                    break;

                case GATTClientProcedureReadMultipleCharacteristicValuesIndication_Error_gHciUnspecifiedError_c:
                    shell_write(gHciUnspecifiedc);
                    break;

                case GATTClientProcedureReadMultipleCharacteristicValuesIndication_Error_gHciUnsupportedLpmParameterValue_c:
                    shell_write(gHciUnsupportedLpmParameterValue_c);
                    break;

                case GATTClientProcedureReadMultipleCharacteristicValuesIndication_Error_gHciRoleChangeNotAllowed_c:
                    shell_write(gHciRoleChangeNotAllowed_c);
                    break;

                case GATTClientProcedureReadMultipleCharacteristicValuesIndication_Error_gHciLLResponseTimeout_c:
                    shell_write(gHciLLResponseTimeout_c);
                    break;

                case GATTClientProcedureReadMultipleCharacteristicValuesIndication_Error_gHciLmpErrorTransactionCollision_c:
                    shell_write(gHciLmpErrorTransactionCollision_c);
                    break;

                case GATTClientProcedureReadMultipleCharacteristicValuesIndication_Error_gHciLmpPduNotAllowed_c:
                    shell_write(gHciLmpPduNotAllowed_c);
                    break;

                case GATTClientProcedureReadMultipleCharacteristicValuesIndication_Error_gHciEncryptionModeNotAcceptable_c:
                    shell_write(gHciEncryptionModeNotAcceptable_c);
                    break;

                case GATTClientProcedureReadMultipleCharacteristicValuesIndication_Error_gHciLinkKeyCannotBeChanged_c:
                    shell_write(gHciLinkKeyCannotBeChanged_c);
                    break;

                case GATTClientProcedureReadMultipleCharacteristicValuesIndication_Error_gHciRequestedQosNotSupported_c:
                    shell_write(gHciRequestedQosNotSupported_c);
                    break;

                case GATTClientProcedureReadMultipleCharacteristicValuesIndication_Error_gHciInstantPassed_c:
                    shell_write(gHciInstantPassed_c);
                    break;

                case GATTClientProcedureReadMultipleCharacteristicValuesIndication_Error_gHciPairingWithUnitKeyNotSupported_c:
                    shell_write(gHciPairingWithUnitKeyNotSupported_c);
                    break;

                case GATTClientProcedureReadMultipleCharacteristicValuesIndication_Error_gHciDifferentTransactionCollision_c:
                    shell_write(gHciDifferentTransactionCollision_c);
                    break;

                case GATTClientProcedureReadMultipleCharacteristicValuesIndication_Error_gHciReserved_0x2B_c:
                    shell_write(gHciReserved_0x2B_c);
                    break;

                case GATTClientProcedureReadMultipleCharacteristicValuesIndication_Error_gHciQosNotAcceptableParameter_c:
                    shell_write(gHciQosNotAcceptableParameter_c);
                    break;

                case GATTClientProcedureReadMultipleCharacteristicValuesIndication_Error_gHciQosRejected_c:
                    shell_write(gHciQosRejected_c);
                    break;

                case GATTClientProcedureReadMultipleCharacteristicValuesIndication_Error_gHciChannelClassificationNotSupported_c:
                    shell_write(gHciChannelClassificationNotSupported_c);
                    break;

                case GATTClientProcedureReadMultipleCharacteristicValuesIndication_Error_gHciInsufficientSecurity_c:
                    shell_write(gHciInsufficientSecurity_c);
                    break;

                case GATTClientProcedureReadMultipleCharacteristicValuesIndication_Error_gHciParameterOutOfMandatoryRange_c:
                    shell_write(gHciParameterOutOfMandatoryRange_c);
                    break;

                case GATTClientProcedureReadMultipleCharacteristicValuesIndication_Error_gHciReserved_0x31_c:
                    shell_write(gHciReserved_0x31_c);
                    break;

                case GATTClientProcedureReadMultipleCharacteristicValuesIndication_Error_gHciRoleSwitchPending_c:
                    shell_write(gHciRoleSwitchPending_c);
                    break;

                case GATTClientProcedureReadMultipleCharacteristicValuesIndication_Error_gHciReserved_0x33_c:
                    shell_write(gHciReserved_0x33_c);
                    break;

                case GATTClientProcedureReadMultipleCharacteristicValuesIndication_Error_gHciReservedSlotViolation_c:
                    shell_write(gHciReservedSlotViolation_c);
                    break;

                case GATTClientProcedureReadMultipleCharacteristicValuesIndication_Error_gHciRoleSwitchFailed_c:
                    shell_write(gHciRoleSwitchFailed_c);
                    break;

                case GATTClientProcedureReadMultipleCharacteristicValuesIndication_Error_gHciExtendedInquiryResponseTooLarge_c:
                    shell_write(gHciExtendedInquiryResponseTooLarge_c);
                    break;

                case GATTClientProcedureReadMultipleCharacteristicValuesIndication_Error_gHciSecureSimplePairingNotSupportedByHost_c:
                    shell_write(gHciSecureSimplePairingNotSupportedByHost_c);
                    break;

                case GATTClientProcedureReadMultipleCharacteristicValuesIndication_Error_gHciHostBusyPairing_c:
                    shell_write(gHciHostBusyPairing_c);
                    break;

                case GATTClientProcedureReadMultipleCharacteristicValuesIndication_Error_gHciConnectionRejectedDueToNoSuitableChannelFound_c:
                    shell_write(gHciConnectionRejectedDueToNoSuitableChannelFound_c);
                    break;

                case GATTClientProcedureReadMultipleCharacteristicValuesIndication_Error_gHciControllerBusy_c:
                    shell_write(gHciControllerBusy_c);
                    break;

                case GATTClientProcedureReadMultipleCharacteristicValuesIndication_Error_gHciUnacceptableConnectionParameters_c:
                    shell_write(gHciUnacceptableConnectionParameters_c);
                    break;

                case GATTClientProcedureReadMultipleCharacteristicValuesIndication_Error_gHciDirectedAdvertisingTimeout_c:
                    shell_write(gHciDirectedAdvertisingTimeout_c);
                    break;

                case GATTClientProcedureReadMultipleCharacteristicValuesIndication_Error_gHciConnectionTerminatedDueToMicFailure_c:
                    shell_write(gHciConnectionTerminatedDueToMicFailure_c);
                    break;

                case GATTClientProcedureReadMultipleCharacteristicValuesIndication_Error_gHciConnectionFailedToBeEstablished_c:
                    shell_write(gHciConnectionFailedToBeEstablished_c);
                    break;

                case GATTClientProcedureReadMultipleCharacteristicValuesIndication_Error_gHciMacConnectionFailed_c:
                    shell_write(gHciMacConnectionFailed_c);
                    break;

                case GATTClientProcedureReadMultipleCharacteristicValuesIndication_Error_gHciCoarseClockAdjustmentRejected_c:
                    shell_write(gHciCoarseClockAdjustmentRejected_c);
                    break;

                case GATTClientProcedureReadMultipleCharacteristicValuesIndication_Error_gHciAlreadyInit_c:
                    shell_write(gHciAlreadyInit_c);
                    break;

                case GATTClientProcedureReadMultipleCharacteristicValuesIndication_Error_gHciInvalidParameter_c:
                    shell_write(gHciInvalidParameter_c);
                    break;

                case GATTClientProcedureReadMultipleCharacteristicValuesIndication_Error_gHciCallbackNotInstalled_c:
                    shell_write(gHciCallbackNotInstalled_c);
                    break;

                case GATTClientProcedureReadMultipleCharacteristicValuesIndication_Error_gHciCallbackAlreadyInstalled_c:
                    shell_write(gHciCallbackAlreadyInstalled_c);
                    break;

                case GATTClientProcedureReadMultipleCharacteristicValuesIndication_Error_gHciCommandNotSupported_c:
                    shell_write(gHciCommandNotSupported_c);
                    break;

                case GATTClientProcedureReadMultipleCharacteristicValuesIndication_Error_gHciEventNotSupported_c:
                    shell_write(gHciEventNotSupported_c);
                    break;

                case GATTClientProcedureReadMultipleCharacteristicValuesIndication_Error_gHciTransportError_c:
                    shell_write(gHciTransportc);
                    break;

                case GATTClientProcedureReadMultipleCharacteristicValuesIndication_Error_gL2caAlreadyInit_c:
                    shell_write(gL2caAlreadyInit_c);
                    break;

                case GATTClientProcedureReadMultipleCharacteristicValuesIndication_Error_gL2caInsufficientResources_c:
                    shell_write(gL2caInsufficientResources_c);
                    break;

                case GATTClientProcedureReadMultipleCharacteristicValuesIndication_Error_gL2caCallbackNotInstalled_c:
                    shell_write(gL2caCallbackNotInstalled_c);
                    break;

                case GATTClientProcedureReadMultipleCharacteristicValuesIndication_Error_gL2caCallbackAlreadyInstalled_c:
                    shell_write(gL2caCallbackAlreadyInstalled_c);
                    break;

                case GATTClientProcedureReadMultipleCharacteristicValuesIndication_Error_gL2caLePsmInvalid_c:
                    shell_write(gL2caLePsmInvalid_c);
                    break;

                case GATTClientProcedureReadMultipleCharacteristicValuesIndication_Error_gL2caLePsmAlreadyRegistered_c:
                    shell_write(gL2caLePsmAlreadyRegistered_c);
                    break;

                case GATTClientProcedureReadMultipleCharacteristicValuesIndication_Error_gL2caLePsmNotRegistered_c:
                    shell_write(gL2caLePsmNotRegistered_c);
                    break;

                case GATTClientProcedureReadMultipleCharacteristicValuesIndication_Error_gL2caLePsmInsufficientResources_c:
                    shell_write(gL2caLePsmInsufficientResources_c);
                    break;

                case GATTClientProcedureReadMultipleCharacteristicValuesIndication_Error_gL2caChannelInvalid_c:
                    shell_write(gL2caChannelInvalid_c);
                    break;

                case GATTClientProcedureReadMultipleCharacteristicValuesIndication_Error_gL2caChannelClosed_c:
                    shell_write(gL2caChannelClosed_c);
                    break;

                case GATTClientProcedureReadMultipleCharacteristicValuesIndication_Error_gL2caChannelAlreadyConnected_c:
                    shell_write(gL2caChannelAlreadyConnected_c);
                    break;

                case GATTClientProcedureReadMultipleCharacteristicValuesIndication_Error_gL2caConnectionParametersRejected_c:
                    shell_write(gL2caConnectionParametersRejected_c);
                    break;

                case GATTClientProcedureReadMultipleCharacteristicValuesIndication_Error_gL2caChannelBusy_c:
                    shell_write(gL2caChannelBusy_c);
                    break;

                case GATTClientProcedureReadMultipleCharacteristicValuesIndication_Error_gL2caInvalidParameter_c:
                    shell_write(gL2caInvalidParameter_c);
                    break;

                case GATTClientProcedureReadMultipleCharacteristicValuesIndication_Error_gL2caError_c:
                    shell_write(gL2cac);
                    break;

                case GATTClientProcedureReadMultipleCharacteristicValuesIndication_Error_gSmNullCBFunction_c:
                    shell_write(gSmNullCBFunction_c);
                    break;

                case GATTClientProcedureReadMultipleCharacteristicValuesIndication_Error_gSmCommandNotSupported_c:
                    shell_write(gSmCommandNotSupported_c);
                    break;

                case GATTClientProcedureReadMultipleCharacteristicValuesIndication_Error_gSmUnexpectedCommand_c:
                    shell_write(gSmUnexpectedCommand_c);
                    break;

                case GATTClientProcedureReadMultipleCharacteristicValuesIndication_Error_gSmInvalidCommandCode_c:
                    shell_write(gSmInvalidCommandCode_c);
                    break;

                case GATTClientProcedureReadMultipleCharacteristicValuesIndication_Error_gSmInvalidCommandLength_c:
                    shell_write(gSmInvalidCommandLength_c);
                    break;

                case GATTClientProcedureReadMultipleCharacteristicValuesIndication_Error_gSmInvalidCommandParameter_c:
                    shell_write(gSmInvalidCommandParameter_c);
                    break;

                case GATTClientProcedureReadMultipleCharacteristicValuesIndication_Error_gSmInvalidDeviceId_c:
                    shell_write(gSmInvalidDeviceId_c);
                    break;

                case GATTClientProcedureReadMultipleCharacteristicValuesIndication_Error_gSmInvalidInternalOperation_c:
                    shell_write(gSmInvalidInternalOperation_c);
                    break;

                case GATTClientProcedureReadMultipleCharacteristicValuesIndication_Error_gSmInvalidConnectionHandle_c:
                    shell_write(gSmInvalidConnectionHandle_c);
                    break;

                case GATTClientProcedureReadMultipleCharacteristicValuesIndication_Error_gSmInproperKeyDistributionField_c:
                    shell_write(gSmInproperKeyDistributionField_c);
                    break;

                case GATTClientProcedureReadMultipleCharacteristicValuesIndication_Error_gSmUnexpectedKeyType_c:
                    shell_write(gSmUnexpectedKeyType_c);
                    break;

                case GATTClientProcedureReadMultipleCharacteristicValuesIndication_Error_gSmUnexpectedPairingTerminationReason_c:
                    shell_write(gSmUnexpectedPairingTerminationReason_c);
                    break;

                case GATTClientProcedureReadMultipleCharacteristicValuesIndication_Error_gSmUnexpectedKeyset_c:
                    shell_write(gSmUnexpectedKeyset_c);
                    break;

                case GATTClientProcedureReadMultipleCharacteristicValuesIndication_Error_gSmSmpTimeoutOccurred_c:
                    shell_write(gSmSmpTimeoutOccurred_c);
                    break;

                case GATTClientProcedureReadMultipleCharacteristicValuesIndication_Error_gSmUnknownSmpPacketType_c:
                    shell_write(gSmUnknownSmpPacketType_c);
                    break;

                case GATTClientProcedureReadMultipleCharacteristicValuesIndication_Error_gSmInvalidSmpPacketLength_c:
                    shell_write(gSmInvalidSmpPacketLength_c);
                    break;

                case GATTClientProcedureReadMultipleCharacteristicValuesIndication_Error_gSmInvalidSmpPacketParameter_c:
                    shell_write(gSmInvalidSmpPacketParameter_c);
                    break;

                case GATTClientProcedureReadMultipleCharacteristicValuesIndication_Error_gSmReceivedUnexpectedSmpPacket_c:
                    shell_write(gSmReceivedUnexpectedSmpPacket_c);
                    break;

                case GATTClientProcedureReadMultipleCharacteristicValuesIndication_Error_gSmReceivedSmpPacketFromUnknownDevice_c:
                    shell_write(gSmReceivedSmpPacketFromUnknownDevice_c);
                    break;

                case GATTClientProcedureReadMultipleCharacteristicValuesIndication_Error_gSmReceivedUnexpectedHciEvent_c:
                    shell_write(gSmReceivedUnexpectedHciEvent_c);
                    break;

                case GATTClientProcedureReadMultipleCharacteristicValuesIndication_Error_gSmReceivedHciEventFromUnknownDevice_c:
                    shell_write(gSmReceivedHciEventFromUnknownDevice_c);
                    break;

                case GATTClientProcedureReadMultipleCharacteristicValuesIndication_Error_gSmInvalidHciEventParameter_c:
                    shell_write(gSmInvalidHciEventParameter_c);
                    break;

                case GATTClientProcedureReadMultipleCharacteristicValuesIndication_Error_gSmLlConnectionEncryptionInProgress_c:
                    shell_write(gSmLlConnectionEncryptionInProgress_c);
                    break;

                case GATTClientProcedureReadMultipleCharacteristicValuesIndication_Error_gSmLlConnectionEncryptionFailure_c:
                    shell_write(gSmLlConnectionEncryptionFailure_c);
                    break;

                case GATTClientProcedureReadMultipleCharacteristicValuesIndication_Error_gSmInsufficientResources_c:
                    shell_write(gSmInsufficientResources_c);
                    break;

                case GATTClientProcedureReadMultipleCharacteristicValuesIndication_Error_gSmOobDataAddressMismatch_c:
                    shell_write(gSmOobDataAddressMismatch_c);
                    break;

//                case GATTClientProcedureReadMultipleCharacteristicValuesIndication_Error_gSmSmpPacketReceivedAfterTimeoutOccurred_c:
//                    shell_write(gSmSmpPacketReceivedAfterTimeoutOccurred_c);
//                    break;

                case GATTClientProcedureReadMultipleCharacteristicValuesIndication_Error_gSmPairingErrorPasskeyEntryFailed_c:
                    shell_write(gSmPairingErrorPasskeyEntryFailed_c);
                    break;

                case GATTClientProcedureReadMultipleCharacteristicValuesIndication_Error_gSmPairingErrorConfirmValueFailed_c:
                    shell_write(gSmPairingErrorConfirmValueFailed_c);
                    break;

                case GATTClientProcedureReadMultipleCharacteristicValuesIndication_Error_gSmPairingErrorCommandNotSupported_c:
                    shell_write(gSmPairingErrorCommandNotSupported_c);
                    break;

                case GATTClientProcedureReadMultipleCharacteristicValuesIndication_Error_gSmPairingErrorInvalidParameters_c:
                    shell_write(gSmPairingErrorInvalidParameters_c);
                    break;

                case GATTClientProcedureReadMultipleCharacteristicValuesIndication_Error_gSmPairingErrorUnknownReason_c:
                    shell_write(gSmPairingErrorUnknownReason_c);
                    break;

                case GATTClientProcedureReadMultipleCharacteristicValuesIndication_Error_gSmTbResolvableAddressDoesNotMatchIrk_c:
                    shell_write(gSmTbResolvableAddressDoesNotMatchIrk_c);
                    break;

                case GATTClientProcedureReadMultipleCharacteristicValuesIndication_Error_gSmTbInvalidDataSignature_c:
                    shell_write(gSmTbInvalidDataSignature_c);
                    break;

                case GATTClientProcedureReadMultipleCharacteristicValuesIndication_Error_gAttInvalidHandle_c:
                    shell_write(gAttInvalidHandle_c);
                    break;

                case GATTClientProcedureReadMultipleCharacteristicValuesIndication_Error_gAttReadNotPermitted_c:
                    shell_write(gAttReadNotPermitted_c);
                    break;

                case GATTClientProcedureReadMultipleCharacteristicValuesIndication_Error_gAttWriteNotPermitted_c:
                    shell_write(gAttWriteNotPermitted_c);
                    break;

                case GATTClientProcedureReadMultipleCharacteristicValuesIndication_Error_gAttInvalidPdu_c:
                    shell_write(gAttInvalidPdu_c);
                    break;

                case GATTClientProcedureReadMultipleCharacteristicValuesIndication_Error_gAttInsufficientAuthentication_c:
                    shell_write(gAttInsufficientAuthentication_c);
                    break;

                case GATTClientProcedureReadMultipleCharacteristicValuesIndication_Error_gAttRequestNotSupported_c:
                    shell_write(gAttRequestNotSupported_c);
                    break;

                case GATTClientProcedureReadMultipleCharacteristicValuesIndication_Error_gAttInvalidOffset_c:
                    shell_write(gAttInvalidOffset_c);
                    break;

                case GATTClientProcedureReadMultipleCharacteristicValuesIndication_Error_gAttInsufficientAuthorization_c:
                    shell_write(gAttInsufficientAuthorization_c);
                    break;

                case GATTClientProcedureReadMultipleCharacteristicValuesIndication_Error_gAttPrepareQueueFull_c:
                    shell_write(gAttPrepareQueueFull_c);
                    break;

                case GATTClientProcedureReadMultipleCharacteristicValuesIndication_Error_gAttAttributeNotFound_c:
                    shell_write(gAttAttributeNotFound_c);
                    break;

                case GATTClientProcedureReadMultipleCharacteristicValuesIndication_Error_gAttAttributeNotLong_c:
                    shell_write(gAttAttributeNotLong_c);
                    break;

                case GATTClientProcedureReadMultipleCharacteristicValuesIndication_Error_gAttInsufficientEncryptionKeySize_c:
                    shell_write(gAttInsufficientEncryptionKeySize_c);
                    break;

                case GATTClientProcedureReadMultipleCharacteristicValuesIndication_Error_gAttInvalidAttributeValueLength_c:
                    shell_write(gAttInvalidAttributeValueLength_c);
                    break;

                case GATTClientProcedureReadMultipleCharacteristicValuesIndication_Error_gAttUnlikelyor_c:
                    shell_write(gAttUnlikelyor_c);
                    break;

                case GATTClientProcedureReadMultipleCharacteristicValuesIndication_Error_gAttInsufficientEncryption_c:
                    shell_write(gAttInsufficientEncryption_c);
                    break;

                case GATTClientProcedureReadMultipleCharacteristicValuesIndication_Error_gAttUnsupportedGroupType_c:
                    shell_write(gAttUnsupportedGroupType_c);
                    break;

                case GATTClientProcedureReadMultipleCharacteristicValuesIndication_Error_gAttInsufficientResources_c:
                    shell_write(gAttInsufficientResources_c);
                    break;

                case GATTClientProcedureReadMultipleCharacteristicValuesIndication_Error_gGattAnotherProcedureInProgress_c:
                    shell_write(gGattAnotherProcedureInProgress_c);
                    break;

                case GATTClientProcedureReadMultipleCharacteristicValuesIndication_Error_gGattLongAttributePacketsCorrupted_c:
                    shell_write(gGattLongAttributePacketsCorrupted_c);
                    break;

                case GATTClientProcedureReadMultipleCharacteristicValuesIndication_Error_gGattMultipleAttributesOverflow_c:
                    shell_write(gGattMultipleAttributesOverflow_c);
                    break;

                case GATTClientProcedureReadMultipleCharacteristicValuesIndication_Error_gGattUnexpectedReadMultipleResponseLength_c:
                    shell_write(gGattUnexpectedReadMultipleResponseLength_c);
                    break;

                case GATTClientProcedureReadMultipleCharacteristicValuesIndication_Error_gGattInvalidValueLength_c:
                    shell_write(gGattInvalidValueLength_c);
                    break;

                case GATTClientProcedureReadMultipleCharacteristicValuesIndication_Error_gGattServerTimeout_c:
                    shell_write(gGattServerTimeout_c);
                    break;

                case GATTClientProcedureReadMultipleCharacteristicValuesIndication_Error_gGattIndicationAlreadyInProgress_c:
                    shell_write(gGattIndicationAlreadyInProgress_c);
                    break;

                case GATTClientProcedureReadMultipleCharacteristicValuesIndication_Error_gGattClientConfirmationTimeout_c:
                    shell_write(gGattClientConfirmationTimeout_c);
                    break;

                case GATTClientProcedureReadMultipleCharacteristicValuesIndication_Error_gGapAdvDataTooLong_c:
                    shell_write(gGapAdvDataTooLong_c);
                    break;

                case GATTClientProcedureReadMultipleCharacteristicValuesIndication_Error_gGapScanRspDataTooLong_c:
                    shell_write(gGapScanRspDataTooLong_c);
                    break;

                case GATTClientProcedureReadMultipleCharacteristicValuesIndication_Error_gGapDeviceNotBonded_c:
                    shell_write(gGapDeviceNotBonded_c);
                    break;

                case GATTClientProcedureReadMultipleCharacteristicValuesIndication_Error_gDevDbCccdLimitReached_c:
                    shell_write(gDevDbCccdLimitReached_c);
                    break;

                case GATTClientProcedureReadMultipleCharacteristicValuesIndication_Error_gDevDbCccdNotFound_c:
                    shell_write(gDevDbCccdNotFound_c);
                    break;

                case GATTClientProcedureReadMultipleCharacteristicValuesIndication_Error_gGattDbInvalidHandle_c:
                    shell_write(gGattDbInvalidHandle_c);
                    break;

                case GATTClientProcedureReadMultipleCharacteristicValuesIndication_Error_gGattDbCharacteristicNotFound_c:
                    shell_write(gGattDbCharacteristicNotFound_c);
                    break;

                case GATTClientProcedureReadMultipleCharacteristicValuesIndication_Error_gGattDbCccdNotFound_c:
                    shell_write(gGattDbCccdNotFound_c);
                    break;

                case GATTClientProcedureReadMultipleCharacteristicValuesIndication_Error_gGattDbServiceNotFound_c:
                    shell_write(gGattDbServiceNotFound_c);
                    break;

                case GATTClientProcedureReadMultipleCharacteristicValuesIndication_Error_gGattDbDescriptorNotFound_c:
                    shell_write(gGattDbDescriptorNotFound_c);
                    break;

                default:
                    shell_printf("Unrecognized status 0x%04X", container->Data.GATTClientProcedureReadMultipleCharacteristicValuesIndication.Error);
                    break;
            }

            break;

        case 0x448C:
            shell_write("\r\nGATTClientProcedureWriteCharacteristicValueIndication");
            shell_write(" -> ");

            switch (container->Data.GATTClientProcedureWriteCharacteristicValueIndication.ProcedureResult)
            {
                case GATTClientProcedureWriteCharacteristicValueIndication_ProcedureResult_gGattProcSuccess_c:
                    shell_write(gGattProcSuccess_c);
                    break;

                case GATTClientProcedureWriteCharacteristicValueIndication_ProcedureResult_gProcedureError_c:
                    shell_write(gProcedureError_c);
                    break;

                default:
                    shell_printf("Unrecognized status 0x%02X", container->Data.GATTClientProcedureWriteCharacteristicValueIndication.ProcedureResult);
                    break;
            }

            shell_write(" -> ");

            switch (container->Data.GATTClientProcedureWriteCharacteristicValueIndication.Error)
            {
                case GATTClientProcedureWriteCharacteristicValueIndication_Error_gBleSuccess_c:
                    shell_write(gBleSuccess_c);
                    break;

                case GATTClientProcedureWriteCharacteristicValueIndication_Error_gBleInvalidParameter_c:
                    shell_write(gBleInvalidParameter_c);
                    break;

                case GATTClientProcedureWriteCharacteristicValueIndication_Error_gBleOverflow_c:
                    shell_write(gBleOverflow_c);
                    break;

                case GATTClientProcedureWriteCharacteristicValueIndication_Error_gBleUnavailable_c:
                    shell_write(gBleUnavailable_c);
                    break;

                case GATTClientProcedureWriteCharacteristicValueIndication_Error_gBleFeatureNotSupported_c:
                    shell_write(gBleFeatureNotSupported_c);
                    break;

                case GATTClientProcedureWriteCharacteristicValueIndication_Error_gBleOutOfMemory_c:
                    shell_write(gBleOutOfMemory_c);
                    break;

                case GATTClientProcedureWriteCharacteristicValueIndication_Error_gBleAlreadyInitialized_c:
                    shell_write(gBleAlreadyInitialized_c);
                    break;

                case GATTClientProcedureWriteCharacteristicValueIndication_Error_gBleOsError_c:
                    shell_write(gBleOsc);
                    break;

                case GATTClientProcedureWriteCharacteristicValueIndication_Error_gBleUnexpectedError_c:
                    shell_write(gBleUnexpectedc);
                    break;

                case GATTClientProcedureWriteCharacteristicValueIndication_Error_gBleInvalidState_c:
                    shell_write(gBleInvalidState_c);
                    break;

                case GATTClientProcedureWriteCharacteristicValueIndication_Error_gHciUnknownHciCommand_c:
                    shell_write(gHciUnknownHciCommand_c);
                    break;

                case GATTClientProcedureWriteCharacteristicValueIndication_Error_gHciUnknownConnectionIdentifier_c:
                    shell_write(gHciUnknownConnectionIdentifier_c);
                    break;

                case GATTClientProcedureWriteCharacteristicValueIndication_Error_gHciHardwareFailure_c:
                    shell_write(gHciHardwareFailure_c);
                    break;

                case GATTClientProcedureWriteCharacteristicValueIndication_Error_gHciPageTimeout_c:
                    shell_write(gHciPageTimeout_c);
                    break;

                case GATTClientProcedureWriteCharacteristicValueIndication_Error_gHciAuthenticationFailure_c:
                    shell_write(gHciAuthenticationFailure_c);
                    break;

                case GATTClientProcedureWriteCharacteristicValueIndication_Error_gHciPinOrKeyMissing_c:
                    shell_write(gHciPinOrKeyMissing_c);
                    break;

                case GATTClientProcedureWriteCharacteristicValueIndication_Error_gHciMemoryCapacityExceeded_c:
                    shell_write(gHciMemoryCapacityExceeded_c);
                    break;

                case GATTClientProcedureWriteCharacteristicValueIndication_Error_gHciConnectionTimeout_c:
                    shell_write(gHciConnectionTimeout_c);
                    break;

                case GATTClientProcedureWriteCharacteristicValueIndication_Error_gHciConnectionLimitExceeded_c:
                    shell_write(gHciConnectionLimitExceeded_c);
                    break;

                case GATTClientProcedureWriteCharacteristicValueIndication_Error_gHciSynchronousConnectionLimitToADeviceExceeded_c:
                    shell_write(gHciSynchronousConnectionLimitToADeviceExceeded_c);
                    break;

                case GATTClientProcedureWriteCharacteristicValueIndication_Error_gHciAclConnectionAlreadyExists_c:
                    shell_write(gHciAclConnectionAlreadyExists_c);
                    break;

                case GATTClientProcedureWriteCharacteristicValueIndication_Error_gHciCommandDisallowed_c:
                    shell_write(gHciCommandDisallowed_c);
                    break;

                case GATTClientProcedureWriteCharacteristicValueIndication_Error_gHciConnectionRejectedDueToLimitedResources_c:
                    shell_write(gHciConnectionRejectedDueToLimitedResources_c);
                    break;

                case GATTClientProcedureWriteCharacteristicValueIndication_Error_gHciConnectionRejectedDueToSecurityReasons_c:
                    shell_write(gHciConnectionRejectedDueToSecurityReasons_c);
                    break;

                case GATTClientProcedureWriteCharacteristicValueIndication_Error_gHciConnectionRejectedDueToUnacceptableBdAddr_c:
                    shell_write(gHciConnectionRejectedDueToUnacceptableBdAddr_c);
                    break;

                case GATTClientProcedureWriteCharacteristicValueIndication_Error_gHciConnectionAcceptTimeoutExceeded_c:
                    shell_write(gHciConnectionAcceptTimeoutExceeded_c);
                    break;

                case GATTClientProcedureWriteCharacteristicValueIndication_Error_gHciUnsupportedFeatureOrParameterValue_c:
                    shell_write(gHciUnsupportedFeatureOrParameterValue_c);
                    break;

                case GATTClientProcedureWriteCharacteristicValueIndication_Error_gHciInvalidHciCommandParameters_c:
                    shell_write(gHciInvalidHciCommandParameters_c);
                    break;

                case GATTClientProcedureWriteCharacteristicValueIndication_Error_gHciRemoteUserTerminatedConnection_c:
                    shell_write(gHciRemoteUserTerminatedConnection_c);
                    break;

                case GATTClientProcedureWriteCharacteristicValueIndication_Error_gHciRemoteDeviceTerminatedConnectionLowResources_c:
                    shell_write(gHciRemoteDeviceTerminatedConnectionLowResources_c);
                    break;

                case GATTClientProcedureWriteCharacteristicValueIndication_Error_gHciRemoteDeviceTerminatedConnectionPowerOff_c:
                    shell_write(gHciRemoteDeviceTerminatedConnectionPowerOff_c);
                    break;

                case GATTClientProcedureWriteCharacteristicValueIndication_Error_gHciConnectionTerminatedByLocalHost_c:
                    shell_write(gHciConnectionTerminatedByLocalHost_c);
                    break;

                case GATTClientProcedureWriteCharacteristicValueIndication_Error_gHciRepeatedAttempts_c:
                    shell_write(gHciRepeatedAttempts_c);
                    break;

                case GATTClientProcedureWriteCharacteristicValueIndication_Error_gHciPairingNotAllowed_c:
                    shell_write(gHciPairingNotAllowed_c);
                    break;

                case GATTClientProcedureWriteCharacteristicValueIndication_Error_gHciUnknownLpmPdu_c:
                    shell_write(gHciUnknownLpmPdu_c);
                    break;

                case GATTClientProcedureWriteCharacteristicValueIndication_Error_gHciUnsupportedRemoteFeature_c:
                    shell_write(gHciUnsupportedRemoteFeature_c);
                    break;

                case GATTClientProcedureWriteCharacteristicValueIndication_Error_gHciScoOffsetRejected_c:
                    shell_write(gHciScoOffsetRejected_c);
                    break;

                case GATTClientProcedureWriteCharacteristicValueIndication_Error_gHciScoIntervalRejected_c:
                    shell_write(gHciScoIntervalRejected_c);
                    break;

                case GATTClientProcedureWriteCharacteristicValueIndication_Error_gHciScoAirModeRejected_c:
                    shell_write(gHciScoAirModeRejected_c);
                    break;

                case GATTClientProcedureWriteCharacteristicValueIndication_Error_gHciInvalidLpmParameters_c:
                    shell_write(gHciInvalidLpmParameters_c);
                    break;

                case GATTClientProcedureWriteCharacteristicValueIndication_Error_gHciUnspecifiedError_c:
                    shell_write(gHciUnspecifiedc);
                    break;

                case GATTClientProcedureWriteCharacteristicValueIndication_Error_gHciUnsupportedLpmParameterValue_c:
                    shell_write(gHciUnsupportedLpmParameterValue_c);
                    break;

                case GATTClientProcedureWriteCharacteristicValueIndication_Error_gHciRoleChangeNotAllowed_c:
                    shell_write(gHciRoleChangeNotAllowed_c);
                    break;

                case GATTClientProcedureWriteCharacteristicValueIndication_Error_gHciLLResponseTimeout_c:
                    shell_write(gHciLLResponseTimeout_c);
                    break;

                case GATTClientProcedureWriteCharacteristicValueIndication_Error_gHciLmpErrorTransactionCollision_c:
                    shell_write(gHciLmpErrorTransactionCollision_c);
                    break;

                case GATTClientProcedureWriteCharacteristicValueIndication_Error_gHciLmpPduNotAllowed_c:
                    shell_write(gHciLmpPduNotAllowed_c);
                    break;

                case GATTClientProcedureWriteCharacteristicValueIndication_Error_gHciEncryptionModeNotAcceptable_c:
                    shell_write(gHciEncryptionModeNotAcceptable_c);
                    break;

                case GATTClientProcedureWriteCharacteristicValueIndication_Error_gHciLinkKeyCannotBeChanged_c:
                    shell_write(gHciLinkKeyCannotBeChanged_c);
                    break;

                case GATTClientProcedureWriteCharacteristicValueIndication_Error_gHciRequestedQosNotSupported_c:
                    shell_write(gHciRequestedQosNotSupported_c);
                    break;

                case GATTClientProcedureWriteCharacteristicValueIndication_Error_gHciInstantPassed_c:
                    shell_write(gHciInstantPassed_c);
                    break;

                case GATTClientProcedureWriteCharacteristicValueIndication_Error_gHciPairingWithUnitKeyNotSupported_c:
                    shell_write(gHciPairingWithUnitKeyNotSupported_c);
                    break;

                case GATTClientProcedureWriteCharacteristicValueIndication_Error_gHciDifferentTransactionCollision_c:
                    shell_write(gHciDifferentTransactionCollision_c);
                    break;

                case GATTClientProcedureWriteCharacteristicValueIndication_Error_gHciReserved_0x2B_c:
                    shell_write(gHciReserved_0x2B_c);
                    break;

                case GATTClientProcedureWriteCharacteristicValueIndication_Error_gHciQosNotAcceptableParameter_c:
                    shell_write(gHciQosNotAcceptableParameter_c);
                    break;

                case GATTClientProcedureWriteCharacteristicValueIndication_Error_gHciQosRejected_c:
                    shell_write(gHciQosRejected_c);
                    break;

                case GATTClientProcedureWriteCharacteristicValueIndication_Error_gHciChannelClassificationNotSupported_c:
                    shell_write(gHciChannelClassificationNotSupported_c);
                    break;

                case GATTClientProcedureWriteCharacteristicValueIndication_Error_gHciInsufficientSecurity_c:
                    shell_write(gHciInsufficientSecurity_c);
                    break;

                case GATTClientProcedureWriteCharacteristicValueIndication_Error_gHciParameterOutOfMandatoryRange_c:
                    shell_write(gHciParameterOutOfMandatoryRange_c);
                    break;

                case GATTClientProcedureWriteCharacteristicValueIndication_Error_gHciReserved_0x31_c:
                    shell_write(gHciReserved_0x31_c);
                    break;

                case GATTClientProcedureWriteCharacteristicValueIndication_Error_gHciRoleSwitchPending_c:
                    shell_write(gHciRoleSwitchPending_c);
                    break;

                case GATTClientProcedureWriteCharacteristicValueIndication_Error_gHciReserved_0x33_c:
                    shell_write(gHciReserved_0x33_c);
                    break;

                case GATTClientProcedureWriteCharacteristicValueIndication_Error_gHciReservedSlotViolation_c:
                    shell_write(gHciReservedSlotViolation_c);
                    break;

                case GATTClientProcedureWriteCharacteristicValueIndication_Error_gHciRoleSwitchFailed_c:
                    shell_write(gHciRoleSwitchFailed_c);
                    break;

                case GATTClientProcedureWriteCharacteristicValueIndication_Error_gHciExtendedInquiryResponseTooLarge_c:
                    shell_write(gHciExtendedInquiryResponseTooLarge_c);
                    break;

                case GATTClientProcedureWriteCharacteristicValueIndication_Error_gHciSecureSimplePairingNotSupportedByHost_c:
                    shell_write(gHciSecureSimplePairingNotSupportedByHost_c);
                    break;

                case GATTClientProcedureWriteCharacteristicValueIndication_Error_gHciHostBusyPairing_c:
                    shell_write(gHciHostBusyPairing_c);
                    break;

                case GATTClientProcedureWriteCharacteristicValueIndication_Error_gHciConnectionRejectedDueToNoSuitableChannelFound_c:
                    shell_write(gHciConnectionRejectedDueToNoSuitableChannelFound_c);
                    break;

                case GATTClientProcedureWriteCharacteristicValueIndication_Error_gHciControllerBusy_c:
                    shell_write(gHciControllerBusy_c);
                    break;

                case GATTClientProcedureWriteCharacteristicValueIndication_Error_gHciUnacceptableConnectionParameters_c:
                    shell_write(gHciUnacceptableConnectionParameters_c);
                    break;

                case GATTClientProcedureWriteCharacteristicValueIndication_Error_gHciDirectedAdvertisingTimeout_c:
                    shell_write(gHciDirectedAdvertisingTimeout_c);
                    break;

                case GATTClientProcedureWriteCharacteristicValueIndication_Error_gHciConnectionTerminatedDueToMicFailure_c:
                    shell_write(gHciConnectionTerminatedDueToMicFailure_c);
                    break;

                case GATTClientProcedureWriteCharacteristicValueIndication_Error_gHciConnectionFailedToBeEstablished_c:
                    shell_write(gHciConnectionFailedToBeEstablished_c);
                    break;

                case GATTClientProcedureWriteCharacteristicValueIndication_Error_gHciMacConnectionFailed_c:
                    shell_write(gHciMacConnectionFailed_c);
                    break;

                case GATTClientProcedureWriteCharacteristicValueIndication_Error_gHciCoarseClockAdjustmentRejected_c:
                    shell_write(gHciCoarseClockAdjustmentRejected_c);
                    break;

                case GATTClientProcedureWriteCharacteristicValueIndication_Error_gHciAlreadyInit_c:
                    shell_write(gHciAlreadyInit_c);
                    break;

                case GATTClientProcedureWriteCharacteristicValueIndication_Error_gHciInvalidParameter_c:
                    shell_write(gHciInvalidParameter_c);
                    break;

                case GATTClientProcedureWriteCharacteristicValueIndication_Error_gHciCallbackNotInstalled_c:
                    shell_write(gHciCallbackNotInstalled_c);
                    break;

                case GATTClientProcedureWriteCharacteristicValueIndication_Error_gHciCallbackAlreadyInstalled_c:
                    shell_write(gHciCallbackAlreadyInstalled_c);
                    break;

                case GATTClientProcedureWriteCharacteristicValueIndication_Error_gHciCommandNotSupported_c:
                    shell_write(gHciCommandNotSupported_c);
                    break;

                case GATTClientProcedureWriteCharacteristicValueIndication_Error_gHciEventNotSupported_c:
                    shell_write(gHciEventNotSupported_c);
                    break;

                case GATTClientProcedureWriteCharacteristicValueIndication_Error_gHciTransportError_c:
                    shell_write(gHciTransportc);
                    break;

                case GATTClientProcedureWriteCharacteristicValueIndication_Error_gL2caAlreadyInit_c:
                    shell_write(gL2caAlreadyInit_c);
                    break;

                case GATTClientProcedureWriteCharacteristicValueIndication_Error_gL2caInsufficientResources_c:
                    shell_write(gL2caInsufficientResources_c);
                    break;

                case GATTClientProcedureWriteCharacteristicValueIndication_Error_gL2caCallbackNotInstalled_c:
                    shell_write(gL2caCallbackNotInstalled_c);
                    break;

                case GATTClientProcedureWriteCharacteristicValueIndication_Error_gL2caCallbackAlreadyInstalled_c:
                    shell_write(gL2caCallbackAlreadyInstalled_c);
                    break;

                case GATTClientProcedureWriteCharacteristicValueIndication_Error_gL2caLePsmInvalid_c:
                    shell_write(gL2caLePsmInvalid_c);
                    break;

                case GATTClientProcedureWriteCharacteristicValueIndication_Error_gL2caLePsmAlreadyRegistered_c:
                    shell_write(gL2caLePsmAlreadyRegistered_c);
                    break;

                case GATTClientProcedureWriteCharacteristicValueIndication_Error_gL2caLePsmNotRegistered_c:
                    shell_write(gL2caLePsmNotRegistered_c);
                    break;

                case GATTClientProcedureWriteCharacteristicValueIndication_Error_gL2caLePsmInsufficientResources_c:
                    shell_write(gL2caLePsmInsufficientResources_c);
                    break;

                case GATTClientProcedureWriteCharacteristicValueIndication_Error_gL2caChannelInvalid_c:
                    shell_write(gL2caChannelInvalid_c);
                    break;

                case GATTClientProcedureWriteCharacteristicValueIndication_Error_gL2caChannelClosed_c:
                    shell_write(gL2caChannelClosed_c);
                    break;

                case GATTClientProcedureWriteCharacteristicValueIndication_Error_gL2caChannelAlreadyConnected_c:
                    shell_write(gL2caChannelAlreadyConnected_c);
                    break;

                case GATTClientProcedureWriteCharacteristicValueIndication_Error_gL2caConnectionParametersRejected_c:
                    shell_write(gL2caConnectionParametersRejected_c);
                    break;

                case GATTClientProcedureWriteCharacteristicValueIndication_Error_gL2caChannelBusy_c:
                    shell_write(gL2caChannelBusy_c);
                    break;

                case GATTClientProcedureWriteCharacteristicValueIndication_Error_gL2caInvalidParameter_c:
                    shell_write(gL2caInvalidParameter_c);
                    break;

                case GATTClientProcedureWriteCharacteristicValueIndication_Error_gL2caError_c:
                    shell_write(gL2cac);
                    break;

                case GATTClientProcedureWriteCharacteristicValueIndication_Error_gSmNullCBFunction_c:
                    shell_write(gSmNullCBFunction_c);
                    break;

                case GATTClientProcedureWriteCharacteristicValueIndication_Error_gSmCommandNotSupported_c:
                    shell_write(gSmCommandNotSupported_c);
                    break;

                case GATTClientProcedureWriteCharacteristicValueIndication_Error_gSmUnexpectedCommand_c:
                    shell_write(gSmUnexpectedCommand_c);
                    break;

                case GATTClientProcedureWriteCharacteristicValueIndication_Error_gSmInvalidCommandCode_c:
                    shell_write(gSmInvalidCommandCode_c);
                    break;

                case GATTClientProcedureWriteCharacteristicValueIndication_Error_gSmInvalidCommandLength_c:
                    shell_write(gSmInvalidCommandLength_c);
                    break;

                case GATTClientProcedureWriteCharacteristicValueIndication_Error_gSmInvalidCommandParameter_c:
                    shell_write(gSmInvalidCommandParameter_c);
                    break;

                case GATTClientProcedureWriteCharacteristicValueIndication_Error_gSmInvalidDeviceId_c:
                    shell_write(gSmInvalidDeviceId_c);
                    break;

                case GATTClientProcedureWriteCharacteristicValueIndication_Error_gSmInvalidInternalOperation_c:
                    shell_write(gSmInvalidInternalOperation_c);
                    break;

                case GATTClientProcedureWriteCharacteristicValueIndication_Error_gSmInvalidConnectionHandle_c:
                    shell_write(gSmInvalidConnectionHandle_c);
                    break;

                case GATTClientProcedureWriteCharacteristicValueIndication_Error_gSmInproperKeyDistributionField_c:
                    shell_write(gSmInproperKeyDistributionField_c);
                    break;

                case GATTClientProcedureWriteCharacteristicValueIndication_Error_gSmUnexpectedKeyType_c:
                    shell_write(gSmUnexpectedKeyType_c);
                    break;

                case GATTClientProcedureWriteCharacteristicValueIndication_Error_gSmUnexpectedPairingTerminationReason_c:
                    shell_write(gSmUnexpectedPairingTerminationReason_c);
                    break;

                case GATTClientProcedureWriteCharacteristicValueIndication_Error_gSmUnexpectedKeyset_c:
                    shell_write(gSmUnexpectedKeyset_c);
                    break;

                case GATTClientProcedureWriteCharacteristicValueIndication_Error_gSmSmpTimeoutOccurred_c:
                    shell_write(gSmSmpTimeoutOccurred_c);
                    break;

                case GATTClientProcedureWriteCharacteristicValueIndication_Error_gSmUnknownSmpPacketType_c:
                    shell_write(gSmUnknownSmpPacketType_c);
                    break;

                case GATTClientProcedureWriteCharacteristicValueIndication_Error_gSmInvalidSmpPacketLength_c:
                    shell_write(gSmInvalidSmpPacketLength_c);
                    break;

                case GATTClientProcedureWriteCharacteristicValueIndication_Error_gSmInvalidSmpPacketParameter_c:
                    shell_write(gSmInvalidSmpPacketParameter_c);
                    break;

                case GATTClientProcedureWriteCharacteristicValueIndication_Error_gSmReceivedUnexpectedSmpPacket_c:
                    shell_write(gSmReceivedUnexpectedSmpPacket_c);
                    break;

                case GATTClientProcedureWriteCharacteristicValueIndication_Error_gSmReceivedSmpPacketFromUnknownDevice_c:
                    shell_write(gSmReceivedSmpPacketFromUnknownDevice_c);
                    break;

                case GATTClientProcedureWriteCharacteristicValueIndication_Error_gSmReceivedUnexpectedHciEvent_c:
                    shell_write(gSmReceivedUnexpectedHciEvent_c);
                    break;

                case GATTClientProcedureWriteCharacteristicValueIndication_Error_gSmReceivedHciEventFromUnknownDevice_c:
                    shell_write(gSmReceivedHciEventFromUnknownDevice_c);
                    break;

                case GATTClientProcedureWriteCharacteristicValueIndication_Error_gSmInvalidHciEventParameter_c:
                    shell_write(gSmInvalidHciEventParameter_c);
                    break;

                case GATTClientProcedureWriteCharacteristicValueIndication_Error_gSmLlConnectionEncryptionInProgress_c:
                    shell_write(gSmLlConnectionEncryptionInProgress_c);
                    break;

                case GATTClientProcedureWriteCharacteristicValueIndication_Error_gSmLlConnectionEncryptionFailure_c:
                    shell_write(gSmLlConnectionEncryptionFailure_c);
                    break;

                case GATTClientProcedureWriteCharacteristicValueIndication_Error_gSmInsufficientResources_c:
                    shell_write(gSmInsufficientResources_c);
                    break;

                case GATTClientProcedureWriteCharacteristicValueIndication_Error_gSmOobDataAddressMismatch_c:
                    shell_write(gSmOobDataAddressMismatch_c);
                    break;

//                case GATTClientProcedureWriteCharacteristicValueIndication_Error_gSmSmpPacketReceivedAfterTimeoutOccurred_c:
//                    shell_write(gSmSmpPacketReceivedAfterTimeoutOccurred_c);
//                    break;

                case GATTClientProcedureWriteCharacteristicValueIndication_Error_gSmPairingErrorPasskeyEntryFailed_c:
                    shell_write(gSmPairingErrorPasskeyEntryFailed_c);
                    break;

                case GATTClientProcedureWriteCharacteristicValueIndication_Error_gSmPairingErrorConfirmValueFailed_c:
                    shell_write(gSmPairingErrorConfirmValueFailed_c);
                    break;

                case GATTClientProcedureWriteCharacteristicValueIndication_Error_gSmPairingErrorCommandNotSupported_c:
                    shell_write(gSmPairingErrorCommandNotSupported_c);
                    break;

                case GATTClientProcedureWriteCharacteristicValueIndication_Error_gSmPairingErrorInvalidParameters_c:
                    shell_write(gSmPairingErrorInvalidParameters_c);
                    break;

                case GATTClientProcedureWriteCharacteristicValueIndication_Error_gSmPairingErrorUnknownReason_c:
                    shell_write(gSmPairingErrorUnknownReason_c);
                    break;

                case GATTClientProcedureWriteCharacteristicValueIndication_Error_gSmTbResolvableAddressDoesNotMatchIrk_c:
                    shell_write(gSmTbResolvableAddressDoesNotMatchIrk_c);
                    break;

                case GATTClientProcedureWriteCharacteristicValueIndication_Error_gSmTbInvalidDataSignature_c:
                    shell_write(gSmTbInvalidDataSignature_c);
                    break;

                case GATTClientProcedureWriteCharacteristicValueIndication_Error_gAttInvalidHandle_c:
                    shell_write(gAttInvalidHandle_c);
                    break;

                case GATTClientProcedureWriteCharacteristicValueIndication_Error_gAttReadNotPermitted_c:
                    shell_write(gAttReadNotPermitted_c);
                    break;

                case GATTClientProcedureWriteCharacteristicValueIndication_Error_gAttWriteNotPermitted_c:
                    shell_write(gAttWriteNotPermitted_c);
                    break;

                case GATTClientProcedureWriteCharacteristicValueIndication_Error_gAttInvalidPdu_c:
                    shell_write(gAttInvalidPdu_c);
                    break;

                case GATTClientProcedureWriteCharacteristicValueIndication_Error_gAttInsufficientAuthentication_c:
                    shell_write(gAttInsufficientAuthentication_c);
                    break;

                case GATTClientProcedureWriteCharacteristicValueIndication_Error_gAttRequestNotSupported_c:
                    shell_write(gAttRequestNotSupported_c);
                    break;

                case GATTClientProcedureWriteCharacteristicValueIndication_Error_gAttInvalidOffset_c:
                    shell_write(gAttInvalidOffset_c);
                    break;

                case GATTClientProcedureWriteCharacteristicValueIndication_Error_gAttInsufficientAuthorization_c:
                    shell_write(gAttInsufficientAuthorization_c);
                    break;

                case GATTClientProcedureWriteCharacteristicValueIndication_Error_gAttPrepareQueueFull_c:
                    shell_write(gAttPrepareQueueFull_c);
                    break;

                case GATTClientProcedureWriteCharacteristicValueIndication_Error_gAttAttributeNotFound_c:
                    shell_write(gAttAttributeNotFound_c);
                    break;

                case GATTClientProcedureWriteCharacteristicValueIndication_Error_gAttAttributeNotLong_c:
                    shell_write(gAttAttributeNotLong_c);
                    break;

                case GATTClientProcedureWriteCharacteristicValueIndication_Error_gAttInsufficientEncryptionKeySize_c:
                    shell_write(gAttInsufficientEncryptionKeySize_c);
                    break;

                case GATTClientProcedureWriteCharacteristicValueIndication_Error_gAttInvalidAttributeValueLength_c:
                    shell_write(gAttInvalidAttributeValueLength_c);
                    break;

                case GATTClientProcedureWriteCharacteristicValueIndication_Error_gAttUnlikelyor_c:
                    shell_write(gAttUnlikelyor_c);
                    break;

                case GATTClientProcedureWriteCharacteristicValueIndication_Error_gAttInsufficientEncryption_c:
                    shell_write(gAttInsufficientEncryption_c);
                    break;

                case GATTClientProcedureWriteCharacteristicValueIndication_Error_gAttUnsupportedGroupType_c:
                    shell_write(gAttUnsupportedGroupType_c);
                    break;

                case GATTClientProcedureWriteCharacteristicValueIndication_Error_gAttInsufficientResources_c:
                    shell_write(gAttInsufficientResources_c);
                    break;

                case GATTClientProcedureWriteCharacteristicValueIndication_Error_gGattAnotherProcedureInProgress_c:
                    shell_write(gGattAnotherProcedureInProgress_c);
                    break;

                case GATTClientProcedureWriteCharacteristicValueIndication_Error_gGattLongAttributePacketsCorrupted_c:
                    shell_write(gGattLongAttributePacketsCorrupted_c);
                    break;

                case GATTClientProcedureWriteCharacteristicValueIndication_Error_gGattMultipleAttributesOverflow_c:
                    shell_write(gGattMultipleAttributesOverflow_c);
                    break;

                case GATTClientProcedureWriteCharacteristicValueIndication_Error_gGattUnexpectedReadMultipleResponseLength_c:
                    shell_write(gGattUnexpectedReadMultipleResponseLength_c);
                    break;

                case GATTClientProcedureWriteCharacteristicValueIndication_Error_gGattInvalidValueLength_c:
                    shell_write(gGattInvalidValueLength_c);
                    break;

                case GATTClientProcedureWriteCharacteristicValueIndication_Error_gGattServerTimeout_c:
                    shell_write(gGattServerTimeout_c);
                    break;

                case GATTClientProcedureWriteCharacteristicValueIndication_Error_gGattIndicationAlreadyInProgress_c:
                    shell_write(gGattIndicationAlreadyInProgress_c);
                    break;

                case GATTClientProcedureWriteCharacteristicValueIndication_Error_gGattClientConfirmationTimeout_c:
                    shell_write(gGattClientConfirmationTimeout_c);
                    break;

                case GATTClientProcedureWriteCharacteristicValueIndication_Error_gGapAdvDataTooLong_c:
                    shell_write(gGapAdvDataTooLong_c);
                    break;

                case GATTClientProcedureWriteCharacteristicValueIndication_Error_gGapScanRspDataTooLong_c:
                    shell_write(gGapScanRspDataTooLong_c);
                    break;

                case GATTClientProcedureWriteCharacteristicValueIndication_Error_gGapDeviceNotBonded_c:
                    shell_write(gGapDeviceNotBonded_c);
                    break;

                case GATTClientProcedureWriteCharacteristicValueIndication_Error_gDevDbCccdLimitReached_c:
                    shell_write(gDevDbCccdLimitReached_c);
                    break;

                case GATTClientProcedureWriteCharacteristicValueIndication_Error_gDevDbCccdNotFound_c:
                    shell_write(gDevDbCccdNotFound_c);
                    break;

                case GATTClientProcedureWriteCharacteristicValueIndication_Error_gGattDbInvalidHandle_c:
                    shell_write(gGattDbInvalidHandle_c);
                    break;

                case GATTClientProcedureWriteCharacteristicValueIndication_Error_gGattDbCharacteristicNotFound_c:
                    shell_write(gGattDbCharacteristicNotFound_c);
                    break;

                case GATTClientProcedureWriteCharacteristicValueIndication_Error_gGattDbCccdNotFound_c:
                    shell_write(gGattDbCccdNotFound_c);
                    break;

                case GATTClientProcedureWriteCharacteristicValueIndication_Error_gGattDbServiceNotFound_c:
                    shell_write(gGattDbServiceNotFound_c);
                    break;

                case GATTClientProcedureWriteCharacteristicValueIndication_Error_gGattDbDescriptorNotFound_c:
                    shell_write(gGattDbDescriptorNotFound_c);
                    break;

                default:
                    shell_printf("Unrecognized status 0x%04X", container->Data.GATTClientProcedureWriteCharacteristicValueIndication.Error);
                    break;
            }

            break;

        case 0x448D:
            shell_write("\r\nGATTClientProcedureReadCharacteristicDescriptorIndication");
            shell_write(" -> ");

            switch (container->Data.GATTClientProcedureReadCharacteristicDescriptorIndication.ProcedureResult)
            {
                case GATTClientProcedureReadCharacteristicDescriptorIndication_ProcedureResult_gGattProcSuccess_c:
                    shell_write(gGattProcSuccess_c);
                    break;

                case GATTClientProcedureReadCharacteristicDescriptorIndication_ProcedureResult_gProcedureError_c:
                    shell_write(gProcedureError_c);
                    break;

                default:
                    shell_printf("Unrecognized status 0x%02X", container->Data.GATTClientProcedureReadCharacteristicDescriptorIndication.ProcedureResult);
                    break;
            }

            shell_write(" -> ");

            switch (container->Data.GATTClientProcedureReadCharacteristicDescriptorIndication.Error)
            {
                case GATTClientProcedureReadCharacteristicDescriptorIndication_Error_gBleSuccess_c:
                    shell_write(gBleSuccess_c);
                    break;

                case GATTClientProcedureReadCharacteristicDescriptorIndication_Error_gBleInvalidParameter_c:
                    shell_write(gBleInvalidParameter_c);
                    break;

                case GATTClientProcedureReadCharacteristicDescriptorIndication_Error_gBleOverflow_c:
                    shell_write(gBleOverflow_c);
                    break;

                case GATTClientProcedureReadCharacteristicDescriptorIndication_Error_gBleUnavailable_c:
                    shell_write(gBleUnavailable_c);
                    break;

                case GATTClientProcedureReadCharacteristicDescriptorIndication_Error_gBleFeatureNotSupported_c:
                    shell_write(gBleFeatureNotSupported_c);
                    break;

                case GATTClientProcedureReadCharacteristicDescriptorIndication_Error_gBleOutOfMemory_c:
                    shell_write(gBleOutOfMemory_c);
                    break;

                case GATTClientProcedureReadCharacteristicDescriptorIndication_Error_gBleAlreadyInitialized_c:
                    shell_write(gBleAlreadyInitialized_c);
                    break;

                case GATTClientProcedureReadCharacteristicDescriptorIndication_Error_gBleOsError_c:
                    shell_write(gBleOsc);
                    break;

                case GATTClientProcedureReadCharacteristicDescriptorIndication_Error_gBleUnexpectedError_c:
                    shell_write(gBleUnexpectedc);
                    break;

                case GATTClientProcedureReadCharacteristicDescriptorIndication_Error_gBleInvalidState_c:
                    shell_write(gBleInvalidState_c);
                    break;

                case GATTClientProcedureReadCharacteristicDescriptorIndication_Error_gHciUnknownHciCommand_c:
                    shell_write(gHciUnknownHciCommand_c);
                    break;

                case GATTClientProcedureReadCharacteristicDescriptorIndication_Error_gHciUnknownConnectionIdentifier_c:
                    shell_write(gHciUnknownConnectionIdentifier_c);
                    break;

                case GATTClientProcedureReadCharacteristicDescriptorIndication_Error_gHciHardwareFailure_c:
                    shell_write(gHciHardwareFailure_c);
                    break;

                case GATTClientProcedureReadCharacteristicDescriptorIndication_Error_gHciPageTimeout_c:
                    shell_write(gHciPageTimeout_c);
                    break;

                case GATTClientProcedureReadCharacteristicDescriptorIndication_Error_gHciAuthenticationFailure_c:
                    shell_write(gHciAuthenticationFailure_c);
                    break;

                case GATTClientProcedureReadCharacteristicDescriptorIndication_Error_gHciPinOrKeyMissing_c:
                    shell_write(gHciPinOrKeyMissing_c);
                    break;

                case GATTClientProcedureReadCharacteristicDescriptorIndication_Error_gHciMemoryCapacityExceeded_c:
                    shell_write(gHciMemoryCapacityExceeded_c);
                    break;

                case GATTClientProcedureReadCharacteristicDescriptorIndication_Error_gHciConnectionTimeout_c:
                    shell_write(gHciConnectionTimeout_c);
                    break;

                case GATTClientProcedureReadCharacteristicDescriptorIndication_Error_gHciConnectionLimitExceeded_c:
                    shell_write(gHciConnectionLimitExceeded_c);
                    break;

                case GATTClientProcedureReadCharacteristicDescriptorIndication_Error_gHciSynchronousConnectionLimitToADeviceExceeded_c:
                    shell_write(gHciSynchronousConnectionLimitToADeviceExceeded_c);
                    break;

                case GATTClientProcedureReadCharacteristicDescriptorIndication_Error_gHciAclConnectionAlreadyExists_c:
                    shell_write(gHciAclConnectionAlreadyExists_c);
                    break;

                case GATTClientProcedureReadCharacteristicDescriptorIndication_Error_gHciCommandDisallowed_c:
                    shell_write(gHciCommandDisallowed_c);
                    break;

                case GATTClientProcedureReadCharacteristicDescriptorIndication_Error_gHciConnectionRejectedDueToLimitedResources_c:
                    shell_write(gHciConnectionRejectedDueToLimitedResources_c);
                    break;

                case GATTClientProcedureReadCharacteristicDescriptorIndication_Error_gHciConnectionRejectedDueToSecurityReasons_c:
                    shell_write(gHciConnectionRejectedDueToSecurityReasons_c);
                    break;

                case GATTClientProcedureReadCharacteristicDescriptorIndication_Error_gHciConnectionRejectedDueToUnacceptableBdAddr_c:
                    shell_write(gHciConnectionRejectedDueToUnacceptableBdAddr_c);
                    break;

                case GATTClientProcedureReadCharacteristicDescriptorIndication_Error_gHciConnectionAcceptTimeoutExceeded_c:
                    shell_write(gHciConnectionAcceptTimeoutExceeded_c);
                    break;

                case GATTClientProcedureReadCharacteristicDescriptorIndication_Error_gHciUnsupportedFeatureOrParameterValue_c:
                    shell_write(gHciUnsupportedFeatureOrParameterValue_c);
                    break;

                case GATTClientProcedureReadCharacteristicDescriptorIndication_Error_gHciInvalidHciCommandParameters_c:
                    shell_write(gHciInvalidHciCommandParameters_c);
                    break;

                case GATTClientProcedureReadCharacteristicDescriptorIndication_Error_gHciRemoteUserTerminatedConnection_c:
                    shell_write(gHciRemoteUserTerminatedConnection_c);
                    break;

                case GATTClientProcedureReadCharacteristicDescriptorIndication_Error_gHciRemoteDeviceTerminatedConnectionLowResources_c:
                    shell_write(gHciRemoteDeviceTerminatedConnectionLowResources_c);
                    break;

                case GATTClientProcedureReadCharacteristicDescriptorIndication_Error_gHciRemoteDeviceTerminatedConnectionPowerOff_c:
                    shell_write(gHciRemoteDeviceTerminatedConnectionPowerOff_c);
                    break;

                case GATTClientProcedureReadCharacteristicDescriptorIndication_Error_gHciConnectionTerminatedByLocalHost_c:
                    shell_write(gHciConnectionTerminatedByLocalHost_c);
                    break;

                case GATTClientProcedureReadCharacteristicDescriptorIndication_Error_gHciRepeatedAttempts_c:
                    shell_write(gHciRepeatedAttempts_c);
                    break;

                case GATTClientProcedureReadCharacteristicDescriptorIndication_Error_gHciPairingNotAllowed_c:
                    shell_write(gHciPairingNotAllowed_c);
                    break;

                case GATTClientProcedureReadCharacteristicDescriptorIndication_Error_gHciUnknownLpmPdu_c:
                    shell_write(gHciUnknownLpmPdu_c);
                    break;

                case GATTClientProcedureReadCharacteristicDescriptorIndication_Error_gHciUnsupportedRemoteFeature_c:
                    shell_write(gHciUnsupportedRemoteFeature_c);
                    break;

                case GATTClientProcedureReadCharacteristicDescriptorIndication_Error_gHciScoOffsetRejected_c:
                    shell_write(gHciScoOffsetRejected_c);
                    break;

                case GATTClientProcedureReadCharacteristicDescriptorIndication_Error_gHciScoIntervalRejected_c:
                    shell_write(gHciScoIntervalRejected_c);
                    break;

                case GATTClientProcedureReadCharacteristicDescriptorIndication_Error_gHciScoAirModeRejected_c:
                    shell_write(gHciScoAirModeRejected_c);
                    break;

                case GATTClientProcedureReadCharacteristicDescriptorIndication_Error_gHciInvalidLpmParameters_c:
                    shell_write(gHciInvalidLpmParameters_c);
                    break;

                case GATTClientProcedureReadCharacteristicDescriptorIndication_Error_gHciUnspecifiedError_c:
                    shell_write(gHciUnspecifiedc);
                    break;

                case GATTClientProcedureReadCharacteristicDescriptorIndication_Error_gHciUnsupportedLpmParameterValue_c:
                    shell_write(gHciUnsupportedLpmParameterValue_c);
                    break;

                case GATTClientProcedureReadCharacteristicDescriptorIndication_Error_gHciRoleChangeNotAllowed_c:
                    shell_write(gHciRoleChangeNotAllowed_c);
                    break;

                case GATTClientProcedureReadCharacteristicDescriptorIndication_Error_gHciLLResponseTimeout_c:
                    shell_write(gHciLLResponseTimeout_c);
                    break;

                case GATTClientProcedureReadCharacteristicDescriptorIndication_Error_gHciLmpErrorTransactionCollision_c:
                    shell_write(gHciLmpErrorTransactionCollision_c);
                    break;

                case GATTClientProcedureReadCharacteristicDescriptorIndication_Error_gHciLmpPduNotAllowed_c:
                    shell_write(gHciLmpPduNotAllowed_c);
                    break;

                case GATTClientProcedureReadCharacteristicDescriptorIndication_Error_gHciEncryptionModeNotAcceptable_c:
                    shell_write(gHciEncryptionModeNotAcceptable_c);
                    break;

                case GATTClientProcedureReadCharacteristicDescriptorIndication_Error_gHciLinkKeyCannotBeChanged_c:
                    shell_write(gHciLinkKeyCannotBeChanged_c);
                    break;

                case GATTClientProcedureReadCharacteristicDescriptorIndication_Error_gHciRequestedQosNotSupported_c:
                    shell_write(gHciRequestedQosNotSupported_c);
                    break;

                case GATTClientProcedureReadCharacteristicDescriptorIndication_Error_gHciInstantPassed_c:
                    shell_write(gHciInstantPassed_c);
                    break;

                case GATTClientProcedureReadCharacteristicDescriptorIndication_Error_gHciPairingWithUnitKeyNotSupported_c:
                    shell_write(gHciPairingWithUnitKeyNotSupported_c);
                    break;

                case GATTClientProcedureReadCharacteristicDescriptorIndication_Error_gHciDifferentTransactionCollision_c:
                    shell_write(gHciDifferentTransactionCollision_c);
                    break;

                case GATTClientProcedureReadCharacteristicDescriptorIndication_Error_gHciReserved_0x2B_c:
                    shell_write(gHciReserved_0x2B_c);
                    break;

                case GATTClientProcedureReadCharacteristicDescriptorIndication_Error_gHciQosNotAcceptableParameter_c:
                    shell_write(gHciQosNotAcceptableParameter_c);
                    break;

                case GATTClientProcedureReadCharacteristicDescriptorIndication_Error_gHciQosRejected_c:
                    shell_write(gHciQosRejected_c);
                    break;

                case GATTClientProcedureReadCharacteristicDescriptorIndication_Error_gHciChannelClassificationNotSupported_c:
                    shell_write(gHciChannelClassificationNotSupported_c);
                    break;

                case GATTClientProcedureReadCharacteristicDescriptorIndication_Error_gHciInsufficientSecurity_c:
                    shell_write(gHciInsufficientSecurity_c);
                    break;

                case GATTClientProcedureReadCharacteristicDescriptorIndication_Error_gHciParameterOutOfMandatoryRange_c:
                    shell_write(gHciParameterOutOfMandatoryRange_c);
                    break;

                case GATTClientProcedureReadCharacteristicDescriptorIndication_Error_gHciReserved_0x31_c:
                    shell_write(gHciReserved_0x31_c);
                    break;

                case GATTClientProcedureReadCharacteristicDescriptorIndication_Error_gHciRoleSwitchPending_c:
                    shell_write(gHciRoleSwitchPending_c);
                    break;

                case GATTClientProcedureReadCharacteristicDescriptorIndication_Error_gHciReserved_0x33_c:
                    shell_write(gHciReserved_0x33_c);
                    break;

                case GATTClientProcedureReadCharacteristicDescriptorIndication_Error_gHciReservedSlotViolation_c:
                    shell_write(gHciReservedSlotViolation_c);
                    break;

                case GATTClientProcedureReadCharacteristicDescriptorIndication_Error_gHciRoleSwitchFailed_c:
                    shell_write(gHciRoleSwitchFailed_c);
                    break;

                case GATTClientProcedureReadCharacteristicDescriptorIndication_Error_gHciExtendedInquiryResponseTooLarge_c:
                    shell_write(gHciExtendedInquiryResponseTooLarge_c);
                    break;

                case GATTClientProcedureReadCharacteristicDescriptorIndication_Error_gHciSecureSimplePairingNotSupportedByHost_c:
                    shell_write(gHciSecureSimplePairingNotSupportedByHost_c);
                    break;

                case GATTClientProcedureReadCharacteristicDescriptorIndication_Error_gHciHostBusyPairing_c:
                    shell_write(gHciHostBusyPairing_c);
                    break;

                case GATTClientProcedureReadCharacteristicDescriptorIndication_Error_gHciConnectionRejectedDueToNoSuitableChannelFound_c:
                    shell_write(gHciConnectionRejectedDueToNoSuitableChannelFound_c);
                    break;

                case GATTClientProcedureReadCharacteristicDescriptorIndication_Error_gHciControllerBusy_c:
                    shell_write(gHciControllerBusy_c);
                    break;

                case GATTClientProcedureReadCharacteristicDescriptorIndication_Error_gHciUnacceptableConnectionParameters_c:
                    shell_write(gHciUnacceptableConnectionParameters_c);
                    break;

                case GATTClientProcedureReadCharacteristicDescriptorIndication_Error_gHciDirectedAdvertisingTimeout_c:
                    shell_write(gHciDirectedAdvertisingTimeout_c);
                    break;

                case GATTClientProcedureReadCharacteristicDescriptorIndication_Error_gHciConnectionTerminatedDueToMicFailure_c:
                    shell_write(gHciConnectionTerminatedDueToMicFailure_c);
                    break;

                case GATTClientProcedureReadCharacteristicDescriptorIndication_Error_gHciConnectionFailedToBeEstablished_c:
                    shell_write(gHciConnectionFailedToBeEstablished_c);
                    break;

                case GATTClientProcedureReadCharacteristicDescriptorIndication_Error_gHciMacConnectionFailed_c:
                    shell_write(gHciMacConnectionFailed_c);
                    break;

                case GATTClientProcedureReadCharacteristicDescriptorIndication_Error_gHciCoarseClockAdjustmentRejected_c:
                    shell_write(gHciCoarseClockAdjustmentRejected_c);
                    break;

                case GATTClientProcedureReadCharacteristicDescriptorIndication_Error_gHciAlreadyInit_c:
                    shell_write(gHciAlreadyInit_c);
                    break;

                case GATTClientProcedureReadCharacteristicDescriptorIndication_Error_gHciInvalidParameter_c:
                    shell_write(gHciInvalidParameter_c);
                    break;

                case GATTClientProcedureReadCharacteristicDescriptorIndication_Error_gHciCallbackNotInstalled_c:
                    shell_write(gHciCallbackNotInstalled_c);
                    break;

                case GATTClientProcedureReadCharacteristicDescriptorIndication_Error_gHciCallbackAlreadyInstalled_c:
                    shell_write(gHciCallbackAlreadyInstalled_c);
                    break;

                case GATTClientProcedureReadCharacteristicDescriptorIndication_Error_gHciCommandNotSupported_c:
                    shell_write(gHciCommandNotSupported_c);
                    break;

                case GATTClientProcedureReadCharacteristicDescriptorIndication_Error_gHciEventNotSupported_c:
                    shell_write(gHciEventNotSupported_c);
                    break;

                case GATTClientProcedureReadCharacteristicDescriptorIndication_Error_gHciTransportError_c:
                    shell_write(gHciTransportc);
                    break;

                case GATTClientProcedureReadCharacteristicDescriptorIndication_Error_gL2caAlreadyInit_c:
                    shell_write(gL2caAlreadyInit_c);
                    break;

                case GATTClientProcedureReadCharacteristicDescriptorIndication_Error_gL2caInsufficientResources_c:
                    shell_write(gL2caInsufficientResources_c);
                    break;

                case GATTClientProcedureReadCharacteristicDescriptorIndication_Error_gL2caCallbackNotInstalled_c:
                    shell_write(gL2caCallbackNotInstalled_c);
                    break;

                case GATTClientProcedureReadCharacteristicDescriptorIndication_Error_gL2caCallbackAlreadyInstalled_c:
                    shell_write(gL2caCallbackAlreadyInstalled_c);
                    break;

                case GATTClientProcedureReadCharacteristicDescriptorIndication_Error_gL2caLePsmInvalid_c:
                    shell_write(gL2caLePsmInvalid_c);
                    break;

                case GATTClientProcedureReadCharacteristicDescriptorIndication_Error_gL2caLePsmAlreadyRegistered_c:
                    shell_write(gL2caLePsmAlreadyRegistered_c);
                    break;

                case GATTClientProcedureReadCharacteristicDescriptorIndication_Error_gL2caLePsmNotRegistered_c:
                    shell_write(gL2caLePsmNotRegistered_c);
                    break;

                case GATTClientProcedureReadCharacteristicDescriptorIndication_Error_gL2caLePsmInsufficientResources_c:
                    shell_write(gL2caLePsmInsufficientResources_c);
                    break;

                case GATTClientProcedureReadCharacteristicDescriptorIndication_Error_gL2caChannelInvalid_c:
                    shell_write(gL2caChannelInvalid_c);
                    break;

                case GATTClientProcedureReadCharacteristicDescriptorIndication_Error_gL2caChannelClosed_c:
                    shell_write(gL2caChannelClosed_c);
                    break;

                case GATTClientProcedureReadCharacteristicDescriptorIndication_Error_gL2caChannelAlreadyConnected_c:
                    shell_write(gL2caChannelAlreadyConnected_c);
                    break;

                case GATTClientProcedureReadCharacteristicDescriptorIndication_Error_gL2caConnectionParametersRejected_c:
                    shell_write(gL2caConnectionParametersRejected_c);
                    break;

                case GATTClientProcedureReadCharacteristicDescriptorIndication_Error_gL2caChannelBusy_c:
                    shell_write(gL2caChannelBusy_c);
                    break;

                case GATTClientProcedureReadCharacteristicDescriptorIndication_Error_gL2caInvalidParameter_c:
                    shell_write(gL2caInvalidParameter_c);
                    break;

                case GATTClientProcedureReadCharacteristicDescriptorIndication_Error_gL2caError_c:
                    shell_write(gL2cac);
                    break;

                case GATTClientProcedureReadCharacteristicDescriptorIndication_Error_gSmNullCBFunction_c:
                    shell_write(gSmNullCBFunction_c);
                    break;

                case GATTClientProcedureReadCharacteristicDescriptorIndication_Error_gSmCommandNotSupported_c:
                    shell_write(gSmCommandNotSupported_c);
                    break;

                case GATTClientProcedureReadCharacteristicDescriptorIndication_Error_gSmUnexpectedCommand_c:
                    shell_write(gSmUnexpectedCommand_c);
                    break;

                case GATTClientProcedureReadCharacteristicDescriptorIndication_Error_gSmInvalidCommandCode_c:
                    shell_write(gSmInvalidCommandCode_c);
                    break;

                case GATTClientProcedureReadCharacteristicDescriptorIndication_Error_gSmInvalidCommandLength_c:
                    shell_write(gSmInvalidCommandLength_c);
                    break;

                case GATTClientProcedureReadCharacteristicDescriptorIndication_Error_gSmInvalidCommandParameter_c:
                    shell_write(gSmInvalidCommandParameter_c);
                    break;

                case GATTClientProcedureReadCharacteristicDescriptorIndication_Error_gSmInvalidDeviceId_c:
                    shell_write(gSmInvalidDeviceId_c);
                    break;

                case GATTClientProcedureReadCharacteristicDescriptorIndication_Error_gSmInvalidInternalOperation_c:
                    shell_write(gSmInvalidInternalOperation_c);
                    break;

                case GATTClientProcedureReadCharacteristicDescriptorIndication_Error_gSmInvalidConnectionHandle_c:
                    shell_write(gSmInvalidConnectionHandle_c);
                    break;

                case GATTClientProcedureReadCharacteristicDescriptorIndication_Error_gSmInproperKeyDistributionField_c:
                    shell_write(gSmInproperKeyDistributionField_c);
                    break;

                case GATTClientProcedureReadCharacteristicDescriptorIndication_Error_gSmUnexpectedKeyType_c:
                    shell_write(gSmUnexpectedKeyType_c);
                    break;

                case GATTClientProcedureReadCharacteristicDescriptorIndication_Error_gSmUnexpectedPairingTerminationReason_c:
                    shell_write(gSmUnexpectedPairingTerminationReason_c);
                    break;

                case GATTClientProcedureReadCharacteristicDescriptorIndication_Error_gSmUnexpectedKeyset_c:
                    shell_write(gSmUnexpectedKeyset_c);
                    break;

                case GATTClientProcedureReadCharacteristicDescriptorIndication_Error_gSmSmpTimeoutOccurred_c:
                    shell_write(gSmSmpTimeoutOccurred_c);
                    break;

                case GATTClientProcedureReadCharacteristicDescriptorIndication_Error_gSmUnknownSmpPacketType_c:
                    shell_write(gSmUnknownSmpPacketType_c);
                    break;

                case GATTClientProcedureReadCharacteristicDescriptorIndication_Error_gSmInvalidSmpPacketLength_c:
                    shell_write(gSmInvalidSmpPacketLength_c);
                    break;

                case GATTClientProcedureReadCharacteristicDescriptorIndication_Error_gSmInvalidSmpPacketParameter_c:
                    shell_write(gSmInvalidSmpPacketParameter_c);
                    break;

                case GATTClientProcedureReadCharacteristicDescriptorIndication_Error_gSmReceivedUnexpectedSmpPacket_c:
                    shell_write(gSmReceivedUnexpectedSmpPacket_c);
                    break;

                case GATTClientProcedureReadCharacteristicDescriptorIndication_Error_gSmReceivedSmpPacketFromUnknownDevice_c:
                    shell_write(gSmReceivedSmpPacketFromUnknownDevice_c);
                    break;

                case GATTClientProcedureReadCharacteristicDescriptorIndication_Error_gSmReceivedUnexpectedHciEvent_c:
                    shell_write(gSmReceivedUnexpectedHciEvent_c);
                    break;

                case GATTClientProcedureReadCharacteristicDescriptorIndication_Error_gSmReceivedHciEventFromUnknownDevice_c:
                    shell_write(gSmReceivedHciEventFromUnknownDevice_c);
                    break;

                case GATTClientProcedureReadCharacteristicDescriptorIndication_Error_gSmInvalidHciEventParameter_c:
                    shell_write(gSmInvalidHciEventParameter_c);
                    break;

                case GATTClientProcedureReadCharacteristicDescriptorIndication_Error_gSmLlConnectionEncryptionInProgress_c:
                    shell_write(gSmLlConnectionEncryptionInProgress_c);
                    break;

                case GATTClientProcedureReadCharacteristicDescriptorIndication_Error_gSmLlConnectionEncryptionFailure_c:
                    shell_write(gSmLlConnectionEncryptionFailure_c);
                    break;

                case GATTClientProcedureReadCharacteristicDescriptorIndication_Error_gSmInsufficientResources_c:
                    shell_write(gSmInsufficientResources_c);
                    break;

                case GATTClientProcedureReadCharacteristicDescriptorIndication_Error_gSmOobDataAddressMismatch_c:
                    shell_write(gSmOobDataAddressMismatch_c);
                    break;

//                case GATTClientProcedureReadCharacteristicDescriptorIndication_Error_gSmSmpPacketReceivedAfterTimeoutOccurred_c:
//                    shell_write(gSmSmpPacketReceivedAfterTimeoutOccurred_c);
//                    break;

                case GATTClientProcedureReadCharacteristicDescriptorIndication_Error_gSmPairingErrorPasskeyEntryFailed_c:
                    shell_write(gSmPairingErrorPasskeyEntryFailed_c);
                    break;

                case GATTClientProcedureReadCharacteristicDescriptorIndication_Error_gSmPairingErrorConfirmValueFailed_c:
                    shell_write(gSmPairingErrorConfirmValueFailed_c);
                    break;

                case GATTClientProcedureReadCharacteristicDescriptorIndication_Error_gSmPairingErrorCommandNotSupported_c:
                    shell_write(gSmPairingErrorCommandNotSupported_c);
                    break;

                case GATTClientProcedureReadCharacteristicDescriptorIndication_Error_gSmPairingErrorInvalidParameters_c:
                    shell_write(gSmPairingErrorInvalidParameters_c);
                    break;

                case GATTClientProcedureReadCharacteristicDescriptorIndication_Error_gSmPairingErrorUnknownReason_c:
                    shell_write(gSmPairingErrorUnknownReason_c);
                    break;

                case GATTClientProcedureReadCharacteristicDescriptorIndication_Error_gSmTbResolvableAddressDoesNotMatchIrk_c:
                    shell_write(gSmTbResolvableAddressDoesNotMatchIrk_c);
                    break;

                case GATTClientProcedureReadCharacteristicDescriptorIndication_Error_gSmTbInvalidDataSignature_c:
                    shell_write(gSmTbInvalidDataSignature_c);
                    break;

                case GATTClientProcedureReadCharacteristicDescriptorIndication_Error_gAttInvalidHandle_c:
                    shell_write(gAttInvalidHandle_c);
                    break;

                case GATTClientProcedureReadCharacteristicDescriptorIndication_Error_gAttReadNotPermitted_c:
                    shell_write(gAttReadNotPermitted_c);
                    break;

                case GATTClientProcedureReadCharacteristicDescriptorIndication_Error_gAttWriteNotPermitted_c:
                    shell_write(gAttWriteNotPermitted_c);
                    break;

                case GATTClientProcedureReadCharacteristicDescriptorIndication_Error_gAttInvalidPdu_c:
                    shell_write(gAttInvalidPdu_c);
                    break;

                case GATTClientProcedureReadCharacteristicDescriptorIndication_Error_gAttInsufficientAuthentication_c:
                    shell_write(gAttInsufficientAuthentication_c);
                    break;

                case GATTClientProcedureReadCharacteristicDescriptorIndication_Error_gAttRequestNotSupported_c:
                    shell_write(gAttRequestNotSupported_c);
                    break;

                case GATTClientProcedureReadCharacteristicDescriptorIndication_Error_gAttInvalidOffset_c:
                    shell_write(gAttInvalidOffset_c);
                    break;

                case GATTClientProcedureReadCharacteristicDescriptorIndication_Error_gAttInsufficientAuthorization_c:
                    shell_write(gAttInsufficientAuthorization_c);
                    break;

                case GATTClientProcedureReadCharacteristicDescriptorIndication_Error_gAttPrepareQueueFull_c:
                    shell_write(gAttPrepareQueueFull_c);
                    break;

                case GATTClientProcedureReadCharacteristicDescriptorIndication_Error_gAttAttributeNotFound_c:
                    shell_write(gAttAttributeNotFound_c);
                    break;

                case GATTClientProcedureReadCharacteristicDescriptorIndication_Error_gAttAttributeNotLong_c:
                    shell_write(gAttAttributeNotLong_c);
                    break;

                case GATTClientProcedureReadCharacteristicDescriptorIndication_Error_gAttInsufficientEncryptionKeySize_c:
                    shell_write(gAttInsufficientEncryptionKeySize_c);
                    break;

                case GATTClientProcedureReadCharacteristicDescriptorIndication_Error_gAttInvalidAttributeValueLength_c:
                    shell_write(gAttInvalidAttributeValueLength_c);
                    break;

                case GATTClientProcedureReadCharacteristicDescriptorIndication_Error_gAttUnlikelyor_c:
                    shell_write(gAttUnlikelyor_c);
                    break;

                case GATTClientProcedureReadCharacteristicDescriptorIndication_Error_gAttInsufficientEncryption_c:
                    shell_write(gAttInsufficientEncryption_c);
                    break;

                case GATTClientProcedureReadCharacteristicDescriptorIndication_Error_gAttUnsupportedGroupType_c:
                    shell_write(gAttUnsupportedGroupType_c);
                    break;

                case GATTClientProcedureReadCharacteristicDescriptorIndication_Error_gAttInsufficientResources_c:
                    shell_write(gAttInsufficientResources_c);
                    break;

                case GATTClientProcedureReadCharacteristicDescriptorIndication_Error_gGattAnotherProcedureInProgress_c:
                    shell_write(gGattAnotherProcedureInProgress_c);
                    break;

                case GATTClientProcedureReadCharacteristicDescriptorIndication_Error_gGattLongAttributePacketsCorrupted_c:
                    shell_write(gGattLongAttributePacketsCorrupted_c);
                    break;

                case GATTClientProcedureReadCharacteristicDescriptorIndication_Error_gGattMultipleAttributesOverflow_c:
                    shell_write(gGattMultipleAttributesOverflow_c);
                    break;

                case GATTClientProcedureReadCharacteristicDescriptorIndication_Error_gGattUnexpectedReadMultipleResponseLength_c:
                    shell_write(gGattUnexpectedReadMultipleResponseLength_c);
                    break;

                case GATTClientProcedureReadCharacteristicDescriptorIndication_Error_gGattInvalidValueLength_c:
                    shell_write(gGattInvalidValueLength_c);
                    break;

                case GATTClientProcedureReadCharacteristicDescriptorIndication_Error_gGattServerTimeout_c:
                    shell_write(gGattServerTimeout_c);
                    break;

                case GATTClientProcedureReadCharacteristicDescriptorIndication_Error_gGattIndicationAlreadyInProgress_c:
                    shell_write(gGattIndicationAlreadyInProgress_c);
                    break;

                case GATTClientProcedureReadCharacteristicDescriptorIndication_Error_gGattClientConfirmationTimeout_c:
                    shell_write(gGattClientConfirmationTimeout_c);
                    break;

                case GATTClientProcedureReadCharacteristicDescriptorIndication_Error_gGapAdvDataTooLong_c:
                    shell_write(gGapAdvDataTooLong_c);
                    break;

                case GATTClientProcedureReadCharacteristicDescriptorIndication_Error_gGapScanRspDataTooLong_c:
                    shell_write(gGapScanRspDataTooLong_c);
                    break;

                case GATTClientProcedureReadCharacteristicDescriptorIndication_Error_gGapDeviceNotBonded_c:
                    shell_write(gGapDeviceNotBonded_c);
                    break;

                case GATTClientProcedureReadCharacteristicDescriptorIndication_Error_gDevDbCccdLimitReached_c:
                    shell_write(gDevDbCccdLimitReached_c);
                    break;

                case GATTClientProcedureReadCharacteristicDescriptorIndication_Error_gDevDbCccdNotFound_c:
                    shell_write(gDevDbCccdNotFound_c);
                    break;

                case GATTClientProcedureReadCharacteristicDescriptorIndication_Error_gGattDbInvalidHandle_c:
                    shell_write(gGattDbInvalidHandle_c);
                    break;

                case GATTClientProcedureReadCharacteristicDescriptorIndication_Error_gGattDbCharacteristicNotFound_c:
                    shell_write(gGattDbCharacteristicNotFound_c);
                    break;

                case GATTClientProcedureReadCharacteristicDescriptorIndication_Error_gGattDbCccdNotFound_c:
                    shell_write(gGattDbCccdNotFound_c);
                    break;

                case GATTClientProcedureReadCharacteristicDescriptorIndication_Error_gGattDbServiceNotFound_c:
                    shell_write(gGattDbServiceNotFound_c);
                    break;

                case GATTClientProcedureReadCharacteristicDescriptorIndication_Error_gGattDbDescriptorNotFound_c:
                    shell_write(gGattDbDescriptorNotFound_c);
                    break;

                default:
                    shell_printf("Unrecognized status 0x%04X", container->Data.GATTClientProcedureReadCharacteristicDescriptorIndication.Error);
                    break;
            }

            break;

        case 0x448E:
            shell_write("\r\nGATTClientProcedureWriteCharacteristicDescriptorIndication");
            shell_write(" -> ");

            switch (container->Data.GATTClientProcedureWriteCharacteristicDescriptorIndication.ProcedureResult)
            {
                case GATTClientProcedureWriteCharacteristicDescriptorIndication_ProcedureResult_gGattProcSuccess_c:
                    shell_write(gGattProcSuccess_c);
                    break;

                case GATTClientProcedureWriteCharacteristicDescriptorIndication_ProcedureResult_gProcedureError_c:
                    shell_write(gProcedureError_c);
                    break;

                default:
                    shell_printf("Unrecognized status 0x%02X", container->Data.GATTClientProcedureWriteCharacteristicDescriptorIndication.ProcedureResult);
                    break;
            }

            shell_write(" -> ");

            switch (container->Data.GATTClientProcedureWriteCharacteristicDescriptorIndication.Error)
            {
                case GATTClientProcedureWriteCharacteristicDescriptorIndication_Error_gBleSuccess_c:
                    shell_write(gBleSuccess_c);
                    break;

                case GATTClientProcedureWriteCharacteristicDescriptorIndication_Error_gBleInvalidParameter_c:
                    shell_write(gBleInvalidParameter_c);
                    break;

                case GATTClientProcedureWriteCharacteristicDescriptorIndication_Error_gBleOverflow_c:
                    shell_write(gBleOverflow_c);
                    break;

                case GATTClientProcedureWriteCharacteristicDescriptorIndication_Error_gBleUnavailable_c:
                    shell_write(gBleUnavailable_c);
                    break;

                case GATTClientProcedureWriteCharacteristicDescriptorIndication_Error_gBleFeatureNotSupported_c:
                    shell_write(gBleFeatureNotSupported_c);
                    break;

                case GATTClientProcedureWriteCharacteristicDescriptorIndication_Error_gBleOutOfMemory_c:
                    shell_write(gBleOutOfMemory_c);
                    break;

                case GATTClientProcedureWriteCharacteristicDescriptorIndication_Error_gBleAlreadyInitialized_c:
                    shell_write(gBleAlreadyInitialized_c);
                    break;

                case GATTClientProcedureWriteCharacteristicDescriptorIndication_Error_gBleOsError_c:
                    shell_write(gBleOsc);
                    break;

                case GATTClientProcedureWriteCharacteristicDescriptorIndication_Error_gBleUnexpectedError_c:
                    shell_write(gBleUnexpectedc);
                    break;

                case GATTClientProcedureWriteCharacteristicDescriptorIndication_Error_gBleInvalidState_c:
                    shell_write(gBleInvalidState_c);
                    break;

                case GATTClientProcedureWriteCharacteristicDescriptorIndication_Error_gHciUnknownHciCommand_c:
                    shell_write(gHciUnknownHciCommand_c);
                    break;

                case GATTClientProcedureWriteCharacteristicDescriptorIndication_Error_gHciUnknownConnectionIdentifier_c:
                    shell_write(gHciUnknownConnectionIdentifier_c);
                    break;

                case GATTClientProcedureWriteCharacteristicDescriptorIndication_Error_gHciHardwareFailure_c:
                    shell_write(gHciHardwareFailure_c);
                    break;

                case GATTClientProcedureWriteCharacteristicDescriptorIndication_Error_gHciPageTimeout_c:
                    shell_write(gHciPageTimeout_c);
                    break;

                case GATTClientProcedureWriteCharacteristicDescriptorIndication_Error_gHciAuthenticationFailure_c:
                    shell_write(gHciAuthenticationFailure_c);
                    break;

                case GATTClientProcedureWriteCharacteristicDescriptorIndication_Error_gHciPinOrKeyMissing_c:
                    shell_write(gHciPinOrKeyMissing_c);
                    break;

                case GATTClientProcedureWriteCharacteristicDescriptorIndication_Error_gHciMemoryCapacityExceeded_c:
                    shell_write(gHciMemoryCapacityExceeded_c);
                    break;

                case GATTClientProcedureWriteCharacteristicDescriptorIndication_Error_gHciConnectionTimeout_c:
                    shell_write(gHciConnectionTimeout_c);
                    break;

                case GATTClientProcedureWriteCharacteristicDescriptorIndication_Error_gHciConnectionLimitExceeded_c:
                    shell_write(gHciConnectionLimitExceeded_c);
                    break;

                case GATTClientProcedureWriteCharacteristicDescriptorIndication_Error_gHciSynchronousConnectionLimitToADeviceExceeded_c:
                    shell_write(gHciSynchronousConnectionLimitToADeviceExceeded_c);
                    break;

                case GATTClientProcedureWriteCharacteristicDescriptorIndication_Error_gHciAclConnectionAlreadyExists_c:
                    shell_write(gHciAclConnectionAlreadyExists_c);
                    break;

                case GATTClientProcedureWriteCharacteristicDescriptorIndication_Error_gHciCommandDisallowed_c:
                    shell_write(gHciCommandDisallowed_c);
                    break;

                case GATTClientProcedureWriteCharacteristicDescriptorIndication_Error_gHciConnectionRejectedDueToLimitedResources_c:
                    shell_write(gHciConnectionRejectedDueToLimitedResources_c);
                    break;

                case GATTClientProcedureWriteCharacteristicDescriptorIndication_Error_gHciConnectionRejectedDueToSecurityReasons_c:
                    shell_write(gHciConnectionRejectedDueToSecurityReasons_c);
                    break;

                case GATTClientProcedureWriteCharacteristicDescriptorIndication_Error_gHciConnectionRejectedDueToUnacceptableBdAddr_c:
                    shell_write(gHciConnectionRejectedDueToUnacceptableBdAddr_c);
                    break;

                case GATTClientProcedureWriteCharacteristicDescriptorIndication_Error_gHciConnectionAcceptTimeoutExceeded_c:
                    shell_write(gHciConnectionAcceptTimeoutExceeded_c);
                    break;

                case GATTClientProcedureWriteCharacteristicDescriptorIndication_Error_gHciUnsupportedFeatureOrParameterValue_c:
                    shell_write(gHciUnsupportedFeatureOrParameterValue_c);
                    break;

                case GATTClientProcedureWriteCharacteristicDescriptorIndication_Error_gHciInvalidHciCommandParameters_c:
                    shell_write(gHciInvalidHciCommandParameters_c);
                    break;

                case GATTClientProcedureWriteCharacteristicDescriptorIndication_Error_gHciRemoteUserTerminatedConnection_c:
                    shell_write(gHciRemoteUserTerminatedConnection_c);
                    break;

                case GATTClientProcedureWriteCharacteristicDescriptorIndication_Error_gHciRemoteDeviceTerminatedConnectionLowResources_c:
                    shell_write(gHciRemoteDeviceTerminatedConnectionLowResources_c);
                    break;

                case GATTClientProcedureWriteCharacteristicDescriptorIndication_Error_gHciRemoteDeviceTerminatedConnectionPowerOff_c:
                    shell_write(gHciRemoteDeviceTerminatedConnectionPowerOff_c);
                    break;

                case GATTClientProcedureWriteCharacteristicDescriptorIndication_Error_gHciConnectionTerminatedByLocalHost_c:
                    shell_write(gHciConnectionTerminatedByLocalHost_c);
                    break;

                case GATTClientProcedureWriteCharacteristicDescriptorIndication_Error_gHciRepeatedAttempts_c:
                    shell_write(gHciRepeatedAttempts_c);
                    break;

                case GATTClientProcedureWriteCharacteristicDescriptorIndication_Error_gHciPairingNotAllowed_c:
                    shell_write(gHciPairingNotAllowed_c);
                    break;

                case GATTClientProcedureWriteCharacteristicDescriptorIndication_Error_gHciUnknownLpmPdu_c:
                    shell_write(gHciUnknownLpmPdu_c);
                    break;

                case GATTClientProcedureWriteCharacteristicDescriptorIndication_Error_gHciUnsupportedRemoteFeature_c:
                    shell_write(gHciUnsupportedRemoteFeature_c);
                    break;

                case GATTClientProcedureWriteCharacteristicDescriptorIndication_Error_gHciScoOffsetRejected_c:
                    shell_write(gHciScoOffsetRejected_c);
                    break;

                case GATTClientProcedureWriteCharacteristicDescriptorIndication_Error_gHciScoIntervalRejected_c:
                    shell_write(gHciScoIntervalRejected_c);
                    break;

                case GATTClientProcedureWriteCharacteristicDescriptorIndication_Error_gHciScoAirModeRejected_c:
                    shell_write(gHciScoAirModeRejected_c);
                    break;

                case GATTClientProcedureWriteCharacteristicDescriptorIndication_Error_gHciInvalidLpmParameters_c:
                    shell_write(gHciInvalidLpmParameters_c);
                    break;

                case GATTClientProcedureWriteCharacteristicDescriptorIndication_Error_gHciUnspecifiedError_c:
                    shell_write(gHciUnspecifiedc);
                    break;

                case GATTClientProcedureWriteCharacteristicDescriptorIndication_Error_gHciUnsupportedLpmParameterValue_c:
                    shell_write(gHciUnsupportedLpmParameterValue_c);
                    break;

                case GATTClientProcedureWriteCharacteristicDescriptorIndication_Error_gHciRoleChangeNotAllowed_c:
                    shell_write(gHciRoleChangeNotAllowed_c);
                    break;

                case GATTClientProcedureWriteCharacteristicDescriptorIndication_Error_gHciLLResponseTimeout_c:
                    shell_write(gHciLLResponseTimeout_c);
                    break;

                case GATTClientProcedureWriteCharacteristicDescriptorIndication_Error_gHciLmpErrorTransactionCollision_c:
                    shell_write(gHciLmpErrorTransactionCollision_c);
                    break;

                case GATTClientProcedureWriteCharacteristicDescriptorIndication_Error_gHciLmpPduNotAllowed_c:
                    shell_write(gHciLmpPduNotAllowed_c);
                    break;

                case GATTClientProcedureWriteCharacteristicDescriptorIndication_Error_gHciEncryptionModeNotAcceptable_c:
                    shell_write(gHciEncryptionModeNotAcceptable_c);
                    break;

                case GATTClientProcedureWriteCharacteristicDescriptorIndication_Error_gHciLinkKeyCannotBeChanged_c:
                    shell_write(gHciLinkKeyCannotBeChanged_c);
                    break;

                case GATTClientProcedureWriteCharacteristicDescriptorIndication_Error_gHciRequestedQosNotSupported_c:
                    shell_write(gHciRequestedQosNotSupported_c);
                    break;

                case GATTClientProcedureWriteCharacteristicDescriptorIndication_Error_gHciInstantPassed_c:
                    shell_write(gHciInstantPassed_c);
                    break;

                case GATTClientProcedureWriteCharacteristicDescriptorIndication_Error_gHciPairingWithUnitKeyNotSupported_c:
                    shell_write(gHciPairingWithUnitKeyNotSupported_c);
                    break;

                case GATTClientProcedureWriteCharacteristicDescriptorIndication_Error_gHciDifferentTransactionCollision_c:
                    shell_write(gHciDifferentTransactionCollision_c);
                    break;

                case GATTClientProcedureWriteCharacteristicDescriptorIndication_Error_gHciReserved_0x2B_c:
                    shell_write(gHciReserved_0x2B_c);
                    break;

                case GATTClientProcedureWriteCharacteristicDescriptorIndication_Error_gHciQosNotAcceptableParameter_c:
                    shell_write(gHciQosNotAcceptableParameter_c);
                    break;

                case GATTClientProcedureWriteCharacteristicDescriptorIndication_Error_gHciQosRejected_c:
                    shell_write(gHciQosRejected_c);
                    break;

                case GATTClientProcedureWriteCharacteristicDescriptorIndication_Error_gHciChannelClassificationNotSupported_c:
                    shell_write(gHciChannelClassificationNotSupported_c);
                    break;

                case GATTClientProcedureWriteCharacteristicDescriptorIndication_Error_gHciInsufficientSecurity_c:
                    shell_write(gHciInsufficientSecurity_c);
                    break;

                case GATTClientProcedureWriteCharacteristicDescriptorIndication_Error_gHciParameterOutOfMandatoryRange_c:
                    shell_write(gHciParameterOutOfMandatoryRange_c);
                    break;

                case GATTClientProcedureWriteCharacteristicDescriptorIndication_Error_gHciReserved_0x31_c:
                    shell_write(gHciReserved_0x31_c);
                    break;

                case GATTClientProcedureWriteCharacteristicDescriptorIndication_Error_gHciRoleSwitchPending_c:
                    shell_write(gHciRoleSwitchPending_c);
                    break;

                case GATTClientProcedureWriteCharacteristicDescriptorIndication_Error_gHciReserved_0x33_c:
                    shell_write(gHciReserved_0x33_c);
                    break;

                case GATTClientProcedureWriteCharacteristicDescriptorIndication_Error_gHciReservedSlotViolation_c:
                    shell_write(gHciReservedSlotViolation_c);
                    break;

                case GATTClientProcedureWriteCharacteristicDescriptorIndication_Error_gHciRoleSwitchFailed_c:
                    shell_write(gHciRoleSwitchFailed_c);
                    break;

                case GATTClientProcedureWriteCharacteristicDescriptorIndication_Error_gHciExtendedInquiryResponseTooLarge_c:
                    shell_write(gHciExtendedInquiryResponseTooLarge_c);
                    break;

                case GATTClientProcedureWriteCharacteristicDescriptorIndication_Error_gHciSecureSimplePairingNotSupportedByHost_c:
                    shell_write(gHciSecureSimplePairingNotSupportedByHost_c);
                    break;

                case GATTClientProcedureWriteCharacteristicDescriptorIndication_Error_gHciHostBusyPairing_c:
                    shell_write(gHciHostBusyPairing_c);
                    break;

                case GATTClientProcedureWriteCharacteristicDescriptorIndication_Error_gHciConnectionRejectedDueToNoSuitableChannelFound_c:
                    shell_write(gHciConnectionRejectedDueToNoSuitableChannelFound_c);
                    break;

                case GATTClientProcedureWriteCharacteristicDescriptorIndication_Error_gHciControllerBusy_c:
                    shell_write(gHciControllerBusy_c);
                    break;

                case GATTClientProcedureWriteCharacteristicDescriptorIndication_Error_gHciUnacceptableConnectionParameters_c:
                    shell_write(gHciUnacceptableConnectionParameters_c);
                    break;

                case GATTClientProcedureWriteCharacteristicDescriptorIndication_Error_gHciDirectedAdvertisingTimeout_c:
                    shell_write(gHciDirectedAdvertisingTimeout_c);
                    break;

                case GATTClientProcedureWriteCharacteristicDescriptorIndication_Error_gHciConnectionTerminatedDueToMicFailure_c:
                    shell_write(gHciConnectionTerminatedDueToMicFailure_c);
                    break;

                case GATTClientProcedureWriteCharacteristicDescriptorIndication_Error_gHciConnectionFailedToBeEstablished_c:
                    shell_write(gHciConnectionFailedToBeEstablished_c);
                    break;

                case GATTClientProcedureWriteCharacteristicDescriptorIndication_Error_gHciMacConnectionFailed_c:
                    shell_write(gHciMacConnectionFailed_c);
                    break;

                case GATTClientProcedureWriteCharacteristicDescriptorIndication_Error_gHciCoarseClockAdjustmentRejected_c:
                    shell_write(gHciCoarseClockAdjustmentRejected_c);
                    break;

                case GATTClientProcedureWriteCharacteristicDescriptorIndication_Error_gHciAlreadyInit_c:
                    shell_write(gHciAlreadyInit_c);
                    break;

                case GATTClientProcedureWriteCharacteristicDescriptorIndication_Error_gHciInvalidParameter_c:
                    shell_write(gHciInvalidParameter_c);
                    break;

                case GATTClientProcedureWriteCharacteristicDescriptorIndication_Error_gHciCallbackNotInstalled_c:
                    shell_write(gHciCallbackNotInstalled_c);
                    break;

                case GATTClientProcedureWriteCharacteristicDescriptorIndication_Error_gHciCallbackAlreadyInstalled_c:
                    shell_write(gHciCallbackAlreadyInstalled_c);
                    break;

                case GATTClientProcedureWriteCharacteristicDescriptorIndication_Error_gHciCommandNotSupported_c:
                    shell_write(gHciCommandNotSupported_c);
                    break;

                case GATTClientProcedureWriteCharacteristicDescriptorIndication_Error_gHciEventNotSupported_c:
                    shell_write(gHciEventNotSupported_c);
                    break;

                case GATTClientProcedureWriteCharacteristicDescriptorIndication_Error_gHciTransportError_c:
                    shell_write(gHciTransportc);
                    break;

                case GATTClientProcedureWriteCharacteristicDescriptorIndication_Error_gL2caAlreadyInit_c:
                    shell_write(gL2caAlreadyInit_c);
                    break;

                case GATTClientProcedureWriteCharacteristicDescriptorIndication_Error_gL2caInsufficientResources_c:
                    shell_write(gL2caInsufficientResources_c);
                    break;

                case GATTClientProcedureWriteCharacteristicDescriptorIndication_Error_gL2caCallbackNotInstalled_c:
                    shell_write(gL2caCallbackNotInstalled_c);
                    break;

                case GATTClientProcedureWriteCharacteristicDescriptorIndication_Error_gL2caCallbackAlreadyInstalled_c:
                    shell_write(gL2caCallbackAlreadyInstalled_c);
                    break;

                case GATTClientProcedureWriteCharacteristicDescriptorIndication_Error_gL2caLePsmInvalid_c:
                    shell_write(gL2caLePsmInvalid_c);
                    break;

                case GATTClientProcedureWriteCharacteristicDescriptorIndication_Error_gL2caLePsmAlreadyRegistered_c:
                    shell_write(gL2caLePsmAlreadyRegistered_c);
                    break;

                case GATTClientProcedureWriteCharacteristicDescriptorIndication_Error_gL2caLePsmNotRegistered_c:
                    shell_write(gL2caLePsmNotRegistered_c);
                    break;

                case GATTClientProcedureWriteCharacteristicDescriptorIndication_Error_gL2caLePsmInsufficientResources_c:
                    shell_write(gL2caLePsmInsufficientResources_c);
                    break;

                case GATTClientProcedureWriteCharacteristicDescriptorIndication_Error_gL2caChannelInvalid_c:
                    shell_write(gL2caChannelInvalid_c);
                    break;

                case GATTClientProcedureWriteCharacteristicDescriptorIndication_Error_gL2caChannelClosed_c:
                    shell_write(gL2caChannelClosed_c);
                    break;

                case GATTClientProcedureWriteCharacteristicDescriptorIndication_Error_gL2caChannelAlreadyConnected_c:
                    shell_write(gL2caChannelAlreadyConnected_c);
                    break;

                case GATTClientProcedureWriteCharacteristicDescriptorIndication_Error_gL2caConnectionParametersRejected_c:
                    shell_write(gL2caConnectionParametersRejected_c);
                    break;

                case GATTClientProcedureWriteCharacteristicDescriptorIndication_Error_gL2caChannelBusy_c:
                    shell_write(gL2caChannelBusy_c);
                    break;

                case GATTClientProcedureWriteCharacteristicDescriptorIndication_Error_gL2caInvalidParameter_c:
                    shell_write(gL2caInvalidParameter_c);
                    break;

                case GATTClientProcedureWriteCharacteristicDescriptorIndication_Error_gL2caError_c:
                    shell_write(gL2cac);
                    break;

                case GATTClientProcedureWriteCharacteristicDescriptorIndication_Error_gSmNullCBFunction_c:
                    shell_write(gSmNullCBFunction_c);
                    break;

                case GATTClientProcedureWriteCharacteristicDescriptorIndication_Error_gSmCommandNotSupported_c:
                    shell_write(gSmCommandNotSupported_c);
                    break;

                case GATTClientProcedureWriteCharacteristicDescriptorIndication_Error_gSmUnexpectedCommand_c:
                    shell_write(gSmUnexpectedCommand_c);
                    break;

                case GATTClientProcedureWriteCharacteristicDescriptorIndication_Error_gSmInvalidCommandCode_c:
                    shell_write(gSmInvalidCommandCode_c);
                    break;

                case GATTClientProcedureWriteCharacteristicDescriptorIndication_Error_gSmInvalidCommandLength_c:
                    shell_write(gSmInvalidCommandLength_c);
                    break;

                case GATTClientProcedureWriteCharacteristicDescriptorIndication_Error_gSmInvalidCommandParameter_c:
                    shell_write(gSmInvalidCommandParameter_c);
                    break;

                case GATTClientProcedureWriteCharacteristicDescriptorIndication_Error_gSmInvalidDeviceId_c:
                    shell_write(gSmInvalidDeviceId_c);
                    break;

                case GATTClientProcedureWriteCharacteristicDescriptorIndication_Error_gSmInvalidInternalOperation_c:
                    shell_write(gSmInvalidInternalOperation_c);
                    break;

                case GATTClientProcedureWriteCharacteristicDescriptorIndication_Error_gSmInvalidConnectionHandle_c:
                    shell_write(gSmInvalidConnectionHandle_c);
                    break;

                case GATTClientProcedureWriteCharacteristicDescriptorIndication_Error_gSmInproperKeyDistributionField_c:
                    shell_write(gSmInproperKeyDistributionField_c);
                    break;

                case GATTClientProcedureWriteCharacteristicDescriptorIndication_Error_gSmUnexpectedKeyType_c:
                    shell_write(gSmUnexpectedKeyType_c);
                    break;

                case GATTClientProcedureWriteCharacteristicDescriptorIndication_Error_gSmUnexpectedPairingTerminationReason_c:
                    shell_write(gSmUnexpectedPairingTerminationReason_c);
                    break;

                case GATTClientProcedureWriteCharacteristicDescriptorIndication_Error_gSmUnexpectedKeyset_c:
                    shell_write(gSmUnexpectedKeyset_c);
                    break;

                case GATTClientProcedureWriteCharacteristicDescriptorIndication_Error_gSmSmpTimeoutOccurred_c:
                    shell_write(gSmSmpTimeoutOccurred_c);
                    break;

                case GATTClientProcedureWriteCharacteristicDescriptorIndication_Error_gSmUnknownSmpPacketType_c:
                    shell_write(gSmUnknownSmpPacketType_c);
                    break;

                case GATTClientProcedureWriteCharacteristicDescriptorIndication_Error_gSmInvalidSmpPacketLength_c:
                    shell_write(gSmInvalidSmpPacketLength_c);
                    break;

                case GATTClientProcedureWriteCharacteristicDescriptorIndication_Error_gSmInvalidSmpPacketParameter_c:
                    shell_write(gSmInvalidSmpPacketParameter_c);
                    break;

                case GATTClientProcedureWriteCharacteristicDescriptorIndication_Error_gSmReceivedUnexpectedSmpPacket_c:
                    shell_write(gSmReceivedUnexpectedSmpPacket_c);
                    break;

                case GATTClientProcedureWriteCharacteristicDescriptorIndication_Error_gSmReceivedSmpPacketFromUnknownDevice_c:
                    shell_write(gSmReceivedSmpPacketFromUnknownDevice_c);
                    break;

                case GATTClientProcedureWriteCharacteristicDescriptorIndication_Error_gSmReceivedUnexpectedHciEvent_c:
                    shell_write(gSmReceivedUnexpectedHciEvent_c);
                    break;

                case GATTClientProcedureWriteCharacteristicDescriptorIndication_Error_gSmReceivedHciEventFromUnknownDevice_c:
                    shell_write(gSmReceivedHciEventFromUnknownDevice_c);
                    break;

                case GATTClientProcedureWriteCharacteristicDescriptorIndication_Error_gSmInvalidHciEventParameter_c:
                    shell_write(gSmInvalidHciEventParameter_c);
                    break;

                case GATTClientProcedureWriteCharacteristicDescriptorIndication_Error_gSmLlConnectionEncryptionInProgress_c:
                    shell_write(gSmLlConnectionEncryptionInProgress_c);
                    break;

                case GATTClientProcedureWriteCharacteristicDescriptorIndication_Error_gSmLlConnectionEncryptionFailure_c:
                    shell_write(gSmLlConnectionEncryptionFailure_c);
                    break;

                case GATTClientProcedureWriteCharacteristicDescriptorIndication_Error_gSmInsufficientResources_c:
                    shell_write(gSmInsufficientResources_c);
                    break;

                case GATTClientProcedureWriteCharacteristicDescriptorIndication_Error_gSmOobDataAddressMismatch_c:
                    shell_write(gSmOobDataAddressMismatch_c);
                    break;

//                case GATTClientProcedureWriteCharacteristicDescriptorIndication_Error_gSmSmpPacketReceivedAfterTimeoutOccurred_c:
//                    shell_write(gSmSmpPacketReceivedAfterTimeoutOccurred_c);
//                    break;

                case GATTClientProcedureWriteCharacteristicDescriptorIndication_Error_gSmPairingErrorPasskeyEntryFailed_c:
                    shell_write(gSmPairingErrorPasskeyEntryFailed_c);
                    break;

                case GATTClientProcedureWriteCharacteristicDescriptorIndication_Error_gSmPairingErrorConfirmValueFailed_c:
                    shell_write(gSmPairingErrorConfirmValueFailed_c);
                    break;

                case GATTClientProcedureWriteCharacteristicDescriptorIndication_Error_gSmPairingErrorCommandNotSupported_c:
                    shell_write(gSmPairingErrorCommandNotSupported_c);
                    break;

                case GATTClientProcedureWriteCharacteristicDescriptorIndication_Error_gSmPairingErrorInvalidParameters_c:
                    shell_write(gSmPairingErrorInvalidParameters_c);
                    break;

                case GATTClientProcedureWriteCharacteristicDescriptorIndication_Error_gSmPairingErrorUnknownReason_c:
                    shell_write(gSmPairingErrorUnknownReason_c);
                    break;

                case GATTClientProcedureWriteCharacteristicDescriptorIndication_Error_gSmTbResolvableAddressDoesNotMatchIrk_c:
                    shell_write(gSmTbResolvableAddressDoesNotMatchIrk_c);
                    break;

                case GATTClientProcedureWriteCharacteristicDescriptorIndication_Error_gSmTbInvalidDataSignature_c:
                    shell_write(gSmTbInvalidDataSignature_c);
                    break;

                case GATTClientProcedureWriteCharacteristicDescriptorIndication_Error_gAttInvalidHandle_c:
                    shell_write(gAttInvalidHandle_c);
                    break;

                case GATTClientProcedureWriteCharacteristicDescriptorIndication_Error_gAttReadNotPermitted_c:
                    shell_write(gAttReadNotPermitted_c);
                    break;

                case GATTClientProcedureWriteCharacteristicDescriptorIndication_Error_gAttWriteNotPermitted_c:
                    shell_write(gAttWriteNotPermitted_c);
                    break;

                case GATTClientProcedureWriteCharacteristicDescriptorIndication_Error_gAttInvalidPdu_c:
                    shell_write(gAttInvalidPdu_c);
                    break;

                case GATTClientProcedureWriteCharacteristicDescriptorIndication_Error_gAttInsufficientAuthentication_c:
                    shell_write(gAttInsufficientAuthentication_c);
                    break;

                case GATTClientProcedureWriteCharacteristicDescriptorIndication_Error_gAttRequestNotSupported_c:
                    shell_write(gAttRequestNotSupported_c);
                    break;

                case GATTClientProcedureWriteCharacteristicDescriptorIndication_Error_gAttInvalidOffset_c:
                    shell_write(gAttInvalidOffset_c);
                    break;

                case GATTClientProcedureWriteCharacteristicDescriptorIndication_Error_gAttInsufficientAuthorization_c:
                    shell_write(gAttInsufficientAuthorization_c);
                    break;

                case GATTClientProcedureWriteCharacteristicDescriptorIndication_Error_gAttPrepareQueueFull_c:
                    shell_write(gAttPrepareQueueFull_c);
                    break;

                case GATTClientProcedureWriteCharacteristicDescriptorIndication_Error_gAttAttributeNotFound_c:
                    shell_write(gAttAttributeNotFound_c);
                    break;

                case GATTClientProcedureWriteCharacteristicDescriptorIndication_Error_gAttAttributeNotLong_c:
                    shell_write(gAttAttributeNotLong_c);
                    break;

                case GATTClientProcedureWriteCharacteristicDescriptorIndication_Error_gAttInsufficientEncryptionKeySize_c:
                    shell_write(gAttInsufficientEncryptionKeySize_c);
                    break;

                case GATTClientProcedureWriteCharacteristicDescriptorIndication_Error_gAttInvalidAttributeValueLength_c:
                    shell_write(gAttInvalidAttributeValueLength_c);
                    break;

                case GATTClientProcedureWriteCharacteristicDescriptorIndication_Error_gAttUnlikelyor_c:
                    shell_write(gAttUnlikelyor_c);
                    break;

                case GATTClientProcedureWriteCharacteristicDescriptorIndication_Error_gAttInsufficientEncryption_c:
                    shell_write(gAttInsufficientEncryption_c);
                    break;

                case GATTClientProcedureWriteCharacteristicDescriptorIndication_Error_gAttUnsupportedGroupType_c:
                    shell_write(gAttUnsupportedGroupType_c);
                    break;

                case GATTClientProcedureWriteCharacteristicDescriptorIndication_Error_gAttInsufficientResources_c:
                    shell_write(gAttInsufficientResources_c);
                    break;

                case GATTClientProcedureWriteCharacteristicDescriptorIndication_Error_gGattAnotherProcedureInProgress_c:
                    shell_write(gGattAnotherProcedureInProgress_c);
                    break;

                case GATTClientProcedureWriteCharacteristicDescriptorIndication_Error_gGattLongAttributePacketsCorrupted_c:
                    shell_write(gGattLongAttributePacketsCorrupted_c);
                    break;

                case GATTClientProcedureWriteCharacteristicDescriptorIndication_Error_gGattMultipleAttributesOverflow_c:
                    shell_write(gGattMultipleAttributesOverflow_c);
                    break;

                case GATTClientProcedureWriteCharacteristicDescriptorIndication_Error_gGattUnexpectedReadMultipleResponseLength_c:
                    shell_write(gGattUnexpectedReadMultipleResponseLength_c);
                    break;

                case GATTClientProcedureWriteCharacteristicDescriptorIndication_Error_gGattInvalidValueLength_c:
                    shell_write(gGattInvalidValueLength_c);
                    break;

                case GATTClientProcedureWriteCharacteristicDescriptorIndication_Error_gGattServerTimeout_c:
                    shell_write(gGattServerTimeout_c);
                    break;

                case GATTClientProcedureWriteCharacteristicDescriptorIndication_Error_gGattIndicationAlreadyInProgress_c:
                    shell_write(gGattIndicationAlreadyInProgress_c);
                    break;

                case GATTClientProcedureWriteCharacteristicDescriptorIndication_Error_gGattClientConfirmationTimeout_c:
                    shell_write(gGattClientConfirmationTimeout_c);
                    break;

                case GATTClientProcedureWriteCharacteristicDescriptorIndication_Error_gGapAdvDataTooLong_c:
                    shell_write(gGapAdvDataTooLong_c);
                    break;

                case GATTClientProcedureWriteCharacteristicDescriptorIndication_Error_gGapScanRspDataTooLong_c:
                    shell_write(gGapScanRspDataTooLong_c);
                    break;

                case GATTClientProcedureWriteCharacteristicDescriptorIndication_Error_gGapDeviceNotBonded_c:
                    shell_write(gGapDeviceNotBonded_c);
                    break;

                case GATTClientProcedureWriteCharacteristicDescriptorIndication_Error_gDevDbCccdLimitReached_c:
                    shell_write(gDevDbCccdLimitReached_c);
                    break;

                case GATTClientProcedureWriteCharacteristicDescriptorIndication_Error_gDevDbCccdNotFound_c:
                    shell_write(gDevDbCccdNotFound_c);
                    break;

                case GATTClientProcedureWriteCharacteristicDescriptorIndication_Error_gGattDbInvalidHandle_c:
                    shell_write(gGattDbInvalidHandle_c);
                    break;

                case GATTClientProcedureWriteCharacteristicDescriptorIndication_Error_gGattDbCharacteristicNotFound_c:
                    shell_write(gGattDbCharacteristicNotFound_c);
                    break;

                case GATTClientProcedureWriteCharacteristicDescriptorIndication_Error_gGattDbCccdNotFound_c:
                    shell_write(gGattDbCccdNotFound_c);
                    break;

                case GATTClientProcedureWriteCharacteristicDescriptorIndication_Error_gGattDbServiceNotFound_c:
                    shell_write(gGattDbServiceNotFound_c);
                    break;

                case GATTClientProcedureWriteCharacteristicDescriptorIndication_Error_gGattDbDescriptorNotFound_c:
                    shell_write(gGattDbDescriptorNotFound_c);
                    break;

                default:
                    shell_printf("Unrecognized status 0x%04X", container->Data.GATTClientProcedureWriteCharacteristicDescriptorIndication.Error);
                    break;
            }

            break;

        case 0x448F:
            shell_write("\r\nGATTClientNotificationIndication");
            SHELL_NEWLINE();
            shell_printf("\tCharacteristic Value Handle -> %d", container->Data.GATTClientNotificationIndication.CharacteristicValueHandle);
            SHELL_NEWLINE();
            shell_write("\tValue -> ");
            SHELL_PrintBuff(container->Data.GATTClientNotificationIndication.Value, container->Data.GATTClientNotificationIndication.ValueLength);
            MEM_BufferFree(container->Data.GATTClientNotificationIndication.Value);
            break;

        case 0x4490:
            shell_write("\r\nGATTClientIndicationIndication");
            break;

        case 0x4491:
            shell_write("\r\nGATTServerMtuChangedIndication");
            BleApp_AttMtuChanged(container->Data.GATTServerMtuChangedIndication.DeviceId,
                                 container->Data.GATTServerMtuChangedIndication.MtuChangedEvent_NewMtu);
            break;

        case 0x4492:
            if (!mSuppressBleEventPrint)
            {
                shell_write("\r\nGATTServerHandleValueConfirmationIndication");
            }
            BleApp_HandleValueConfirmation(container->Data.GATTServerHandleValueConfirmationIndication.DeviceId);
            break;

        case 0x4493:
            shell_write("\r\nGATTServerAttributeWrittenIndication");
            BleApp_DemoRpkWriteCallback(&container->Data.GATTServerAttributeWrittenIndication);
            break;

        case 0x4494:
            shell_write("\r\nGATTServerCharacteristicCccdWrittenIndication");
            BleApp_CccdWritten(container->Data.GATTServerCharacteristicCccdWrittenIndication.DeviceId,
                               container->Data.GATTServerCharacteristicCccdWrittenIndication.CharCccdWrittenEvent.Handle);

            break;

        case 0x4495:
            BleApp_AttributeWrittenWithoutResponse(container->Data.GATTServerAttributeWrittenWithoutResponseIndication.DeviceId,
                            container->Data.GATTServerAttributeWrittenWithoutResponseIndication.AttributeWrittenEvent.Handle,
                            container->Data.GATTServerAttributeWrittenWithoutResponseIndication.AttributeWrittenEvent.ValueLength,
                            container->Data.GATTServerAttributeWrittenWithoutResponseIndication.AttributeWrittenEvent.Value);
            MEM_BufferFree(container->Data.GATTServerAttributeWrittenWithoutResponseIndication.AttributeWrittenEvent.Value);
            break;

        case 0x4496:
            shell_write("\r\nGATTServerErrorIndication ");
            shell_writeHex((uint8_t *)(&(container->Data.GATTServerErrorIndication.ProcedureError.Error)),2);
            shell_writeHex(&(container->Data.GATTServerErrorIndication.ProcedureError.ProcedureType),1);
            break;

        case 0x4497:
            shell_write("\r\nGATTServerLongCharacteristicWrittenIndication");
            break;

        case 0x4498:
            shell_write("\r\nGATTServerAttributeReadIndication");
            break;

        case 0x4580:

            if (mSuppressBleEventPrint)
            {
                break;
            }

            shell_write("\r\nGATTDBConfirm");
            shell_write(" -> ");

            switch (container->Data.GATTDBConfirm.Status)
            {
                case GATTDBConfirm_Status_gBleSuccess_c:
                    shell_write(gBleSuccess_c);
                    break;

                case GATTDBConfirm_Status_gBleInvalidParameter_c:
                    shell_write(gBleInvalidParameter_c);
                    break;

                case GATTDBConfirm_Status_gBleOverflow_c:
                    shell_write(gBleOverflow_c);
                    break;

                case GATTDBConfirm_Status_gBleUnavailable_c:
                    shell_write(gBleUnavailable_c);
                    break;

                case GATTDBConfirm_Status_gBleFeatureNotSupported_c:
                    shell_write(gBleFeatureNotSupported_c);
                    break;

                case GATTDBConfirm_Status_gBleOutOfMemory_c:
                    shell_write(gBleOutOfMemory_c);
                    break;

                case GATTDBConfirm_Status_gBleAlreadyInitialized_c:
                    shell_write(gBleAlreadyInitialized_c);
                    break;

                case GATTDBConfirm_Status_gBleOsError_c:
                    shell_write(gBleOsError_c);
                    break;

                case GATTDBConfirm_Status_gBleUnexpectedError_c:
                    shell_write(gBleUnexpectedError_c);
                    break;

                case GATTDBConfirm_Status_gBleInvalidState_c:
                    shell_write(gBleInvalidState_c);
                    break;

                case GATTDBConfirm_Status_gHciUnknownHciCommand_c:
                    shell_write(gHciUnknownHciCommand_c);
                    break;

                case GATTDBConfirm_Status_gHciUnknownConnectionIdentifier_c:
                    shell_write(gHciUnknownConnectionIdentifier_c);
                    break;

                case GATTDBConfirm_Status_gHciHardwareFailure_c:
                    shell_write(gHciHardwareFailure_c);
                    break;

                case GATTDBConfirm_Status_gHciPageTimeout_c:
                    shell_write(gHciPageTimeout_c);
                    break;

                case GATTDBConfirm_Status_gHciAuthenticationFailure_c:
                    shell_write(gHciAuthenticationFailure_c);
                    break;

                case GATTDBConfirm_Status_gHciPinOrKeyMissing_c:
                    shell_write(gHciPinOrKeyMissing_c);
                    break;

                case GATTDBConfirm_Status_gHciMemoryCapacityExceeded_c:
                    shell_write(gHciMemoryCapacityExceeded_c);
                    break;

                case GATTDBConfirm_Status_gHciConnectionTimeout_c:
                    shell_write(gHciConnectionTimeout_c);
                    break;

                case GATTDBConfirm_Status_gHciConnectionLimitExceeded_c:
                    shell_write(gHciConnectionLimitExceeded_c);
                    break;

                case GATTDBConfirm_Status_gHciSynchronousConnectionLimitToADeviceExceeded_c:
                    shell_write(gHciSynchronousConnectionLimitToADeviceExceeded_c);
                    break;

                case GATTDBConfirm_Status_gHciAclConnectionAlreadyExists_c:
                    shell_write(gHciAclConnectionAlreadyExists_c);
                    break;

                case GATTDBConfirm_Status_gHciCommandDisallowed_c:
                    shell_write(gHciCommandDisallowed_c);
                    break;

                case GATTDBConfirm_Status_gHciConnectionRejectedDueToLimitedResources_c:
                    shell_write(gHciConnectionRejectedDueToLimitedResources_c);
                    break;

                case GATTDBConfirm_Status_gHciConnectionRejectedDueToSecurityReasons_c:
                    shell_write(gHciConnectionRejectedDueToSecurityReasons_c);
                    break;

                case GATTDBConfirm_Status_gHciConnectionRejectedDueToUnacceptableBdAddr_c:
                    shell_write(gHciConnectionRejectedDueToUnacceptableBdAddr_c);
                    break;

                case GATTDBConfirm_Status_gHciConnectionAcceptTimeoutExceeded_c:
                    shell_write(gHciConnectionAcceptTimeoutExceeded_c);
                    break;

                case GATTDBConfirm_Status_gHciUnsupportedFeatureOrParameterValue_c:
                    shell_write(gHciUnsupportedFeatureOrParameterValue_c);
                    break;

                case GATTDBConfirm_Status_gHciInvalidHciCommandParameters_c:
                    shell_write(gHciInvalidHciCommandParameters_c);
                    break;

                case GATTDBConfirm_Status_gHciRemoteUserTerminatedConnection_c:
                    shell_write(gHciRemoteUserTerminatedConnection_c);
                    break;

                case GATTDBConfirm_Status_gHciRemoteDeviceTerminatedConnectionLowResources_c:
                    shell_write(gHciRemoteDeviceTerminatedConnectionLowResources_c);
                    break;

                case GATTDBConfirm_Status_gHciRemoteDeviceTerminatedConnectionPowerOff_c:
                    shell_write(gHciRemoteDeviceTerminatedConnectionPowerOff_c);
                    break;

                case GATTDBConfirm_Status_gHciConnectionTerminatedByLocalHost_c:
                    shell_write(gHciConnectionTerminatedByLocalHost_c);
                    break;

                case GATTDBConfirm_Status_gHciRepeatedAttempts_c:
                    shell_write(gHciRepeatedAttempts_c);
                    break;

                case GATTDBConfirm_Status_gHciPairingNotAllowed_c:
                    shell_write(gHciPairingNotAllowed_c);
                    break;

                case GATTDBConfirm_Status_gHciUnknownLpmPdu_c:
                    shell_write(gHciUnknownLpmPdu_c);
                    break;

                case GATTDBConfirm_Status_gHciUnsupportedRemoteFeature_c:
                    shell_write(gHciUnsupportedRemoteFeature_c);
                    break;

                case GATTDBConfirm_Status_gHciScoOffsetRejected_c:
                    shell_write(gHciScoOffsetRejected_c);
                    break;

                case GATTDBConfirm_Status_gHciScoIntervalRejected_c:
                    shell_write(gHciScoIntervalRejected_c);
                    break;

                case GATTDBConfirm_Status_gHciScoAirModeRejected_c:
                    shell_write(gHciScoAirModeRejected_c);
                    break;

                case GATTDBConfirm_Status_gHciInvalidLpmParameters_c:
                    shell_write(gHciInvalidLpmParameters_c);
                    break;

                case GATTDBConfirm_Status_gHciUnspecifiedError_c:
                    shell_write(gHciUnspecifiedError_c);
                    break;

                case GATTDBConfirm_Status_gHciUnsupportedLpmParameterValue_c:
                    shell_write(gHciUnsupportedLpmParameterValue_c);
                    break;

                case GATTDBConfirm_Status_gHciRoleChangeNotAllowed_c:
                    shell_write(gHciRoleChangeNotAllowed_c);
                    break;

                case GATTDBConfirm_Status_gHciLLResponseTimeout_c:
                    shell_write(gHciLLResponseTimeout_c);
                    break;

                case GATTDBConfirm_Status_gHciLmpErrorTransactionCollision_c:
                    shell_write(gHciLmpErrorTransactionCollision_c);
                    break;

                case GATTDBConfirm_Status_gHciLmpPduNotAllowed_c:
                    shell_write(gHciLmpPduNotAllowed_c);
                    break;

                case GATTDBConfirm_Status_gHciEncryptionModeNotAcceptable_c:
                    shell_write(gHciEncryptionModeNotAcceptable_c);
                    break;

                case GATTDBConfirm_Status_gHciLinkKeyCannotBeChanged_c:
                    shell_write(gHciLinkKeyCannotBeChanged_c);
                    break;

                case GATTDBConfirm_Status_gHciRequestedQosNotSupported_c:
                    shell_write(gHciRequestedQosNotSupported_c);
                    break;

                case GATTDBConfirm_Status_gHciInstantPassed_c:
                    shell_write(gHciInstantPassed_c);
                    break;

                case GATTDBConfirm_Status_gHciPairingWithUnitKeyNotSupported_c:
                    shell_write(gHciPairingWithUnitKeyNotSupported_c);
                    break;

                case GATTDBConfirm_Status_gHciDifferentTransactionCollision_c:
                    shell_write(gHciDifferentTransactionCollision_c);
                    break;

                case GATTDBConfirm_Status_gHciReserved_0x2B_c:
                    shell_write(gHciReserved_0x2B_c);
                    break;

                case GATTDBConfirm_Status_gHciQosNotAcceptableParameter_c:
                    shell_write(gHciQosNotAcceptableParameter_c);
                    break;

                case GATTDBConfirm_Status_gHciQosRejected_c:
                    shell_write(gHciQosRejected_c);
                    break;

                case GATTDBConfirm_Status_gHciChannelClassificationNotSupported_c:
                    shell_write(gHciChannelClassificationNotSupported_c);
                    break;

                case GATTDBConfirm_Status_gHciInsufficientSecurity_c:
                    shell_write(gHciInsufficientSecurity_c);
                    break;

                case GATTDBConfirm_Status_gHciParameterOutOfMandatoryRange_c:
                    shell_write(gHciParameterOutOfMandatoryRange_c);
                    break;

                case GATTDBConfirm_Status_gHciReserved_0x31_c:
                    shell_write(gHciReserved_0x31_c);
                    break;

                case GATTDBConfirm_Status_gHciRoleSwitchPending_c:
                    shell_write(gHciRoleSwitchPending_c);
                    break;

                case GATTDBConfirm_Status_gHciReserved_0x33_c:
                    shell_write(gHciReserved_0x33_c);
                    break;

                case GATTDBConfirm_Status_gHciReservedSlotViolation_c:
                    shell_write(gHciReservedSlotViolation_c);
                    break;

                case GATTDBConfirm_Status_gHciRoleSwitchFailed_c:
                    shell_write(gHciRoleSwitchFailed_c);
                    break;

                case GATTDBConfirm_Status_gHciExtendedInquiryResponseTooLarge_c:
                    shell_write(gHciExtendedInquiryResponseTooLarge_c);
                    break;

                case GATTDBConfirm_Status_gHciSecureSimplePairingNotSupportedByHost_c:
                    shell_write(gHciSecureSimplePairingNotSupportedByHost_c);
                    break;

                case GATTDBConfirm_Status_gHciHostBusyPairing_c:
                    shell_write(gHciHostBusyPairing_c);
                    break;

                case GATTDBConfirm_Status_gHciConnectionRejectedDueToNoSuitableChannelFound_c:
                    shell_write(gHciConnectionRejectedDueToNoSuitableChannelFound_c);
                    break;

                case GATTDBConfirm_Status_gHciControllerBusy_c:
                    shell_write(gHciControllerBusy_c);
                    break;

                case GATTDBConfirm_Status_gHciUnacceptableConnectionParameters_c:
                    shell_write(gHciUnacceptableConnectionParameters_c);
                    break;

                case GATTDBConfirm_Status_gHciDirectedAdvertisingTimeout_c:
                    shell_write(gHciDirectedAdvertisingTimeout_c);
                    break;

                case GATTDBConfirm_Status_gHciConnectionTerminatedDueToMicFailure_c:
                    shell_write(gHciConnectionTerminatedDueToMicFailure_c);
                    break;

                case GATTDBConfirm_Status_gHciConnectionFailedToBeEstablished_c:
                    shell_write(gHciConnectionFailedToBeEstablished_c);
                    break;

                case GATTDBConfirm_Status_gHciMacConnectionFailed_c:
                    shell_write(gHciMacConnectionFailed_c);
                    break;

                case GATTDBConfirm_Status_gHciCoarseClockAdjustmentRejected_c:
                    shell_write(gHciCoarseClockAdjustmentRejected_c);
                    break;

                case GATTDBConfirm_Status_gHciAlreadyInit_c:
                    shell_write(gHciAlreadyInit_c);
                    break;

                case GATTDBConfirm_Status_gHciInvalidParameter_c:
                    shell_write(gHciInvalidParameter_c);
                    break;

                case GATTDBConfirm_Status_gHciCallbackNotInstalled_c:
                    shell_write(gHciCallbackNotInstalled_c);
                    break;

                case GATTDBConfirm_Status_gHciCallbackAlreadyInstalled_c:
                    shell_write(gHciCallbackAlreadyInstalled_c);
                    break;

                case GATTDBConfirm_Status_gHciCommandNotSupported_c:
                    shell_write(gHciCommandNotSupported_c);
                    break;

                case GATTDBConfirm_Status_gHciEventNotSupported_c:
                    shell_write(gHciEventNotSupported_c);
                    break;

                case GATTDBConfirm_Status_gHciTransportError_c:
                    shell_write(gHciTransportError_c);
                    break;

                case GATTDBConfirm_Status_gL2caAlreadyInit_c:
                    shell_write(gL2caAlreadyInit_c);
                    break;

                case GATTDBConfirm_Status_gL2caInsufficientResources_c:
                    shell_write(gL2caInsufficientResources_c);
                    break;

                case GATTDBConfirm_Status_gL2caCallbackNotInstalled_c:
                    shell_write(gL2caCallbackNotInstalled_c);
                    break;

                case GATTDBConfirm_Status_gL2caCallbackAlreadyInstalled_c:
                    shell_write(gL2caCallbackAlreadyInstalled_c);
                    break;

                case GATTDBConfirm_Status_gL2caLePsmInvalid_c:
                    shell_write(gL2caLePsmInvalid_c);
                    break;

                case GATTDBConfirm_Status_gL2caLePsmAlreadyRegistered_c:
                    shell_write(gL2caLePsmAlreadyRegistered_c);
                    break;

                case GATTDBConfirm_Status_gL2caLePsmNotRegistered_c:
                    shell_write(gL2caLePsmNotRegistered_c);
                    break;

                case GATTDBConfirm_Status_gL2caLePsmInsufficientResources_c:
                    shell_write(gL2caLePsmInsufficientResources_c);
                    break;

                case GATTDBConfirm_Status_gL2caChannelInvalid_c:
                    shell_write(gL2caChannelInvalid_c);
                    break;

                case GATTDBConfirm_Status_gL2caChannelClosed_c:
                    shell_write(gL2caChannelClosed_c);
                    break;

                case GATTDBConfirm_Status_gL2caChannelAlreadyConnected_c:
                    shell_write(gL2caChannelAlreadyConnected_c);
                    break;

                case GATTDBConfirm_Status_gL2caConnectionParametersRejected_c:
                    shell_write(gL2caConnectionParametersRejected_c);
                    break;

                case GATTDBConfirm_Status_gL2caChannelBusy_c:
                    shell_write(gL2caChannelBusy_c);
                    break;

                case GATTDBConfirm_Status_gL2caInvalidParameter_c:
                    shell_write(gL2caInvalidParameter_c);
                    break;

                case GATTDBConfirm_Status_gL2caError_c:
                    shell_write(gL2caError_c);
                    break;

                case GATTDBConfirm_Status_gSmNullCBFunction_c:
                    shell_write(gSmNullCBFunction_c);
                    break;

                case GATTDBConfirm_Status_gSmCommandNotSupported_c:
                    shell_write(gSmCommandNotSupported_c);
                    break;

                case GATTDBConfirm_Status_gSmUnexpectedCommand_c:
                    shell_write(gSmUnexpectedCommand_c);
                    break;

                case GATTDBConfirm_Status_gSmInvalidCommandCode_c:
                    shell_write(gSmInvalidCommandCode_c);
                    break;

                case GATTDBConfirm_Status_gSmInvalidCommandLength_c:
                    shell_write(gSmInvalidCommandLength_c);
                    break;

                case GATTDBConfirm_Status_gSmInvalidCommandParameter_c:
                    shell_write(gSmInvalidCommandParameter_c);
                    break;

                case GATTDBConfirm_Status_gSmInvalidDeviceId_c:
                    shell_write(gSmInvalidDeviceId_c);
                    break;

                case GATTDBConfirm_Status_gSmInvalidInternalOperation_c:
                    shell_write(gSmInvalidInternalOperation_c);
                    break;

                case GATTDBConfirm_Status_gSmInvalidConnectionHandle_c:
                    shell_write(gSmInvalidConnectionHandle_c);
                    break;

                case GATTDBConfirm_Status_gSmInproperKeyDistributionField_c:
                    shell_write(gSmInproperKeyDistributionField_c);
                    break;

                case GATTDBConfirm_Status_gSmUnexpectedKeyType_c:
                    shell_write(gSmUnexpectedKeyType_c);
                    break;

                case GATTDBConfirm_Status_gSmUnexpectedPairingTerminationReason_c:
                    shell_write(gSmUnexpectedPairingTerminationReason_c);
                    break;

                case GATTDBConfirm_Status_gSmUnexpectedKeyset_c:
                    shell_write(gSmUnexpectedKeyset_c);
                    break;

                case GATTDBConfirm_Status_gSmSmpTimeoutOccurred_c:
                    shell_write(gSmSmpTimeoutOccurred_c);
                    break;

                case GATTDBConfirm_Status_gSmUnknownSmpPacketType_c:
                    shell_write(gSmUnknownSmpPacketType_c);
                    break;

                case GATTDBConfirm_Status_gSmInvalidSmpPacketLength_c:
                    shell_write(gSmInvalidSmpPacketLength_c);
                    break;

                case GATTDBConfirm_Status_gSmInvalidSmpPacketParameter_c:
                    shell_write(gSmInvalidSmpPacketParameter_c);
                    break;

                case GATTDBConfirm_Status_gSmReceivedUnexpectedSmpPacket_c:
                    shell_write(gSmReceivedUnexpectedSmpPacket_c);
                    break;

                case GATTDBConfirm_Status_gSmReceivedSmpPacketFromUnknownDevice_c:
                    shell_write(gSmReceivedSmpPacketFromUnknownDevice_c);
                    break;

                case GATTDBConfirm_Status_gSmReceivedUnexpectedHciEvent_c:
                    shell_write(gSmReceivedUnexpectedHciEvent_c);
                    break;

                case GATTDBConfirm_Status_gSmReceivedHciEventFromUnknownDevice_c:
                    shell_write(gSmReceivedHciEventFromUnknownDevice_c);
                    break;

                case GATTDBConfirm_Status_gSmInvalidHciEventParameter_c:
                    shell_write(gSmInvalidHciEventParameter_c);
                    break;

                case GATTDBConfirm_Status_gSmLlConnectionEncryptionInProgress_c:
                    shell_write(gSmLlConnectionEncryptionInProgress_c);
                    break;

                case GATTDBConfirm_Status_gSmLlConnectionEncryptionFailure_c:
                    shell_write(gSmLlConnectionEncryptionFailure_c);
                    break;

                case GATTDBConfirm_Status_gSmInsufficientResources_c:
                    shell_write(gSmInsufficientResources_c);
                    break;

                case GATTDBConfirm_Status_gSmOobDataAddressMismatch_c:
                    shell_write(gSmOobDataAddressMismatch_c);
                    break;

//                case GATTDBConfirm_Status_gSmSmpPacketReceivedAfterTimeoutOccurred_c:
//                    shell_write(gSmSmpPacketReceivedAfterTimeoutOccurred_c);
//                    break;

                case GATTDBConfirm_Status_gSmPairingErrorPasskeyEntryFailed_c:
                    shell_write(gSmPairingErrorPasskeyEntryFailed_c);
                    break;

                case GATTDBConfirm_Status_gSmPairingErrorConfirmValueFailed_c:
                    shell_write(gSmPairingErrorConfirmValueFailed_c);
                    break;

                case GATTDBConfirm_Status_gSmPairingErrorCommandNotSupported_c:
                    shell_write(gSmPairingErrorCommandNotSupported_c);
                    break;

                case GATTDBConfirm_Status_gSmPairingErrorInvalidParameters_c:
                    shell_write(gSmPairingErrorInvalidParameters_c);
                    break;

                case GATTDBConfirm_Status_gSmPairingErrorUnknownReason_c:
                    shell_write(gSmPairingErrorUnknownReason_c);
                    break;

                case GATTDBConfirm_Status_gSmTbResolvableAddressDoesNotMatchIrk_c:
                    shell_write(gSmTbResolvableAddressDoesNotMatchIrk_c);
                    break;

                case GATTDBConfirm_Status_gSmTbInvalidDataSignature_c:
                    shell_write(gSmTbInvalidDataSignature_c);
                    break;

                case GATTDBConfirm_Status_gAttInvalidHandle_c:
                    shell_write(gAttInvalidHandle_c);
                    break;

                case GATTDBConfirm_Status_gAttReadNotPermitted_c:
                    shell_write(gAttReadNotPermitted_c);
                    break;

                case GATTDBConfirm_Status_gAttWriteNotPermitted_c:
                    shell_write(gAttWriteNotPermitted_c);
                    break;

                case GATTDBConfirm_Status_gAttInvalidPdu_c:
                    shell_write(gAttInvalidPdu_c);
                    break;

                case GATTDBConfirm_Status_gAttInsufficientAuthentication_c:
                    shell_write(gAttInsufficientAuthentication_c);
                    break;

                case GATTDBConfirm_Status_gAttRequestNotSupported_c:
                    shell_write(gAttRequestNotSupported_c);
                    break;

                case GATTDBConfirm_Status_gAttInvalidOffset_c:
                    shell_write(gAttInvalidOffset_c);
                    break;

                case GATTDBConfirm_Status_gAttInsufficientAuthorization_c:
                    shell_write(gAttInsufficientAuthorization_c);
                    break;

                case GATTDBConfirm_Status_gAttPrepareQueueFull_c:
                    shell_write(gAttPrepareQueueFull_c);
                    break;

                case GATTDBConfirm_Status_gAttAttributeNotFound_c:
                    shell_write(gAttAttributeNotFound_c);
                    break;

                case GATTDBConfirm_Status_gAttAttributeNotLong_c:
                    shell_write(gAttAttributeNotLong_c);
                    break;

                case GATTDBConfirm_Status_gAttInsufficientEncryptionKeySize_c:
                    shell_write(gAttInsufficientEncryptionKeySize_c);
                    break;

                case GATTDBConfirm_Status_gAttInvalidAttributeValueLength_c:
                    shell_write(gAttInvalidAttributeValueLength_c);
                    break;

                case GATTDBConfirm_Status_gAttUnlikelyor_c:
                    shell_write(gAttUnlikelyor_c);
                    break;

                case GATTDBConfirm_Status_gAttInsufficientEncryption_c:
                    shell_write(gAttInsufficientEncryption_c);
                    break;

                case GATTDBConfirm_Status_gAttUnsupportedGroupType_c:
                    shell_write(gAttUnsupportedGroupType_c);
                    break;

                case GATTDBConfirm_Status_gAttInsufficientResources_c:
                    shell_write(gAttInsufficientResources_c);
                    break;

                case GATTDBConfirm_Status_gGattAnotherProcedureInProgress_c:
                    shell_write(gGattAnotherProcedureInProgress_c);
                    break;

                case GATTDBConfirm_Status_gGattLongAttributePacketsCorrupted_c:
                    shell_write(gGattLongAttributePacketsCorrupted_c);
                    break;

                case GATTDBConfirm_Status_gGattMultipleAttributesOverflow_c:
                    shell_write(gGattMultipleAttributesOverflow_c);
                    break;

                case GATTDBConfirm_Status_gGattUnexpectedReadMultipleResponseLength_c:
                    shell_write(gGattUnexpectedReadMultipleResponseLength_c);
                    break;

                case GATTDBConfirm_Status_gGattInvalidValueLength_c:
                    shell_write(gGattInvalidValueLength_c);
                    break;

                case GATTDBConfirm_Status_gGattServerTimeout_c:
                    shell_write(gGattServerTimeout_c);
                    break;

                case GATTDBConfirm_Status_gGattIndicationAlreadyInProgress_c:
                    shell_write(gGattIndicationAlreadyInProgress_c);
                    break;

                case GATTDBConfirm_Status_gGattClientConfirmationTimeout_c:
                    shell_write(gGattClientConfirmationTimeout_c);
                    break;

                case GATTDBConfirm_Status_gGapAdvDataTooLong_c:
                    shell_write(gGapAdvDataTooLong_c);
                    break;

                case GATTDBConfirm_Status_gGapScanRspDataTooLong_c:
                    shell_write(gGapScanRspDataTooLong_c);
                    break;

                case GATTDBConfirm_Status_gGapDeviceNotBonded_c:
                    shell_write(gGapDeviceNotBonded_c);
                    break;

                case GATTDBConfirm_Status_gDevDbCccdLimitReached_c:
                    shell_write(gDevDbCccdLimitReached_c);
                    break;

                case GATTDBConfirm_Status_gDevDbCccdNotFound_c:
                    shell_write(gDevDbCccdNotFound_c);
                    break;

                case GATTDBConfirm_Status_gGattDbInvalidHandle_c:
                    shell_write(gGattDbInvalidHandle_c);
                    break;

                case GATTDBConfirm_Status_gGattDbCharacteristicNotFound_c:
                    shell_write(gGattDbCharacteristicNotFound_c);
                    break;

                case GATTDBConfirm_Status_gGattDbCccdNotFound_c:
                    shell_write(gGattDbCccdNotFound_c);
                    break;

                case GATTDBConfirm_Status_gGattDbServiceNotFound_c:
                    shell_write(gGattDbServiceNotFound_c);
                    break;

                case GATTDBConfirm_Status_gGattDbDescriptorNotFound_c:
                    shell_write(gGattDbDescriptorNotFound_c);
                    break;

                default:
                    shell_printf("Unrecognized status 0x%04X", container->Data.GATTDBConfirm.Status);
                    break;
            }

            break;

        case 0x4581:
            shell_write("\r\nGATTDBReadAttributeIndication");
            SHELL_GattDb_PrintReadAttr(&(container->Data.GATTDBReadAttributeIndication));
            break;

        case 0x4582:
            shell_write("\r\nGATTDBFindServiceHandleIndication");
            shell_printf("\r\n\tService Handle --> %d", container->Data.GATTDBFindServiceHandleIndication.ServiceHandle);
            gLatestHandle = container->Data.GATTDBFindServiceHandleIndication.ServiceHandle;
            break;

        case 0x4583:
            if (!mSuppressBleEventPrint)
            {
                shell_write("\r\nGATTDBFindCharValueHandleInServiceIndication");
                shell_printf("\r\n\tCharacteristic Value Handle --> %d", container->Data.GATTDBFindCharValueHandleInServiceIndication.CharValueHandle);
            }

            gLatestHandle = container->Data.GATTDBFindCharValueHandleInServiceIndication.CharValueHandle;
            break;

        case 0x4584:

            if (!mSuppressBleEventPrint)
            {
                shell_write("\r\nGATTDBFindCccdHandleForCharValueHandleIndication");
            }

            gCccdHandle = container->Data.GATTDBFindCccdHandleForCharValueHandleIndication.CccdHandle;
            break;

        case 0x4585:
            shell_write("\r\nGATTDBFindDescriptorHandleForCharValueHandleIndication");
            break;

        case 0x4586:
            shell_write("\r\nGATTDBDynamicAddPrimaryServiceDeclarationIndication");
            SHELL_GattDb_PrintPrimaryService(&(container->Data.GATTDBDynamicAddPrimaryServiceDeclarationIndication));
            break;

        case 0x4587:
            shell_write("\r\nGATTDBDynamicAddSecondaryServiceDeclarationIndication");
            break;

        case 0x4588:
            shell_write("\r\nGATTDBDynamicAddIncludeDeclarationIndication");
            break;

        case 0x4589:
            shell_write("\r\nGATTDBDynamicAddCharacteristicDeclarationAndValueIndication");
            SHELL_GattDb_PrintChar(&(container->Data.GATTDBDynamicAddCharacteristicDeclarationAndValueIndication));
            break;

        case 0x458A:
            shell_write("\r\nGATTDBDynamicAddCharacteristicDescriptorIndication");
            SHELL_GattDb_PrintCharDesc(&(container->Data.GATTDBDynamicAddCharacteristicDescriptorIndication));
            break;

        case 0x458B:
            shell_write("\r\nGATTDBDynamicAddCccdIndication");
            SHELL_GattDb_PrintCCCD(&(container->Data.GATTDBDynamicAddCccdIndication));
            break;

        case 0x458C:
            shell_write("\r\nGATTDBDynamicAddCharacteristicDeclarationWithUniqueValueIndication");
            break;

        case 0x4780:

            if (mSuppressBleEventPrint)
            {
                break;
            }

            shell_write("\r\nGAPConfirm");
            shell_write(" -> ");

            switch (container->Data.GAPConfirm.Status)
            {
                case GAPConfirm_Status_gBleSuccess_c:
                    shell_write(gBleSuccess_c);
                    break;

                case GAPConfirm_Status_gBleInvalidParameter_c:
                    shell_write(gBleInvalidParameter_c);
                    break;

                case GAPConfirm_Status_gBleOverflow_c:
                    shell_write(gBleOverflow_c);
                    break;

                case GAPConfirm_Status_gBleUnavailable_c:
                    shell_write(gBleUnavailable_c);
                    break;

                case GAPConfirm_Status_gBleFeatureNotSupported_c:
                    shell_write(gBleFeatureNotSupported_c);
                    break;

                case GAPConfirm_Status_gBleOutOfMemory_c:
                    shell_write(gBleOutOfMemory_c);
                    break;

                case GAPConfirm_Status_gBleAlreadyInitialized_c:
                    shell_write(gBleAlreadyInitialized_c);
                    break;

                case GAPConfirm_Status_gBleOsError_c:
                    shell_write(gBleOsError_c);
                    break;

                case GAPConfirm_Status_gBleUnexpectedError_c:
                    shell_write(gBleUnexpectedError_c);
                    break;

                case GAPConfirm_Status_gBleInvalidState_c:
                    shell_write(gBleInvalidState_c);
                    break;

                case GAPConfirm_Status_gHciUnknownHciCommand_c:
                    shell_write(gHciUnknownHciCommand_c);
                    break;

                case GAPConfirm_Status_gHciUnknownConnectionIdentifier_c:
                    shell_write(gHciUnknownConnectionIdentifier_c);
                    break;

                case GAPConfirm_Status_gHciHardwareFailure_c:
                    shell_write(gHciHardwareFailure_c);
                    break;

                case GAPConfirm_Status_gHciPageTimeout_c:
                    shell_write(gHciPageTimeout_c);
                    break;

                case GAPConfirm_Status_gHciAuthenticationFailure_c:
                    shell_write(gHciAuthenticationFailure_c);
                    break;

                case GAPConfirm_Status_gHciPinOrKeyMissing_c:
                    shell_write(gHciPinOrKeyMissing_c);
                    break;

                case GAPConfirm_Status_gHciMemoryCapacityExceeded_c:
                    shell_write(gHciMemoryCapacityExceeded_c);
                    break;

                case GAPConfirm_Status_gHciConnectionTimeout_c:
                    shell_write(gHciConnectionTimeout_c);
                    break;

                case GAPConfirm_Status_gHciConnectionLimitExceeded_c:
                    shell_write(gHciConnectionLimitExceeded_c);
                    break;

                case GAPConfirm_Status_gHciSynchronousConnectionLimitToADeviceExceeded_c:
                    shell_write(gHciSynchronousConnectionLimitToADeviceExceeded_c);
                    break;

                case GAPConfirm_Status_gHciAclConnectionAlreadyExists_c:
                    shell_write(gHciAclConnectionAlreadyExists_c);
                    break;

                case GAPConfirm_Status_gHciCommandDisallowed_c:
                    shell_write(gHciCommandDisallowed_c);
                    break;

                case GAPConfirm_Status_gHciConnectionRejectedDueToLimitedResources_c:
                    shell_write(gHciConnectionRejectedDueToLimitedResources_c);
                    break;

                case GAPConfirm_Status_gHciConnectionRejectedDueToSecurityReasons_c:
                    shell_write(gHciConnectionRejectedDueToSecurityReasons_c);
                    break;

                case GAPConfirm_Status_gHciConnectionRejectedDueToUnacceptableBdAddr_c:
                    shell_write(gHciConnectionRejectedDueToUnacceptableBdAddr_c);
                    break;

                case GAPConfirm_Status_gHciConnectionAcceptTimeoutExceeded_c:
                    shell_write(gHciConnectionAcceptTimeoutExceeded_c);
                    break;

                case GAPConfirm_Status_gHciUnsupportedFeatureOrParameterValue_c:
                    shell_write(gHciUnsupportedFeatureOrParameterValue_c);
                    break;

                case GAPConfirm_Status_gHciInvalidHciCommandParameters_c:
                    shell_write(gHciInvalidHciCommandParameters_c);
                    break;

                case GAPConfirm_Status_gHciRemoteUserTerminatedConnection_c:
                    shell_write(gHciRemoteUserTerminatedConnection_c);
                    break;

                case GAPConfirm_Status_gHciRemoteDeviceTerminatedConnectionLowResources_c:
                    shell_write(gHciRemoteDeviceTerminatedConnectionLowResources_c);
                    break;

                case GAPConfirm_Status_gHciRemoteDeviceTerminatedConnectionPowerOff_c:
                    shell_write(gHciRemoteDeviceTerminatedConnectionPowerOff_c);
                    break;

                case GAPConfirm_Status_gHciConnectionTerminatedByLocalHost_c:
                    shell_write(gHciConnectionTerminatedByLocalHost_c);
                    break;

                case GAPConfirm_Status_gHciRepeatedAttempts_c:
                    shell_write(gHciRepeatedAttempts_c);
                    break;

                case GAPConfirm_Status_gHciPairingNotAllowed_c:
                    shell_write(gHciPairingNotAllowed_c);
                    break;

                case GAPConfirm_Status_gHciUnknownLpmPdu_c:
                    shell_write(gHciUnknownLpmPdu_c);
                    break;

                case GAPConfirm_Status_gHciUnsupportedRemoteFeature_c:
                    shell_write(gHciUnsupportedRemoteFeature_c);
                    break;

                case GAPConfirm_Status_gHciScoOffsetRejected_c:
                    shell_write(gHciScoOffsetRejected_c);
                    break;

                case GAPConfirm_Status_gHciScoIntervalRejected_c:
                    shell_write(gHciScoIntervalRejected_c);
                    break;

                case GAPConfirm_Status_gHciScoAirModeRejected_c:
                    shell_write(gHciScoAirModeRejected_c);
                    break;

                case GAPConfirm_Status_gHciInvalidLpmParameters_c:
                    shell_write(gHciInvalidLpmParameters_c);
                    break;

                case GAPConfirm_Status_gHciUnspecifiedError_c:
                    shell_write(gHciUnspecifiedError_c);
                    break;

                case GAPConfirm_Status_gHciUnsupportedLpmParameterValue_c:
                    shell_write(gHciUnsupportedLpmParameterValue_c);
                    break;

                case GAPConfirm_Status_gHciRoleChangeNotAllowed_c:
                    shell_write(gHciRoleChangeNotAllowed_c);
                    break;

                case GAPConfirm_Status_gHciLLResponseTimeout_c:
                    shell_write(gHciLLResponseTimeout_c);
                    break;

                case GAPConfirm_Status_gHciLmpErrorTransactionCollision_c:
                    shell_write(gHciLmpErrorTransactionCollision_c);
                    break;

                case GAPConfirm_Status_gHciLmpPduNotAllowed_c:
                    shell_write(gHciLmpPduNotAllowed_c);
                    break;

                case GAPConfirm_Status_gHciEncryptionModeNotAcceptable_c:
                    shell_write(gHciEncryptionModeNotAcceptable_c);
                    break;

                case GAPConfirm_Status_gHciLinkKeyCannotBeChanged_c:
                    shell_write(gHciLinkKeyCannotBeChanged_c);
                    break;

                case GAPConfirm_Status_gHciRequestedQosNotSupported_c:
                    shell_write(gHciRequestedQosNotSupported_c);
                    break;

                case GAPConfirm_Status_gHciInstantPassed_c:
                    shell_write(gHciInstantPassed_c);
                    break;

                case GAPConfirm_Status_gHciPairingWithUnitKeyNotSupported_c:
                    shell_write(gHciPairingWithUnitKeyNotSupported_c);
                    break;

                case GAPConfirm_Status_gHciDifferentTransactionCollision_c:
                    shell_write(gHciDifferentTransactionCollision_c);
                    break;

                case GAPConfirm_Status_gHciReserved_0x2B_c:
                    shell_write(gHciReserved_0x2B_c);
                    break;

                case GAPConfirm_Status_gHciQosNotAcceptableParameter_c:
                    shell_write(gHciQosNotAcceptableParameter_c);
                    break;

                case GAPConfirm_Status_gHciQosRejected_c:
                    shell_write(gHciQosRejected_c);
                    break;

                case GAPConfirm_Status_gHciChannelClassificationNotSupported_c:
                    shell_write(gHciChannelClassificationNotSupported_c);
                    break;

                case GAPConfirm_Status_gHciInsufficientSecurity_c:
                    shell_write(gHciInsufficientSecurity_c);
                    break;

                case GAPConfirm_Status_gHciParameterOutOfMandatoryRange_c:
                    shell_write(gHciParameterOutOfMandatoryRange_c);
                    break;

                case GAPConfirm_Status_gHciReserved_0x31_c:
                    shell_write(gHciReserved_0x31_c);
                    break;

                case GAPConfirm_Status_gHciRoleSwitchPending_c:
                    shell_write(gHciRoleSwitchPending_c);
                    break;

                case GAPConfirm_Status_gHciReserved_0x33_c:
                    shell_write(gHciReserved_0x33_c);
                    break;

                case GAPConfirm_Status_gHciReservedSlotViolation_c:
                    shell_write(gHciReservedSlotViolation_c);
                    break;

                case GAPConfirm_Status_gHciRoleSwitchFailed_c:
                    shell_write(gHciRoleSwitchFailed_c);
                    break;

                case GAPConfirm_Status_gHciExtendedInquiryResponseTooLarge_c:
                    shell_write(gHciExtendedInquiryResponseTooLarge_c);
                    break;

                case GAPConfirm_Status_gHciSecureSimplePairingNotSupportedByHost_c:
                    shell_write(gHciSecureSimplePairingNotSupportedByHost_c);
                    break;

                case GAPConfirm_Status_gHciHostBusyPairing_c:
                    shell_write(gHciHostBusyPairing_c);
                    break;

                case GAPConfirm_Status_gHciConnectionRejectedDueToNoSuitableChannelFound_c:
                    shell_write(gHciConnectionRejectedDueToNoSuitableChannelFound_c);
                    break;

                case GAPConfirm_Status_gHciControllerBusy_c:
                    shell_write(gHciControllerBusy_c);
                    break;

                case GAPConfirm_Status_gHciUnacceptableConnectionParameters_c:
                    shell_write(gHciUnacceptableConnectionParameters_c);
                    break;

                case GAPConfirm_Status_gHciDirectedAdvertisingTimeout_c:
                    shell_write(gHciDirectedAdvertisingTimeout_c);
                    break;

                case GAPConfirm_Status_gHciConnectionTerminatedDueToMicFailure_c:
                    shell_write(gHciConnectionTerminatedDueToMicFailure_c);
                    break;

                case GAPConfirm_Status_gHciConnectionFailedToBeEstablished_c:
                    shell_write(gHciConnectionFailedToBeEstablished_c);
                    break;

                case GAPConfirm_Status_gHciMacConnectionFailed_c:
                    shell_write(gHciMacConnectionFailed_c);
                    break;

                case GAPConfirm_Status_gHciCoarseClockAdjustmentRejected_c:
                    shell_write(gHciCoarseClockAdjustmentRejected_c);
                    break;

                case GAPConfirm_Status_gHciAlreadyInit_c:
                    shell_write(gHciAlreadyInit_c);
                    break;

                case GAPConfirm_Status_gHciInvalidParameter_c:
                    shell_write(gHciInvalidParameter_c);
                    break;

                case GAPConfirm_Status_gHciCallbackNotInstalled_c:
                    shell_write(gHciCallbackNotInstalled_c);
                    break;

                case GAPConfirm_Status_gHciCallbackAlreadyInstalled_c:
                    shell_write(gHciCallbackAlreadyInstalled_c);
                    break;

                case GAPConfirm_Status_gHciCommandNotSupported_c:
                    shell_write(gHciCommandNotSupported_c);
                    break;

                case GAPConfirm_Status_gHciEventNotSupported_c:
                    shell_write(gHciEventNotSupported_c);
                    break;

                case GAPConfirm_Status_gHciTransportError_c:
                    shell_write(gHciTransportError_c);
                    break;

                case GAPConfirm_Status_gL2caAlreadyInit_c:
                    shell_write(gL2caAlreadyInit_c);
                    break;

                case GAPConfirm_Status_gL2caInsufficientResources_c:
                    shell_write(gL2caInsufficientResources_c);
                    break;

                case GAPConfirm_Status_gL2caCallbackNotInstalled_c:
                    shell_write(gL2caCallbackNotInstalled_c);
                    break;

                case GAPConfirm_Status_gL2caCallbackAlreadyInstalled_c:
                    shell_write(gL2caCallbackAlreadyInstalled_c);
                    break;

                case GAPConfirm_Status_gL2caLePsmInvalid_c:
                    shell_write(gL2caLePsmInvalid_c);
                    break;

                case GAPConfirm_Status_gL2caLePsmAlreadyRegistered_c:
                    shell_write(gL2caLePsmAlreadyRegistered_c);
                    break;

                case GAPConfirm_Status_gL2caLePsmNotRegistered_c:
                    shell_write(gL2caLePsmNotRegistered_c);
                    break;

                case GAPConfirm_Status_gL2caLePsmInsufficientResources_c:
                    shell_write(gL2caLePsmInsufficientResources_c);
                    break;

                case GAPConfirm_Status_gL2caChannelInvalid_c:
                    shell_write(gL2caChannelInvalid_c);
                    break;

                case GAPConfirm_Status_gL2caChannelClosed_c:
                    shell_write(gL2caChannelClosed_c);
                    break;

                case GAPConfirm_Status_gL2caChannelAlreadyConnected_c:
                    shell_write(gL2caChannelAlreadyConnected_c);
                    break;

                case GAPConfirm_Status_gL2caConnectionParametersRejected_c:
                    shell_write(gL2caConnectionParametersRejected_c);
                    break;

                case GAPConfirm_Status_gL2caChannelBusy_c:
                    shell_write(gL2caChannelBusy_c);
                    break;

                case GAPConfirm_Status_gL2caInvalidParameter_c:
                    shell_write(gL2caInvalidParameter_c);
                    break;

                case GAPConfirm_Status_gL2caError_c:
                    shell_write(gL2caError_c);
                    break;

                case GAPConfirm_Status_gSmNullCBFunction_c:
                    shell_write(gSmNullCBFunction_c);
                    break;

                case GAPConfirm_Status_gSmCommandNotSupported_c:
                    shell_write(gSmCommandNotSupported_c);
                    break;

                case GAPConfirm_Status_gSmUnexpectedCommand_c:
                    shell_write(gSmUnexpectedCommand_c);
                    break;

                case GAPConfirm_Status_gSmInvalidCommandCode_c:
                    shell_write(gSmInvalidCommandCode_c);
                    break;

                case GAPConfirm_Status_gSmInvalidCommandLength_c:
                    shell_write(gSmInvalidCommandLength_c);
                    break;

                case GAPConfirm_Status_gSmInvalidCommandParameter_c:
                    shell_write(gSmInvalidCommandParameter_c);
                    break;

                case GAPConfirm_Status_gSmInvalidDeviceId_c:
                    shell_write(gSmInvalidDeviceId_c);
                    break;

                case GAPConfirm_Status_gSmInvalidInternalOperation_c:
                    shell_write(gSmInvalidInternalOperation_c);
                    break;

                case GAPConfirm_Status_gSmInvalidConnectionHandle_c:
                    shell_write(gSmInvalidConnectionHandle_c);
                    break;

                case GAPConfirm_Status_gSmInproperKeyDistributionField_c:
                    shell_write(gSmInproperKeyDistributionField_c);
                    break;

                case GAPConfirm_Status_gSmUnexpectedKeyType_c:
                    shell_write(gSmUnexpectedKeyType_c);
                    break;

                case GAPConfirm_Status_gSmUnexpectedPairingTerminationReason_c:
                    shell_write(gSmUnexpectedPairingTerminationReason_c);
                    break;

                case GAPConfirm_Status_gSmUnexpectedKeyset_c:
                    shell_write(gSmUnexpectedKeyset_c);
                    break;

                case GAPConfirm_Status_gSmSmpTimeoutOccurred_c:
                    shell_write(gSmSmpTimeoutOccurred_c);
                    break;

                case GAPConfirm_Status_gSmUnknownSmpPacketType_c:
                    shell_write(gSmUnknownSmpPacketType_c);
                    break;

                case GAPConfirm_Status_gSmInvalidSmpPacketLength_c:
                    shell_write(gSmInvalidSmpPacketLength_c);
                    break;

                case GAPConfirm_Status_gSmInvalidSmpPacketParameter_c:
                    shell_write(gSmInvalidSmpPacketParameter_c);
                    break;

                case GAPConfirm_Status_gSmReceivedUnexpectedSmpPacket_c:
                    shell_write(gSmReceivedUnexpectedSmpPacket_c);
                    break;

                case GAPConfirm_Status_gSmReceivedSmpPacketFromUnknownDevice_c:
                    shell_write(gSmReceivedSmpPacketFromUnknownDevice_c);
                    break;

                case GAPConfirm_Status_gSmReceivedUnexpectedHciEvent_c:
                    shell_write(gSmReceivedUnexpectedHciEvent_c);
                    break;

                case GAPConfirm_Status_gSmReceivedHciEventFromUnknownDevice_c:
                    shell_write(gSmReceivedHciEventFromUnknownDevice_c);
                    break;

                case GAPConfirm_Status_gSmInvalidHciEventParameter_c:
                    shell_write(gSmInvalidHciEventParameter_c);
                    break;

                case GAPConfirm_Status_gSmLlConnectionEncryptionInProgress_c:
                    shell_write(gSmLlConnectionEncryptionInProgress_c);
                    break;

                case GAPConfirm_Status_gSmLlConnectionEncryptionFailure_c:
                    shell_write(gSmLlConnectionEncryptionFailure_c);
                    break;

                case GAPConfirm_Status_gSmInsufficientResources_c:
                    shell_write(gSmInsufficientResources_c);
                    break;

                case GAPConfirm_Status_gSmOobDataAddressMismatch_c:
                    shell_write(gSmOobDataAddressMismatch_c);
                    break;

//                case GAPConfirm_Status_gSmSmpPacketReceivedAfterTimeoutOccurred_c:
//                    shell_write(gSmSmpPacketReceivedAfterTimeoutOccurred_c);
//                    break;

                case GAPConfirm_Status_gSmPairingErrorPasskeyEntryFailed_c:
                    shell_write(gSmPairingErrorPasskeyEntryFailed_c);
                    break;

                case GAPConfirm_Status_gSmPairingErrorConfirmValueFailed_c:
                    shell_write(gSmPairingErrorConfirmValueFailed_c);
                    break;

                case GAPConfirm_Status_gSmPairingErrorCommandNotSupported_c:
                    shell_write(gSmPairingErrorCommandNotSupported_c);
                    break;

                case GAPConfirm_Status_gSmPairingErrorInvalidParameters_c:
                    shell_write(gSmPairingErrorInvalidParameters_c);
                    break;

                case GAPConfirm_Status_gSmPairingErrorUnknownReason_c:
                    shell_write(gSmPairingErrorUnknownReason_c);
                    break;

                case GAPConfirm_Status_gSmTbResolvableAddressDoesNotMatchIrk_c:
                    shell_write(gSmTbResolvableAddressDoesNotMatchIrk_c);
                    break;

                case GAPConfirm_Status_gSmTbInvalidDataSignature_c:
                    shell_write(gSmTbInvalidDataSignature_c);
                    break;

                case GAPConfirm_Status_gAttInvalidHandle_c:
                    shell_write(gAttInvalidHandle_c);
                    break;

                case GAPConfirm_Status_gAttReadNotPermitted_c:
                    shell_write(gAttReadNotPermitted_c);
                    break;

                case GAPConfirm_Status_gAttWriteNotPermitted_c:
                    shell_write(gAttWriteNotPermitted_c);
                    break;

                case GAPConfirm_Status_gAttInvalidPdu_c:
                    shell_write(gAttInvalidPdu_c);
                    break;

                case GAPConfirm_Status_gAttInsufficientAuthentication_c:
                    shell_write(gAttInsufficientAuthentication_c);
                    break;

                case GAPConfirm_Status_gAttRequestNotSupported_c:
                    shell_write(gAttRequestNotSupported_c);
                    break;

                case GAPConfirm_Status_gAttInvalidOffset_c:
                    shell_write(gAttInvalidOffset_c);
                    break;

                case GAPConfirm_Status_gAttInsufficientAuthorization_c:
                    shell_write(gAttInsufficientAuthorization_c);
                    break;

                case GAPConfirm_Status_gAttPrepareQueueFull_c:
                    shell_write(gAttPrepareQueueFull_c);
                    break;

                case GAPConfirm_Status_gAttAttributeNotFound_c:
                    shell_write(gAttAttributeNotFound_c);
                    break;

                case GAPConfirm_Status_gAttAttributeNotLong_c:
                    shell_write(gAttAttributeNotLong_c);
                    break;

                case GAPConfirm_Status_gAttInsufficientEncryptionKeySize_c:
                    shell_write(gAttInsufficientEncryptionKeySize_c);
                    break;

                case GAPConfirm_Status_gAttInvalidAttributeValueLength_c:
                    shell_write(gAttInvalidAttributeValueLength_c);
                    break;

                case GAPConfirm_Status_gAttUnlikelyor_c:
                    shell_write(gAttUnlikelyor_c);
                    break;

                case GAPConfirm_Status_gAttInsufficientEncryption_c:
                    shell_write(gAttInsufficientEncryption_c);
                    break;

                case GAPConfirm_Status_gAttUnsupportedGroupType_c:
                    shell_write(gAttUnsupportedGroupType_c);
                    break;

                case GAPConfirm_Status_gAttInsufficientResources_c:
                    shell_write(gAttInsufficientResources_c);
                    break;

                case GAPConfirm_Status_gGattAnotherProcedureInProgress_c:
                    shell_write(gGattAnotherProcedureInProgress_c);
                    break;

                case GAPConfirm_Status_gGattLongAttributePacketsCorrupted_c:
                    shell_write(gGattLongAttributePacketsCorrupted_c);
                    break;

                case GAPConfirm_Status_gGattMultipleAttributesOverflow_c:
                    shell_write(gGattMultipleAttributesOverflow_c);
                    break;

                case GAPConfirm_Status_gGattUnexpectedReadMultipleResponseLength_c:
                    shell_write(gGattUnexpectedReadMultipleResponseLength_c);
                    break;

                case GAPConfirm_Status_gGattInvalidValueLength_c:
                    shell_write(gGattInvalidValueLength_c);
                    break;

                case GAPConfirm_Status_gGattServerTimeout_c:
                    shell_write(gGattServerTimeout_c);
                    break;

                case GAPConfirm_Status_gGattIndicationAlreadyInProgress_c:
                    shell_write(gGattIndicationAlreadyInProgress_c);
                    break;

                case GAPConfirm_Status_gGattClientConfirmationTimeout_c:
                    shell_write(gGattClientConfirmationTimeout_c);
                    break;

                case GAPConfirm_Status_gGapAdvDataTooLong_c:
                    shell_write(gGapAdvDataTooLong_c);
                    break;

                case GAPConfirm_Status_gGapScanRspDataTooLong_c:
                    shell_write(gGapScanRspDataTooLong_c);
                    break;

                case GAPConfirm_Status_gGapDeviceNotBonded_c:
                    shell_write(gGapDeviceNotBonded_c);
                    break;

                case GAPConfirm_Status_gDevDbCccdLimitReached_c:
                    shell_write(gDevDbCccdLimitReached_c);
                    break;

                case GAPConfirm_Status_gDevDbCccdNotFound_c:
                    shell_write(gDevDbCccdNotFound_c);
                    break;

                case GAPConfirm_Status_gGattDbInvalidHandle_c:
                    shell_write(gGattDbInvalidHandle_c);
                    break;

                case GAPConfirm_Status_gGattDbCharacteristicNotFound_c:
                    shell_write(gGattDbCharacteristicNotFound_c);
                    break;

                case GAPConfirm_Status_gGattDbCccdNotFound_c:
                    shell_write(gGattDbCccdNotFound_c);
                    break;

                case GAPConfirm_Status_gGattDbServiceNotFound_c:
                    shell_write(gGattDbServiceNotFound_c);
                    break;

                case GAPConfirm_Status_gGattDbDescriptorNotFound_c:
                    shell_write(gGattDbDescriptorNotFound_c);
                    break;

                default:
                    shell_printf("Unrecognized status 0x%04X", container->Data.GAPConfirm.Status);
                    break;
            }

            break;

        case 0x4781:
            if (!mSuppressBleEventPrint)
            {
                shell_write("\r\nGAPCheckNotificationStatusIndication");
            }

            gIsNotificationActive = container->Data.GAPCheckNotificationStatusIndication.IsActive;
            break;

        case 0x4782:
            if (!mSuppressBleEventPrint)
            {
                shell_write("\r\nGAPCheckIndicationStatusIndication");
            }
            gIsIndicationActive = container->Data.GAPCheckIndicationStatusIndication.IsActive;

            break;

        case 0x4783:
            shell_write("\r\nGAPGetBondedStaticAddressesIndication");
            break;

        case 0x4784:
            shell_write("\r\nGAPLoadEncryptionInformationIndication");
            break;

        case 0x4785:
            shell_write("\r\nGAPLoadCustomPeerInformationIndication");
            break;

        case 0x4786:
            shell_write("\r\nGAPCheckIfBondedIndication");
            if (!container->Data.GAPCheckIfBondedIndication.IsBonded)
            {
                shell_write(" -> Not Bonded!");
                GAPSendSlaveSecurityRequestRequest_t req = {gPeerDeviceId, gPairingParameters.PairingParameters.WithBonding, gPairingParameters.PairingParameters.SecurityModeAndLevel};
                GAPSendSlaveSecurityRequestRequest(&req, BLE_FSCI_IF);
            }
            else
            {
                shell_write(" -> Bonded!");
                OtapCS_Subscribe(gPeerDeviceId);

                if (mDelayTimerID == gTmrInvalidTimerID_c)
                {
                    mDelayTimerID = TMR_AllocateTimer();
                }

                // keep the loop going
                TMR_StartSingleShotTimer(mDelayTimerID, 200, (pfTmrCallBack_t)OtapClient_HandleConnectionEvent, (void *)((uint32_t)gPeerDeviceId));
            }

            GAPEnableUpdateConnectionParametersRequest_t update_req = {gPeerDeviceId, TRUE};
            GAPEnableUpdateConnectionParametersRequest(&update_req, BLE_FSCI_IF);
            break;

        case 0x4787:
            shell_write("\r\nGAPGetBondedDevicesCountIndication");
            break;

        case 0x4788:
            shell_write("\r\nGAPGetBondedDeviceNameIndication");
            break;

        case 0x4789:
            shell_write("\r\nGAPGenericEventInitializationCompleteIndication");
            break;

        case 0x478A:
            shell_write("\r\nGAPGenericEventInternalErrorIndication");
            shell_write(" -> ");

            switch (container->Data.GAPGenericEventInternalErrorIndication.ErrorCode)
            {
                case GAPGenericEventInternalErrorIndication_ErrorCode_gBleSuccess_c:
                    shell_write(gBleSuccess_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorCode_gBleInvalidParameter_c:
                    shell_write(gBleInvalidParameter_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorCode_gBleOverflow_c:
                    shell_write(gBleOverflow_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorCode_gBleUnavailable_c:
                    shell_write(gBleUnavailable_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorCode_gBleFeatureNotSupported_c:
                    shell_write(gBleFeatureNotSupported_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorCode_gBleOutOfMemory_c:
                    shell_write(gBleOutOfMemory_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorCode_gBleAlreadyInitialized_c:
                    shell_write(gBleAlreadyInitialized_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorCode_gBleOsError_c:
                    shell_write(gBleOsError_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorCode_gBleUnexpectedError_c:
                    shell_write(gBleUnexpectedError_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorCode_gBleInvalidState_c:
                    shell_write(gBleInvalidState_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorCode_gHciUnknownHciCommand_c:
                    shell_write(gHciUnknownHciCommand_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorCode_gHciUnknownConnectionIdentifier_c:
                    shell_write(gHciUnknownConnectionIdentifier_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorCode_gHciHardwareFailure_c:
                    shell_write(gHciHardwareFailure_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorCode_gHciPageTimeout_c:
                    shell_write(gHciPageTimeout_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorCode_gHciAuthenticationFailure_c:
                    shell_write(gHciAuthenticationFailure_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorCode_gHciPinOrKeyMissing_c:
                    shell_write(gHciPinOrKeyMissing_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorCode_gHciMemoryCapacityExceeded_c:
                    shell_write(gHciMemoryCapacityExceeded_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorCode_gHciConnectionTimeout_c:
                    shell_write(gHciConnectionTimeout_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorCode_gHciConnectionLimitExceeded_c:
                    shell_write(gHciConnectionLimitExceeded_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorCode_gHciSynchronousConnectionLimitToADeviceExceeded_c:
                    shell_write(gHciSynchronousConnectionLimitToADeviceExceeded_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorCode_gHciAclConnectionAlreadyExists_c:
                    shell_write(gHciAclConnectionAlreadyExists_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorCode_gHciCommandDisallowed_c:
                    shell_write(gHciCommandDisallowed_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorCode_gHciConnectionRejectedDueToLimitedResources_c:
                    shell_write(gHciConnectionRejectedDueToLimitedResources_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorCode_gHciConnectionRejectedDueToSecurityReasons_c:
                    shell_write(gHciConnectionRejectedDueToSecurityReasons_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorCode_gHciConnectionRejectedDueToUnacceptableBdAddr_c:
                    shell_write(gHciConnectionRejectedDueToUnacceptableBdAddr_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorCode_gHciConnectionAcceptTimeoutExceeded_c:
                    shell_write(gHciConnectionAcceptTimeoutExceeded_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorCode_gHciUnsupportedFeatureOrParameterValue_c:
                    shell_write(gHciUnsupportedFeatureOrParameterValue_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorCode_gHciInvalidHciCommandParameters_c:
                    shell_write(gHciInvalidHciCommandParameters_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorCode_gHciRemoteUserTerminatedConnection_c:
                    shell_write(gHciRemoteUserTerminatedConnection_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorCode_gHciRemoteDeviceTerminatedConnectionLowResources_c:
                    shell_write(gHciRemoteDeviceTerminatedConnectionLowResources_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorCode_gHciRemoteDeviceTerminatedConnectionPowerOff_c:
                    shell_write(gHciRemoteDeviceTerminatedConnectionPowerOff_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorCode_gHciConnectionTerminatedByLocalHost_c:
                    shell_write(gHciConnectionTerminatedByLocalHost_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorCode_gHciRepeatedAttempts_c:
                    shell_write(gHciRepeatedAttempts_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorCode_gHciPairingNotAllowed_c:
                    shell_write(gHciPairingNotAllowed_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorCode_gHciUnknownLpmPdu_c:
                    shell_write(gHciUnknownLpmPdu_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorCode_gHciUnsupportedRemoteFeature_c:
                    shell_write(gHciUnsupportedRemoteFeature_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorCode_gHciScoOffsetRejected_c:
                    shell_write(gHciScoOffsetRejected_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorCode_gHciScoIntervalRejected_c:
                    shell_write(gHciScoIntervalRejected_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorCode_gHciScoAirModeRejected_c:
                    shell_write(gHciScoAirModeRejected_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorCode_gHciInvalidLpmParameters_c:
                    shell_write(gHciInvalidLpmParameters_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorCode_gHciUnspecifiedError_c:
                    shell_write(gHciUnspecifiedError_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorCode_gHciUnsupportedLpmParameterValue_c:
                    shell_write(gHciUnsupportedLpmParameterValue_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorCode_gHciRoleChangeNotAllowed_c:
                    shell_write(gHciRoleChangeNotAllowed_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorCode_gHciLLResponseTimeout_c:
                    shell_write(gHciLLResponseTimeout_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorCode_gHciLmpErrorTransactionCollision_c:
                    shell_write(gHciLmpErrorTransactionCollision_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorCode_gHciLmpPduNotAllowed_c:
                    shell_write(gHciLmpPduNotAllowed_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorCode_gHciEncryptionModeNotAcceptable_c:
                    shell_write(gHciEncryptionModeNotAcceptable_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorCode_gHciLinkKeyCannotBeChanged_c:
                    shell_write(gHciLinkKeyCannotBeChanged_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorCode_gHciRequestedQosNotSupported_c:
                    shell_write(gHciRequestedQosNotSupported_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorCode_gHciInstantPassed_c:
                    shell_write(gHciInstantPassed_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorCode_gHciPairingWithUnitKeyNotSupported_c:
                    shell_write(gHciPairingWithUnitKeyNotSupported_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorCode_gHciDifferentTransactionCollision_c:
                    shell_write(gHciDifferentTransactionCollision_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorCode_gHciReserved_0x2B_c:
                    shell_write(gHciReserved_0x2B_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorCode_gHciQosNotAcceptableParameter_c:
                    shell_write(gHciQosNotAcceptableParameter_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorCode_gHciQosRejected_c:
                    shell_write(gHciQosRejected_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorCode_gHciChannelClassificationNotSupported_c:
                    shell_write(gHciChannelClassificationNotSupported_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorCode_gHciInsufficientSecurity_c:
                    shell_write(gHciInsufficientSecurity_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorCode_gHciParameterOutOfMandatoryRange_c:
                    shell_write(gHciParameterOutOfMandatoryRange_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorCode_gHciReserved_0x31_c:
                    shell_write(gHciReserved_0x31_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorCode_gHciRoleSwitchPending_c:
                    shell_write(gHciRoleSwitchPending_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorCode_gHciReserved_0x33_c:
                    shell_write(gHciReserved_0x33_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorCode_gHciReservedSlotViolation_c:
                    shell_write(gHciReservedSlotViolation_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorCode_gHciRoleSwitchFailed_c:
                    shell_write(gHciRoleSwitchFailed_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorCode_gHciExtendedInquiryResponseTooLarge_c:
                    shell_write(gHciExtendedInquiryResponseTooLarge_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorCode_gHciSecureSimplePairingNotSupportedByHost_c:
                    shell_write(gHciSecureSimplePairingNotSupportedByHost_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorCode_gHciHostBusyPairing_c:
                    shell_write(gHciHostBusyPairing_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorCode_gHciConnectionRejectedDueToNoSuitableChannelFound_c:
                    shell_write(gHciConnectionRejectedDueToNoSuitableChannelFound_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorCode_gHciControllerBusy_c:
                    shell_write(gHciControllerBusy_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorCode_gHciUnacceptableConnectionParameters_c:
                    shell_write(gHciUnacceptableConnectionParameters_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorCode_gHciDirectedAdvertisingTimeout_c:
                    shell_write(gHciDirectedAdvertisingTimeout_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorCode_gHciConnectionTerminatedDueToMicFailure_c:
                    shell_write(gHciConnectionTerminatedDueToMicFailure_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorCode_gHciConnectionFailedToBeEstablished_c:
                    shell_write(gHciConnectionFailedToBeEstablished_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorCode_gHciMacConnectionFailed_c:
                    shell_write(gHciMacConnectionFailed_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorCode_gHciCoarseClockAdjustmentRejected_c:
                    shell_write(gHciCoarseClockAdjustmentRejected_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorCode_gHciAlreadyInit_c:
                    shell_write(gHciAlreadyInit_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorCode_gHciInvalidParameter_c:
                    shell_write(gHciInvalidParameter_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorCode_gHciCallbackNotInstalled_c:
                    shell_write(gHciCallbackNotInstalled_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorCode_gHciCallbackAlreadyInstalled_c:
                    shell_write(gHciCallbackAlreadyInstalled_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorCode_gHciCommandNotSupported_c:
                    shell_write(gHciCommandNotSupported_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorCode_gHciEventNotSupported_c:
                    shell_write(gHciEventNotSupported_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorCode_gHciTransportError_c:
                    shell_write(gHciTransportError_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorCode_gL2caAlreadyInit_c:
                    shell_write(gL2caAlreadyInit_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorCode_gL2caInsufficientResources_c:
                    shell_write(gL2caInsufficientResources_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorCode_gL2caCallbackNotInstalled_c:
                    shell_write(gL2caCallbackNotInstalled_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorCode_gL2caCallbackAlreadyInstalled_c:
                    shell_write(gL2caCallbackAlreadyInstalled_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorCode_gL2caLePsmInvalid_c:
                    shell_write(gL2caLePsmInvalid_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorCode_gL2caLePsmAlreadyRegistered_c:
                    shell_write(gL2caLePsmAlreadyRegistered_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorCode_gL2caLePsmNotRegistered_c:
                    shell_write(gL2caLePsmNotRegistered_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorCode_gL2caLePsmInsufficientResources_c:
                    shell_write(gL2caLePsmInsufficientResources_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorCode_gL2caChannelInvalid_c:
                    shell_write(gL2caChannelInvalid_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorCode_gL2caChannelClosed_c:
                    shell_write(gL2caChannelClosed_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorCode_gL2caChannelAlreadyConnected_c:
                    shell_write(gL2caChannelAlreadyConnected_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorCode_gL2caConnectionParametersRejected_c:
                    shell_write(gL2caConnectionParametersRejected_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorCode_gL2caChannelBusy_c:
                    shell_write(gL2caChannelBusy_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorCode_gL2caInvalidParameter_c:
                    shell_write(gL2caInvalidParameter_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorCode_gL2caError_c:
                    shell_write(gL2caError_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorCode_gSmNullCBFunction_c:
                    shell_write(gSmNullCBFunction_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorCode_gSmCommandNotSupported_c:
                    shell_write(gSmCommandNotSupported_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorCode_gSmUnexpectedCommand_c:
                    shell_write(gSmUnexpectedCommand_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorCode_gSmInvalidCommandCode_c:
                    shell_write(gSmInvalidCommandCode_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorCode_gSmInvalidCommandLength_c:
                    shell_write(gSmInvalidCommandLength_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorCode_gSmInvalidCommandParameter_c:
                    shell_write(gSmInvalidCommandParameter_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorCode_gSmInvalidDeviceId_c:
                    shell_write(gSmInvalidDeviceId_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorCode_gSmInvalidInternalOperation_c:
                    shell_write(gSmInvalidInternalOperation_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorCode_gSmInvalidConnectionHandle_c:
                    shell_write(gSmInvalidConnectionHandle_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorCode_gSmInproperKeyDistributionField_c:
                    shell_write(gSmInproperKeyDistributionField_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorCode_gSmUnexpectedKeyType_c:
                    shell_write(gSmUnexpectedKeyType_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorCode_gSmUnexpectedPairingTerminationReason_c:
                    shell_write(gSmUnexpectedPairingTerminationReason_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorCode_gSmUnexpectedKeyset_c:
                    shell_write(gSmUnexpectedKeyset_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorCode_gSmSmpTimeoutOccurred_c:
                    shell_write(gSmSmpTimeoutOccurred_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorCode_gSmUnknownSmpPacketType_c:
                    shell_write(gSmUnknownSmpPacketType_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorCode_gSmInvalidSmpPacketLength_c:
                    shell_write(gSmInvalidSmpPacketLength_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorCode_gSmInvalidSmpPacketParameter_c:
                    shell_write(gSmInvalidSmpPacketParameter_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorCode_gSmReceivedUnexpectedSmpPacket_c:
                    shell_write(gSmReceivedUnexpectedSmpPacket_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorCode_gSmReceivedSmpPacketFromUnknownDevice_c:
                    shell_write(gSmReceivedSmpPacketFromUnknownDevice_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorCode_gSmReceivedUnexpectedHciEvent_c:
                    shell_write(gSmReceivedUnexpectedHciEvent_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorCode_gSmReceivedHciEventFromUnknownDevice_c:
                    shell_write(gSmReceivedHciEventFromUnknownDevice_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorCode_gSmInvalidHciEventParameter_c:
                    shell_write(gSmInvalidHciEventParameter_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorCode_gSmLlConnectionEncryptionInProgress_c:
                    shell_write(gSmLlConnectionEncryptionInProgress_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorCode_gSmLlConnectionEncryptionFailure_c:
                    shell_write(gSmLlConnectionEncryptionFailure_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorCode_gSmInsufficientResources_c:
                    shell_write(gSmInsufficientResources_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorCode_gSmOobDataAddressMismatch_c:
                    shell_write(gSmOobDataAddressMismatch_c);
                    break;

//                case GAPGenericEventInternalErrorIndication_ErrorCode_gSmSmpPacketReceivedAfterTimeoutOccurred_c:
//                    shell_write(gSmSmpPacketReceivedAfterTimeoutOccurred_c);
//                    break;

                case GAPGenericEventInternalErrorIndication_ErrorCode_gSmPairingErrorPasskeyEntryFailed_c:
                    shell_write(gSmPairingErrorPasskeyEntryFailed_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorCode_gSmPairingErrorConfirmValueFailed_c:
                    shell_write(gSmPairingErrorConfirmValueFailed_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorCode_gSmPairingErrorCommandNotSupported_c:
                    shell_write(gSmPairingErrorCommandNotSupported_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorCode_gSmPairingErrorInvalidParameters_c:
                    shell_write(gSmPairingErrorInvalidParameters_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorCode_gSmPairingErrorUnknownReason_c:
                    shell_write(gSmPairingErrorUnknownReason_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorCode_gSmTbResolvableAddressDoesNotMatchIrk_c:
                    shell_write(gSmTbResolvableAddressDoesNotMatchIrk_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorCode_gSmTbInvalidDataSignature_c:
                    shell_write(gSmTbInvalidDataSignature_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorCode_gAttInvalidHandle_c:
                    shell_write(gAttInvalidHandle_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorCode_gAttReadNotPermitted_c:
                    shell_write(gAttReadNotPermitted_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorCode_gAttWriteNotPermitted_c:
                    shell_write(gAttWriteNotPermitted_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorCode_gAttInvalidPdu_c:
                    shell_write(gAttInvalidPdu_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorCode_gAttInsufficientAuthentication_c:
                    shell_write(gAttInsufficientAuthentication_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorCode_gAttRequestNotSupported_c:
                    shell_write(gAttRequestNotSupported_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorCode_gAttInvalidOffset_c:
                    shell_write(gAttInvalidOffset_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorCode_gAttInsufficientAuthorization_c:
                    shell_write(gAttInsufficientAuthorization_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorCode_gAttPrepareQueueFull_c:
                    shell_write(gAttPrepareQueueFull_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorCode_gAttAttributeNotFound_c:
                    shell_write(gAttAttributeNotFound_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorCode_gAttAttributeNotLong_c:
                    shell_write(gAttAttributeNotLong_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorCode_gAttInsufficientEncryptionKeySize_c:
                    shell_write(gAttInsufficientEncryptionKeySize_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorCode_gAttInvalidAttributeValueLength_c:
                    shell_write(gAttInvalidAttributeValueLength_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorCode_gAttUnlikelyor_c:
                    shell_write(gAttUnlikelyor_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorCode_gAttInsufficientEncryption_c:
                    shell_write(gAttInsufficientEncryption_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorCode_gAttUnsupportedGroupType_c:
                    shell_write(gAttUnsupportedGroupType_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorCode_gAttInsufficientResources_c:
                    shell_write(gAttInsufficientResources_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorCode_gGattAnotherProcedureInProgress_c:
                    shell_write(gGattAnotherProcedureInProgress_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorCode_gGattLongAttributePacketsCorrupted_c:
                    shell_write(gGattLongAttributePacketsCorrupted_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorCode_gGattMultipleAttributesOverflow_c:
                    shell_write(gGattMultipleAttributesOverflow_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorCode_gGattUnexpectedReadMultipleResponseLength_c:
                    shell_write(gGattUnexpectedReadMultipleResponseLength_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorCode_gGattInvalidValueLength_c:
                    shell_write(gGattInvalidValueLength_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorCode_gGattServerTimeout_c:
                    shell_write(gGattServerTimeout_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorCode_gGattIndicationAlreadyInProgress_c:
                    shell_write(gGattIndicationAlreadyInProgress_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorCode_gGattClientConfirmationTimeout_c:
                    shell_write(gGattClientConfirmationTimeout_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorCode_gGapAdvDataTooLong_c:
                    shell_write(gGapAdvDataTooLong_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorCode_gGapScanRspDataTooLong_c:
                    shell_write(gGapScanRspDataTooLong_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorCode_gGapDeviceNotBonded_c:
                    shell_write(gGapDeviceNotBonded_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorCode_gDevDbCccdLimitReached_c:
                    shell_write(gDevDbCccdLimitReached_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorCode_gDevDbCccdNotFound_c:
                    shell_write(gDevDbCccdNotFound_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorCode_gGattDbInvalidHandle_c:
                    shell_write(gGattDbInvalidHandle_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorCode_gGattDbCharacteristicNotFound_c:
                    shell_write(gGattDbCharacteristicNotFound_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorCode_gGattDbCccdNotFound_c:
                    shell_write(gGattDbCccdNotFound_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorCode_gGattDbServiceNotFound_c:
                    shell_write(gGattDbServiceNotFound_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorCode_gGattDbDescriptorNotFound_c:
                    shell_write(gGattDbDescriptorNotFound_c);
                    break;

                default:
                    shell_printf("Unrecognized status 0x%04X", container->Data.GAPGenericEventInternalErrorIndication.ErrorCode);
                    break;
            }

            shell_write(" -> ");

            switch (container->Data.GAPGenericEventInternalErrorIndication.ErrorSource)
            {
                case GAPGenericEventInternalErrorIndication_ErrorSource_gHciCommandStatus_c:
                    shell_write(gHciCommandStatus_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorSource_gCheckPrivateResolvableAddress_c:
                    shell_write(gCheckPrivateResolvableAddress_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorSource_gVerifySignature_c:
                    shell_write(gVerifySignature_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorSource_gAddNewConnection_c:
                    shell_write(gAddNewConnection_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorSource_gResetController_c:
                    shell_write(gResetController_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorSource_gSetEventMask_c:
                    shell_write(gSetEventMask_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorSource_gReadLeBufferSize_c:
                    shell_write(gReadLeBufferSize_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorSource_gSetLeEventMask_c:
                    shell_write(gSetLeEventMask_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorSource_gReadDeviceAddress_c:
                    shell_write(gReadDeviceAddress_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorSource_gReadLocalSupportedFeatures_c:
                    shell_write(gReadLocalSupportedFeatures_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorSource_gReadWhiteListSize_c:
                    shell_write(gReadWhiteListSize_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorSource_gClearWhiteList_c:
                    shell_write(gClearWhiteList_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorSource_gAddDeviceToWhiteList_c:
                    shell_write(gAddDeviceToWhiteList_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorSource_gRemoveDeviceFromWhiteList_c:
                    shell_write(gRemoveDeviceFromWhiteList_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorSource_gCancelCreateConnection_c:
                    shell_write(gCancelCreateConnection_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorSource_gReadRadioPower_c:
                    shell_write(gReadRadioPower_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorSource_gSetRandomAddress_c:
                    shell_write(gSetRandomAddress_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorSource_gCreateRandomAddress_c:
                    shell_write(gCreateRandomAddress_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorSource_gEncryptLink_c:
                    shell_write(gEncryptLink_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorSource_gProvideLongTermKey_c:
                    shell_write(gProvideLongTermKey_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorSource_gDenyLongTermKey_c:
                    shell_write(gDenyLongTermKey_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorSource_gConnect_c:
                    shell_write(gConnect_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorSource_gDisconnect_c:
                    shell_write(gDisconnect_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorSource_gTerminatePairing_c:
                    shell_write(gTerminatePairing_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorSource_gSendSlaveSecurityRequest_c:
                    shell_write(gSendSlaveSecurityRequest_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorSource_gEnterPasskey_c:
                    shell_write(gEnterPasskey_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorSource_gProvideOob_c:
                    shell_write(gProvideOob_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorSource_gSendSmpKeys_c:
                    shell_write(gSendSmpKeys_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorSource_gWriteSuggestedDefaultDataLength_c:
                    shell_write(gWriteSuggestedDefaultDataLength_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorSource_gReadSuggestedDefaultDataLength_c:
                    shell_write(gReadSuggestedDefaultDataLength_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorSource_gUpdateLeDataLength_c:
                    shell_write(gUpdateLeDataLength_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorSource_gEnableControllerPrivacy_c:
                    shell_write(gEnableControllerPrivacy_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorSource_gLeScSendKeypressNotification_c:
                    shell_write(gLeScSendKeypressNotification_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorSource_gLeScSetPeerOobData_c:
                    shell_write(gLeScSetPeerOobData_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorSource_gLeScGetLocalOobData_c:
                    shell_write(gLeScGetLocalOobData_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorSource_gLeScValidateNumericValue_c:
                    shell_write(gLeScValidateNumericValue_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorSource_gLeScRegeneratePublicKey_c:
                    shell_write(gLeScRegeneratePublicKey_c);
                    break;

                case GAPGenericEventInternalErrorIndication_ErrorSource_gLeSetResolvablePrivateAddressTimeout_c:
                    shell_write(gLeSetResolvablePrivateAddressTimeout_c);
                    break;

                default:
                    shell_printf("Unrecognized status 0x%02X", container->Data.GAPGenericEventInternalErrorIndication.ErrorSource);
                    break;
            }

            break;

        case 0x478B:
            shell_write("\r\nGAPGenericEventAdvertisingSetupFailedIndication");
            shell_write(" -> ");

            switch (container->Data.GAPGenericEventAdvertisingSetupFailedIndication.SetupFailReason)
            {
                case GAPGenericEventAdvertisingSetupFailedIndication_SetupFailReason_gBleSuccess_c:
                    shell_write(gBleSuccess_c);
                    break;

                case GAPGenericEventAdvertisingSetupFailedIndication_SetupFailReason_gBleInvalidParameter_c:
                    shell_write(gBleInvalidParameter_c);
                    break;

                case GAPGenericEventAdvertisingSetupFailedIndication_SetupFailReason_gBleOverflow_c:
                    shell_write(gBleOverflow_c);
                    break;

                case GAPGenericEventAdvertisingSetupFailedIndication_SetupFailReason_gBleUnavailable_c:
                    shell_write(gBleUnavailable_c);
                    break;

                case GAPGenericEventAdvertisingSetupFailedIndication_SetupFailReason_gBleFeatureNotSupported_c:
                    shell_write(gBleFeatureNotSupported_c);
                    break;

                case GAPGenericEventAdvertisingSetupFailedIndication_SetupFailReason_gBleOutOfMemory_c:
                    shell_write(gBleOutOfMemory_c);
                    break;

                case GAPGenericEventAdvertisingSetupFailedIndication_SetupFailReason_gBleAlreadyInitialized_c:
                    shell_write(gBleAlreadyInitialized_c);
                    break;

                case GAPGenericEventAdvertisingSetupFailedIndication_SetupFailReason_gBleOsError_c:
                    shell_write(gBleOsError_c);
                    break;

                case GAPGenericEventAdvertisingSetupFailedIndication_SetupFailReason_gBleUnexpectedError_c:
                    shell_write(gBleUnexpectedError_c);
                    break;

                case GAPGenericEventAdvertisingSetupFailedIndication_SetupFailReason_gBleInvalidState_c:
                    shell_write(gBleInvalidState_c);
                    break;

                case GAPGenericEventAdvertisingSetupFailedIndication_SetupFailReason_gHciUnknownHciCommand_c:
                    shell_write(gHciUnknownHciCommand_c);
                    break;

                case GAPGenericEventAdvertisingSetupFailedIndication_SetupFailReason_gHciUnknownConnectionIdentifier_c:
                    shell_write(gHciUnknownConnectionIdentifier_c);
                    break;

                case GAPGenericEventAdvertisingSetupFailedIndication_SetupFailReason_gHciHardwareFailure_c:
                    shell_write(gHciHardwareFailure_c);
                    break;

                case GAPGenericEventAdvertisingSetupFailedIndication_SetupFailReason_gHciPageTimeout_c:
                    shell_write(gHciPageTimeout_c);
                    break;

                case GAPGenericEventAdvertisingSetupFailedIndication_SetupFailReason_gHciAuthenticationFailure_c:
                    shell_write(gHciAuthenticationFailure_c);
                    break;

                case GAPGenericEventAdvertisingSetupFailedIndication_SetupFailReason_gHciPinOrKeyMissing_c:
                    shell_write(gHciPinOrKeyMissing_c);
                    break;

                case GAPGenericEventAdvertisingSetupFailedIndication_SetupFailReason_gHciMemoryCapacityExceeded_c:
                    shell_write(gHciMemoryCapacityExceeded_c);
                    break;

                case GAPGenericEventAdvertisingSetupFailedIndication_SetupFailReason_gHciConnectionTimeout_c:
                    shell_write(gHciConnectionTimeout_c);
                    break;

                case GAPGenericEventAdvertisingSetupFailedIndication_SetupFailReason_gHciConnectionLimitExceeded_c:
                    shell_write(gHciConnectionLimitExceeded_c);
                    break;

                case GAPGenericEventAdvertisingSetupFailedIndication_SetupFailReason_gHciSynchronousConnectionLimitToADeviceExceeded_c:
                    shell_write(gHciSynchronousConnectionLimitToADeviceExceeded_c);
                    break;

                case GAPGenericEventAdvertisingSetupFailedIndication_SetupFailReason_gHciAclConnectionAlreadyExists_c:
                    shell_write(gHciAclConnectionAlreadyExists_c);
                    break;

                case GAPGenericEventAdvertisingSetupFailedIndication_SetupFailReason_gHciCommandDisallowed_c:
                    shell_write(gHciCommandDisallowed_c);
                    break;

                case GAPGenericEventAdvertisingSetupFailedIndication_SetupFailReason_gHciConnectionRejectedDueToLimitedResources_c:
                    shell_write(gHciConnectionRejectedDueToLimitedResources_c);
                    break;

                case GAPGenericEventAdvertisingSetupFailedIndication_SetupFailReason_gHciConnectionRejectedDueToSecurityReasons_c:
                    shell_write(gHciConnectionRejectedDueToSecurityReasons_c);
                    break;

                case GAPGenericEventAdvertisingSetupFailedIndication_SetupFailReason_gHciConnectionRejectedDueToUnacceptableBdAddr_c:
                    shell_write(gHciConnectionRejectedDueToUnacceptableBdAddr_c);
                    break;

                case GAPGenericEventAdvertisingSetupFailedIndication_SetupFailReason_gHciConnectionAcceptTimeoutExceeded_c:
                    shell_write(gHciConnectionAcceptTimeoutExceeded_c);
                    break;

                case GAPGenericEventAdvertisingSetupFailedIndication_SetupFailReason_gHciUnsupportedFeatureOrParameterValue_c:
                    shell_write(gHciUnsupportedFeatureOrParameterValue_c);
                    break;

                case GAPGenericEventAdvertisingSetupFailedIndication_SetupFailReason_gHciInvalidHciCommandParameters_c:
                    shell_write(gHciInvalidHciCommandParameters_c);
                    break;

                case GAPGenericEventAdvertisingSetupFailedIndication_SetupFailReason_gHciRemoteUserTerminatedConnection_c:
                    shell_write(gHciRemoteUserTerminatedConnection_c);
                    break;

                case GAPGenericEventAdvertisingSetupFailedIndication_SetupFailReason_gHciRemoteDeviceTerminatedConnectionLowResources_c:
                    shell_write(gHciRemoteDeviceTerminatedConnectionLowResources_c);
                    break;

                case GAPGenericEventAdvertisingSetupFailedIndication_SetupFailReason_gHciRemoteDeviceTerminatedConnectionPowerOff_c:
                    shell_write(gHciRemoteDeviceTerminatedConnectionPowerOff_c);
                    break;

                case GAPGenericEventAdvertisingSetupFailedIndication_SetupFailReason_gHciConnectionTerminatedByLocalHost_c:
                    shell_write(gHciConnectionTerminatedByLocalHost_c);
                    break;

                case GAPGenericEventAdvertisingSetupFailedIndication_SetupFailReason_gHciRepeatedAttempts_c:
                    shell_write(gHciRepeatedAttempts_c);
                    break;

                case GAPGenericEventAdvertisingSetupFailedIndication_SetupFailReason_gHciPairingNotAllowed_c:
                    shell_write(gHciPairingNotAllowed_c);
                    break;

                case GAPGenericEventAdvertisingSetupFailedIndication_SetupFailReason_gHciUnknownLpmPdu_c:
                    shell_write(gHciUnknownLpmPdu_c);
                    break;

                case GAPGenericEventAdvertisingSetupFailedIndication_SetupFailReason_gHciUnsupportedRemoteFeature_c:
                    shell_write(gHciUnsupportedRemoteFeature_c);
                    break;

                case GAPGenericEventAdvertisingSetupFailedIndication_SetupFailReason_gHciScoOffsetRejected_c:
                    shell_write(gHciScoOffsetRejected_c);
                    break;

                case GAPGenericEventAdvertisingSetupFailedIndication_SetupFailReason_gHciScoIntervalRejected_c:
                    shell_write(gHciScoIntervalRejected_c);
                    break;

                case GAPGenericEventAdvertisingSetupFailedIndication_SetupFailReason_gHciScoAirModeRejected_c:
                    shell_write(gHciScoAirModeRejected_c);
                    break;

                case GAPGenericEventAdvertisingSetupFailedIndication_SetupFailReason_gHciInvalidLpmParameters_c:
                    shell_write(gHciInvalidLpmParameters_c);
                    break;

                case GAPGenericEventAdvertisingSetupFailedIndication_SetupFailReason_gHciUnspecifiedError_c:
                    shell_write(gHciUnspecifiedError_c);
                    break;

                case GAPGenericEventAdvertisingSetupFailedIndication_SetupFailReason_gHciUnsupportedLpmParameterValue_c:
                    shell_write(gHciUnsupportedLpmParameterValue_c);
                    break;

                case GAPGenericEventAdvertisingSetupFailedIndication_SetupFailReason_gHciRoleChangeNotAllowed_c:
                    shell_write(gHciRoleChangeNotAllowed_c);
                    break;

                case GAPGenericEventAdvertisingSetupFailedIndication_SetupFailReason_gHciLLResponseTimeout_c:
                    shell_write(gHciLLResponseTimeout_c);
                    break;

                case GAPGenericEventAdvertisingSetupFailedIndication_SetupFailReason_gHciLmpErrorTransactionCollision_c:
                    shell_write(gHciLmpErrorTransactionCollision_c);
                    break;

                case GAPGenericEventAdvertisingSetupFailedIndication_SetupFailReason_gHciLmpPduNotAllowed_c:
                    shell_write(gHciLmpPduNotAllowed_c);
                    break;

                case GAPGenericEventAdvertisingSetupFailedIndication_SetupFailReason_gHciEncryptionModeNotAcceptable_c:
                    shell_write(gHciEncryptionModeNotAcceptable_c);
                    break;

                case GAPGenericEventAdvertisingSetupFailedIndication_SetupFailReason_gHciLinkKeyCannotBeChanged_c:
                    shell_write(gHciLinkKeyCannotBeChanged_c);
                    break;

                case GAPGenericEventAdvertisingSetupFailedIndication_SetupFailReason_gHciRequestedQosNotSupported_c:
                    shell_write(gHciRequestedQosNotSupported_c);
                    break;

                case GAPGenericEventAdvertisingSetupFailedIndication_SetupFailReason_gHciInstantPassed_c:
                    shell_write(gHciInstantPassed_c);
                    break;

                case GAPGenericEventAdvertisingSetupFailedIndication_SetupFailReason_gHciPairingWithUnitKeyNotSupported_c:
                    shell_write(gHciPairingWithUnitKeyNotSupported_c);
                    break;

                case GAPGenericEventAdvertisingSetupFailedIndication_SetupFailReason_gHciDifferentTransactionCollision_c:
                    shell_write(gHciDifferentTransactionCollision_c);
                    break;

                case GAPGenericEventAdvertisingSetupFailedIndication_SetupFailReason_gHciReserved_0x2B_c:
                    shell_write(gHciReserved_0x2B_c);
                    break;

                case GAPGenericEventAdvertisingSetupFailedIndication_SetupFailReason_gHciQosNotAcceptableParameter_c:
                    shell_write(gHciQosNotAcceptableParameter_c);
                    break;

                case GAPGenericEventAdvertisingSetupFailedIndication_SetupFailReason_gHciQosRejected_c:
                    shell_write(gHciQosRejected_c);
                    break;

                case GAPGenericEventAdvertisingSetupFailedIndication_SetupFailReason_gHciChannelClassificationNotSupported_c:
                    shell_write(gHciChannelClassificationNotSupported_c);
                    break;

                case GAPGenericEventAdvertisingSetupFailedIndication_SetupFailReason_gHciInsufficientSecurity_c:
                    shell_write(gHciInsufficientSecurity_c);
                    break;

                case GAPGenericEventAdvertisingSetupFailedIndication_SetupFailReason_gHciParameterOutOfMandatoryRange_c:
                    shell_write(gHciParameterOutOfMandatoryRange_c);
                    break;

                case GAPGenericEventAdvertisingSetupFailedIndication_SetupFailReason_gHciReserved_0x31_c:
                    shell_write(gHciReserved_0x31_c);
                    break;

                case GAPGenericEventAdvertisingSetupFailedIndication_SetupFailReason_gHciRoleSwitchPending_c:
                    shell_write(gHciRoleSwitchPending_c);
                    break;

                case GAPGenericEventAdvertisingSetupFailedIndication_SetupFailReason_gHciReserved_0x33_c:
                    shell_write(gHciReserved_0x33_c);
                    break;

                case GAPGenericEventAdvertisingSetupFailedIndication_SetupFailReason_gHciReservedSlotViolation_c:
                    shell_write(gHciReservedSlotViolation_c);
                    break;

                case GAPGenericEventAdvertisingSetupFailedIndication_SetupFailReason_gHciRoleSwitchFailed_c:
                    shell_write(gHciRoleSwitchFailed_c);
                    break;

                case GAPGenericEventAdvertisingSetupFailedIndication_SetupFailReason_gHciExtendedInquiryResponseTooLarge_c:
                    shell_write(gHciExtendedInquiryResponseTooLarge_c);
                    break;

                case GAPGenericEventAdvertisingSetupFailedIndication_SetupFailReason_gHciSecureSimplePairingNotSupportedByHost_c:
                    shell_write(gHciSecureSimplePairingNotSupportedByHost_c);
                    break;

                case GAPGenericEventAdvertisingSetupFailedIndication_SetupFailReason_gHciHostBusyPairing_c:
                    shell_write(gHciHostBusyPairing_c);
                    break;

                case GAPGenericEventAdvertisingSetupFailedIndication_SetupFailReason_gHciConnectionRejectedDueToNoSuitableChannelFound_c:
                    shell_write(gHciConnectionRejectedDueToNoSuitableChannelFound_c);
                    break;

                case GAPGenericEventAdvertisingSetupFailedIndication_SetupFailReason_gHciControllerBusy_c:
                    shell_write(gHciControllerBusy_c);
                    break;

                case GAPGenericEventAdvertisingSetupFailedIndication_SetupFailReason_gHciUnacceptableConnectionParameters_c:
                    shell_write(gHciUnacceptableConnectionParameters_c);
                    break;

                case GAPGenericEventAdvertisingSetupFailedIndication_SetupFailReason_gHciDirectedAdvertisingTimeout_c:
                    shell_write(gHciDirectedAdvertisingTimeout_c);
                    break;

                case GAPGenericEventAdvertisingSetupFailedIndication_SetupFailReason_gHciConnectionTerminatedDueToMicFailure_c:
                    shell_write(gHciConnectionTerminatedDueToMicFailure_c);
                    break;

                case GAPGenericEventAdvertisingSetupFailedIndication_SetupFailReason_gHciConnectionFailedToBeEstablished_c:
                    shell_write(gHciConnectionFailedToBeEstablished_c);
                    break;

                case GAPGenericEventAdvertisingSetupFailedIndication_SetupFailReason_gHciMacConnectionFailed_c:
                    shell_write(gHciMacConnectionFailed_c);
                    break;

                case GAPGenericEventAdvertisingSetupFailedIndication_SetupFailReason_gHciCoarseClockAdjustmentRejected_c:
                    shell_write(gHciCoarseClockAdjustmentRejected_c);
                    break;

                case GAPGenericEventAdvertisingSetupFailedIndication_SetupFailReason_gHciAlreadyInit_c:
                    shell_write(gHciAlreadyInit_c);
                    break;

                case GAPGenericEventAdvertisingSetupFailedIndication_SetupFailReason_gHciInvalidParameter_c:
                    shell_write(gHciInvalidParameter_c);
                    break;

                case GAPGenericEventAdvertisingSetupFailedIndication_SetupFailReason_gHciCallbackNotInstalled_c:
                    shell_write(gHciCallbackNotInstalled_c);
                    break;

                case GAPGenericEventAdvertisingSetupFailedIndication_SetupFailReason_gHciCallbackAlreadyInstalled_c:
                    shell_write(gHciCallbackAlreadyInstalled_c);
                    break;

                case GAPGenericEventAdvertisingSetupFailedIndication_SetupFailReason_gHciCommandNotSupported_c:
                    shell_write(gHciCommandNotSupported_c);
                    break;

                case GAPGenericEventAdvertisingSetupFailedIndication_SetupFailReason_gHciEventNotSupported_c:
                    shell_write(gHciEventNotSupported_c);
                    break;

                case GAPGenericEventAdvertisingSetupFailedIndication_SetupFailReason_gHciTransportError_c:
                    shell_write(gHciTransportError_c);
                    break;

                case GAPGenericEventAdvertisingSetupFailedIndication_SetupFailReason_gL2caAlreadyInit_c:
                    shell_write(gL2caAlreadyInit_c);
                    break;

                case GAPGenericEventAdvertisingSetupFailedIndication_SetupFailReason_gL2caInsufficientResources_c:
                    shell_write(gL2caInsufficientResources_c);
                    break;

                case GAPGenericEventAdvertisingSetupFailedIndication_SetupFailReason_gL2caCallbackNotInstalled_c:
                    shell_write(gL2caCallbackNotInstalled_c);
                    break;

                case GAPGenericEventAdvertisingSetupFailedIndication_SetupFailReason_gL2caCallbackAlreadyInstalled_c:
                    shell_write(gL2caCallbackAlreadyInstalled_c);
                    break;

                case GAPGenericEventAdvertisingSetupFailedIndication_SetupFailReason_gL2caLePsmInvalid_c:
                    shell_write(gL2caLePsmInvalid_c);
                    break;

                case GAPGenericEventAdvertisingSetupFailedIndication_SetupFailReason_gL2caLePsmAlreadyRegistered_c:
                    shell_write(gL2caLePsmAlreadyRegistered_c);
                    break;

                case GAPGenericEventAdvertisingSetupFailedIndication_SetupFailReason_gL2caLePsmNotRegistered_c:
                    shell_write(gL2caLePsmNotRegistered_c);
                    break;

                case GAPGenericEventAdvertisingSetupFailedIndication_SetupFailReason_gL2caLePsmInsufficientResources_c:
                    shell_write(gL2caLePsmInsufficientResources_c);
                    break;

                case GAPGenericEventAdvertisingSetupFailedIndication_SetupFailReason_gL2caChannelInvalid_c:
                    shell_write(gL2caChannelInvalid_c);
                    break;

                case GAPGenericEventAdvertisingSetupFailedIndication_SetupFailReason_gL2caChannelClosed_c:
                    shell_write(gL2caChannelClosed_c);
                    break;

                case GAPGenericEventAdvertisingSetupFailedIndication_SetupFailReason_gL2caChannelAlreadyConnected_c:
                    shell_write(gL2caChannelAlreadyConnected_c);
                    break;

                case GAPGenericEventAdvertisingSetupFailedIndication_SetupFailReason_gL2caConnectionParametersRejected_c:
                    shell_write(gL2caConnectionParametersRejected_c);
                    break;

                case GAPGenericEventAdvertisingSetupFailedIndication_SetupFailReason_gL2caChannelBusy_c:
                    shell_write(gL2caChannelBusy_c);
                    break;

                case GAPGenericEventAdvertisingSetupFailedIndication_SetupFailReason_gL2caInvalidParameter_c:
                    shell_write(gL2caInvalidParameter_c);
                    break;

                case GAPGenericEventAdvertisingSetupFailedIndication_SetupFailReason_gL2caError_c:
                    shell_write(gL2caError_c);
                    break;

                case GAPGenericEventAdvertisingSetupFailedIndication_SetupFailReason_gSmNullCBFunction_c:
                    shell_write(gSmNullCBFunction_c);
                    break;

                case GAPGenericEventAdvertisingSetupFailedIndication_SetupFailReason_gSmCommandNotSupported_c:
                    shell_write(gSmCommandNotSupported_c);
                    break;

                case GAPGenericEventAdvertisingSetupFailedIndication_SetupFailReason_gSmUnexpectedCommand_c:
                    shell_write(gSmUnexpectedCommand_c);
                    break;

                case GAPGenericEventAdvertisingSetupFailedIndication_SetupFailReason_gSmInvalidCommandCode_c:
                    shell_write(gSmInvalidCommandCode_c);
                    break;

                case GAPGenericEventAdvertisingSetupFailedIndication_SetupFailReason_gSmInvalidCommandLength_c:
                    shell_write(gSmInvalidCommandLength_c);
                    break;

                case GAPGenericEventAdvertisingSetupFailedIndication_SetupFailReason_gSmInvalidCommandParameter_c:
                    shell_write(gSmInvalidCommandParameter_c);
                    break;

                case GAPGenericEventAdvertisingSetupFailedIndication_SetupFailReason_gSmInvalidDeviceId_c:
                    shell_write(gSmInvalidDeviceId_c);
                    break;

                case GAPGenericEventAdvertisingSetupFailedIndication_SetupFailReason_gSmInvalidInternalOperation_c:
                    shell_write(gSmInvalidInternalOperation_c);
                    break;

                case GAPGenericEventAdvertisingSetupFailedIndication_SetupFailReason_gSmInvalidConnectionHandle_c:
                    shell_write(gSmInvalidConnectionHandle_c);
                    break;

                case GAPGenericEventAdvertisingSetupFailedIndication_SetupFailReason_gSmInproperKeyDistributionField_c:
                    shell_write(gSmInproperKeyDistributionField_c);
                    break;

                case GAPGenericEventAdvertisingSetupFailedIndication_SetupFailReason_gSmUnexpectedKeyType_c:
                    shell_write(gSmUnexpectedKeyType_c);
                    break;

                case GAPGenericEventAdvertisingSetupFailedIndication_SetupFailReason_gSmUnexpectedPairingTerminationReason_c:
                    shell_write(gSmUnexpectedPairingTerminationReason_c);
                    break;

                case GAPGenericEventAdvertisingSetupFailedIndication_SetupFailReason_gSmUnexpectedKeyset_c:
                    shell_write(gSmUnexpectedKeyset_c);
                    break;

                case GAPGenericEventAdvertisingSetupFailedIndication_SetupFailReason_gSmSmpTimeoutOccurred_c:
                    shell_write(gSmSmpTimeoutOccurred_c);
                    break;

                case GAPGenericEventAdvertisingSetupFailedIndication_SetupFailReason_gSmUnknownSmpPacketType_c:
                    shell_write(gSmUnknownSmpPacketType_c);
                    break;

                case GAPGenericEventAdvertisingSetupFailedIndication_SetupFailReason_gSmInvalidSmpPacketLength_c:
                    shell_write(gSmInvalidSmpPacketLength_c);
                    break;

                case GAPGenericEventAdvertisingSetupFailedIndication_SetupFailReason_gSmInvalidSmpPacketParameter_c:
                    shell_write(gSmInvalidSmpPacketParameter_c);
                    break;

                case GAPGenericEventAdvertisingSetupFailedIndication_SetupFailReason_gSmReceivedUnexpectedSmpPacket_c:
                    shell_write(gSmReceivedUnexpectedSmpPacket_c);
                    break;

                case GAPGenericEventAdvertisingSetupFailedIndication_SetupFailReason_gSmReceivedSmpPacketFromUnknownDevice_c:
                    shell_write(gSmReceivedSmpPacketFromUnknownDevice_c);
                    break;

                case GAPGenericEventAdvertisingSetupFailedIndication_SetupFailReason_gSmReceivedUnexpectedHciEvent_c:
                    shell_write(gSmReceivedUnexpectedHciEvent_c);
                    break;

                case GAPGenericEventAdvertisingSetupFailedIndication_SetupFailReason_gSmReceivedHciEventFromUnknownDevice_c:
                    shell_write(gSmReceivedHciEventFromUnknownDevice_c);
                    break;

                case GAPGenericEventAdvertisingSetupFailedIndication_SetupFailReason_gSmInvalidHciEventParameter_c:
                    shell_write(gSmInvalidHciEventParameter_c);
                    break;

                case GAPGenericEventAdvertisingSetupFailedIndication_SetupFailReason_gSmLlConnectionEncryptionInProgress_c:
                    shell_write(gSmLlConnectionEncryptionInProgress_c);
                    break;

                case GAPGenericEventAdvertisingSetupFailedIndication_SetupFailReason_gSmLlConnectionEncryptionFailure_c:
                    shell_write(gSmLlConnectionEncryptionFailure_c);
                    break;

                case GAPGenericEventAdvertisingSetupFailedIndication_SetupFailReason_gSmInsufficientResources_c:
                    shell_write(gSmInsufficientResources_c);
                    break;

                case GAPGenericEventAdvertisingSetupFailedIndication_SetupFailReason_gSmOobDataAddressMismatch_c:
                    shell_write(gSmOobDataAddressMismatch_c);
                    break;

//                case GAPGenericEventAdvertisingSetupFailedIndication_SetupFailReason_gSmSmpPacketReceivedAfterTimeoutOccurred_c:
//                    shell_write(gSmSmpPacketReceivedAfterTimeoutOccurred_c);
//                    break;

                case GAPGenericEventAdvertisingSetupFailedIndication_SetupFailReason_gSmPairingErrorPasskeyEntryFailed_c:
                    shell_write(gSmPairingErrorPasskeyEntryFailed_c);
                    break;

                case GAPGenericEventAdvertisingSetupFailedIndication_SetupFailReason_gSmPairingErrorConfirmValueFailed_c:
                    shell_write(gSmPairingErrorConfirmValueFailed_c);
                    break;

                case GAPGenericEventAdvertisingSetupFailedIndication_SetupFailReason_gSmPairingErrorCommandNotSupported_c:
                    shell_write(gSmPairingErrorCommandNotSupported_c);
                    break;

                case GAPGenericEventAdvertisingSetupFailedIndication_SetupFailReason_gSmPairingErrorInvalidParameters_c:
                    shell_write(gSmPairingErrorInvalidParameters_c);
                    break;

                case GAPGenericEventAdvertisingSetupFailedIndication_SetupFailReason_gSmPairingErrorUnknownReason_c:
                    shell_write(gSmPairingErrorUnknownReason_c);
                    break;

                case GAPGenericEventAdvertisingSetupFailedIndication_SetupFailReason_gSmTbResolvableAddressDoesNotMatchIrk_c:
                    shell_write(gSmTbResolvableAddressDoesNotMatchIrk_c);
                    break;

                case GAPGenericEventAdvertisingSetupFailedIndication_SetupFailReason_gSmTbInvalidDataSignature_c:
                    shell_write(gSmTbInvalidDataSignature_c);
                    break;

                case GAPGenericEventAdvertisingSetupFailedIndication_SetupFailReason_gAttInvalidHandle_c:
                    shell_write(gAttInvalidHandle_c);
                    break;

                case GAPGenericEventAdvertisingSetupFailedIndication_SetupFailReason_gAttReadNotPermitted_c:
                    shell_write(gAttReadNotPermitted_c);
                    break;

                case GAPGenericEventAdvertisingSetupFailedIndication_SetupFailReason_gAttWriteNotPermitted_c:
                    shell_write(gAttWriteNotPermitted_c);
                    break;

                case GAPGenericEventAdvertisingSetupFailedIndication_SetupFailReason_gAttInvalidPdu_c:
                    shell_write(gAttInvalidPdu_c);
                    break;

                case GAPGenericEventAdvertisingSetupFailedIndication_SetupFailReason_gAttInsufficientAuthentication_c:
                    shell_write(gAttInsufficientAuthentication_c);
                    break;

                case GAPGenericEventAdvertisingSetupFailedIndication_SetupFailReason_gAttRequestNotSupported_c:
                    shell_write(gAttRequestNotSupported_c);
                    break;

                case GAPGenericEventAdvertisingSetupFailedIndication_SetupFailReason_gAttInvalidOffset_c:
                    shell_write(gAttInvalidOffset_c);
                    break;

                case GAPGenericEventAdvertisingSetupFailedIndication_SetupFailReason_gAttInsufficientAuthorization_c:
                    shell_write(gAttInsufficientAuthorization_c);
                    break;

                case GAPGenericEventAdvertisingSetupFailedIndication_SetupFailReason_gAttPrepareQueueFull_c:
                    shell_write(gAttPrepareQueueFull_c);
                    break;

                case GAPGenericEventAdvertisingSetupFailedIndication_SetupFailReason_gAttAttributeNotFound_c:
                    shell_write(gAttAttributeNotFound_c);
                    break;

                case GAPGenericEventAdvertisingSetupFailedIndication_SetupFailReason_gAttAttributeNotLong_c:
                    shell_write(gAttAttributeNotLong_c);
                    break;

                case GAPGenericEventAdvertisingSetupFailedIndication_SetupFailReason_gAttInsufficientEncryptionKeySize_c:
                    shell_write(gAttInsufficientEncryptionKeySize_c);
                    break;

                case GAPGenericEventAdvertisingSetupFailedIndication_SetupFailReason_gAttInvalidAttributeValueLength_c:
                    shell_write(gAttInvalidAttributeValueLength_c);
                    break;

                case GAPGenericEventAdvertisingSetupFailedIndication_SetupFailReason_gAttUnlikelyor_c:
                    shell_write(gAttUnlikelyor_c);
                    break;

                case GAPGenericEventAdvertisingSetupFailedIndication_SetupFailReason_gAttInsufficientEncryption_c:
                    shell_write(gAttInsufficientEncryption_c);
                    break;

                case GAPGenericEventAdvertisingSetupFailedIndication_SetupFailReason_gAttUnsupportedGroupType_c:
                    shell_write(gAttUnsupportedGroupType_c);
                    break;

                case GAPGenericEventAdvertisingSetupFailedIndication_SetupFailReason_gAttInsufficientResources_c:
                    shell_write(gAttInsufficientResources_c);
                    break;

                case GAPGenericEventAdvertisingSetupFailedIndication_SetupFailReason_gGattAnotherProcedureInProgress_c:
                    shell_write(gGattAnotherProcedureInProgress_c);
                    break;

                case GAPGenericEventAdvertisingSetupFailedIndication_SetupFailReason_gGattLongAttributePacketsCorrupted_c:
                    shell_write(gGattLongAttributePacketsCorrupted_c);
                    break;

                case GAPGenericEventAdvertisingSetupFailedIndication_SetupFailReason_gGattMultipleAttributesOverflow_c:
                    shell_write(gGattMultipleAttributesOverflow_c);
                    break;

                case GAPGenericEventAdvertisingSetupFailedIndication_SetupFailReason_gGattUnexpectedReadMultipleResponseLength_c:
                    shell_write(gGattUnexpectedReadMultipleResponseLength_c);
                    break;

                case GAPGenericEventAdvertisingSetupFailedIndication_SetupFailReason_gGattInvalidValueLength_c:
                    shell_write(gGattInvalidValueLength_c);
                    break;

                case GAPGenericEventAdvertisingSetupFailedIndication_SetupFailReason_gGattServerTimeout_c:
                    shell_write(gGattServerTimeout_c);
                    break;

                case GAPGenericEventAdvertisingSetupFailedIndication_SetupFailReason_gGattIndicationAlreadyInProgress_c:
                    shell_write(gGattIndicationAlreadyInProgress_c);
                    break;

                case GAPGenericEventAdvertisingSetupFailedIndication_SetupFailReason_gGattClientConfirmationTimeout_c:
                    shell_write(gGattClientConfirmationTimeout_c);
                    break;

                case GAPGenericEventAdvertisingSetupFailedIndication_SetupFailReason_gGapAdvDataTooLong_c:
                    shell_write(gGapAdvDataTooLong_c);
                    break;

                case GAPGenericEventAdvertisingSetupFailedIndication_SetupFailReason_gGapScanRspDataTooLong_c:
                    shell_write(gGapScanRspDataTooLong_c);
                    break;

                case GAPGenericEventAdvertisingSetupFailedIndication_SetupFailReason_gGapDeviceNotBonded_c:
                    shell_write(gGapDeviceNotBonded_c);
                    break;

                case GAPGenericEventAdvertisingSetupFailedIndication_SetupFailReason_gDevDbCccdLimitReached_c:
                    shell_write(gDevDbCccdLimitReached_c);
                    break;

                case GAPGenericEventAdvertisingSetupFailedIndication_SetupFailReason_gDevDbCccdNotFound_c:
                    shell_write(gDevDbCccdNotFound_c);
                    break;

                case GAPGenericEventAdvertisingSetupFailedIndication_SetupFailReason_gGattDbInvalidHandle_c:
                    shell_write(gGattDbInvalidHandle_c);
                    break;

                case GAPGenericEventAdvertisingSetupFailedIndication_SetupFailReason_gGattDbCharacteristicNotFound_c:
                    shell_write(gGattDbCharacteristicNotFound_c);
                    break;

                case GAPGenericEventAdvertisingSetupFailedIndication_SetupFailReason_gGattDbCccdNotFound_c:
                    shell_write(gGattDbCccdNotFound_c);
                    break;

                case GAPGenericEventAdvertisingSetupFailedIndication_SetupFailReason_gGattDbServiceNotFound_c:
                    shell_write(gGattDbServiceNotFound_c);
                    break;

                case GAPGenericEventAdvertisingSetupFailedIndication_SetupFailReason_gGattDbDescriptorNotFound_c:
                    shell_write(gGattDbDescriptorNotFound_c);
                    break;

                default:
                    shell_printf("Unrecognized status 0x%04X", container->Data.GAPGenericEventAdvertisingSetupFailedIndication.SetupFailReason);
                    break;
            }

            break;

        case 0x478C:
            shell_write("\r\nGAPGenericEventAdvertisingParametersSetupCompleteIndication");
            break;

        case 0x478D:
            shell_write("\r\nGAPGenericEventAdvertisingDataSetupCompleteIndication");
            break;

        case 0x478E:
            shell_write("\r\nGAPGenericEventWhiteListSizeReadIndication");
            break;

        case 0x478F:
            shell_write("\r\nGAPGenericEventDeviceAddedToWhiteListIndication");
            break;

        case 0x4790:
            shell_write("\r\nGAPGenericEventDeviceRemovedFromWhiteListIndication");
            break;

        case 0x4791:
            shell_write("\r\nGAPGenericEventWhiteListClearedIndication");
            break;

        case 0x4792:
            shell_write("\r\nGAPGenericEventRandomAddressReadyIndication->");
            shell_writeHex(container->Data.GAPGenericEventRandomAddressReadyIndication.Address, 6);
            break;

        case 0x4793:
            shell_write("\r\nGAPGenericEventCreateConnectionCanceledIndication");
            break;

        case 0x4794:
            shell_write("\r\nGAPGenericEventPublicAddressReadIndication");
            SHELL_GapPrintPublicAddr(&(container->Data.GAPGenericEventPublicAddressReadIndication));
            break;

        case 0x4795:
            shell_write("\r\nGAPGenericEventAdvTxPowerLevelReadIndication");
            break;

        case 0x4796:
            shell_write("\r\nGAPGenericEventPrivateResolvableAddressVerifiedIndication");
            break;

        case 0x4797:
            shell_write("\r\nGAPGenericEventRandomAddressSetIndication");
            break;

        case 0x4798:
            shell_write("\r\nGAPAdvertisingEventStateChangedIndication");
            break;

        case 0x4799:
            shell_write("\r\nGAPAdvertisingEventCommandFailedIndication");
            shell_write(" -> ");

            switch (container->Data.GAPAdvertisingEventCommandFailedIndication.FailReason)
            {
                case GAPAdvertisingEventCommandFailedIndication_FailReason_gBleSuccess_c:
                    shell_write(gBleSuccess_c);
                    break;

                case GAPAdvertisingEventCommandFailedIndication_FailReason_gBleInvalidParameter_c:
                    shell_write(gBleInvalidParameter_c);
                    break;

                case GAPAdvertisingEventCommandFailedIndication_FailReason_gBleOverflow_c:
                    shell_write(gBleOverflow_c);
                    break;

                case GAPAdvertisingEventCommandFailedIndication_FailReason_gBleUnavailable_c:
                    shell_write(gBleUnavailable_c);
                    break;

                case GAPAdvertisingEventCommandFailedIndication_FailReason_gBleFeatureNotSupported_c:
                    shell_write(gBleFeatureNotSupported_c);
                    break;

                case GAPAdvertisingEventCommandFailedIndication_FailReason_gBleOutOfMemory_c:
                    shell_write(gBleOutOfMemory_c);
                    break;

                case GAPAdvertisingEventCommandFailedIndication_FailReason_gBleAlreadyInitialized_c:
                    shell_write(gBleAlreadyInitialized_c);
                    break;

                case GAPAdvertisingEventCommandFailedIndication_FailReason_gBleOsError_c:
                    shell_write(gBleOsError_c);
                    break;

                case GAPAdvertisingEventCommandFailedIndication_FailReason_gBleUnexpectedError_c:
                    shell_write(gBleUnexpectedError_c);
                    break;

                case GAPAdvertisingEventCommandFailedIndication_FailReason_gBleInvalidState_c:
                    shell_write(gBleInvalidState_c);
                    break;

                case GAPAdvertisingEventCommandFailedIndication_FailReason_gHciUnknownHciCommand_c:
                    shell_write(gHciUnknownHciCommand_c);
                    break;

                case GAPAdvertisingEventCommandFailedIndication_FailReason_gHciUnknownConnectionIdentifier_c:
                    shell_write(gHciUnknownConnectionIdentifier_c);
                    break;

                case GAPAdvertisingEventCommandFailedIndication_FailReason_gHciHardwareFailure_c:
                    shell_write(gHciHardwareFailure_c);
                    break;

                case GAPAdvertisingEventCommandFailedIndication_FailReason_gHciPageTimeout_c:
                    shell_write(gHciPageTimeout_c);
                    break;

                case GAPAdvertisingEventCommandFailedIndication_FailReason_gHciAuthenticationFailure_c:
                    shell_write(gHciAuthenticationFailure_c);
                    break;

                case GAPAdvertisingEventCommandFailedIndication_FailReason_gHciPinOrKeyMissing_c:
                    shell_write(gHciPinOrKeyMissing_c);
                    break;

                case GAPAdvertisingEventCommandFailedIndication_FailReason_gHciMemoryCapacityExceeded_c:
                    shell_write(gHciMemoryCapacityExceeded_c);
                    break;

                case GAPAdvertisingEventCommandFailedIndication_FailReason_gHciConnectionTimeout_c:
                    shell_write(gHciConnectionTimeout_c);
                    break;

                case GAPAdvertisingEventCommandFailedIndication_FailReason_gHciConnectionLimitExceeded_c:
                    shell_write(gHciConnectionLimitExceeded_c);
                    break;

                case GAPAdvertisingEventCommandFailedIndication_FailReason_gHciSynchronousConnectionLimitToADeviceExceeded_c:
                    shell_write(gHciSynchronousConnectionLimitToADeviceExceeded_c);
                    break;

                case GAPAdvertisingEventCommandFailedIndication_FailReason_gHciAclConnectionAlreadyExists_c:
                    shell_write(gHciAclConnectionAlreadyExists_c);
                    break;

                case GAPAdvertisingEventCommandFailedIndication_FailReason_gHciCommandDisallowed_c:
                    shell_write(gHciCommandDisallowed_c);
                    break;

                case GAPAdvertisingEventCommandFailedIndication_FailReason_gHciConnectionRejectedDueToLimitedResources_c:
                    shell_write(gHciConnectionRejectedDueToLimitedResources_c);
                    break;

                case GAPAdvertisingEventCommandFailedIndication_FailReason_gHciConnectionRejectedDueToSecurityReasons_c:
                    shell_write(gHciConnectionRejectedDueToSecurityReasons_c);
                    break;

                case GAPAdvertisingEventCommandFailedIndication_FailReason_gHciConnectionRejectedDueToUnacceptableBdAddr_c:
                    shell_write(gHciConnectionRejectedDueToUnacceptableBdAddr_c);
                    break;

                case GAPAdvertisingEventCommandFailedIndication_FailReason_gHciConnectionAcceptTimeoutExceeded_c:
                    shell_write(gHciConnectionAcceptTimeoutExceeded_c);
                    break;

                case GAPAdvertisingEventCommandFailedIndication_FailReason_gHciUnsupportedFeatureOrParameterValue_c:
                    shell_write(gHciUnsupportedFeatureOrParameterValue_c);
                    break;

                case GAPAdvertisingEventCommandFailedIndication_FailReason_gHciInvalidHciCommandParameters_c:
                    shell_write(gHciInvalidHciCommandParameters_c);
                    break;

                case GAPAdvertisingEventCommandFailedIndication_FailReason_gHciRemoteUserTerminatedConnection_c:
                    shell_write(gHciRemoteUserTerminatedConnection_c);
                    break;

                case GAPAdvertisingEventCommandFailedIndication_FailReason_gHciRemoteDeviceTerminatedConnectionLowResources_c:
                    shell_write(gHciRemoteDeviceTerminatedConnectionLowResources_c);
                    break;

                case GAPAdvertisingEventCommandFailedIndication_FailReason_gHciRemoteDeviceTerminatedConnectionPowerOff_c:
                    shell_write(gHciRemoteDeviceTerminatedConnectionPowerOff_c);
                    break;

                case GAPAdvertisingEventCommandFailedIndication_FailReason_gHciConnectionTerminatedByLocalHost_c:
                    shell_write(gHciConnectionTerminatedByLocalHost_c);
                    break;

                case GAPAdvertisingEventCommandFailedIndication_FailReason_gHciRepeatedAttempts_c:
                    shell_write(gHciRepeatedAttempts_c);
                    break;

                case GAPAdvertisingEventCommandFailedIndication_FailReason_gHciPairingNotAllowed_c:
                    shell_write(gHciPairingNotAllowed_c);
                    break;

                case GAPAdvertisingEventCommandFailedIndication_FailReason_gHciUnknownLpmPdu_c:
                    shell_write(gHciUnknownLpmPdu_c);
                    break;

                case GAPAdvertisingEventCommandFailedIndication_FailReason_gHciUnsupportedRemoteFeature_c:
                    shell_write(gHciUnsupportedRemoteFeature_c);
                    break;

                case GAPAdvertisingEventCommandFailedIndication_FailReason_gHciScoOffsetRejected_c:
                    shell_write(gHciScoOffsetRejected_c);
                    break;

                case GAPAdvertisingEventCommandFailedIndication_FailReason_gHciScoIntervalRejected_c:
                    shell_write(gHciScoIntervalRejected_c);
                    break;

                case GAPAdvertisingEventCommandFailedIndication_FailReason_gHciScoAirModeRejected_c:
                    shell_write(gHciScoAirModeRejected_c);
                    break;

                case GAPAdvertisingEventCommandFailedIndication_FailReason_gHciInvalidLpmParameters_c:
                    shell_write(gHciInvalidLpmParameters_c);
                    break;

                case GAPAdvertisingEventCommandFailedIndication_FailReason_gHciUnspecifiedError_c:
                    shell_write(gHciUnspecifiedError_c);
                    break;

                case GAPAdvertisingEventCommandFailedIndication_FailReason_gHciUnsupportedLpmParameterValue_c:
                    shell_write(gHciUnsupportedLpmParameterValue_c);
                    break;

                case GAPAdvertisingEventCommandFailedIndication_FailReason_gHciRoleChangeNotAllowed_c:
                    shell_write(gHciRoleChangeNotAllowed_c);
                    break;

                case GAPAdvertisingEventCommandFailedIndication_FailReason_gHciLLResponseTimeout_c:
                    shell_write(gHciLLResponseTimeout_c);
                    break;

                case GAPAdvertisingEventCommandFailedIndication_FailReason_gHciLmpErrorTransactionCollision_c:
                    shell_write(gHciLmpErrorTransactionCollision_c);
                    break;

                case GAPAdvertisingEventCommandFailedIndication_FailReason_gHciLmpPduNotAllowed_c:
                    shell_write(gHciLmpPduNotAllowed_c);
                    break;

                case GAPAdvertisingEventCommandFailedIndication_FailReason_gHciEncryptionModeNotAcceptable_c:
                    shell_write(gHciEncryptionModeNotAcceptable_c);
                    break;

                case GAPAdvertisingEventCommandFailedIndication_FailReason_gHciLinkKeyCannotBeChanged_c:
                    shell_write(gHciLinkKeyCannotBeChanged_c);
                    break;

                case GAPAdvertisingEventCommandFailedIndication_FailReason_gHciRequestedQosNotSupported_c:
                    shell_write(gHciRequestedQosNotSupported_c);
                    break;

                case GAPAdvertisingEventCommandFailedIndication_FailReason_gHciInstantPassed_c:
                    shell_write(gHciInstantPassed_c);
                    break;

                case GAPAdvertisingEventCommandFailedIndication_FailReason_gHciPairingWithUnitKeyNotSupported_c:
                    shell_write(gHciPairingWithUnitKeyNotSupported_c);
                    break;

                case GAPAdvertisingEventCommandFailedIndication_FailReason_gHciDifferentTransactionCollision_c:
                    shell_write(gHciDifferentTransactionCollision_c);
                    break;

                case GAPAdvertisingEventCommandFailedIndication_FailReason_gHciReserved_0x2B_c:
                    shell_write(gHciReserved_0x2B_c);
                    break;

                case GAPAdvertisingEventCommandFailedIndication_FailReason_gHciQosNotAcceptableParameter_c:
                    shell_write(gHciQosNotAcceptableParameter_c);
                    break;

                case GAPAdvertisingEventCommandFailedIndication_FailReason_gHciQosRejected_c:
                    shell_write(gHciQosRejected_c);
                    break;

                case GAPAdvertisingEventCommandFailedIndication_FailReason_gHciChannelClassificationNotSupported_c:
                    shell_write(gHciChannelClassificationNotSupported_c);
                    break;

                case GAPAdvertisingEventCommandFailedIndication_FailReason_gHciInsufficientSecurity_c:
                    shell_write(gHciInsufficientSecurity_c);
                    break;

                case GAPAdvertisingEventCommandFailedIndication_FailReason_gHciParameterOutOfMandatoryRange_c:
                    shell_write(gHciParameterOutOfMandatoryRange_c);
                    break;

                case GAPAdvertisingEventCommandFailedIndication_FailReason_gHciReserved_0x31_c:
                    shell_write(gHciReserved_0x31_c);
                    break;

                case GAPAdvertisingEventCommandFailedIndication_FailReason_gHciRoleSwitchPending_c:
                    shell_write(gHciRoleSwitchPending_c);
                    break;

                case GAPAdvertisingEventCommandFailedIndication_FailReason_gHciReserved_0x33_c:
                    shell_write(gHciReserved_0x33_c);
                    break;

                case GAPAdvertisingEventCommandFailedIndication_FailReason_gHciReservedSlotViolation_c:
                    shell_write(gHciReservedSlotViolation_c);
                    break;

                case GAPAdvertisingEventCommandFailedIndication_FailReason_gHciRoleSwitchFailed_c:
                    shell_write(gHciRoleSwitchFailed_c);
                    break;

                case GAPAdvertisingEventCommandFailedIndication_FailReason_gHciExtendedInquiryResponseTooLarge_c:
                    shell_write(gHciExtendedInquiryResponseTooLarge_c);
                    break;

                case GAPAdvertisingEventCommandFailedIndication_FailReason_gHciSecureSimplePairingNotSupportedByHost_c:
                    shell_write(gHciSecureSimplePairingNotSupportedByHost_c);
                    break;

                case GAPAdvertisingEventCommandFailedIndication_FailReason_gHciHostBusyPairing_c:
                    shell_write(gHciHostBusyPairing_c);
                    break;

                case GAPAdvertisingEventCommandFailedIndication_FailReason_gHciConnectionRejectedDueToNoSuitableChannelFound_c:
                    shell_write(gHciConnectionRejectedDueToNoSuitableChannelFound_c);
                    break;

                case GAPAdvertisingEventCommandFailedIndication_FailReason_gHciControllerBusy_c:
                    shell_write(gHciControllerBusy_c);
                    break;

                case GAPAdvertisingEventCommandFailedIndication_FailReason_gHciUnacceptableConnectionParameters_c:
                    shell_write(gHciUnacceptableConnectionParameters_c);
                    break;

                case GAPAdvertisingEventCommandFailedIndication_FailReason_gHciDirectedAdvertisingTimeout_c:
                    shell_write(gHciDirectedAdvertisingTimeout_c);
                    break;

                case GAPAdvertisingEventCommandFailedIndication_FailReason_gHciConnectionTerminatedDueToMicFailure_c:
                    shell_write(gHciConnectionTerminatedDueToMicFailure_c);
                    break;

                case GAPAdvertisingEventCommandFailedIndication_FailReason_gHciConnectionFailedToBeEstablished_c:
                    shell_write(gHciConnectionFailedToBeEstablished_c);
                    break;

                case GAPAdvertisingEventCommandFailedIndication_FailReason_gHciMacConnectionFailed_c:
                    shell_write(gHciMacConnectionFailed_c);
                    break;

                case GAPAdvertisingEventCommandFailedIndication_FailReason_gHciCoarseClockAdjustmentRejected_c:
                    shell_write(gHciCoarseClockAdjustmentRejected_c);
                    break;

                case GAPAdvertisingEventCommandFailedIndication_FailReason_gHciAlreadyInit_c:
                    shell_write(gHciAlreadyInit_c);
                    break;

                case GAPAdvertisingEventCommandFailedIndication_FailReason_gHciInvalidParameter_c:
                    shell_write(gHciInvalidParameter_c);
                    break;

                case GAPAdvertisingEventCommandFailedIndication_FailReason_gHciCallbackNotInstalled_c:
                    shell_write(gHciCallbackNotInstalled_c);
                    break;

                case GAPAdvertisingEventCommandFailedIndication_FailReason_gHciCallbackAlreadyInstalled_c:
                    shell_write(gHciCallbackAlreadyInstalled_c);
                    break;

                case GAPAdvertisingEventCommandFailedIndication_FailReason_gHciCommandNotSupported_c:
                    shell_write(gHciCommandNotSupported_c);
                    break;

                case GAPAdvertisingEventCommandFailedIndication_FailReason_gHciEventNotSupported_c:
                    shell_write(gHciEventNotSupported_c);
                    break;

                case GAPAdvertisingEventCommandFailedIndication_FailReason_gHciTransportError_c:
                    shell_write(gHciTransportError_c);
                    break;

                case GAPAdvertisingEventCommandFailedIndication_FailReason_gL2caAlreadyInit_c:
                    shell_write(gL2caAlreadyInit_c);
                    break;

                case GAPAdvertisingEventCommandFailedIndication_FailReason_gL2caInsufficientResources_c:
                    shell_write(gL2caInsufficientResources_c);
                    break;

                case GAPAdvertisingEventCommandFailedIndication_FailReason_gL2caCallbackNotInstalled_c:
                    shell_write(gL2caCallbackNotInstalled_c);
                    break;

                case GAPAdvertisingEventCommandFailedIndication_FailReason_gL2caCallbackAlreadyInstalled_c:
                    shell_write(gL2caCallbackAlreadyInstalled_c);
                    break;

                case GAPAdvertisingEventCommandFailedIndication_FailReason_gL2caLePsmInvalid_c:
                    shell_write(gL2caLePsmInvalid_c);
                    break;

                case GAPAdvertisingEventCommandFailedIndication_FailReason_gL2caLePsmAlreadyRegistered_c:
                    shell_write(gL2caLePsmAlreadyRegistered_c);
                    break;

                case GAPAdvertisingEventCommandFailedIndication_FailReason_gL2caLePsmNotRegistered_c:
                    shell_write(gL2caLePsmNotRegistered_c);
                    break;

                case GAPAdvertisingEventCommandFailedIndication_FailReason_gL2caLePsmInsufficientResources_c:
                    shell_write(gL2caLePsmInsufficientResources_c);
                    break;

                case GAPAdvertisingEventCommandFailedIndication_FailReason_gL2caChannelInvalid_c:
                    shell_write(gL2caChannelInvalid_c);
                    break;

                case GAPAdvertisingEventCommandFailedIndication_FailReason_gL2caChannelClosed_c:
                    shell_write(gL2caChannelClosed_c);
                    break;

                case GAPAdvertisingEventCommandFailedIndication_FailReason_gL2caChannelAlreadyConnected_c:
                    shell_write(gL2caChannelAlreadyConnected_c);
                    break;

                case GAPAdvertisingEventCommandFailedIndication_FailReason_gL2caConnectionParametersRejected_c:
                    shell_write(gL2caConnectionParametersRejected_c);
                    break;

                case GAPAdvertisingEventCommandFailedIndication_FailReason_gL2caChannelBusy_c:
                    shell_write(gL2caChannelBusy_c);
                    break;

                case GAPAdvertisingEventCommandFailedIndication_FailReason_gL2caInvalidParameter_c:
                    shell_write(gL2caInvalidParameter_c);
                    break;

                case GAPAdvertisingEventCommandFailedIndication_FailReason_gL2caError_c:
                    shell_write(gL2caError_c);
                    break;

                case GAPAdvertisingEventCommandFailedIndication_FailReason_gSmNullCBFunction_c:
                    shell_write(gSmNullCBFunction_c);
                    break;

                case GAPAdvertisingEventCommandFailedIndication_FailReason_gSmCommandNotSupported_c:
                    shell_write(gSmCommandNotSupported_c);
                    break;

                case GAPAdvertisingEventCommandFailedIndication_FailReason_gSmUnexpectedCommand_c:
                    shell_write(gSmUnexpectedCommand_c);
                    break;

                case GAPAdvertisingEventCommandFailedIndication_FailReason_gSmInvalidCommandCode_c:
                    shell_write(gSmInvalidCommandCode_c);
                    break;

                case GAPAdvertisingEventCommandFailedIndication_FailReason_gSmInvalidCommandLength_c:
                    shell_write(gSmInvalidCommandLength_c);
                    break;

                case GAPAdvertisingEventCommandFailedIndication_FailReason_gSmInvalidCommandParameter_c:
                    shell_write(gSmInvalidCommandParameter_c);
                    break;

                case GAPAdvertisingEventCommandFailedIndication_FailReason_gSmInvalidDeviceId_c:
                    shell_write(gSmInvalidDeviceId_c);
                    break;

                case GAPAdvertisingEventCommandFailedIndication_FailReason_gSmInvalidInternalOperation_c:
                    shell_write(gSmInvalidInternalOperation_c);
                    break;

                case GAPAdvertisingEventCommandFailedIndication_FailReason_gSmInvalidConnectionHandle_c:
                    shell_write(gSmInvalidConnectionHandle_c);
                    break;

                case GAPAdvertisingEventCommandFailedIndication_FailReason_gSmInproperKeyDistributionField_c:
                    shell_write(gSmInproperKeyDistributionField_c);
                    break;

                case GAPAdvertisingEventCommandFailedIndication_FailReason_gSmUnexpectedKeyType_c:
                    shell_write(gSmUnexpectedKeyType_c);
                    break;

                case GAPAdvertisingEventCommandFailedIndication_FailReason_gSmUnexpectedPairingTerminationReason_c:
                    shell_write(gSmUnexpectedPairingTerminationReason_c);
                    break;

                case GAPAdvertisingEventCommandFailedIndication_FailReason_gSmUnexpectedKeyset_c:
                    shell_write(gSmUnexpectedKeyset_c);
                    break;

                case GAPAdvertisingEventCommandFailedIndication_FailReason_gSmSmpTimeoutOccurred_c:
                    shell_write(gSmSmpTimeoutOccurred_c);
                    break;

                case GAPAdvertisingEventCommandFailedIndication_FailReason_gSmUnknownSmpPacketType_c:
                    shell_write(gSmUnknownSmpPacketType_c);
                    break;

                case GAPAdvertisingEventCommandFailedIndication_FailReason_gSmInvalidSmpPacketLength_c:
                    shell_write(gSmInvalidSmpPacketLength_c);
                    break;

                case GAPAdvertisingEventCommandFailedIndication_FailReason_gSmInvalidSmpPacketParameter_c:
                    shell_write(gSmInvalidSmpPacketParameter_c);
                    break;

                case GAPAdvertisingEventCommandFailedIndication_FailReason_gSmReceivedUnexpectedSmpPacket_c:
                    shell_write(gSmReceivedUnexpectedSmpPacket_c);
                    break;

                case GAPAdvertisingEventCommandFailedIndication_FailReason_gSmReceivedSmpPacketFromUnknownDevice_c:
                    shell_write(gSmReceivedSmpPacketFromUnknownDevice_c);
                    break;

                case GAPAdvertisingEventCommandFailedIndication_FailReason_gSmReceivedUnexpectedHciEvent_c:
                    shell_write(gSmReceivedUnexpectedHciEvent_c);
                    break;

                case GAPAdvertisingEventCommandFailedIndication_FailReason_gSmReceivedHciEventFromUnknownDevice_c:
                    shell_write(gSmReceivedHciEventFromUnknownDevice_c);
                    break;

                case GAPAdvertisingEventCommandFailedIndication_FailReason_gSmInvalidHciEventParameter_c:
                    shell_write(gSmInvalidHciEventParameter_c);
                    break;

                case GAPAdvertisingEventCommandFailedIndication_FailReason_gSmLlConnectionEncryptionInProgress_c:
                    shell_write(gSmLlConnectionEncryptionInProgress_c);
                    break;

                case GAPAdvertisingEventCommandFailedIndication_FailReason_gSmLlConnectionEncryptionFailure_c:
                    shell_write(gSmLlConnectionEncryptionFailure_c);
                    break;

                case GAPAdvertisingEventCommandFailedIndication_FailReason_gSmInsufficientResources_c:
                    shell_write(gSmInsufficientResources_c);
                    break;

                case GAPAdvertisingEventCommandFailedIndication_FailReason_gSmOobDataAddressMismatch_c:
                    shell_write(gSmOobDataAddressMismatch_c);
                    break;

//                case GAPAdvertisingEventCommandFailedIndication_FailReason_gSmSmpPacketReceivedAfterTimeoutOccurred_c:
//                    shell_write(gSmSmpPacketReceivedAfterTimeoutOccurred_c);
//                    break;

                case GAPAdvertisingEventCommandFailedIndication_FailReason_gSmPairingErrorPasskeyEntryFailed_c:
                    shell_write(gSmPairingErrorPasskeyEntryFailed_c);
                    break;

                case GAPAdvertisingEventCommandFailedIndication_FailReason_gSmPairingErrorConfirmValueFailed_c:
                    shell_write(gSmPairingErrorConfirmValueFailed_c);
                    break;

                case GAPAdvertisingEventCommandFailedIndication_FailReason_gSmPairingErrorCommandNotSupported_c:
                    shell_write(gSmPairingErrorCommandNotSupported_c);
                    break;

                case GAPAdvertisingEventCommandFailedIndication_FailReason_gSmPairingErrorInvalidParameters_c:
                    shell_write(gSmPairingErrorInvalidParameters_c);
                    break;

                case GAPAdvertisingEventCommandFailedIndication_FailReason_gSmPairingErrorUnknownReason_c:
                    shell_write(gSmPairingErrorUnknownReason_c);
                    break;

                case GAPAdvertisingEventCommandFailedIndication_FailReason_gSmTbResolvableAddressDoesNotMatchIrk_c:
                    shell_write(gSmTbResolvableAddressDoesNotMatchIrk_c);
                    break;

                case GAPAdvertisingEventCommandFailedIndication_FailReason_gSmTbInvalidDataSignature_c:
                    shell_write(gSmTbInvalidDataSignature_c);
                    break;

                case GAPAdvertisingEventCommandFailedIndication_FailReason_gAttInvalidHandle_c:
                    shell_write(gAttInvalidHandle_c);
                    break;

                case GAPAdvertisingEventCommandFailedIndication_FailReason_gAttReadNotPermitted_c:
                    shell_write(gAttReadNotPermitted_c);
                    break;

                case GAPAdvertisingEventCommandFailedIndication_FailReason_gAttWriteNotPermitted_c:
                    shell_write(gAttWriteNotPermitted_c);
                    break;

                case GAPAdvertisingEventCommandFailedIndication_FailReason_gAttInvalidPdu_c:
                    shell_write(gAttInvalidPdu_c);
                    break;

                case GAPAdvertisingEventCommandFailedIndication_FailReason_gAttInsufficientAuthentication_c:
                    shell_write(gAttInsufficientAuthentication_c);
                    break;

                case GAPAdvertisingEventCommandFailedIndication_FailReason_gAttRequestNotSupported_c:
                    shell_write(gAttRequestNotSupported_c);
                    break;

                case GAPAdvertisingEventCommandFailedIndication_FailReason_gAttInvalidOffset_c:
                    shell_write(gAttInvalidOffset_c);
                    break;

                case GAPAdvertisingEventCommandFailedIndication_FailReason_gAttInsufficientAuthorization_c:
                    shell_write(gAttInsufficientAuthorization_c);
                    break;

                case GAPAdvertisingEventCommandFailedIndication_FailReason_gAttPrepareQueueFull_c:
                    shell_write(gAttPrepareQueueFull_c);
                    break;

                case GAPAdvertisingEventCommandFailedIndication_FailReason_gAttAttributeNotFound_c:
                    shell_write(gAttAttributeNotFound_c);
                    break;

                case GAPAdvertisingEventCommandFailedIndication_FailReason_gAttAttributeNotLong_c:
                    shell_write(gAttAttributeNotLong_c);
                    break;

                case GAPAdvertisingEventCommandFailedIndication_FailReason_gAttInsufficientEncryptionKeySize_c:
                    shell_write(gAttInsufficientEncryptionKeySize_c);
                    break;

                case GAPAdvertisingEventCommandFailedIndication_FailReason_gAttInvalidAttributeValueLength_c:
                    shell_write(gAttInvalidAttributeValueLength_c);
                    break;

                case GAPAdvertisingEventCommandFailedIndication_FailReason_gAttUnlikelyor_c:
                    shell_write(gAttUnlikelyor_c);
                    break;

                case GAPAdvertisingEventCommandFailedIndication_FailReason_gAttInsufficientEncryption_c:
                    shell_write(gAttInsufficientEncryption_c);
                    break;

                case GAPAdvertisingEventCommandFailedIndication_FailReason_gAttUnsupportedGroupType_c:
                    shell_write(gAttUnsupportedGroupType_c);
                    break;

                case GAPAdvertisingEventCommandFailedIndication_FailReason_gAttInsufficientResources_c:
                    shell_write(gAttInsufficientResources_c);
                    break;

                case GAPAdvertisingEventCommandFailedIndication_FailReason_gGattAnotherProcedureInProgress_c:
                    shell_write(gGattAnotherProcedureInProgress_c);
                    break;

                case GAPAdvertisingEventCommandFailedIndication_FailReason_gGattLongAttributePacketsCorrupted_c:
                    shell_write(gGattLongAttributePacketsCorrupted_c);
                    break;

                case GAPAdvertisingEventCommandFailedIndication_FailReason_gGattMultipleAttributesOverflow_c:
                    shell_write(gGattMultipleAttributesOverflow_c);
                    break;

                case GAPAdvertisingEventCommandFailedIndication_FailReason_gGattUnexpectedReadMultipleResponseLength_c:
                    shell_write(gGattUnexpectedReadMultipleResponseLength_c);
                    break;

                case GAPAdvertisingEventCommandFailedIndication_FailReason_gGattInvalidValueLength_c:
                    shell_write(gGattInvalidValueLength_c);
                    break;

                case GAPAdvertisingEventCommandFailedIndication_FailReason_gGattServerTimeout_c:
                    shell_write(gGattServerTimeout_c);
                    break;

                case GAPAdvertisingEventCommandFailedIndication_FailReason_gGattIndicationAlreadyInProgress_c:
                    shell_write(gGattIndicationAlreadyInProgress_c);
                    break;

                case GAPAdvertisingEventCommandFailedIndication_FailReason_gGattClientConfirmationTimeout_c:
                    shell_write(gGattClientConfirmationTimeout_c);
                    break;

                case GAPAdvertisingEventCommandFailedIndication_FailReason_gGapAdvDataTooLong_c:
                    shell_write(gGapAdvDataTooLong_c);
                    break;

                case GAPAdvertisingEventCommandFailedIndication_FailReason_gGapScanRspDataTooLong_c:
                    shell_write(gGapScanRspDataTooLong_c);
                    break;

                case GAPAdvertisingEventCommandFailedIndication_FailReason_gGapDeviceNotBonded_c:
                    shell_write(gGapDeviceNotBonded_c);
                    break;

                case GAPAdvertisingEventCommandFailedIndication_FailReason_gDevDbCccdLimitReached_c:
                    shell_write(gDevDbCccdLimitReached_c);
                    break;

                case GAPAdvertisingEventCommandFailedIndication_FailReason_gDevDbCccdNotFound_c:
                    shell_write(gDevDbCccdNotFound_c);
                    break;

                case GAPAdvertisingEventCommandFailedIndication_FailReason_gGattDbInvalidHandle_c:
                    shell_write(gGattDbInvalidHandle_c);
                    break;

                case GAPAdvertisingEventCommandFailedIndication_FailReason_gGattDbCharacteristicNotFound_c:
                    shell_write(gGattDbCharacteristicNotFound_c);
                    break;

                case GAPAdvertisingEventCommandFailedIndication_FailReason_gGattDbCccdNotFound_c:
                    shell_write(gGattDbCccdNotFound_c);
                    break;

                case GAPAdvertisingEventCommandFailedIndication_FailReason_gGattDbServiceNotFound_c:
                    shell_write(gGattDbServiceNotFound_c);
                    break;

                case GAPAdvertisingEventCommandFailedIndication_FailReason_gGattDbDescriptorNotFound_c:
                    shell_write(gGattDbDescriptorNotFound_c);
                    break;

                default:
                    shell_printf("Unrecognized status 0x%04X", container->Data.GAPAdvertisingEventCommandFailedIndication.FailReason);
                    break;
            }

            break;

        case 0x479A:
            shell_write("\r\nGAPScanningEventStateChangedIndication");
            break;

        case 0x479B:
            shell_write("\r\nGAPScanningEventCommandFailedIndication");
            shell_write(" -> ");

            switch (container->Data.GAPScanningEventCommandFailedIndication.FailReason)
            {
                case GAPScanningEventCommandFailedIndication_FailReason_gBleSuccess_c:
                    shell_write(gBleSuccess_c);
                    break;

                case GAPScanningEventCommandFailedIndication_FailReason_gBleInvalidParameter_c:
                    shell_write(gBleInvalidParameter_c);
                    break;

                case GAPScanningEventCommandFailedIndication_FailReason_gBleOverflow_c:
                    shell_write(gBleOverflow_c);
                    break;

                case GAPScanningEventCommandFailedIndication_FailReason_gBleUnavailable_c:
                    shell_write(gBleUnavailable_c);
                    break;

                case GAPScanningEventCommandFailedIndication_FailReason_gBleFeatureNotSupported_c:
                    shell_write(gBleFeatureNotSupported_c);
                    break;

                case GAPScanningEventCommandFailedIndication_FailReason_gBleOutOfMemory_c:
                    shell_write(gBleOutOfMemory_c);
                    break;

                case GAPScanningEventCommandFailedIndication_FailReason_gBleAlreadyInitialized_c:
                    shell_write(gBleAlreadyInitialized_c);
                    break;

                case GAPScanningEventCommandFailedIndication_FailReason_gBleOsError_c:
                    shell_write(gBleOsError_c);
                    break;

                case GAPScanningEventCommandFailedIndication_FailReason_gBleUnexpectedError_c:
                    shell_write(gBleUnexpectedError_c);
                    break;

                case GAPScanningEventCommandFailedIndication_FailReason_gBleInvalidState_c:
                    shell_write(gBleInvalidState_c);
                    break;

                case GAPScanningEventCommandFailedIndication_FailReason_gHciUnknownHciCommand_c:
                    shell_write(gHciUnknownHciCommand_c);
                    break;

                case GAPScanningEventCommandFailedIndication_FailReason_gHciUnknownConnectionIdentifier_c:
                    shell_write(gHciUnknownConnectionIdentifier_c);
                    break;

                case GAPScanningEventCommandFailedIndication_FailReason_gHciHardwareFailure_c:
                    shell_write(gHciHardwareFailure_c);
                    break;

                case GAPScanningEventCommandFailedIndication_FailReason_gHciPageTimeout_c:
                    shell_write(gHciPageTimeout_c);
                    break;

                case GAPScanningEventCommandFailedIndication_FailReason_gHciAuthenticationFailure_c:
                    shell_write(gHciAuthenticationFailure_c);
                    break;

                case GAPScanningEventCommandFailedIndication_FailReason_gHciPinOrKeyMissing_c:
                    shell_write(gHciPinOrKeyMissing_c);
                    break;

                case GAPScanningEventCommandFailedIndication_FailReason_gHciMemoryCapacityExceeded_c:
                    shell_write(gHciMemoryCapacityExceeded_c);
                    break;

                case GAPScanningEventCommandFailedIndication_FailReason_gHciConnectionTimeout_c:
                    shell_write(gHciConnectionTimeout_c);
                    break;

                case GAPScanningEventCommandFailedIndication_FailReason_gHciConnectionLimitExceeded_c:
                    shell_write(gHciConnectionLimitExceeded_c);
                    break;

                case GAPScanningEventCommandFailedIndication_FailReason_gHciSynchronousConnectionLimitToADeviceExceeded_c:
                    shell_write(gHciSynchronousConnectionLimitToADeviceExceeded_c);
                    break;

                case GAPScanningEventCommandFailedIndication_FailReason_gHciAclConnectionAlreadyExists_c:
                    shell_write(gHciAclConnectionAlreadyExists_c);
                    break;

                case GAPScanningEventCommandFailedIndication_FailReason_gHciCommandDisallowed_c:
                    shell_write(gHciCommandDisallowed_c);
                    break;

                case GAPScanningEventCommandFailedIndication_FailReason_gHciConnectionRejectedDueToLimitedResources_c:
                    shell_write(gHciConnectionRejectedDueToLimitedResources_c);
                    break;

                case GAPScanningEventCommandFailedIndication_FailReason_gHciConnectionRejectedDueToSecurityReasons_c:
                    shell_write(gHciConnectionRejectedDueToSecurityReasons_c);
                    break;

                case GAPScanningEventCommandFailedIndication_FailReason_gHciConnectionRejectedDueToUnacceptableBdAddr_c:
                    shell_write(gHciConnectionRejectedDueToUnacceptableBdAddr_c);
                    break;

                case GAPScanningEventCommandFailedIndication_FailReason_gHciConnectionAcceptTimeoutExceeded_c:
                    shell_write(gHciConnectionAcceptTimeoutExceeded_c);
                    break;

                case GAPScanningEventCommandFailedIndication_FailReason_gHciUnsupportedFeatureOrParameterValue_c:
                    shell_write(gHciUnsupportedFeatureOrParameterValue_c);
                    break;

                case GAPScanningEventCommandFailedIndication_FailReason_gHciInvalidHciCommandParameters_c:
                    shell_write(gHciInvalidHciCommandParameters_c);
                    break;

                case GAPScanningEventCommandFailedIndication_FailReason_gHciRemoteUserTerminatedConnection_c:
                    shell_write(gHciRemoteUserTerminatedConnection_c);
                    break;

                case GAPScanningEventCommandFailedIndication_FailReason_gHciRemoteDeviceTerminatedConnectionLowResources_c:
                    shell_write(gHciRemoteDeviceTerminatedConnectionLowResources_c);
                    break;

                case GAPScanningEventCommandFailedIndication_FailReason_gHciRemoteDeviceTerminatedConnectionPowerOff_c:
                    shell_write(gHciRemoteDeviceTerminatedConnectionPowerOff_c);
                    break;

                case GAPScanningEventCommandFailedIndication_FailReason_gHciConnectionTerminatedByLocalHost_c:
                    shell_write(gHciConnectionTerminatedByLocalHost_c);
                    break;

                case GAPScanningEventCommandFailedIndication_FailReason_gHciRepeatedAttempts_c:
                    shell_write(gHciRepeatedAttempts_c);
                    break;

                case GAPScanningEventCommandFailedIndication_FailReason_gHciPairingNotAllowed_c:
                    shell_write(gHciPairingNotAllowed_c);
                    break;

                case GAPScanningEventCommandFailedIndication_FailReason_gHciUnknownLpmPdu_c:
                    shell_write(gHciUnknownLpmPdu_c);
                    break;

                case GAPScanningEventCommandFailedIndication_FailReason_gHciUnsupportedRemoteFeature_c:
                    shell_write(gHciUnsupportedRemoteFeature_c);
                    break;

                case GAPScanningEventCommandFailedIndication_FailReason_gHciScoOffsetRejected_c:
                    shell_write(gHciScoOffsetRejected_c);
                    break;

                case GAPScanningEventCommandFailedIndication_FailReason_gHciScoIntervalRejected_c:
                    shell_write(gHciScoIntervalRejected_c);
                    break;

                case GAPScanningEventCommandFailedIndication_FailReason_gHciScoAirModeRejected_c:
                    shell_write(gHciScoAirModeRejected_c);
                    break;

                case GAPScanningEventCommandFailedIndication_FailReason_gHciInvalidLpmParameters_c:
                    shell_write(gHciInvalidLpmParameters_c);
                    break;

                case GAPScanningEventCommandFailedIndication_FailReason_gHciUnspecifiedError_c:
                    shell_write(gHciUnspecifiedError_c);
                    break;

                case GAPScanningEventCommandFailedIndication_FailReason_gHciUnsupportedLpmParameterValue_c:
                    shell_write(gHciUnsupportedLpmParameterValue_c);
                    break;

                case GAPScanningEventCommandFailedIndication_FailReason_gHciRoleChangeNotAllowed_c:
                    shell_write(gHciRoleChangeNotAllowed_c);
                    break;

                case GAPScanningEventCommandFailedIndication_FailReason_gHciLLResponseTimeout_c:
                    shell_write(gHciLLResponseTimeout_c);
                    break;

                case GAPScanningEventCommandFailedIndication_FailReason_gHciLmpErrorTransactionCollision_c:
                    shell_write(gHciLmpErrorTransactionCollision_c);
                    break;

                case GAPScanningEventCommandFailedIndication_FailReason_gHciLmpPduNotAllowed_c:
                    shell_write(gHciLmpPduNotAllowed_c);
                    break;

                case GAPScanningEventCommandFailedIndication_FailReason_gHciEncryptionModeNotAcceptable_c:
                    shell_write(gHciEncryptionModeNotAcceptable_c);
                    break;

                case GAPScanningEventCommandFailedIndication_FailReason_gHciLinkKeyCannotBeChanged_c:
                    shell_write(gHciLinkKeyCannotBeChanged_c);
                    break;

                case GAPScanningEventCommandFailedIndication_FailReason_gHciRequestedQosNotSupported_c:
                    shell_write(gHciRequestedQosNotSupported_c);
                    break;

                case GAPScanningEventCommandFailedIndication_FailReason_gHciInstantPassed_c:
                    shell_write(gHciInstantPassed_c);
                    break;

                case GAPScanningEventCommandFailedIndication_FailReason_gHciPairingWithUnitKeyNotSupported_c:
                    shell_write(gHciPairingWithUnitKeyNotSupported_c);
                    break;

                case GAPScanningEventCommandFailedIndication_FailReason_gHciDifferentTransactionCollision_c:
                    shell_write(gHciDifferentTransactionCollision_c);
                    break;

                case GAPScanningEventCommandFailedIndication_FailReason_gHciReserved_0x2B_c:
                    shell_write(gHciReserved_0x2B_c);
                    break;

                case GAPScanningEventCommandFailedIndication_FailReason_gHciQosNotAcceptableParameter_c:
                    shell_write(gHciQosNotAcceptableParameter_c);
                    break;

                case GAPScanningEventCommandFailedIndication_FailReason_gHciQosRejected_c:
                    shell_write(gHciQosRejected_c);
                    break;

                case GAPScanningEventCommandFailedIndication_FailReason_gHciChannelClassificationNotSupported_c:
                    shell_write(gHciChannelClassificationNotSupported_c);
                    break;

                case GAPScanningEventCommandFailedIndication_FailReason_gHciInsufficientSecurity_c:
                    shell_write(gHciInsufficientSecurity_c);
                    break;

                case GAPScanningEventCommandFailedIndication_FailReason_gHciParameterOutOfMandatoryRange_c:
                    shell_write(gHciParameterOutOfMandatoryRange_c);
                    break;

                case GAPScanningEventCommandFailedIndication_FailReason_gHciReserved_0x31_c:
                    shell_write(gHciReserved_0x31_c);
                    break;

                case GAPScanningEventCommandFailedIndication_FailReason_gHciRoleSwitchPending_c:
                    shell_write(gHciRoleSwitchPending_c);
                    break;

                case GAPScanningEventCommandFailedIndication_FailReason_gHciReserved_0x33_c:
                    shell_write(gHciReserved_0x33_c);
                    break;

                case GAPScanningEventCommandFailedIndication_FailReason_gHciReservedSlotViolation_c:
                    shell_write(gHciReservedSlotViolation_c);
                    break;

                case GAPScanningEventCommandFailedIndication_FailReason_gHciRoleSwitchFailed_c:
                    shell_write(gHciRoleSwitchFailed_c);
                    break;

                case GAPScanningEventCommandFailedIndication_FailReason_gHciExtendedInquiryResponseTooLarge_c:
                    shell_write(gHciExtendedInquiryResponseTooLarge_c);
                    break;

                case GAPScanningEventCommandFailedIndication_FailReason_gHciSecureSimplePairingNotSupportedByHost_c:
                    shell_write(gHciSecureSimplePairingNotSupportedByHost_c);
                    break;

                case GAPScanningEventCommandFailedIndication_FailReason_gHciHostBusyPairing_c:
                    shell_write(gHciHostBusyPairing_c);
                    break;

                case GAPScanningEventCommandFailedIndication_FailReason_gHciConnectionRejectedDueToNoSuitableChannelFound_c:
                    shell_write(gHciConnectionRejectedDueToNoSuitableChannelFound_c);
                    break;

                case GAPScanningEventCommandFailedIndication_FailReason_gHciControllerBusy_c:
                    shell_write(gHciControllerBusy_c);
                    break;

                case GAPScanningEventCommandFailedIndication_FailReason_gHciUnacceptableConnectionParameters_c:
                    shell_write(gHciUnacceptableConnectionParameters_c);
                    break;

                case GAPScanningEventCommandFailedIndication_FailReason_gHciDirectedAdvertisingTimeout_c:
                    shell_write(gHciDirectedAdvertisingTimeout_c);
                    break;

                case GAPScanningEventCommandFailedIndication_FailReason_gHciConnectionTerminatedDueToMicFailure_c:
                    shell_write(gHciConnectionTerminatedDueToMicFailure_c);
                    break;

                case GAPScanningEventCommandFailedIndication_FailReason_gHciConnectionFailedToBeEstablished_c:
                    shell_write(gHciConnectionFailedToBeEstablished_c);
                    break;

                case GAPScanningEventCommandFailedIndication_FailReason_gHciMacConnectionFailed_c:
                    shell_write(gHciMacConnectionFailed_c);
                    break;

                case GAPScanningEventCommandFailedIndication_FailReason_gHciCoarseClockAdjustmentRejected_c:
                    shell_write(gHciCoarseClockAdjustmentRejected_c);
                    break;

                case GAPScanningEventCommandFailedIndication_FailReason_gHciAlreadyInit_c:
                    shell_write(gHciAlreadyInit_c);
                    break;

                case GAPScanningEventCommandFailedIndication_FailReason_gHciInvalidParameter_c:
                    shell_write(gHciInvalidParameter_c);
                    break;

                case GAPScanningEventCommandFailedIndication_FailReason_gHciCallbackNotInstalled_c:
                    shell_write(gHciCallbackNotInstalled_c);
                    break;

                case GAPScanningEventCommandFailedIndication_FailReason_gHciCallbackAlreadyInstalled_c:
                    shell_write(gHciCallbackAlreadyInstalled_c);
                    break;

                case GAPScanningEventCommandFailedIndication_FailReason_gHciCommandNotSupported_c:
                    shell_write(gHciCommandNotSupported_c);
                    break;

                case GAPScanningEventCommandFailedIndication_FailReason_gHciEventNotSupported_c:
                    shell_write(gHciEventNotSupported_c);
                    break;

                case GAPScanningEventCommandFailedIndication_FailReason_gHciTransportError_c:
                    shell_write(gHciTransportError_c);
                    break;

                case GAPScanningEventCommandFailedIndication_FailReason_gL2caAlreadyInit_c:
                    shell_write(gL2caAlreadyInit_c);
                    break;

                case GAPScanningEventCommandFailedIndication_FailReason_gL2caInsufficientResources_c:
                    shell_write(gL2caInsufficientResources_c);
                    break;

                case GAPScanningEventCommandFailedIndication_FailReason_gL2caCallbackNotInstalled_c:
                    shell_write(gL2caCallbackNotInstalled_c);
                    break;

                case GAPScanningEventCommandFailedIndication_FailReason_gL2caCallbackAlreadyInstalled_c:
                    shell_write(gL2caCallbackAlreadyInstalled_c);
                    break;

                case GAPScanningEventCommandFailedIndication_FailReason_gL2caLePsmInvalid_c:
                    shell_write(gL2caLePsmInvalid_c);
                    break;

                case GAPScanningEventCommandFailedIndication_FailReason_gL2caLePsmAlreadyRegistered_c:
                    shell_write(gL2caLePsmAlreadyRegistered_c);
                    break;

                case GAPScanningEventCommandFailedIndication_FailReason_gL2caLePsmNotRegistered_c:
                    shell_write(gL2caLePsmNotRegistered_c);
                    break;

                case GAPScanningEventCommandFailedIndication_FailReason_gL2caLePsmInsufficientResources_c:
                    shell_write(gL2caLePsmInsufficientResources_c);
                    break;

                case GAPScanningEventCommandFailedIndication_FailReason_gL2caChannelInvalid_c:
                    shell_write(gL2caChannelInvalid_c);
                    break;

                case GAPScanningEventCommandFailedIndication_FailReason_gL2caChannelClosed_c:
                    shell_write(gL2caChannelClosed_c);
                    break;

                case GAPScanningEventCommandFailedIndication_FailReason_gL2caChannelAlreadyConnected_c:
                    shell_write(gL2caChannelAlreadyConnected_c);
                    break;

                case GAPScanningEventCommandFailedIndication_FailReason_gL2caConnectionParametersRejected_c:
                    shell_write(gL2caConnectionParametersRejected_c);
                    break;

                case GAPScanningEventCommandFailedIndication_FailReason_gL2caChannelBusy_c:
                    shell_write(gL2caChannelBusy_c);
                    break;

                case GAPScanningEventCommandFailedIndication_FailReason_gL2caInvalidParameter_c:
                    shell_write(gL2caInvalidParameter_c);
                    break;

                case GAPScanningEventCommandFailedIndication_FailReason_gL2caError_c:
                    shell_write(gL2caError_c);
                    break;

                case GAPScanningEventCommandFailedIndication_FailReason_gSmNullCBFunction_c:
                    shell_write(gSmNullCBFunction_c);
                    break;

                case GAPScanningEventCommandFailedIndication_FailReason_gSmCommandNotSupported_c:
                    shell_write(gSmCommandNotSupported_c);
                    break;

                case GAPScanningEventCommandFailedIndication_FailReason_gSmUnexpectedCommand_c:
                    shell_write(gSmUnexpectedCommand_c);
                    break;

                case GAPScanningEventCommandFailedIndication_FailReason_gSmInvalidCommandCode_c:
                    shell_write(gSmInvalidCommandCode_c);
                    break;

                case GAPScanningEventCommandFailedIndication_FailReason_gSmInvalidCommandLength_c:
                    shell_write(gSmInvalidCommandLength_c);
                    break;

                case GAPScanningEventCommandFailedIndication_FailReason_gSmInvalidCommandParameter_c:
                    shell_write(gSmInvalidCommandParameter_c);
                    break;

                case GAPScanningEventCommandFailedIndication_FailReason_gSmInvalidDeviceId_c:
                    shell_write(gSmInvalidDeviceId_c);
                    break;

                case GAPScanningEventCommandFailedIndication_FailReason_gSmInvalidInternalOperation_c:
                    shell_write(gSmInvalidInternalOperation_c);
                    break;

                case GAPScanningEventCommandFailedIndication_FailReason_gSmInvalidConnectionHandle_c:
                    shell_write(gSmInvalidConnectionHandle_c);
                    break;

                case GAPScanningEventCommandFailedIndication_FailReason_gSmInproperKeyDistributionField_c:
                    shell_write(gSmInproperKeyDistributionField_c);
                    break;

                case GAPScanningEventCommandFailedIndication_FailReason_gSmUnexpectedKeyType_c:
                    shell_write(gSmUnexpectedKeyType_c);
                    break;

                case GAPScanningEventCommandFailedIndication_FailReason_gSmUnexpectedPairingTerminationReason_c:
                    shell_write(gSmUnexpectedPairingTerminationReason_c);
                    break;

                case GAPScanningEventCommandFailedIndication_FailReason_gSmUnexpectedKeyset_c:
                    shell_write(gSmUnexpectedKeyset_c);
                    break;

                case GAPScanningEventCommandFailedIndication_FailReason_gSmSmpTimeoutOccurred_c:
                    shell_write(gSmSmpTimeoutOccurred_c);
                    break;

                case GAPScanningEventCommandFailedIndication_FailReason_gSmUnknownSmpPacketType_c:
                    shell_write(gSmUnknownSmpPacketType_c);
                    break;

                case GAPScanningEventCommandFailedIndication_FailReason_gSmInvalidSmpPacketLength_c:
                    shell_write(gSmInvalidSmpPacketLength_c);
                    break;

                case GAPScanningEventCommandFailedIndication_FailReason_gSmInvalidSmpPacketParameter_c:
                    shell_write(gSmInvalidSmpPacketParameter_c);
                    break;

                case GAPScanningEventCommandFailedIndication_FailReason_gSmReceivedUnexpectedSmpPacket_c:
                    shell_write(gSmReceivedUnexpectedSmpPacket_c);
                    break;

                case GAPScanningEventCommandFailedIndication_FailReason_gSmReceivedSmpPacketFromUnknownDevice_c:
                    shell_write(gSmReceivedSmpPacketFromUnknownDevice_c);
                    break;

                case GAPScanningEventCommandFailedIndication_FailReason_gSmReceivedUnexpectedHciEvent_c:
                    shell_write(gSmReceivedUnexpectedHciEvent_c);
                    break;

                case GAPScanningEventCommandFailedIndication_FailReason_gSmReceivedHciEventFromUnknownDevice_c:
                    shell_write(gSmReceivedHciEventFromUnknownDevice_c);
                    break;

                case GAPScanningEventCommandFailedIndication_FailReason_gSmInvalidHciEventParameter_c:
                    shell_write(gSmInvalidHciEventParameter_c);
                    break;

                case GAPScanningEventCommandFailedIndication_FailReason_gSmLlConnectionEncryptionInProgress_c:
                    shell_write(gSmLlConnectionEncryptionInProgress_c);
                    break;

                case GAPScanningEventCommandFailedIndication_FailReason_gSmLlConnectionEncryptionFailure_c:
                    shell_write(gSmLlConnectionEncryptionFailure_c);
                    break;

                case GAPScanningEventCommandFailedIndication_FailReason_gSmInsufficientResources_c:
                    shell_write(gSmInsufficientResources_c);
                    break;

                case GAPScanningEventCommandFailedIndication_FailReason_gSmOobDataAddressMismatch_c:
                    shell_write(gSmOobDataAddressMismatch_c);
                    break;

//                case GAPScanningEventCommandFailedIndication_FailReason_gSmSmpPacketReceivedAfterTimeoutOccurred_c:
//                    shell_write(gSmSmpPacketReceivedAfterTimeoutOccurred_c);
//                    break;

                case GAPScanningEventCommandFailedIndication_FailReason_gSmPairingErrorPasskeyEntryFailed_c:
                    shell_write(gSmPairingErrorPasskeyEntryFailed_c);
                    break;

                case GAPScanningEventCommandFailedIndication_FailReason_gSmPairingErrorConfirmValueFailed_c:
                    shell_write(gSmPairingErrorConfirmValueFailed_c);
                    break;

                case GAPScanningEventCommandFailedIndication_FailReason_gSmPairingErrorCommandNotSupported_c:
                    shell_write(gSmPairingErrorCommandNotSupported_c);
                    break;

                case GAPScanningEventCommandFailedIndication_FailReason_gSmPairingErrorInvalidParameters_c:
                    shell_write(gSmPairingErrorInvalidParameters_c);
                    break;

                case GAPScanningEventCommandFailedIndication_FailReason_gSmPairingErrorUnknownReason_c:
                    shell_write(gSmPairingErrorUnknownReason_c);
                    break;

                case GAPScanningEventCommandFailedIndication_FailReason_gSmTbResolvableAddressDoesNotMatchIrk_c:
                    shell_write(gSmTbResolvableAddressDoesNotMatchIrk_c);
                    break;

                case GAPScanningEventCommandFailedIndication_FailReason_gSmTbInvalidDataSignature_c:
                    shell_write(gSmTbInvalidDataSignature_c);
                    break;

                case GAPScanningEventCommandFailedIndication_FailReason_gAttInvalidHandle_c:
                    shell_write(gAttInvalidHandle_c);
                    break;

                case GAPScanningEventCommandFailedIndication_FailReason_gAttReadNotPermitted_c:
                    shell_write(gAttReadNotPermitted_c);
                    break;

                case GAPScanningEventCommandFailedIndication_FailReason_gAttWriteNotPermitted_c:
                    shell_write(gAttWriteNotPermitted_c);
                    break;

                case GAPScanningEventCommandFailedIndication_FailReason_gAttInvalidPdu_c:
                    shell_write(gAttInvalidPdu_c);
                    break;

                case GAPScanningEventCommandFailedIndication_FailReason_gAttInsufficientAuthentication_c:
                    shell_write(gAttInsufficientAuthentication_c);
                    break;

                case GAPScanningEventCommandFailedIndication_FailReason_gAttRequestNotSupported_c:
                    shell_write(gAttRequestNotSupported_c);
                    break;

                case GAPScanningEventCommandFailedIndication_FailReason_gAttInvalidOffset_c:
                    shell_write(gAttInvalidOffset_c);
                    break;

                case GAPScanningEventCommandFailedIndication_FailReason_gAttInsufficientAuthorization_c:
                    shell_write(gAttInsufficientAuthorization_c);
                    break;

                case GAPScanningEventCommandFailedIndication_FailReason_gAttPrepareQueueFull_c:
                    shell_write(gAttPrepareQueueFull_c);
                    break;

                case GAPScanningEventCommandFailedIndication_FailReason_gAttAttributeNotFound_c:
                    shell_write(gAttAttributeNotFound_c);
                    break;

                case GAPScanningEventCommandFailedIndication_FailReason_gAttAttributeNotLong_c:
                    shell_write(gAttAttributeNotLong_c);
                    break;

                case GAPScanningEventCommandFailedIndication_FailReason_gAttInsufficientEncryptionKeySize_c:
                    shell_write(gAttInsufficientEncryptionKeySize_c);
                    break;

                case GAPScanningEventCommandFailedIndication_FailReason_gAttInvalidAttributeValueLength_c:
                    shell_write(gAttInvalidAttributeValueLength_c);
                    break;

                case GAPScanningEventCommandFailedIndication_FailReason_gAttUnlikelyor_c:
                    shell_write(gAttUnlikelyor_c);
                    break;

                case GAPScanningEventCommandFailedIndication_FailReason_gAttInsufficientEncryption_c:
                    shell_write(gAttInsufficientEncryption_c);
                    break;

                case GAPScanningEventCommandFailedIndication_FailReason_gAttUnsupportedGroupType_c:
                    shell_write(gAttUnsupportedGroupType_c);
                    break;

                case GAPScanningEventCommandFailedIndication_FailReason_gAttInsufficientResources_c:
                    shell_write(gAttInsufficientResources_c);
                    break;

                case GAPScanningEventCommandFailedIndication_FailReason_gGattAnotherProcedureInProgress_c:
                    shell_write(gGattAnotherProcedureInProgress_c);
                    break;

                case GAPScanningEventCommandFailedIndication_FailReason_gGattLongAttributePacketsCorrupted_c:
                    shell_write(gGattLongAttributePacketsCorrupted_c);
                    break;

                case GAPScanningEventCommandFailedIndication_FailReason_gGattMultipleAttributesOverflow_c:
                    shell_write(gGattMultipleAttributesOverflow_c);
                    break;

                case GAPScanningEventCommandFailedIndication_FailReason_gGattUnexpectedReadMultipleResponseLength_c:
                    shell_write(gGattUnexpectedReadMultipleResponseLength_c);
                    break;

                case GAPScanningEventCommandFailedIndication_FailReason_gGattInvalidValueLength_c:
                    shell_write(gGattInvalidValueLength_c);
                    break;

                case GAPScanningEventCommandFailedIndication_FailReason_gGattServerTimeout_c:
                    shell_write(gGattServerTimeout_c);
                    break;

                case GAPScanningEventCommandFailedIndication_FailReason_gGattIndicationAlreadyInProgress_c:
                    shell_write(gGattIndicationAlreadyInProgress_c);
                    break;

                case GAPScanningEventCommandFailedIndication_FailReason_gGattClientConfirmationTimeout_c:
                    shell_write(gGattClientConfirmationTimeout_c);
                    break;

                case GAPScanningEventCommandFailedIndication_FailReason_gGapAdvDataTooLong_c:
                    shell_write(gGapAdvDataTooLong_c);
                    break;

                case GAPScanningEventCommandFailedIndication_FailReason_gGapScanRspDataTooLong_c:
                    shell_write(gGapScanRspDataTooLong_c);
                    break;

                case GAPScanningEventCommandFailedIndication_FailReason_gGapDeviceNotBonded_c:
                    shell_write(gGapDeviceNotBonded_c);
                    break;

                case GAPScanningEventCommandFailedIndication_FailReason_gDevDbCccdLimitReached_c:
                    shell_write(gDevDbCccdLimitReached_c);
                    break;

                case GAPScanningEventCommandFailedIndication_FailReason_gDevDbCccdNotFound_c:
                    shell_write(gDevDbCccdNotFound_c);
                    break;

                case GAPScanningEventCommandFailedIndication_FailReason_gGattDbInvalidHandle_c:
                    shell_write(gGattDbInvalidHandle_c);
                    break;

                case GAPScanningEventCommandFailedIndication_FailReason_gGattDbCharacteristicNotFound_c:
                    shell_write(gGattDbCharacteristicNotFound_c);
                    break;

                case GAPScanningEventCommandFailedIndication_FailReason_gGattDbCccdNotFound_c:
                    shell_write(gGattDbCccdNotFound_c);
                    break;

                case GAPScanningEventCommandFailedIndication_FailReason_gGattDbServiceNotFound_c:
                    shell_write(gGattDbServiceNotFound_c);
                    break;

                case GAPScanningEventCommandFailedIndication_FailReason_gGattDbDescriptorNotFound_c:
                    shell_write(gGattDbDescriptorNotFound_c);
                    break;

                default:
                    shell_printf("Unrecognized status 0x%04X", container->Data.GAPScanningEventCommandFailedIndication.FailReason);
                    break;
            }

            break;

        case 0x479C:
            shell_write("\r\nGAPScanningEventDeviceScannedIndication");
            shell_write(" -> ");

            switch (container->Data.GAPScanningEventDeviceScannedIndication.AddressType)
            {
                case GAPScanningEventDeviceScannedIndication_AddressType_gPublic_c:
                    shell_write(gPublic_c);
                    break;

                case GAPScanningEventDeviceScannedIndication_AddressType_gRandom_c:
                    shell_write(gRandom_c);
                    break;

                default:
                    shell_printf("Unrecognized status 0x%02X", container->Data.GAPScanningEventDeviceScannedIndication.AddressType);
                    break;
            }

            shell_write(" -> ");

            switch (container->Data.GAPScanningEventDeviceScannedIndication.AdvEventType)
            {
                case GAPScanningEventDeviceScannedIndication_AdvEventType_gBleAdvRepAdvInd_c:
                    shell_write(gBleAdvRepAdvInd_c);
                    break;

                case GAPScanningEventDeviceScannedIndication_AdvEventType_gBleAdvRepAdvDirectInd_c:
                    shell_write(gBleAdvRepAdvDirectInd_c);
                    break;

                case GAPScanningEventDeviceScannedIndication_AdvEventType_gBleAdvRepAdvScanInd_c:
                    shell_write(gBleAdvRepAdvScanInd_c);
                    break;

                case GAPScanningEventDeviceScannedIndication_AdvEventType_gBleAdvRepAdvNonconnInd_c:
                    shell_write(gBleAdvRepAdvNonconnInd_c);
                    break;

                case GAPScanningEventDeviceScannedIndication_AdvEventType_gBleAdvRepScanRsp_c:
                    shell_write(gBleAdvRepScanRsp_c);
                    break;

                default:
                    shell_printf("Unrecognized status 0x%02X", container->Data.GAPScanningEventDeviceScannedIndication.AdvEventType);
                    break;
            }

            SHELL_GapPrintDeviceScannedIndication(&(container->Data.GAPScanningEventDeviceScannedIndication));

            break;

        case 0x479D:
            shell_write("\r\nGAPConnectionEventConnectedIndication");
            shell_write(" -> ");

            /* Transmit FSCI Message to set BLE Status LED to indicate Connected  */
        	FSCI_transmitPayload(0xb9, 0x03, NULL, 0, BLE_FSCI_IF);

            switch (container->Data.GAPConnectionEventConnectedIndication.PeerAddressType)
            {
                case GAPConnectionEventConnectedIndication_PeerAddressType_gPublic_c:
                    shell_write(gPublic_c);
                    mPeerDeviceAddressType = GAPAddDeviceToWhiteListRequest_AddressType_gPublic_c;
                    break;

                case GAPConnectionEventConnectedIndication_PeerAddressType_gRandom_c:
                    mPeerDeviceAddressType = GAPAddDeviceToWhiteListRequest_AddressType_gRandom_c;
                    shell_write(gRandom_c);
                    break;

                default:
                    shell_printf("Unrecognized status 0x%02X", container->Data.GAPConnectionEventConnectedIndication.PeerAddressType);
                    break;
            }

            FLib_MemCpy(maPeerDeviceAddress, container->Data.GAPConnectionEventConnectedIndication.PeerAddress, 6);

            gPeerDeviceId = container->Data.GAPConnectionEventConnectedIndication.DeviceId;

            if (gRpkDemoState == gDemoWaitingConnection_c || gRpkDemoState == gDemoPrintMessage_c )
            {
                gRpkDemoState = gDemoConnectionEstablish_c;
                mSuppressBleEventPrint = TRUE;
                BleApp_DemoRpkNotify();
                OtapCS_Subscribe(container->Data.GAPConnectionEventConnectedIndication.DeviceId);
//                OtapClient_HandleConnectionEvent(container->Data.GAPConnectionEventConnectedIndication.DeviceId);

                GAPCheckIfBondedRequest_t req_bonded = {gPeerDeviceId};
                GAPCheckIfBondedRequest (&req_bonded, BLE_FSCI_IF);
            }

            break;

        case 0x479E:
            shell_write("\r\nGAPConnectionEventPairingRequestIndication");

            // SHELL_GapAcceptPairingRequest(&(container->Data.GAPConnectionEventPairingRequestIndication));
            if (mDelayTimerID == gTmrInvalidTimerID_c)
            {
                mDelayTimerID = TMR_AllocateTimer();
            }

            gPairingParameters.PairingParameters.CentralKeys = container->Data.GAPConnectionEventPairingRequestIndication.PairingParameters.CentralKeys;
            TMR_StartSingleShotTimer(mDelayTimerID, 100, SHELL_GapAcceptPairingRequest, (void *)(uint32_t)container->Data.GAPConnectionEventPairingRequestIndication.DeviceId);
            break;

        case 0x479F:
            shell_write("\r\nGAPConnectionEventSlaveSecurityRequestIndication");
            break;

        case 0x47A0:
            shell_write("\r\nGAPConnectionEventPairingResponseIndication");
            break;

        case 0x47A1:
            shell_write("\r\nGAPConnectionEventAuthenticationRejectedIndication");
            shell_write(" -> ");

            switch (container->Data.GAPConnectionEventAuthenticationRejectedIndication.RejectReason)
            {
                case GAPConnectionEventAuthenticationRejectedIndication_RejectReason_gOobNotAvailable_c:
                    shell_write(gOobNotAvailable_c);
                    break;

                case GAPConnectionEventAuthenticationRejectedIndication_RejectReason_gIncompatibleIoCapabilities_c:
                    shell_write(gIncompatibleIoCapabilities_c);
                    break;

                case GAPConnectionEventAuthenticationRejectedIndication_RejectReason_gPairingNotSupported_c:
                    shell_write(gPairingNotSupported_c);
                    break;

                case GAPConnectionEventAuthenticationRejectedIndication_RejectReason_gLowEncryptionKeySize_c:
                    shell_write(gLowEncryptionKeySize_c);
                    break;

                case GAPConnectionEventAuthenticationRejectedIndication_RejectReason_gUnspecifiedReason_c:
                    shell_write(gUnspecifiedReason_c);
                    break;

                case GAPConnectionEventAuthenticationRejectedIndication_RejectReason_gRepeatedAttempts_c:
                    shell_write(gRepeatedAttempts_c);
                    break;

                case GAPConnectionEventAuthenticationRejectedIndication_RejectReason_gLinkEncryptionFailed_c:
                    shell_write(gLinkEncryptionFailed_c);
                    break;

                default:
                    shell_printf("Unrecognized status 0x%02X", container->Data.GAPConnectionEventAuthenticationRejectedIndication.RejectReason);
                    break;
            }

            break;

        case 0x47A2:
            shell_write("\r\nGAPConnectionEventPasskeyRequestIndication");
            break;

        case 0x47A3:
            shell_write("\r\nGAPConnectionEventOobRequestIndication");
            break;

        case 0x47A4:
        	shell_write("\r\nGAPConnectionEventPasskeyDisplayIndication");
            BleApp_DisplayCode(container->Data.GAPConnectionEventPasskeyDisplayIndication.PasskeyForDisplay);
            break;

        case 0x47A5:
            shell_write("\r\nGAPConnectionEventKeyExchangeRequestIndication");
            shell_write(" -> ");

            BleApp_SentKey(&container->Data.GAPConnectionEventKeyExchangeRequestIndication);
            switch (container->Data.GAPConnectionEventKeyExchangeRequestIndication.RequestedKeys)
            {
                case GAPConnectionEventKeyExchangeRequestIndication_RequestedKeys_gNoKeys_c:
                    shell_write(gNoKeys_c);
                    break;

                case GAPConnectionEventKeyExchangeRequestIndication_RequestedKeys_gLtk_c:
                    shell_write(gLtk_c);
                    break;

                case GAPConnectionEventKeyExchangeRequestIndication_RequestedKeys_gIrk_c:
                    shell_write(gIrk_c);
                    break;

                case GAPConnectionEventKeyExchangeRequestIndication_RequestedKeys_gCsrk_c:
                    shell_write(gCsrk_c);
                    break;

                default:
                    shell_printf("Unrecognized status 0x%02X", container->Data.GAPConnectionEventKeyExchangeRequestIndication.RequestedKeys);
                    break;
            }

            break;

        case 0x47A6:
            shell_write("\r\nGAPConnectionEventKeysReceivedIndication");
            break;

        case 0x47A7:
            shell_write("\r\nGAPConnectionEventLongTermKeyRequestIndication");
            BleApp_ProvideLongTermKey(&container->Data.GAPConnectionEventLongTermKeyRequestIndication);
            break;

        case 0x47A8:
            shell_write("\r\nGAPConnectionEventEncryptionChangedIndication");
            break;

        case 0x47A9:
            shell_write("\r\nGAPConnectionEventPairingCompleteIndication");
            shell_write(" -> ");

            switch (container->Data.GAPConnectionEventPairingCompleteIndication.PairingStatus)
            {
                case GAPConnectionEventPairingCompleteIndication_PairingStatus_PairingSuccessful:
                    shell_write(gPairingSuccessful_c);

                    GUI_DispStringAt("Pairing Successful!        ", 10, 18);
                    GAPAddDeviceToWhiteListRequest_t req;
                    req.AddressType = mPeerDeviceAddressType;
                    FLib_MemCpy(req.Address, maPeerDeviceAddress, 6);
                    GAPAddDeviceToWhiteListRequest(&req, BLE_FSCI_IF);

                    OtapCS_Subscribe(gPeerDeviceId);
                    BleApp_CccdWritten_lazy();

                    if (mDelayTimerID == gTmrInvalidTimerID_c)
                    {
                        mDelayTimerID = TMR_AllocateTimer();
                    }

                    TMR_StartSingleShotTimer(mDelayTimerID, (tmrTimeInMilliseconds_t)300, (pfTmrCallBack_t)OtapClient_HandleConnectionEvent, (void *)((uint32_t)gPeerDeviceId));

//                    OtapClient_HandleConnectionEvent(gPeerDeviceId);

                    // TODO privacy
//    #if gAppUsePrivacy_d
//                    BleConnManager_DisablePrivacy();
//                    BleConnManager_EnablePrivacyInternal(TRUE);
//    #endif
                    break;

                case GAPConnectionEventPairingCompleteIndication_PairingStatus_PairingFailed:
                    shell_write(gPairingFailed_c);
                    break;

                default:
                    shell_printf("Unrecognized status 0x%02X", container->Data.GAPConnectionEventPairingCompleteIndication.PairingStatus);
                    break;
            }

            break;

        case 0x47AA:
            shell_write("\r\nGAPConnectionEventDisconnectedIndication");
            shell_write(" -> ");

            OtapClient_HandleDisconnectionEvent (container->Data.GAPConnectionEventDisconnectedIndication.DeviceId);

            /* Transmit FSCI Message to set BLE Status LED to indicate Disconnected/BLE OFF  */
        	FSCI_transmitPayload(0xb9, 0x00, NULL, 0, BLE_FSCI_IF);

            switch (container->Data.GAPConnectionEventDisconnectedIndication.Reason)
            {
                case GAPConnectionEventDisconnectedIndication_Reason_gBleSuccess_c:
                    shell_write(gBleSuccess_c);
                    break;

                case GAPConnectionEventDisconnectedIndication_Reason_gBleInvalidParameter_c:
                    shell_write(gBleInvalidParameter_c);
                    break;

                case GAPConnectionEventDisconnectedIndication_Reason_gBleOverflow_c:
                    shell_write(gBleOverflow_c);
                    break;

                case GAPConnectionEventDisconnectedIndication_Reason_gBleUnavailable_c:
                    shell_write(gBleUnavailable_c);
                    break;

                case GAPConnectionEventDisconnectedIndication_Reason_gBleFeatureNotSupported_c:
                    shell_write(gBleFeatureNotSupported_c);
                    break;

                case GAPConnectionEventDisconnectedIndication_Reason_gBleOutOfMemory_c:
                    shell_write(gBleOutOfMemory_c);
                    break;

                case GAPConnectionEventDisconnectedIndication_Reason_gBleAlreadyInitialized_c:
                    shell_write(gBleAlreadyInitialized_c);
                    break;

                case GAPConnectionEventDisconnectedIndication_Reason_gBleOsError_c:
                    shell_write(gBleOsError_c);
                    break;

                case GAPConnectionEventDisconnectedIndication_Reason_gBleUnexpectedError_c:
                    shell_write(gBleUnexpectedError_c);
                    break;

                case GAPConnectionEventDisconnectedIndication_Reason_gBleInvalidState_c:
                    shell_write(gBleInvalidState_c);
                    break;

                case GAPConnectionEventDisconnectedIndication_Reason_gHciUnknownHciCommand_c:
                    shell_write(gHciUnknownHciCommand_c);
                    break;

                case GAPConnectionEventDisconnectedIndication_Reason_gHciUnknownConnectionIdentifier_c:
                    shell_write(gHciUnknownConnectionIdentifier_c);
                    break;

                case GAPConnectionEventDisconnectedIndication_Reason_gHciHardwareFailure_c:
                    shell_write(gHciHardwareFailure_c);
                    break;

                case GAPConnectionEventDisconnectedIndication_Reason_gHciPageTimeout_c:
                    shell_write(gHciPageTimeout_c);
                    break;

                case GAPConnectionEventDisconnectedIndication_Reason_gHciAuthenticationFailure_c:
                    shell_write(gHciAuthenticationFailure_c);
                    break;

                case GAPConnectionEventDisconnectedIndication_Reason_gHciPinOrKeyMissing_c:
                    shell_write(gHciPinOrKeyMissing_c);
                    break;

                case GAPConnectionEventDisconnectedIndication_Reason_gHciMemoryCapacityExceeded_c:
                    shell_write(gHciMemoryCapacityExceeded_c);
                    break;

                case GAPConnectionEventDisconnectedIndication_Reason_gHciConnectionTimeout_c:
                    shell_write(gHciConnectionTimeout_c);
                    break;

                case GAPConnectionEventDisconnectedIndication_Reason_gHciConnectionLimitExceeded_c:
                    shell_write(gHciConnectionLimitExceeded_c);
                    break;

                case GAPConnectionEventDisconnectedIndication_Reason_gHciSynchronousConnectionLimitToADeviceExceeded_c:
                    shell_write(gHciSynchronousConnectionLimitToADeviceExceeded_c);
                    break;

                case GAPConnectionEventDisconnectedIndication_Reason_gHciAclConnectionAlreadyExists_c:
                    shell_write(gHciAclConnectionAlreadyExists_c);
                    break;

                case GAPConnectionEventDisconnectedIndication_Reason_gHciCommandDisallowed_c:
                    shell_write(gHciCommandDisallowed_c);
                    break;

                case GAPConnectionEventDisconnectedIndication_Reason_gHciConnectionRejectedDueToLimitedResources_c:
                    shell_write(gHciConnectionRejectedDueToLimitedResources_c);
                    break;

                case GAPConnectionEventDisconnectedIndication_Reason_gHciConnectionRejectedDueToSecurityReasons_c:
                    shell_write(gHciConnectionRejectedDueToSecurityReasons_c);
                    break;

                case GAPConnectionEventDisconnectedIndication_Reason_gHciConnectionRejectedDueToUnacceptableBdAddr_c:
                    shell_write(gHciConnectionRejectedDueToUnacceptableBdAddr_c);
                    break;

                case GAPConnectionEventDisconnectedIndication_Reason_gHciConnectionAcceptTimeoutExceeded_c:
                    shell_write(gHciConnectionAcceptTimeoutExceeded_c);
                    break;

                case GAPConnectionEventDisconnectedIndication_Reason_gHciUnsupportedFeatureOrParameterValue_c:
                    shell_write(gHciUnsupportedFeatureOrParameterValue_c);
                    break;

                case GAPConnectionEventDisconnectedIndication_Reason_gHciInvalidHciCommandParameters_c:
                    shell_write(gHciInvalidHciCommandParameters_c);
                    break;

                case GAPConnectionEventDisconnectedIndication_Reason_gHciRemoteUserTerminatedConnection_c:
                    shell_write(gHciRemoteUserTerminatedConnection_c);
                    break;

                case GAPConnectionEventDisconnectedIndication_Reason_gHciRemoteDeviceTerminatedConnectionLowResources_c:
                    shell_write(gHciRemoteDeviceTerminatedConnectionLowResources_c);
                    break;

                case GAPConnectionEventDisconnectedIndication_Reason_gHciRemoteDeviceTerminatedConnectionPowerOff_c:
                    shell_write(gHciRemoteDeviceTerminatedConnectionPowerOff_c);
                    break;

                case GAPConnectionEventDisconnectedIndication_Reason_gHciConnectionTerminatedByLocalHost_c:
                    shell_write(gHciConnectionTerminatedByLocalHost_c);
                    break;

                case GAPConnectionEventDisconnectedIndication_Reason_gHciRepeatedAttempts_c:
                    shell_write(gHciRepeatedAttempts_c);
                    break;

                case GAPConnectionEventDisconnectedIndication_Reason_gHciPairingNotAllowed_c:
                    shell_write(gHciPairingNotAllowed_c);
                    break;

                case GAPConnectionEventDisconnectedIndication_Reason_gHciUnknownLpmPdu_c:
                    shell_write(gHciUnknownLpmPdu_c);
                    break;

                case GAPConnectionEventDisconnectedIndication_Reason_gHciUnsupportedRemoteFeature_c:
                    shell_write(gHciUnsupportedRemoteFeature_c);
                    break;

                case GAPConnectionEventDisconnectedIndication_Reason_gHciScoOffsetRejected_c:
                    shell_write(gHciScoOffsetRejected_c);
                    break;

                case GAPConnectionEventDisconnectedIndication_Reason_gHciScoIntervalRejected_c:
                    shell_write(gHciScoIntervalRejected_c);
                    break;

                case GAPConnectionEventDisconnectedIndication_Reason_gHciScoAirModeRejected_c:
                    shell_write(gHciScoAirModeRejected_c);
                    break;

                case GAPConnectionEventDisconnectedIndication_Reason_gHciInvalidLpmParameters_c:
                    shell_write(gHciInvalidLpmParameters_c);
                    break;

                case GAPConnectionEventDisconnectedIndication_Reason_gHciUnspecifiedError_c:
                    shell_write(gHciUnspecifiedError_c);
                    break;

                case GAPConnectionEventDisconnectedIndication_Reason_gHciUnsupportedLpmParameterValue_c:
                    shell_write(gHciUnsupportedLpmParameterValue_c);
                    break;

                case GAPConnectionEventDisconnectedIndication_Reason_gHciRoleChangeNotAllowed_c:
                    shell_write(gHciRoleChangeNotAllowed_c);
                    break;

                case GAPConnectionEventDisconnectedIndication_Reason_gHciLLResponseTimeout_c:
                    shell_write(gHciLLResponseTimeout_c);
                    break;

                case GAPConnectionEventDisconnectedIndication_Reason_gHciLmpErrorTransactionCollision_c:
                    shell_write(gHciLmpErrorTransactionCollision_c);
                    break;

                case GAPConnectionEventDisconnectedIndication_Reason_gHciLmpPduNotAllowed_c:
                    shell_write(gHciLmpPduNotAllowed_c);
                    break;

                case GAPConnectionEventDisconnectedIndication_Reason_gHciEncryptionModeNotAcceptable_c:
                    shell_write(gHciEncryptionModeNotAcceptable_c);
                    break;

                case GAPConnectionEventDisconnectedIndication_Reason_gHciLinkKeyCannotBeChanged_c:
                    shell_write(gHciLinkKeyCannotBeChanged_c);
                    break;

                case GAPConnectionEventDisconnectedIndication_Reason_gHciRequestedQosNotSupported_c:
                    shell_write(gHciRequestedQosNotSupported_c);
                    break;

                case GAPConnectionEventDisconnectedIndication_Reason_gHciInstantPassed_c:
                    shell_write(gHciInstantPassed_c);
                    break;

                case GAPConnectionEventDisconnectedIndication_Reason_gHciPairingWithUnitKeyNotSupported_c:
                    shell_write(gHciPairingWithUnitKeyNotSupported_c);
                    break;

                case GAPConnectionEventDisconnectedIndication_Reason_gHciDifferentTransactionCollision_c:
                    shell_write(gHciDifferentTransactionCollision_c);
                    break;

                case GAPConnectionEventDisconnectedIndication_Reason_gHciReserved_0x2B_c:
                    shell_write(gHciReserved_0x2B_c);
                    break;

                case GAPConnectionEventDisconnectedIndication_Reason_gHciQosNotAcceptableParameter_c:
                    shell_write(gHciQosNotAcceptableParameter_c);
                    break;

                case GAPConnectionEventDisconnectedIndication_Reason_gHciQosRejected_c:
                    shell_write(gHciQosRejected_c);
                    break;

                case GAPConnectionEventDisconnectedIndication_Reason_gHciChannelClassificationNotSupported_c:
                    shell_write(gHciChannelClassificationNotSupported_c);
                    break;

                case GAPConnectionEventDisconnectedIndication_Reason_gHciInsufficientSecurity_c:
                    shell_write(gHciInsufficientSecurity_c);
                    break;

                case GAPConnectionEventDisconnectedIndication_Reason_gHciParameterOutOfMandatoryRange_c:
                    shell_write(gHciParameterOutOfMandatoryRange_c);
                    break;

                case GAPConnectionEventDisconnectedIndication_Reason_gHciReserved_0x31_c:
                    shell_write(gHciReserved_0x31_c);
                    break;

                case GAPConnectionEventDisconnectedIndication_Reason_gHciRoleSwitchPending_c:
                    shell_write(gHciRoleSwitchPending_c);
                    break;

                case GAPConnectionEventDisconnectedIndication_Reason_gHciReserved_0x33_c:
                    shell_write(gHciReserved_0x33_c);
                    break;

                case GAPConnectionEventDisconnectedIndication_Reason_gHciReservedSlotViolation_c:
                    shell_write(gHciReservedSlotViolation_c);
                    break;

                case GAPConnectionEventDisconnectedIndication_Reason_gHciRoleSwitchFailed_c:
                    shell_write(gHciRoleSwitchFailed_c);
                    break;

                case GAPConnectionEventDisconnectedIndication_Reason_gHciExtendedInquiryResponseTooLarge_c:
                    shell_write(gHciExtendedInquiryResponseTooLarge_c);
                    break;

                case GAPConnectionEventDisconnectedIndication_Reason_gHciSecureSimplePairingNotSupportedByHost_c:
                    shell_write(gHciSecureSimplePairingNotSupportedByHost_c);
                    break;

                case GAPConnectionEventDisconnectedIndication_Reason_gHciHostBusyPairing_c:
                    shell_write(gHciHostBusyPairing_c);
                    break;

                case GAPConnectionEventDisconnectedIndication_Reason_gHciConnectionRejectedDueToNoSuitableChannelFound_c:
                    shell_write(gHciConnectionRejectedDueToNoSuitableChannelFound_c);
                    break;

                case GAPConnectionEventDisconnectedIndication_Reason_gHciControllerBusy_c:
                    shell_write(gHciControllerBusy_c);
                    break;

                case GAPConnectionEventDisconnectedIndication_Reason_gHciUnacceptableConnectionParameters_c:
                    shell_write(gHciUnacceptableConnectionParameters_c);
                    break;

                case GAPConnectionEventDisconnectedIndication_Reason_gHciDirectedAdvertisingTimeout_c:
                    shell_write(gHciDirectedAdvertisingTimeout_c);
                    break;

                case GAPConnectionEventDisconnectedIndication_Reason_gHciConnectionTerminatedDueToMicFailure_c:
                    shell_write(gHciConnectionTerminatedDueToMicFailure_c);
                    break;

                case GAPConnectionEventDisconnectedIndication_Reason_gHciConnectionFailedToBeEstablished_c:
                    shell_write(gHciConnectionFailedToBeEstablished_c);
                    break;

                case GAPConnectionEventDisconnectedIndication_Reason_gHciMacConnectionFailed_c:
                    shell_write(gHciMacConnectionFailed_c);
                    break;

                case GAPConnectionEventDisconnectedIndication_Reason_gHciCoarseClockAdjustmentRejected_c:
                    shell_write(gHciCoarseClockAdjustmentRejected_c);
                    break;

                case GAPConnectionEventDisconnectedIndication_Reason_gHciAlreadyInit_c:
                    shell_write(gHciAlreadyInit_c);
                    break;

                case GAPConnectionEventDisconnectedIndication_Reason_gHciInvalidParameter_c:
                    shell_write(gHciInvalidParameter_c);
                    break;

                case GAPConnectionEventDisconnectedIndication_Reason_gHciCallbackNotInstalled_c:
                    shell_write(gHciCallbackNotInstalled_c);
                    break;

                case GAPConnectionEventDisconnectedIndication_Reason_gHciCallbackAlreadyInstalled_c:
                    shell_write(gHciCallbackAlreadyInstalled_c);
                    break;

                case GAPConnectionEventDisconnectedIndication_Reason_gHciCommandNotSupported_c:
                    shell_write(gHciCommandNotSupported_c);
                    break;

                case GAPConnectionEventDisconnectedIndication_Reason_gHciEventNotSupported_c:
                    shell_write(gHciEventNotSupported_c);
                    break;

                case GAPConnectionEventDisconnectedIndication_Reason_gHciTransportError_c:
                    shell_write(gHciTransportError_c);
                    break;

                case GAPConnectionEventDisconnectedIndication_Reason_gL2caAlreadyInit_c:
                    shell_write(gL2caAlreadyInit_c);
                    break;

                case GAPConnectionEventDisconnectedIndication_Reason_gL2caInsufficientResources_c:
                    shell_write(gL2caInsufficientResources_c);
                    break;

                case GAPConnectionEventDisconnectedIndication_Reason_gL2caCallbackNotInstalled_c:
                    shell_write(gL2caCallbackNotInstalled_c);
                    break;

                case GAPConnectionEventDisconnectedIndication_Reason_gL2caCallbackAlreadyInstalled_c:
                    shell_write(gL2caCallbackAlreadyInstalled_c);
                    break;

                case GAPConnectionEventDisconnectedIndication_Reason_gL2caLePsmInvalid_c:
                    shell_write(gL2caLePsmInvalid_c);
                    break;

                case GAPConnectionEventDisconnectedIndication_Reason_gL2caLePsmAlreadyRegistered_c:
                    shell_write(gL2caLePsmAlreadyRegistered_c);
                    break;

                case GAPConnectionEventDisconnectedIndication_Reason_gL2caLePsmNotRegistered_c:
                    shell_write(gL2caLePsmNotRegistered_c);
                    break;

                case GAPConnectionEventDisconnectedIndication_Reason_gL2caLePsmInsufficientResources_c:
                    shell_write(gL2caLePsmInsufficientResources_c);
                    break;

                case GAPConnectionEventDisconnectedIndication_Reason_gL2caChannelInvalid_c:
                    shell_write(gL2caChannelInvalid_c);
                    break;

                case GAPConnectionEventDisconnectedIndication_Reason_gL2caChannelClosed_c:
                    shell_write(gL2caChannelClosed_c);
                    break;

                case GAPConnectionEventDisconnectedIndication_Reason_gL2caChannelAlreadyConnected_c:
                    shell_write(gL2caChannelAlreadyConnected_c);
                    break;

                case GAPConnectionEventDisconnectedIndication_Reason_gL2caConnectionParametersRejected_c:
                    shell_write(gL2caConnectionParametersRejected_c);
                    break;

                case GAPConnectionEventDisconnectedIndication_Reason_gL2caChannelBusy_c:
                    shell_write(gL2caChannelBusy_c);
                    break;

                case GAPConnectionEventDisconnectedIndication_Reason_gL2caInvalidParameter_c:
                    shell_write(gL2caInvalidParameter_c);
                    break;

                case GAPConnectionEventDisconnectedIndication_Reason_gL2caError_c:
                    shell_write(gL2caError_c);
                    break;

                case GAPConnectionEventDisconnectedIndication_Reason_gSmNullCBFunction_c:
                    shell_write(gSmNullCBFunction_c);
                    break;

                case GAPConnectionEventDisconnectedIndication_Reason_gSmCommandNotSupported_c:
                    shell_write(gSmCommandNotSupported_c);
                    break;

                case GAPConnectionEventDisconnectedIndication_Reason_gSmUnexpectedCommand_c:
                    shell_write(gSmUnexpectedCommand_c);
                    break;

                case GAPConnectionEventDisconnectedIndication_Reason_gSmInvalidCommandCode_c:
                    shell_write(gSmInvalidCommandCode_c);
                    break;

                case GAPConnectionEventDisconnectedIndication_Reason_gSmInvalidCommandLength_c:
                    shell_write(gSmInvalidCommandLength_c);
                    break;

                case GAPConnectionEventDisconnectedIndication_Reason_gSmInvalidCommandParameter_c:
                    shell_write(gSmInvalidCommandParameter_c);
                    break;

                case GAPConnectionEventDisconnectedIndication_Reason_gSmInvalidDeviceId_c:
                    shell_write(gSmInvalidDeviceId_c);
                    break;

                case GAPConnectionEventDisconnectedIndication_Reason_gSmInvalidInternalOperation_c:
                    shell_write(gSmInvalidInternalOperation_c);
                    break;

                case GAPConnectionEventDisconnectedIndication_Reason_gSmInvalidConnectionHandle_c:
                    shell_write(gSmInvalidConnectionHandle_c);
                    break;

                case GAPConnectionEventDisconnectedIndication_Reason_gSmInproperKeyDistributionField_c:
                    shell_write(gSmInproperKeyDistributionField_c);
                    break;

                case GAPConnectionEventDisconnectedIndication_Reason_gSmUnexpectedKeyType_c:
                    shell_write(gSmUnexpectedKeyType_c);
                    break;

                case GAPConnectionEventDisconnectedIndication_Reason_gSmUnexpectedPairingTerminationReason_c:
                    shell_write(gSmUnexpectedPairingTermination_c);
                    break;

                case GAPConnectionEventDisconnectedIndication_Reason_gSmUnexpectedKeyset_c:
                    shell_write(gSmUnexpectedKeyset_c);
                    break;

                case GAPConnectionEventDisconnectedIndication_Reason_gSmSmpTimeoutOccurred_c:
                    shell_write(gSmSmpTimeoutOccurred_c);
                    break;

                case GAPConnectionEventDisconnectedIndication_Reason_gSmUnknownSmpPacketType_c:
                    shell_write(gSmUnknownSmpPacketType_c);
                    break;

                case GAPConnectionEventDisconnectedIndication_Reason_gSmInvalidSmpPacketLength_c:
                    shell_write(gSmInvalidSmpPacketLength_c);
                    break;

                case GAPConnectionEventDisconnectedIndication_Reason_gSmInvalidSmpPacketParameter_c:
                    shell_write(gSmInvalidSmpPacketParameter_c);
                    break;

                case GAPConnectionEventDisconnectedIndication_Reason_gSmReceivedUnexpectedSmpPacket_c:
                    shell_write(gSmReceivedUnexpectedSmpPacket_c);
                    break;

                case GAPConnectionEventDisconnectedIndication_Reason_gSmReceivedSmpPacketFromUnknownDevice_c:
                    shell_write(gSmReceivedSmpPacketFromUnknownDevice_c);
                    break;

                case GAPConnectionEventDisconnectedIndication_Reason_gSmReceivedUnexpectedHciEvent_c:
                    shell_write(gSmReceivedUnexpectedHciEvent_c);
                    break;

                case GAPConnectionEventDisconnectedIndication_Reason_gSmReceivedHciEventFromUnknownDevice_c:
                    shell_write(gSmReceivedHciEventFromUnknownDevice_c);
                    break;

                case GAPConnectionEventDisconnectedIndication_Reason_gSmInvalidHciEventParameter_c:
                    shell_write(gSmInvalidHciEventParameter_c);
                    break;

                case GAPConnectionEventDisconnectedIndication_Reason_gSmLlConnectionEncryptionInProgress_c:
                    shell_write(gSmLlConnectionEncryptionInProgress_c);
                    break;

                case GAPConnectionEventDisconnectedIndication_Reason_gSmLlConnectionEncryptionFailure_c:
                    shell_write(gSmLlConnectionEncryptionFailure_c);
                    break;

                case GAPConnectionEventDisconnectedIndication_Reason_gSmInsufficientResources_c:
                    shell_write(gSmInsufficientResources_c);
                    break;

                case GAPConnectionEventDisconnectedIndication_Reason_gSmOobDataAddressMismatch_c:
                    shell_write(gSmOobDataAddressMismatch_c);
                    break;

//                case GAPConnectionEventDisconnectedIndication_Reason_gSmSmpPacketReceivedAfterTimeoutOccurred_c:
//                    shell_write(gSmSmpPacketReceivedAfterTimeoutOccurred_c);
//                    break;

                case GAPConnectionEventDisconnectedIndication_Reason_gSmPairingErrorPasskeyEntryFailed_c:
                    shell_write(gSmPairingErrorPasskeyEntryFailed_c);
                    break;

                case GAPConnectionEventDisconnectedIndication_Reason_gSmPairingErrorConfirmValueFailed_c:
                    shell_write(gSmPairingErrorConfirmValueFailed_c);
                    break;

                case GAPConnectionEventDisconnectedIndication_Reason_gSmPairingErrorCommandNotSupported_c:
                    shell_write(gSmPairingErrorCommandNotSupported_c);
                    break;

                case GAPConnectionEventDisconnectedIndication_Reason_gSmPairingErrorInvalidParameters_c:
                    shell_write(gSmPairingErrorInvalidParameters_c);
                    break;

                case GAPConnectionEventDisconnectedIndication_Reason_gSmPairingErrorUnknownReason_c:
                    shell_write(gSmPairingErrorUnknown_c);
                    break;

                case GAPConnectionEventDisconnectedIndication_Reason_gSmTbResolvableAddressDoesNotMatchIrk_c:
                    shell_write(gSmTbResolvableAddressDoesNotMatchIrk_c);
                    break;

                case GAPConnectionEventDisconnectedIndication_Reason_gSmTbInvalidDataSignature_c:
                    shell_write(gSmTbInvalidDataSignature_c);
                    break;

                case GAPConnectionEventDisconnectedIndication_Reason_gAttInvalidHandle_c:
                    shell_write(gAttInvalidHandle_c);
                    break;

                case GAPConnectionEventDisconnectedIndication_Reason_gAttReadNotPermitted_c:
                    shell_write(gAttReadNotPermitted_c);
                    break;

                case GAPConnectionEventDisconnectedIndication_Reason_gAttWriteNotPermitted_c:
                    shell_write(gAttWriteNotPermitted_c);
                    break;

                case GAPConnectionEventDisconnectedIndication_Reason_gAttInvalidPdu_c:
                    shell_write(gAttInvalidPdu_c);
                    break;

                case GAPConnectionEventDisconnectedIndication_Reason_gAttInsufficientAuthentication_c:
                    shell_write(gAttInsufficientAuthentication_c);
                    break;

                case GAPConnectionEventDisconnectedIndication_Reason_gAttRequestNotSupported_c:
                    shell_write(gAttRequestNotSupported_c);
                    break;

                case GAPConnectionEventDisconnectedIndication_Reason_gAttInvalidOffset_c:
                    shell_write(gAttInvalidOffset_c);
                    break;

                case GAPConnectionEventDisconnectedIndication_Reason_gAttInsufficientAuthorization_c:
                    shell_write(gAttInsufficientAuthorization_c);
                    break;

                case GAPConnectionEventDisconnectedIndication_Reason_gAttPrepareQueueFull_c:
                    shell_write(gAttPrepareQueueFull_c);
                    break;

                case GAPConnectionEventDisconnectedIndication_Reason_gAttAttributeNotFound_c:
                    shell_write(gAttAttributeNotFound_c);
                    break;

                case GAPConnectionEventDisconnectedIndication_Reason_gAttAttributeNotLong_c:
                    shell_write(gAttAttributeNotLong_c);
                    break;

                case GAPConnectionEventDisconnectedIndication_Reason_gAttInsufficientEncryptionKeySize_c:
                    shell_write(gAttInsufficientEncryptionKeySize_c);
                    break;

                case GAPConnectionEventDisconnectedIndication_Reason_gAttInvalidAttributeValueLength_c:
                    shell_write(gAttInvalidAttributeValueLength_c);
                    break;

                case GAPConnectionEventDisconnectedIndication_Reason_gAttUnlikelyor_c:
                    shell_write(gAttUnlikelyor_c);
                    break;

                case GAPConnectionEventDisconnectedIndication_Reason_gAttInsufficientEncryption_c:
                    shell_write(gAttInsufficientEncryption_c);
                    break;

                case GAPConnectionEventDisconnectedIndication_Reason_gAttUnsupportedGroupType_c:
                    shell_write(gAttUnsupportedGroupType_c);
                    break;

                case GAPConnectionEventDisconnectedIndication_Reason_gAttInsufficientResources_c:
                    shell_write(gAttInsufficientResources_c);
                    break;

                case GAPConnectionEventDisconnectedIndication_Reason_gGattAnotherProcedureInProgress_c:
                    shell_write(gGattAnotherProcedureInProgress_c);
                    break;

                case GAPConnectionEventDisconnectedIndication_Reason_gGattLongAttributePacketsCorrupted_c:
                    shell_write(gGattLongAttributePacketsCorrupted_c);
                    break;

                case GAPConnectionEventDisconnectedIndication_Reason_gGattMultipleAttributesOverflow_c:
                    shell_write(gGattMultipleAttributesOverflow_c);
                    break;

                case GAPConnectionEventDisconnectedIndication_Reason_gGattUnexpectedReadMultipleResponseLength_c:
                    shell_write(gGattUnexpectedReadMultipleResponseLength_c);
                    break;

                case GAPConnectionEventDisconnectedIndication_Reason_gGattInvalidValueLength_c:
                    shell_write(gGattInvalidValueLength_c);
                    break;

                case GAPConnectionEventDisconnectedIndication_Reason_gGattServerTimeout_c:
                    shell_write(gGattServerTimeout_c);
                    break;

                case GAPConnectionEventDisconnectedIndication_Reason_gGattIndicationAlreadyInProgress_c:
                    shell_write(gGattIndicationAlreadyInProgress_c);
                    break;

                case GAPConnectionEventDisconnectedIndication_Reason_gGattClientConfirmationTimeout_c:
                    shell_write(gGattClientConfirmationTimeout_c);
                    break;

                case GAPConnectionEventDisconnectedIndication_Reason_gGapAdvDataTooLong_c:
                    shell_write(gGapAdvDataTooLong_c);
                    break;

                case GAPConnectionEventDisconnectedIndication_Reason_gGapScanRspDataTooLong_c:
                    shell_write(gGapScanRspDataTooLong_c);
                    break;

                case GAPConnectionEventDisconnectedIndication_Reason_gGapDeviceNotBonded_c:
                    shell_write(gGapDeviceNotBonded_c);
                    break;

                case GAPConnectionEventDisconnectedIndication_Reason_gDevDbCccdLimitReached_c:
                    shell_write(gDevDbCccdLimitReached_c);
                    break;

                case GAPConnectionEventDisconnectedIndication_Reason_gDevDbCccdNotFound_c:
                    shell_write(gDevDbCccdNotFound_c);
                    break;

                case GAPConnectionEventDisconnectedIndication_Reason_gGattDbInvalidHandle_c:
                    shell_write(gGattDbInvalidHandle_c);
                    break;

                case GAPConnectionEventDisconnectedIndication_Reason_gGattDbCharacteristicNotFound_c:
                    shell_write(gGattDbCharacteristicNotFound_c);
                    break;

                case GAPConnectionEventDisconnectedIndication_Reason_gGattDbCccdNotFound_c:
                    shell_write(gGattDbCccdNotFound_c);
                    break;

                case GAPConnectionEventDisconnectedIndication_Reason_gGattDbServiceNotFound_c:
                    shell_write(gGattDbServiceNotFound_c);
                    break;

                case GAPConnectionEventDisconnectedIndication_Reason_gGattDbDescriptorNotFound_c:
                    shell_write(gGattDbDescriptorNotFound_c);
                    break;

                default:
                    shell_printf("Unrecognized status 0x%04X", container->Data.GAPConnectionEventDisconnectedIndication.Reason);
                    break;
            }

            gPeerDeviceId = gInvalidDeviceId_c;

            if (gRpkDemoState == gDemoConnectionEstablish_c)
            {
                gRpkDemoState = gDemoSetAdvParam_c;
                mSuppressBleEventPrint = FALSE;
                BleApp_DemoRpk();
            }

            break;

        case 0x47AB:
            shell_write("\r\nGAPConnectionEventRssiReadIndication");
            break;

        case 0x47AC:
            shell_write("\r\nGAPConnectionEventTxPowerLevelReadIndication");
            break;

        case 0x47AD:
            shell_write("\r\nGAPConnectionEventPowerReadFailureIndication");
            shell_write(" -> ");

            switch (container->Data.GAPConnectionEventPowerReadFailureIndication.FailReason)
            {
                case GAPConnectionEventPowerReadFailureIndication_FailReason_gBleSuccess_c:
                    shell_write(gBleSuccess_c);
                    break;

                case GAPConnectionEventPowerReadFailureIndication_FailReason_gBleInvalidParameter_c:
                    shell_write(gBleInvalidParameter_c);
                    break;

                case GAPConnectionEventPowerReadFailureIndication_FailReason_gBleOverflow_c:
                    shell_write(gBleOverflow_c);
                    break;

                case GAPConnectionEventPowerReadFailureIndication_FailReason_gBleUnavailable_c:
                    shell_write(gBleUnavailable_c);
                    break;

                case GAPConnectionEventPowerReadFailureIndication_FailReason_gBleFeatureNotSupported_c:
                    shell_write(gBleFeatureNotSupported_c);
                    break;

                case GAPConnectionEventPowerReadFailureIndication_FailReason_gBleOutOfMemory_c:
                    shell_write(gBleOutOfMemory_c);
                    break;

                case GAPConnectionEventPowerReadFailureIndication_FailReason_gBleAlreadyInitialized_c:
                    shell_write(gBleAlreadyInitialized_c);
                    break;

                case GAPConnectionEventPowerReadFailureIndication_FailReason_gBleOsError_c:
                    shell_write(gBleOsError_c);
                    break;

                case GAPConnectionEventPowerReadFailureIndication_FailReason_gBleUnexpectedError_c:
                    shell_write(gBleUnexpectedError_c);
                    break;

                case GAPConnectionEventPowerReadFailureIndication_FailReason_gBleInvalidState_c:
                    shell_write(gBleInvalidState_c);
                    break;

                case GAPConnectionEventPowerReadFailureIndication_FailReason_gHciUnknownHciCommand_c:
                    shell_write(gHciUnknownHciCommand_c);
                    break;

                case GAPConnectionEventPowerReadFailureIndication_FailReason_gHciUnknownConnectionIdentifier_c:
                    shell_write(gHciUnknownConnectionIdentifier_c);
                    break;

                case GAPConnectionEventPowerReadFailureIndication_FailReason_gHciHardwareFailure_c:
                    shell_write(gHciHardwareFailure_c);
                    break;

                case GAPConnectionEventPowerReadFailureIndication_FailReason_gHciPageTimeout_c:
                    shell_write(gHciPageTimeout_c);
                    break;

                case GAPConnectionEventPowerReadFailureIndication_FailReason_gHciAuthenticationFailure_c:
                    shell_write(gHciAuthenticationFailure_c);
                    break;

                case GAPConnectionEventPowerReadFailureIndication_FailReason_gHciPinOrKeyMissing_c:
                    shell_write(gHciPinOrKeyMissing_c);
                    break;

                case GAPConnectionEventPowerReadFailureIndication_FailReason_gHciMemoryCapacityExceeded_c:
                    shell_write(gHciMemoryCapacityExceeded_c);
                    break;

                case GAPConnectionEventPowerReadFailureIndication_FailReason_gHciConnectionTimeout_c:
                    shell_write(gHciConnectionTimeout_c);
                    break;

                case GAPConnectionEventPowerReadFailureIndication_FailReason_gHciConnectionLimitExceeded_c:
                    shell_write(gHciConnectionLimitExceeded_c);
                    break;

                case GAPConnectionEventPowerReadFailureIndication_FailReason_gHciSynchronousConnectionLimitToADeviceExceeded_c:
                    shell_write(gHciSynchronousConnectionLimitToADeviceExceeded_c);
                    break;

                case GAPConnectionEventPowerReadFailureIndication_FailReason_gHciAclConnectionAlreadyExists_c:
                    shell_write(gHciAclConnectionAlreadyExists_c);
                    break;

                case GAPConnectionEventPowerReadFailureIndication_FailReason_gHciCommandDisallowed_c:
                    shell_write(gHciCommandDisallowed_c);
                    break;

                case GAPConnectionEventPowerReadFailureIndication_FailReason_gHciConnectionRejectedDueToLimitedResources_c:
                    shell_write(gHciConnectionRejectedDueToLimitedResources_c);
                    break;

                case GAPConnectionEventPowerReadFailureIndication_FailReason_gHciConnectionRejectedDueToSecurityReasons_c:
                    shell_write(gHciConnectionRejectedDueToSecurityReasons_c);
                    break;

                case GAPConnectionEventPowerReadFailureIndication_FailReason_gHciConnectionRejectedDueToUnacceptableBdAddr_c:
                    shell_write(gHciConnectionRejectedDueToUnacceptableBdAddr_c);
                    break;

                case GAPConnectionEventPowerReadFailureIndication_FailReason_gHciConnectionAcceptTimeoutExceeded_c:
                    shell_write(gHciConnectionAcceptTimeoutExceeded_c);
                    break;

                case GAPConnectionEventPowerReadFailureIndication_FailReason_gHciUnsupportedFeatureOrParameterValue_c:
                    shell_write(gHciUnsupportedFeatureOrParameterValue_c);
                    break;

                case GAPConnectionEventPowerReadFailureIndication_FailReason_gHciInvalidHciCommandParameters_c:
                    shell_write(gHciInvalidHciCommandParameters_c);
                    break;

                case GAPConnectionEventPowerReadFailureIndication_FailReason_gHciRemoteUserTerminatedConnection_c:
                    shell_write(gHciRemoteUserTerminatedConnection_c);
                    break;

                case GAPConnectionEventPowerReadFailureIndication_FailReason_gHciRemoteDeviceTerminatedConnectionLowResources_c:
                    shell_write(gHciRemoteDeviceTerminatedConnectionLowResources_c);
                    break;

                case GAPConnectionEventPowerReadFailureIndication_FailReason_gHciRemoteDeviceTerminatedConnectionPowerOff_c:
                    shell_write(gHciRemoteDeviceTerminatedConnectionPowerOff_c);
                    break;

                case GAPConnectionEventPowerReadFailureIndication_FailReason_gHciConnectionTerminatedByLocalHost_c:
                    shell_write(gHciConnectionTerminatedByLocalHost_c);
                    break;

                case GAPConnectionEventPowerReadFailureIndication_FailReason_gHciRepeatedAttempts_c:
                    shell_write(gHciRepeatedAttempts_c);
                    break;

                case GAPConnectionEventPowerReadFailureIndication_FailReason_gHciPairingNotAllowed_c:
                    shell_write(gHciPairingNotAllowed_c);
                    break;

                case GAPConnectionEventPowerReadFailureIndication_FailReason_gHciUnknownLpmPdu_c:
                    shell_write(gHciUnknownLpmPdu_c);
                    break;

                case GAPConnectionEventPowerReadFailureIndication_FailReason_gHciUnsupportedRemoteFeature_c:
                    shell_write(gHciUnsupportedRemoteFeature_c);
                    break;

                case GAPConnectionEventPowerReadFailureIndication_FailReason_gHciScoOffsetRejected_c:
                    shell_write(gHciScoOffsetRejected_c);
                    break;

                case GAPConnectionEventPowerReadFailureIndication_FailReason_gHciScoIntervalRejected_c:
                    shell_write(gHciScoIntervalRejected_c);
                    break;

                case GAPConnectionEventPowerReadFailureIndication_FailReason_gHciScoAirModeRejected_c:
                    shell_write(gHciScoAirModeRejected_c);
                    break;

                case GAPConnectionEventPowerReadFailureIndication_FailReason_gHciInvalidLpmParameters_c:
                    shell_write(gHciInvalidLpmParameters_c);
                    break;

                case GAPConnectionEventPowerReadFailureIndication_FailReason_gHciUnspecifiedError_c:
                    shell_write(gHciUnspecifiedError_c);
                    break;

                case GAPConnectionEventPowerReadFailureIndication_FailReason_gHciUnsupportedLpmParameterValue_c:
                    shell_write(gHciUnsupportedLpmParameterValue_c);
                    break;

                case GAPConnectionEventPowerReadFailureIndication_FailReason_gHciRoleChangeNotAllowed_c:
                    shell_write(gHciRoleChangeNotAllowed_c);
                    break;

                case GAPConnectionEventPowerReadFailureIndication_FailReason_gHciLLResponseTimeout_c:
                    shell_write(gHciLLResponseTimeout_c);
                    break;

                case GAPConnectionEventPowerReadFailureIndication_FailReason_gHciLmpErrorTransactionCollision_c:
                    shell_write(gHciLmpErrorTransactionCollision_c);
                    break;

                case GAPConnectionEventPowerReadFailureIndication_FailReason_gHciLmpPduNotAllowed_c:
                    shell_write(gHciLmpPduNotAllowed_c);
                    break;

                case GAPConnectionEventPowerReadFailureIndication_FailReason_gHciEncryptionModeNotAcceptable_c:
                    shell_write(gHciEncryptionModeNotAcceptable_c);
                    break;

                case GAPConnectionEventPowerReadFailureIndication_FailReason_gHciLinkKeyCannotBeChanged_c:
                    shell_write(gHciLinkKeyCannotBeChanged_c);
                    break;

                case GAPConnectionEventPowerReadFailureIndication_FailReason_gHciRequestedQosNotSupported_c:
                    shell_write(gHciRequestedQosNotSupported_c);
                    break;

                case GAPConnectionEventPowerReadFailureIndication_FailReason_gHciInstantPassed_c:
                    shell_write(gHciInstantPassed_c);
                    break;

                case GAPConnectionEventPowerReadFailureIndication_FailReason_gHciPairingWithUnitKeyNotSupported_c:
                    shell_write(gHciPairingWithUnitKeyNotSupported_c);
                    break;

                case GAPConnectionEventPowerReadFailureIndication_FailReason_gHciDifferentTransactionCollision_c:
                    shell_write(gHciDifferentTransactionCollision_c);
                    break;

                case GAPConnectionEventPowerReadFailureIndication_FailReason_gHciReserved_0x2B_c:
                    shell_write(gHciReserved_0x2B_c);
                    break;

                case GAPConnectionEventPowerReadFailureIndication_FailReason_gHciQosNotAcceptableParameter_c:
                    shell_write(gHciQosNotAcceptableParameter_c);
                    break;

                case GAPConnectionEventPowerReadFailureIndication_FailReason_gHciQosRejected_c:
                    shell_write(gHciQosRejected_c);
                    break;

                case GAPConnectionEventPowerReadFailureIndication_FailReason_gHciChannelClassificationNotSupported_c:
                    shell_write(gHciChannelClassificationNotSupported_c);
                    break;

                case GAPConnectionEventPowerReadFailureIndication_FailReason_gHciInsufficientSecurity_c:
                    shell_write(gHciInsufficientSecurity_c);
                    break;

                case GAPConnectionEventPowerReadFailureIndication_FailReason_gHciParameterOutOfMandatoryRange_c:
                    shell_write(gHciParameterOutOfMandatoryRange_c);
                    break;

                case GAPConnectionEventPowerReadFailureIndication_FailReason_gHciReserved_0x31_c:
                    shell_write(gHciReserved_0x31_c);
                    break;

                case GAPConnectionEventPowerReadFailureIndication_FailReason_gHciRoleSwitchPending_c:
                    shell_write(gHciRoleSwitchPending_c);
                    break;

                case GAPConnectionEventPowerReadFailureIndication_FailReason_gHciReserved_0x33_c:
                    shell_write(gHciReserved_0x33_c);
                    break;

                case GAPConnectionEventPowerReadFailureIndication_FailReason_gHciReservedSlotViolation_c:
                    shell_write(gHciReservedSlotViolation_c);
                    break;

                case GAPConnectionEventPowerReadFailureIndication_FailReason_gHciRoleSwitchFailed_c:
                    shell_write(gHciRoleSwitchFailed_c);
                    break;

                case GAPConnectionEventPowerReadFailureIndication_FailReason_gHciExtendedInquiryResponseTooLarge_c:
                    shell_write(gHciExtendedInquiryResponseTooLarge_c);
                    break;

                case GAPConnectionEventPowerReadFailureIndication_FailReason_gHciSecureSimplePairingNotSupportedByHost_c:
                    shell_write(gHciSecureSimplePairingNotSupportedByHost_c);
                    break;

                case GAPConnectionEventPowerReadFailureIndication_FailReason_gHciHostBusyPairing_c:
                    shell_write(gHciHostBusyPairing_c);
                    break;

                case GAPConnectionEventPowerReadFailureIndication_FailReason_gHciConnectionRejectedDueToNoSuitableChannelFound_c:
                    shell_write(gHciConnectionRejectedDueToNoSuitableChannelFound_c);
                    break;

                case GAPConnectionEventPowerReadFailureIndication_FailReason_gHciControllerBusy_c:
                    shell_write(gHciControllerBusy_c);
                    break;

                case GAPConnectionEventPowerReadFailureIndication_FailReason_gHciUnacceptableConnectionParameters_c:
                    shell_write(gHciUnacceptableConnectionParameters_c);
                    break;

                case GAPConnectionEventPowerReadFailureIndication_FailReason_gHciDirectedAdvertisingTimeout_c:
                    shell_write(gHciDirectedAdvertisingTimeout_c);
                    break;

                case GAPConnectionEventPowerReadFailureIndication_FailReason_gHciConnectionTerminatedDueToMicFailure_c:
                    shell_write(gHciConnectionTerminatedDueToMicFailure_c);
                    break;

                case GAPConnectionEventPowerReadFailureIndication_FailReason_gHciConnectionFailedToBeEstablished_c:
                    shell_write(gHciConnectionFailedToBeEstablished_c);
                    break;

                case GAPConnectionEventPowerReadFailureIndication_FailReason_gHciMacConnectionFailed_c:
                    shell_write(gHciMacConnectionFailed_c);
                    break;

                case GAPConnectionEventPowerReadFailureIndication_FailReason_gHciCoarseClockAdjustmentRejected_c:
                    shell_write(gHciCoarseClockAdjustmentRejected_c);
                    break;

                case GAPConnectionEventPowerReadFailureIndication_FailReason_gHciAlreadyInit_c:
                    shell_write(gHciAlreadyInit_c);
                    break;

                case GAPConnectionEventPowerReadFailureIndication_FailReason_gHciInvalidParameter_c:
                    shell_write(gHciInvalidParameter_c);
                    break;

                case GAPConnectionEventPowerReadFailureIndication_FailReason_gHciCallbackNotInstalled_c:
                    shell_write(gHciCallbackNotInstalled_c);
                    break;

                case GAPConnectionEventPowerReadFailureIndication_FailReason_gHciCallbackAlreadyInstalled_c:
                    shell_write(gHciCallbackAlreadyInstalled_c);
                    break;

                case GAPConnectionEventPowerReadFailureIndication_FailReason_gHciCommandNotSupported_c:
                    shell_write(gHciCommandNotSupported_c);
                    break;

                case GAPConnectionEventPowerReadFailureIndication_FailReason_gHciEventNotSupported_c:
                    shell_write(gHciEventNotSupported_c);
                    break;

                case GAPConnectionEventPowerReadFailureIndication_FailReason_gHciTransportError_c:
                    shell_write(gHciTransportError_c);
                    break;

                case GAPConnectionEventPowerReadFailureIndication_FailReason_gL2caAlreadyInit_c:
                    shell_write(gL2caAlreadyInit_c);
                    break;

                case GAPConnectionEventPowerReadFailureIndication_FailReason_gL2caInsufficientResources_c:
                    shell_write(gL2caInsufficientResources_c);
                    break;

                case GAPConnectionEventPowerReadFailureIndication_FailReason_gL2caCallbackNotInstalled_c:
                    shell_write(gL2caCallbackNotInstalled_c);
                    break;

                case GAPConnectionEventPowerReadFailureIndication_FailReason_gL2caCallbackAlreadyInstalled_c:
                    shell_write(gL2caCallbackAlreadyInstalled_c);
                    break;

                case GAPConnectionEventPowerReadFailureIndication_FailReason_gL2caLePsmInvalid_c:
                    shell_write(gL2caLePsmInvalid_c);
                    break;

                case GAPConnectionEventPowerReadFailureIndication_FailReason_gL2caLePsmAlreadyRegistered_c:
                    shell_write(gL2caLePsmAlreadyRegistered_c);
                    break;

                case GAPConnectionEventPowerReadFailureIndication_FailReason_gL2caLePsmNotRegistered_c:
                    shell_write(gL2caLePsmNotRegistered_c);
                    break;

                case GAPConnectionEventPowerReadFailureIndication_FailReason_gL2caLePsmInsufficientResources_c:
                    shell_write(gL2caLePsmInsufficientResources_c);
                    break;

                case GAPConnectionEventPowerReadFailureIndication_FailReason_gL2caChannelInvalid_c:
                    shell_write(gL2caChannelInvalid_c);
                    break;

                case GAPConnectionEventPowerReadFailureIndication_FailReason_gL2caChannelClosed_c:
                    shell_write(gL2caChannelClosed_c);
                    break;

                case GAPConnectionEventPowerReadFailureIndication_FailReason_gL2caChannelAlreadyConnected_c:
                    shell_write(gL2caChannelAlreadyConnected_c);
                    break;

                case GAPConnectionEventPowerReadFailureIndication_FailReason_gL2caConnectionParametersRejected_c:
                    shell_write(gL2caConnectionParametersRejected_c);
                    break;

                case GAPConnectionEventPowerReadFailureIndication_FailReason_gL2caChannelBusy_c:
                    shell_write(gL2caChannelBusy_c);
                    break;

                case GAPConnectionEventPowerReadFailureIndication_FailReason_gL2caInvalidParameter_c:
                    shell_write(gL2caInvalidParameter_c);
                    break;

                case GAPConnectionEventPowerReadFailureIndication_FailReason_gL2caError_c:
                    shell_write(gL2caError_c);
                    break;

                case GAPConnectionEventPowerReadFailureIndication_FailReason_gSmNullCBFunction_c:
                    shell_write(gSmNullCBFunction_c);
                    break;

                case GAPConnectionEventPowerReadFailureIndication_FailReason_gSmCommandNotSupported_c:
                    shell_write(gSmCommandNotSupported_c);
                    break;

                case GAPConnectionEventPowerReadFailureIndication_FailReason_gSmUnexpectedCommand_c:
                    shell_write(gSmUnexpectedCommand_c);
                    break;

                case GAPConnectionEventPowerReadFailureIndication_FailReason_gSmInvalidCommandCode_c:
                    shell_write(gSmInvalidCommandCode_c);
                    break;

                case GAPConnectionEventPowerReadFailureIndication_FailReason_gSmInvalidCommandLength_c:
                    shell_write(gSmInvalidCommandLength_c);
                    break;

                case GAPConnectionEventPowerReadFailureIndication_FailReason_gSmInvalidCommandParameter_c:
                    shell_write(gSmInvalidCommandParameter_c);
                    break;

                case GAPConnectionEventPowerReadFailureIndication_FailReason_gSmInvalidDeviceId_c:
                    shell_write(gSmInvalidDeviceId_c);
                    break;

                case GAPConnectionEventPowerReadFailureIndication_FailReason_gSmInvalidInternalOperation_c:
                    shell_write(gSmInvalidInternalOperation_c);
                    break;

                case GAPConnectionEventPowerReadFailureIndication_FailReason_gSmInvalidConnectionHandle_c:
                    shell_write(gSmInvalidConnectionHandle_c);
                    break;

                case GAPConnectionEventPowerReadFailureIndication_FailReason_gSmInproperKeyDistributionField_c:
                    shell_write(gSmInproperKeyDistributionField_c);
                    break;

                case GAPConnectionEventPowerReadFailureIndication_FailReason_gSmUnexpectedKeyType_c:
                    shell_write(gSmUnexpectedKeyType_c);
                    break;

                case GAPConnectionEventPowerReadFailureIndication_FailReason_gSmUnexpectedPairingTerminationReason_c:
                    shell_write(gSmUnexpectedPairingTerminationReason_c);
                    break;

                case GAPConnectionEventPowerReadFailureIndication_FailReason_gSmUnexpectedKeyset_c:
                    shell_write(gSmUnexpectedKeyset_c);
                    break;

                case GAPConnectionEventPowerReadFailureIndication_FailReason_gSmSmpTimeoutOccurred_c:
                    shell_write(gSmSmpTimeoutOccurred_c);
                    break;

                case GAPConnectionEventPowerReadFailureIndication_FailReason_gSmUnknownSmpPacketType_c:
                    shell_write(gSmUnknownSmpPacketType_c);
                    break;

                case GAPConnectionEventPowerReadFailureIndication_FailReason_gSmInvalidSmpPacketLength_c:
                    shell_write(gSmInvalidSmpPacketLength_c);
                    break;

                case GAPConnectionEventPowerReadFailureIndication_FailReason_gSmInvalidSmpPacketParameter_c:
                    shell_write(gSmInvalidSmpPacketParameter_c);
                    break;

                case GAPConnectionEventPowerReadFailureIndication_FailReason_gSmReceivedUnexpectedSmpPacket_c:
                    shell_write(gSmReceivedUnexpectedSmpPacket_c);
                    break;

                case GAPConnectionEventPowerReadFailureIndication_FailReason_gSmReceivedSmpPacketFromUnknownDevice_c:
                    shell_write(gSmReceivedSmpPacketFromUnknownDevice_c);
                    break;

                case GAPConnectionEventPowerReadFailureIndication_FailReason_gSmReceivedUnexpectedHciEvent_c:
                    shell_write(gSmReceivedUnexpectedHciEvent_c);
                    break;

                case GAPConnectionEventPowerReadFailureIndication_FailReason_gSmReceivedHciEventFromUnknownDevice_c:
                    shell_write(gSmReceivedHciEventFromUnknownDevice_c);
                    break;

                case GAPConnectionEventPowerReadFailureIndication_FailReason_gSmInvalidHciEventParameter_c:
                    shell_write(gSmInvalidHciEventParameter_c);
                    break;

                case GAPConnectionEventPowerReadFailureIndication_FailReason_gSmLlConnectionEncryptionInProgress_c:
                    shell_write(gSmLlConnectionEncryptionInProgress_c);
                    break;

                case GAPConnectionEventPowerReadFailureIndication_FailReason_gSmLlConnectionEncryptionFailure_c:
                    shell_write(gSmLlConnectionEncryptionFailure_c);
                    break;

                case GAPConnectionEventPowerReadFailureIndication_FailReason_gSmInsufficientResources_c:
                    shell_write(gSmInsufficientResources_c);
                    break;

                case GAPConnectionEventPowerReadFailureIndication_FailReason_gSmOobDataAddressMismatch_c:
                    shell_write(gSmOobDataAddressMismatch_c);
                    break;

//                case GAPConnectionEventPowerReadFailureIndication_FailReason_gSmSmpPacketReceivedAfterTimeoutOccurred_c:
//                    shell_write(gSmSmpPacketReceivedAfterTimeoutOccurred_c);
//                    break;

                case GAPConnectionEventPowerReadFailureIndication_FailReason_gSmPairingErrorPasskeyEntryFailed_c:
                    shell_write(gSmPairingErrorPasskeyEntryFailed_c);
                    break;

                case GAPConnectionEventPowerReadFailureIndication_FailReason_gSmPairingErrorConfirmValueFailed_c:
                    shell_write(gSmPairingErrorConfirmValueFailed_c);
                    break;

                case GAPConnectionEventPowerReadFailureIndication_FailReason_gSmPairingErrorCommandNotSupported_c:
                    shell_write(gSmPairingErrorCommandNotSupported_c);
                    break;

                case GAPConnectionEventPowerReadFailureIndication_FailReason_gSmPairingErrorInvalidParameters_c:
                    shell_write(gSmPairingErrorInvalidParameters_c);
                    break;

                case GAPConnectionEventPowerReadFailureIndication_FailReason_gSmPairingErrorUnknownReason_c:
                    shell_write(gSmPairingErrorUnknownReason_c);
                    break;

                case GAPConnectionEventPowerReadFailureIndication_FailReason_gSmTbResolvableAddressDoesNotMatchIrk_c:
                    shell_write(gSmTbResolvableAddressDoesNotMatchIrk_c);
                    break;

                case GAPConnectionEventPowerReadFailureIndication_FailReason_gSmTbInvalidDataSignature_c:
                    shell_write(gSmTbInvalidDataSignature_c);
                    break;

                case GAPConnectionEventPowerReadFailureIndication_FailReason_gAttInvalidHandle_c:
                    shell_write(gAttInvalidHandle_c);
                    break;

                case GAPConnectionEventPowerReadFailureIndication_FailReason_gAttReadNotPermitted_c:
                    shell_write(gAttReadNotPermitted_c);
                    break;

                case GAPConnectionEventPowerReadFailureIndication_FailReason_gAttWriteNotPermitted_c:
                    shell_write(gAttWriteNotPermitted_c);
                    break;

                case GAPConnectionEventPowerReadFailureIndication_FailReason_gAttInvalidPdu_c:
                    shell_write(gAttInvalidPdu_c);
                    break;

                case GAPConnectionEventPowerReadFailureIndication_FailReason_gAttInsufficientAuthentication_c:
                    shell_write(gAttInsufficientAuthentication_c);
                    break;

                case GAPConnectionEventPowerReadFailureIndication_FailReason_gAttRequestNotSupported_c:
                    shell_write(gAttRequestNotSupported_c);
                    break;

                case GAPConnectionEventPowerReadFailureIndication_FailReason_gAttInvalidOffset_c:
                    shell_write(gAttInvalidOffset_c);
                    break;

                case GAPConnectionEventPowerReadFailureIndication_FailReason_gAttInsufficientAuthorization_c:
                    shell_write(gAttInsufficientAuthorization_c);
                    break;

                case GAPConnectionEventPowerReadFailureIndication_FailReason_gAttPrepareQueueFull_c:
                    shell_write(gAttPrepareQueueFull_c);
                    break;

                case GAPConnectionEventPowerReadFailureIndication_FailReason_gAttAttributeNotFound_c:
                    shell_write(gAttAttributeNotFound_c);
                    break;

                case GAPConnectionEventPowerReadFailureIndication_FailReason_gAttAttributeNotLong_c:
                    shell_write(gAttAttributeNotLong_c);
                    break;

                case GAPConnectionEventPowerReadFailureIndication_FailReason_gAttInsufficientEncryptionKeySize_c:
                    shell_write(gAttInsufficientEncryptionKeySize_c);
                    break;

                case GAPConnectionEventPowerReadFailureIndication_FailReason_gAttInvalidAttributeValueLength_c:
                    shell_write(gAttInvalidAttributeValueLength_c);
                    break;

                case GAPConnectionEventPowerReadFailureIndication_FailReason_gAttUnlikelyor_c:
                    shell_write(gAttUnlikelyor_c);
                    break;

                case GAPConnectionEventPowerReadFailureIndication_FailReason_gAttInsufficientEncryption_c:
                    shell_write(gAttInsufficientEncryption_c);
                    break;

                case GAPConnectionEventPowerReadFailureIndication_FailReason_gAttUnsupportedGroupType_c:
                    shell_write(gAttUnsupportedGroupType_c);
                    break;

                case GAPConnectionEventPowerReadFailureIndication_FailReason_gAttInsufficientResources_c:
                    shell_write(gAttInsufficientResources_c);
                    break;

                case GAPConnectionEventPowerReadFailureIndication_FailReason_gGattAnotherProcedureInProgress_c:
                    shell_write(gGattAnotherProcedureInProgress_c);
                    break;

                case GAPConnectionEventPowerReadFailureIndication_FailReason_gGattLongAttributePacketsCorrupted_c:
                    shell_write(gGattLongAttributePacketsCorrupted_c);
                    break;

                case GAPConnectionEventPowerReadFailureIndication_FailReason_gGattMultipleAttributesOverflow_c:
                    shell_write(gGattMultipleAttributesOverflow_c);
                    break;

                case GAPConnectionEventPowerReadFailureIndication_FailReason_gGattUnexpectedReadMultipleResponseLength_c:
                    shell_write(gGattUnexpectedReadMultipleResponseLength_c);
                    break;

                case GAPConnectionEventPowerReadFailureIndication_FailReason_gGattInvalidValueLength_c:
                    shell_write(gGattInvalidValueLength_c);
                    break;

                case GAPConnectionEventPowerReadFailureIndication_FailReason_gGattServerTimeout_c:
                    shell_write(gGattServerTimeout_c);
                    break;

                case GAPConnectionEventPowerReadFailureIndication_FailReason_gGattIndicationAlreadyInProgress_c:
                    shell_write(gGattIndicationAlreadyInProgress_c);
                    break;

                case GAPConnectionEventPowerReadFailureIndication_FailReason_gGattClientConfirmationTimeout_c:
                    shell_write(gGattClientConfirmationTimeout_c);
                    break;

                case GAPConnectionEventPowerReadFailureIndication_FailReason_gGapAdvDataTooLong_c:
                    shell_write(gGapAdvDataTooLong_c);
                    break;

                case GAPConnectionEventPowerReadFailureIndication_FailReason_gGapScanRspDataTooLong_c:
                    shell_write(gGapScanRspDataTooLong_c);
                    break;

                case GAPConnectionEventPowerReadFailureIndication_FailReason_gGapDeviceNotBonded_c:
                    shell_write(gGapDeviceNotBonded_c);
                    break;

                case GAPConnectionEventPowerReadFailureIndication_FailReason_gDevDbCccdLimitReached_c:
                    shell_write(gDevDbCccdLimitReached_c);
                    break;

                case GAPConnectionEventPowerReadFailureIndication_FailReason_gDevDbCccdNotFound_c:
                    shell_write(gDevDbCccdNotFound_c);
                    break;

                case GAPConnectionEventPowerReadFailureIndication_FailReason_gGattDbInvalidHandle_c:
                    shell_write(gGattDbInvalidHandle_c);
                    break;

                case GAPConnectionEventPowerReadFailureIndication_FailReason_gGattDbCharacteristicNotFound_c:
                    shell_write(gGattDbCharacteristicNotFound_c);
                    break;

                case GAPConnectionEventPowerReadFailureIndication_FailReason_gGattDbCccdNotFound_c:
                    shell_write(gGattDbCccdNotFound_c);
                    break;

                case GAPConnectionEventPowerReadFailureIndication_FailReason_gGattDbServiceNotFound_c:
                    shell_write(gGattDbServiceNotFound_c);
                    break;

                case GAPConnectionEventPowerReadFailureIndication_FailReason_gGattDbDescriptorNotFound_c:
                    shell_write(gGattDbDescriptorNotFound_c);
                    break;

                default:
                    shell_printf("Unrecognized status 0x%04X", container->Data.GAPConnectionEventPowerReadFailureIndication.FailReason);
                    break;
            }

            break;

        case 0x47AE:
            shell_write("\r\nGAPConnectionEventParameterUpdateRequestIndication");
            break;

        case 0x47AF:
            shell_write("\r\nGAPConnectionEventParameterUpdateCompleteIndication");
            break;

        case 0x47B0:
            shell_write("\r\nGAPConnectionEventLeDataLengthChangedIndication");
            break;

        case 0x47B1:
            shell_write("\r\nGAPConnectionEventLeScOobDataRequestIndication");
            break;

        case 0x47B2:
            shell_write("\r\nGAPConnectionEventLeScDisplayNumericValueIndication ***** ");
            shell_writeDec(container->Data.GAPConnectionEventPasskeyDisplayIndication.PasskeyForDisplay);
            break;

        case 0x47B3:
            shell_write("\r\nGAPConnectionEventLeScKeypressNotificationIndication");
            shell_write(" -> ");

            switch (container->Data.GAPConnectionEventLeScKeypressNotificationIndication.GapLeScKeypressNotificationParams_keypressNotifType)
            {
                default:
                    shell_printf("Unrecognized status 0x%02X", container->Data.GAPConnectionEventLeScKeypressNotificationIndication.GapLeScKeypressNotificationParams_keypressNotifType);
                    break;
            }

            break;

        case 0x47B4:
            shell_write("\r\nGAPGenericEventControllerResetCompleteIndication");
            break;

        case 0x47B5:
            shell_write("\r\nGAPLeScPublicKeyRegeneratedIndication");
            break;

        case 0x47B6:
            shell_write("\r\nGAPGenericEventLeScLocalOobDataIndication");
            break;

        case 0x47B7:
            shell_write("\r\nGAPGenericEventControllerPrivacyStateChangedIndication");
            break;

//        case 0x4783:
//            shell_write("\r\nGAPGetBondedDevicesIdentityInformationIndication");
//            break;

    }

    if (!mSuppressBleEventPrint)
    {
        shell_cmd_finished();
    }
}

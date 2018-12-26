/* Source file generated from BLE.xml */

/*==================================================================================================
Include Files
==================================================================================================*/
#include "cmd_ble.h"
#include "FunctionLib.h"

/*==================================================================================================
Private Prototypes
==================================================================================================*/
static memStatus_t Load_FSCIErrorIndication(bleEvtContainer_t *container, uint8_t *pPayload);
static memStatus_t Load_FSCIAllowDeviceToSleepConfirm(bleEvtContainer_t *container, uint8_t *pPayload);
static memStatus_t Load_FSCIWakeUpIndication(bleEvtContainer_t *container, uint8_t *pPayload);
static memStatus_t Load_FSCIGetWakeupReasonResponse(bleEvtContainer_t *container, uint8_t *pPayload);
static memStatus_t Load_L2CAPConfirm(bleEvtContainer_t *container, uint8_t *pPayload);
static memStatus_t Load_L2CAPLePsmConnectionRequestIndication(bleEvtContainer_t *container, uint8_t *pPayload);
static memStatus_t Load_L2CAPLePsmConnectionCompleteIndication(bleEvtContainer_t *container, uint8_t *pPayload);
static memStatus_t Load_L2CAPLePsmDisconnectNotificationIndication(bleEvtContainer_t *container, uint8_t *pPayload);
static memStatus_t Load_L2CAPNoPeerCreditsIndication(bleEvtContainer_t *container, uint8_t *pPayload);
static memStatus_t Load_L2CAPLocalCreditsNotificationIndication(bleEvtContainer_t *container, uint8_t *pPayload);
static memStatus_t Load_L2CAPLeCbDataIndication(bleEvtContainer_t *container, uint8_t *pPayload);
static memStatus_t Load_GATTConfirm(bleEvtContainer_t *container, uint8_t *pPayload);
static memStatus_t Load_GATTGetMtuIndication(bleEvtContainer_t *container, uint8_t *pPayload);
static memStatus_t Load_GATTClientProcedureExchangeMtuIndication(bleEvtContainer_t *container, uint8_t *pPayload);
static memStatus_t Load_GATTClientProcedureDiscoverAllPrimaryServicesIndication(bleEvtContainer_t *container, uint8_t *pPayload);
static memStatus_t Load_GATTClientProcedureDiscoverPrimaryServicesByUuidIndication(bleEvtContainer_t *container, uint8_t *pPayload);
static memStatus_t Load_GATTClientProcedureFindIncludedServicesIndication(bleEvtContainer_t *container, uint8_t *pPayload);
static memStatus_t Load_GATTClientProcedureDiscoverAllCharacteristicsIndication(bleEvtContainer_t *container, uint8_t *pPayload);
static memStatus_t Load_GATTClientProcedureDiscoverCharacteristicByUuidIndication(bleEvtContainer_t *container, uint8_t *pPayload);
static memStatus_t Load_GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication(bleEvtContainer_t *container, uint8_t *pPayload);
static memStatus_t Load_GATTClientProcedureReadCharacteristicValueIndication(bleEvtContainer_t *container, uint8_t *pPayload);
static memStatus_t Load_GATTClientProcedureReadUsingCharacteristicUuidIndication(bleEvtContainer_t *container, uint8_t *pPayload);
static memStatus_t Load_GATTClientProcedureReadMultipleCharacteristicValuesIndication(bleEvtContainer_t *container, uint8_t *pPayload);
static memStatus_t Load_GATTClientProcedureWriteCharacteristicValueIndication(bleEvtContainer_t *container, uint8_t *pPayload);
static memStatus_t Load_GATTClientProcedureReadCharacteristicDescriptorIndication(bleEvtContainer_t *container, uint8_t *pPayload);
static memStatus_t Load_GATTClientProcedureWriteCharacteristicDescriptorIndication(bleEvtContainer_t *container, uint8_t *pPayload);
static memStatus_t Load_GATTClientNotificationIndication(bleEvtContainer_t *container, uint8_t *pPayload);
static memStatus_t Load_GATTClientIndicationIndication(bleEvtContainer_t *container, uint8_t *pPayload);
static memStatus_t Load_GATTServerMtuChangedIndication(bleEvtContainer_t *container, uint8_t *pPayload);
static memStatus_t Load_GATTServerHandleValueConfirmationIndication(bleEvtContainer_t *container, uint8_t *pPayload);
static memStatus_t Load_GATTServerAttributeWrittenIndication(bleEvtContainer_t *container, uint8_t *pPayload);
static memStatus_t Load_GATTServerCharacteristicCccdWrittenIndication(bleEvtContainer_t *container, uint8_t *pPayload);
static memStatus_t Load_GATTServerAttributeWrittenWithoutResponseIndication(bleEvtContainer_t *container, uint8_t *pPayload);
static memStatus_t Load_GATTServerErrorIndication(bleEvtContainer_t *container, uint8_t *pPayload);
static memStatus_t Load_GATTServerLongCharacteristicWrittenIndication(bleEvtContainer_t *container, uint8_t *pPayload);
static memStatus_t Load_GATTServerAttributeReadIndication(bleEvtContainer_t *container, uint8_t *pPayload);
static memStatus_t Load_GATTDBConfirm(bleEvtContainer_t *container, uint8_t *pPayload);
static memStatus_t Load_GATTDBReadAttributeIndication(bleEvtContainer_t *container, uint8_t *pPayload);
static memStatus_t Load_GATTDBFindServiceHandleIndication(bleEvtContainer_t *container, uint8_t *pPayload);
static memStatus_t Load_GATTDBFindCharValueHandleInServiceIndication(bleEvtContainer_t *container, uint8_t *pPayload);
static memStatus_t Load_GATTDBFindCccdHandleForCharValueHandleIndication(bleEvtContainer_t *container, uint8_t *pPayload);
static memStatus_t Load_GATTDBFindDescriptorHandleForCharValueHandleIndication(bleEvtContainer_t *container, uint8_t *pPayload);
static memStatus_t Load_GATTDBDynamicAddPrimaryServiceDeclarationIndication(bleEvtContainer_t *container, uint8_t *pPayload);
static memStatus_t Load_GATTDBDynamicAddSecondaryServiceDeclarationIndication(bleEvtContainer_t *container, uint8_t *pPayload);
static memStatus_t Load_GATTDBDynamicAddIncludeDeclarationIndication(bleEvtContainer_t *container, uint8_t *pPayload);
static memStatus_t Load_GATTDBDynamicAddCharacteristicDeclarationAndValueIndication(bleEvtContainer_t *container, uint8_t *pPayload);
static memStatus_t Load_GATTDBDynamicAddCharacteristicDescriptorIndication(bleEvtContainer_t *container, uint8_t *pPayload);
static memStatus_t Load_GATTDBDynamicAddCccdIndication(bleEvtContainer_t *container, uint8_t *pPayload);
static memStatus_t Load_GATTDBDynamicAddCharacteristicDeclarationWithUniqueValueIndication(bleEvtContainer_t *container, uint8_t *pPayload);
static memStatus_t Load_GAPConfirm(bleEvtContainer_t *container, uint8_t *pPayload);
static memStatus_t Load_GAPCheckNotificationStatusIndication(bleEvtContainer_t *container, uint8_t *pPayload);
static memStatus_t Load_GAPCheckIndicationStatusIndication(bleEvtContainer_t *container, uint8_t *pPayload);
static memStatus_t Load_GAPGetBondedStaticAddressesIndication(bleEvtContainer_t *container, uint8_t *pPayload);
static memStatus_t Load_GAPLoadEncryptionInformationIndication(bleEvtContainer_t *container, uint8_t *pPayload);
static memStatus_t Load_GAPLoadCustomPeerInformationIndication(bleEvtContainer_t *container, uint8_t *pPayload);
static memStatus_t Load_GAPCheckIfBondedIndication(bleEvtContainer_t *container, uint8_t *pPayload);
static memStatus_t Load_GAPGetBondedDevicesCountIndication(bleEvtContainer_t *container, uint8_t *pPayload);
static memStatus_t Load_GAPGetBondedDeviceNameIndication(bleEvtContainer_t *container, uint8_t *pPayload);
static memStatus_t Load_GAPGenericEventInitializationCompleteIndication(bleEvtContainer_t *container, uint8_t *pPayload);
static memStatus_t Load_GAPGenericEventInternalErrorIndication(bleEvtContainer_t *container, uint8_t *pPayload);
static memStatus_t Load_GAPGenericEventAdvertisingSetupFailedIndication(bleEvtContainer_t *container, uint8_t *pPayload);
static memStatus_t Load_GAPGenericEventAdvertisingParametersSetupCompleteIndication(bleEvtContainer_t *container, uint8_t *pPayload);
static memStatus_t Load_GAPGenericEventAdvertisingDataSetupCompleteIndication(bleEvtContainer_t *container, uint8_t *pPayload);
static memStatus_t Load_GAPGenericEventWhiteListSizeReadIndication(bleEvtContainer_t *container, uint8_t *pPayload);
static memStatus_t Load_GAPGenericEventDeviceAddedToWhiteListIndication(bleEvtContainer_t *container, uint8_t *pPayload);
static memStatus_t Load_GAPGenericEventDeviceRemovedFromWhiteListIndication(bleEvtContainer_t *container, uint8_t *pPayload);
static memStatus_t Load_GAPGenericEventWhiteListClearedIndication(bleEvtContainer_t *container, uint8_t *pPayload);
static memStatus_t Load_GAPGenericEventRandomAddressReadyIndication(bleEvtContainer_t *container, uint8_t *pPayload);
static memStatus_t Load_GAPGenericEventCreateConnectionCanceledIndication(bleEvtContainer_t *container, uint8_t *pPayload);
static memStatus_t Load_GAPGenericEventPublicAddressReadIndication(bleEvtContainer_t *container, uint8_t *pPayload);
static memStatus_t Load_GAPGenericEventAdvTxPowerLevelReadIndication(bleEvtContainer_t *container, uint8_t *pPayload);
static memStatus_t Load_GAPGenericEventPrivateResolvableAddressVerifiedIndication(bleEvtContainer_t *container, uint8_t *pPayload);
static memStatus_t Load_GAPGenericEventRandomAddressSetIndication(bleEvtContainer_t *container, uint8_t *pPayload);
static memStatus_t Load_GAPAdvertisingEventStateChangedIndication(bleEvtContainer_t *container, uint8_t *pPayload);
static memStatus_t Load_GAPAdvertisingEventCommandFailedIndication(bleEvtContainer_t *container, uint8_t *pPayload);
static memStatus_t Load_GAPScanningEventStateChangedIndication(bleEvtContainer_t *container, uint8_t *pPayload);
static memStatus_t Load_GAPScanningEventCommandFailedIndication(bleEvtContainer_t *container, uint8_t *pPayload);
static memStatus_t Load_GAPScanningEventDeviceScannedIndication(bleEvtContainer_t *container, uint8_t *pPayload);
static memStatus_t Load_GAPConnectionEventConnectedIndication(bleEvtContainer_t *container, uint8_t *pPayload);
static memStatus_t Load_GAPConnectionEventPairingRequestIndication(bleEvtContainer_t *container, uint8_t *pPayload);
static memStatus_t Load_GAPConnectionEventSlaveSecurityRequestIndication(bleEvtContainer_t *container, uint8_t *pPayload);
static memStatus_t Load_GAPConnectionEventPairingResponseIndication(bleEvtContainer_t *container, uint8_t *pPayload);
static memStatus_t Load_GAPConnectionEventAuthenticationRejectedIndication(bleEvtContainer_t *container, uint8_t *pPayload);
static memStatus_t Load_GAPConnectionEventPasskeyRequestIndication(bleEvtContainer_t *container, uint8_t *pPayload);
static memStatus_t Load_GAPConnectionEventOobRequestIndication(bleEvtContainer_t *container, uint8_t *pPayload);
static memStatus_t Load_GAPConnectionEventPasskeyDisplayIndication(bleEvtContainer_t *container, uint8_t *pPayload);
static memStatus_t Load_GAPConnectionEventKeyExchangeRequestIndication(bleEvtContainer_t *container, uint8_t *pPayload);
static memStatus_t Load_GAPConnectionEventKeysReceivedIndication(bleEvtContainer_t *container, uint8_t *pPayload);
static memStatus_t Load_GAPConnectionEventLongTermKeyRequestIndication(bleEvtContainer_t *container, uint8_t *pPayload);
static memStatus_t Load_GAPConnectionEventEncryptionChangedIndication(bleEvtContainer_t *container, uint8_t *pPayload);
static memStatus_t Load_GAPConnectionEventPairingCompleteIndication(bleEvtContainer_t *container, uint8_t *pPayload);
static memStatus_t Load_GAPConnectionEventDisconnectedIndication(bleEvtContainer_t *container, uint8_t *pPayload);
static memStatus_t Load_GAPConnectionEventRssiReadIndication(bleEvtContainer_t *container, uint8_t *pPayload);
static memStatus_t Load_GAPConnectionEventTxPowerLevelReadIndication(bleEvtContainer_t *container, uint8_t *pPayload);
static memStatus_t Load_GAPConnectionEventPowerReadFailureIndication(bleEvtContainer_t *container, uint8_t *pPayload);
static memStatus_t Load_GAPConnectionEventParameterUpdateRequestIndication(bleEvtContainer_t *container, uint8_t *pPayload);
static memStatus_t Load_GAPConnectionEventParameterUpdateCompleteIndication(bleEvtContainer_t *container, uint8_t *pPayload);
static memStatus_t Load_GAPConnectionEventLeDataLengthChangedIndication(bleEvtContainer_t *container, uint8_t *pPayload);
static memStatus_t Load_GAPConnectionEventLeScOobDataRequestIndication(bleEvtContainer_t *container, uint8_t *pPayload);
static memStatus_t Load_GAPConnectionEventLeScDisplayNumericValueIndication(bleEvtContainer_t *container, uint8_t *pPayload);
static memStatus_t Load_GAPConnectionEventLeScKeypressNotificationIndication(bleEvtContainer_t *container, uint8_t *pPayload);
static memStatus_t Load_GAPGenericEventControllerResetCompleteIndication(bleEvtContainer_t *container, uint8_t *pPayload);
static memStatus_t Load_GAPLeScPublicKeyRegeneratedIndication(bleEvtContainer_t *container, uint8_t *pPayload);
static memStatus_t Load_GAPGenericEventLeScLocalOobDataIndication(bleEvtContainer_t *container, uint8_t *pPayload);
static memStatus_t Load_GAPGenericEventControllerPrivacyStateChangedIndication(bleEvtContainer_t *container, uint8_t *pPayload);
static memStatus_t Load_GAPGetBondedDevicesIdentityInformationIndication(bleEvtContainer_t *container, uint8_t *pPayload);

/*==================================================================================================
Private global variables declarations
==================================================================================================*/
static const bleEvtHandler_t evtHandlerTbl[] =
{
	{0xA4FE, Load_FSCIErrorIndication},
	{0xA470, Load_FSCIAllowDeviceToSleepConfirm},
	{0xA471, Load_FSCIWakeUpIndication},
	{0xA472, Load_FSCIGetWakeupReasonResponse},
	{0x4180, Load_L2CAPConfirm},
	{0x4183, Load_L2CAPLePsmConnectionRequestIndication},
	{0x4184, Load_L2CAPLePsmConnectionCompleteIndication},
	{0x4185, Load_L2CAPLePsmDisconnectNotificationIndication},
	{0x4186, Load_L2CAPNoPeerCreditsIndication},
	{0x4187, Load_L2CAPLocalCreditsNotificationIndication},
	{0x4188, Load_L2CAPLeCbDataIndication},
	{0x4480, Load_GATTConfirm},
	{0x4481, Load_GATTGetMtuIndication},
	{0x4482, Load_GATTClientProcedureExchangeMtuIndication},
	{0x4483, Load_GATTClientProcedureDiscoverAllPrimaryServicesIndication},
	{0x4484, Load_GATTClientProcedureDiscoverPrimaryServicesByUuidIndication},
	{0x4485, Load_GATTClientProcedureFindIncludedServicesIndication},
	{0x4486, Load_GATTClientProcedureDiscoverAllCharacteristicsIndication},
	{0x4487, Load_GATTClientProcedureDiscoverCharacteristicByUuidIndication},
	{0x4488, Load_GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication},
	{0x4489, Load_GATTClientProcedureReadCharacteristicValueIndication},
	{0x448A, Load_GATTClientProcedureReadUsingCharacteristicUuidIndication},
	{0x448B, Load_GATTClientProcedureReadMultipleCharacteristicValuesIndication},
	{0x448C, Load_GATTClientProcedureWriteCharacteristicValueIndication},
	{0x448D, Load_GATTClientProcedureReadCharacteristicDescriptorIndication},
	{0x448E, Load_GATTClientProcedureWriteCharacteristicDescriptorIndication},
	{0x448F, Load_GATTClientNotificationIndication},
	{0x4490, Load_GATTClientIndicationIndication},
	{0x4491, Load_GATTServerMtuChangedIndication},
	{0x4492, Load_GATTServerHandleValueConfirmationIndication},
	{0x4493, Load_GATTServerAttributeWrittenIndication},
	{0x4494, Load_GATTServerCharacteristicCccdWrittenIndication},
	{0x4495, Load_GATTServerAttributeWrittenWithoutResponseIndication},
	{0x4496, Load_GATTServerErrorIndication},
	{0x4497, Load_GATTServerLongCharacteristicWrittenIndication},
	{0x4498, Load_GATTServerAttributeReadIndication},
	{0x4580, Load_GATTDBConfirm},
	{0x4581, Load_GATTDBReadAttributeIndication},
	{0x4582, Load_GATTDBFindServiceHandleIndication},
	{0x4583, Load_GATTDBFindCharValueHandleInServiceIndication},
	{0x4584, Load_GATTDBFindCccdHandleForCharValueHandleIndication},
	{0x4585, Load_GATTDBFindDescriptorHandleForCharValueHandleIndication},
	{0x4586, Load_GATTDBDynamicAddPrimaryServiceDeclarationIndication},
	{0x4587, Load_GATTDBDynamicAddSecondaryServiceDeclarationIndication},
	{0x4588, Load_GATTDBDynamicAddIncludeDeclarationIndication},
	{0x4589, Load_GATTDBDynamicAddCharacteristicDeclarationAndValueIndication},
	{0x458A, Load_GATTDBDynamicAddCharacteristicDescriptorIndication},
	{0x458B, Load_GATTDBDynamicAddCccdIndication},
	{0x458C, Load_GATTDBDynamicAddCharacteristicDeclarationWithUniqueValueIndication},
	{0x4780, Load_GAPConfirm},
	{0x4781, Load_GAPCheckNotificationStatusIndication},
	{0x4782, Load_GAPCheckIndicationStatusIndication},
	{0x4783, Load_GAPGetBondedStaticAddressesIndication},
	{0x4784, Load_GAPLoadEncryptionInformationIndication},
	{0x4785, Load_GAPLoadCustomPeerInformationIndication},
	{0x4786, Load_GAPCheckIfBondedIndication},
	{0x4787, Load_GAPGetBondedDevicesCountIndication},
	{0x4788, Load_GAPGetBondedDeviceNameIndication},
	{0x4789, Load_GAPGenericEventInitializationCompleteIndication},
	{0x478A, Load_GAPGenericEventInternalErrorIndication},
	{0x478B, Load_GAPGenericEventAdvertisingSetupFailedIndication},
	{0x478C, Load_GAPGenericEventAdvertisingParametersSetupCompleteIndication},
	{0x478D, Load_GAPGenericEventAdvertisingDataSetupCompleteIndication},
	{0x478E, Load_GAPGenericEventWhiteListSizeReadIndication},
	{0x478F, Load_GAPGenericEventDeviceAddedToWhiteListIndication},
	{0x4790, Load_GAPGenericEventDeviceRemovedFromWhiteListIndication},
	{0x4791, Load_GAPGenericEventWhiteListClearedIndication},
	{0x4792, Load_GAPGenericEventRandomAddressReadyIndication},
	{0x4793, Load_GAPGenericEventCreateConnectionCanceledIndication},
	{0x4794, Load_GAPGenericEventPublicAddressReadIndication},
	{0x4795, Load_GAPGenericEventAdvTxPowerLevelReadIndication},
	{0x4796, Load_GAPGenericEventPrivateResolvableAddressVerifiedIndication},
	{0x4797, Load_GAPGenericEventRandomAddressSetIndication},
	{0x4798, Load_GAPAdvertisingEventStateChangedIndication},
	{0x4799, Load_GAPAdvertisingEventCommandFailedIndication},
	{0x479A, Load_GAPScanningEventStateChangedIndication},
	{0x479B, Load_GAPScanningEventCommandFailedIndication},
	{0x479C, Load_GAPScanningEventDeviceScannedIndication},
	{0x479D, Load_GAPConnectionEventConnectedIndication},
	{0x479E, Load_GAPConnectionEventPairingRequestIndication},
	{0x479F, Load_GAPConnectionEventSlaveSecurityRequestIndication},
	{0x47A0, Load_GAPConnectionEventPairingResponseIndication},
	{0x47A1, Load_GAPConnectionEventAuthenticationRejectedIndication},
	{0x47A2, Load_GAPConnectionEventPasskeyRequestIndication},
	{0x47A3, Load_GAPConnectionEventOobRequestIndication},
	{0x47A4, Load_GAPConnectionEventPasskeyDisplayIndication},
	{0x47A5, Load_GAPConnectionEventKeyExchangeRequestIndication},
	{0x47A6, Load_GAPConnectionEventKeysReceivedIndication},
	{0x47A7, Load_GAPConnectionEventLongTermKeyRequestIndication},
	{0x47A8, Load_GAPConnectionEventEncryptionChangedIndication},
	{0x47A9, Load_GAPConnectionEventPairingCompleteIndication},
	{0x47AA, Load_GAPConnectionEventDisconnectedIndication},
	{0x47AB, Load_GAPConnectionEventRssiReadIndication},
	{0x47AC, Load_GAPConnectionEventTxPowerLevelReadIndication},
	{0x47AD, Load_GAPConnectionEventPowerReadFailureIndication},
	{0x47AE, Load_GAPConnectionEventParameterUpdateRequestIndication},
	{0x47AF, Load_GAPConnectionEventParameterUpdateCompleteIndication},
	{0x47B0, Load_GAPConnectionEventLeDataLengthChangedIndication},
	{0x47B1, Load_GAPConnectionEventLeScOobDataRequestIndication},
	{0x47B2, Load_GAPConnectionEventLeScDisplayNumericValueIndication},
	{0x47B3, Load_GAPConnectionEventLeScKeypressNotificationIndication},
	{0x47B4, Load_GAPGenericEventControllerResetCompleteIndication},
	{0x47B5, Load_GAPLeScPublicKeyRegeneratedIndication},
	{0x47B6, Load_GAPGenericEventLeScLocalOobDataIndication},
	{0x47B7, Load_GAPGenericEventControllerPrivacyStateChangedIndication},
	{0x4783, Load_GAPGetBondedDevicesIdentityInformationIndication},
};

/*==================================================================================================
Public Functions
==================================================================================================*/
/*!*************************************************************************************************
\fn		static memStatus_t Load_FSCIErrorIndication(bleEvtContainer_t *container, uint8_t *pPayload)
\brief	FSCI is reporting an error condition.
***************************************************************************************************/
static memStatus_t Load_FSCIErrorIndication(bleEvtContainer_t *container, uint8_t *pPayload)
{
	FSCIErrorIndication_t *evt = &(container->Data.FSCIErrorIndication);

	/* Store (OG, OC) in ID */
	container->id = 0xA4FE;

	FLib_MemCpy(evt, pPayload, sizeof(FSCIErrorIndication_t));

	return MEM_SUCCESS_c;
}

/*!*************************************************************************************************
\fn		static memStatus_t Load_FSCIAllowDeviceToSleepConfirm(bleEvtContainer_t *container, uint8_t *pPayload)
\brief	FSCI-AllowDeviceToSleep.Confirm description
***************************************************************************************************/
static memStatus_t Load_FSCIAllowDeviceToSleepConfirm(bleEvtContainer_t *container, uint8_t *pPayload)
{
	FSCIAllowDeviceToSleepConfirm_t *evt = &(container->Data.FSCIAllowDeviceToSleepConfirm);

	/* Store (OG, OC) in ID */
	container->id = 0xA470;

	FLib_MemCpy(evt, pPayload, sizeof(FSCIAllowDeviceToSleepConfirm_t));

	return MEM_SUCCESS_c;
}

/*!*************************************************************************************************
\fn		static memStatus_t Load_FSCIWakeUpIndication(bleEvtContainer_t *container, uint8_t *pPayload)
\brief	FSCI-WakeUp.Indication description
***************************************************************************************************/
static memStatus_t Load_FSCIWakeUpIndication(bleEvtContainer_t *container, uint8_t *pPayload)
{
	FSCIWakeUpIndication_t *evt = &(container->Data.FSCIWakeUpIndication);

	/* Store (OG, OC) in ID */
	container->id = 0xA471;

	FLib_MemCpy(evt, pPayload, sizeof(FSCIWakeUpIndication_t));

	return MEM_SUCCESS_c;
}

/*!*************************************************************************************************
\fn		static memStatus_t Load_FSCIGetWakeupReasonResponse(bleEvtContainer_t *container, uint8_t *pPayload)
\brief	FSCI-GetWakeupReason.Response description
***************************************************************************************************/
static memStatus_t Load_FSCIGetWakeupReasonResponse(bleEvtContainer_t *container, uint8_t *pPayload)
{
	FSCIGetWakeupReasonResponse_t *evt = &(container->Data.FSCIGetWakeupReasonResponse);

	/* Store (OG, OC) in ID */
	container->id = 0xA472;

	FLib_MemCpy(evt, pPayload, sizeof(FSCIGetWakeupReasonResponse_t));

	return MEM_SUCCESS_c;
}

/*!*************************************************************************************************
\fn		static memStatus_t Load_L2CAPConfirm(bleEvtContainer_t *container, uint8_t *pPayload)
\brief	Status of the L2CAP request
***************************************************************************************************/
static memStatus_t Load_L2CAPConfirm(bleEvtContainer_t *container, uint8_t *pPayload)
{
	L2CAPConfirm_t *evt = &(container->Data.L2CAPConfirm);

	/* Store (OG, OC) in ID */
	container->id = 0x4180;

	FLib_MemCpy(evt, pPayload, sizeof(L2CAPConfirm_t));

	return MEM_SUCCESS_c;
}

/*!*************************************************************************************************
\fn		static memStatus_t Load_L2CAPLePsmConnectionRequestIndication(bleEvtContainer_t *container, uint8_t *pPayload)
\brief	Credit Based - Connection request event
***************************************************************************************************/
static memStatus_t Load_L2CAPLePsmConnectionRequestIndication(bleEvtContainer_t *container, uint8_t *pPayload)
{
	L2CAPLePsmConnectionRequestIndication_t *evt = &(container->Data.L2CAPLePsmConnectionRequestIndication);

	uint32_t idx = 0;

	/* Store (OG, OC) in ID */
	container->id = 0x4183;

	evt->InformationIncluded = (bool_t)pPayload[idx]; idx++;

	if (evt->InformationIncluded)
	{
		evt->LeCbConnectionRequest.DeviceId = pPayload[idx]; idx++;
		FLib_MemCpy(&(evt->LeCbConnectionRequest.LePsm), pPayload + idx, sizeof(evt->LeCbConnectionRequest.LePsm)); idx += sizeof(evt->LeCbConnectionRequest.LePsm);
		FLib_MemCpy(&(evt->LeCbConnectionRequest.PeerMtu), pPayload + idx, sizeof(evt->LeCbConnectionRequest.PeerMtu)); idx += sizeof(evt->LeCbConnectionRequest.PeerMtu);
		FLib_MemCpy(&(evt->LeCbConnectionRequest.PeerMps), pPayload + idx, sizeof(evt->LeCbConnectionRequest.PeerMps)); idx += sizeof(evt->LeCbConnectionRequest.PeerMps);
		FLib_MemCpy(&(evt->LeCbConnectionRequest.InitialCredits), pPayload + idx, sizeof(evt->LeCbConnectionRequest.InitialCredits)); idx += sizeof(evt->LeCbConnectionRequest.InitialCredits);
	}


	return MEM_SUCCESS_c;
}

/*!*************************************************************************************************
\fn		static memStatus_t Load_L2CAPLePsmConnectionCompleteIndication(bleEvtContainer_t *container, uint8_t *pPayload)
\brief	Credit Based - Connection complete event
***************************************************************************************************/
static memStatus_t Load_L2CAPLePsmConnectionCompleteIndication(bleEvtContainer_t *container, uint8_t *pPayload)
{
	L2CAPLePsmConnectionCompleteIndication_t *evt = &(container->Data.L2CAPLePsmConnectionCompleteIndication);

	uint32_t idx = 0;

	/* Store (OG, OC) in ID */
	container->id = 0x4184;

	evt->InformationIncluded = (bool_t)pPayload[idx]; idx++;

	if (evt->InformationIncluded)
	{
		evt->LeCbConnectionComplete.DeviceId = pPayload[idx]; idx++;
		FLib_MemCpy(&(evt->LeCbConnectionComplete.ChannelId), pPayload + idx, sizeof(evt->LeCbConnectionComplete.ChannelId)); idx += sizeof(evt->LeCbConnectionComplete.ChannelId);
		FLib_MemCpy(&(evt->LeCbConnectionComplete.PeerMtu), pPayload + idx, sizeof(evt->LeCbConnectionComplete.PeerMtu)); idx += sizeof(evt->LeCbConnectionComplete.PeerMtu);
		FLib_MemCpy(&(evt->LeCbConnectionComplete.PeerMps), pPayload + idx, sizeof(evt->LeCbConnectionComplete.PeerMps)); idx += sizeof(evt->LeCbConnectionComplete.PeerMps);
		FLib_MemCpy(&(evt->LeCbConnectionComplete.InitialCredits), pPayload + idx, sizeof(evt->LeCbConnectionComplete.InitialCredits)); idx += sizeof(evt->LeCbConnectionComplete.InitialCredits);
		FLib_MemCpy(&(evt->LeCbConnectionComplete.Result), pPayload + idx, 2); idx += 2;
	}


	return MEM_SUCCESS_c;
}

/*!*************************************************************************************************
\fn		static memStatus_t Load_L2CAPLePsmDisconnectNotificationIndication(bleEvtContainer_t *container, uint8_t *pPayload)
\brief	Credit Based - Disconnection notification event
***************************************************************************************************/
static memStatus_t Load_L2CAPLePsmDisconnectNotificationIndication(bleEvtContainer_t *container, uint8_t *pPayload)
{
	L2CAPLePsmDisconnectNotificationIndication_t *evt = &(container->Data.L2CAPLePsmDisconnectNotificationIndication);

	uint32_t idx = 0;

	/* Store (OG, OC) in ID */
	container->id = 0x4185;

	evt->InformationIncluded = (bool_t)pPayload[idx]; idx++;

	if (evt->InformationIncluded)
	{
		evt->LeCbDisconnection.DeviceId = pPayload[idx]; idx++;
		FLib_MemCpy(&(evt->LeCbDisconnection.ChannelId), pPayload + idx, sizeof(evt->LeCbDisconnection.ChannelId)); idx += sizeof(evt->LeCbDisconnection.ChannelId);
	}


	return MEM_SUCCESS_c;
}

/*!*************************************************************************************************
\fn		static memStatus_t Load_L2CAPNoPeerCreditsIndication(bleEvtContainer_t *container, uint8_t *pPayload)
\brief	Credit Based - No peer credits event
***************************************************************************************************/
static memStatus_t Load_L2CAPNoPeerCreditsIndication(bleEvtContainer_t *container, uint8_t *pPayload)
{
	L2CAPNoPeerCreditsIndication_t *evt = &(container->Data.L2CAPNoPeerCreditsIndication);

	uint32_t idx = 0;

	/* Store (OG, OC) in ID */
	container->id = 0x4186;

	evt->InformationIncluded = (bool_t)pPayload[idx]; idx++;

	if (evt->InformationIncluded)
	{
		evt->LeCbNoPeerCredits.DeviceId = pPayload[idx]; idx++;
		FLib_MemCpy(&(evt->LeCbNoPeerCredits.ChannelId), pPayload + idx, sizeof(evt->LeCbNoPeerCredits.ChannelId)); idx += sizeof(evt->LeCbNoPeerCredits.ChannelId);
	}


	return MEM_SUCCESS_c;
}

/*!*************************************************************************************************
\fn		static memStatus_t Load_L2CAPLocalCreditsNotificationIndication(bleEvtContainer_t *container, uint8_t *pPayload)
\brief	Credit Based - Local credits notification event
***************************************************************************************************/
static memStatus_t Load_L2CAPLocalCreditsNotificationIndication(bleEvtContainer_t *container, uint8_t *pPayload)
{
	L2CAPLocalCreditsNotificationIndication_t *evt = &(container->Data.L2CAPLocalCreditsNotificationIndication);

	uint32_t idx = 0;

	/* Store (OG, OC) in ID */
	container->id = 0x4187;

	evt->InformationIncluded = (bool_t)pPayload[idx]; idx++;

	if (evt->InformationIncluded)
	{
		evt->LeCbLocalCreditsNotification.DeviceId = pPayload[idx]; idx++;
		FLib_MemCpy(&(evt->LeCbLocalCreditsNotification.ChannelId), pPayload + idx, sizeof(evt->LeCbLocalCreditsNotification.ChannelId)); idx += sizeof(evt->LeCbLocalCreditsNotification.ChannelId);
		FLib_MemCpy(&(evt->LeCbLocalCreditsNotification.LocalCredits), pPayload + idx, sizeof(evt->LeCbLocalCreditsNotification.LocalCredits)); idx += sizeof(evt->LeCbLocalCreditsNotification.LocalCredits);
	}


	return MEM_SUCCESS_c;
}

/*!*************************************************************************************************
\fn		static memStatus_t Load_L2CAPLeCbDataIndication(bleEvtContainer_t *container, uint8_t *pPayload)
\brief	Data packet received through Credit Based Channel
***************************************************************************************************/
static memStatus_t Load_L2CAPLeCbDataIndication(bleEvtContainer_t *container, uint8_t *pPayload)
{
	L2CAPLeCbDataIndication_t *evt = &(container->Data.L2CAPLeCbDataIndication);

	uint32_t idx = 0;

	/* Store (OG, OC) in ID */
	container->id = 0x4188;

	evt->DeviceId = pPayload[idx]; idx++;
	FLib_MemCpy(&(evt->SrcCid), pPayload + idx, sizeof(evt->SrcCid)); idx += sizeof(evt->SrcCid);
	FLib_MemCpy(&(evt->PacketLength), pPayload + idx, sizeof(evt->PacketLength)); idx += sizeof(evt->PacketLength);

	if (evt->PacketLength > 0)
	{
		evt->Packet = MEM_BufferAlloc(evt->PacketLength);

		if (!evt->Packet)
		{
			return MEM_ALLOC_ERROR_c;
		}

	}
	else
	{
		evt->Packet = NULL;
	}

	FLib_MemCpy(evt->Packet, pPayload + idx, evt->PacketLength); idx += evt->PacketLength;

	return MEM_SUCCESS_c;
}

/*!*************************************************************************************************
\fn		static memStatus_t Load_GATTConfirm(bleEvtContainer_t *container, uint8_t *pPayload)
\brief	Status of the GATT request
***************************************************************************************************/
static memStatus_t Load_GATTConfirm(bleEvtContainer_t *container, uint8_t *pPayload)
{
	GATTConfirm_t *evt = &(container->Data.GATTConfirm);

	/* Store (OG, OC) in ID */
	container->id = 0x4480;

	FLib_MemCpy(evt, pPayload, sizeof(GATTConfirm_t));

	return MEM_SUCCESS_c;
}

/*!*************************************************************************************************
\fn		static memStatus_t Load_GATTGetMtuIndication(bleEvtContainer_t *container, uint8_t *pPayload)
\brief	Specifies the MTU used with a given connected device
***************************************************************************************************/
static memStatus_t Load_GATTGetMtuIndication(bleEvtContainer_t *container, uint8_t *pPayload)
{
	GATTGetMtuIndication_t *evt = &(container->Data.GATTGetMtuIndication);

	/* Store (OG, OC) in ID */
	container->id = 0x4481;

	FLib_MemCpy(evt, pPayload, sizeof(GATTGetMtuIndication_t));

	return MEM_SUCCESS_c;
}

/*!*************************************************************************************************
\fn		static memStatus_t Load_GATTClientProcedureExchangeMtuIndication(bleEvtContainer_t *container, uint8_t *pPayload)
\brief	Specifies that the MTU exchange procedure ended
***************************************************************************************************/
static memStatus_t Load_GATTClientProcedureExchangeMtuIndication(bleEvtContainer_t *container, uint8_t *pPayload)
{
	GATTClientProcedureExchangeMtuIndication_t *evt = &(container->Data.GATTClientProcedureExchangeMtuIndication);

	/* Store (OG, OC) in ID */
	container->id = 0x4482;

	FLib_MemCpy(evt, pPayload, sizeof(GATTClientProcedureExchangeMtuIndication_t));

	return MEM_SUCCESS_c;
}

/*!*************************************************************************************************
\fn		static memStatus_t Load_GATTClientProcedureDiscoverAllPrimaryServicesIndication(bleEvtContainer_t *container, uint8_t *pPayload)
\brief	Specifies that the Primary Service Discovery procedure ended
***************************************************************************************************/
static memStatus_t Load_GATTClientProcedureDiscoverAllPrimaryServicesIndication(bleEvtContainer_t *container, uint8_t *pPayload)
{
	GATTClientProcedureDiscoverAllPrimaryServicesIndication_t *evt = &(container->Data.GATTClientProcedureDiscoverAllPrimaryServicesIndication);

	uint32_t idx = 0;

	/* Store (OG, OC) in ID */
	container->id = 0x4483;

	evt->DeviceId = pPayload[idx]; idx++;
	evt->ProcedureResult = (GATTClientProcedureDiscoverAllPrimaryServicesIndication_ProcedureResult_t)pPayload[idx]; idx++;
	FLib_MemCpy(&(evt->Error), pPayload + idx, 2); idx += 2;
	evt->NbOfDiscoveredServices = pPayload[idx]; idx++;

	if (evt->NbOfDiscoveredServices > 0)
	{
		evt->DiscoveredServices = MEM_BufferAlloc(evt->NbOfDiscoveredServices * sizeof(evt->DiscoveredServices[0]));

		if (!evt->DiscoveredServices)
		{
			return MEM_ALLOC_ERROR_c;
		}

	}
	else
	{
		evt->DiscoveredServices = NULL;
	}


	for (uint32_t i = 0; i < evt->NbOfDiscoveredServices; i++)
	{
		FLib_MemCpy(&(evt->DiscoveredServices[i].StartHandle), pPayload + idx, sizeof(evt->DiscoveredServices[i].StartHandle)); idx += sizeof(evt->DiscoveredServices[i].StartHandle);
		FLib_MemCpy(&(evt->DiscoveredServices[i].EndHandle), pPayload + idx, sizeof(evt->DiscoveredServices[i].EndHandle)); idx += sizeof(evt->DiscoveredServices[i].EndHandle);
		evt->DiscoveredServices[i].UuidType = (UuidType_t)pPayload[idx]; idx++;

		switch (evt->DiscoveredServices[i].UuidType)
		{
			case Uuid16Bits:
				FLib_MemCpy(evt->DiscoveredServices[i].Uuid.Uuid16Bits, pPayload + idx, 2); idx += 2;
				break;

			case Uuid128Bits:
				FLib_MemCpy(evt->DiscoveredServices[i].Uuid.Uuid128Bits, pPayload + idx, 16); idx += 16;
				break;

			case Uuid32Bits:
				FLib_MemCpy(evt->DiscoveredServices[i].Uuid.Uuid32Bits, pPayload + idx, 4); idx += 4;
				break;
		}
		evt->DiscoveredServices[i].NbOfCharacteristics = pPayload[idx]; idx++;

		if (evt->DiscoveredServices[i].NbOfCharacteristics > 0)
		{
			evt->DiscoveredServices[i].Characteristics = MEM_BufferAlloc(evt->DiscoveredServices[i].NbOfCharacteristics * sizeof(evt->DiscoveredServices[i].Characteristics[0]));

			if (!evt->DiscoveredServices[i].Characteristics)
			{
				MEM_BufferFree(evt->DiscoveredServices);
				return MEM_ALLOC_ERROR_c;
			}

		}
		else
		{
			evt->DiscoveredServices[i].Characteristics = NULL;
		}


		for (uint32_t j = 0; j < evt->DiscoveredServices[i].NbOfCharacteristics; j++)
		{
			evt->DiscoveredServices[i].Characteristics[j].Properties = (Properties_t)pPayload[idx]; idx++;
			FLib_MemCpy(&(evt->DiscoveredServices[i].Characteristics[j].Value.Handle), pPayload + idx, sizeof(evt->DiscoveredServices[i].Characteristics[j].Value.Handle)); idx += sizeof(evt->DiscoveredServices[i].Characteristics[j].Value.Handle);
			evt->DiscoveredServices[i].Characteristics[j].Value.UuidType = (UuidType_t)pPayload[idx]; idx++;

			switch (evt->DiscoveredServices[i].Characteristics[j].Value.UuidType)
			{
				case Uuid16Bits:
					FLib_MemCpy(evt->DiscoveredServices[i].Characteristics[j].Value.Uuid.Uuid16Bits, pPayload + idx, 2); idx += 2;
					break;

				case Uuid128Bits:
					FLib_MemCpy(evt->DiscoveredServices[i].Characteristics[j].Value.Uuid.Uuid128Bits, pPayload + idx, 16); idx += 16;
					break;

				case Uuid32Bits:
					FLib_MemCpy(evt->DiscoveredServices[i].Characteristics[j].Value.Uuid.Uuid32Bits, pPayload + idx, 4); idx += 4;
					break;
			}
			FLib_MemCpy(&(evt->DiscoveredServices[i].Characteristics[j].Value.ValueLength), pPayload + idx, sizeof(evt->DiscoveredServices[i].Characteristics[j].Value.ValueLength)); idx += sizeof(evt->DiscoveredServices[i].Characteristics[j].Value.ValueLength);
			FLib_MemCpy(&(evt->DiscoveredServices[i].Characteristics[j].Value.MaxValueLength), pPayload + idx, sizeof(evt->DiscoveredServices[i].Characteristics[j].Value.MaxValueLength)); idx += sizeof(evt->DiscoveredServices[i].Characteristics[j].Value.MaxValueLength);

			if (evt->DiscoveredServices[i].Characteristics[j].Value.ValueLength > 0)
			{
				evt->DiscoveredServices[i].Characteristics[j].Value.Value = MEM_BufferAlloc(evt->DiscoveredServices[i].Characteristics[j].Value.ValueLength);

				if (!evt->DiscoveredServices[i].Characteristics[j].Value.Value)
				{
					for (uint32_t i = 0; i < evt->NbOfDiscoveredServices; i++)
					{
						MEM_BufferFree(evt->DiscoveredServices[i].Characteristics);
					}
					MEM_BufferFree(evt->DiscoveredServices);
					return MEM_ALLOC_ERROR_c;
				}

			}
			else
			{
				evt->DiscoveredServices[i].Characteristics[j].Value.Value = NULL;
			}

			FLib_MemCpy(evt->DiscoveredServices[i].Characteristics[j].Value.Value, pPayload + idx, evt->DiscoveredServices[i].Characteristics[j].Value.ValueLength); idx += evt->DiscoveredServices[i].Characteristics[j].Value.ValueLength;
			evt->DiscoveredServices[i].Characteristics[j].NbOfDescriptors = pPayload[idx]; idx++;

			if (evt->DiscoveredServices[i].Characteristics[j].NbOfDescriptors > 0)
			{
				evt->DiscoveredServices[i].Characteristics[j].Descriptors = MEM_BufferAlloc(evt->DiscoveredServices[i].Characteristics[j].NbOfDescriptors * sizeof(evt->DiscoveredServices[i].Characteristics[j].Descriptors[0]));

				if (!evt->DiscoveredServices[i].Characteristics[j].Descriptors)
				{
					for (uint32_t i = 0; i < evt->NbOfDiscoveredServices; i++)
					{
						for (uint32_t j = 0; j < evt->DiscoveredServices[i].NbOfCharacteristics; j++)
						{
							MEM_BufferFree(evt->DiscoveredServices[i].Characteristics[j].Value.Value);
						}
					}
					for (uint32_t i = 0; i < evt->NbOfDiscoveredServices; i++)
					{
						MEM_BufferFree(evt->DiscoveredServices[i].Characteristics);
					}
					MEM_BufferFree(evt->DiscoveredServices);
					return MEM_ALLOC_ERROR_c;
				}

			}
			else
			{
				evt->DiscoveredServices[i].Characteristics[j].Descriptors = NULL;
			}


			for (uint32_t k = 0; k < evt->DiscoveredServices[i].Characteristics[j].NbOfDescriptors; k++)
			{
				FLib_MemCpy(&(evt->DiscoveredServices[i].Characteristics[j].Descriptors[k].Handle), pPayload + idx, sizeof(evt->DiscoveredServices[i].Characteristics[j].Descriptors[k].Handle)); idx += sizeof(evt->DiscoveredServices[i].Characteristics[j].Descriptors[k].Handle);
				evt->DiscoveredServices[i].Characteristics[j].Descriptors[k].UuidType = (UuidType_t)pPayload[idx]; idx++;

				switch (evt->DiscoveredServices[i].Characteristics[j].Descriptors[k].UuidType)
				{
					case Uuid16Bits:
						FLib_MemCpy(evt->DiscoveredServices[i].Characteristics[j].Descriptors[k].Uuid.Uuid16Bits, pPayload + idx, 2); idx += 2;
						break;

					case Uuid128Bits:
						FLib_MemCpy(evt->DiscoveredServices[i].Characteristics[j].Descriptors[k].Uuid.Uuid128Bits, pPayload + idx, 16); idx += 16;
						break;

					case Uuid32Bits:
						FLib_MemCpy(evt->DiscoveredServices[i].Characteristics[j].Descriptors[k].Uuid.Uuid32Bits, pPayload + idx, 4); idx += 4;
						break;
				}
				FLib_MemCpy(&(evt->DiscoveredServices[i].Characteristics[j].Descriptors[k].ValueLength), pPayload + idx, sizeof(evt->DiscoveredServices[i].Characteristics[j].Descriptors[k].ValueLength)); idx += sizeof(evt->DiscoveredServices[i].Characteristics[j].Descriptors[k].ValueLength);
				FLib_MemCpy(&(evt->DiscoveredServices[i].Characteristics[j].Descriptors[k].MaxValueLength), pPayload + idx, sizeof(evt->DiscoveredServices[i].Characteristics[j].Descriptors[k].MaxValueLength)); idx += sizeof(evt->DiscoveredServices[i].Characteristics[j].Descriptors[k].MaxValueLength);

				if (evt->DiscoveredServices[i].Characteristics[j].Descriptors[k].ValueLength > 0)
				{
					evt->DiscoveredServices[i].Characteristics[j].Descriptors[k].Value = MEM_BufferAlloc(evt->DiscoveredServices[i].Characteristics[j].Descriptors[k].ValueLength);

					if (!evt->DiscoveredServices[i].Characteristics[j].Descriptors[k].Value)
					{
						for (uint32_t i = 0; i < evt->NbOfDiscoveredServices; i++)
						{
							for (uint32_t j = 0; j < evt->DiscoveredServices[i].NbOfCharacteristics; j++)
							{
								MEM_BufferFree(evt->DiscoveredServices[i].Characteristics[j].Descriptors);
							}
						}
						for (uint32_t i = 0; i < evt->NbOfDiscoveredServices; i++)
						{
							for (uint32_t j = 0; j < evt->DiscoveredServices[i].NbOfCharacteristics; j++)
							{
								MEM_BufferFree(evt->DiscoveredServices[i].Characteristics[j].Value.Value);
							}
						}
						for (uint32_t i = 0; i < evt->NbOfDiscoveredServices; i++)
						{
							MEM_BufferFree(evt->DiscoveredServices[i].Characteristics);
						}
						MEM_BufferFree(evt->DiscoveredServices);
						return MEM_ALLOC_ERROR_c;
					}

				}
				else
				{
					evt->DiscoveredServices[i].Characteristics[j].Descriptors[k].Value = NULL;
				}

				FLib_MemCpy(evt->DiscoveredServices[i].Characteristics[j].Descriptors[k].Value, pPayload + idx, evt->DiscoveredServices[i].Characteristics[j].Descriptors[k].ValueLength); idx += evt->DiscoveredServices[i].Characteristics[j].Descriptors[k].ValueLength;
			}
		}
		evt->DiscoveredServices[i].NbOfIncludedServices = pPayload[idx]; idx++;

		if (evt->DiscoveredServices[i].NbOfIncludedServices > 0)
		{
			evt->DiscoveredServices[i].IncludedServices = MEM_BufferAlloc(evt->DiscoveredServices[i].NbOfIncludedServices * sizeof(evt->DiscoveredServices[i].IncludedServices[0]));

			if (!evt->DiscoveredServices[i].IncludedServices)
			{
				for (uint32_t i = 0; i < evt->NbOfDiscoveredServices; i++)
				{
					for (uint32_t j = 0; j < evt->DiscoveredServices[i].NbOfCharacteristics; j++)
					{
						for (uint32_t k = 0; k < evt->DiscoveredServices[i].Characteristics[j].NbOfDescriptors; k++)
						{
							MEM_BufferFree(evt->DiscoveredServices[i].Characteristics[j].Descriptors[k].Value);
						}
					}
				}
				for (uint32_t i = 0; i < evt->NbOfDiscoveredServices; i++)
				{
					for (uint32_t j = 0; j < evt->DiscoveredServices[i].NbOfCharacteristics; j++)
					{
						MEM_BufferFree(evt->DiscoveredServices[i].Characteristics[j].Descriptors);
					}
				}
				for (uint32_t i = 0; i < evt->NbOfDiscoveredServices; i++)
				{
					for (uint32_t j = 0; j < evt->DiscoveredServices[i].NbOfCharacteristics; j++)
					{
						MEM_BufferFree(evt->DiscoveredServices[i].Characteristics[j].Value.Value);
					}
				}
				for (uint32_t i = 0; i < evt->NbOfDiscoveredServices; i++)
				{
					MEM_BufferFree(evt->DiscoveredServices[i].Characteristics);
				}
				MEM_BufferFree(evt->DiscoveredServices);
				return MEM_ALLOC_ERROR_c;
			}

		}
		else
		{
			evt->DiscoveredServices[i].IncludedServices = NULL;
		}


		for (uint32_t j = 0; j < evt->DiscoveredServices[i].NbOfIncludedServices; j++)
		{
			FLib_MemCpy(&(evt->DiscoveredServices[i].IncludedServices[j].StartHandle), pPayload + idx, sizeof(evt->DiscoveredServices[i].IncludedServices[j].StartHandle)); idx += sizeof(evt->DiscoveredServices[i].IncludedServices[j].StartHandle);
			FLib_MemCpy(&(evt->DiscoveredServices[i].IncludedServices[j].EndHandle), pPayload + idx, sizeof(evt->DiscoveredServices[i].IncludedServices[j].EndHandle)); idx += sizeof(evt->DiscoveredServices[i].IncludedServices[j].EndHandle);
			evt->DiscoveredServices[i].IncludedServices[j].UuidType = (UuidType_t)pPayload[idx]; idx++;

			switch (evt->DiscoveredServices[i].IncludedServices[j].UuidType)
			{
				case Uuid16Bits:
					FLib_MemCpy(evt->DiscoveredServices[i].IncludedServices[j].Uuid.Uuid16Bits, pPayload + idx, 2); idx += 2;
					break;

				case Uuid128Bits:
					FLib_MemCpy(evt->DiscoveredServices[i].IncludedServices[j].Uuid.Uuid128Bits, pPayload + idx, 16); idx += 16;
					break;

				case Uuid32Bits:
					FLib_MemCpy(evt->DiscoveredServices[i].IncludedServices[j].Uuid.Uuid32Bits, pPayload + idx, 4); idx += 4;
					break;
			}
			evt->DiscoveredServices[i].IncludedServices[j].NbOfCharacteristics = pPayload[idx]; idx++;
			evt->DiscoveredServices[i].IncludedServices[j].NbOfIncludedServices = pPayload[idx]; idx++;
		}
	}

	return MEM_SUCCESS_c;
}

/*!*************************************************************************************************
\fn		static memStatus_t Load_GATTClientProcedureDiscoverPrimaryServicesByUuidIndication(bleEvtContainer_t *container, uint8_t *pPayload)
\brief	Specifies that the Primary Service Discovery By UUID procedure ended
***************************************************************************************************/
static memStatus_t Load_GATTClientProcedureDiscoverPrimaryServicesByUuidIndication(bleEvtContainer_t *container, uint8_t *pPayload)
{
	GATTClientProcedureDiscoverPrimaryServicesByUuidIndication_t *evt = &(container->Data.GATTClientProcedureDiscoverPrimaryServicesByUuidIndication);

	uint32_t idx = 0;

	/* Store (OG, OC) in ID */
	container->id = 0x4484;

	evt->DeviceId = pPayload[idx]; idx++;
	evt->ProcedureResult = (GATTClientProcedureDiscoverPrimaryServicesByUuidIndication_ProcedureResult_t)pPayload[idx]; idx++;
	FLib_MemCpy(&(evt->Error), pPayload + idx, 2); idx += 2;
	evt->NbOfDiscoveredServices = pPayload[idx]; idx++;

	if (evt->NbOfDiscoveredServices > 0)
	{
		evt->DiscoveredServices = MEM_BufferAlloc(evt->NbOfDiscoveredServices * sizeof(evt->DiscoveredServices[0]));

		if (!evt->DiscoveredServices)
		{
			return MEM_ALLOC_ERROR_c;
		}

	}
	else
	{
		evt->DiscoveredServices = NULL;
	}


	for (uint32_t i = 0; i < evt->NbOfDiscoveredServices; i++)
	{
		FLib_MemCpy(&(evt->DiscoveredServices[i].StartHandle), pPayload + idx, sizeof(evt->DiscoveredServices[i].StartHandle)); idx += sizeof(evt->DiscoveredServices[i].StartHandle);
		FLib_MemCpy(&(evt->DiscoveredServices[i].EndHandle), pPayload + idx, sizeof(evt->DiscoveredServices[i].EndHandle)); idx += sizeof(evt->DiscoveredServices[i].EndHandle);
		evt->DiscoveredServices[i].UuidType = (UuidType_t)pPayload[idx]; idx++;

		switch (evt->DiscoveredServices[i].UuidType)
		{
			case Uuid16Bits:
				FLib_MemCpy(evt->DiscoveredServices[i].Uuid.Uuid16Bits, pPayload + idx, 2); idx += 2;
				break;

			case Uuid128Bits:
				FLib_MemCpy(evt->DiscoveredServices[i].Uuid.Uuid128Bits, pPayload + idx, 16); idx += 16;
				break;

			case Uuid32Bits:
				FLib_MemCpy(evt->DiscoveredServices[i].Uuid.Uuid32Bits, pPayload + idx, 4); idx += 4;
				break;
		}
		evt->DiscoveredServices[i].NbOfCharacteristics = pPayload[idx]; idx++;

		if (evt->DiscoveredServices[i].NbOfCharacteristics > 0)
		{
			evt->DiscoveredServices[i].Characteristics = MEM_BufferAlloc(evt->DiscoveredServices[i].NbOfCharacteristics * sizeof(evt->DiscoveredServices[i].Characteristics[0]));

			if (!evt->DiscoveredServices[i].Characteristics)
			{
				MEM_BufferFree(evt->DiscoveredServices);
				return MEM_ALLOC_ERROR_c;
			}

		}
		else
		{
			evt->DiscoveredServices[i].Characteristics = NULL;
		}


		for (uint32_t j = 0; j < evt->DiscoveredServices[i].NbOfCharacteristics; j++)
		{
			evt->DiscoveredServices[i].Characteristics[j].Properties = (Properties_t)pPayload[idx]; idx++;
			FLib_MemCpy(&(evt->DiscoveredServices[i].Characteristics[j].Value.Handle), pPayload + idx, sizeof(evt->DiscoveredServices[i].Characteristics[j].Value.Handle)); idx += sizeof(evt->DiscoveredServices[i].Characteristics[j].Value.Handle);
			evt->DiscoveredServices[i].Characteristics[j].Value.UuidType = (UuidType_t)pPayload[idx]; idx++;

			switch (evt->DiscoveredServices[i].Characteristics[j].Value.UuidType)
			{
				case Uuid16Bits:
					FLib_MemCpy(evt->DiscoveredServices[i].Characteristics[j].Value.Uuid.Uuid16Bits, pPayload + idx, 2); idx += 2;
					break;

				case Uuid128Bits:
					FLib_MemCpy(evt->DiscoveredServices[i].Characteristics[j].Value.Uuid.Uuid128Bits, pPayload + idx, 16); idx += 16;
					break;

				case Uuid32Bits:
					FLib_MemCpy(evt->DiscoveredServices[i].Characteristics[j].Value.Uuid.Uuid32Bits, pPayload + idx, 4); idx += 4;
					break;
			}
			FLib_MemCpy(&(evt->DiscoveredServices[i].Characteristics[j].Value.ValueLength), pPayload + idx, sizeof(evt->DiscoveredServices[i].Characteristics[j].Value.ValueLength)); idx += sizeof(evt->DiscoveredServices[i].Characteristics[j].Value.ValueLength);
			FLib_MemCpy(&(evt->DiscoveredServices[i].Characteristics[j].Value.MaxValueLength), pPayload + idx, sizeof(evt->DiscoveredServices[i].Characteristics[j].Value.MaxValueLength)); idx += sizeof(evt->DiscoveredServices[i].Characteristics[j].Value.MaxValueLength);

			if (evt->DiscoveredServices[i].Characteristics[j].Value.ValueLength > 0)
			{
				evt->DiscoveredServices[i].Characteristics[j].Value.Value = MEM_BufferAlloc(evt->DiscoveredServices[i].Characteristics[j].Value.ValueLength);

				if (!evt->DiscoveredServices[i].Characteristics[j].Value.Value)
				{
					for (uint32_t i = 0; i < evt->NbOfDiscoveredServices; i++)
					{
						MEM_BufferFree(evt->DiscoveredServices[i].Characteristics);
					}
					MEM_BufferFree(evt->DiscoveredServices);
					return MEM_ALLOC_ERROR_c;
				}

			}
			else
			{
				evt->DiscoveredServices[i].Characteristics[j].Value.Value = NULL;
			}

			FLib_MemCpy(evt->DiscoveredServices[i].Characteristics[j].Value.Value, pPayload + idx, evt->DiscoveredServices[i].Characteristics[j].Value.ValueLength); idx += evt->DiscoveredServices[i].Characteristics[j].Value.ValueLength;
			evt->DiscoveredServices[i].Characteristics[j].NbOfDescriptors = pPayload[idx]; idx++;

			if (evt->DiscoveredServices[i].Characteristics[j].NbOfDescriptors > 0)
			{
				evt->DiscoveredServices[i].Characteristics[j].Descriptors = MEM_BufferAlloc(evt->DiscoveredServices[i].Characteristics[j].NbOfDescriptors * sizeof(evt->DiscoveredServices[i].Characteristics[j].Descriptors[0]));

				if (!evt->DiscoveredServices[i].Characteristics[j].Descriptors)
				{
					for (uint32_t i = 0; i < evt->NbOfDiscoveredServices; i++)
					{
						for (uint32_t j = 0; j < evt->DiscoveredServices[i].NbOfCharacteristics; j++)
						{
							MEM_BufferFree(evt->DiscoveredServices[i].Characteristics[j].Value.Value);
						}
					}
					for (uint32_t i = 0; i < evt->NbOfDiscoveredServices; i++)
					{
						MEM_BufferFree(evt->DiscoveredServices[i].Characteristics);
					}
					MEM_BufferFree(evt->DiscoveredServices);
					return MEM_ALLOC_ERROR_c;
				}

			}
			else
			{
				evt->DiscoveredServices[i].Characteristics[j].Descriptors = NULL;
			}


			for (uint32_t k = 0; k < evt->DiscoveredServices[i].Characteristics[j].NbOfDescriptors; k++)
			{
				FLib_MemCpy(&(evt->DiscoveredServices[i].Characteristics[j].Descriptors[k].Handle), pPayload + idx, sizeof(evt->DiscoveredServices[i].Characteristics[j].Descriptors[k].Handle)); idx += sizeof(evt->DiscoveredServices[i].Characteristics[j].Descriptors[k].Handle);
				evt->DiscoveredServices[i].Characteristics[j].Descriptors[k].UuidType = (UuidType_t)pPayload[idx]; idx++;

				switch (evt->DiscoveredServices[i].Characteristics[j].Descriptors[k].UuidType)
				{
					case Uuid16Bits:
						FLib_MemCpy(evt->DiscoveredServices[i].Characteristics[j].Descriptors[k].Uuid.Uuid16Bits, pPayload + idx, 2); idx += 2;
						break;

					case Uuid128Bits:
						FLib_MemCpy(evt->DiscoveredServices[i].Characteristics[j].Descriptors[k].Uuid.Uuid128Bits, pPayload + idx, 16); idx += 16;
						break;

					case Uuid32Bits:
						FLib_MemCpy(evt->DiscoveredServices[i].Characteristics[j].Descriptors[k].Uuid.Uuid32Bits, pPayload + idx, 4); idx += 4;
						break;
				}
				FLib_MemCpy(&(evt->DiscoveredServices[i].Characteristics[j].Descriptors[k].ValueLength), pPayload + idx, sizeof(evt->DiscoveredServices[i].Characteristics[j].Descriptors[k].ValueLength)); idx += sizeof(evt->DiscoveredServices[i].Characteristics[j].Descriptors[k].ValueLength);
				FLib_MemCpy(&(evt->DiscoveredServices[i].Characteristics[j].Descriptors[k].MaxValueLength), pPayload + idx, sizeof(evt->DiscoveredServices[i].Characteristics[j].Descriptors[k].MaxValueLength)); idx += sizeof(evt->DiscoveredServices[i].Characteristics[j].Descriptors[k].MaxValueLength);

				if (evt->DiscoveredServices[i].Characteristics[j].Descriptors[k].ValueLength > 0)
				{
					evt->DiscoveredServices[i].Characteristics[j].Descriptors[k].Value = MEM_BufferAlloc(evt->DiscoveredServices[i].Characteristics[j].Descriptors[k].ValueLength);

					if (!evt->DiscoveredServices[i].Characteristics[j].Descriptors[k].Value)
					{
						for (uint32_t i = 0; i < evt->NbOfDiscoveredServices; i++)
						{
							for (uint32_t j = 0; j < evt->DiscoveredServices[i].NbOfCharacteristics; j++)
							{
								MEM_BufferFree(evt->DiscoveredServices[i].Characteristics[j].Descriptors);
							}
						}
						for (uint32_t i = 0; i < evt->NbOfDiscoveredServices; i++)
						{
							for (uint32_t j = 0; j < evt->DiscoveredServices[i].NbOfCharacteristics; j++)
							{
								MEM_BufferFree(evt->DiscoveredServices[i].Characteristics[j].Value.Value);
							}
						}
						for (uint32_t i = 0; i < evt->NbOfDiscoveredServices; i++)
						{
							MEM_BufferFree(evt->DiscoveredServices[i].Characteristics);
						}
						MEM_BufferFree(evt->DiscoveredServices);
						return MEM_ALLOC_ERROR_c;
					}

				}
				else
				{
					evt->DiscoveredServices[i].Characteristics[j].Descriptors[k].Value = NULL;
				}

				FLib_MemCpy(evt->DiscoveredServices[i].Characteristics[j].Descriptors[k].Value, pPayload + idx, evt->DiscoveredServices[i].Characteristics[j].Descriptors[k].ValueLength); idx += evt->DiscoveredServices[i].Characteristics[j].Descriptors[k].ValueLength;
			}
		}
		evt->DiscoveredServices[i].NbOfIncludedServices = pPayload[idx]; idx++;

		if (evt->DiscoveredServices[i].NbOfIncludedServices > 0)
		{
			evt->DiscoveredServices[i].IncludedServices = MEM_BufferAlloc(evt->DiscoveredServices[i].NbOfIncludedServices * sizeof(evt->DiscoveredServices[i].IncludedServices[0]));

			if (!evt->DiscoveredServices[i].IncludedServices)
			{
				for (uint32_t i = 0; i < evt->NbOfDiscoveredServices; i++)
				{
					for (uint32_t j = 0; j < evt->DiscoveredServices[i].NbOfCharacteristics; j++)
					{
						for (uint32_t k = 0; k < evt->DiscoveredServices[i].Characteristics[j].NbOfDescriptors; k++)
						{
							MEM_BufferFree(evt->DiscoveredServices[i].Characteristics[j].Descriptors[k].Value);
						}
					}
				}
				for (uint32_t i = 0; i < evt->NbOfDiscoveredServices; i++)
				{
					for (uint32_t j = 0; j < evt->DiscoveredServices[i].NbOfCharacteristics; j++)
					{
						MEM_BufferFree(evt->DiscoveredServices[i].Characteristics[j].Descriptors);
					}
				}
				for (uint32_t i = 0; i < evt->NbOfDiscoveredServices; i++)
				{
					for (uint32_t j = 0; j < evt->DiscoveredServices[i].NbOfCharacteristics; j++)
					{
						MEM_BufferFree(evt->DiscoveredServices[i].Characteristics[j].Value.Value);
					}
				}
				for (uint32_t i = 0; i < evt->NbOfDiscoveredServices; i++)
				{
					MEM_BufferFree(evt->DiscoveredServices[i].Characteristics);
				}
				MEM_BufferFree(evt->DiscoveredServices);
				return MEM_ALLOC_ERROR_c;
			}

		}
		else
		{
			evt->DiscoveredServices[i].IncludedServices = NULL;
		}


		for (uint32_t j = 0; j < evt->DiscoveredServices[i].NbOfIncludedServices; j++)
		{
			FLib_MemCpy(&(evt->DiscoveredServices[i].IncludedServices[j].StartHandle), pPayload + idx, sizeof(evt->DiscoveredServices[i].IncludedServices[j].StartHandle)); idx += sizeof(evt->DiscoveredServices[i].IncludedServices[j].StartHandle);
			FLib_MemCpy(&(evt->DiscoveredServices[i].IncludedServices[j].EndHandle), pPayload + idx, sizeof(evt->DiscoveredServices[i].IncludedServices[j].EndHandle)); idx += sizeof(evt->DiscoveredServices[i].IncludedServices[j].EndHandle);
			evt->DiscoveredServices[i].IncludedServices[j].UuidType = (UuidType_t)pPayload[idx]; idx++;

			switch (evt->DiscoveredServices[i].IncludedServices[j].UuidType)
			{
				case Uuid16Bits:
					FLib_MemCpy(evt->DiscoveredServices[i].IncludedServices[j].Uuid.Uuid16Bits, pPayload + idx, 2); idx += 2;
					break;

				case Uuid128Bits:
					FLib_MemCpy(evt->DiscoveredServices[i].IncludedServices[j].Uuid.Uuid128Bits, pPayload + idx, 16); idx += 16;
					break;

				case Uuid32Bits:
					FLib_MemCpy(evt->DiscoveredServices[i].IncludedServices[j].Uuid.Uuid32Bits, pPayload + idx, 4); idx += 4;
					break;
			}
			evt->DiscoveredServices[i].IncludedServices[j].NbOfCharacteristics = pPayload[idx]; idx++;
			evt->DiscoveredServices[i].IncludedServices[j].NbOfIncludedServices = pPayload[idx]; idx++;
		}
	}

	return MEM_SUCCESS_c;
}

/*!*************************************************************************************************
\fn		static memStatus_t Load_GATTClientProcedureFindIncludedServicesIndication(bleEvtContainer_t *container, uint8_t *pPayload)
\brief	Specifies that the Find Included Services procedure ended
***************************************************************************************************/
static memStatus_t Load_GATTClientProcedureFindIncludedServicesIndication(bleEvtContainer_t *container, uint8_t *pPayload)
{
	GATTClientProcedureFindIncludedServicesIndication_t *evt = &(container->Data.GATTClientProcedureFindIncludedServicesIndication);

	uint32_t idx = 0;

	/* Store (OG, OC) in ID */
	container->id = 0x4485;

	evt->DeviceId = pPayload[idx]; idx++;
	evt->ProcedureResult = (GATTClientProcedureFindIncludedServicesIndication_ProcedureResult_t)pPayload[idx]; idx++;
	FLib_MemCpy(&(evt->Error), pPayload + idx, 2); idx += 2;
	FLib_MemCpy(&(evt->Service.StartHandle), pPayload + idx, sizeof(evt->Service.StartHandle)); idx += sizeof(evt->Service.StartHandle);
	FLib_MemCpy(&(evt->Service.EndHandle), pPayload + idx, sizeof(evt->Service.EndHandle)); idx += sizeof(evt->Service.EndHandle);
	evt->Service.UuidType = (UuidType_t)pPayload[idx]; idx++;

	switch (evt->Service.UuidType)
	{
		case Uuid16Bits:
			FLib_MemCpy(evt->Service.Uuid.Uuid16Bits, pPayload + idx, 2); idx += 2;
			break;

		case Uuid128Bits:
			FLib_MemCpy(evt->Service.Uuid.Uuid128Bits, pPayload + idx, 16); idx += 16;
			break;

		case Uuid32Bits:
			FLib_MemCpy(evt->Service.Uuid.Uuid32Bits, pPayload + idx, 4); idx += 4;
			break;
	}
	evt->Service.NbOfCharacteristics = pPayload[idx]; idx++;

	if (evt->Service.NbOfCharacteristics > 0)
	{
		evt->Service.Characteristics = MEM_BufferAlloc(evt->Service.NbOfCharacteristics * sizeof(evt->Service.Characteristics[0]));

		if (!evt->Service.Characteristics)
		{
			return MEM_ALLOC_ERROR_c;
		}

	}
	else
	{
		evt->Service.Characteristics = NULL;
	}


	for (uint32_t i = 0; i < evt->Service.NbOfCharacteristics; i++)
	{
		evt->Service.Characteristics[i].Properties = (Properties_t)pPayload[idx]; idx++;
		FLib_MemCpy(&(evt->Service.Characteristics[i].Value.Handle), pPayload + idx, sizeof(evt->Service.Characteristics[i].Value.Handle)); idx += sizeof(evt->Service.Characteristics[i].Value.Handle);
		evt->Service.Characteristics[i].Value.UuidType = (UuidType_t)pPayload[idx]; idx++;

		switch (evt->Service.Characteristics[i].Value.UuidType)
		{
			case Uuid16Bits:
				FLib_MemCpy(evt->Service.Characteristics[i].Value.Uuid.Uuid16Bits, pPayload + idx, 2); idx += 2;
				break;

			case Uuid128Bits:
				FLib_MemCpy(evt->Service.Characteristics[i].Value.Uuid.Uuid128Bits, pPayload + idx, 16); idx += 16;
				break;

			case Uuid32Bits:
				FLib_MemCpy(evt->Service.Characteristics[i].Value.Uuid.Uuid32Bits, pPayload + idx, 4); idx += 4;
				break;
		}
		FLib_MemCpy(&(evt->Service.Characteristics[i].Value.ValueLength), pPayload + idx, sizeof(evt->Service.Characteristics[i].Value.ValueLength)); idx += sizeof(evt->Service.Characteristics[i].Value.ValueLength);
		FLib_MemCpy(&(evt->Service.Characteristics[i].Value.MaxValueLength), pPayload + idx, sizeof(evt->Service.Characteristics[i].Value.MaxValueLength)); idx += sizeof(evt->Service.Characteristics[i].Value.MaxValueLength);

		if (evt->Service.Characteristics[i].Value.ValueLength > 0)
		{
			evt->Service.Characteristics[i].Value.Value = MEM_BufferAlloc(evt->Service.Characteristics[i].Value.ValueLength);

			if (!evt->Service.Characteristics[i].Value.Value)
			{
				MEM_BufferFree(evt->Service.Characteristics);
				return MEM_ALLOC_ERROR_c;
			}

		}
		else
		{
			evt->Service.Characteristics[i].Value.Value = NULL;
		}

		FLib_MemCpy(evt->Service.Characteristics[i].Value.Value, pPayload + idx, evt->Service.Characteristics[i].Value.ValueLength); idx += evt->Service.Characteristics[i].Value.ValueLength;
		evt->Service.Characteristics[i].NbOfDescriptors = pPayload[idx]; idx++;

		if (evt->Service.Characteristics[i].NbOfDescriptors > 0)
		{
			evt->Service.Characteristics[i].Descriptors = MEM_BufferAlloc(evt->Service.Characteristics[i].NbOfDescriptors * sizeof(evt->Service.Characteristics[i].Descriptors[0]));

			if (!evt->Service.Characteristics[i].Descriptors)
			{
				for (uint32_t i = 0; i < evt->Service.NbOfCharacteristics; i++)
				{
					MEM_BufferFree(evt->Service.Characteristics[i].Value.Value);
				}
				MEM_BufferFree(evt->Service.Characteristics);
				return MEM_ALLOC_ERROR_c;
			}

		}
		else
		{
			evt->Service.Characteristics[i].Descriptors = NULL;
		}


		for (uint32_t j = 0; j < evt->Service.Characteristics[i].NbOfDescriptors; j++)
		{
			FLib_MemCpy(&(evt->Service.Characteristics[i].Descriptors[j].Handle), pPayload + idx, sizeof(evt->Service.Characteristics[i].Descriptors[j].Handle)); idx += sizeof(evt->Service.Characteristics[i].Descriptors[j].Handle);
			evt->Service.Characteristics[i].Descriptors[j].UuidType = (UuidType_t)pPayload[idx]; idx++;

			switch (evt->Service.Characteristics[i].Descriptors[j].UuidType)
			{
				case Uuid16Bits:
					FLib_MemCpy(evt->Service.Characteristics[i].Descriptors[j].Uuid.Uuid16Bits, pPayload + idx, 2); idx += 2;
					break;

				case Uuid128Bits:
					FLib_MemCpy(evt->Service.Characteristics[i].Descriptors[j].Uuid.Uuid128Bits, pPayload + idx, 16); idx += 16;
					break;

				case Uuid32Bits:
					FLib_MemCpy(evt->Service.Characteristics[i].Descriptors[j].Uuid.Uuid32Bits, pPayload + idx, 4); idx += 4;
					break;
			}
			FLib_MemCpy(&(evt->Service.Characteristics[i].Descriptors[j].ValueLength), pPayload + idx, sizeof(evt->Service.Characteristics[i].Descriptors[j].ValueLength)); idx += sizeof(evt->Service.Characteristics[i].Descriptors[j].ValueLength);
			FLib_MemCpy(&(evt->Service.Characteristics[i].Descriptors[j].MaxValueLength), pPayload + idx, sizeof(evt->Service.Characteristics[i].Descriptors[j].MaxValueLength)); idx += sizeof(evt->Service.Characteristics[i].Descriptors[j].MaxValueLength);

			if (evt->Service.Characteristics[i].Descriptors[j].ValueLength > 0)
			{
				evt->Service.Characteristics[i].Descriptors[j].Value = MEM_BufferAlloc(evt->Service.Characteristics[i].Descriptors[j].ValueLength);

				if (!evt->Service.Characteristics[i].Descriptors[j].Value)
				{
					for (uint32_t i = 0; i < evt->Service.NbOfCharacteristics; i++)
					{
						MEM_BufferFree(evt->Service.Characteristics[i].Descriptors);
					}
					for (uint32_t i = 0; i < evt->Service.NbOfCharacteristics; i++)
					{
						MEM_BufferFree(evt->Service.Characteristics[i].Value.Value);
					}
					MEM_BufferFree(evt->Service.Characteristics);
					return MEM_ALLOC_ERROR_c;
				}

			}
			else
			{
				evt->Service.Characteristics[i].Descriptors[j].Value = NULL;
			}

			FLib_MemCpy(evt->Service.Characteristics[i].Descriptors[j].Value, pPayload + idx, evt->Service.Characteristics[i].Descriptors[j].ValueLength); idx += evt->Service.Characteristics[i].Descriptors[j].ValueLength;
		}
	}
	evt->Service.NbOfIncludedServices = pPayload[idx]; idx++;

	if (evt->Service.NbOfIncludedServices > 0)
	{
		evt->Service.IncludedServices = MEM_BufferAlloc(evt->Service.NbOfIncludedServices * sizeof(evt->Service.IncludedServices[0]));

		if (!evt->Service.IncludedServices)
		{
			for (uint32_t i = 0; i < evt->Service.NbOfCharacteristics; i++)
			{
				for (uint32_t j = 0; j < evt->Service.Characteristics[i].NbOfDescriptors; j++)
				{
					MEM_BufferFree(evt->Service.Characteristics[i].Descriptors[j].Value);
				}
			}
			for (uint32_t i = 0; i < evt->Service.NbOfCharacteristics; i++)
			{
				MEM_BufferFree(evt->Service.Characteristics[i].Descriptors);
			}
			for (uint32_t i = 0; i < evt->Service.NbOfCharacteristics; i++)
			{
				MEM_BufferFree(evt->Service.Characteristics[i].Value.Value);
			}
			MEM_BufferFree(evt->Service.Characteristics);
			return MEM_ALLOC_ERROR_c;
		}

	}
	else
	{
		evt->Service.IncludedServices = NULL;
	}


	for (uint32_t i = 0; i < evt->Service.NbOfIncludedServices; i++)
	{
		FLib_MemCpy(&(evt->Service.IncludedServices[i].StartHandle), pPayload + idx, sizeof(evt->Service.IncludedServices[i].StartHandle)); idx += sizeof(evt->Service.IncludedServices[i].StartHandle);
		FLib_MemCpy(&(evt->Service.IncludedServices[i].EndHandle), pPayload + idx, sizeof(evt->Service.IncludedServices[i].EndHandle)); idx += sizeof(evt->Service.IncludedServices[i].EndHandle);
		evt->Service.IncludedServices[i].UuidType = (UuidType_t)pPayload[idx]; idx++;

		switch (evt->Service.IncludedServices[i].UuidType)
		{
			case Uuid16Bits:
				FLib_MemCpy(evt->Service.IncludedServices[i].Uuid.Uuid16Bits, pPayload + idx, 2); idx += 2;
				break;

			case Uuid128Bits:
				FLib_MemCpy(evt->Service.IncludedServices[i].Uuid.Uuid128Bits, pPayload + idx, 16); idx += 16;
				break;

			case Uuid32Bits:
				FLib_MemCpy(evt->Service.IncludedServices[i].Uuid.Uuid32Bits, pPayload + idx, 4); idx += 4;
				break;
		}
		evt->Service.IncludedServices[i].NbOfCharacteristics = pPayload[idx]; idx++;
		evt->Service.IncludedServices[i].NbOfIncludedServices = pPayload[idx]; idx++;
	}

	return MEM_SUCCESS_c;
}

/*!*************************************************************************************************
\fn		static memStatus_t Load_GATTClientProcedureDiscoverAllCharacteristicsIndication(bleEvtContainer_t *container, uint8_t *pPayload)
\brief	Specifies that the Characteristic Discovery procedure for a given service ended
***************************************************************************************************/
static memStatus_t Load_GATTClientProcedureDiscoverAllCharacteristicsIndication(bleEvtContainer_t *container, uint8_t *pPayload)
{
	GATTClientProcedureDiscoverAllCharacteristicsIndication_t *evt = &(container->Data.GATTClientProcedureDiscoverAllCharacteristicsIndication);

	uint32_t idx = 0;

	/* Store (OG, OC) in ID */
	container->id = 0x4486;

	evt->DeviceId = pPayload[idx]; idx++;
	evt->ProcedureResult = (GATTClientProcedureDiscoverAllCharacteristicsIndication_ProcedureResult_t)pPayload[idx]; idx++;
	FLib_MemCpy(&(evt->Error), pPayload + idx, 2); idx += 2;
	FLib_MemCpy(&(evt->Service.StartHandle), pPayload + idx, sizeof(evt->Service.StartHandle)); idx += sizeof(evt->Service.StartHandle);
	FLib_MemCpy(&(evt->Service.EndHandle), pPayload + idx, sizeof(evt->Service.EndHandle)); idx += sizeof(evt->Service.EndHandle);
	evt->Service.UuidType = (UuidType_t)pPayload[idx]; idx++;

	switch (evt->Service.UuidType)
	{
		case Uuid16Bits:
			FLib_MemCpy(evt->Service.Uuid.Uuid16Bits, pPayload + idx, 2); idx += 2;
			break;

		case Uuid128Bits:
			FLib_MemCpy(evt->Service.Uuid.Uuid128Bits, pPayload + idx, 16); idx += 16;
			break;

		case Uuid32Bits:
			FLib_MemCpy(evt->Service.Uuid.Uuid32Bits, pPayload + idx, 4); idx += 4;
			break;
	}
	evt->Service.NbOfCharacteristics = pPayload[idx]; idx++;

	if (evt->Service.NbOfCharacteristics > 0)
	{
		evt->Service.Characteristics = MEM_BufferAlloc(evt->Service.NbOfCharacteristics * sizeof(evt->Service.Characteristics[0]));

		if (!evt->Service.Characteristics)
		{
			return MEM_ALLOC_ERROR_c;
		}

	}
	else
	{
		evt->Service.Characteristics = NULL;
	}


	for (uint32_t i = 0; i < evt->Service.NbOfCharacteristics; i++)
	{
		evt->Service.Characteristics[i].Properties = (Properties_t)pPayload[idx]; idx++;
		FLib_MemCpy(&(evt->Service.Characteristics[i].Value.Handle), pPayload + idx, sizeof(evt->Service.Characteristics[i].Value.Handle)); idx += sizeof(evt->Service.Characteristics[i].Value.Handle);
		evt->Service.Characteristics[i].Value.UuidType = (UuidType_t)pPayload[idx]; idx++;

		switch (evt->Service.Characteristics[i].Value.UuidType)
		{
			case Uuid16Bits:
				FLib_MemCpy(evt->Service.Characteristics[i].Value.Uuid.Uuid16Bits, pPayload + idx, 2); idx += 2;
				break;

			case Uuid128Bits:
				FLib_MemCpy(evt->Service.Characteristics[i].Value.Uuid.Uuid128Bits, pPayload + idx, 16); idx += 16;
				break;

			case Uuid32Bits:
				FLib_MemCpy(evt->Service.Characteristics[i].Value.Uuid.Uuid32Bits, pPayload + idx, 4); idx += 4;
				break;
		}
		FLib_MemCpy(&(evt->Service.Characteristics[i].Value.ValueLength), pPayload + idx, sizeof(evt->Service.Characteristics[i].Value.ValueLength)); idx += sizeof(evt->Service.Characteristics[i].Value.ValueLength);
		FLib_MemCpy(&(evt->Service.Characteristics[i].Value.MaxValueLength), pPayload + idx, sizeof(evt->Service.Characteristics[i].Value.MaxValueLength)); idx += sizeof(evt->Service.Characteristics[i].Value.MaxValueLength);

		if (evt->Service.Characteristics[i].Value.ValueLength > 0)
		{
			evt->Service.Characteristics[i].Value.Value = MEM_BufferAlloc(evt->Service.Characteristics[i].Value.ValueLength);

			if (!evt->Service.Characteristics[i].Value.Value)
			{
				MEM_BufferFree(evt->Service.Characteristics);
				return MEM_ALLOC_ERROR_c;
			}

		}
		else
		{
			evt->Service.Characteristics[i].Value.Value = NULL;
		}

		FLib_MemCpy(evt->Service.Characteristics[i].Value.Value, pPayload + idx, evt->Service.Characteristics[i].Value.ValueLength); idx += evt->Service.Characteristics[i].Value.ValueLength;
		evt->Service.Characteristics[i].NbOfDescriptors = pPayload[idx]; idx++;

		if (evt->Service.Characteristics[i].NbOfDescriptors > 0)
		{
			evt->Service.Characteristics[i].Descriptors = MEM_BufferAlloc(evt->Service.Characteristics[i].NbOfDescriptors * sizeof(evt->Service.Characteristics[i].Descriptors[0]));

			if (!evt->Service.Characteristics[i].Descriptors)
			{
				for (uint32_t i = 0; i < evt->Service.NbOfCharacteristics; i++)
				{
					MEM_BufferFree(evt->Service.Characteristics[i].Value.Value);
				}
				MEM_BufferFree(evt->Service.Characteristics);
				return MEM_ALLOC_ERROR_c;
			}

		}
		else
		{
			evt->Service.Characteristics[i].Descriptors = NULL;
		}


		for (uint32_t j = 0; j < evt->Service.Characteristics[i].NbOfDescriptors; j++)
		{
			FLib_MemCpy(&(evt->Service.Characteristics[i].Descriptors[j].Handle), pPayload + idx, sizeof(evt->Service.Characteristics[i].Descriptors[j].Handle)); idx += sizeof(evt->Service.Characteristics[i].Descriptors[j].Handle);
			evt->Service.Characteristics[i].Descriptors[j].UuidType = (UuidType_t)pPayload[idx]; idx++;

			switch (evt->Service.Characteristics[i].Descriptors[j].UuidType)
			{
				case Uuid16Bits:
					FLib_MemCpy(evt->Service.Characteristics[i].Descriptors[j].Uuid.Uuid16Bits, pPayload + idx, 2); idx += 2;
					break;

				case Uuid128Bits:
					FLib_MemCpy(evt->Service.Characteristics[i].Descriptors[j].Uuid.Uuid128Bits, pPayload + idx, 16); idx += 16;
					break;

				case Uuid32Bits:
					FLib_MemCpy(evt->Service.Characteristics[i].Descriptors[j].Uuid.Uuid32Bits, pPayload + idx, 4); idx += 4;
					break;
			}
			FLib_MemCpy(&(evt->Service.Characteristics[i].Descriptors[j].ValueLength), pPayload + idx, sizeof(evt->Service.Characteristics[i].Descriptors[j].ValueLength)); idx += sizeof(evt->Service.Characteristics[i].Descriptors[j].ValueLength);
			FLib_MemCpy(&(evt->Service.Characteristics[i].Descriptors[j].MaxValueLength), pPayload + idx, sizeof(evt->Service.Characteristics[i].Descriptors[j].MaxValueLength)); idx += sizeof(evt->Service.Characteristics[i].Descriptors[j].MaxValueLength);

			if (evt->Service.Characteristics[i].Descriptors[j].ValueLength > 0)
			{
				evt->Service.Characteristics[i].Descriptors[j].Value = MEM_BufferAlloc(evt->Service.Characteristics[i].Descriptors[j].ValueLength);

				if (!evt->Service.Characteristics[i].Descriptors[j].Value)
				{
					for (uint32_t i = 0; i < evt->Service.NbOfCharacteristics; i++)
					{
						MEM_BufferFree(evt->Service.Characteristics[i].Descriptors);
					}
					for (uint32_t i = 0; i < evt->Service.NbOfCharacteristics; i++)
					{
						MEM_BufferFree(evt->Service.Characteristics[i].Value.Value);
					}
					MEM_BufferFree(evt->Service.Characteristics);
					return MEM_ALLOC_ERROR_c;
				}

			}
			else
			{
				evt->Service.Characteristics[i].Descriptors[j].Value = NULL;
			}

			FLib_MemCpy(evt->Service.Characteristics[i].Descriptors[j].Value, pPayload + idx, evt->Service.Characteristics[i].Descriptors[j].ValueLength); idx += evt->Service.Characteristics[i].Descriptors[j].ValueLength;
		}
	}
	evt->Service.NbOfIncludedServices = pPayload[idx]; idx++;

	if (evt->Service.NbOfIncludedServices > 0)
	{
		evt->Service.IncludedServices = MEM_BufferAlloc(evt->Service.NbOfIncludedServices * sizeof(evt->Service.IncludedServices[0]));

		if (!evt->Service.IncludedServices)
		{
			for (uint32_t i = 0; i < evt->Service.NbOfCharacteristics; i++)
			{
				for (uint32_t j = 0; j < evt->Service.Characteristics[i].NbOfDescriptors; j++)
				{
					MEM_BufferFree(evt->Service.Characteristics[i].Descriptors[j].Value);
				}
			}
			for (uint32_t i = 0; i < evt->Service.NbOfCharacteristics; i++)
			{
				MEM_BufferFree(evt->Service.Characteristics[i].Descriptors);
			}
			for (uint32_t i = 0; i < evt->Service.NbOfCharacteristics; i++)
			{
				MEM_BufferFree(evt->Service.Characteristics[i].Value.Value);
			}
			MEM_BufferFree(evt->Service.Characteristics);
			return MEM_ALLOC_ERROR_c;
		}

	}
	else
	{
		evt->Service.IncludedServices = NULL;
	}


	for (uint32_t i = 0; i < evt->Service.NbOfIncludedServices; i++)
	{
		FLib_MemCpy(&(evt->Service.IncludedServices[i].StartHandle), pPayload + idx, sizeof(evt->Service.IncludedServices[i].StartHandle)); idx += sizeof(evt->Service.IncludedServices[i].StartHandle);
		FLib_MemCpy(&(evt->Service.IncludedServices[i].EndHandle), pPayload + idx, sizeof(evt->Service.IncludedServices[i].EndHandle)); idx += sizeof(evt->Service.IncludedServices[i].EndHandle);
		evt->Service.IncludedServices[i].UuidType = (UuidType_t)pPayload[idx]; idx++;

		switch (evt->Service.IncludedServices[i].UuidType)
		{
			case Uuid16Bits:
				FLib_MemCpy(evt->Service.IncludedServices[i].Uuid.Uuid16Bits, pPayload + idx, 2); idx += 2;
				break;

			case Uuid128Bits:
				FLib_MemCpy(evt->Service.IncludedServices[i].Uuid.Uuid128Bits, pPayload + idx, 16); idx += 16;
				break;

			case Uuid32Bits:
				FLib_MemCpy(evt->Service.IncludedServices[i].Uuid.Uuid32Bits, pPayload + idx, 4); idx += 4;
				break;
		}
		evt->Service.IncludedServices[i].NbOfCharacteristics = pPayload[idx]; idx++;
		evt->Service.IncludedServices[i].NbOfIncludedServices = pPayload[idx]; idx++;
	}

	return MEM_SUCCESS_c;
}

/*!*************************************************************************************************
\fn		static memStatus_t Load_GATTClientProcedureDiscoverCharacteristicByUuidIndication(bleEvtContainer_t *container, uint8_t *pPayload)
\brief	Specifies that the Characteristic Discovery procedure for a given service (with a given UUID) ended
***************************************************************************************************/
static memStatus_t Load_GATTClientProcedureDiscoverCharacteristicByUuidIndication(bleEvtContainer_t *container, uint8_t *pPayload)
{
	GATTClientProcedureDiscoverCharacteristicByUuidIndication_t *evt = &(container->Data.GATTClientProcedureDiscoverCharacteristicByUuidIndication);

	uint32_t idx = 0;

	/* Store (OG, OC) in ID */
	container->id = 0x4487;

	evt->DeviceId = pPayload[idx]; idx++;
	evt->ProcedureResult = (GATTClientProcedureDiscoverCharacteristicByUuidIndication_ProcedureResult_t)pPayload[idx]; idx++;
	FLib_MemCpy(&(evt->Error), pPayload + idx, 2); idx += 2;
	evt->NbOfCharacteristics = pPayload[idx]; idx++;

	if (evt->NbOfCharacteristics > 0)
	{
		evt->Characteristics = MEM_BufferAlloc(evt->NbOfCharacteristics * sizeof(evt->Characteristics[0]));

		if (!evt->Characteristics)
		{
			return MEM_ALLOC_ERROR_c;
		}

	}
	else
	{
		evt->Characteristics = NULL;
	}


	for (uint32_t i = 0; i < evt->NbOfCharacteristics; i++)
	{
		evt->Characteristics[i].Properties = (Properties_t)pPayload[idx]; idx++;
		FLib_MemCpy(&(evt->Characteristics[i].Value.Handle), pPayload + idx, sizeof(evt->Characteristics[i].Value.Handle)); idx += sizeof(evt->Characteristics[i].Value.Handle);
		evt->Characteristics[i].Value.UuidType = (UuidType_t)pPayload[idx]; idx++;

		switch (evt->Characteristics[i].Value.UuidType)
		{
			case Uuid16Bits:
				FLib_MemCpy(evt->Characteristics[i].Value.Uuid.Uuid16Bits, pPayload + idx, 2); idx += 2;
				break;

			case Uuid128Bits:
				FLib_MemCpy(evt->Characteristics[i].Value.Uuid.Uuid128Bits, pPayload + idx, 16); idx += 16;
				break;

			case Uuid32Bits:
				FLib_MemCpy(evt->Characteristics[i].Value.Uuid.Uuid32Bits, pPayload + idx, 4); idx += 4;
				break;
		}
		FLib_MemCpy(&(evt->Characteristics[i].Value.ValueLength), pPayload + idx, sizeof(evt->Characteristics[i].Value.ValueLength)); idx += sizeof(evt->Characteristics[i].Value.ValueLength);
		FLib_MemCpy(&(evt->Characteristics[i].Value.MaxValueLength), pPayload + idx, sizeof(evt->Characteristics[i].Value.MaxValueLength)); idx += sizeof(evt->Characteristics[i].Value.MaxValueLength);

		if (evt->Characteristics[i].Value.ValueLength > 0)
		{
			evt->Characteristics[i].Value.Value = MEM_BufferAlloc(evt->Characteristics[i].Value.ValueLength);

			if (!evt->Characteristics[i].Value.Value)
			{
				MEM_BufferFree(evt->Characteristics);
				return MEM_ALLOC_ERROR_c;
			}

		}
		else
		{
			evt->Characteristics[i].Value.Value = NULL;
		}

		FLib_MemCpy(evt->Characteristics[i].Value.Value, pPayload + idx, evt->Characteristics[i].Value.ValueLength); idx += evt->Characteristics[i].Value.ValueLength;
		evt->Characteristics[i].NbOfDescriptors = pPayload[idx]; idx++;

		if (evt->Characteristics[i].NbOfDescriptors > 0)
		{
			evt->Characteristics[i].Descriptors = MEM_BufferAlloc(evt->Characteristics[i].NbOfDescriptors * sizeof(evt->Characteristics[i].Descriptors[0]));

			if (!evt->Characteristics[i].Descriptors)
			{
				for (uint32_t i = 0; i < evt->NbOfCharacteristics; i++)
				{
					MEM_BufferFree(evt->Characteristics[i].Value.Value);
				}
				MEM_BufferFree(evt->Characteristics);
				return MEM_ALLOC_ERROR_c;
			}

		}
		else
		{
			evt->Characteristics[i].Descriptors = NULL;
		}


		for (uint32_t j = 0; j < evt->Characteristics[i].NbOfDescriptors; j++)
		{
			FLib_MemCpy(&(evt->Characteristics[i].Descriptors[j].Handle), pPayload + idx, sizeof(evt->Characteristics[i].Descriptors[j].Handle)); idx += sizeof(evt->Characteristics[i].Descriptors[j].Handle);
			evt->Characteristics[i].Descriptors[j].UuidType = (UuidType_t)pPayload[idx]; idx++;

			switch (evt->Characteristics[i].Descriptors[j].UuidType)
			{
				case Uuid16Bits:
					FLib_MemCpy(evt->Characteristics[i].Descriptors[j].Uuid.Uuid16Bits, pPayload + idx, 2); idx += 2;
					break;

				case Uuid128Bits:
					FLib_MemCpy(evt->Characteristics[i].Descriptors[j].Uuid.Uuid128Bits, pPayload + idx, 16); idx += 16;
					break;

				case Uuid32Bits:
					FLib_MemCpy(evt->Characteristics[i].Descriptors[j].Uuid.Uuid32Bits, pPayload + idx, 4); idx += 4;
					break;
			}
			FLib_MemCpy(&(evt->Characteristics[i].Descriptors[j].ValueLength), pPayload + idx, sizeof(evt->Characteristics[i].Descriptors[j].ValueLength)); idx += sizeof(evt->Characteristics[i].Descriptors[j].ValueLength);
			FLib_MemCpy(&(evt->Characteristics[i].Descriptors[j].MaxValueLength), pPayload + idx, sizeof(evt->Characteristics[i].Descriptors[j].MaxValueLength)); idx += sizeof(evt->Characteristics[i].Descriptors[j].MaxValueLength);

			if (evt->Characteristics[i].Descriptors[j].ValueLength > 0)
			{
				evt->Characteristics[i].Descriptors[j].Value = MEM_BufferAlloc(evt->Characteristics[i].Descriptors[j].ValueLength);

				if (!evt->Characteristics[i].Descriptors[j].Value)
				{
					for (uint32_t i = 0; i < evt->NbOfCharacteristics; i++)
					{
						MEM_BufferFree(evt->Characteristics[i].Descriptors);
					}
					for (uint32_t i = 0; i < evt->NbOfCharacteristics; i++)
					{
						MEM_BufferFree(evt->Characteristics[i].Value.Value);
					}
					MEM_BufferFree(evt->Characteristics);
					return MEM_ALLOC_ERROR_c;
				}

			}
			else
			{
				evt->Characteristics[i].Descriptors[j].Value = NULL;
			}

			FLib_MemCpy(evt->Characteristics[i].Descriptors[j].Value, pPayload + idx, evt->Characteristics[i].Descriptors[j].ValueLength); idx += evt->Characteristics[i].Descriptors[j].ValueLength;
		}
	}

	return MEM_SUCCESS_c;
}

/*!*************************************************************************************************
\fn		static memStatus_t Load_GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication(bleEvtContainer_t *container, uint8_t *pPayload)
\brief	Specifies that the Characteristic Descriptor Discovery procedure for a given characteristic ended
***************************************************************************************************/
static memStatus_t Load_GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication(bleEvtContainer_t *container, uint8_t *pPayload)
{
	GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication_t *evt = &(container->Data.GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication);

	uint32_t idx = 0;

	/* Store (OG, OC) in ID */
	container->id = 0x4488;

	evt->DeviceId = pPayload[idx]; idx++;
	evt->ProcedureResult = (GATTClientProcedureDiscoverAllCharacteristicDescriptorsIndication_ProcedureResult_t)pPayload[idx]; idx++;
	FLib_MemCpy(&(evt->Error), pPayload + idx, 2); idx += 2;
	evt->Characteristic.Properties = (Properties_t)pPayload[idx]; idx++;
	FLib_MemCpy(&(evt->Characteristic.Value.Handle), pPayload + idx, sizeof(evt->Characteristic.Value.Handle)); idx += sizeof(evt->Characteristic.Value.Handle);
	evt->Characteristic.Value.UuidType = (UuidType_t)pPayload[idx]; idx++;

	switch (evt->Characteristic.Value.UuidType)
	{
		case Uuid16Bits:
			FLib_MemCpy(evt->Characteristic.Value.Uuid.Uuid16Bits, pPayload + idx, 2); idx += 2;
			break;

		case Uuid128Bits:
			FLib_MemCpy(evt->Characteristic.Value.Uuid.Uuid128Bits, pPayload + idx, 16); idx += 16;
			break;

		case Uuid32Bits:
			FLib_MemCpy(evt->Characteristic.Value.Uuid.Uuid32Bits, pPayload + idx, 4); idx += 4;
			break;
	}
	FLib_MemCpy(&(evt->Characteristic.Value.ValueLength), pPayload + idx, sizeof(evt->Characteristic.Value.ValueLength)); idx += sizeof(evt->Characteristic.Value.ValueLength);
	FLib_MemCpy(&(evt->Characteristic.Value.MaxValueLength), pPayload + idx, sizeof(evt->Characteristic.Value.MaxValueLength)); idx += sizeof(evt->Characteristic.Value.MaxValueLength);

	if (evt->Characteristic.Value.ValueLength > 0)
	{
		evt->Characteristic.Value.Value = MEM_BufferAlloc(evt->Characteristic.Value.ValueLength);

		if (!evt->Characteristic.Value.Value)
		{
			return MEM_ALLOC_ERROR_c;
		}

	}
	else
	{
		evt->Characteristic.Value.Value = NULL;
	}

	FLib_MemCpy(evt->Characteristic.Value.Value, pPayload + idx, evt->Characteristic.Value.ValueLength); idx += evt->Characteristic.Value.ValueLength;
	evt->Characteristic.NbOfDescriptors = pPayload[idx]; idx++;

	if (evt->Characteristic.NbOfDescriptors > 0)
	{
		evt->Characteristic.Descriptors = MEM_BufferAlloc(evt->Characteristic.NbOfDescriptors * sizeof(evt->Characteristic.Descriptors[0]));

		if (!evt->Characteristic.Descriptors)
		{
			MEM_BufferFree(evt->Characteristic.Value.Value);
			return MEM_ALLOC_ERROR_c;
		}

	}
	else
	{
		evt->Characteristic.Descriptors = NULL;
	}


	for (uint32_t i = 0; i < evt->Characteristic.NbOfDescriptors; i++)
	{
		FLib_MemCpy(&(evt->Characteristic.Descriptors[i].Handle), pPayload + idx, sizeof(evt->Characteristic.Descriptors[i].Handle)); idx += sizeof(evt->Characteristic.Descriptors[i].Handle);
		evt->Characteristic.Descriptors[i].UuidType = (UuidType_t)pPayload[idx]; idx++;

		switch (evt->Characteristic.Descriptors[i].UuidType)
		{
			case Uuid16Bits:
				FLib_MemCpy(evt->Characteristic.Descriptors[i].Uuid.Uuid16Bits, pPayload + idx, 2); idx += 2;
				break;

			case Uuid128Bits:
				FLib_MemCpy(evt->Characteristic.Descriptors[i].Uuid.Uuid128Bits, pPayload + idx, 16); idx += 16;
				break;

			case Uuid32Bits:
				FLib_MemCpy(evt->Characteristic.Descriptors[i].Uuid.Uuid32Bits, pPayload + idx, 4); idx += 4;
				break;
		}
		FLib_MemCpy(&(evt->Characteristic.Descriptors[i].ValueLength), pPayload + idx, sizeof(evt->Characteristic.Descriptors[i].ValueLength)); idx += sizeof(evt->Characteristic.Descriptors[i].ValueLength);
		FLib_MemCpy(&(evt->Characteristic.Descriptors[i].MaxValueLength), pPayload + idx, sizeof(evt->Characteristic.Descriptors[i].MaxValueLength)); idx += sizeof(evt->Characteristic.Descriptors[i].MaxValueLength);

		if (evt->Characteristic.Descriptors[i].ValueLength > 0)
		{
			evt->Characteristic.Descriptors[i].Value = MEM_BufferAlloc(evt->Characteristic.Descriptors[i].ValueLength);

			if (!evt->Characteristic.Descriptors[i].Value)
			{
				MEM_BufferFree(evt->Characteristic.Descriptors);
				MEM_BufferFree(evt->Characteristic.Value.Value);
				return MEM_ALLOC_ERROR_c;
			}

		}
		else
		{
			evt->Characteristic.Descriptors[i].Value = NULL;
		}

		FLib_MemCpy(evt->Characteristic.Descriptors[i].Value, pPayload + idx, evt->Characteristic.Descriptors[i].ValueLength); idx += evt->Characteristic.Descriptors[i].ValueLength;
	}

	return MEM_SUCCESS_c;
}

/*!*************************************************************************************************
\fn		static memStatus_t Load_GATTClientProcedureReadCharacteristicValueIndication(bleEvtContainer_t *container, uint8_t *pPayload)
\brief	Specifies that the Characteristic Read procedure for a given characteristic ended
***************************************************************************************************/
static memStatus_t Load_GATTClientProcedureReadCharacteristicValueIndication(bleEvtContainer_t *container, uint8_t *pPayload)
{
	GATTClientProcedureReadCharacteristicValueIndication_t *evt = &(container->Data.GATTClientProcedureReadCharacteristicValueIndication);

	uint32_t idx = 0;

	/* Store (OG, OC) in ID */
	container->id = 0x4489;

	evt->DeviceId = pPayload[idx]; idx++;
	evt->ProcedureResult = (GATTClientProcedureReadCharacteristicValueIndication_ProcedureResult_t)pPayload[idx]; idx++;
	FLib_MemCpy(&(evt->Error), pPayload + idx, 2); idx += 2;
	evt->Characteristic.Properties = (Properties_t)pPayload[idx]; idx++;
	FLib_MemCpy(&(evt->Characteristic.Value.Handle), pPayload + idx, sizeof(evt->Characteristic.Value.Handle)); idx += sizeof(evt->Characteristic.Value.Handle);
	evt->Characteristic.Value.UuidType = (UuidType_t)pPayload[idx]; idx++;

	switch (evt->Characteristic.Value.UuidType)
	{
		case Uuid16Bits:
			FLib_MemCpy(evt->Characteristic.Value.Uuid.Uuid16Bits, pPayload + idx, 2); idx += 2;
			break;

		case Uuid128Bits:
			FLib_MemCpy(evt->Characteristic.Value.Uuid.Uuid128Bits, pPayload + idx, 16); idx += 16;
			break;

		case Uuid32Bits:
			FLib_MemCpy(evt->Characteristic.Value.Uuid.Uuid32Bits, pPayload + idx, 4); idx += 4;
			break;
	}
	FLib_MemCpy(&(evt->Characteristic.Value.ValueLength), pPayload + idx, sizeof(evt->Characteristic.Value.ValueLength)); idx += sizeof(evt->Characteristic.Value.ValueLength);
	FLib_MemCpy(&(evt->Characteristic.Value.MaxValueLength), pPayload + idx, sizeof(evt->Characteristic.Value.MaxValueLength)); idx += sizeof(evt->Characteristic.Value.MaxValueLength);

	if (evt->Characteristic.Value.ValueLength > 0)
	{
		evt->Characteristic.Value.Value = MEM_BufferAlloc(evt->Characteristic.Value.ValueLength);

		if (!evt->Characteristic.Value.Value)
		{
			return MEM_ALLOC_ERROR_c;
		}

	}
	else
	{
		evt->Characteristic.Value.Value = NULL;
	}

	FLib_MemCpy(evt->Characteristic.Value.Value, pPayload + idx, evt->Characteristic.Value.ValueLength); idx += evt->Characteristic.Value.ValueLength;
	evt->Characteristic.NbOfDescriptors = pPayload[idx]; idx++;

	if (evt->Characteristic.NbOfDescriptors > 0)
	{
		evt->Characteristic.Descriptors = MEM_BufferAlloc(evt->Characteristic.NbOfDescriptors * sizeof(evt->Characteristic.Descriptors[0]));

		if (!evt->Characteristic.Descriptors)
		{
			MEM_BufferFree(evt->Characteristic.Value.Value);
			return MEM_ALLOC_ERROR_c;
		}

	}
	else
	{
		evt->Characteristic.Descriptors = NULL;
	}


	for (uint32_t i = 0; i < evt->Characteristic.NbOfDescriptors; i++)
	{
		FLib_MemCpy(&(evt->Characteristic.Descriptors[i].Handle), pPayload + idx, sizeof(evt->Characteristic.Descriptors[i].Handle)); idx += sizeof(evt->Characteristic.Descriptors[i].Handle);
		evt->Characteristic.Descriptors[i].UuidType = (UuidType_t)pPayload[idx]; idx++;

		switch (evt->Characteristic.Descriptors[i].UuidType)
		{
			case Uuid16Bits:
				FLib_MemCpy(evt->Characteristic.Descriptors[i].Uuid.Uuid16Bits, pPayload + idx, 2); idx += 2;
				break;

			case Uuid128Bits:
				FLib_MemCpy(evt->Characteristic.Descriptors[i].Uuid.Uuid128Bits, pPayload + idx, 16); idx += 16;
				break;

			case Uuid32Bits:
				FLib_MemCpy(evt->Characteristic.Descriptors[i].Uuid.Uuid32Bits, pPayload + idx, 4); idx += 4;
				break;
		}
		FLib_MemCpy(&(evt->Characteristic.Descriptors[i].ValueLength), pPayload + idx, sizeof(evt->Characteristic.Descriptors[i].ValueLength)); idx += sizeof(evt->Characteristic.Descriptors[i].ValueLength);
		FLib_MemCpy(&(evt->Characteristic.Descriptors[i].MaxValueLength), pPayload + idx, sizeof(evt->Characteristic.Descriptors[i].MaxValueLength)); idx += sizeof(evt->Characteristic.Descriptors[i].MaxValueLength);

		if (evt->Characteristic.Descriptors[i].ValueLength > 0)
		{
			evt->Characteristic.Descriptors[i].Value = MEM_BufferAlloc(evt->Characteristic.Descriptors[i].ValueLength);

			if (!evt->Characteristic.Descriptors[i].Value)
			{
				MEM_BufferFree(evt->Characteristic.Descriptors);
				MEM_BufferFree(evt->Characteristic.Value.Value);
				return MEM_ALLOC_ERROR_c;
			}

		}
		else
		{
			evt->Characteristic.Descriptors[i].Value = NULL;
		}

		FLib_MemCpy(evt->Characteristic.Descriptors[i].Value, pPayload + idx, evt->Characteristic.Descriptors[i].ValueLength); idx += evt->Characteristic.Descriptors[i].ValueLength;
	}

	return MEM_SUCCESS_c;
}

/*!*************************************************************************************************
\fn		static memStatus_t Load_GATTClientProcedureReadUsingCharacteristicUuidIndication(bleEvtContainer_t *container, uint8_t *pPayload)
\brief	Specifies that the Characteristic Read By UUID procedure ended
***************************************************************************************************/
static memStatus_t Load_GATTClientProcedureReadUsingCharacteristicUuidIndication(bleEvtContainer_t *container, uint8_t *pPayload)
{
	GATTClientProcedureReadUsingCharacteristicUuidIndication_t *evt = &(container->Data.GATTClientProcedureReadUsingCharacteristicUuidIndication);

	uint32_t idx = 0;

	/* Store (OG, OC) in ID */
	container->id = 0x448A;

	evt->DeviceId = pPayload[idx]; idx++;
	evt->ProcedureResult = (GATTClientProcedureReadUsingCharacteristicUuidIndication_ProcedureResult_t)pPayload[idx]; idx++;
	FLib_MemCpy(&(evt->Error), pPayload + idx, 2); idx += 2;
	FLib_MemCpy(&(evt->NbOfReadBytes), pPayload + idx, sizeof(evt->NbOfReadBytes)); idx += sizeof(evt->NbOfReadBytes);

	if (evt->NbOfReadBytes > 0)
	{
		evt->ReadBytes = MEM_BufferAlloc(evt->NbOfReadBytes);

		if (!evt->ReadBytes)
		{
			return MEM_ALLOC_ERROR_c;
		}

	}
	else
	{
		evt->ReadBytes = NULL;
	}

	FLib_MemCpy(evt->ReadBytes, pPayload + idx, evt->NbOfReadBytes); idx += evt->NbOfReadBytes;

	return MEM_SUCCESS_c;
}

/*!*************************************************************************************************
\fn		static memStatus_t Load_GATTClientProcedureReadMultipleCharacteristicValuesIndication(bleEvtContainer_t *container, uint8_t *pPayload)
\brief	Specifies that the Characteristic Read Multiple procedure ended
***************************************************************************************************/
static memStatus_t Load_GATTClientProcedureReadMultipleCharacteristicValuesIndication(bleEvtContainer_t *container, uint8_t *pPayload)
{
	GATTClientProcedureReadMultipleCharacteristicValuesIndication_t *evt = &(container->Data.GATTClientProcedureReadMultipleCharacteristicValuesIndication);

	uint32_t idx = 0;

	/* Store (OG, OC) in ID */
	container->id = 0x448B;

	evt->DeviceId = pPayload[idx]; idx++;
	evt->ProcedureResult = (GATTClientProcedureReadMultipleCharacteristicValuesIndication_ProcedureResult_t)pPayload[idx]; idx++;
	FLib_MemCpy(&(evt->Error), pPayload + idx, 2); idx += 2;
	evt->NbOfCharacteristics = pPayload[idx]; idx++;

	if (evt->NbOfCharacteristics > 0)
	{
		evt->Characteristics = MEM_BufferAlloc(evt->NbOfCharacteristics * sizeof(evt->Characteristics[0]));

		if (!evt->Characteristics)
		{
			return MEM_ALLOC_ERROR_c;
		}

	}
	else
	{
		evt->Characteristics = NULL;
	}


	for (uint32_t i = 0; i < evt->NbOfCharacteristics; i++)
	{
		evt->Characteristics[i].Properties = (Properties_t)pPayload[idx]; idx++;
		FLib_MemCpy(&(evt->Characteristics[i].Value.Handle), pPayload + idx, sizeof(evt->Characteristics[i].Value.Handle)); idx += sizeof(evt->Characteristics[i].Value.Handle);
		evt->Characteristics[i].Value.UuidType = (UuidType_t)pPayload[idx]; idx++;

		switch (evt->Characteristics[i].Value.UuidType)
		{
			case Uuid16Bits:
				FLib_MemCpy(evt->Characteristics[i].Value.Uuid.Uuid16Bits, pPayload + idx, 2); idx += 2;
				break;

			case Uuid128Bits:
				FLib_MemCpy(evt->Characteristics[i].Value.Uuid.Uuid128Bits, pPayload + idx, 16); idx += 16;
				break;

			case Uuid32Bits:
				FLib_MemCpy(evt->Characteristics[i].Value.Uuid.Uuid32Bits, pPayload + idx, 4); idx += 4;
				break;
		}
		FLib_MemCpy(&(evt->Characteristics[i].Value.ValueLength), pPayload + idx, sizeof(evt->Characteristics[i].Value.ValueLength)); idx += sizeof(evt->Characteristics[i].Value.ValueLength);
		FLib_MemCpy(&(evt->Characteristics[i].Value.MaxValueLength), pPayload + idx, sizeof(evt->Characteristics[i].Value.MaxValueLength)); idx += sizeof(evt->Characteristics[i].Value.MaxValueLength);

		if (evt->Characteristics[i].Value.ValueLength > 0)
		{
			evt->Characteristics[i].Value.Value = MEM_BufferAlloc(evt->Characteristics[i].Value.ValueLength);

			if (!evt->Characteristics[i].Value.Value)
			{
				MEM_BufferFree(evt->Characteristics);
				return MEM_ALLOC_ERROR_c;
			}

		}
		else
		{
			evt->Characteristics[i].Value.Value = NULL;
		}

		FLib_MemCpy(evt->Characteristics[i].Value.Value, pPayload + idx, evt->Characteristics[i].Value.ValueLength); idx += evt->Characteristics[i].Value.ValueLength;
		evt->Characteristics[i].NbOfDescriptors = pPayload[idx]; idx++;

		if (evt->Characteristics[i].NbOfDescriptors > 0)
		{
			evt->Characteristics[i].Descriptors = MEM_BufferAlloc(evt->Characteristics[i].NbOfDescriptors * sizeof(evt->Characteristics[i].Descriptors[0]));

			if (!evt->Characteristics[i].Descriptors)
			{
				for (uint32_t i = 0; i < evt->NbOfCharacteristics; i++)
				{
					MEM_BufferFree(evt->Characteristics[i].Value.Value);
				}
				MEM_BufferFree(evt->Characteristics);
				return MEM_ALLOC_ERROR_c;
			}

		}
		else
		{
			evt->Characteristics[i].Descriptors = NULL;
		}


		for (uint32_t j = 0; j < evt->Characteristics[i].NbOfDescriptors; j++)
		{
			FLib_MemCpy(&(evt->Characteristics[i].Descriptors[j].Handle), pPayload + idx, sizeof(evt->Characteristics[i].Descriptors[j].Handle)); idx += sizeof(evt->Characteristics[i].Descriptors[j].Handle);
			evt->Characteristics[i].Descriptors[j].UuidType = (UuidType_t)pPayload[idx]; idx++;

			switch (evt->Characteristics[i].Descriptors[j].UuidType)
			{
				case Uuid16Bits:
					FLib_MemCpy(evt->Characteristics[i].Descriptors[j].Uuid.Uuid16Bits, pPayload + idx, 2); idx += 2;
					break;

				case Uuid128Bits:
					FLib_MemCpy(evt->Characteristics[i].Descriptors[j].Uuid.Uuid128Bits, pPayload + idx, 16); idx += 16;
					break;

				case Uuid32Bits:
					FLib_MemCpy(evt->Characteristics[i].Descriptors[j].Uuid.Uuid32Bits, pPayload + idx, 4); idx += 4;
					break;
			}
			FLib_MemCpy(&(evt->Characteristics[i].Descriptors[j].ValueLength), pPayload + idx, sizeof(evt->Characteristics[i].Descriptors[j].ValueLength)); idx += sizeof(evt->Characteristics[i].Descriptors[j].ValueLength);
			FLib_MemCpy(&(evt->Characteristics[i].Descriptors[j].MaxValueLength), pPayload + idx, sizeof(evt->Characteristics[i].Descriptors[j].MaxValueLength)); idx += sizeof(evt->Characteristics[i].Descriptors[j].MaxValueLength);

			if (evt->Characteristics[i].Descriptors[j].ValueLength > 0)
			{
				evt->Characteristics[i].Descriptors[j].Value = MEM_BufferAlloc(evt->Characteristics[i].Descriptors[j].ValueLength);

				if (!evt->Characteristics[i].Descriptors[j].Value)
				{
					for (uint32_t i = 0; i < evt->NbOfCharacteristics; i++)
					{
						MEM_BufferFree(evt->Characteristics[i].Descriptors);
					}
					for (uint32_t i = 0; i < evt->NbOfCharacteristics; i++)
					{
						MEM_BufferFree(evt->Characteristics[i].Value.Value);
					}
					MEM_BufferFree(evt->Characteristics);
					return MEM_ALLOC_ERROR_c;
				}

			}
			else
			{
				evt->Characteristics[i].Descriptors[j].Value = NULL;
			}

			FLib_MemCpy(evt->Characteristics[i].Descriptors[j].Value, pPayload + idx, evt->Characteristics[i].Descriptors[j].ValueLength); idx += evt->Characteristics[i].Descriptors[j].ValueLength;
		}
	}

	return MEM_SUCCESS_c;
}

/*!*************************************************************************************************
\fn		static memStatus_t Load_GATTClientProcedureWriteCharacteristicValueIndication(bleEvtContainer_t *container, uint8_t *pPayload)
\brief	Specifies that the Characteristic Write procedure for a given characteristic ended
***************************************************************************************************/
static memStatus_t Load_GATTClientProcedureWriteCharacteristicValueIndication(bleEvtContainer_t *container, uint8_t *pPayload)
{
	GATTClientProcedureWriteCharacteristicValueIndication_t *evt = &(container->Data.GATTClientProcedureWriteCharacteristicValueIndication);

	/* Store (OG, OC) in ID */
	container->id = 0x448C;

	FLib_MemCpy(evt, pPayload, sizeof(GATTClientProcedureWriteCharacteristicValueIndication_t));

	return MEM_SUCCESS_c;
}

/*!*************************************************************************************************
\fn		static memStatus_t Load_GATTClientProcedureReadCharacteristicDescriptorIndication(bleEvtContainer_t *container, uint8_t *pPayload)
\brief	Specifies that the Characteristic Descriptor Read procedure for a given characteristic's descriptor ended
***************************************************************************************************/
static memStatus_t Load_GATTClientProcedureReadCharacteristicDescriptorIndication(bleEvtContainer_t *container, uint8_t *pPayload)
{
	GATTClientProcedureReadCharacteristicDescriptorIndication_t *evt = &(container->Data.GATTClientProcedureReadCharacteristicDescriptorIndication);

	uint32_t idx = 0;

	/* Store (OG, OC) in ID */
	container->id = 0x448D;

	evt->DeviceId = pPayload[idx]; idx++;
	evt->ProcedureResult = (GATTClientProcedureReadCharacteristicDescriptorIndication_ProcedureResult_t)pPayload[idx]; idx++;
	FLib_MemCpy(&(evt->Error), pPayload + idx, 2); idx += 2;
	FLib_MemCpy(&(evt->Descriptor.Handle), pPayload + idx, sizeof(evt->Descriptor.Handle)); idx += sizeof(evt->Descriptor.Handle);
	evt->Descriptor.UuidType = (UuidType_t)pPayload[idx]; idx++;

	switch (evt->Descriptor.UuidType)
	{
		case Uuid16Bits:
			FLib_MemCpy(evt->Descriptor.Uuid.Uuid16Bits, pPayload + idx, 2); idx += 2;
			break;

		case Uuid128Bits:
			FLib_MemCpy(evt->Descriptor.Uuid.Uuid128Bits, pPayload + idx, 16); idx += 16;
			break;

		case Uuid32Bits:
			FLib_MemCpy(evt->Descriptor.Uuid.Uuid32Bits, pPayload + idx, 4); idx += 4;
			break;
	}
	FLib_MemCpy(&(evt->Descriptor.ValueLength), pPayload + idx, sizeof(evt->Descriptor.ValueLength)); idx += sizeof(evt->Descriptor.ValueLength);
	FLib_MemCpy(&(evt->Descriptor.MaxValueLength), pPayload + idx, sizeof(evt->Descriptor.MaxValueLength)); idx += sizeof(evt->Descriptor.MaxValueLength);

	if (evt->Descriptor.ValueLength > 0)
	{
		evt->Descriptor.Value = MEM_BufferAlloc(evt->Descriptor.ValueLength);

		if (!evt->Descriptor.Value)
		{
			return MEM_ALLOC_ERROR_c;
		}

	}
	else
	{
		evt->Descriptor.Value = NULL;
	}

	FLib_MemCpy(evt->Descriptor.Value, pPayload + idx, evt->Descriptor.ValueLength); idx += evt->Descriptor.ValueLength;

	return MEM_SUCCESS_c;
}

/*!*************************************************************************************************
\fn		static memStatus_t Load_GATTClientProcedureWriteCharacteristicDescriptorIndication(bleEvtContainer_t *container, uint8_t *pPayload)
\brief	Specifies that the Characteristic Descriptor Write procedure for a given characteristic's descriptor ended
***************************************************************************************************/
static memStatus_t Load_GATTClientProcedureWriteCharacteristicDescriptorIndication(bleEvtContainer_t *container, uint8_t *pPayload)
{
	GATTClientProcedureWriteCharacteristicDescriptorIndication_t *evt = &(container->Data.GATTClientProcedureWriteCharacteristicDescriptorIndication);

	/* Store (OG, OC) in ID */
	container->id = 0x448E;

	FLib_MemCpy(evt, pPayload, sizeof(GATTClientProcedureWriteCharacteristicDescriptorIndication_t));

	return MEM_SUCCESS_c;
}

/*!*************************************************************************************************
\fn		static memStatus_t Load_GATTClientNotificationIndication(bleEvtContainer_t *container, uint8_t *pPayload)
\brief	GATT Client notification
***************************************************************************************************/
static memStatus_t Load_GATTClientNotificationIndication(bleEvtContainer_t *container, uint8_t *pPayload)
{
	GATTClientNotificationIndication_t *evt = &(container->Data.GATTClientNotificationIndication);

	uint32_t idx = 0;

	/* Store (OG, OC) in ID */
	container->id = 0x448F;

	evt->DeviceId = pPayload[idx]; idx++;
	FLib_MemCpy(&(evt->CharacteristicValueHandle), pPayload + idx, sizeof(evt->CharacteristicValueHandle)); idx += sizeof(evt->CharacteristicValueHandle);
	FLib_MemCpy(&(evt->ValueLength), pPayload + idx, sizeof(evt->ValueLength)); idx += sizeof(evt->ValueLength);

	if (evt->ValueLength > 0)
	{
		evt->Value = MEM_BufferAlloc(evt->ValueLength);

		if (!evt->Value)
		{
			return MEM_ALLOC_ERROR_c;
		}

	}
	else
	{
		evt->Value = NULL;
	}

	FLib_MemCpy(evt->Value, pPayload + idx, evt->ValueLength); idx += evt->ValueLength;

	return MEM_SUCCESS_c;
}

/*!*************************************************************************************************
\fn		static memStatus_t Load_GATTClientIndicationIndication(bleEvtContainer_t *container, uint8_t *pPayload)
\brief	GATT Client indication
***************************************************************************************************/
static memStatus_t Load_GATTClientIndicationIndication(bleEvtContainer_t *container, uint8_t *pPayload)
{
	GATTClientIndicationIndication_t *evt = &(container->Data.GATTClientIndicationIndication);

	uint32_t idx = 0;

	/* Store (OG, OC) in ID */
	container->id = 0x4490;

	evt->DeviceId = pPayload[idx]; idx++;
	FLib_MemCpy(&(evt->CharacteristicValueHandle), pPayload + idx, sizeof(evt->CharacteristicValueHandle)); idx += sizeof(evt->CharacteristicValueHandle);
	FLib_MemCpy(&(evt->ValueLength), pPayload + idx, sizeof(evt->ValueLength)); idx += sizeof(evt->ValueLength);

	if (evt->ValueLength > 0)
	{
		evt->Value = MEM_BufferAlloc(evt->ValueLength);

		if (!evt->Value)
		{
			return MEM_ALLOC_ERROR_c;
		}

	}
	else
	{
		evt->Value = NULL;
	}

	FLib_MemCpy(evt->Value, pPayload + idx, evt->ValueLength); idx += evt->ValueLength;

	return MEM_SUCCESS_c;
}

/*!*************************************************************************************************
\fn		static memStatus_t Load_GATTServerMtuChangedIndication(bleEvtContainer_t *container, uint8_t *pPayload)
\brief	GATT Server MTU changed indication
***************************************************************************************************/
static memStatus_t Load_GATTServerMtuChangedIndication(bleEvtContainer_t *container, uint8_t *pPayload)
{
	GATTServerMtuChangedIndication_t *evt = &(container->Data.GATTServerMtuChangedIndication);

	/* Store (OG, OC) in ID */
	container->id = 0x4491;

	FLib_MemCpy(evt, pPayload, sizeof(GATTServerMtuChangedIndication_t));

	return MEM_SUCCESS_c;
}

/*!*************************************************************************************************
\fn		static memStatus_t Load_GATTServerHandleValueConfirmationIndication(bleEvtContainer_t *container, uint8_t *pPayload)
\brief	GATT Server handle value confirmation
***************************************************************************************************/
static memStatus_t Load_GATTServerHandleValueConfirmationIndication(bleEvtContainer_t *container, uint8_t *pPayload)
{
	GATTServerHandleValueConfirmationIndication_t *evt = &(container->Data.GATTServerHandleValueConfirmationIndication);

	/* Store (OG, OC) in ID */
	container->id = 0x4492;

	FLib_MemCpy(evt, pPayload, sizeof(GATTServerHandleValueConfirmationIndication_t));

	return MEM_SUCCESS_c;
}

/*!*************************************************************************************************
\fn		static memStatus_t Load_GATTServerAttributeWrittenIndication(bleEvtContainer_t *container, uint8_t *pPayload)
\brief	GATT Server attribute written
***************************************************************************************************/
static memStatus_t Load_GATTServerAttributeWrittenIndication(bleEvtContainer_t *container, uint8_t *pPayload)
{
	GATTServerAttributeWrittenIndication_t *evt = &(container->Data.GATTServerAttributeWrittenIndication);

	uint32_t idx = 0;

	/* Store (OG, OC) in ID */
	container->id = 0x4493;

	evt->DeviceId = pPayload[idx]; idx++;
	FLib_MemCpy(&(evt->AttributeWrittenEvent.Handle), pPayload + idx, sizeof(evt->AttributeWrittenEvent.Handle)); idx += sizeof(evt->AttributeWrittenEvent.Handle);
	FLib_MemCpy(&(evt->AttributeWrittenEvent.ValueLength), pPayload + idx, sizeof(evt->AttributeWrittenEvent.ValueLength)); idx += sizeof(evt->AttributeWrittenEvent.ValueLength);

	if (evt->AttributeWrittenEvent.ValueLength > 0)
	{
		evt->AttributeWrittenEvent.Value = MEM_BufferAlloc(evt->AttributeWrittenEvent.ValueLength);

		if (!evt->AttributeWrittenEvent.Value)
		{
			return MEM_ALLOC_ERROR_c;
		}

	}
	else
	{
		evt->AttributeWrittenEvent.Value = NULL;
	}

	FLib_MemCpy(evt->AttributeWrittenEvent.Value, pPayload + idx, evt->AttributeWrittenEvent.ValueLength); idx += evt->AttributeWrittenEvent.ValueLength;

	return MEM_SUCCESS_c;
}

/*!*************************************************************************************************
\fn		static memStatus_t Load_GATTServerCharacteristicCccdWrittenIndication(bleEvtContainer_t *container, uint8_t *pPayload)
\brief	GATT Server characteristic cccd written
***************************************************************************************************/
static memStatus_t Load_GATTServerCharacteristicCccdWrittenIndication(bleEvtContainer_t *container, uint8_t *pPayload)
{
	GATTServerCharacteristicCccdWrittenIndication_t *evt = &(container->Data.GATTServerCharacteristicCccdWrittenIndication);

	/* Store (OG, OC) in ID */
	container->id = 0x4494;

	FLib_MemCpy(evt, pPayload, sizeof(GATTServerCharacteristicCccdWrittenIndication_t));

	return MEM_SUCCESS_c;
}

/*!*************************************************************************************************
\fn		static memStatus_t Load_GATTServerAttributeWrittenWithoutResponseIndication(bleEvtContainer_t *container, uint8_t *pPayload)
\brief	GATT Server attribute written without response
***************************************************************************************************/
static memStatus_t Load_GATTServerAttributeWrittenWithoutResponseIndication(bleEvtContainer_t *container, uint8_t *pPayload)
{
	GATTServerAttributeWrittenWithoutResponseIndication_t *evt = &(container->Data.GATTServerAttributeWrittenWithoutResponseIndication);

	uint32_t idx = 0;

	/* Store (OG, OC) in ID */
	container->id = 0x4495;

	evt->DeviceId = pPayload[idx]; idx++;
	FLib_MemCpy(&(evt->AttributeWrittenEvent.Handle), pPayload + idx, sizeof(evt->AttributeWrittenEvent.Handle)); idx += sizeof(evt->AttributeWrittenEvent.Handle);
	FLib_MemCpy(&(evt->AttributeWrittenEvent.ValueLength), pPayload + idx, sizeof(evt->AttributeWrittenEvent.ValueLength)); idx += sizeof(evt->AttributeWrittenEvent.ValueLength);

	if (evt->AttributeWrittenEvent.ValueLength > 0)
	{
		evt->AttributeWrittenEvent.Value = MEM_BufferAlloc(evt->AttributeWrittenEvent.ValueLength);

		if (!evt->AttributeWrittenEvent.Value)
		{
			return MEM_ALLOC_ERROR_c;
		}

	}
	else
	{
		evt->AttributeWrittenEvent.Value = NULL;
	}

	FLib_MemCpy(evt->AttributeWrittenEvent.Value, pPayload + idx, evt->AttributeWrittenEvent.ValueLength); idx += evt->AttributeWrittenEvent.ValueLength;

	return MEM_SUCCESS_c;
}

/*!*************************************************************************************************
\fn		static memStatus_t Load_GATTServerErrorIndication(bleEvtContainer_t *container, uint8_t *pPayload)
\brief	GATT Server error
***************************************************************************************************/
static memStatus_t Load_GATTServerErrorIndication(bleEvtContainer_t *container, uint8_t *pPayload)
{
	GATTServerErrorIndication_t *evt = &(container->Data.GATTServerErrorIndication);

	/* Store (OG, OC) in ID */
	container->id = 0x4496;

	FLib_MemCpy(evt, pPayload, sizeof(GATTServerErrorIndication_t));

	return MEM_SUCCESS_c;
}

/*!*************************************************************************************************
\fn		static memStatus_t Load_GATTServerLongCharacteristicWrittenIndication(bleEvtContainer_t *container, uint8_t *pPayload)
\brief	GATT Server long characteristic written
***************************************************************************************************/
static memStatus_t Load_GATTServerLongCharacteristicWrittenIndication(bleEvtContainer_t *container, uint8_t *pPayload)
{
	GATTServerLongCharacteristicWrittenIndication_t *evt = &(container->Data.GATTServerLongCharacteristicWrittenIndication);

	uint32_t idx = 0;

	/* Store (OG, OC) in ID */
	container->id = 0x4497;

	evt->DeviceId = pPayload[idx]; idx++;
	FLib_MemCpy(&(evt->LongCharacteristicWrittenEvent.Handle), pPayload + idx, sizeof(evt->LongCharacteristicWrittenEvent.Handle)); idx += sizeof(evt->LongCharacteristicWrittenEvent.Handle);
	FLib_MemCpy(&(evt->LongCharacteristicWrittenEvent.ValueLength), pPayload + idx, sizeof(evt->LongCharacteristicWrittenEvent.ValueLength)); idx += sizeof(evt->LongCharacteristicWrittenEvent.ValueLength);

	if (evt->LongCharacteristicWrittenEvent.ValueLength > 0)
	{
		evt->LongCharacteristicWrittenEvent.Value = MEM_BufferAlloc(evt->LongCharacteristicWrittenEvent.ValueLength);

		if (!evt->LongCharacteristicWrittenEvent.Value)
		{
			return MEM_ALLOC_ERROR_c;
		}

	}
	else
	{
		evt->LongCharacteristicWrittenEvent.Value = NULL;
	}

	FLib_MemCpy(evt->LongCharacteristicWrittenEvent.Value, pPayload + idx, evt->LongCharacteristicWrittenEvent.ValueLength); idx += evt->LongCharacteristicWrittenEvent.ValueLength;

	return MEM_SUCCESS_c;
}

/*!*************************************************************************************************
\fn		static memStatus_t Load_GATTServerAttributeReadIndication(bleEvtContainer_t *container, uint8_t *pPayload)
\brief	GATT Server attribute read
***************************************************************************************************/
static memStatus_t Load_GATTServerAttributeReadIndication(bleEvtContainer_t *container, uint8_t *pPayload)
{
	GATTServerAttributeReadIndication_t *evt = &(container->Data.GATTServerAttributeReadIndication);

	/* Store (OG, OC) in ID */
	container->id = 0x4498;

	FLib_MemCpy(evt, pPayload, sizeof(GATTServerAttributeReadIndication_t));

	return MEM_SUCCESS_c;
}

/*!*************************************************************************************************
\fn		static memStatus_t Load_GATTDBConfirm(bleEvtContainer_t *container, uint8_t *pPayload)
\brief	Status of the GATT Database (application) request
***************************************************************************************************/
static memStatus_t Load_GATTDBConfirm(bleEvtContainer_t *container, uint8_t *pPayload)
{
	GATTDBConfirm_t *evt = &(container->Data.GATTDBConfirm);

	/* Store (OG, OC) in ID */
	container->id = 0x4580;

	FLib_MemCpy(evt, pPayload, sizeof(GATTDBConfirm_t));

	return MEM_SUCCESS_c;
}

/*!*************************************************************************************************
\fn		static memStatus_t Load_GATTDBReadAttributeIndication(bleEvtContainer_t *container, uint8_t *pPayload)
\brief	Reads an attribute value (from application level)
***************************************************************************************************/
static memStatus_t Load_GATTDBReadAttributeIndication(bleEvtContainer_t *container, uint8_t *pPayload)
{
	GATTDBReadAttributeIndication_t *evt = &(container->Data.GATTDBReadAttributeIndication);

	uint32_t idx = 0;

	/* Store (OG, OC) in ID */
	container->id = 0x4581;

	FLib_MemCpy(&(evt->ValueLength), pPayload + idx, sizeof(evt->ValueLength)); idx += sizeof(evt->ValueLength);

	if (evt->ValueLength > 0)
	{
		evt->Value = MEM_BufferAlloc(evt->ValueLength);

		if (!evt->Value)
		{
			return MEM_ALLOC_ERROR_c;
		}

	}
	else
	{
		evt->Value = NULL;
	}

	FLib_MemCpy(evt->Value, pPayload + idx, evt->ValueLength); idx += evt->ValueLength;

	return MEM_SUCCESS_c;
}

/*!*************************************************************************************************
\fn		static memStatus_t Load_GATTDBFindServiceHandleIndication(bleEvtContainer_t *container, uint8_t *pPayload)
\brief	Finds the handle of a Service Declaration with a given UUID inside the database
***************************************************************************************************/
static memStatus_t Load_GATTDBFindServiceHandleIndication(bleEvtContainer_t *container, uint8_t *pPayload)
{
	GATTDBFindServiceHandleIndication_t *evt = &(container->Data.GATTDBFindServiceHandleIndication);

	/* Store (OG, OC) in ID */
	container->id = 0x4582;

	FLib_MemCpy(evt, pPayload, sizeof(GATTDBFindServiceHandleIndication_t));

	return MEM_SUCCESS_c;
}

/*!*************************************************************************************************
\fn		static memStatus_t Load_GATTDBFindCharValueHandleInServiceIndication(bleEvtContainer_t *container, uint8_t *pPayload)
\brief	Finds the handle of a characteristic value (with a given UUID) inside a service
***************************************************************************************************/
static memStatus_t Load_GATTDBFindCharValueHandleInServiceIndication(bleEvtContainer_t *container, uint8_t *pPayload)
{
	GATTDBFindCharValueHandleInServiceIndication_t *evt = &(container->Data.GATTDBFindCharValueHandleInServiceIndication);

	/* Store (OG, OC) in ID */
	container->id = 0x4583;

	FLib_MemCpy(evt, pPayload, sizeof(GATTDBFindCharValueHandleInServiceIndication_t));

	return MEM_SUCCESS_c;
}

/*!*************************************************************************************************
\fn		static memStatus_t Load_GATTDBFindCccdHandleForCharValueHandleIndication(bleEvtContainer_t *container, uint8_t *pPayload)
\brief	Finds the handle of a characteristic's CCCD (giving the characteristic's value handle)
***************************************************************************************************/
static memStatus_t Load_GATTDBFindCccdHandleForCharValueHandleIndication(bleEvtContainer_t *container, uint8_t *pPayload)
{
	GATTDBFindCccdHandleForCharValueHandleIndication_t *evt = &(container->Data.GATTDBFindCccdHandleForCharValueHandleIndication);

	/* Store (OG, OC) in ID */
	container->id = 0x4584;

	FLib_MemCpy(evt, pPayload, sizeof(GATTDBFindCccdHandleForCharValueHandleIndication_t));

	return MEM_SUCCESS_c;
}

/*!*************************************************************************************************
\fn		static memStatus_t Load_GATTDBFindDescriptorHandleForCharValueHandleIndication(bleEvtContainer_t *container, uint8_t *pPayload)
\brief	Finds the handle of a characteristic descriptor (giving the characteristic's value handle and descriptor's UUID)
***************************************************************************************************/
static memStatus_t Load_GATTDBFindDescriptorHandleForCharValueHandleIndication(bleEvtContainer_t *container, uint8_t *pPayload)
{
	GATTDBFindDescriptorHandleForCharValueHandleIndication_t *evt = &(container->Data.GATTDBFindDescriptorHandleForCharValueHandleIndication);

	/* Store (OG, OC) in ID */
	container->id = 0x4585;

	FLib_MemCpy(evt, pPayload, sizeof(GATTDBFindDescriptorHandleForCharValueHandleIndication_t));

	return MEM_SUCCESS_c;
}

/*!*************************************************************************************************
\fn		static memStatus_t Load_GATTDBDynamicAddPrimaryServiceDeclarationIndication(bleEvtContainer_t *container, uint8_t *pPayload)
\brief	Adds a Primary Service declaration into the database
***************************************************************************************************/
static memStatus_t Load_GATTDBDynamicAddPrimaryServiceDeclarationIndication(bleEvtContainer_t *container, uint8_t *pPayload)
{
	GATTDBDynamicAddPrimaryServiceDeclarationIndication_t *evt = &(container->Data.GATTDBDynamicAddPrimaryServiceDeclarationIndication);

	/* Store (OG, OC) in ID */
	container->id = 0x4586;

	FLib_MemCpy(evt, pPayload, sizeof(GATTDBDynamicAddPrimaryServiceDeclarationIndication_t));

	return MEM_SUCCESS_c;
}

/*!*************************************************************************************************
\fn		static memStatus_t Load_GATTDBDynamicAddSecondaryServiceDeclarationIndication(bleEvtContainer_t *container, uint8_t *pPayload)
\brief	Adds a Secondary Service declaration into the database
***************************************************************************************************/
static memStatus_t Load_GATTDBDynamicAddSecondaryServiceDeclarationIndication(bleEvtContainer_t *container, uint8_t *pPayload)
{
	GATTDBDynamicAddSecondaryServiceDeclarationIndication_t *evt = &(container->Data.GATTDBDynamicAddSecondaryServiceDeclarationIndication);

	/* Store (OG, OC) in ID */
	container->id = 0x4587;

	FLib_MemCpy(evt, pPayload, sizeof(GATTDBDynamicAddSecondaryServiceDeclarationIndication_t));

	return MEM_SUCCESS_c;
}

/*!*************************************************************************************************
\fn		static memStatus_t Load_GATTDBDynamicAddIncludeDeclarationIndication(bleEvtContainer_t *container, uint8_t *pPayload)
\brief	Adds an Include declaration into the database
***************************************************************************************************/
static memStatus_t Load_GATTDBDynamicAddIncludeDeclarationIndication(bleEvtContainer_t *container, uint8_t *pPayload)
{
	GATTDBDynamicAddIncludeDeclarationIndication_t *evt = &(container->Data.GATTDBDynamicAddIncludeDeclarationIndication);

	/* Store (OG, OC) in ID */
	container->id = 0x4588;

	FLib_MemCpy(evt, pPayload, sizeof(GATTDBDynamicAddIncludeDeclarationIndication_t));

	return MEM_SUCCESS_c;
}

/*!*************************************************************************************************
\fn		static memStatus_t Load_GATTDBDynamicAddCharacteristicDeclarationAndValueIndication(bleEvtContainer_t *container, uint8_t *pPayload)
\brief	Adds a Characteristic declaration and its Value into the database
***************************************************************************************************/
static memStatus_t Load_GATTDBDynamicAddCharacteristicDeclarationAndValueIndication(bleEvtContainer_t *container, uint8_t *pPayload)
{
	GATTDBDynamicAddCharacteristicDeclarationAndValueIndication_t *evt = &(container->Data.GATTDBDynamicAddCharacteristicDeclarationAndValueIndication);

	/* Store (OG, OC) in ID */
	container->id = 0x4589;

	FLib_MemCpy(evt, pPayload, sizeof(GATTDBDynamicAddCharacteristicDeclarationAndValueIndication_t));

	return MEM_SUCCESS_c;
}

/*!*************************************************************************************************
\fn		static memStatus_t Load_GATTDBDynamicAddCharacteristicDescriptorIndication(bleEvtContainer_t *container, uint8_t *pPayload)
\brief	Adds a Characteristic descriptor into the database
***************************************************************************************************/
static memStatus_t Load_GATTDBDynamicAddCharacteristicDescriptorIndication(bleEvtContainer_t *container, uint8_t *pPayload)
{
	GATTDBDynamicAddCharacteristicDescriptorIndication_t *evt = &(container->Data.GATTDBDynamicAddCharacteristicDescriptorIndication);

	/* Store (OG, OC) in ID */
	container->id = 0x458A;

	FLib_MemCpy(evt, pPayload, sizeof(GATTDBDynamicAddCharacteristicDescriptorIndication_t));

	return MEM_SUCCESS_c;
}

/*!*************************************************************************************************
\fn		static memStatus_t Load_GATTDBDynamicAddCccdIndication(bleEvtContainer_t *container, uint8_t *pPayload)
\brief	Adds a CCCD in the database
***************************************************************************************************/
static memStatus_t Load_GATTDBDynamicAddCccdIndication(bleEvtContainer_t *container, uint8_t *pPayload)
{
	GATTDBDynamicAddCccdIndication_t *evt = &(container->Data.GATTDBDynamicAddCccdIndication);

	/* Store (OG, OC) in ID */
	container->id = 0x458B;

	FLib_MemCpy(evt, pPayload, sizeof(GATTDBDynamicAddCccdIndication_t));

	return MEM_SUCCESS_c;
}

/*!*************************************************************************************************
\fn		static memStatus_t Load_GATTDBDynamicAddCharacteristicDeclarationWithUniqueValueIndication(bleEvtContainer_t *container, uint8_t *pPayload)
\brief	Adds a Characteristic declaration with a Value contained in an universal value buffer
***************************************************************************************************/
static memStatus_t Load_GATTDBDynamicAddCharacteristicDeclarationWithUniqueValueIndication(bleEvtContainer_t *container, uint8_t *pPayload)
{
	GATTDBDynamicAddCharacteristicDeclarationWithUniqueValueIndication_t *evt = &(container->Data.GATTDBDynamicAddCharacteristicDeclarationWithUniqueValueIndication);

	/* Store (OG, OC) in ID */
	container->id = 0x458C;

	FLib_MemCpy(evt, pPayload, sizeof(GATTDBDynamicAddCharacteristicDeclarationWithUniqueValueIndication_t));

	return MEM_SUCCESS_c;
}

/*!*************************************************************************************************
\fn		static memStatus_t Load_GAPConfirm(bleEvtContainer_t *container, uint8_t *pPayload)
\brief	Status of the GAP request
***************************************************************************************************/
static memStatus_t Load_GAPConfirm(bleEvtContainer_t *container, uint8_t *pPayload)
{
	GAPConfirm_t *evt = &(container->Data.GAPConfirm);

	/* Store (OG, OC) in ID */
	container->id = 0x4780;

	FLib_MemCpy(evt, pPayload, sizeof(GAPConfirm_t));

	return MEM_SUCCESS_c;
}

/*!*************************************************************************************************
\fn		static memStatus_t Load_GAPCheckNotificationStatusIndication(bleEvtContainer_t *container, uint8_t *pPayload)
\brief	Returns the notification status for a given Client and a given CCCD handle
***************************************************************************************************/
static memStatus_t Load_GAPCheckNotificationStatusIndication(bleEvtContainer_t *container, uint8_t *pPayload)
{
	GAPCheckNotificationStatusIndication_t *evt = &(container->Data.GAPCheckNotificationStatusIndication);

	/* Store (OG, OC) in ID */
	container->id = 0x4781;

	FLib_MemCpy(evt, pPayload, sizeof(GAPCheckNotificationStatusIndication_t));

	return MEM_SUCCESS_c;
}

/*!*************************************************************************************************
\fn		static memStatus_t Load_GAPCheckIndicationStatusIndication(bleEvtContainer_t *container, uint8_t *pPayload)
\brief	Returns the indication status for a given Client and a given CCCD handle
***************************************************************************************************/
static memStatus_t Load_GAPCheckIndicationStatusIndication(bleEvtContainer_t *container, uint8_t *pPayload)
{
	GAPCheckIndicationStatusIndication_t *evt = &(container->Data.GAPCheckIndicationStatusIndication);

	/* Store (OG, OC) in ID */
	container->id = 0x4782;

	FLib_MemCpy(evt, pPayload, sizeof(GAPCheckIndicationStatusIndication_t));

	return MEM_SUCCESS_c;
}

/*!*************************************************************************************************
\fn		static memStatus_t Load_GAPGetBondedStaticAddressesIndication(bleEvtContainer_t *container, uint8_t *pPayload)
\brief	Returns a list with the static addresses of bonded devices
***************************************************************************************************/
static memStatus_t Load_GAPGetBondedStaticAddressesIndication(bleEvtContainer_t *container, uint8_t *pPayload)
{
	GAPGetBondedStaticAddressesIndication_t *evt = &(container->Data.GAPGetBondedStaticAddressesIndication);

	uint32_t idx = 0;

	/* Store (OG, OC) in ID */
	container->id = 0x4783;

	evt->NbOfDeviceAddresses = pPayload[idx]; idx++;

	if (evt->NbOfDeviceAddresses > 0)
	{
		evt->DeviceAddresses = MEM_BufferAlloc(evt->NbOfDeviceAddresses * 6);

		if (!evt->DeviceAddresses)
		{
			return MEM_ALLOC_ERROR_c;
		}

	}
	else
	{
		evt->DeviceAddresses = NULL;
	}

	FLib_MemCpy(evt->DeviceAddresses, pPayload + idx, evt->NbOfDeviceAddresses * 6); idx += evt->NbOfDeviceAddresses * 6;

	return MEM_SUCCESS_c;
}

/*!*************************************************************************************************
\fn		static memStatus_t Load_GAPLoadEncryptionInformationIndication(bleEvtContainer_t *container, uint8_t *pPayload)
\brief	Returns the encryption key for a bonded device
***************************************************************************************************/
static memStatus_t Load_GAPLoadEncryptionInformationIndication(bleEvtContainer_t *container, uint8_t *pPayload)
{
	GAPLoadEncryptionInformationIndication_t *evt = &(container->Data.GAPLoadEncryptionInformationIndication);

	uint32_t idx = 0;

	/* Store (OG, OC) in ID */
	container->id = 0x4784;

	evt->LtkSize = pPayload[idx]; idx++;

	if (evt->LtkSize > 0)
	{
		evt->Ltk = MEM_BufferAlloc(evt->LtkSize);

		if (!evt->Ltk)
		{
			return MEM_ALLOC_ERROR_c;
		}

	}
	else
	{
		evt->Ltk = NULL;
	}

	FLib_MemCpy(evt->Ltk, pPayload + idx, evt->LtkSize); idx += evt->LtkSize;

	return MEM_SUCCESS_c;
}

/*!*************************************************************************************************
\fn		static memStatus_t Load_GAPLoadCustomPeerInformationIndication(bleEvtContainer_t *container, uint8_t *pPayload)
\brief	Returns the custom peer information in raw data format
***************************************************************************************************/
static memStatus_t Load_GAPLoadCustomPeerInformationIndication(bleEvtContainer_t *container, uint8_t *pPayload)
{
	GAPLoadCustomPeerInformationIndication_t *evt = &(container->Data.GAPLoadCustomPeerInformationIndication);

	uint32_t idx = 0;

	/* Store (OG, OC) in ID */
	container->id = 0x4785;

	FLib_MemCpy(&(evt->InfoSize), pPayload + idx, sizeof(evt->InfoSize)); idx += sizeof(evt->InfoSize);

	if (evt->InfoSize > 0)
	{
		evt->Info = MEM_BufferAlloc(evt->InfoSize);

		if (!evt->Info)
		{
			return MEM_ALLOC_ERROR_c;
		}

	}
	else
	{
		evt->Info = NULL;
	}

	FLib_MemCpy(evt->Info, pPayload + idx, evt->InfoSize); idx += evt->InfoSize;

	return MEM_SUCCESS_c;
}

/*!*************************************************************************************************
\fn		static memStatus_t Load_GAPCheckIfBondedIndication(bleEvtContainer_t *container, uint8_t *pPayload)
\brief	Returns whether a connected peer device is bonded or not
***************************************************************************************************/
static memStatus_t Load_GAPCheckIfBondedIndication(bleEvtContainer_t *container, uint8_t *pPayload)
{
	GAPCheckIfBondedIndication_t *evt = &(container->Data.GAPCheckIfBondedIndication);

	/* Store (OG, OC) in ID */
	container->id = 0x4786;

	FLib_MemCpy(evt, pPayload, sizeof(GAPCheckIfBondedIndication_t));

	return MEM_SUCCESS_c;
}

/*!*************************************************************************************************
\fn		static memStatus_t Load_GAPGetBondedDevicesCountIndication(bleEvtContainer_t *container, uint8_t *pPayload)
\brief	Returns the number of bonded devices
***************************************************************************************************/
static memStatus_t Load_GAPGetBondedDevicesCountIndication(bleEvtContainer_t *container, uint8_t *pPayload)
{
	GAPGetBondedDevicesCountIndication_t *evt = &(container->Data.GAPGetBondedDevicesCountIndication);

	/* Store (OG, OC) in ID */
	container->id = 0x4787;

	FLib_MemCpy(evt, pPayload, sizeof(GAPGetBondedDevicesCountIndication_t));

	return MEM_SUCCESS_c;
}

/*!*************************************************************************************************
\fn		static memStatus_t Load_GAPGetBondedDeviceNameIndication(bleEvtContainer_t *container, uint8_t *pPayload)
\brief	Returns the name of a bonded device
***************************************************************************************************/
static memStatus_t Load_GAPGetBondedDeviceNameIndication(bleEvtContainer_t *container, uint8_t *pPayload)
{
	GAPGetBondedDeviceNameIndication_t *evt = &(container->Data.GAPGetBondedDeviceNameIndication);

	uint32_t idx = 0;

	/* Store (OG, OC) in ID */
	container->id = 0x4788;

	evt->NameSize = pPayload[idx]; idx++;

	if (evt->NameSize > 0)
	{
		evt->Name = MEM_BufferAlloc(evt->NameSize);

		if (!evt->Name)
		{
			return MEM_ALLOC_ERROR_c;
		}

	}
	else
	{
		evt->Name = NULL;
	}

	FLib_MemCpy(evt->Name, pPayload + idx, evt->NameSize); idx += evt->NameSize;

	return MEM_SUCCESS_c;
}

/*!*************************************************************************************************
\fn		static memStatus_t Load_GAPGenericEventInitializationCompleteIndication(bleEvtContainer_t *container, uint8_t *pPayload)
\brief	Controller event - initialization complete
***************************************************************************************************/
static memStatus_t Load_GAPGenericEventInitializationCompleteIndication(bleEvtContainer_t *container, uint8_t *pPayload)
{
	GAPGenericEventInitializationCompleteIndication_t *evt = &(container->Data.GAPGenericEventInitializationCompleteIndication);

	/* Store (OG, OC) in ID */
	container->id = 0x4789;

	FLib_MemCpy(evt, pPayload, sizeof(GAPGenericEventInitializationCompleteIndication_t));

	return MEM_SUCCESS_c;
}

/*!*************************************************************************************************
\fn		static memStatus_t Load_GAPGenericEventInternalErrorIndication(bleEvtContainer_t *container, uint8_t *pPayload)
\brief	Controller event - controller error
***************************************************************************************************/
static memStatus_t Load_GAPGenericEventInternalErrorIndication(bleEvtContainer_t *container, uint8_t *pPayload)
{
	GAPGenericEventInternalErrorIndication_t *evt = &(container->Data.GAPGenericEventInternalErrorIndication);

	/* Store (OG, OC) in ID */
	container->id = 0x478A;

	FLib_MemCpy(evt, pPayload, sizeof(GAPGenericEventInternalErrorIndication_t));

	return MEM_SUCCESS_c;
}

/*!*************************************************************************************************
\fn		static memStatus_t Load_GAPGenericEventAdvertisingSetupFailedIndication(bleEvtContainer_t *container, uint8_t *pPayload)
\brief	Controller event - advertising setup failed
***************************************************************************************************/
static memStatus_t Load_GAPGenericEventAdvertisingSetupFailedIndication(bleEvtContainer_t *container, uint8_t *pPayload)
{
	GAPGenericEventAdvertisingSetupFailedIndication_t *evt = &(container->Data.GAPGenericEventAdvertisingSetupFailedIndication);

	/* Store (OG, OC) in ID */
	container->id = 0x478B;

	FLib_MemCpy(evt, pPayload, sizeof(GAPGenericEventAdvertisingSetupFailedIndication_t));

	return MEM_SUCCESS_c;
}

/*!*************************************************************************************************
\fn		static memStatus_t Load_GAPGenericEventAdvertisingParametersSetupCompleteIndication(bleEvtContainer_t *container, uint8_t *pPayload)
\brief	Controller event - advertising parameters setup completed
***************************************************************************************************/
static memStatus_t Load_GAPGenericEventAdvertisingParametersSetupCompleteIndication(bleEvtContainer_t *container, uint8_t *pPayload)
{
	GAPGenericEventAdvertisingParametersSetupCompleteIndication_t *evt = &(container->Data.GAPGenericEventAdvertisingParametersSetupCompleteIndication);

	/* Store (OG, OC) in ID */
	container->id = 0x478C;

	FLib_MemCpy(evt, pPayload, sizeof(GAPGenericEventAdvertisingParametersSetupCompleteIndication_t));

	return MEM_SUCCESS_c;
}

/*!*************************************************************************************************
\fn		static memStatus_t Load_GAPGenericEventAdvertisingDataSetupCompleteIndication(bleEvtContainer_t *container, uint8_t *pPayload)
\brief	Controller event - advertising data setup completed
***************************************************************************************************/
static memStatus_t Load_GAPGenericEventAdvertisingDataSetupCompleteIndication(bleEvtContainer_t *container, uint8_t *pPayload)
{
	GAPGenericEventAdvertisingDataSetupCompleteIndication_t *evt = &(container->Data.GAPGenericEventAdvertisingDataSetupCompleteIndication);

	/* Store (OG, OC) in ID */
	container->id = 0x478D;

	FLib_MemCpy(evt, pPayload, sizeof(GAPGenericEventAdvertisingDataSetupCompleteIndication_t));

	return MEM_SUCCESS_c;
}

/*!*************************************************************************************************
\fn		static memStatus_t Load_GAPGenericEventWhiteListSizeReadIndication(bleEvtContainer_t *container, uint8_t *pPayload)
\brief	Controller event - White List size
***************************************************************************************************/
static memStatus_t Load_GAPGenericEventWhiteListSizeReadIndication(bleEvtContainer_t *container, uint8_t *pPayload)
{
	GAPGenericEventWhiteListSizeReadIndication_t *evt = &(container->Data.GAPGenericEventWhiteListSizeReadIndication);

	/* Store (OG, OC) in ID */
	container->id = 0x478E;

	FLib_MemCpy(evt, pPayload, sizeof(GAPGenericEventWhiteListSizeReadIndication_t));

	return MEM_SUCCESS_c;
}

/*!*************************************************************************************************
\fn		static memStatus_t Load_GAPGenericEventDeviceAddedToWhiteListIndication(bleEvtContainer_t *container, uint8_t *pPayload)
\brief	Controller event - device added to White List
***************************************************************************************************/
static memStatus_t Load_GAPGenericEventDeviceAddedToWhiteListIndication(bleEvtContainer_t *container, uint8_t *pPayload)
{
	GAPGenericEventDeviceAddedToWhiteListIndication_t *evt = &(container->Data.GAPGenericEventDeviceAddedToWhiteListIndication);

	/* Store (OG, OC) in ID */
	container->id = 0x478F;

	FLib_MemCpy(evt, pPayload, sizeof(GAPGenericEventDeviceAddedToWhiteListIndication_t));

	return MEM_SUCCESS_c;
}

/*!*************************************************************************************************
\fn		static memStatus_t Load_GAPGenericEventDeviceRemovedFromWhiteListIndication(bleEvtContainer_t *container, uint8_t *pPayload)
\brief	Controller event - device removed from White List
***************************************************************************************************/
static memStatus_t Load_GAPGenericEventDeviceRemovedFromWhiteListIndication(bleEvtContainer_t *container, uint8_t *pPayload)
{
	GAPGenericEventDeviceRemovedFromWhiteListIndication_t *evt = &(container->Data.GAPGenericEventDeviceRemovedFromWhiteListIndication);

	/* Store (OG, OC) in ID */
	container->id = 0x4790;

	FLib_MemCpy(evt, pPayload, sizeof(GAPGenericEventDeviceRemovedFromWhiteListIndication_t));

	return MEM_SUCCESS_c;
}

/*!*************************************************************************************************
\fn		static memStatus_t Load_GAPGenericEventWhiteListClearedIndication(bleEvtContainer_t *container, uint8_t *pPayload)
\brief	Controller event - White List cleared
***************************************************************************************************/
static memStatus_t Load_GAPGenericEventWhiteListClearedIndication(bleEvtContainer_t *container, uint8_t *pPayload)
{
	GAPGenericEventWhiteListClearedIndication_t *evt = &(container->Data.GAPGenericEventWhiteListClearedIndication);

	/* Store (OG, OC) in ID */
	container->id = 0x4791;

	FLib_MemCpy(evt, pPayload, sizeof(GAPGenericEventWhiteListClearedIndication_t));

	return MEM_SUCCESS_c;
}

/*!*************************************************************************************************
\fn		static memStatus_t Load_GAPGenericEventRandomAddressReadyIndication(bleEvtContainer_t *container, uint8_t *pPayload)
\brief	Controller event - random address ready
***************************************************************************************************/
static memStatus_t Load_GAPGenericEventRandomAddressReadyIndication(bleEvtContainer_t *container, uint8_t *pPayload)
{
	GAPGenericEventRandomAddressReadyIndication_t *evt = &(container->Data.GAPGenericEventRandomAddressReadyIndication);

	/* Store (OG, OC) in ID */
	container->id = 0x4792;

	FLib_MemCpy(evt, pPayload, sizeof(GAPGenericEventRandomAddressReadyIndication_t));

	return MEM_SUCCESS_c;
}

/*!*************************************************************************************************
\fn		static memStatus_t Load_GAPGenericEventCreateConnectionCanceledIndication(bleEvtContainer_t *container, uint8_t *pPayload)
\brief	Controller event - create connection procedure canceled
***************************************************************************************************/
static memStatus_t Load_GAPGenericEventCreateConnectionCanceledIndication(bleEvtContainer_t *container, uint8_t *pPayload)
{
	GAPGenericEventCreateConnectionCanceledIndication_t *evt = &(container->Data.GAPGenericEventCreateConnectionCanceledIndication);

	/* Store (OG, OC) in ID */
	container->id = 0x4793;

	FLib_MemCpy(evt, pPayload, sizeof(GAPGenericEventCreateConnectionCanceledIndication_t));

	return MEM_SUCCESS_c;
}

/*!*************************************************************************************************
\fn		static memStatus_t Load_GAPGenericEventPublicAddressReadIndication(bleEvtContainer_t *container, uint8_t *pPayload)
\brief	Controller event - public address read
***************************************************************************************************/
static memStatus_t Load_GAPGenericEventPublicAddressReadIndication(bleEvtContainer_t *container, uint8_t *pPayload)
{
	GAPGenericEventPublicAddressReadIndication_t *evt = &(container->Data.GAPGenericEventPublicAddressReadIndication);

	/* Store (OG, OC) in ID */
	container->id = 0x4794;

	FLib_MemCpy(evt, pPayload, sizeof(GAPGenericEventPublicAddressReadIndication_t));

	return MEM_SUCCESS_c;
}

/*!*************************************************************************************************
\fn		static memStatus_t Load_GAPGenericEventAdvTxPowerLevelReadIndication(bleEvtContainer_t *container, uint8_t *pPayload)
\brief	Controller event - advertising transmission power level
***************************************************************************************************/
static memStatus_t Load_GAPGenericEventAdvTxPowerLevelReadIndication(bleEvtContainer_t *container, uint8_t *pPayload)
{
	GAPGenericEventAdvTxPowerLevelReadIndication_t *evt = &(container->Data.GAPGenericEventAdvTxPowerLevelReadIndication);

	/* Store (OG, OC) in ID */
	container->id = 0x4795;

	FLib_MemCpy(evt, pPayload, sizeof(GAPGenericEventAdvTxPowerLevelReadIndication_t));

	return MEM_SUCCESS_c;
}

/*!*************************************************************************************************
\fn		static memStatus_t Load_GAPGenericEventPrivateResolvableAddressVerifiedIndication(bleEvtContainer_t *container, uint8_t *pPayload)
\brief	Controller event - Private Resolvable Address verified
***************************************************************************************************/
static memStatus_t Load_GAPGenericEventPrivateResolvableAddressVerifiedIndication(bleEvtContainer_t *container, uint8_t *pPayload)
{
	GAPGenericEventPrivateResolvableAddressVerifiedIndication_t *evt = &(container->Data.GAPGenericEventPrivateResolvableAddressVerifiedIndication);

	/* Store (OG, OC) in ID */
	container->id = 0x4796;

	FLib_MemCpy(evt, pPayload, sizeof(GAPGenericEventPrivateResolvableAddressVerifiedIndication_t));

	return MEM_SUCCESS_c;
}

/*!*************************************************************************************************
\fn		static memStatus_t Load_GAPGenericEventRandomAddressSetIndication(bleEvtContainer_t *container, uint8_t *pPayload)
\brief	Controller event - Private Resolvable Address verified
***************************************************************************************************/
static memStatus_t Load_GAPGenericEventRandomAddressSetIndication(bleEvtContainer_t *container, uint8_t *pPayload)
{
	GAPGenericEventRandomAddressSetIndication_t *evt = &(container->Data.GAPGenericEventRandomAddressSetIndication);

	/* Store (OG, OC) in ID */
	container->id = 0x4797;

	FLib_MemCpy(evt, pPayload, sizeof(GAPGenericEventRandomAddressSetIndication_t));

	return MEM_SUCCESS_c;
}

/*!*************************************************************************************************
\fn		static memStatus_t Load_GAPAdvertisingEventStateChangedIndication(bleEvtContainer_t *container, uint8_t *pPayload)
\brief	Event received when advertising has been successfully enabled or disabled
***************************************************************************************************/
static memStatus_t Load_GAPAdvertisingEventStateChangedIndication(bleEvtContainer_t *container, uint8_t *pPayload)
{
	GAPAdvertisingEventStateChangedIndication_t *evt = &(container->Data.GAPAdvertisingEventStateChangedIndication);

	/* Store (OG, OC) in ID */
	container->id = 0x4798;

	FLib_MemCpy(evt, pPayload, sizeof(GAPAdvertisingEventStateChangedIndication_t));

	return MEM_SUCCESS_c;
}

/*!*************************************************************************************************
\fn		static memStatus_t Load_GAPAdvertisingEventCommandFailedIndication(bleEvtContainer_t *container, uint8_t *pPayload)
\brief	Event received when advertising could not be enabled or disabled
***************************************************************************************************/
static memStatus_t Load_GAPAdvertisingEventCommandFailedIndication(bleEvtContainer_t *container, uint8_t *pPayload)
{
	GAPAdvertisingEventCommandFailedIndication_t *evt = &(container->Data.GAPAdvertisingEventCommandFailedIndication);

	/* Store (OG, OC) in ID */
	container->id = 0x4799;

	FLib_MemCpy(evt, pPayload, sizeof(GAPAdvertisingEventCommandFailedIndication_t));

	return MEM_SUCCESS_c;
}

/*!*************************************************************************************************
\fn		static memStatus_t Load_GAPScanningEventStateChangedIndication(bleEvtContainer_t *container, uint8_t *pPayload)
\brief	Event received when scanning had been successfully enabled or disabled
***************************************************************************************************/
static memStatus_t Load_GAPScanningEventStateChangedIndication(bleEvtContainer_t *container, uint8_t *pPayload)
{
	GAPScanningEventStateChangedIndication_t *evt = &(container->Data.GAPScanningEventStateChangedIndication);

	/* Store (OG, OC) in ID */
	container->id = 0x479A;

	FLib_MemCpy(evt, pPayload, sizeof(GAPScanningEventStateChangedIndication_t));

	return MEM_SUCCESS_c;
}

/*!*************************************************************************************************
\fn		static memStatus_t Load_GAPScanningEventCommandFailedIndication(bleEvtContainer_t *container, uint8_t *pPayload)
\brief	Event received when scanning could not be enabled or disabled
***************************************************************************************************/
static memStatus_t Load_GAPScanningEventCommandFailedIndication(bleEvtContainer_t *container, uint8_t *pPayload)
{
	GAPScanningEventCommandFailedIndication_t *evt = &(container->Data.GAPScanningEventCommandFailedIndication);

	/* Store (OG, OC) in ID */
	container->id = 0x479B;

	FLib_MemCpy(evt, pPayload, sizeof(GAPScanningEventCommandFailedIndication_t));

	return MEM_SUCCESS_c;
}

/*!*************************************************************************************************
\fn		static memStatus_t Load_GAPScanningEventDeviceScannedIndication(bleEvtContainer_t *container, uint8_t *pPayload)
\brief	Event received when an advertising device has been scanned
***************************************************************************************************/
static memStatus_t Load_GAPScanningEventDeviceScannedIndication(bleEvtContainer_t *container, uint8_t *pPayload)
{
	GAPScanningEventDeviceScannedIndication_t *evt = &(container->Data.GAPScanningEventDeviceScannedIndication);

	uint32_t idx = 0;

	/* Store (OG, OC) in ID */
	container->id = 0x479C;

	evt->AddressType = (GAPScanningEventDeviceScannedIndication_AddressType_t)pPayload[idx]; idx++;
	FLib_MemCpy(evt->Address, pPayload + idx, 6); idx += 6;
	evt->Rssi = pPayload[idx]; idx++;
	evt->DataLength = pPayload[idx]; idx++;

	if (evt->DataLength > 0)
	{
		evt->Data = MEM_BufferAlloc(evt->DataLength);

		if (!evt->Data)
		{
			return MEM_ALLOC_ERROR_c;
		}

	}
	else
	{
		evt->Data = NULL;
	}

	FLib_MemCpy(evt->Data, pPayload + idx, evt->DataLength); idx += evt->DataLength;
	evt->AdvEventType = (GAPScanningEventDeviceScannedIndication_AdvEventType_t)pPayload[idx]; idx++;
	evt->DirectRpaUsed = (bool_t)pPayload[idx]; idx++;


	evt->advertisingAddressResolved = (bool_t)pPayload[idx]; idx++;

	return MEM_SUCCESS_c;
}

/*!*************************************************************************************************
\fn		static memStatus_t Load_GAPConnectionEventConnectedIndication(bleEvtContainer_t *container, uint8_t *pPayload)
\brief	A connection has been established
***************************************************************************************************/
static memStatus_t Load_GAPConnectionEventConnectedIndication(bleEvtContainer_t *container, uint8_t *pPayload)
{
	GAPConnectionEventConnectedIndication_t *evt = &(container->Data.GAPConnectionEventConnectedIndication);

	uint32_t idx = 0;

	/* Store (OG, OC) in ID */
	container->id = 0x479D;

	evt->DeviceId = pPayload[idx]; idx++;
	FLib_MemCpy(&(evt->ConnectionParameters.ConnInterval), pPayload + idx, sizeof(evt->ConnectionParameters.ConnInterval)); idx += sizeof(evt->ConnectionParameters.ConnInterval);
	FLib_MemCpy(&(evt->ConnectionParameters.ConnLatency), pPayload + idx, sizeof(evt->ConnectionParameters.ConnLatency)); idx += sizeof(evt->ConnectionParameters.ConnLatency);
	FLib_MemCpy(&(evt->ConnectionParameters.SupervisionTimeout), pPayload + idx, sizeof(evt->ConnectionParameters.SupervisionTimeout)); idx += sizeof(evt->ConnectionParameters.SupervisionTimeout);
	evt->ConnectionParameters.MasterClockAccuracy = (GAPConnectionEventConnectedIndication_ConnectionParameters_MasterClockAccuracy_t)pPayload[idx]; idx++;
	evt->PeerAddressType = (GAPConnectionEventConnectedIndication_PeerAddressType_t)pPayload[idx]; idx++;
	FLib_MemCpy(evt->PeerAddress, pPayload + idx, 6); idx += 6;
	evt->peerRpaResolved = (bool_t)pPayload[idx]; idx++;


	evt->localRpaUsed = (bool_t)pPayload[idx]; idx++;



	return MEM_SUCCESS_c;
}

/*!*************************************************************************************************
\fn		static memStatus_t Load_GAPConnectionEventPairingRequestIndication(bleEvtContainer_t *container, uint8_t *pPayload)
\brief	A pairing request has been received from the peer Master
***************************************************************************************************/
static memStatus_t Load_GAPConnectionEventPairingRequestIndication(bleEvtContainer_t *container, uint8_t *pPayload)
{
	GAPConnectionEventPairingRequestIndication_t *evt = &(container->Data.GAPConnectionEventPairingRequestIndication);

	/* Store (OG, OC) in ID */
	container->id = 0x479E;

	FLib_MemCpy(evt, pPayload, sizeof(GAPConnectionEventPairingRequestIndication_t));

	return MEM_SUCCESS_c;
}

/*!*************************************************************************************************
\fn		static memStatus_t Load_GAPConnectionEventSlaveSecurityRequestIndication(bleEvtContainer_t *container, uint8_t *pPayload)
\brief	A Slave Security Request has been received from the peer Slave
***************************************************************************************************/
static memStatus_t Load_GAPConnectionEventSlaveSecurityRequestIndication(bleEvtContainer_t *container, uint8_t *pPayload)
{
	GAPConnectionEventSlaveSecurityRequestIndication_t *evt = &(container->Data.GAPConnectionEventSlaveSecurityRequestIndication);

	/* Store (OG, OC) in ID */
	container->id = 0x479F;

	FLib_MemCpy(evt, pPayload, sizeof(GAPConnectionEventSlaveSecurityRequestIndication_t));

	return MEM_SUCCESS_c;
}

/*!*************************************************************************************************
\fn		static memStatus_t Load_GAPConnectionEventPairingResponseIndication(bleEvtContainer_t *container, uint8_t *pPayload)
\brief	A pairing response has been received from the peer Slave
***************************************************************************************************/
static memStatus_t Load_GAPConnectionEventPairingResponseIndication(bleEvtContainer_t *container, uint8_t *pPayload)
{
	GAPConnectionEventPairingResponseIndication_t *evt = &(container->Data.GAPConnectionEventPairingResponseIndication);

	/* Store (OG, OC) in ID */
	container->id = 0x47A0;

	FLib_MemCpy(evt, pPayload, sizeof(GAPConnectionEventPairingResponseIndication_t));

	return MEM_SUCCESS_c;
}

/*!*************************************************************************************************
\fn		static memStatus_t Load_GAPConnectionEventAuthenticationRejectedIndication(bleEvtContainer_t *container, uint8_t *pPayload)
\brief	A link encryption or pairing request has been rejected by the peer Slave
***************************************************************************************************/
static memStatus_t Load_GAPConnectionEventAuthenticationRejectedIndication(bleEvtContainer_t *container, uint8_t *pPayload)
{
	GAPConnectionEventAuthenticationRejectedIndication_t *evt = &(container->Data.GAPConnectionEventAuthenticationRejectedIndication);

	/* Store (OG, OC) in ID */
	container->id = 0x47A1;

	FLib_MemCpy(evt, pPayload, sizeof(GAPConnectionEventAuthenticationRejectedIndication_t));

	return MEM_SUCCESS_c;
}

/*!*************************************************************************************************
\fn		static memStatus_t Load_GAPConnectionEventPasskeyRequestIndication(bleEvtContainer_t *container, uint8_t *pPayload)
\brief	Peer Slave has requested a passkey (maximum 6 digit PIN) for the pairing procedure
***************************************************************************************************/
static memStatus_t Load_GAPConnectionEventPasskeyRequestIndication(bleEvtContainer_t *container, uint8_t *pPayload)
{
	GAPConnectionEventPasskeyRequestIndication_t *evt = &(container->Data.GAPConnectionEventPasskeyRequestIndication);

	/* Store (OG, OC) in ID */
	container->id = 0x47A2;

	FLib_MemCpy(evt, pPayload, sizeof(GAPConnectionEventPasskeyRequestIndication_t));

	return MEM_SUCCESS_c;
}

/*!*************************************************************************************************
\fn		static memStatus_t Load_GAPConnectionEventOobRequestIndication(bleEvtContainer_t *container, uint8_t *pPayload)
\brief	Out-of-Band data must be provided for the pairing procedure
***************************************************************************************************/
static memStatus_t Load_GAPConnectionEventOobRequestIndication(bleEvtContainer_t *container, uint8_t *pPayload)
{
	GAPConnectionEventOobRequestIndication_t *evt = &(container->Data.GAPConnectionEventOobRequestIndication);

	/* Store (OG, OC) in ID */
	container->id = 0x47A3;

	FLib_MemCpy(evt, pPayload, sizeof(GAPConnectionEventOobRequestIndication_t));

	return MEM_SUCCESS_c;
}

/*!*************************************************************************************************
\fn		static memStatus_t Load_GAPConnectionEventPasskeyDisplayIndication(bleEvtContainer_t *container, uint8_t *pPayload)
\brief	The pairing procedure requires this Slave to display the passkey for the Master's user
***************************************************************************************************/
static memStatus_t Load_GAPConnectionEventPasskeyDisplayIndication(bleEvtContainer_t *container, uint8_t *pPayload)
{
	GAPConnectionEventPasskeyDisplayIndication_t *evt = &(container->Data.GAPConnectionEventPasskeyDisplayIndication);

	/* Store (OG, OC) in ID */
	container->id = 0x47A4;

	FLib_MemCpy(evt, pPayload, sizeof(GAPConnectionEventPasskeyDisplayIndication_t));

	return MEM_SUCCESS_c;
}

/*!*************************************************************************************************
\fn		static memStatus_t Load_GAPConnectionEventKeyExchangeRequestIndication(bleEvtContainer_t *container, uint8_t *pPayload)
\brief	The pairing procedure requires the SMP keys to be distributed to the peer
***************************************************************************************************/
static memStatus_t Load_GAPConnectionEventKeyExchangeRequestIndication(bleEvtContainer_t *container, uint8_t *pPayload)
{
	GAPConnectionEventKeyExchangeRequestIndication_t *evt = &(container->Data.GAPConnectionEventKeyExchangeRequestIndication);

	/* Store (OG, OC) in ID */
	container->id = 0x47A5;

	FLib_MemCpy(evt, pPayload, sizeof(GAPConnectionEventKeyExchangeRequestIndication_t));

	return MEM_SUCCESS_c;
}

/*!*************************************************************************************************
\fn		static memStatus_t Load_GAPConnectionEventKeysReceivedIndication(bleEvtContainer_t *container, uint8_t *pPayload)
\brief	SMP keys distributed by the peer during pairing have been received
***************************************************************************************************/
static memStatus_t Load_GAPConnectionEventKeysReceivedIndication(bleEvtContainer_t *container, uint8_t *pPayload)
{
	GAPConnectionEventKeysReceivedIndication_t *evt = &(container->Data.GAPConnectionEventKeysReceivedIndication);

	uint32_t idx = 0;

	/* Store (OG, OC) in ID */
	container->id = 0x47A6;

	evt->DeviceId = pPayload[idx]; idx++;
	evt->Keys.LtkIncluded = (bool_t)pPayload[idx]; idx++;

	if (evt->Keys.LtkIncluded)
	{
		evt->Keys.LtkInfo.LtkSize = pPayload[idx]; idx++;

		if (evt->Keys.LtkInfo.LtkSize > 0)
		{
			evt->Keys.LtkInfo.Ltk = MEM_BufferAlloc(evt->Keys.LtkInfo.LtkSize);

			if (!evt->Keys.LtkInfo.Ltk)
			{
				return MEM_ALLOC_ERROR_c;
			}

		}
		else
		{
			evt->Keys.LtkInfo.Ltk = NULL;
		}

		FLib_MemCpy(evt->Keys.LtkInfo.Ltk, pPayload + idx, evt->Keys.LtkInfo.LtkSize); idx += evt->Keys.LtkInfo.LtkSize;
	}

	evt->Keys.IrkIncluded = (bool_t)pPayload[idx]; idx++;


	evt->Keys.CsrkIncluded = (bool_t)pPayload[idx]; idx++;



	if (evt->Keys.LtkIncluded)
	{
		evt->Keys.RandEdivInfo.RandSize = pPayload[idx]; idx++;

		if (evt->Keys.RandEdivInfo.RandSize > 0)
		{
			evt->Keys.RandEdivInfo.Rand = MEM_BufferAlloc(evt->Keys.RandEdivInfo.RandSize);

			if (!evt->Keys.RandEdivInfo.Rand)
			{
				MEM_BufferFree(evt->Keys.LtkInfo.Ltk);
				return MEM_ALLOC_ERROR_c;
			}

		}
		else
		{
			evt->Keys.RandEdivInfo.Rand = NULL;
		}

		FLib_MemCpy(evt->Keys.RandEdivInfo.Rand, pPayload + idx, evt->Keys.RandEdivInfo.RandSize); idx += evt->Keys.RandEdivInfo.RandSize;
		FLib_MemCpy(&(evt->Keys.RandEdivInfo.Ediv), pPayload + idx, sizeof(evt->Keys.RandEdivInfo.Ediv)); idx += sizeof(evt->Keys.RandEdivInfo.Ediv);
	}


	if (evt->Keys.IrkIncluded)
	{
		evt->Keys.AddressIncluded = (bool_t)pPayload[idx]; idx++;
	}



	if (evt->Keys.AddressIncluded)
	{
		evt->Keys.AddressInfo.DeviceAddressType = (GAPConnectionEventKeysReceivedIndication_Keys_AddressInfo_DeviceAddressType_t)pPayload[idx]; idx++;
		FLib_MemCpy(evt->Keys.AddressInfo.DeviceAddress, pPayload + idx, 6); idx += 6;
	}


	return MEM_SUCCESS_c;
}

/*!*************************************************************************************************
\fn		static memStatus_t Load_GAPConnectionEventLongTermKeyRequestIndication(bleEvtContainer_t *container, uint8_t *pPayload)
\brief	The bonded peer Master has requested link encryption and the LTK must be provided
***************************************************************************************************/
static memStatus_t Load_GAPConnectionEventLongTermKeyRequestIndication(bleEvtContainer_t *container, uint8_t *pPayload)
{
	GAPConnectionEventLongTermKeyRequestIndication_t *evt = &(container->Data.GAPConnectionEventLongTermKeyRequestIndication);

	uint32_t idx = 0;

	/* Store (OG, OC) in ID */
	container->id = 0x47A7;

	evt->DeviceId = pPayload[idx]; idx++;
	FLib_MemCpy(&(evt->Ediv), pPayload + idx, sizeof(evt->Ediv)); idx += sizeof(evt->Ediv);
	evt->RandSize = pPayload[idx]; idx++;

	if (evt->RandSize > 0)
	{
		evt->Rand = MEM_BufferAlloc(evt->RandSize);

		if (!evt->Rand)
		{
			return MEM_ALLOC_ERROR_c;
		}

	}
	else
	{
		evt->Rand = NULL;
	}

	FLib_MemCpy(evt->Rand, pPayload + idx, evt->RandSize); idx += evt->RandSize;

	return MEM_SUCCESS_c;
}

/*!*************************************************************************************************
\fn		static memStatus_t Load_GAPConnectionEventEncryptionChangedIndication(bleEvtContainer_t *container, uint8_t *pPayload)
\brief	Link's encryption state has changed, e.g. during pairing or after a reconnection with a bonded peer
***************************************************************************************************/
static memStatus_t Load_GAPConnectionEventEncryptionChangedIndication(bleEvtContainer_t *container, uint8_t *pPayload)
{
	GAPConnectionEventEncryptionChangedIndication_t *evt = &(container->Data.GAPConnectionEventEncryptionChangedIndication);

	/* Store (OG, OC) in ID */
	container->id = 0x47A8;

	FLib_MemCpy(evt, pPayload, sizeof(GAPConnectionEventEncryptionChangedIndication_t));

	return MEM_SUCCESS_c;
}

/*!*************************************************************************************************
\fn		static memStatus_t Load_GAPConnectionEventPairingCompleteIndication(bleEvtContainer_t *container, uint8_t *pPayload)
\brief	Pairing procedure is complete, either successfully or with failure
***************************************************************************************************/
static memStatus_t Load_GAPConnectionEventPairingCompleteIndication(bleEvtContainer_t *container, uint8_t *pPayload)
{
	GAPConnectionEventPairingCompleteIndication_t *evt = &(container->Data.GAPConnectionEventPairingCompleteIndication);

	uint32_t idx = 0;

	/* Store (OG, OC) in ID */
	container->id = 0x47A9;

	evt->DeviceId = pPayload[idx]; idx++;
	evt->PairingStatus = (GAPConnectionEventPairingCompleteIndication_PairingStatus_t)pPayload[idx]; idx++;

	switch (evt->PairingStatus)
	{
		case GAPConnectionEventPairingCompleteIndication_PairingStatus_PairingSuccessful:
			evt->PairingData.PairingSuccessful_WithBonding = (bool_t)pPayload[idx]; idx++;
			break;

		case GAPConnectionEventPairingCompleteIndication_PairingStatus_PairingFailed:
			FLib_MemCpy(&(evt->PairingData.PairingFailed_FailReason), pPayload + idx, 2); idx += 2;
			break;
	}

	return MEM_SUCCESS_c;
}

/*!*************************************************************************************************
\fn		static memStatus_t Load_GAPConnectionEventDisconnectedIndication(bleEvtContainer_t *container, uint8_t *pPayload)
\brief	A connection has been terminated
***************************************************************************************************/
static memStatus_t Load_GAPConnectionEventDisconnectedIndication(bleEvtContainer_t *container, uint8_t *pPayload)
{
	GAPConnectionEventDisconnectedIndication_t *evt = &(container->Data.GAPConnectionEventDisconnectedIndication);

	/* Store (OG, OC) in ID */
	container->id = 0x47AA;

	FLib_MemCpy(evt, pPayload, sizeof(GAPConnectionEventDisconnectedIndication_t));

	return MEM_SUCCESS_c;
}

/*!*************************************************************************************************
\fn		static memStatus_t Load_GAPConnectionEventRssiReadIndication(bleEvtContainer_t *container, uint8_t *pPayload)
\brief	RSSI for an active connection has been read
***************************************************************************************************/
static memStatus_t Load_GAPConnectionEventRssiReadIndication(bleEvtContainer_t *container, uint8_t *pPayload)
{
	GAPConnectionEventRssiReadIndication_t *evt = &(container->Data.GAPConnectionEventRssiReadIndication);

	/* Store (OG, OC) in ID */
	container->id = 0x47AB;

	FLib_MemCpy(evt, pPayload, sizeof(GAPConnectionEventRssiReadIndication_t));

	return MEM_SUCCESS_c;
}

/*!*************************************************************************************************
\fn		static memStatus_t Load_GAPConnectionEventTxPowerLevelReadIndication(bleEvtContainer_t *container, uint8_t *pPayload)
\brief	TX power level for an active connection has been read
***************************************************************************************************/
static memStatus_t Load_GAPConnectionEventTxPowerLevelReadIndication(bleEvtContainer_t *container, uint8_t *pPayload)
{
	GAPConnectionEventTxPowerLevelReadIndication_t *evt = &(container->Data.GAPConnectionEventTxPowerLevelReadIndication);

	/* Store (OG, OC) in ID */
	container->id = 0x47AC;

	FLib_MemCpy(evt, pPayload, sizeof(GAPConnectionEventTxPowerLevelReadIndication_t));

	return MEM_SUCCESS_c;
}

/*!*************************************************************************************************
\fn		static memStatus_t Load_GAPConnectionEventPowerReadFailureIndication(bleEvtContainer_t *container, uint8_t *pPayload)
\brief	Power reading could not be performed
***************************************************************************************************/
static memStatus_t Load_GAPConnectionEventPowerReadFailureIndication(bleEvtContainer_t *container, uint8_t *pPayload)
{
	GAPConnectionEventPowerReadFailureIndication_t *evt = &(container->Data.GAPConnectionEventPowerReadFailureIndication);

	/* Store (OG, OC) in ID */
	container->id = 0x47AD;

	FLib_MemCpy(evt, pPayload, sizeof(GAPConnectionEventPowerReadFailureIndication_t));

	return MEM_SUCCESS_c;
}

/*!*************************************************************************************************
\fn		static memStatus_t Load_GAPConnectionEventParameterUpdateRequestIndication(bleEvtContainer_t *container, uint8_t *pPayload)
\brief	A connection parameter update request has been received
***************************************************************************************************/
static memStatus_t Load_GAPConnectionEventParameterUpdateRequestIndication(bleEvtContainer_t *container, uint8_t *pPayload)
{
	GAPConnectionEventParameterUpdateRequestIndication_t *evt = &(container->Data.GAPConnectionEventParameterUpdateRequestIndication);

	/* Store (OG, OC) in ID */
	container->id = 0x47AE;

	FLib_MemCpy(evt, pPayload, sizeof(GAPConnectionEventParameterUpdateRequestIndication_t));

	return MEM_SUCCESS_c;
}

/*!*************************************************************************************************
\fn		static memStatus_t Load_GAPConnectionEventParameterUpdateCompleteIndication(bleEvtContainer_t *container, uint8_t *pPayload)
\brief	The connection has new parameters
***************************************************************************************************/
static memStatus_t Load_GAPConnectionEventParameterUpdateCompleteIndication(bleEvtContainer_t *container, uint8_t *pPayload)
{
	GAPConnectionEventParameterUpdateCompleteIndication_t *evt = &(container->Data.GAPConnectionEventParameterUpdateCompleteIndication);

	/* Store (OG, OC) in ID */
	container->id = 0x47AF;

	FLib_MemCpy(evt, pPayload, sizeof(GAPConnectionEventParameterUpdateCompleteIndication_t));

	return MEM_SUCCESS_c;
}

/*!*************************************************************************************************
\fn		static memStatus_t Load_GAPConnectionEventLeDataLengthChangedIndication(bleEvtContainer_t *container, uint8_t *pPayload)
\brief	The new TX/RX Data Length paramaters
***************************************************************************************************/
static memStatus_t Load_GAPConnectionEventLeDataLengthChangedIndication(bleEvtContainer_t *container, uint8_t *pPayload)
{
	GAPConnectionEventLeDataLengthChangedIndication_t *evt = &(container->Data.GAPConnectionEventLeDataLengthChangedIndication);

	/* Store (OG, OC) in ID */
	container->id = 0x47B0;

	FLib_MemCpy(evt, pPayload, sizeof(GAPConnectionEventLeDataLengthChangedIndication_t));

	return MEM_SUCCESS_c;
}

/*!*************************************************************************************************
\fn		static memStatus_t Load_GAPConnectionEventLeScOobDataRequestIndication(bleEvtContainer_t *container, uint8_t *pPayload)
\brief	Event sent to request LE SC OOB Data (r, Cr and Addr) received from a peer
***************************************************************************************************/
static memStatus_t Load_GAPConnectionEventLeScOobDataRequestIndication(bleEvtContainer_t *container, uint8_t *pPayload)
{
	GAPConnectionEventLeScOobDataRequestIndication_t *evt = &(container->Data.GAPConnectionEventLeScOobDataRequestIndication);

	/* Store (OG, OC) in ID */
	container->id = 0x47B1;

	FLib_MemCpy(evt, pPayload, sizeof(GAPConnectionEventLeScOobDataRequestIndication_t));

	return MEM_SUCCESS_c;
}

/*!*************************************************************************************************
\fn		static memStatus_t Load_GAPConnectionEventLeScDisplayNumericValueIndication(bleEvtContainer_t *container, uint8_t *pPayload)
\brief	Event sent to display and confirm a Numeric Comparison Value when using the LE SC Numeric Comparison pairing method
***************************************************************************************************/
static memStatus_t Load_GAPConnectionEventLeScDisplayNumericValueIndication(bleEvtContainer_t *container, uint8_t *pPayload)
{
	GAPConnectionEventLeScDisplayNumericValueIndication_t *evt = &(container->Data.GAPConnectionEventLeScDisplayNumericValueIndication);

	/* Store (OG, OC) in ID */
	container->id = 0x47B2;

	FLib_MemCpy(evt, pPayload, sizeof(GAPConnectionEventLeScDisplayNumericValueIndication_t));

	return MEM_SUCCESS_c;
}

/*!*************************************************************************************************
\fn		static memStatus_t Load_GAPConnectionEventLeScKeypressNotificationIndication(bleEvtContainer_t *container, uint8_t *pPayload)
\brief	Remote Keypress Notification recieved during Passkey Entry Pairing Method
***************************************************************************************************/
static memStatus_t Load_GAPConnectionEventLeScKeypressNotificationIndication(bleEvtContainer_t *container, uint8_t *pPayload)
{
	GAPConnectionEventLeScKeypressNotificationIndication_t *evt = &(container->Data.GAPConnectionEventLeScKeypressNotificationIndication);

	/* Store (OG, OC) in ID */
	container->id = 0x47B3;

	FLib_MemCpy(evt, pPayload, sizeof(GAPConnectionEventLeScKeypressNotificationIndication_t));

	return MEM_SUCCESS_c;
}

/*!*************************************************************************************************
\fn		static memStatus_t Load_GAPGenericEventControllerResetCompleteIndication(bleEvtContainer_t *container, uint8_t *pPayload)
\brief	The controller has been reset
***************************************************************************************************/
static memStatus_t Load_GAPGenericEventControllerResetCompleteIndication(bleEvtContainer_t *container, uint8_t *pPayload)
{
	GAPGenericEventControllerResetCompleteIndication_t *evt = &(container->Data.GAPGenericEventControllerResetCompleteIndication);

	/* Store (OG, OC) in ID */
	container->id = 0x47B4;

	FLib_MemCpy(evt, pPayload, sizeof(GAPGenericEventControllerResetCompleteIndication_t));

	return MEM_SUCCESS_c;
}

/*!*************************************************************************************************
\fn		static memStatus_t Load_GAPLeScPublicKeyRegeneratedIndication(bleEvtContainer_t *container, uint8_t *pPayload)
\brief	The private/public key pair used for LE Secure Connections pairing has been regenerated
***************************************************************************************************/
static memStatus_t Load_GAPLeScPublicKeyRegeneratedIndication(bleEvtContainer_t *container, uint8_t *pPayload)
{
	GAPLeScPublicKeyRegeneratedIndication_t *evt = &(container->Data.GAPLeScPublicKeyRegeneratedIndication);

	/* Store (OG, OC) in ID */
	container->id = 0x47B5;

	FLib_MemCpy(evt, pPayload, sizeof(GAPLeScPublicKeyRegeneratedIndication_t));

	return MEM_SUCCESS_c;
}

/*!*************************************************************************************************
\fn		static memStatus_t Load_GAPGenericEventLeScLocalOobDataIndication(bleEvtContainer_t *container, uint8_t *pPayload)
\brief	Local OOB data used for LE Secure Connections pairing
***************************************************************************************************/
static memStatus_t Load_GAPGenericEventLeScLocalOobDataIndication(bleEvtContainer_t *container, uint8_t *pPayload)
{
	GAPGenericEventLeScLocalOobDataIndication_t *evt = &(container->Data.GAPGenericEventLeScLocalOobDataIndication);

	/* Store (OG, OC) in ID */
	container->id = 0x47B6;

	FLib_MemCpy(evt, pPayload, sizeof(GAPGenericEventLeScLocalOobDataIndication_t));

	return MEM_SUCCESS_c;
}

/*!*************************************************************************************************
\fn		static memStatus_t Load_GAPGenericEventControllerPrivacyStateChangedIndication(bleEvtContainer_t *container, uint8_t *pPayload)
\brief	The Controller Privacy was enabled or disabled
***************************************************************************************************/
static memStatus_t Load_GAPGenericEventControllerPrivacyStateChangedIndication(bleEvtContainer_t *container, uint8_t *pPayload)
{
	GAPGenericEventControllerPrivacyStateChangedIndication_t *evt = &(container->Data.GAPGenericEventControllerPrivacyStateChangedIndication);

	/* Store (OG, OC) in ID */
	container->id = 0x47B7;

	FLib_MemCpy(evt, pPayload, sizeof(GAPGenericEventControllerPrivacyStateChangedIndication_t));

	return MEM_SUCCESS_c;
}

/*!*************************************************************************************************
\fn		static memStatus_t Load_GAPGetBondedDevicesIdentityInformationIndication(bleEvtContainer_t *container, uint8_t *pPayload)
\brief	Returns a list of the identity information of bonded devices
***************************************************************************************************/
static memStatus_t Load_GAPGetBondedDevicesIdentityInformationIndication(bleEvtContainer_t *container, uint8_t *pPayload)
{
	GAPGetBondedDevicesIdentityInformationIndication_t *evt = &(container->Data.GAPGetBondedDevicesIdentityInformationIndication);

	uint32_t idx = 0;

	/* Store (OG, OC) in ID */
	container->id = 0x4783;

	evt->NbOfDeviceIdentityAddresses = pPayload[idx]; idx++;

	if (evt->NbOfDeviceIdentityAddresses > 0)
	{
		evt->IdentityAddresses = MEM_BufferAlloc(evt->NbOfDeviceIdentityAddresses * sizeof(evt->IdentityAddresses[0]));

		if (!evt->IdentityAddresses)
		{
			return MEM_ALLOC_ERROR_c;
		}

	}
	else
	{
		evt->IdentityAddresses = NULL;
	}


	for (uint32_t i = 0; i < evt->NbOfDeviceIdentityAddresses; i++)
	{
		evt->IdentityAddresses[i].IdentityAddressType = (GAPGetBondedDevicesIdentityInformationIndication_IdentityAddresses_IdentityAddressType_t)pPayload[idx]; idx++;
		FLib_MemCpy(evt->IdentityAddresses[i].IdentityAddress, pPayload + idx, 6); idx += 6;
		FLib_MemCpy(evt->IdentityAddresses[i].Irk, pPayload + idx, 16); idx += 16;
	}

	return MEM_SUCCESS_c;
}

void KHC_BLE_RX_MsgHandler(void *pData, void *param, uint32_t fsciInterface)
{
	if (!pData || !param)
	{
		return;
	}

	bleEvtContainer_t *container = (bleEvtContainer_t *)param;
	uint8_t og = *((uint8_t *)pData + 1);
	uint8_t oc = *((uint8_t *)pData + 2);
	uint8_t dataSize = *((uint8_t *)pData + 3) + (*((uint8_t *)pData + 4) << 8);
	uint8_t *pPayload = (uint8_t *)pData + 5;
	uint16_t id = (og << 8) + oc, i;

	if (dataSize == 0)
	{
		MEM_BufferFree(pData);
		return;
	}

	for (i = 0; i < sizeof(evtHandlerTbl) / sizeof(evtHandlerTbl[0]); i++)
	{
		if (evtHandlerTbl[i].id == id)
		{
			evtHandlerTbl[i].handlerFunc(container, pPayload);
			break;
		}
	}

	/* Clear received packet */
	MEM_BufferFree(pData);
}

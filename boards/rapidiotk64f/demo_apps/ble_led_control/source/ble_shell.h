/*! *********************************************************************************
 * \defgroup Bluetooth Shell Application
 * @{
 ********************************************************************************** */
/*!
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 *
 * \file
 *
 * This file is the interface file for the Bluetooth Shell Application
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

#ifndef _APP_H_
#define _APP_H_

/*************************************************************************************
**************************************************************************************
* Public macros
**************************************************************************************
*************************************************************************************/
#define UuidToArray(arr, value) \
        arr[1] = value >> 8; \
        arr[0] = value & 0x00FF;

#define PAIRING_CODE_SIZE 6

/************************************************************************************
*************************************************************************************
* Public memory declarations
*************************************************************************************
********************************************************************************** */
enum RpkDemoState_t
{
    gDemoAddServices_c,
    gDemoRegisterCallback,
    gDemoReadPublicAddresss_c,
    gDemoSetAdvData_c,
	gDemoFindGapServiceHandle_c,
	gDemoFindDeviceNameHandle_c,
	gDemoWriteDeviceName_c,
	gDemoFindBatteryServiceHandle_c,
    gDemoFindBatteryLevelHandle_c,
    gDemoFindWeatherServiceHandle_c,
    gDemoFindBatteryStatusHandle_c,
    gDemoFindTempMeasurementHandle_c,
    gDemoFindHumidityHandle_c,
    gDemoFindPressureHandle_c,
    gDemoFindAirQualityHandle_c,
    gDemoFindAmbientLightHandle_c,
    gDemoWriteTempAttribute_c,
    gDemoWriteHumidityParam_c,
    gDemoWritePressureParam_c,
    gDemoWriteAirParam_c,
    gDemoWriteLightParam_c,
    gDemoFindMotionServiceHandle_c,
    gDemoFindGyroscopeHandle_c,
    gDemoFindAccelerometerHandle_c,
    gDemoFindMagnetometerHandle_c,
    gDemoWriteGyroscope_c,
    gDemoWriteAccelerometer_c,
    gDemoWriteMagnetometer_c,
    gDemoFindInteractionServiceHandler_c,
    gDemoFindLEDHandle_c,
    gDemoFindBuzzerHandle_c,
    gDemoFindBacklightHandle_c,
    gDemoFindAuthServiceHandle_c,
    gDemoFindAuthUidHandle_c,
    gDemoFindAuthCertHandle_c,
    gDemoFindAuthChallengeHandle_c,
    gDemoFindAuthResponseHandle_c,
    gDemoFindOTAServiceHandle_c,
    gDemoFindOTAControlPointHandle_c,
    gDemoFindOTADataHandle_c,
    gDemoFindOTAControlPointCCCDHandle_c,
    gDemoWriteUidAttribute_c,
    gDemoWriteCertificateAttribute_c,
    gDemoRegisterNotification_c,
    gDemoSetPairingParameters,
    gDemoSetAdvParam_c,
    gDemoStartAdvertising_c,
    gDemoPrintMessage_c,
    gDemoWaitingConnection_c,
    gDemoConnectionEstablish_c,
    gDemoError_c
};

enum RpkNotifyState_t
{
    gNotifyTemperature_c,
    gNotifyHumidity_c,
    gNotifyPressure_c,
    gNotifyAirQuality_c,
    gNotifyAmbientLight_c,
    gNotifyGyroscope_c,
    gNotifyAccelerometer_c,
    gNotifyMagnetometer_c,
    gNotifyBatteryLevel_c,
    gNotifyBatteryStatus_c,
    gNotifyUid_c,
    gNotifyCert_c,
    gNotifyResponse_c
};

/************************************************************************************
*************************************************************************************
* Public prototypes
*************************************************************************************
************************************************************************************/

#ifdef __cplusplus
extern "C" {
#endif


void BleApp_Init(void);

void BleApp_Terminate(void);

void BleApp_DemoHrs(void);
void BleApp_DemoHrsNotify(void);
void BleApp_DemoOtap(void);
void BleApp_DemoRpkNotify(void);
void BleApp_DemoRpkWriteCallback(void *data);
void BleApp_DisplayCode(uint32_t code);
void BleApp_SentKey(void *data);
void BleApp_ProvideLongTermKey(void *data);
void BleApp_DemoRpkWithoutResponseCallback(void *data);
void BleApp_DemoRpk(void);

uint8_t BleApp_ParseHexValue(char *pInput);

/*!*************************************************************************************************
\fn     void SHELL_BleEventNotify(void *container)
\brief  Print to host's shell information about BLE events coming from black-box

\param  [in]    container       the received event
***************************************************************************************************/
void SHELL_BleEventNotify(void *container);

#ifdef __cplusplus
}
#endif


#endif /* _APP_H_ */

/*! *********************************************************************************
 * @}
 ********************************************************************************** */

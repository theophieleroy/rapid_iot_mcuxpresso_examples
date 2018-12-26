/*! *********************************************************************************
 * \addtogroup Bluetooth Shell Application
 * @{
 ********************************************************************************** */
/*!
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 *
 * \file
 *
 * This file is the source file for the Bluetooth Shell Application
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
#include <stdio.h>
#include <stdlib.h>

/* Framework / Drivers */
#include "LED.h"
#include "TimersManager.h"
#include "FunctionLib.h"
#include "fsl_os_abstraction.h"
#include "shell.h"
#include "Panic.h"
#include "MemManager.h"
#include "board.h"
#include "sensors.h"

/* Shell APIs */
#include "shell_gap.h"
#include "shell_gatt.h"
#include "shell_gattdb.h"

#include "ble_sig_defines.h"
#include "ble_shell.h"
#include "cmd_ble.h"
#include "peripherals.h"
#include "rgb_led.h"
#include "ble_otap_client.h"
#include "otap_interface.h"
#include "GUI.h"

#include "FsciInterface.h"
/************************************************************************************
*************************************************************************************
* Private macros
*************************************************************************************
************************************************************************************/
#define RPK_DEMO_TIMEOUT             300    /* ms */
#define DEVICE_NAME_PREFIX           "RPK-" /* The prefix will be added in front of
                                            the last 2 bytes of the MAC address */
#define DEVICE_PREFIX_LENGTH         4      /* Length of DEVICE_NAME_PREFIX */
#define gFastConnMinAdvInterval_c    32     /* 20 ms */
#define gFastConnMaxAdvInterval_c    48     /* 30 ms */
#define mDemoReportInterval_c        (0.3)  /* RPK demo report interval in seconds */
/************************************************************************************
*************************************************************************************
* Public type definitions
*************************************************************************************
************************************************************************************/

/************************************************************************************
*************************************************************************************
* Private functions prototypes
*************************************************************************************
************************************************************************************/
static int8_t BleApp_DemoCommand(uint8_t argc, char *argv[]);

static void BleApp_StartRpkDemoSm(void *param);
static void BleApp_NotifyRpkSm(void *param);
static void BleApp_Notify(uint16_t hValue, uint8_t *value, uint8_t ValueLenght);
/************************************************************************************
*************************************************************************************
* Private memory declarations
*************************************************************************************
************************************************************************************/
static uint16_t hServiceGap;
static uint16_t hServiceBattery;
static uint16_t hServiceWeather;
static uint16_t hServiceMotion;
static uint16_t hServiceInteraction;
static uint16_t hServiceAuthentication;

static uint16_t hValueDeviceName;
static uint16_t hValueBatteryStatus;
static uint16_t hValueBatteryLevel;
static uint16_t hValueTempMeasurement;
static uint16_t hValueHumidity;
static uint16_t hValuePressure;
static uint16_t hValueAirQuality;
static uint16_t hValueAmbientLight;
static uint16_t hValueGyroscope;
static uint16_t hValueAccelerometer;
static uint16_t hValueMagnetometer;
static uint16_t hValueLED;
static uint16_t hValueBuzzer;
static uint16_t hValueBacklight;
static uint16_t hValueUid;
static uint16_t hValueCertificate;
static uint16_t hValueChallenge;
static uint16_t hValueResponse;
static char device_name[DEVICE_PREFIX_LENGTH + 5] = {DEVICE_NAME_PREFIX};

const char mpGapHelp[] = "\r\n"
                         "gap address\r\n"
                         "gap advcfg [-interval intervalInMs] [-type type]\r\n"
                         "gap advstart\r\n"
                         "gap advstop\r\n"
                         "gap advdata <-erase | [type] [payload]>\r\n"
                         "gap scanstart\r\n"
                         "gap scanstop\r\n"
                         "gap scancfg [-type type] [-interval intervalInMs] [-window windowInMs]\r\n"
                         "gap scandata [-erase] [-add type payload]\r\n"
                         "gap connectcfg [-interval intervalInMs] [-latency latency] [-timeout timeout]\r\n"
                         "gap connect scannedDeviceId\r\n"
                         "gap disconnect\r\n"
                         "gap connupdate mininterval maxinterval latency timeout\r\n"
                         "gap paircfg [-usebonding usebonding] [-seclevel seclevel] [-keyflags flags]\r\n"
                         "gap pair\r\n"
                         "gap enterpin pin\r\n"
                         "gap bonds [-erase] [-remove deviceIndex]\r\n";

const char mpGattHelp[] = "\r\n"
                          "gatt discover [-all] [-service serviceUuid16InHex] \r\n"
                          "gatt read handle\r\n"
                          "gatt write handle valueInHex\r\n"
                          "gatt writecmd handle valueInHex\r\n"
                          "gatt notify handle\r\n"
                          "gatt indicate handle valueInHex\r\n";

const char mpGattDbHelp[] = "\r\n"
                            "gattdb read handle\r\n"
                            "gattdb write handle valueInHex\r\n"
                            "gattdb addservice <gatt | gap | battery | weather | motion | devinfo | serviceUuid16InHex>\r\n"
                            "gattdb erase\r\n";

const char mpBleDemoHelp[] = "\r\n"
                             "bledemo rpk\r\n"
                             "bledemo ota\r\n";

/* Shell */
const cmd_tbl_t mGapCmd =
{
    .name = "gap",
    .maxargs = 11,
    .repeatable = 1,
    .cmd = ShellGap_Command,
    .usage = (char *)mpGapHelp,
    .help = "Contains commands for advertising, scanning, connecting, pairing or disconnecting."
};

const cmd_tbl_t mGattCmd =
{
    .name = "gatt",
    .maxargs = 11,
    .repeatable = 1,
    .cmd = ShellGatt_Command,
    .usage = (char *)mpGattHelp,
    .help = "Contains commands for service discovery, read, write, notify and indicate. "
    "Values in hex must be formatted as 0xXX..X"
};

const cmd_tbl_t mGattDbCmd =
{
    .name = "gattdb",
    .maxargs = 11,
    .repeatable = 1,
    .cmd = ShellGattDb_Command,
    .usage = (char *)mpGattDbHelp,
    .help = "Contains commands for adding services, reading and writing characteristics on the local database."
    "Values in hex must be formatted as 0xXX..X"
};

const cmd_tbl_t mBleDemoCmd =
{
    .name = "bledemo",
    .maxargs = 3,
    .repeatable = 1,
    .cmd = BleApp_DemoCommand,
    .usage = (char *)mpBleDemoHelp,
    .help = "Commands that implement complete profiles. RPK (rpk) profiles are supported."
};

/* RPK variables */
static tmrTimerID_t mRpkDemoTimerID = gTmrInvalidTimerID_c;
static tmrTimerID_t mRpkNotifyTimerID = gTmrInvalidTimerID_c;

extern int16_t read_accel_value(bool_t);

/* Authentication Challange Received Flag */
static bool_t mChallengeReceived = FALSE;

/* Authentication Soft Reset Flag */
static bool_t mAuthSetSoftReset = FALSE;

/************************************************************************************
*************************************************************************************
* Public memory declarations
*************************************************************************************
************************************************************************************/

/* They are used in the OTA client */
uint16_t hServiceOTA;
uint16_t hValueControlPointOTA;
uint16_t hValueDataOTA;
uint16_t hValueOTACCCD;
uint16_t ota_demo = 0;

extern uint8_t bleAddress[];
extern char gDeviceName[];
extern GAPSetAdvertisingDataRequest_t gAppAdvertisingData;
extern GAPSetAdvertisingParametersRequest_t gAdvParams;
extern GAPPairRequest_t gPairingParameters;
extern uint16_t gLatestHandle;
extern uint16_t gCccdHandle;
extern bool_t mSuppressBleEventPrint;
extern GAPSetDefaultPairingParametersRequest_t gParingParametersRequest;

/* Weather */
extern uint8_t uuid_service_weather[];
extern uint8_t uuid_characteristic_airQuality[];
extern uint8_t uuid_characteristic_ambientLight[];

/* Motion */
extern uint8_t uuid_service_motion[];
extern uint8_t uuid_characteristic_gyroscope[];
extern uint8_t uuid_characteristic_accelerometer[];
extern uint8_t uuid_characteristic_magnetometer[];

/* Interaction */
extern uint8_t uuid_service_interaction[16];
extern uint8_t uuid_characteristic_led[16];
extern uint8_t uuid_characteristic_buzzer[16];
extern uint8_t uuid_characteristic_backlight[16];

/* Authentication */
extern uint8_t uuid_service_auth[16];
extern uint8_t uuid_char_auth_uid[16];
extern uint8_t uuid_char_auth_cert[16];
extern uint8_t uuid_char_auth_chl[16];
extern uint8_t uuid_char_auth_res[16];

/* OTAP */
extern uint8_t uuid_service_otap[16];
extern uint8_t uuid_char_otap_control_point[16];
extern uint8_t uuid_char_otap_data[16];

extern gapSmpKeys_t gSmpKeys;

enum RpkNotifyState_t gRpkNotifyState = gNotifyTemperature_c;
enum RpkDemoState_t gRpkDemoState = gDemoAddServices_c;

/************************************************************************************
*************************************************************************************
* Public functions
*************************************************************************************
************************************************************************************/

/*! *********************************************************************************
* \brief    Initializes application specific functionality before the BLE stack init.
*
********************************************************************************** */
void BleApp_Init(void)
{
    /* UI */
    shell_register_function((cmd_tbl_t *)&mGapCmd);
    shell_register_function((cmd_tbl_t *)&mGattCmd);
    shell_register_function((cmd_tbl_t *)&mGattDbCmd);
    shell_register_function((cmd_tbl_t *)&mBleDemoCmd);
}

/*! *********************************************************************************
* \brief    Marks all global variables as invalid.
*
********************************************************************************** */
void BleApp_Terminate(void)
{
    // clear device name
    gDeviceName[0] = '\0';

    // clear advertising and scanning data
    for (uint8_t i = 0; i < gAppAdvertisingData.AdvertisingData.NbOfAdStructures; i++)
    {
        MEM_BufferFree(gAppAdvertisingData.AdvertisingData.AdStructures[i].Data);
    }

    for (uint8_t i = 0; i < gAppAdvertisingData.ScanResponseData.NbOfAdStructures; i++)
    {
        MEM_BufferFree(gAppAdvertisingData.ScanResponseData.AdStructures[i].Data);
    }

    MEM_BufferFree(gAppAdvertisingData.AdvertisingData.AdStructures);
    MEM_BufferFree(gAppAdvertisingData.ScanResponseData.AdStructures);
    FLib_MemSet(&gAppAdvertisingData.AdvertisingData, 0, sizeof(gAppAdvertisingData.AdvertisingData));
    FLib_MemSet(&gAppAdvertisingData.ScanResponseData, 0, sizeof(gAppAdvertisingData.ScanResponseData));
    gAppAdvertisingData.AdvertisingDataIncluded = FALSE;
    gAppAdvertisingData.ScanResponseDataIncluded = FALSE;

    gLatestHandle = INVALID_HANDLE;


    mSuppressBleEventPrint = FALSE;
    shell_change_prompt("$ ");
}

uint8_t BleApp_ParseHexValue(char *pInput)
{
    uint8_t i, length = strlen(pInput);
    uint32_t value;

    /* If the hex misses a 0, return error */
    if (length % 2)
    {
        return 0;
    }

    if (!strncmp(pInput, "0x", 2))
    {
        length -= 2;

        /* Save as little endian hex value */
        value = strtoul(pInput + 2, NULL, 16);

        FLib_MemCpy(pInput, &value, sizeof(uint32_t));

        return (length - 2);
    }
    else
    {
        char octet[2];

        /* Save as big endian hex */
        for (i = 0; i < length / 2; i++)
        {
            FLib_MemCpy(octet, &pInput[i * 2], 2);

            pInput[i] = strtoul(octet, NULL, 16);
        }

        return length / 2;
    }

}

void BleApp_DemoRpk(void)
{
    if (mRpkDemoTimerID == gTmrInvalidTimerID_c)
    {
        mRpkDemoTimerID = TMR_AllocateTimer();
    }

    TMR_StartSingleShotTimer(mRpkDemoTimerID, RPK_DEMO_TIMEOUT, BleApp_StartRpkDemoSm, NULL);
}

void BleApp_SentKey(void* data)
{
    GAPConnectionEventKeyExchangeRequestIndication_t *rcvData = data;
    GAPSendSmpKeysRequest_t *req;
    req = MEM_BufferAlloc(sizeof(GAPSendSmpKeysRequest_t));
    if (!req)
    {
        shell_write("\r\n-->  Send Key: Insufficient memory. ");
        return;
    }
    FLib_MemSet(req, 0x00, sizeof(GAPSendSmpKeysRequest_t));

    req->DeviceId = rcvData->DeviceId;

    switch (rcvData->RequestedKeys)
    {
        case GAPConnectionEventKeyExchangeRequestIndication_RequestedKeys_gLtk_c:
            req->Keys.LtkIncluded = 1;

            req->Keys.LtkInfo.Ltk = MEM_BufferAlloc(mcEncryptionKeySize_c);
            if (!req->Keys.LtkInfo.Ltk)
            {
                shell_write("\r\n-->  Send Key: Insufficient memory. ");
                MEM_BufferFree(req);
                return;
            }

            FLib_MemCpy(req->Keys.LtkInfo.Ltk, &gSmpKeys.aLtk, mcEncryptionKeySize_c);
            req->Keys.LtkInfo.LtkSize = mcEncryptionKeySize_c;

            req->Keys.RandEdivInfo.Ediv = gSmpKeys.ediv;
            req->Keys.RandEdivInfo.RandSize = gSmpKeys.cRandSize;

            req->Keys.RandEdivInfo.Rand = MEM_BufferAlloc(gSmpKeys.cRandSize);
            if (!req->Keys.RandEdivInfo.Rand)
            {
                shell_write("\r\n-->  Send Key: Insufficient memory. ");
                MEM_BufferFree(req->Keys.LtkInfo.Ltk);
                MEM_BufferFree(req);
                return;
            }

            FLib_MemCpy(req->Keys.RandEdivInfo.Rand, gSmpKeys.aRand, gSmpKeys.cRandSize);
            break;

        case GAPConnectionEventKeyExchangeRequestIndication_RequestedKeys_gIrk_c:
            req->Keys.AddressIncluded = 1;
            FLib_MemCpy(req->Keys.AddressInfo.DeviceAddress, bleAddress, 6);
            req->Keys.AddressInfo.DeviceAddressType = GAPConnectionEventKeysReceivedIndication_Keys_AddressInfo_DeviceAddressType_gPublic_c;

            req->Keys.IrkIncluded = 1;
            FLib_MemCpy(req->Keys.Irk, gSmpKeys.aIrk, gcSmpIrkSize_c);
            break;

        case GAPConnectionEventKeyExchangeRequestIndication_RequestedKeys_gCsrk_c:
            req->Keys.CsrkIncluded = 1;
            FLib_MemCpy(req->Keys.Csrk, gSmpKeys.aCsrk, gcSmpCsrkSize_c);
            break;

        case GAPConnectionEventKeyExchangeRequestIndication_RequestedKeys_gNoKeys_c:
        default:
        	break;
    }

    GAPSendSmpKeysRequest(req, BLE_FSCI_IF);
    MEM_BufferFree(req->Keys.RandEdivInfo.Rand);
    MEM_BufferFree(req->Keys.LtkInfo.Ltk);
    MEM_BufferFree(req);
}

void BleApp_ProvideLongTermKey(void *data)
{
    GAPConnectionEventLongTermKeyRequestIndication_t *rcvData = data;
    GAPProvideLongTermKeyRequest_t *req;
    GAPLoadEncryptionInformationRequest_t *provideReq;
    GAPDenyLongTermKeyRequest_t *denyReq;

    if (rcvData->Ediv == gSmpKeys.ediv && rcvData->RandSize == gSmpKeys.cRandSize)
    {
    	req = MEM_BufferAlloc(sizeof(GAPProvideLongTermKeyRequest_t));
    	provideReq = MEM_BufferAlloc(sizeof(GAPLoadEncryptionInformationRequest_t));
		if (!req || !provideReq)
		{
			MEM_BufferFree(req);
			MEM_BufferFree(provideReq);
			shell_write("\r\n-->  ProvideLongTermKey: Insufficient memory. ");
			return;
		}

		provideReq->DeviceId = rcvData->DeviceId;

        req->DeviceId = rcvData->DeviceId;
        req->LtkSize = gSmpKeys.cLtkSize;

        req->Ltk = MEM_BufferAlloc(gSmpKeys.cLtkSize);
        if (!req->Ltk)
        {
            shell_write("\r\n-->  ProvideLongTermKey: Insufficient memory. ");
            MEM_BufferFree(req);
            return;
        }
        FLib_MemCpy(req->Ltk, gSmpKeys.aLtk, gSmpKeys.cLtkSize);

        GAPProvideLongTermKeyRequest(req, BLE_FSCI_IF);
        GAPLoadEncryptionInformationRequest(provideReq, BLE_FSCI_IF);

        MEM_BufferFree(req->Ltk);
        MEM_BufferFree(req);
        MEM_BufferFree(provideReq);
    }
    else
    {
    	denyReq = MEM_BufferAlloc(sizeof(GAPDenyLongTermKeyRequest_t));
    	shell_write("\r\n--> ProvideLongTermKey: Wrong Key");
		if (!denyReq)
		{
			shell_write("\r\n-->  ProvideLongTermKey: Insufficient memory. ");
			return;
		}

		denyReq->DeviceId = rcvData->DeviceId;
		GAPDenyLongTermKeyRequest(denyReq, BLE_FSCI_IF);

		MEM_BufferFree(denyReq);
	}
}

void BleApp_DisplayCode(uint32_t code)
{
	uint32_t aux = code;
    uint8_t len = 0;
    uint8_t i = 0;

    while (aux != 0)
    {
        aux /= 10;
        len ++;
    }

	shell_write("\r\n\r\n");

	for (i = len; i < PAIRING_CODE_SIZE; i++)
		shell_writeDec(0);

	shell_writeDec(code);
	shell_write("\r\n\r\n");

	GUI_SetBkColor(GUI_BLACK);
    GUI_Clear();
    GUI_SetColor(GUI_WHITE);
    GUI_DispStringAt("Pairing code: ", 40, 50);
    GUI_DispDecAt(code, 70, 75, PAIRING_CODE_SIZE);
}

void BleApp_DemoRpkWriteCallback(void *data)
{
    if (data == NULL)
    {
        shell_write("Invalid write callback pointer");
        return;
    }

    GATTServerAttributeWrittenIndication_t *rcvData = data;
    uint8_t status = 0;

    if (rcvData->AttributeWrittenEvent.Handle == hValueLED)
    {
        if (rcvData->AttributeWrittenEvent.ValueLength == 2)
            RGB_Led_Set_State(rcvData->AttributeWrittenEvent.Value[0], rcvData->AttributeWrittenEvent.Value[1]);
        else
            status = 0xd;

    }
    else if (rcvData->AttributeWrittenEvent.Handle ==  hValueBuzzer)
    {
        if (rcvData->AttributeWrittenEvent.ValueLength == 1)
        {
            if (*rcvData->AttributeWrittenEvent.Value == 1 || *rcvData->AttributeWrittenEvent.Value == 49)
                BUZZER_ON;
            else if (*rcvData->AttributeWrittenEvent.Value == 0 || *rcvData->AttributeWrittenEvent.Value == 48)
                BUZZER_OFF;
            else
                status = 0xd;
        }
        else
            status = 0xd;

    }
    else if (rcvData->AttributeWrittenEvent.Handle == hValueBacklight)
    {
        if (rcvData->AttributeWrittenEvent.ValueLength == 1)
        {
            if (*rcvData->AttributeWrittenEvent.Value >= 0 && *rcvData->AttributeWrittenEvent.Value <= 3)
            {
                if (Backlight_SetLevel(*rcvData->AttributeWrittenEvent.Value) == 1)
                {
                    shell_write("Can not set the backlight level\n\r");
                    status = 0xd;
                }
            }
            else
                status = 0xd;
        }
        else
            status = 0xd;
    }
    else if (rcvData->AttributeWrittenEvent.Handle == hValueChallenge)
    {
        if (rcvData->AttributeWrittenEvent.ValueLength == 44)
        {
            // Write challenge to A100x
            uint8_t challenge[44] = {0};
            FLib_MemCpy(challenge, rcvData->AttributeWrittenEvent.Value, 44);
            if (set_auth_challenge(challenge))
            {
                status = 0xd;
            }
            else
            {
                // Set flag to allow notification to read response
                mChallengeReceived = TRUE;
            }
        }
    }
    else if (rcvData->AttributeWrittenEvent.Handle ==  hValueControlPointOTA)
    {
        BleApp_AttributeWritten(rcvData->DeviceId,
                                rcvData->AttributeWrittenEvent.Handle,
                                rcvData->AttributeWrittenEvent.ValueLength,
                                rcvData->AttributeWrittenEvent.Value);
    }

    if (status == 0)
    {
        GATTDBWriteAttributeRequest_t req;

        req.Handle = rcvData->AttributeWrittenEvent.Handle;
        req.ValueLength = rcvData->AttributeWrittenEvent.ValueLength;
        req.Value = rcvData->AttributeWrittenEvent.Value;

        GATTDBWriteAttributeRequest(&req, BLE_FSCI_IF);
    }

    if (rcvData->AttributeWrittenEvent.Handle !=  hValueControlPointOTA)
        ShellGattDb_SendAttributeWrittenStatusRequest(rcvData->DeviceId,rcvData->AttributeWrittenEvent.Handle, status);

    MEM_BufferFree(rcvData->AttributeWrittenEvent.Value);

    return;
}

void BleApp_DemoRpkNotify(void)
{
    // Do not notify if OTA is tried
    if (ota_demo)
        return;

    if (mRpkNotifyTimerID == gTmrInvalidTimerID_c)
    {
        mRpkNotifyTimerID = TMR_AllocateTimer();
    }

    TMR_StartSingleShotTimer(mRpkNotifyTimerID, mDemoReportInterval_c, BleApp_NotifyRpkSm, NULL);
}

/************************************************************************************
*************************************************************************************
* Private functions
*************************************************************************************
************************************************************************************/
static int8_t BleApp_DemoCommand(uint8_t argc, char *argv[])
{
    if (argc < 2)
    {
        return CMD_RET_USAGE;
    }

    if (!strcmp((char *)argv[1], "ota"))
    {
        ota_demo = 1;
        BleApp_DemoRpk();
        return CMD_RET_SUCCESS;
    }

    if (!strcmp((char *)argv[1], "rpk"))
    {
        BleApp_DemoRpk();
        return CMD_RET_SUCCESS;
    }
    return CMD_RET_USAGE;
}

static void BleApp_NotifyRpkSm(void *param)
{
    if (ota_demo || (gRpkDemoState != gDemoConnectionEstablish_c))
        return;

    uint8_t TemperatureValue[5]   = {0x00};
    uint8_t HumidityValue[4]      = {0x00};
    uint8_t PressureValue[4]      = {0x00};
    uint8_t AirQualityValue[2]    = {0x00};
    uint8_t AmbientLightValue[4]  = {0x00};
    uint8_t GyroscopeValue[6]     = {0x00};
    uint8_t AccelerometerValue[12] = {0x00};
    uint8_t MagnetometerValue[12]  = {0x00};
    uint8_t BatteryLevelValue[1]  = {0x00};
    uint8_t BatteryStatusValue[1] = {0x00};
    uint8_t UidValue[16]          = {0x00};
    uint8_t CertValue[128]        = {0x00};
    uint8_t ResValue[44]          = {0x00};

    uint8_t TempSize = 4;
    uint8_t HumiditySize = 4;
    uint8_t PressureSize = 4;
    uint8_t AirQualitySize = 2;
    uint8_t AmbientLightSize = 4;
    uint8_t GyroscopeSize = 6;
    uint8_t AccelerometerSize = 12;
    uint8_t MagnetometerSize = 12;
    uint8_t UidSize = 16;
    uint8_t CertificateSize = 128;
    uint8_t ResponseSize = 44;

    switch (gRpkNotifyState)
    {
        case gNotifyTemperature_c:
            if (get_temperature(TemperatureValue + 1, &TempSize) == 0)
                BleApp_Notify(hValueTempMeasurement, TemperatureValue, TempSize + 1);
            else
                shell_write("\r\n-->  Notify Event: Can not read Temperature value. ");

            gRpkNotifyState = gNotifyHumidity_c;
            break;

        case gNotifyHumidity_c:
            if (get_humidity(HumidityValue, &HumiditySize) == 0)
                BleApp_Notify(hValueHumidity, HumidityValue, HumiditySize);
            else
                shell_write("\r\n-->  Notify Event: Can not read Humidity value. ");

            gRpkNotifyState = gNotifyPressure_c;
            break;

        case gNotifyPressure_c:
            if (get_pressure(PressureValue, &PressureSize) == 0)
                BleApp_Notify(hValuePressure, PressureValue, PressureSize);
            else
                shell_write("\r\n-->  Notify Event: Can not read Pressure value. ");

            gRpkNotifyState = gNotifyAirQuality_c;
            break;

        case gNotifyAirQuality_c:
            if (get_air_quality(AirQualityValue, &AirQualitySize) ==  0)
                BleApp_Notify(hValueAirQuality, AirQualityValue, AirQualitySize);
            else
                shell_write("\r\n-->  Notify Event: Can not read Air Quality value. ");

            gRpkNotifyState = gNotifyAmbientLight_c;
            break;

        case gNotifyAmbientLight_c:
            if (get_ambient_light(AmbientLightValue, &AmbientLightSize) == 0)
                BleApp_Notify(hValueAmbientLight, AmbientLightValue, AmbientLightSize);
            else
                shell_write("\r\n-->  Notify Event: Can not read Ambient Light value. ");

            gRpkNotifyState = gNotifyGyroscope_c;
            break;

        case gNotifyGyroscope_c:
            if (get_rotation_speed(GyroscopeValue, &GyroscopeSize) == 0)
                BleApp_Notify(hValueGyroscope, GyroscopeValue, GyroscopeSize);
            else
                shell_write("\r\n-->  Notify Event: Can not read Gyroscope value. ");

            gRpkNotifyState = gNotifyAccelerometer_c;
            break;

        case gNotifyAccelerometer_c:
            if (get_acceleration(AccelerometerValue, &AccelerometerSize) == 0)
                BleApp_Notify(hValueAccelerometer, AccelerometerValue, AccelerometerSize);
            else
                shell_write("\r\n-->  Notify Event: Can not read Accelerometer value. ");

            gRpkNotifyState = gNotifyMagnetometer_c;
            break;

        case gNotifyMagnetometer_c:
            if (get_magnetic_field(MagnetometerValue, &MagnetometerSize) == 0)
                BleApp_Notify(hValueMagnetometer, MagnetometerValue, MagnetometerSize);
            else
                shell_write("\r\n-->  Notify Event: Can not read Magnetometer value. ");

            gRpkNotifyState = gNotifyBatteryLevel_c;
            break;

        case gNotifyBatteryLevel_c:
            if (BatterySensor_GetState(BatteryLevelValue,BatteryStatusValue) == 0)
                BleApp_Notify(hValueBatteryLevel, BatteryLevelValue, 1);
            else
                shell_write("\r\n-->  Notify Event: Can not read Battery level. ");

            gRpkNotifyState = gNotifyBatteryStatus_c;
            break;

        case gNotifyBatteryStatus_c:
            if (BatterySensor_GetState(BatteryLevelValue,BatteryStatusValue) == 0)
                BleApp_Notify(hValueBatteryStatus, BatteryStatusValue, 1);
            else
                shell_write("\r\n-->  Notify Event: Can not read Battery charging status. ");

            gRpkNotifyState = gNotifyUid_c;
            break;

        case gNotifyUid_c:
        {
            uint32_t ret = I2C_RESULT_FAIL;
            ret = get_auth_uid(UidValue);
            if (I2C_RESULT_OK == ret)
            {
                BleApp_Notify(hValueUid, UidValue, UidSize);
            }
            else
            {
                shell_write("\r\n-->  Notify Event: Can not read UID value. ");
            }

            gRpkNotifyState = gNotifyCert_c;
            break;
        }

        case gNotifyCert_c:
        {
            uint32_t ret = I2C_RESULT_FAIL;
            ret = get_auth_cert(CertValue);
            if (I2C_RESULT_OK == ret)
            {
                BleApp_Notify(hValueCertificate, CertValue, CertificateSize);
            }
            else
            {
                shell_write("\r\n-->  Notify Event: Can not read Certificate value. ");
            }

            gRpkNotifyState = gNotifyResponse_c;
            break;
        }

        case gNotifyResponse_c:
        {

            if (TRUE == mChallengeReceived)
            {
                uint32_t ret = I2C_RESULT_FAIL;
                ret = get_auth_response(ResValue);
                if (I2C_RESULT_OK == ret)
                {
                    BleApp_Notify(hValueResponse, ResValue, ResponseSize);
                }
                else
                {
                    shell_write("\r\n-->  Notify Event: Can not read Response value. ");
                }
                mChallengeReceived = FALSE;
                mAuthSetSoftReset = TRUE;
            }
            else
            {
                if (mAuthSetSoftReset)
                {
                    uint32_t ret = I2C_RESULT_FAIL;
                    ret = set_auth_softreset();
                    if (ret)
                    {
                        shell_write("\r\n-->  Notify Event: Unable to trigger soft reset. ");
                    }
                    mAuthSetSoftReset = FALSE;
                }

                // Clear out response data
                memset(ResValue, 0, ResponseSize);
                BleApp_Notify(hValueResponse, ResValue, ResponseSize);
            }

            gRpkNotifyState = gNotifyTemperature_c;
            break;
        }

    }

    if (mRpkNotifyTimerID == gTmrInvalidTimerID_c)
    {
        mRpkNotifyTimerID = TMR_AllocateTimer();
    }

   TMR_StartSingleShotTimer(mRpkNotifyTimerID, mDemoReportInterval_c * 1000, BleApp_NotifyRpkSm, NULL);
}

static void BleApp_Notify(uint16_t hValue, uint8_t *value, uint8_t ValueLenght)
{
    if (gRpkDemoState != gDemoConnectionEstablish_c)
    {
        return;
    }
    GATTDBWriteAttributeRequest_t req;

    char **argv = MEM_BufferAlloc(sizeof(char *));

    if (!argv)
    {
        shell_write("\r\n-->  Notify Event: Insufficient memory. ");
        return;
    }

    argv[0] = MEM_BufferAlloc(3);

    if (!argv[0])
    {
        shell_write("\r\n-->  Notify Event: Insufficient memory. ");
        return;
    }

    req.Handle = hValue;
    req.ValueLength = ValueLenght;

    req.Value = value;
    GATTDBWriteAttributeRequest(&req, BLE_FSCI_IF);

    sprintf(argv[0], "%hu", hValue);
    ShellGatt_Notify(1, argv);

    MEM_BufferFree(argv[0]);
    MEM_BufferFree(argv);
}

static void BleApp_StartRpkDemoSm(void *param)
{
    switch (gRpkDemoState)
    {
        case gDemoAddServices_c:
        {
        	ShellGattDb_AddServiceGap();
            ShellGattDb_AddServiceGatt();
            ShellGattDb_AddServiceOtap();
            ShellGattDb_AddServiceDeviceInfo();
            ShellGattDb_AddServiceBattery();
            ShellGattDb_AddServiceWeather();
            ShellGattDb_AddServiceMotion();
            ShellGattDb_AddServiceInteraction();
            ShellGattDb_AddServiceAuth();

            gRpkDemoState = gDemoReadPublicAddresss_c;
            break;
        }

        case gDemoReadPublicAddresss_c:
        {
            ShellGap_DeviceAddress(0, NULL);

            gRpkDemoState = gDemoSetAdvData_c;
            break;
        }

        case gDemoSetAdvData_c:
        {
            uint8_t adData1[] = { 0xE0, 0x1C, 0x4B, 0x5E, 0x1E, 0xEB, 0xA1, 0x5C, 0xEE, 0xF4, 0x5E, 0xBA, 0x50, 0x55, 0xFF, 0x01, 0x00};
            char tmp[] = {0x06,0x00};
            sprintf(device_name, "%s%02X%02X\0", device_name, bleAddress[1], bleAddress[0]);

            ShellGap_AppendAdvData(&gAppAdvertisingData,
                                               GAPSetAdvertisingDataRequest_AdvertisingData_AdStructures_Type_gAdFlags_c,
                                               tmp);
            ShellGap_AppendAdvData(&gAppAdvertisingData,
                                                GAPSetAdvertisingDataRequest_AdvertisingData_AdStructures_Type_gAdComplete128bitServiceList_c,
                                                (char *)adData1);

            ShellGap_AppendAdvData(&gAppAdvertisingData,
                                   GAPSetAdvertisingDataRequest_AdvertisingData_AdStructures_Type_gAdShortenedLocalName_c,
                                   device_name);
            ShellGap_ChangeAdvertisingData(2, NULL);

            gRpkDemoState = gDemoFindGapServiceHandle_c;
            break;
        }

        case gDemoFindGapServiceHandle_c:
		{
			GATTDBFindServiceHandleRequest_t req;
			req.StartHandle = 0x0001;  // should be 0x0001 on the first call
			req.UuidType = Uuid16Bits;
			UuidToArray(req.Uuid.Uuid16Bits, gBleSig_GenericAccessProfile_d);

			gLatestHandle = INVALID_HANDLE;
			GATTDBFindServiceHandleRequest(&req, BLE_FSCI_IF);
			gRpkDemoState = gDemoFindDeviceNameHandle_c;
			break;
		}

        case gDemoFindDeviceNameHandle_c:
		{
			if (gLatestHandle != INVALID_HANDLE)
			{
				// save previous handle requested
				hServiceGap = gLatestHandle;

				GATTDBFindCharValueHandleInServiceRequest_t req;
				req.ServiceHandle = hServiceGap;
				req.UuidType = Uuid16Bits;
				UuidToArray(req.Uuid.Uuid16Bits, gBleSig_GapDeviceName_d);

				gLatestHandle = INVALID_HANDLE;
				GATTDBFindCharValueHandleInServiceRequest(&req, BLE_FSCI_IF);

				gRpkDemoState = gDemoWriteDeviceName_c;
			}
			else
			{
				shell_write("\r\n-->  GATT DB: Could not find handle for the GAP Service.");
				gRpkDemoState = gDemoError_c;
			}

			break;
		}

        case gDemoWriteDeviceName_c:
        {
        	if (gLatestHandle != INVALID_HANDLE)
			{
				// save previous handle requested
				hValueDeviceName = gLatestHandle;

				GATTDBWriteAttributeRequest_t req;
				req.Handle = hValueDeviceName;
				req.ValueLength = DEVICE_PREFIX_LENGTH + 5;
				req.Value = (uint8_t *)device_name;

				GATTDBWriteAttributeRequest(&req, BLE_FSCI_IF);

				gRpkDemoState = gDemoFindBatteryServiceHandle_c;
			}
			else
			{
				shell_write("\r\n-->  GATT DB: Could not find handle for the Device Name .");
				gRpkDemoState = gDemoError_c;
			}

			break;
        }

        case gDemoFindBatteryServiceHandle_c:
        {
            GATTDBFindServiceHandleRequest_t req;
            req.StartHandle = 0x0001;  // should be 0x0001 on the first call
            req.UuidType = Uuid16Bits;
            UuidToArray(req.Uuid.Uuid16Bits, gBleSig_BatteryService_d);

            gLatestHandle = INVALID_HANDLE;
            GATTDBFindServiceHandleRequest(&req, BLE_FSCI_IF);
            gRpkDemoState = gDemoFindBatteryLevelHandle_c;
            break;
        }

        case gDemoFindBatteryLevelHandle_c:
        {
            if (gLatestHandle != INVALID_HANDLE)
            {
                // save previous handle requested
                hServiceBattery = gLatestHandle;

                GATTDBFindCharValueHandleInServiceRequest_t req;
                req.ServiceHandle = hServiceBattery;
                req.UuidType = Uuid16Bits;
                UuidToArray(req.Uuid.Uuid16Bits, gBleSig_BatteryLevel_d);

                gLatestHandle = INVALID_HANDLE;
                GATTDBFindCharValueHandleInServiceRequest(&req, BLE_FSCI_IF);

                gRpkDemoState = gDemoFindBatteryStatusHandle_c;
            }
            else
            {
                shell_write("\r\n-->  GATT DB: Could not find handle for the Battery Service.");
                gRpkDemoState = gDemoError_c;
            }

            break;
        }

        case gDemoFindBatteryStatusHandle_c:
        {
            if (gLatestHandle != INVALID_HANDLE)
            {
                // save previous handle requested
                hValueBatteryLevel = gLatestHandle;

                GATTDBFindCharValueHandleInServiceRequest_t req;
                req.ServiceHandle = hServiceBattery;
                req.UuidType = Uuid16Bits;
                UuidToArray(req.Uuid.Uuid16Bits, gBleSig_BatteryStatus_d);

                gLatestHandle = INVALID_HANDLE;
                GATTDBFindCharValueHandleInServiceRequest(&req, BLE_FSCI_IF);

                gRpkDemoState = gDemoFindWeatherServiceHandle_c;
            }
            else
            {
                shell_write("\r\n-->  GATT DB: Could not find handle for the Battery Level Characteristic.");
                gRpkDemoState = gDemoError_c;
            }

            break;
        }

        case gDemoFindWeatherServiceHandle_c:
        {
            if (gLatestHandle != INVALID_HANDLE)
            {
                // save previous handle requested
                hValueBatteryStatus = gLatestHandle;

                GATTDBFindServiceHandleRequest_t req;
                req.StartHandle = 0x0001;  // should be 0x0001 on the first call
                req.UuidType = Uuid128Bits;
                memcpy(req.Uuid.Uuid128Bits, uuid_service_weather, 16);

                gLatestHandle = INVALID_HANDLE;
                GATTDBFindServiceHandleRequest(&req, BLE_FSCI_IF);

                gRpkDemoState = gDemoFindTempMeasurementHandle_c;
            }
            else
            {
                shell_write("\r\n-->  GATT DB: Could not find char handle for the Battery Status.");
                gRpkDemoState = gDemoError_c;
            }

            break;
        }

        case gDemoFindTempMeasurementHandle_c:
        {
            if (gLatestHandle != INVALID_HANDLE)
            {
                GATTDBFindCharValueHandleInServiceRequest_t req;
                // save previous handle requested
                hServiceWeather = gLatestHandle;

                req.ServiceHandle = hServiceWeather;
                req.UuidType = Uuid16Bits;
                UuidToArray(req.Uuid.Uuid16Bits, gBleSig_TemperatureMeasurement_d);

                gLatestHandle = INVALID_HANDLE;
                GATTDBFindCharValueHandleInServiceRequest(&req, BLE_FSCI_IF);

                gRpkDemoState = gDemoFindHumidityHandle_c;
            }
            else
            {
                shell_write("\r\n-->  GATT DB: Could not find characteristic handle for the Weather Service.");
                gRpkDemoState = gDemoError_c;
            }

            break;
        }

        case gDemoFindHumidityHandle_c:
        {
            if (gLatestHandle != INVALID_HANDLE)
            {
                GATTDBFindCharValueHandleInServiceRequest_t req;
                // save previous handle requested
                hValueTempMeasurement = gLatestHandle;

                req.ServiceHandle = hServiceWeather;
                req.UuidType = Uuid16Bits;
                UuidToArray(req.Uuid.Uuid16Bits, gBleSig_Humudity_d);

                gLatestHandle = INVALID_HANDLE;
                GATTDBFindCharValueHandleInServiceRequest(&req, BLE_FSCI_IF);

                gRpkDemoState = gDemoFindPressureHandle_c;
            }
            else
            {
                shell_write("\r\n-->  GATT DB: Could not find characteristic handle for Temperature.");
                gRpkDemoState = gDemoError_c;
            }

            break;
        }

        case gDemoFindPressureHandle_c:
        {
            if (gLatestHandle != INVALID_HANDLE)
            {
                GATTDBFindCharValueHandleInServiceRequest_t req;
                // save previous handle requested
                hValueHumidity = gLatestHandle;

                req.ServiceHandle = hServiceWeather;
                req.UuidType = Uuid16Bits;
                UuidToArray(req.Uuid.Uuid16Bits, gBleSig_Pressure_d);

                gLatestHandle = INVALID_HANDLE;
                GATTDBFindCharValueHandleInServiceRequest(&req, BLE_FSCI_IF);

                gRpkDemoState = gDemoFindAirQualityHandle_c;
            }
            else
            {
                shell_write("\r\n-->  GATT DB: Could not find characteristic handle for Humidity.");
                gRpkDemoState = gDemoError_c;
            }

            break;
        }

        case gDemoFindAirQualityHandle_c:
        {
            if (gLatestHandle != INVALID_HANDLE)
            {
                GATTDBFindCharValueHandleInServiceRequest_t req;
                // save previous handle requested
                hValuePressure = gLatestHandle;

                req.ServiceHandle = hServiceWeather;
                req.UuidType = Uuid128Bits;
                memcpy(req.Uuid.Uuid128Bits, uuid_characteristic_airQuality, 16);

                gLatestHandle = INVALID_HANDLE;
                GATTDBFindCharValueHandleInServiceRequest(&req, BLE_FSCI_IF);

                gRpkDemoState = gDemoFindAmbientLightHandle_c;
            }
            else
            {
                shell_write("\r\n-->  GATT DB: Could not find characteristic handle for Pressure.");
                gRpkDemoState = gDemoError_c;
            }

            break;
        }

        case gDemoFindAmbientLightHandle_c:
        {
            if (gLatestHandle != INVALID_HANDLE)
            {
                GATTDBFindCharValueHandleInServiceRequest_t req;
                // save previous handle requested
                hValueAirQuality = gLatestHandle;

                req.ServiceHandle = hServiceWeather;
                req.UuidType = Uuid128Bits;
                memcpy(req.Uuid.Uuid128Bits, uuid_characteristic_ambientLight, 16);

                gLatestHandle = INVALID_HANDLE;
                GATTDBFindCharValueHandleInServiceRequest(&req, BLE_FSCI_IF);

                gRpkDemoState = gDemoWriteTempAttribute_c;
            }
            else
            {
                shell_write("\r\n-->  GATT DB: Could not find characteristic handle for Air Quality.");
                gRpkDemoState = gDemoError_c;
            }

            break;
        }

        case gDemoWriteTempAttribute_c:
        {
            if (gLatestHandle != INVALID_HANDLE)
            {
                GATTDBWriteAttributeRequest_t req;
                uint8_t value[5] = {0x00, 0x41, 0xd0, 0x00, 0x00};

                // save previous handle requested
                hValueAmbientLight = gLatestHandle;

                req.Handle = hValueTempMeasurement;
                req.ValueLength = 5;
                req.Value = value;

                GATTDBWriteAttributeRequest(&req, BLE_FSCI_IF);

                gRpkDemoState = gDemoWriteHumidityParam_c;
            }
            else
            {
                shell_write("\r\n-->  GATT DB: Could not find characteristic handle for Ambient Light.");
                gRpkDemoState = gDemoError_c;
            }

            break;
        }

        case gDemoWriteHumidityParam_c:
        {
            GATTDBWriteAttributeRequest_t req;
            uint8_t value[4] = {0x00, 0x13, 0x00, 0x00};

            req.Handle = hValueHumidity;
            req.ValueLength = 4;
            req.Value = value;

            GATTDBWriteAttributeRequest(&req, BLE_FSCI_IF);

            gRpkDemoState = gDemoWritePressureParam_c;

            break;
        }

        case gDemoWritePressureParam_c:
        {
            GATTDBWriteAttributeRequest_t req;
            uint8_t value[4] = {0x00, 0x00, 0x13, 0x13};

            req.Handle = hValuePressure;
            req.ValueLength = 4;
            req.Value = value;

            GATTDBWriteAttributeRequest(&req, BLE_FSCI_IF);

            gRpkDemoState = gDemoWriteAirParam_c;

            break;
        }

        case gDemoWriteAirParam_c:
        {
            GATTDBWriteAttributeRequest_t req;
            uint8_t value[4] = {0x00, 0x01};

            req.Handle = hValueAirQuality;
            req.ValueLength = 2;
            req.Value = value;

            GATTDBWriteAttributeRequest(&req, BLE_FSCI_IF);

            gRpkDemoState = gDemoWriteLightParam_c;

            break;
        }

        case gDemoWriteLightParam_c:
        {
            GATTDBWriteAttributeRequest_t req;
            uint8_t value[4] = {0x00, 0x02, 0x10, 0x10};

            req.Handle = hValueAmbientLight;
            req.ValueLength = 4;
            req.Value = value;

            GATTDBWriteAttributeRequest(&req, BLE_FSCI_IF);

            gRpkDemoState = gDemoFindMotionServiceHandle_c;

            break;
        }

        case gDemoFindMotionServiceHandle_c:
        {
            GATTDBFindServiceHandleRequest_t req;
            req.StartHandle = 0x0001;  // should be 0x0001 on the first call
            req.UuidType = Uuid128Bits;
            memcpy(req.Uuid.Uuid128Bits, uuid_service_motion, 16);

            gLatestHandle = INVALID_HANDLE;
            GATTDBFindServiceHandleRequest(&req, BLE_FSCI_IF);

            gRpkDemoState = gDemoFindGyroscopeHandle_c;
            break;
        }

        case gDemoFindGyroscopeHandle_c:
        {
            if (gLatestHandle != INVALID_HANDLE)
            {
                GATTDBFindCharValueHandleInServiceRequest_t req;
                // save previous handle requested
                hServiceMotion = gLatestHandle;

                req.ServiceHandle = hServiceMotion;
                req.UuidType = Uuid128Bits;
                memcpy(req.Uuid.Uuid128Bits, uuid_characteristic_gyroscope, 16);

                gLatestHandle = INVALID_HANDLE;
                GATTDBFindCharValueHandleInServiceRequest(&req, BLE_FSCI_IF);

                gRpkDemoState = gDemoFindAccelerometerHandle_c;
            }
            else
            {
                shell_write("\r\n-->  GATT DB: Could not find handle for Motion Service.");
                gRpkDemoState = gDemoError_c;
            }

            break;
        }

        case gDemoFindAccelerometerHandle_c:
        {
            if (gLatestHandle != INVALID_HANDLE)
            {
                GATTDBFindCharValueHandleInServiceRequest_t req;
                // save previous handle requested
                hValueGyroscope = gLatestHandle;

                req.ServiceHandle = hServiceMotion;
                req.UuidType = Uuid128Bits;
                memcpy(req.Uuid.Uuid128Bits, uuid_characteristic_accelerometer, 16);

                gLatestHandle = INVALID_HANDLE;
                GATTDBFindCharValueHandleInServiceRequest(&req, BLE_FSCI_IF);

                gRpkDemoState = gDemoFindMagnetometerHandle_c;
            }
            else
            {
                shell_write("\r\n-->  GATT DB: Could not find characteristic handle for Gyroscope.");
                gRpkDemoState = gDemoError_c;
            }

            break;
        }

        case gDemoFindMagnetometerHandle_c:
        {
            if (gLatestHandle != INVALID_HANDLE)
            {
                GATTDBFindCharValueHandleInServiceRequest_t req;
                // save previous handle requested
                hValueAccelerometer = gLatestHandle;

                req.ServiceHandle = hServiceMotion;
                req.UuidType = Uuid128Bits;
                memcpy(req.Uuid.Uuid128Bits, uuid_characteristic_magnetometer, 16);

                gLatestHandle = INVALID_HANDLE;
                GATTDBFindCharValueHandleInServiceRequest(&req, BLE_FSCI_IF);

                gRpkDemoState = gDemoWriteGyroscope_c;
            }
            else
            {
                shell_write("\r\n-->  GATT DB: Could not find characteristic handle for Accelerometer.");
                gRpkDemoState = gDemoError_c;
            }

            break;
        }

        case gDemoWriteGyroscope_c:
        {
            if (gLatestHandle != INVALID_HANDLE)
            {
                GATTDBWriteAttributeRequest_t req;
                uint8_t value[6] = {0x00, 0x52, 0x10, 0x00, 0x52, 0x10};

                // save previous handle requested
                hValueMagnetometer = gLatestHandle;

                req.Handle = hValueGyroscope;
                req.ValueLength = 6;
                req.Value = value;

                GATTDBWriteAttributeRequest(&req, BLE_FSCI_IF);

                gRpkDemoState = gDemoWriteAccelerometer_c;
            }
            else
            {
                shell_write("\r\n-->  GATT DB: Could not find characteristic handle for Magnetometer.");
                gRpkDemoState = gDemoError_c;
            }

            break;
        }

        case gDemoWriteAccelerometer_c:
        {
           GATTDBWriteAttributeRequest_t req;
           uint8_t value[6] = {0x00, 0x52, 0x10, 0x00, 0x52, 0x10};

           req.Handle = hValueAccelerometer;
           req.ValueLength = 6;
           req.Value = value;

           GATTDBWriteAttributeRequest(&req, BLE_FSCI_IF);

           gRpkDemoState = gDemoWriteMagnetometer_c;

           break;
        }

        case gDemoWriteMagnetometer_c:
        {
           GATTDBWriteAttributeRequest_t req;
           uint8_t value[6] = {0x00, 0x52, 0x10, 0x00, 0x52, 0x10};

           req.Handle = hValueMagnetometer;
           req.ValueLength = 6;
           req.Value = value;

           GATTDBWriteAttributeRequest(&req, BLE_FSCI_IF);

           gRpkDemoState = gDemoFindInteractionServiceHandler_c;

           break;
        }

        case gDemoFindInteractionServiceHandler_c:
        {
            GATTDBFindServiceHandleRequest_t req;
            req.StartHandle = 0x0001;
            req.UuidType = Uuid128Bits;
            memcpy(req.Uuid.Uuid128Bits, uuid_service_interaction, 16);

            gLatestHandle = INVALID_HANDLE;
            GATTDBFindServiceHandleRequest(&req, BLE_FSCI_IF);

            gRpkDemoState = gDemoFindLEDHandle_c;
            break;
        }

        case gDemoFindLEDHandle_c:
        {
            if (gLatestHandle != INVALID_HANDLE)
            {
                GATTDBFindCharValueHandleInServiceRequest_t req;
                // save previous handle requested
                hServiceInteraction = gLatestHandle;

                req.ServiceHandle = hServiceInteraction;
                req.UuidType = Uuid128Bits;
                memcpy(req.Uuid.Uuid128Bits, uuid_characteristic_led, 16);

                gLatestHandle = INVALID_HANDLE;
                GATTDBFindCharValueHandleInServiceRequest(&req, BLE_FSCI_IF);

                gRpkDemoState = gDemoFindBuzzerHandle_c;
            }
            else
            {
                shell_write("\r\n-->  GATT DB: Could not find service handle for Interaction.");
                gRpkDemoState = gDemoError_c;
            }

            break;
        }

        case gDemoFindBuzzerHandle_c:
        {
            if (gLatestHandle != INVALID_HANDLE)
            {
                GATTDBFindCharValueHandleInServiceRequest_t req;
                // save previous handle requested
                hValueLED = gLatestHandle;

                req.ServiceHandle = hServiceInteraction;
                req.UuidType = Uuid128Bits;
                memcpy(req.Uuid.Uuid128Bits, uuid_characteristic_buzzer, 16);

                gLatestHandle = INVALID_HANDLE;
                GATTDBFindCharValueHandleInServiceRequest(&req, BLE_FSCI_IF);

                gRpkDemoState = gDemoFindBacklightHandle_c;
            }
            else
            {
                shell_write("\r\n-->  GATT DB: Could not find characteristic handle for LED.");
                gRpkDemoState = gDemoError_c;
            }

            break;
        }

        case gDemoFindBacklightHandle_c:
        {
            if (gLatestHandle != INVALID_HANDLE)
            {
                GATTDBFindCharValueHandleInServiceRequest_t req;
                // save previous handle requested
                hValueBuzzer = gLatestHandle;

                req.ServiceHandle = hServiceInteraction;
                req.UuidType = Uuid128Bits;
                memcpy(req.Uuid.Uuid128Bits, uuid_characteristic_backlight, 16);

                gLatestHandle = INVALID_HANDLE;
                GATTDBFindCharValueHandleInServiceRequest(&req, BLE_FSCI_IF);

                gRpkDemoState = gDemoFindAuthServiceHandle_c;
            }
            else
            {
                shell_write("\r\n-->  GATT DB: Could not find characteristic handle for Buzzer.");
                gRpkDemoState = gDemoError_c;
            }

            break;
        }

        case gDemoFindAuthServiceHandle_c:
        {
            if (gLatestHandle != INVALID_HANDLE)
            {
                GATTDBFindServiceHandleRequest_t req;
                // save previous handle request
                hValueBacklight = gLatestHandle;

                req.StartHandle = 0x0001;  // should be 0x0001 on the first call
                req.UuidType = Uuid128Bits;
                memcpy(req.Uuid.Uuid128Bits, uuid_service_auth, 16);

                gLatestHandle = INVALID_HANDLE;
                GATTDBFindServiceHandleRequest(&req, BLE_FSCI_IF);
                gRpkDemoState = gDemoFindAuthUidHandle_c;
            }
            else
            {
                shell_write("\r\n-->  GATT DB: Could not find characteristic handle for Backlight.");
                gRpkDemoState = gDemoError_c;
            }
            break;
        }

        case gDemoFindAuthUidHandle_c:
        {
            if (gLatestHandle != INVALID_HANDLE)
            {
                GATTDBFindCharValueHandleInServiceRequest_t req;
                // save previous handle requested
                hServiceAuthentication = gLatestHandle;

                shell_write("\r\n-->  GATT DB: Found handle for Authentication Service.");
                req.ServiceHandle = hServiceAuthentication;
                req.UuidType = Uuid128Bits;
                memcpy(req.Uuid.Uuid128Bits, uuid_char_auth_uid, 16);

                gLatestHandle = INVALID_HANDLE;
                GATTDBFindCharValueHandleInServiceRequest(&req, BLE_FSCI_IF);

                gRpkDemoState = gDemoFindAuthCertHandle_c;
            }
            else
            {
                shell_write("\r\n-->  GATT DB: Could not find handle for Authentication Service.");
                gRpkDemoState = gDemoError_c;
            }

            break;
        }

        case gDemoFindAuthCertHandle_c:
        {
            if (gLatestHandle != INVALID_HANDLE)
            {
                GATTDBFindCharValueHandleInServiceRequest_t req;
                // save previous handle requested
                hValueUid = gLatestHandle;

                shell_write("\r\n-->  GATT DB: Found characteristic handle for UID.");

                req.ServiceHandle = hServiceAuthentication;
                req.UuidType = Uuid128Bits;
                memcpy(req.Uuid.Uuid128Bits, uuid_char_auth_cert, 16);

                gLatestHandle = INVALID_HANDLE;
                GATTDBFindCharValueHandleInServiceRequest(&req, BLE_FSCI_IF);

                gRpkDemoState = gDemoFindAuthChallengeHandle_c;
            }
            else
            {
                shell_write("\r\n-->  GATT DB: Could not find characteristic handle for UID.");
                gRpkDemoState = gDemoError_c;
            }

            break;
        }

        case gDemoFindAuthChallengeHandle_c:
        {
            if (gLatestHandle != INVALID_HANDLE)
            {
                GATTDBFindCharValueHandleInServiceRequest_t req;
                // save previous handle requested
                hValueCertificate = gLatestHandle;

                shell_write("\r\n-->  GATT DB: Found characteristic handle for Certificate.");

                req.ServiceHandle = hServiceAuthentication;
                req.UuidType = Uuid128Bits;
                memcpy(req.Uuid.Uuid128Bits, uuid_char_auth_chl, 16);

                gLatestHandle = INVALID_HANDLE;
                GATTDBFindCharValueHandleInServiceRequest(&req, BLE_FSCI_IF);

                gRpkDemoState = gDemoFindAuthResponseHandle_c;
            }
            else
            {
                shell_write("\r\n-->  GATT DB: Could not find characteristic handle for Certificate.");
                gRpkDemoState = gDemoError_c;
            }

            break;
        }

        case gDemoFindAuthResponseHandle_c:
        {
            if (gLatestHandle != INVALID_HANDLE)
            {
                GATTDBFindCharValueHandleInServiceRequest_t req;
                // save previous handle requested
                hValueChallenge = gLatestHandle;

                shell_write("\r\n-->  GATT DB: Found characteristic handle for Challenge.");

                req.ServiceHandle = hServiceAuthentication;
                req.UuidType = Uuid128Bits;
                memcpy(req.Uuid.Uuid128Bits, uuid_char_auth_res, 16);

                gLatestHandle = INVALID_HANDLE;
                GATTDBFindCharValueHandleInServiceRequest(&req, BLE_FSCI_IF);

                gRpkDemoState = gDemoFindOTAServiceHandle_c;
            }
            else
            {
                shell_write("\r\n-->  GATT DB: Could not find characteristic handle for Challenge.");
                gRpkDemoState = gDemoError_c;
            }

            break;
        }

        case gDemoFindOTAServiceHandle_c:
        {
            if (gLatestHandle != INVALID_HANDLE)
            {
                GATTDBFindServiceHandleRequest_t req;
                // save previous handle request
                hValueResponse = gLatestHandle;

                req.StartHandle = 0x0001;  // should be 0x0001 on the first call
                req.UuidType = Uuid128Bits;
                memcpy(req.Uuid.Uuid128Bits, uuid_service_otap, 16);

                gLatestHandle = INVALID_HANDLE;
                GATTDBFindServiceHandleRequest(&req, BLE_FSCI_IF);
                gRpkDemoState = gDemoFindOTAControlPointHandle_c;
            }
            else
            {
                shell_write("\r\n-->  GATT DB: Could not find characteristic handle for Response.");
                gRpkDemoState = gDemoError_c;
            }
            break;
        }

        case gDemoFindOTAControlPointHandle_c:
        {
            if (gLatestHandle != INVALID_HANDLE)
            {
                GATTDBFindCharValueHandleInServiceRequest_t req;
                // save previous handle requested
                hServiceOTA= gLatestHandle;

                req.ServiceHandle = hServiceOTA;
                req.UuidType = Uuid128Bits;
                memcpy(req.Uuid.Uuid128Bits, uuid_char_otap_control_point, 16);

                gLatestHandle = INVALID_HANDLE;
                GATTDBFindCharValueHandleInServiceRequest(&req, BLE_FSCI_IF);

                gRpkDemoState = gDemoFindOTAControlPointCCCDHandle_c;
            }
            else
            {
                shell_write("\r\n-->  GATT DB: Could not find characteristic handle for OTA service.");
                gRpkDemoState = gDemoError_c;
            }

            break;
        }

        case gDemoFindOTAControlPointCCCDHandle_c:
        {
            if (gLatestHandle != INVALID_HANDLE)
            {
                GATTDBFindCccdHandleForCharValueHandleRequest_t req;
                // save previous handle requested
                hValueControlPointOTA = gLatestHandle;

                req.CharValueHandle = hValueControlPointOTA;

                gCccdHandle = INVALID_HANDLE;
                GATTDBFindCccdHandleForCharValueHandleRequest(&req, BLE_FSCI_IF);

                gRpkDemoState = gDemoFindOTADataHandle_c;
            }
            else
            {
                shell_write("\r\n-->  GATT DB: Could not find characteristic handle for OTA control point.");
                gRpkDemoState = gDemoError_c;
            }

            break;
        }

        case gDemoFindOTADataHandle_c:
        {
            if (gCccdHandle != INVALID_HANDLE)
            {
                GATTDBFindCharValueHandleInServiceRequest_t req;
                // save previous handle requested
                hValueOTACCCD = gCccdHandle;

                req.ServiceHandle = hServiceOTA;
                req.UuidType = Uuid128Bits;
                memcpy(req.Uuid.Uuid128Bits, uuid_char_otap_data, 16);

                gLatestHandle = INVALID_HANDLE;
                GATTDBFindCharValueHandleInServiceRequest(&req, BLE_FSCI_IF);

                gRpkDemoState = gDemoWriteUidAttribute_c;
            }
            else
            {
                shell_write("\r\n-->  GATT DB: Could not find characteristic handle for OTA control point CCCD.");
                gRpkDemoState = gDemoError_c;
            }

            break;
        }

        case gDemoWriteUidAttribute_c:
        {
            if (gLatestHandle != INVALID_HANDLE)
            {
                GATTDBWriteAttributeRequest_t req;
                uint8_t value[16] = { 0x00 };

                // save previous handle requested
                hValueDataOTA = gLatestHandle;

                req.Handle = hValueUid;
                req.ValueLength = 16;
                req.Value = value;

                GATTDBWriteAttributeRequest(&req, BLE_FSCI_IF);

                gRpkDemoState = gDemoWriteCertificateAttribute_c;
            }
            else
            {
                shell_write("\r\n-->  GATT DB: Could not find characteristic handle for OTA data.");
                gRpkDemoState = gDemoError_c;
            }

            break;
        }

        case gDemoWriteCertificateAttribute_c:
        {
            GATTDBWriteAttributeRequest_t req;
            uint8_t value[128] = { 0x00 };

            req.Handle = hValueCertificate;
            req.ValueLength = 128;
            req.Value = value;

            GATTDBWriteAttributeRequest(&req, BLE_FSCI_IF);

            gRpkDemoState = gDemoRegisterCallback;

            break;
        }

        case gDemoRegisterCallback:
        {
               // Tells the K41 to register the generic callback so we can receive a notification
               // when a characteristic is written, but first we have to register the value handle
               ShellGattDb_RegisterCallback();
               gRpkDemoState = gDemoRegisterNotification_c;
               break;
        }

        case gDemoRegisterNotification_c:
        {
            uint8_t count = 12;
            uint16_t handles[6];

            handles[0] = hValueDataOTA;
            handles[1] = hValueControlPointOTA;
            handles[2] = hValueBacklight;
            handles[3] = hValueChallenge;
            handles[4] = hValueLED;
            handles[5] = hValueBuzzer;

            // Register the value handles for the characteristics we want to send notifications
            // when they are written
            ShellGattDb_RegisterForWriteNotifications(count, (uint8_t *)handles);
            gRpkDemoState = gDemoSetPairingParameters;

            break;
        }

        case gDemoSetPairingParameters:
        {
        	GAPSetDefaultPairingParametersRequest(&gParingParametersRequest, BLE_FSCI_IF);

            gRpkDemoState = gDemoSetAdvParam_c;
            break;
        }

        case gDemoSetAdvParam_c:
        {
            gRpkDemoState = gDemoStartAdvertising_c;
            OtapCS_Start();

            GAPSetAdvertisingParametersRequest(&gAdvParams, BLE_FSCI_IF);
            break;
        }

        case gDemoStartAdvertising_c:
        {
            gRpkDemoState = gDemoPrintMessage_c;

            GAPStartAdvertisingRequest(BLE_FSCI_IF);
            break;
        }

        case gDemoPrintMessage_c:
        {
            /*Transmit FSCI Message to set BLE Status LED to indicate advertising  */
        	FSCI_transmitPayload(0xb9, 0x01, NULL, 0, BLE_FSCI_IF);
            shell_write("\r\n-->  Waiting connection from a smartphone..");
            gRpkDemoState = gDemoWaitingConnection_c;

//#define gPasskeyValue_c                999999
#ifdef gPasskeyValue_c
            GAPSetLocalPasskeyRequest_t req_pass = {gPasskeyValue_c};
            GAPSetLocalPasskeyRequest(&req_pass, BLE_FSCI_IF);
#endif

            break;
        }

        default:
            break;
    }

    if ((gRpkDemoState != gDemoError_c) && (gRpkDemoState != gDemoWaitingConnection_c))
    {
        TMR_StartSingleShotTimer(mRpkDemoTimerID, RPK_DEMO_TIMEOUT, BleApp_StartRpkDemoSm, NULL);
    }
}
/*! *********************************************************************************
* @}
********************************************************************************** */

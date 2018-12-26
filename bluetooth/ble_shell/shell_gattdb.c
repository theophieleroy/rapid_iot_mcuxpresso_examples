/*! *********************************************************************************
 * \addtogroup SHELL GATTDB
 * @{
 ********************************************************************************** */
/*!
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
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
#include <stdlib.h>
#include <string.h>

/* Framework / Drivers */
#include "TimersManager.h"
#include "FunctionLib.h"
#include "fsl_os_abstraction.h"
#include "shell.h"
#include "Panic.h"
#include "MemManager.h"
#include "board.h"

#include "ble_sig_defines.h"
#include "ble_shell.h"
#include "shell_gap.h"
#include "shell_gatt.h"
#include "shell_gattdb.h"
#include "cmd_ble.h"

/************************************************************************************
*************************************************************************************
* Private macros
*************************************************************************************
************************************************************************************/
#define mShellGattDbCmdsCount_c             4

/************************************************************************************
*************************************************************************************
* Private type definitions
*************************************************************************************
************************************************************************************/
typedef struct gattCmds_tag
{
    char *name;
    int8_t (*cmd)(uint8_t argc, char *argv[]);
} gattCmds_t;

/************************************************************************************
*************************************************************************************
* Private functions prototypes
*************************************************************************************
************************************************************************************/
/* Shell API Functions */
static int8_t ShellGattDb_Read(uint8_t argc, char *argv[]);
static int8_t ShellGattDb_AddService(uint8_t argc, char *argv[]);
static int8_t ShellGattDb_Erase(uint8_t argc, char *argv[]);

static void ShellGattDb_AddPrimaryServiceDecl(uint16_t serviceUuid);
static void ShellGattDb_AddPrimaryServiceDecl128(uint8_t *serviceUuid);
static void ShellGattDb_AddCharDeclValue(uint16_t charUuid, uint8_t properties, uint16_t length, uint8_t *value,
        GATTDBDynamicAddCharacteristicDeclarationAndValueRequest_ValueAccessPermissions_t permissions, bool_t varlen);
static void ShellGattDb_AddCharDeclValue128(uint8_t *charUuid, uint8_t properties, uint16_t length, uint8_t *value,
        GATTDBDynamicAddCharacteristicDeclarationAndValueRequest_ValueAccessPermissions_t permissions, bool_t varlen);
static void ShellGattDb_AddCharDesc(uint16_t charUuid, uint16_t length, uint8_t *value,
                                    GATTDBDynamicAddCharacteristicDescriptorRequest_DescriptorAccessPermissions_t permissions);

/************************************************************************************
*************************************************************************************
* Private memory declarations
*************************************************************************************
************************************************************************************/
const gattCmds_t mGattDbShellCmds[mShellGattDbCmdsCount_c] =
{
    {"read",        ShellGattDb_Read},
    {"write",       ShellGattDb_Write},
    {"addservice",  ShellGattDb_AddService},
    {"erase",       ShellGattDb_Erase}
};

static uint8_t mGattDbDynamic_GattServiceChangedInitValue[]                   = {0x00, 0x00, 0x00, 0x00};

static uint8_t mGattDbDynamic_GapServiceDeviceNameInitValue[]                 = "RPK BLE";
static uint8_t mGattDbDynamic_GapServiceAppearanceInitValue[]                 = {UuidArray(0)}; // gUnknown_c
static uint8_t mGattDbDynamic_GapServicePpcpInitValue[]                       = {0x0A, 0x00, 0x10, 0x00, 0x64, 0x00, 0xE2, 0x04};

static uint8_t mGattDbDynamic_HumidityInitValue[]                             = {0x00, 0x00, 0x00, 0x00};
static uint8_t mGattDbDynamic_PressureInitValue[]                             = {0x00, 0x00, 0x00, 0x00};
static uint8_t mGattDbDynamic_AirQualityInitValue[]                           = {0x00, 0x00};
static uint8_t mGattDbDynamic_AmbientLightInitValue[]                         = {0x00, 0x00, 0x00, 0x00};
static uint8_t mGattDbDynamic_TermpratureMeasurementInitValue[]               = {0x00, 0x41, 0xc0, 0x00, 0x00};

static uint8_t mGattDbDynamic_GyroscopeInitValue[]                            = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
static uint8_t mGattDbDynamic_AccelerometerInitValue[]                        = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
static uint8_t mGattDbDynamic_MagnetometerInitValue[]                         = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

static uint8_t mGattDbDynamic_LedInitValue[]                                  = {0x00, 0x00};
static uint8_t mGattDbDynamic_BuzzerInitValue[]                               = {0x00};
static uint8_t mGattDbDynamic_BacklightInitValue[]                            = {0x00};

static uint8_t mGattDbDynamic_BServiceLevelInitValue[]                        = {0x5A};
static uint8_t mGattDbDynamic_BServiceStatusInitValue[]                        = {0x0C};
static uint8_t mGattDbDynamic_BServiceCharPresFormatDescriptorInitValue[]     = {0x04, 0x00, 0xAD, 0x27, 0x01, 0x00, 0x00};

static uint8_t mGattDbDynamic_DIServiceManufNameInitValue[]                   = "NXP";
static uint8_t mGattDbDynamic_DIServiceModelNbInitValue[]                     = "Kinetis BLE";
static uint8_t mGattDbDynamic_DIServiceSerialNoInitValue[]                    = "BLESN01";
static uint8_t mGattDbDynamic_DIServiceHwRevInitValue[]                       = BOARD_NAME;
static uint8_t mGattDbDynamic_DIServiceFwRevInitValue[]                       = "1.1.1";
static uint8_t mGattDbDynamic_DIServiceSwRevInitValue[]                       = "1.1.4";
static uint8_t mGattDbDynamic_DIServiceSysIdInitValue[]                       = {0x00, 0x00, 0x00, 0xFE, 0xFF, 0x9F, 0x04, 0x00};
static uint8_t mGattDbDynamic_DIServiceIeeeRcdlInitValue[]                    = {0x00, 0x00, 0x00, 0x00};

static uint8_t mGattDbDynamic_OtapServicePCPInitValue[16]                     = { 0x00 };
static uint8_t mGattDbDynamic_OtapServiceDataInitValue[gAttMaxMtu_c - 3]      = { 0x00 };

/*! Authentication */
static uint8_t mGattDbDynamic_AuthServiceUidValue[16]                         = { 0x00 };
static uint8_t mGattDbDynamic_AuthServiceCertValue[128]                       = { 0x00 };
static uint8_t mGattDbDynamic_AuthServiceChallengeValue[44]                   = { 0x00 };
static uint8_t mGattDbDynamic_AuthServiceResponseValue[44]                    = { 0x00 };

/************************************************************************************
*************************************************************************************
* Public memory declarations
*************************************************************************************
************************************************************************************/
/*! Weather Service UUID */
const uint8_t uuid_service_weather[16]             = {0x88,0xc8,0x65,0xaa,0xcb,0x6c,0x11,0xe7,0xab,0xc4,0xce,0xc2,0x70,0xb6,0xb5,0x0a};
/*! AirQuality Characteristic UUID */
const uint8_t uuid_characteristic_airQuality[16]   = {0x88,0xc8,0x65,0xaa,0xcb,0x6c,0x11,0xe7,0xab,0xc4,0xce,0xc2,0x71,0xb6,0xb5,0x0a};
/*! AmbientLight Characteristic UUID */
const uint8_t uuid_characteristic_ambientLight[16] = {0x88,0xc8,0x65,0xaa,0xcb,0x6c,0x11,0xe7,0xab,0xc4,0xce,0xc2,0x72,0xb6,0xb5,0x0a};

/*! Motion Service UUID */
const uint8_t uuid_service_motion[16]               = {0x88,0xc8,0x65,0xaa,0xcb,0x6c,0x11,0xe7,0xab,0xc4,0xce,0xc2,0x80,0xb6,0xb5,0x0a};
/*! Gyroscope Characteristic UUID */
const uint8_t uuid_characteristic_gyroscope[16]     = {0x88,0xc8,0x65,0xaa,0xcb,0x6c,0x11,0xe7,0xab,0xc4,0xce,0xc2,0x81,0xb6,0xb5,0x0a};
/*! Accelerometer Characteristic UUID */
const uint8_t uuid_characteristic_accelerometer[16] = {0x88,0xc8,0x65,0xaa,0xcb,0x6c,0x11,0xe7,0xab,0xc4,0xce,0xc2,0x82,0xb6,0xb5,0x0a};
/*! Magmetometer Characteristic UUID */
const uint8_t uuid_characteristic_magnetometer[16]  = {0x88,0xc8,0x65,0xaa,0xcb,0x6c,0x11,0xe7,0xab,0xc4,0xce,0xc2,0x83,0xb6,0xb5,0x0a};

/*! Interaction Service UUID */
const uint8_t uuid_service_interaction[16]          = {0x88,0xc8,0x65,0xaa,0xcb,0x6c,0x11,0xe7,0xab,0xc4,0xce,0xc2,0x90,0xb6,0xb5,0x0a};
/*! RGB LED characteristic UUID */
const uint8_t uuid_characteristic_led[16]           = {0x88,0xc8,0x65,0xaa,0xcb,0x6c,0x11,0xe7,0xab,0xc4,0xce,0xc2,0x91,0xb6,0xb5,0x0a};
/*! Buzzer characteristic UUID */
const uint8_t uuid_characteristic_buzzer[16]        = {0x88,0xc8,0x65,0xaa,0xcb,0x6c,0x11,0xe7,0xab,0xc4,0xce,0xc2,0x92,0xb6,0xb5,0x0a};
/*! Backlight characteristic UUID */
const uint8_t uuid_characteristic_backlight[16]     = {0x88,0xc8,0x65,0xaa,0xcb,0x6c,0x11,0xe7,0xab,0xc4,0xce,0xc2,0x93,0xb6,0xb5,0x0a};

/*! Motion Service UUID */
const uint8_t uuid_service_otap[16]            = {0xE0, 0x1C, 0x4B, 0x5E, 0x1E, 0xEB, 0xA1, 0x5C, 0xEE, 0xF4, 0x5E, 0xBA, 0x50, 0x55, 0xFF, 0x01};
/*! OTAP control point Characteristic UUID */
const uint8_t uuid_char_otap_control_point[16] = {0xE0, 0x1C, 0x4B, 0x5E, 0x1E, 0xEB, 0xA1, 0x5C, 0xEE, 0xF4, 0x5E, 0xBA, 0x51, 0x55, 0xFF, 0x01};
/*! OTAP data Characteristic UUID */
const uint8_t uuid_char_otap_data[16]          = {0xE0, 0x1C, 0x4B, 0x5E, 0x1E, 0xEB, 0xA1, 0x5C, 0xEE, 0xF4, 0x5E, 0xBA, 0x52, 0x55, 0xFF, 0x01};

/*! Authentication Service UUID */
const uint8_t uuid_service_auth[16]   = {0xde, 0x76, 0x58, 0x6d, 0x09, 0x40, 0xcf, 0xac, 0xcd, 0xc0, 0x6c, 0x82, 0x20, 0x65, 0x38, 0xaa};
/*! Authentication UID Characteristic UUID */
const uint8_t uuid_char_auth_uid[16]  = {0xde, 0x76, 0x58, 0x6d, 0x09, 0x40, 0xcf, 0xac, 0xcd, 0xc0, 0x6c, 0x82, 0x21, 0x65, 0x38, 0xaa};
/*! Authentication Certificate Characteristic UUID */
const uint8_t uuid_char_auth_cert[16] = {0xde, 0x76, 0x58, 0x6d, 0x09, 0x40, 0xcf, 0xac, 0xcd, 0xc0, 0x6c, 0x82, 0x22, 0x65, 0x38, 0xaa};
/*! Authentication Challenge Characteristic UUID */
const uint8_t uuid_char_auth_chl[16]  = {0xde, 0x76, 0x58, 0x6d, 0x09, 0x40, 0xcf, 0xac, 0xcd, 0xc0, 0x6c, 0x82, 0x23, 0x65, 0x38, 0xaa};
/*! Authentication Response Characteristic UUID */
const uint8_t uuid_char_auth_res[16]  = {0xde, 0x76, 0x58, 0x6d, 0x09, 0x40, 0xcf, 0xac, 0xcd, 0xc0, 0x6c, 0x82, 0x24, 0x65, 0x38, 0xaa};

extern char gDeviceName[];

/************************************************************************************
*************************************************************************************
* Public functions
*************************************************************************************
************************************************************************************/
int8_t ShellGattDb_Command(uint8_t argc, char *argv[])
{
    uint8_t i;

    if (argc < 2)
    {
        return CMD_RET_USAGE;
    }

    for (i = 0; i < mShellGattDbCmdsCount_c; i++)
    {
        if (!strcmp((char *)argv[1], mGattDbShellCmds[i].name))
        {
            return mGattDbShellCmds[i].cmd(argc - 2, (char **)(&argv[2]));
        }
    }

    return CMD_RET_USAGE;
}

void ShellGattDb_AddServiceGatt(void)
{
    ShellGattDb_AddPrimaryServiceDecl(gBleSig_GenericAttributeProfile_d);

    ShellGattDb_AddCharDeclValue(
        gBleSig_GattServiceChanged_d,
        gRead_c | gNotify_c,
        sizeof(mGattDbDynamic_GattServiceChangedInitValue),
        mGattDbDynamic_GattServiceChangedInitValue,
        GATTDBDynamicAddCharacteristicDeclarationAndValueRequest_ValueAccessPermissions_gPermissionNone_c,
        FALSE
    );

    GATTDBDynamicAddCccdRequest(BLE_FSCI_IF);
}

void ShellGattDb_AddServiceGap(void)
{
    ShellGattDb_AddPrimaryServiceDecl(gBleSig_GenericAccessProfile_d);

    ShellGattDb_AddCharDeclValue(
        gBleSig_GapDeviceName_d,
        gRead_c,
        sizeof(mGattDbDynamic_GapServiceDeviceNameInitValue),
        mGattDbDynamic_GapServiceDeviceNameInitValue,
        GATTDBDynamicAddCharacteristicDeclarationAndValueRequest_ValueAccessPermissions_gPermissionFlagReadable_c,
        TRUE
    );

    ShellGattDb_AddCharDeclValue(
        gBleSig_GapAppearance_d,
        gRead_c,
        sizeof(mGattDbDynamic_GapServiceAppearanceInitValue),
        mGattDbDynamic_GapServiceAppearanceInitValue,
        GATTDBDynamicAddCharacteristicDeclarationAndValueRequest_ValueAccessPermissions_gPermissionFlagReadable_c,
        FALSE
    );

    ShellGattDb_AddCharDeclValue(
        gBleSig_GapPpcp_d,
        gRead_c,
        sizeof(mGattDbDynamic_GapServicePpcpInitValue),
        mGattDbDynamic_GapServicePpcpInitValue,
        GATTDBDynamicAddCharacteristicDeclarationAndValueRequest_ValueAccessPermissions_gPermissionFlagReadable_c,
        FALSE
    );

    GATTDBDynamicAddCccdRequest(BLE_FSCI_IF);
}

void ShellGattDb_AddServiceWeather(void)
{
    ShellGattDb_AddPrimaryServiceDecl128((uint8_t *)uuid_service_weather);

    // Temperature measurement characteristic
    ShellGattDb_AddCharDeclValue(
            gBleSig_TemperatureMeasurement_d,
            gNotify_c | gRead_c,
            sizeof(mGattDbDynamic_TermpratureMeasurementInitValue),
            mGattDbDynamic_TermpratureMeasurementInitValue,
            GATTDBDynamicAddCharacteristicDeclarationAndValueRequest_ValueAccessPermissions_gPermissionFlagReadable_c,
            TRUE
    );
    GATTDBDynamicAddCccdRequest(BLE_FSCI_IF);

    // Humidity characteristic
    ShellGattDb_AddCharDeclValue(
            gBleSig_Humudity_d,
            gNotify_c | gRead_c,
            sizeof(mGattDbDynamic_HumidityInitValue),
            mGattDbDynamic_HumidityInitValue,
            GATTDBDynamicAddCharacteristicDeclarationAndValueRequest_ValueAccessPermissions_gPermissionFlagReadable_c,
            FALSE
    );
    GATTDBDynamicAddCccdRequest(BLE_FSCI_IF);

    // Barometric pressure characteristic
    ShellGattDb_AddCharDeclValue(
            gBleSig_Pressure_d,
            gNotify_c | gRead_c,
            sizeof(mGattDbDynamic_PressureInitValue),
            mGattDbDynamic_PressureInitValue,
            GATTDBDynamicAddCharacteristicDeclarationAndValueRequest_ValueAccessPermissions_gPermissionFlagReadable_c,
            FALSE
    );
    GATTDBDynamicAddCccdRequest(BLE_FSCI_IF);

    // Air Quality characteristic
    ShellGattDb_AddCharDeclValue128(
            (uint8_t *)uuid_characteristic_airQuality,
            gNotify_c | gRead_c,
            sizeof(mGattDbDynamic_AirQualityInitValue),
            mGattDbDynamic_AirQualityInitValue,
            GATTDBDynamicAddCharacteristicDeclarationAndValueRequest_ValueAccessPermissions_gPermissionFlagReadable_c,
            FALSE
     );
    GATTDBDynamicAddCccdRequest(BLE_FSCI_IF);

    // Ambient Light characteristic
    ShellGattDb_AddCharDeclValue128(
            (uint8_t *)uuid_characteristic_ambientLight,
           gNotify_c | gRead_c,
           sizeof(mGattDbDynamic_AmbientLightInitValue),
           mGattDbDynamic_AmbientLightInitValue,
           GATTDBDynamicAddCharacteristicDeclarationAndValueRequest_ValueAccessPermissions_gPermissionFlagReadable_c,
           FALSE
    );
    GATTDBDynamicAddCccdRequest(BLE_FSCI_IF);
}

void ShellGattDb_AddServiceMotion(void)
{
    ShellGattDb_AddPrimaryServiceDecl128((uint8_t *)uuid_service_motion);

    // Gyroscope characteristic
    ShellGattDb_AddCharDeclValue128(
            (uint8_t *)uuid_characteristic_gyroscope,
            gNotify_c | gRead_c,
            sizeof(mGattDbDynamic_GyroscopeInitValue),
            mGattDbDynamic_GyroscopeInitValue,
            GATTDBDynamicAddCharacteristicDeclarationAndValueRequest_ValueAccessPermissions_gPermissionFlagReadable_c,
            FALSE
    );
    GATTDBDynamicAddCccdRequest(BLE_FSCI_IF);

    // Accelerometer characteristic
    ShellGattDb_AddCharDeclValue128(
            (uint8_t *)uuid_characteristic_accelerometer,
            gNotify_c | gRead_c,
            sizeof(mGattDbDynamic_AccelerometerInitValue),
            mGattDbDynamic_AccelerometerInitValue,
            GATTDBDynamicAddCharacteristicDeclarationAndValueRequest_ValueAccessPermissions_gPermissionFlagReadable_c,
            FALSE
    );
    GATTDBDynamicAddCccdRequest(BLE_FSCI_IF);

    // Gyroscope characteristic
    ShellGattDb_AddCharDeclValue128(
            (uint8_t *)uuid_characteristic_magnetometer,
            gNotify_c | gRead_c,
            sizeof(mGattDbDynamic_MagnetometerInitValue),
            mGattDbDynamic_MagnetometerInitValue,
            GATTDBDynamicAddCharacteristicDeclarationAndValueRequest_ValueAccessPermissions_gPermissionFlagReadable_c,
            FALSE
    );
    GATTDBDynamicAddCccdRequest(BLE_FSCI_IF);
}

void ShellGattDb_AddServiceBattery(void)
{
    ShellGattDb_AddPrimaryServiceDecl(gBleSig_BatteryService_d);

    ShellGattDb_AddCharDeclValue(
        gBleSig_BatteryLevel_d,
        gRead_c | gNotify_c,
        sizeof(mGattDbDynamic_BServiceLevelInitValue),
        mGattDbDynamic_BServiceLevelInitValue,
        GATTDBDynamicAddCharacteristicDeclarationAndValueRequest_ValueAccessPermissions_gPermissionFlagReadable_c,
        FALSE
    );
    GATTDBDynamicAddCccdRequest(BLE_FSCI_IF);

    ShellGattDb_AddCharDeclValue(
        gBleSig_BatteryStatus_d,
        gRead_c | gNotify_c,
        sizeof(mGattDbDynamic_BServiceStatusInitValue),
        mGattDbDynamic_BServiceStatusInitValue,
        GATTDBDynamicAddCharacteristicDeclarationAndValueRequest_ValueAccessPermissions_gPermissionFlagReadable_c,
        FALSE
    );
    GATTDBDynamicAddCccdRequest(BLE_FSCI_IF);

    ShellGattDb_AddCharDesc(
        gBleSig_CharPresFormatDescriptor_d,
        sizeof(mGattDbDynamic_BServiceCharPresFormatDescriptorInitValue),
        mGattDbDynamic_BServiceCharPresFormatDescriptorInitValue,
        GATTDBDynamicAddCharacteristicDescriptorRequest_DescriptorAccessPermissions_gPermissionFlagReadable_c
    );
}

void ShellGattDb_AddServiceDeviceInfo(void)
{
    ShellGattDb_AddPrimaryServiceDecl(gBleSig_DeviceInformationService_d);

    ShellGattDb_AddCharDeclValue(
        gBleSig_ManufacturerNameString_d,
        gRead_c,
        sizeof(mGattDbDynamic_DIServiceManufNameInitValue),
        mGattDbDynamic_DIServiceManufNameInitValue,
        GATTDBDynamicAddCharacteristicDeclarationAndValueRequest_ValueAccessPermissions_gPermissionFlagReadable_c,
        FALSE
    );
    ShellGattDb_AddCharDeclValue(
        gBleSig_ModelNumberString_d,
        gRead_c,
        sizeof(mGattDbDynamic_DIServiceModelNbInitValue),
        mGattDbDynamic_DIServiceModelNbInitValue,
        GATTDBDynamicAddCharacteristicDeclarationAndValueRequest_ValueAccessPermissions_gPermissionFlagReadable_c,
        FALSE
    );
    ShellGattDb_AddCharDeclValue(
        gBleSig_SerialNumberString_d,
        gRead_c,
        sizeof(mGattDbDynamic_DIServiceSerialNoInitValue),
        mGattDbDynamic_DIServiceSerialNoInitValue,
        GATTDBDynamicAddCharacteristicDeclarationAndValueRequest_ValueAccessPermissions_gPermissionFlagReadable_c,
        FALSE
    );
    ShellGattDb_AddCharDeclValue(
        gBleSig_HardwareRevisionString_d,
        gRead_c,
        sizeof(mGattDbDynamic_DIServiceHwRevInitValue),
        mGattDbDynamic_DIServiceHwRevInitValue,
        GATTDBDynamicAddCharacteristicDeclarationAndValueRequest_ValueAccessPermissions_gPermissionFlagReadable_c,
        FALSE
    );
    ShellGattDb_AddCharDeclValue(
        gBleSig_FirmwareRevisionString_d,
        gRead_c,
        sizeof(mGattDbDynamic_DIServiceFwRevInitValue),
        mGattDbDynamic_DIServiceFwRevInitValue,
        GATTDBDynamicAddCharacteristicDeclarationAndValueRequest_ValueAccessPermissions_gPermissionFlagReadable_c,
        FALSE
    );
    ShellGattDb_AddCharDeclValue(
        gBleSig_SoftwareRevisionString_d,
        gRead_c,
        sizeof(mGattDbDynamic_DIServiceSwRevInitValue),
        mGattDbDynamic_DIServiceSwRevInitValue,
        GATTDBDynamicAddCharacteristicDeclarationAndValueRequest_ValueAccessPermissions_gPermissionFlagReadable_c,
        FALSE
    );
    ShellGattDb_AddCharDeclValue(
        gBleSig_SystemId_d,
        gRead_c,
        sizeof(mGattDbDynamic_DIServiceSysIdInitValue),
        mGattDbDynamic_DIServiceSysIdInitValue,
        GATTDBDynamicAddCharacteristicDeclarationAndValueRequest_ValueAccessPermissions_gPermissionFlagReadable_c,
        FALSE
    );
    ShellGattDb_AddCharDeclValue(
        gBleSig_IeeeRcdl_d,
        gRead_c,
        sizeof(mGattDbDynamic_DIServiceIeeeRcdlInitValue),
        mGattDbDynamic_DIServiceIeeeRcdlInitValue,
        GATTDBDynamicAddCharacteristicDeclarationAndValueRequest_ValueAccessPermissions_gPermissionFlagReadable_c,
        FALSE
    );
}

void ShellGattDb_AddServiceInteraction(void)
{
    ShellGattDb_AddPrimaryServiceDecl128((uint8_t *)uuid_service_interaction);

    ShellGattDb_AddCharDeclValue128(
        (uint8_t *)uuid_characteristic_led,
        gWrite_c | gRead_c,
        sizeof(mGattDbDynamic_LedInitValue),
        mGattDbDynamic_LedInitValue,
        GATTDBDynamicAddCharacteristicDeclarationAndValueRequest_ValueAccessPermissions_gPermissionFlagWritable_c |
        GATTDBDynamicAddCharacteristicDeclarationAndValueRequest_ValueAccessPermissions_gPermissionFlagReadable_c,
        FALSE
    );

    ShellGattDb_AddCharDeclValue128(
        (uint8_t *)uuid_characteristic_buzzer,
        gWrite_c | gRead_c,
        sizeof(mGattDbDynamic_BuzzerInitValue),
        mGattDbDynamic_BuzzerInitValue,
        GATTDBDynamicAddCharacteristicDeclarationAndValueRequest_ValueAccessPermissions_gPermissionFlagWritable_c |
        GATTDBDynamicAddCharacteristicDeclarationAndValueRequest_ValueAccessPermissions_gPermissionFlagReadable_c,
        FALSE
    );

    ShellGattDb_AddCharDeclValue128(
        (uint8_t *)uuid_characteristic_backlight,
        gWrite_c | gRead_c,
        sizeof(mGattDbDynamic_BacklightInitValue),
        mGattDbDynamic_BacklightInitValue,
        GATTDBDynamicAddCharacteristicDeclarationAndValueRequest_ValueAccessPermissions_gPermissionFlagWritable_c |
        GATTDBDynamicAddCharacteristicDeclarationAndValueRequest_ValueAccessPermissions_gPermissionFlagReadable_c,
        FALSE
    );

}

void ShellGattDb_AddServiceOtap(void)
{
    ShellGattDb_AddPrimaryServiceDecl128((uint8_t *)uuid_service_otap);

    ShellGattDb_AddCharDeclValue128(
        (uint8_t *)uuid_char_otap_control_point,
        gWrite_c | gIndicate_c,
        sizeof(mGattDbDynamic_OtapServicePCPInitValue),
        mGattDbDynamic_OtapServicePCPInitValue,
        GATTDBDynamicAddCharacteristicDeclarationAndValueRequest_ValueAccessPermissions_gPermissionFlagWritable_c,
        TRUE
    );
    GATTDBDynamicAddCccdRequest(BLE_FSCI_IF);

    ShellGattDb_AddCharDeclValue128(
        (uint8_t *)uuid_char_otap_data,
        gWriteWithoutRsp_c,
        sizeof(mGattDbDynamic_OtapServiceDataInitValue),
        mGattDbDynamic_OtapServiceDataInitValue,
        GATTDBDynamicAddCharacteristicDeclarationAndValueRequest_ValueAccessPermissions_gPermissionFlagWritable_c,
        TRUE
    );
}

void ShellGattDb_AddServiceAuth(void)
{
    ShellGattDb_AddPrimaryServiceDecl128((uint8_t *)uuid_service_auth);

    //gRead_c
    ShellGattDb_AddCharDeclValue128(
        (uint8_t *)uuid_char_auth_uid,
        gRead_c,
        sizeof(mGattDbDynamic_AuthServiceUidValue),
        mGattDbDynamic_AuthServiceUidValue,
        GATTDBDynamicAddCharacteristicDeclarationAndValueRequest_ValueAccessPermissions_gPermissionFlagReadable_c,
        FALSE
    );
    GATTDBDynamicAddCccdRequest(BLE_FSCI_IF);

    ShellGattDb_AddCharDeclValue128(
        (uint8_t *)uuid_char_auth_cert,
        gRead_c,
        sizeof(mGattDbDynamic_AuthServiceCertValue),
        mGattDbDynamic_AuthServiceCertValue,
        GATTDBDynamicAddCharacteristicDeclarationAndValueRequest_ValueAccessPermissions_gPermissionFlagReadable_c,
        FALSE
    );
    GATTDBDynamicAddCccdRequest(BLE_FSCI_IF);

    ShellGattDb_AddCharDeclValue128(
        (uint8_t *)uuid_char_auth_chl,
        gWrite_c,
        sizeof(mGattDbDynamic_AuthServiceChallengeValue),
        mGattDbDynamic_AuthServiceChallengeValue,
        GATTDBDynamicAddCharacteristicDeclarationAndValueRequest_ValueAccessPermissions_gPermissionFlagWritable_c,
        FALSE
    );
    GATTDBDynamicAddCccdRequest(BLE_FSCI_IF);

    ShellGattDb_AddCharDeclValue128(
        (uint8_t *)uuid_char_auth_res,
        gRead_c | gNotify_c,
        sizeof(mGattDbDynamic_AuthServiceResponseValue),
        mGattDbDynamic_AuthServiceResponseValue,
        GATTDBDynamicAddCharacteristicDeclarationAndValueRequest_ValueAccessPermissions_gPermissionFlagReadable_c,
        FALSE
    );
    GATTDBDynamicAddCccdRequest(BLE_FSCI_IF);

}

int8_t ShellGattDb_Write(uint8_t argc, char *argv[])
{
    uint16_t length, handle;
    GATTDBWriteAttributeRequest_t req;

    if (argc != 2)
    {
        return CMD_RET_USAGE;
    }

    handle = atoi(argv[0]);
    length = BleApp_ParseHexValue(argv[1]);

    if (length > mMaxCharValueLength_d)
    {
        shell_write("\n\r-->  Variable length exceeds maximum!");
        return CMD_RET_FAILURE;
    }

    req.Handle = handle;
    req.ValueLength = length;
    req.Value = (uint8_t *)argv[1];

    if (GATTDBWriteAttributeRequest(&req, BLE_FSCI_IF))
    {
        shell_write("\n\r-->  GATTDB Event: Insufficient memory. ");
        SHELL_NEWLINE();
    }

    return CMD_RET_SUCCESS;
}

void ShellGattDb_RegisterCallback(void)
{
    GATTServerRegisterCallbackRequest(BLE_FSCI_IF);
}

void ShellGattDb_RegisterForWriteNotifications(uint8_t count, uint8_t* handles)
{
    GATTServerRegisterHandlesForWriteNotificationsRequest_t req;

    if (count == 0)
    {
        shell_printf("Write register notify: No handlers!");
        return;
    }
    req.HandleCount = count;
    req.AttributeHandles = handles;

    GATTServerRegisterHandlesForWriteNotificationsRequest(&req, BLE_FSCI_IF);
}

void ShellGattDb_SendAttributeWrittenStatusRequest(uint8_t DeviceId, uint16_t AttributeHandle, uint8_t Status)
{
    GATTServerSendAttributeWrittenStatusRequest_t req;

    req.DeviceId = DeviceId;
    req.AttributeHandle = AttributeHandle;
    req.Status = Status;

    GATTServerSendAttributeWrittenStatusRequest(&req, BLE_FSCI_IF);
}

/************************************************************************************
*************************************************************************************
* Private functions
*************************************************************************************
************************************************************************************/
static int8_t ShellGattDb_Read(uint8_t argc, char *argv[])
{
    GATTDBReadAttributeRequest_t req;

    if (argc != 1)
    {
        return CMD_RET_USAGE;
    }

    req.Handle = atoi(argv[0]);
    req.MaxBytes = mMaxCharValueLength_d;

    GATTDBReadAttributeRequest(&req, BLE_FSCI_IF);

    return CMD_RET_SUCCESS;
}

static int8_t ShellGattDb_AddService(uint8_t argc, char *argv[])
{
    uint16_t serviceUuid = 0, length;

    if (argc != 1)
    {
        return CMD_RET_USAGE;
    }

    if (!strcmp(argv[0], "gatt"))
    {
        ShellGattDb_AddServiceGatt();
    }
    else if (!strcmp(argv[0], "gap"))
    {
        ShellGattDb_AddServiceGap();
    }
    else if (!strcmp(argv[0], "battery"))
    {
        ShellGattDb_AddServiceBattery();
    }
    else if (!strcmp(argv[0], "weather"))
   {
       ShellGattDb_AddServiceWeather();
   }
    else if (!strcmp(argv[0], "motion"))
   {
       ShellGattDb_AddServiceMotion();
   }
    else if (!strcmp(argv[0], "devinfo"))
    {
        ShellGattDb_AddServiceDeviceInfo();
    }
    else if (!strcmp(argv[0], "authentication"))
    {
        ShellGattDb_AddServiceAuth();
    }
    else
    {
        length = BleApp_ParseHexValue(argv[0]);

        if (length == 2)
        {
            FLib_MemCpy(&serviceUuid, argv[0], sizeof(uint16_t));
            ShellGattDb_AddPrimaryServiceDecl(serviceUuid);
        }
        else
        {
            return CMD_RET_USAGE;
        }
    }

    return CMD_RET_SUCCESS;
}


static void ShellGattDb_AddPrimaryServiceDecl(uint16_t serviceUuid)
{
    GATTDBDynamicAddPrimaryServiceDeclarationRequest_t req;
    req.UuidType = Uuid16Bits;
    UuidToArray(req.Uuid.Uuid16Bits, serviceUuid);

    if (GATTDBDynamicAddPrimaryServiceDeclarationRequest(&req, BLE_FSCI_IF) != MEM_SUCCESS_c)
    {
        shell_write("\n\r-->  GATTDB Event: Insufficient memory. ");
        SHELL_NEWLINE();
    }
}

static void ShellGattDb_AddPrimaryServiceDecl128(uint8_t *serviceUuid)
{
    GATTDBDynamicAddPrimaryServiceDeclarationRequest_t req;
    req.UuidType = Uuid128Bits;
    FLib_MemCpy(req.Uuid.Uuid128Bits, serviceUuid, 16);

    if (GATTDBDynamicAddPrimaryServiceDeclarationRequest(&req, BLE_FSCI_IF) != MEM_SUCCESS_c)
    {
        shell_write("\n\r-->  GATTDB Event: Insufficient memory. ");
        SHELL_NEWLINE();
    }
}

static void ShellGattDb_AddCharDeclValue
(
    uint16_t charUuid,
    uint8_t properties,
    uint16_t length,
    uint8_t *value,
    GATTDBDynamicAddCharacteristicDeclarationAndValueRequest_ValueAccessPermissions_t permissions,
    bool_t varlen
)
{
    GATTDBDynamicAddCharacteristicDeclarationAndValueRequest_t req;
    req.UuidType = Uuid16Bits;
    UuidToArray(req.Uuid.Uuid16Bits, charUuid);
    req.CharacteristicProperties = properties;

    if (varlen)
    {
        req.MaxValueLength = length < mMaxCharValueLength_d ? mMaxCharValueLength_d : length;
    }
    else
    {
        req.MaxValueLength = 0;  // for fixed lengths this must be set to 0
    }

    req.InitialValueLength = length;
    req.InitialValue = value;
    req.ValueAccessPermissions = permissions;

    if (GATTDBDynamicAddCharacteristicDeclarationAndValueRequest(&req, BLE_FSCI_IF) != MEM_SUCCESS_c)
    {
        shell_write("\n\r-->  GATTDB Event: Insufficient memory. ");
        SHELL_NEWLINE();
    }
}

static void ShellGattDb_AddCharDeclValue128
(
    uint8_t *charUuid,
    uint8_t properties,
    uint16_t length,
    uint8_t *value,
    GATTDBDynamicAddCharacteristicDeclarationAndValueRequest_ValueAccessPermissions_t permissions,
    bool_t varlen
)
{
    GATTDBDynamicAddCharacteristicDeclarationAndValueRequest_t req;
    req.UuidType = Uuid128Bits;
    FLib_MemCpy(req.Uuid.Uuid128Bits, charUuid, 16);
    req.CharacteristicProperties = properties;

    if (varlen)
    {
        req.MaxValueLength = length < mMaxCharValueLength_d ? mMaxCharValueLength_d : length;
    }
    else
    {
        req.MaxValueLength = 0;  // for fixed lengths this must be set to 0
    }

    req.InitialValueLength = length;
    req.InitialValue = value;
    req.ValueAccessPermissions = permissions;

    if (GATTDBDynamicAddCharacteristicDeclarationAndValueRequest(&req, BLE_FSCI_IF) != MEM_SUCCESS_c)
    {
        shell_write("\n\r-->  GATTDB Event: Insufficient memory. ");
        SHELL_NEWLINE();
    }
}

static void ShellGattDb_AddCharDesc
(
    uint16_t charUuid,
    uint16_t length,
    uint8_t *value,
    GATTDBDynamicAddCharacteristicDescriptorRequest_DescriptorAccessPermissions_t permissions
)
{
    GATTDBDynamicAddCharacteristicDescriptorRequest_t req;
    req.UuidType = Uuid16Bits;
    UuidToArray(req.Uuid.Uuid16Bits, charUuid);
    req.DescriptorValueLength = length;
    req.Value = value;
    req.DescriptorAccessPermissions = permissions;

    if (GATTDBDynamicAddCharacteristicDescriptorRequest(&req, BLE_FSCI_IF) != MEM_SUCCESS_c)
    {
        shell_write("\n\r-->  GATTDB Event: Insufficient memory. ");
        SHELL_NEWLINE();
    }
}

static int8_t ShellGattDb_Erase(uint8_t argc, char *argv[])
{
    if (argc != 0)
    {
        return CMD_RET_USAGE;
    }

    GATTDBDynamicReleaseDatabaseRequest(BLE_FSCI_IF);

    return CMD_RET_SUCCESS;
}

/*! *********************************************************************************
* @}
********************************************************************************** */

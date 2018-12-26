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

static uint8_t mGattDbDynamic_TermpratureMeasurementInitValue[]               = {0x00, 0x41, 0xc0, 0x00, 0x00};
static uint8_t mGattDbDynamic_HumidityInitValue[]                             = {0x00, 0x00};
static uint8_t mGattDbDynamic_PressureInitValue[]                             = {0x00, 0x00, 0x00, 0x00};
static uint8_t mGattDbDynamic_AmbientLightInitValue[]                         = {0x00, 0x00, 0x00, 0x00};
static uint8_t mGattDbDynamic_MacAddressPublicInitValue[]                         = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

/************************************************************************************
*************************************************************************************
* Public memory declarations
*************************************************************************************
************************************************************************************/
/*! Weather Service UUID */
const uint8_t uuid_service_weather[16]             = {0x88,0xc8,0x65,0xaa,0xcb,0x6c,0x11,0xe7,0xab,0xc4,0xce,0xc2,0x70,0xb6,0xb5,0x0a};
const uint8_t uuid_characteristic_ambientLight[16] = {0x88,0xc8,0x65,0xaa,0xcb,0x6c,0x11,0xe7,0xab,0xc4,0xce,0xc2,0x72,0xb6,0xb5,0x0a};
const uint8_t uuid_characteristic_macAddressPublic[16] = {0xb7,0x1c,0xec,0x12,0xf7,0x88,0x49,0x0e,0xa9,0xac,0x2e,0x61,0x26,0xb8,0x1b,0x0e};

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

	extern uint8_t macAddressPublic[6];

	// @MAC ADRESS characteristic
	FLib_MemCpy(mGattDbDynamic_MacAddressPublicInitValue, macAddressPublic, sizeof(macAddressPublic) * 6);
	ShellGattDb_AddCharDeclValue128(
		    (uint8_t *)uuid_characteristic_macAddressPublic,
		    gRead_c,
		    sizeof(mGattDbDynamic_MacAddressPublicInitValue),
			mGattDbDynamic_MacAddressPublicInitValue,
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
    else if (!strcmp(argv[0], "weather"))
    {
    	ShellGattDb_AddServiceWeather();
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

/*! *********************************************************************************
 * \addtogroup SHELL GAP
 * @{
 ********************************************************************************** */
/*!
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 *
 * \file
 *
 * This file is the source file for the GAP Shell module
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
#include <string.h>
#include <stdlib.h>

/* Framework / Drivers */
#include "TimersManager.h"
#include "FunctionLib.h"
#include "fsl_os_abstraction.h"
#include "shell.h"
#include "Panic.h"
#include "MemManager.h"
#include "board.h"
#include "network_utils.h"

#include "ble_shell.h"
#include "shell_gattdb.h"
#include "shell_gap.h"
#include "cmd_ble.h"
#include "ble_sig_defines.h"

/************************************************************************************
*************************************************************************************
* Private macros
*************************************************************************************
************************************************************************************/

/************************************************************************************
*************************************************************************************
* Private type definitions
*************************************************************************************
************************************************************************************/
typedef struct gapCmds_tag
{
    char       *name;
    int8_t (*cmd)(uint8_t argc, char *argv[]);
} gapCmds_t;

//typedef struct gapScannedDevices_tag
//{
//    bleAddressType_t    addrType;
//    bleDeviceAddress_t  aAddress;
//    uchar_t             name[10];
//}gapScannedDevices_t;
//
///************************************************************************************
//*************************************************************************************
//* Private functions prototypes
//*************************************************************************************
//************************************************************************************/
//
///* Shell GAP Events Callback */
//static void ShellGap_AdvertisingCallback
//(
//    gapAdvertisingEvent_t* pAdvertisingEvent
//);
//
//static void ShellGap_ScanningCallback
//(
//    gapScanningEvent_t* pScanningEvent
//);
//
//static void ShellGap_ConnectionCallback
//(
//    deviceId_t peerDeviceId,
//    gapConnectionEvent_t* pConnectionEvent
//);
//
/* Shell API Functions */
static int8_t ShellGap_DeviceName(uint8_t argc, char *argv[]);

static int8_t ShellGap_StartAdvertising(uint8_t argc, char *argv[]);
static int8_t ShellGap_StopAdvertising(uint8_t argc, char *argv[]);
static int8_t ShellGap_SetAdvertisingParameters(uint8_t argc, char *argv[]);

static int8_t ShellGap_StartScanning(uint8_t argc, char *argv[]);
static int8_t ShellGap_StopScanning(uint8_t argc, char *argv[]);
static int8_t ShellGap_SetScanParameters(uint8_t argc, char *argv[]);
static int8_t ShellGap_ChangeScanData(uint8_t argc, char *argv[]);

static int8_t ShellGap_Connect(uint8_t argc, char *argv[]);
static int8_t ShellGap_SetConnectionParameters(uint8_t argc, char *argv[]);
static int8_t ShellGap_Disconnect(uint8_t argc, char *argv[]);
static int8_t ShellGap_UpdateConnection(uint8_t argc, char *argv[]);

static int8_t ShellGap_Pair(uint8_t argc, char *argv[]);
static int8_t ShellGap_PairCfg(uint8_t argc, char *argv[]);
static int8_t ShellGap_EnterPasskey(uint8_t argc, char *argv[]);
static int8_t ShellGap_Bonds(uint8_t argc, char *argv[]);
/************************************************************************************
*************************************************************************************
* Private memory declarations
*************************************************************************************
************************************************************************************/
const gapCmds_t mGapShellCmds[mShellGapCmdsCount_c] =
{
    {"address",     ShellGap_DeviceAddress},
    {"devicename",  ShellGap_DeviceName},

    {"advstart",    ShellGap_StartAdvertising},
    {"advstop",     ShellGap_StopAdvertising},
    {"advcfg",      ShellGap_SetAdvertisingParameters},
    {"advdata",     ShellGap_ChangeAdvertisingData},

    {"scanstart",   ShellGap_StartScanning},
    {"scanstop",    ShellGap_StopScanning},
    {"scancfg",     ShellGap_SetScanParameters},
    {"scandata",    ShellGap_ChangeScanData},

    {"connect",     ShellGap_Connect},
    {"connectcfg",  ShellGap_SetConnectionParameters},
    {"disconnect",  ShellGap_Disconnect},
    {"connupdate",  ShellGap_UpdateConnection},

    {"pair",        ShellGap_Pair},
    {"paircfg",     ShellGap_PairCfg},
    {"enterpin",    ShellGap_EnterPasskey},
    {"bonds",       ShellGap_Bonds},

};

static tmrTimerID_t     mDelayTimerID = gTmrInvalidTimerID_c;
static bool_t           mSetDeviceName = FALSE;

//static bool_t mAdvOn = FALSE;
//static bool_t mIsBonded = FALSE;
static bool_t mScanningOn = FALSE;

static bool_t mIsMaster = FALSE;

//
//static gapConnectionParameters_t mConnectionParams;
//

/************************************************************************************
*************************************************************************************
* Public memory declarations
*************************************************************************************
************************************************************************************/
extern GAPSetAdvertisingParametersRequest_t     gAdvParams;
extern GAPSetAdvertisingDataRequest_t           gAppAdvertisingData;
extern GAPStartScanningRequest_t                gAppScanParams;
extern GAPConnectRequest_t                      gConnReqParams;
extern GAPPairRequest_t                         gPairingParameters;
//extern gapSmpKeys_t                 gSmpKeys;
//
deviceId_t   gPeerDeviceId = gInvalidDeviceId_c;
GAPAddDeviceToWhiteListRequest_AddressType_t mPeerDeviceAddressType;
uint8_t maPeerDeviceAddress[6];

char     gDeviceName[mMaxCharValueLength_d + 1] = "";
uint16_t gLatestHandle = INVALID_HANDLE;
uint16_t gLatestUuid = INVALID_HANDLE;
GAPScanningEventDeviceScannedIndication_t gScannedDevices[mShellGapMaxScannedDevicesCount_c];
uint8_t gScannedDevicesCount = 0;

typedef enum cmdState_tag
{
    gStartState_c,
    gGetServiceHandle_c,
    gServiceHandleReceived_c,
    gGetCharValueHandle_c,
    gCharValueHandleReceived_c,
    gReadAttribute_c,
    gAttrValueReceived_c,
    gWriteAttribute_c,
    gAttributeWritten_c,
    gError_c,
    gFinalState_c
} cmdState_t;

cmdState_t cmdState = gStartState_c;

/************************************************************************************
*************************************************************************************
* Public functions
*************************************************************************************
************************************************************************************/
int8_t ShellGap_Command(uint8_t argc, char *argv[])
{
    uint8_t i;

    if (argc < 2)
    {
        return CMD_RET_USAGE;
    }

    for (i = 0; i < mShellGapCmdsCount_c; i++)
    {
        if (!strcmp((char *)argv[1], mGapShellCmds[i].name))
        {
            return mGapShellCmds[i].cmd(argc - 2, (char **)(&argv[2]));
        }
    }

    return CMD_RET_USAGE;
}

int8_t ShellGap_DeviceAddress(uint8_t argc, char *argv[])
{
    switch (argc)
    {
        case 0:
        {
            GAPReadPublicDeviceAddressRequest(BLE_FSCI_IF);
            return CMD_RET_SUCCESS;
        }

        default:
            return CMD_RET_USAGE;
    }
}

int8_t ShellGap_ChangeAdvertisingData(uint8_t argc, char *argv[])
{
    switch (argc)
    {
        case 0:
        {
            shell_printf("\n\r-->  Advertising Data Included -> %d", gAppAdvertisingData.AdvertisingDataIncluded);
            shell_printf("\n\r-->  Advertising Data Structures -> %d", gAppAdvertisingData.AdvertisingData.NbOfAdStructures);
            return CMD_RET_SUCCESS;
        }
        break;

        case 1:
        {
            if (!strcmp((char *)argv[0], "-clear"))
            {
                // free allocated memory for adv data
                for (uint8_t i = 0; i < gAppAdvertisingData.AdvertisingData.NbOfAdStructures; i++)
                {
                    MEM_BufferFree(gAppAdvertisingData.AdvertisingData.AdStructures[i].Data);
                }

                MEM_BufferFree(gAppAdvertisingData.AdvertisingData.AdStructures);

                // set to 0
                FLib_MemSet(&gAppAdvertisingData.AdvertisingData, 0, sizeof(gAppAdvertisingData.AdvertisingData));
                gAppAdvertisingData.AdvertisingDataIncluded = FALSE;

                GAPSetAdvertisingDataRequest(&gAppAdvertisingData, BLE_FSCI_IF);
                shell_write("\n\r-->  Advertising Data Erased.");
                return CMD_RET_SUCCESS;
            }
        }
        break;

        default:
        {
            if (argv)
            {
                GAPSetAdvertisingDataRequest_AdvertisingData_AdStructures_Type_t advType =
                    (GAPSetAdvertisingDataRequest_AdvertisingData_AdStructures_Type_t)atoi(argv[0]);
                ShellGap_AppendAdvData(&gAppAdvertisingData, advType, argv[1]);
            }

            GAPSetAdvertisingDataRequest(&gAppAdvertisingData, BLE_FSCI_IF);
            return CMD_RET_SUCCESS;
        }
        break;
    }

    return CMD_RET_USAGE;
}

void ShellGap_AppendAdvData
(
    GAPSetAdvertisingDataRequest_t *pAdvData,
    GAPSetAdvertisingDataRequest_AdvertisingData_AdStructures_Type_t type,
    char *pData
)
{
    uint8_t i, advIdx = 0;
    uint8_t length;

    if (pAdvData->AdvertisingData.AdStructures == NULL)
    {
        pAdvData->AdvertisingData.AdStructures =
            MEM_BufferAllocForever(gcGapMaxAdStructures * sizeof(pAdvData->AdvertisingData.AdStructures[0]), 0);

        if (pAdvData->AdvertisingData.AdStructures == NULL)
        {
            shell_write("\n\r-->  GATTDB Event: Insufficient memory.");
            return;
        }

        FLib_MemSet(pAdvData->AdvertisingData.AdStructures, 0,
                    gcGapMaxAdStructures * sizeof(pAdvData->AdvertisingData.AdStructures[0]));
    }

    advIdx = pAdvData->AdvertisingData.NbOfAdStructures;

    // find existing entry
    for (i = 0; i < pAdvData->AdvertisingData.NbOfAdStructures; i++)
    {
        if (pAdvData->AdvertisingData.AdStructures[i].Type == type)
        {
            advIdx = i;
            MEM_BufferFree(pAdvData->AdvertisingData.AdStructures[advIdx].Data);
            break;
        }
    }

    length = strlen(pData);

    switch (type)
    {
        case GAPSetAdvertisingDataRequest_ScanResponseData_AdStructures_Type_gAdShortenedLocalName_c:
        case GAPSetAdvertisingDataRequest_ScanResponseData_AdStructures_Type_gAdCompleteLocalName_c:
        {
            if (!pAdvData->AdvertisingData.AdStructures[advIdx].Data)
            {
                pAdvData->AdvertisingData.AdStructures[advIdx].Data = MEM_BufferAllocForever(length, 0);

                if (!pAdvData->AdvertisingData.AdStructures[advIdx].Data)
                {
                    shell_write("\n\r-->  GATTDB Event: Insufficient memory.");
                    return;
                }
            }

            FLib_MemCpy(pAdvData->AdvertisingData.AdStructures[advIdx].Data, pData, length);
        }
        break;

        default:
        {
             // TODO why commented?
//            number = NWKU_AsciiToHex((uint8_t *)pData, length);
            /* halfen length - hex string */
//            length = (length % 2) ? (length / 2) + 1 : (length / 2);

            if (!pAdvData->AdvertisingData.AdStructures[i].Data)
            {
                pAdvData->AdvertisingData.AdStructures[advIdx].Data = MEM_BufferAllocForever(length, 0);

                if (!pAdvData->AdvertisingData.AdStructures[i].Data)
                {
                    shell_write("\n\r-->  GATTDB Event: Insufficient memory.");
                    return;
                }
            }

            FLib_MemCpy(pAdvData->AdvertisingData.AdStructures[advIdx].Data, pData, length);
        }
        break;
    }

    pAdvData->AdvertisingData.AdStructures[advIdx].Type = type;
    pAdvData->AdvertisingData.AdStructures[advIdx].Length = length;

    if (advIdx == pAdvData->AdvertisingData.NbOfAdStructures)
    {
        pAdvData->AdvertisingData.NbOfAdStructures += 1;
    }

    pAdvData->AdvertisingDataIncluded = TRUE;
}

/************************************************************************************
*************************************************************************************
* Private functions
*************************************************************************************
************************************************************************************/
static void ShellGap_DeviceNameSm(void *param)
{
    switch (cmdState)
    {
        case gGetServiceHandle_c:
            if (gLatestHandle != INVALID_HANDLE)
            {
                cmdState = gServiceHandleReceived_c;
            }
            else
            {
                cmdState = gError_c;
            }

            break;

        case gGetCharValueHandle_c:
            if (gLatestHandle != INVALID_HANDLE)
            {
                cmdState = gCharValueHandleReceived_c;
            }
            else
            {
                cmdState = gError_c;
            }

            break;

        case gReadAttribute_c:
            cmdState = gAttrValueReceived_c;
            break;

        case gWriteAttribute_c:
            cmdState = gAttributeWritten_c;
            break;

        default:
            cmdState = gError_c;
            break;
    }

    ShellGap_DeviceName(0, NULL);
}

static int8_t ShellGap_DeviceName(uint8_t argc, char *argv[])
{
    if (argc == 1)
    {
        uint8_t length = strlen(argv[0]);

        if (length > mMaxCharValueLength_d)
        {
            shell_write("\n\r-->  Variable length exceeds maximum!");
            return CMD_RET_FAILURE;
        }

        FLib_MemSet(gDeviceName, 0, sizeof(gDeviceName));
        FLib_MemCpy(gDeviceName, argv[0], length);
        gDeviceName[length] = 0;
        mSetDeviceName = TRUE;
    }

    switch (cmdState)
    {
        case gStartState_c:
        {
            gLatestHandle = INVALID_HANDLE;
            cmdState = gGetServiceHandle_c;

            GATTDBFindServiceHandleRequest_t req;
            req.StartHandle = 0x0001;  // should be 0x0001 on the first call
            req.UuidType = Uuid16Bits;
            UuidToArray(req.Uuid.Uuid16Bits, gBleSig_GenericAccessProfile_d);

            GATTDBFindServiceHandleRequest(&req, BLE_FSCI_IF);

            if (mDelayTimerID == gTmrInvalidTimerID_c)
            {
                mDelayTimerID = TMR_AllocateTimer();
            }

            TMR_StartSingleShotTimer(mDelayTimerID, 100, ShellGap_DeviceNameSm, argv[0]);

            break;
        }

        case gServiceHandleReceived_c:
        {
            cmdState = gGetCharValueHandle_c;

            GATTDBFindCharValueHandleInServiceRequest_t req;
            req.ServiceHandle = gLatestHandle;
            req.UuidType = Uuid16Bits;
            UuidToArray(req.Uuid.Uuid16Bits, gBleSig_GapDeviceName_d);
            GATTDBFindCharValueHandleInServiceRequest(&req, BLE_FSCI_IF);

            gLatestHandle = INVALID_HANDLE;
            TMR_StartSingleShotTimer(mDelayTimerID, 100, ShellGap_DeviceNameSm, NULL);
            break;
        }

        case gCharValueHandleReceived_c:
        {
            if (mSetDeviceName)
            {
                cmdState = gWriteAttribute_c;

                GATTDBWriteAttributeRequest_t req;
                req.Handle = gLatestHandle;
                req.ValueLength = strlen(gDeviceName);
                req.Value = (uint8_t *)gDeviceName;

                if (GATTDBWriteAttributeRequest(&req, BLE_FSCI_IF))
                {
                    shell_write("\n\r-->  GATTDB Event: Insufficient memory.");
                    SHELL_NEWLINE();
                }
            }
            else  // get
            {
                cmdState = gReadAttribute_c;

                GATTDBReadAttributeRequest_t req;
                req.Handle = gLatestHandle;
                req.MaxBytes = mMaxCharValueLength_d;

                gLatestUuid = gBleSig_GapDeviceName_d;
                GATTDBReadAttributeRequest(&req, BLE_FSCI_IF);
            }

            TMR_StartSingleShotTimer(mDelayTimerID, 100, ShellGap_DeviceNameSm, NULL);
            break;
        }

        case gAttrValueReceived_c:
        {
            cmdState = gStartState_c;
            shell_write("\n\r-->  GATTDB Event: Attribute Read ");
            shell_write("\n\r     Value: ");
            shell_write(gDeviceName);
            SHELL_NEWLINE();
            gDeviceName[strlen(gDeviceName)] = '>';
            gDeviceName[strlen(gDeviceName) + 1] = 0;
            break;
        }

        case gAttributeWritten_c:
        {
            cmdState = gStartState_c;
            mSetDeviceName = FALSE;

            shell_write("\n\r-->  GATTDB Event: Attribute Written ");
            SHELL_NEWLINE();

            gDeviceName[strlen(gDeviceName)] = '>';
            gDeviceName[strlen(gDeviceName) + 1] = 0;
            shell_change_prompt(gDeviceName);
            break;
        }

        case gError_c:
        {
            cmdState = gStartState_c;
            mSetDeviceName = FALSE;
            shell_write("\n\r-->  GATTDB Event: Procedure Error: make sure the GAP service was added! ");
            SHELL_NEWLINE();
            break;
        }

        case gFinalState_c:
        {
            cmdState = gStartState_c;
            mSetDeviceName = FALSE;
            break;
        }

        default:
            break;
    }

    return CMD_RET_SUCCESS;
}

static int8_t ShellGap_StartAdvertising(uint8_t argc, char *argv[])
{
    if (argc != 0)
    {
        return CMD_RET_USAGE;
    }

    mIsMaster = FALSE;
    GAPStartAdvertisingRequest(BLE_FSCI_IF);

    return CMD_RET_SUCCESS;
}

static int8_t ShellGap_StopAdvertising(uint8_t argc, char *argv[])
{
    if (argc != 0)
    {
        return CMD_RET_USAGE;
    }

    GAPStopAdvertisingRequest(BLE_FSCI_IF);

    return CMD_RET_SUCCESS;
}

static int8_t ShellGap_SetAdvertisingParameters(uint8_t argc, char *argv[])
{
    uint8_t i;
    bool_t bValidCmd = FALSE;

    if (argc == 0)
    {
        shell_write("\n\r-->  Advertising Parameters:");
        shell_write("\n\r    -->  Advertising Interval: ");
        shell_writeDec(gAdvParams.MaxInterval * 625 / 1000);
        shell_write(" ms");
        shell_write("\n\r    -->  Advertising Type: ");

        switch (gAdvParams.AdvertisingType)
        {
            case GAPSetAdvertisingParametersRequest_AdvertisingType_gConnectableUndirected_c:
            {
                shell_write("ADV_IND");
            }
            break;

            case GAPSetAdvertisingParametersRequest_AdvertisingType_gDirectedHighDutyCycle_c:
            {
                shell_write("ADV_IND");
            }
            break;

            case GAPSetAdvertisingParametersRequest_AdvertisingType_gNonConnectable_c:
            {
                shell_write("ADV_NONCON_IND");
            }
            break;

            case GAPSetAdvertisingParametersRequest_AdvertisingType_gScannable_c:
            {
                shell_write("ADV_SCAN_IND");
            }
            break;

            case GAPSetAdvertisingParametersRequest_AdvertisingType_gDirectedLowDutyCycle_c:
            {
                shell_write("ADV_DIRECT");
            }
            break;

            default:
                break;
        }

        SHELL_NEWLINE();
        return CMD_RET_SUCCESS;
    }

    for (i = 0; i < argc; i += 2)
    {
        if (!strcmp((char *)argv[i], "-interval") && ((i + 1) < argc))
        {
            uint16_t interval = atoi(argv[i + 1]) * 1000 / 625;
            gAdvParams.MaxInterval = interval;
            gAdvParams.MinInterval = interval;
            bValidCmd = TRUE;
        }

        if (!strcmp((char *)argv[i], "-type") && ((i + 1) < argc))
        {
            uint8_t advType = atoi(argv[i + 1]);
            gAdvParams.AdvertisingType = (GAPSetAdvertisingParametersRequest_AdvertisingType_t)advType;
            bValidCmd = TRUE;
        }
    }

    if (bValidCmd)
    {
        GAPSetAdvertisingParametersRequest(&gAdvParams, BLE_FSCI_IF);
        return CMD_RET_SUCCESS;
    }
    else
    {
        return CMD_RET_USAGE;
    }
}

static void ShellGap_AppendScanData
(
    GAPSetAdvertisingDataRequest_t *pAdvData,
    GAPSetAdvertisingDataRequest_ScanResponseData_AdStructures_Type_t type,
    char *pData
)
{
    uint8_t i, advIdx = 0;
    uint8_t length;
    uint32_t number;

    if (pAdvData->ScanResponseData.AdStructures == NULL)
    {
        pAdvData->ScanResponseData.AdStructures =
            MEM_BufferAlloc(gcGapMaxAdStructures * sizeof(pAdvData->ScanResponseData.AdStructures[0]));

        if (pAdvData->ScanResponseData.AdStructures == NULL)
        {
            shell_write("\n\r-->  GATTDB Event: Insufficient memory.");
            return;
        }
    }

    advIdx = pAdvData->ScanResponseData.NbOfAdStructures;

    for (i = 0; i < pAdvData->ScanResponseData.NbOfAdStructures; i++)
    {
        if (pAdvData->ScanResponseData.AdStructures[i].Type == type)
        {
            advIdx = i;
            break;
        }
    }

    length = strlen(pData);

    switch (type)
    {
        case GAPSetAdvertisingDataRequest_ScanResponseData_AdStructures_Type_gAdShortenedLocalName_c:
        case GAPSetAdvertisingDataRequest_ScanResponseData_AdStructures_Type_gAdCompleteLocalName_c:
        {
            pAdvData->ScanResponseData.AdStructures[advIdx].Data = MEM_BufferAlloc(length);

            if (!pAdvData->ScanResponseData.AdStructures[advIdx].Data)
            {
                shell_write("\n\r-->  GATTDB Event: Insufficient memory.");
                return;
            }

            FLib_MemCpy(pAdvData->ScanResponseData.AdStructures[advIdx].Data, pData, length);
        }
        break;

        default:
        {
            number = NWKU_AsciiToHex((uint8_t *)pData, length);
            /* halfen length - hex string */
            length = (length % 2) ? (length / 2) + 1 : (length / 2);

            pAdvData->ScanResponseData.AdStructures[advIdx].Data = MEM_BufferAlloc(length);

            if (!pAdvData->ScanResponseData.AdStructures[i].Data)
            {
                shell_write("\n\r-->  GATTDB Event: Insufficient memory.");
                return;
            }

            FLib_MemCpy(pAdvData->ScanResponseData.AdStructures[advIdx].Data, &number, length);
        }
        break;
    }

    pAdvData->ScanResponseData.AdStructures[advIdx].Type = type;
    pAdvData->ScanResponseData.AdStructures[advIdx].Length = length;
    pAdvData->ScanResponseData.NbOfAdStructures += 1;
    pAdvData->ScanResponseDataIncluded = TRUE;
}

static int8_t ShellGap_StartScanning(uint8_t argc, char *argv[])
{
    if (argc != 0)
    {
        return CMD_RET_USAGE;
    }

    for (uint8_t i = 0; i < gScannedDevicesCount; i++)
    {
        MEM_BufferFree(gScannedDevices[i].Data);
    }

    FLib_MemSet(gScannedDevices, 0, sizeof(gScannedDevices));
    gScannedDevicesCount = 0;

    GAPStartScanningRequest(&gAppScanParams, BLE_FSCI_IF);
    mScanningOn = TRUE;
    mIsMaster = TRUE;

    return CMD_RET_SUCCESS;
}

static int8_t ShellGap_StopScanning(uint8_t argc, char *argv[])
{
    if (argc != 0)
    {
        return CMD_RET_USAGE;
    }

    GAPStopScanningRequest(BLE_FSCI_IF);
    mScanningOn = FALSE;

    return CMD_RET_SUCCESS;
}

static int8_t ShellGap_SetScanParameters(uint8_t argc, char *argv[])
{
    uint8_t i;
    bool_t bValidCmd;

    if (argc == 0)
    {
        shell_write("\n\r-->  Scan Parameters:");
        shell_write("\n\r    -->  Scan Interval: ");
        shell_writeDec(gAppScanParams.ScanningParameters.Interval * 625 / 1000);
        shell_write(" ms");
        shell_write("\n\r    -->  Scan Window: ");
        shell_writeDec(gAppScanParams.ScanningParameters.Window * 625 / 1000);
        shell_write(" ms");
        shell_write("\n\r    -->  Scan Type: ");
        (gAppScanParams.ScanningParameters.Type) ? shell_write("ACTIVE") : shell_write("PASSIVE");
        SHELL_NEWLINE();
        return CMD_RET_SUCCESS;
    }

    for (i = 0; i < argc; i += 2)
    {
        if (!strcmp((char *)argv[i], "-interval") && ((i + 1) < argc))
        {
            uint16_t interval = atoi(argv[i + 1]) * 1000 / 625;
            gAppScanParams.ScanningParameters.Interval = interval;
            gConnReqParams.ScanInterval = interval;
            bValidCmd = TRUE;
        }

        if (!strcmp((char *)argv[i], "-window") && ((i + 1) < argc))
        {
            uint16_t window = atoi(argv[i + 1]) * 1000 / 625;
            gAppScanParams.ScanningParameters.Window = window;
            gConnReqParams.ScanWindow = window;
            bValidCmd = TRUE;
        }

        if (!strcmp((char *)argv[i], "-type") && ((i + 1) < argc))
        {
            uint8_t scanType = atoi(argv[i + 1]);
            gAppScanParams.ScanningParameters.Type = (GAPStartScanningRequest_ScanningParameters_Type_t)scanType;
            bValidCmd = TRUE;
        }
    }

    if (bValidCmd)
    {
        return CMD_RET_SUCCESS;
    }
    else
    {
        return CMD_RET_USAGE;
    }
}

static int8_t ShellGap_ChangeScanData(uint8_t argc, char *argv[])
{
    switch (argc)
    {
        case 0:
        {
            shell_printf("\n\r-->  Scan Response Data Included -> %d", gAppAdvertisingData.ScanResponseDataIncluded);
            shell_printf("\n\r-->  Scan Response Data Structures -> %d", gAppAdvertisingData.ScanResponseData.NbOfAdStructures);
            return CMD_RET_SUCCESS;
        }
        break;

        case 1:
        {
            if (!strcmp((char *)argv[0], "-clear"))
            {
                // free allocated memory for scan data
                for (uint8_t i = 0; i < gAppAdvertisingData.ScanResponseData.NbOfAdStructures; i++)
                {
                    MEM_BufferFree(gAppAdvertisingData.ScanResponseData.AdStructures[i].Data);
                }

                MEM_BufferFree(gAppAdvertisingData.ScanResponseData.AdStructures);

                // set to 0
                FLib_MemSet(&gAppAdvertisingData.ScanResponseData, 0, sizeof(gAppAdvertisingData.ScanResponseData));
                gAppAdvertisingData.ScanResponseDataIncluded = FALSE;

                GAPSetAdvertisingDataRequest(&gAppAdvertisingData, BLE_FSCI_IF);
                shell_write("\n\r-->  Scan Data Erased.");
                return CMD_RET_SUCCESS;
            }
        }
        break;

        default:
        {
            GAPSetAdvertisingDataRequest_ScanResponseData_AdStructures_Type_t advType =
                (GAPSetAdvertisingDataRequest_ScanResponseData_AdStructures_Type_t)atoi(argv[0]);
            ShellGap_AppendScanData(&gAppAdvertisingData, advType, argv[1]);
            GAPSetAdvertisingDataRequest(&gAppAdvertisingData, BLE_FSCI_IF);
            return CMD_RET_SUCCESS;
        }
        break;
    }

    return CMD_RET_USAGE;
}

static int8_t ShellGap_Connect(uint8_t argc, char *argv[])
{
    uint8_t deviceId;

    if (argc != 1)
    {
        return CMD_RET_USAGE;
    }

    deviceId = (uint8_t)atoi(argv[0]);

    if (deviceId > gScannedDevicesCount)
    {
        shell_write("Device ID does not exist!");
        SHELL_NEWLINE();
        return CMD_RET_USAGE;
    }

    mIsMaster = TRUE;

    gConnReqParams.ScanInterval = gAppScanParams.ScanningParameters.Interval;
    gConnReqParams.ScanWindow = gAppScanParams.ScanningParameters.Window;
    gConnReqParams.PeerAddressType = (GAPConnectRequest_PeerAddressType_t)gScannedDevices[deviceId].AddressType;
    FLib_MemCpy(gConnReqParams.PeerAddress, gScannedDevices[deviceId].Address, sizeof(gConnReqParams.PeerAddress));

    /* Stop scanning if it's on */
    if (mScanningOn)
    {
        GAPStopScanningRequest(BLE_FSCI_IF);
    }

    GAPConnectRequest(&gConnReqParams, BLE_FSCI_IF);

    return CMD_RET_SUCCESS;
}

static int8_t ShellGap_SetConnectionParameters(uint8_t argc, char *argv[])
{
    uint8_t i;
    bool_t bValidCmd = FALSE;

    if (argc == 0)
    {
        bValidCmd = TRUE;
    }

    for (i = 0; i < argc; i += 2)
    {
        if (!strcmp((char *)argv[i], "-interval") && ((i + 1) < argc))
        {
            uint16_t interval = atoi(argv[i + 1]) * 8 / 10;
            gConnReqParams.ConnIntervalMin = interval;
            gConnReqParams.ConnIntervalMax = interval;
            bValidCmd = TRUE;
        }

        if (!strcmp((char *)argv[i], "-latency") && ((i + 1) < argc))
        {
            uint16_t latency = atoi(argv[i + 1]);
            gConnReqParams.ConnLatency = latency;
            bValidCmd = TRUE;
        }

        if (!strcmp((char *)argv[i], "-timeout") && ((i + 1) < argc))
        {
            uint16_t timeout = atoi(argv[i + 1]) / 10;
            gConnReqParams.SupervisionTimeout = timeout;
            bValidCmd = TRUE;
        }
    }

    if (bValidCmd)
    {
        shell_write("\n\r-->  Connection Parameters:");
        shell_write("\n\r    -->  Connection Interval: ");
        shell_writeDec(gConnReqParams.ConnIntervalMax * 10 / 8);
        shell_write(" ms");

        shell_write("\n\r    -->  Connection Latency: ");
        shell_writeDec(gConnReqParams.ConnLatency);

        shell_write("\n\r    -->  Supervision Timeout: ");
        shell_writeDec(gConnReqParams.SupervisionTimeout * 10);
        shell_write(" ms\n\r");

        SHELL_NEWLINE();
        return CMD_RET_SUCCESS;
    }
    else
    {
        return CMD_RET_USAGE;
    }
}

static int8_t ShellGap_Disconnect(uint8_t argc, char *argv[])
{
    if (argc != 0)
    {
        return CMD_RET_USAGE;
    }

    GAPDisconnectRequest_t req;
    req.DeviceId = gPeerDeviceId;  // the connected peer to disconnect from
    GAPDisconnectRequest(&req, BLE_FSCI_IF);

    return CMD_RET_SUCCESS;
}

static int8_t ShellGap_UpdateConnection(uint8_t argc, char *argv[])
{
    GAPUpdateConnectionParametersRequest_t req;

    if (argc != 4)
    {
        return CMD_RET_USAGE;
    }

    req.DeviceId = gPeerDeviceId;
    req.IntervalMin = atoi(argv[0]) * 8 / 10;
    req.IntervalMax = atoi(argv[1]) * 8 / 10;
    req.SlaveLatency = atoi(argv[2]);
    req.TimeoutMultiplier = atoi(argv[3]) / 10;
    req.MinCeLength = gConnReqParams.ConnEventLengthMin;
    req.MaxCeLength = gConnReqParams.ConnEventLengthMax;

    GAPUpdateConnectionParametersRequest(&req, BLE_FSCI_IF);
    return CMD_RET_SUCCESS;
}

static int8_t ShellGap_Pair(uint8_t argc, char *argv[])
{
    if (argc != 0)
    {
        return CMD_RET_USAGE;
    }

    if (gPeerDeviceId == gInvalidDeviceId_c)
    {
        shell_write("\n\r-->  Please connect the node first...");
        return CMD_RET_FAILURE;
    }

    shell_write("\n\r-->  Pairing...\n\r");

    if (mIsMaster)
    {
        gPairingParameters.DeviceId = gPeerDeviceId;
        GAPPairRequest(&gPairingParameters, BLE_FSCI_IF);
    }
    else
    {
        GAPSendSlaveSecurityRequestRequest_t req;
        req.DeviceId = gPeerDeviceId;
        req.BondAfterPairing = gPairingParameters.PairingParameters.WithBonding;
        req.SecurityModeLevel = (GAPSendSlaveSecurityRequestRequest_SecurityModeLevel_t)gPairingParameters.PairingParameters.SecurityModeAndLevel;
        GAPSendSlaveSecurityRequestRequest(&req, BLE_FSCI_IF);
    }

    return CMD_RET_SUCCESS;
}

static int8_t ShellGap_PairCfg(uint8_t argc, char *argv[])
{
    uint8_t i;
    bool_t bValidCmd = FALSE;

    if (argc == 0)
    {
        bValidCmd = TRUE;
    }

    for (i = 0; i < argc; i += 2)
    {
        if (!strcmp((char *)argv[i], "-usebonding") && ((i + 1) < argc))
        {
            uint8_t usebonding = atoi(argv[i + 1]);
            gPairingParameters.PairingParameters.WithBonding = usebonding ? TRUE : FALSE;
            bValidCmd = TRUE;
        }

        if (!strcmp((char *)argv[i], "-seclevel") && ((i + 1) < argc))
        {
            uint8_t level = strtoul(argv[i + 1], NULL, 16);

            if ((level & 0x0F) <= gSecurityLevel_WithMitmProtection_c &&
                    (level & 0xF0) <= gSecurityMode_2_c)
            {
                gPairingParameters.PairingParameters.SecurityModeAndLevel =
                    (GAPPairRequest_PairingParameters_SecurityModeAndLevel_t)level;
                bValidCmd = TRUE;
            }
        }

        if (!strcmp((char *)argv[i], "-keyflags") && ((i + 1) < argc))
        {
            uint8_t flags = strtoul(argv[i + 1], NULL, 16);
            gPairingParameters.PairingParameters.CentralKeys =
                (GAPPairRequest_PairingParameters_CentralKeys_t)(flags & 0x07);
            gPairingParameters.PairingParameters.PeripheralKeys =
                (GAPPairRequest_PairingParameters_PeripheralKeys_t)(flags & 0x07);
            bValidCmd = TRUE;
        }
    }

    if (bValidCmd)
    {
        shell_write("\n\r-->  Pairing Configuration:");
        shell_write("\n\r    -->  Use Bonding: ");
        shell_writeBool(gPairingParameters.PairingParameters.WithBonding);

        shell_write("\n\r    -->  SecurityLevel: ");
        shell_writeHex((uint8_t *)&gPairingParameters.PairingParameters.SecurityModeAndLevel, 1);

        shell_write("\n\r    -->  Flags: ");
        shell_writeHex((uint8_t *)&gPairingParameters.PairingParameters.CentralKeys, 1);

        SHELL_NEWLINE();
        return CMD_RET_SUCCESS;
    }
    else
    {
        return CMD_RET_USAGE;
    }
}

static int8_t ShellGap_EnterPasskey(uint8_t argc, char *argv[])
{
    GAPEnterPasskeyRequest_t req;

    if (argc != 1)
    {
        return CMD_RET_USAGE;
    }

    req.DeviceId = gPeerDeviceId;
    req.Passkey = atoi(argv[0]);

    GAPEnterPasskeyRequest(&req, BLE_FSCI_IF);

    return CMD_RET_SUCCESS;
}

static int8_t ShellGap_Bonds(uint8_t argc, char *argv[])
{
    // uint8_t i = 0, count = 0;

    // switch (argc)
    // {
    //     case 0:
    //     {
    //         /* Get number of bonded devices */
    //         Gap_GetBondedDevicesCount(&count);

    //         if (!count)
    //         {
    //             shell_write("\n\r-->  No bonds found on the device! \n\r");
    //             return CMD_RET_SUCCESS;
    //         }
    //         else
    //         {
    //             void *pBuffer = NULL;

    //             /* Allocate buffer for name */
    //             pBuffer = MEM_BufferAlloc(mShellGapMaxDeviceNameLength_c);

    //             if (!pBuffer)
    //             {
    //                 return CMD_RET_FAILURE;
    //             }

    //             for (i = 0; i < count; i++)
    //             {
    //                 shell_write("\n\r-->  ");
    //                 shell_writeDec(i);
    //                 shell_write(". ");
    //                 Gap_GetBondedDeviceName(i, pBuffer, mShellGapMaxDeviceNameLength_c);

    //                 if (strlen(pBuffer) > 0)
    //                 {
    //                     shell_write(pBuffer);
    //                 }
    //                 else
    //                 {
    //                     shell_write("[No saved name]");
    //                 }
    //             }

    //             SHELL_NEWLINE();
    //             return CMD_RET_SUCCESS;
    //         }
    //     }
    //     break;

    //     case 1:
    //     {
    //         if (!strcmp((char *)argv[0], "-erase"))
    //         {
    //             if (gPeerDeviceId != gInvalidDeviceId_c)
    //             {
    //                 shell_write("\n\r-->  Please disconnect the node first...");
    //                 return CMD_RET_FAILURE;
    //             }

    //             if (Gap_RemoveAllBonds() == gBleSuccess_c)
    //             {
    //                 shell_write("\n\r-->  Bonds removed!\n\r");
    //                 return CMD_RET_SUCCESS;
    //             }
    //             else
    //             {
    //                 shell_write("\n\r-->  Operation failed!\n\r");
    //                 return CMD_RET_FAILURE;
    //             }
    //         }
    //     }
    //     break;

    //     case 2:
    //     {
    //         if (!strcmp((char *)argv[0], "-remove"))
    //         {
    //             uint8_t index = atoi(argv[1]);

    //             if (gPeerDeviceId != gInvalidDeviceId_c)
    //             {
    //                 shell_write("\n\r-->  Please disconnect the node first...\n\r");
    //                 return CMD_RET_FAILURE;
    //             }

    //             /* Get number of bonded devices */
    //             Gap_GetBondedDevicesCount(&count);

    //             if ((count > 0) && (index < count) &&
    //                     (Gap_RemoveBond(index) == gBleSuccess_c))
    //             {
    //                 shell_write("\n\r-->  Bond removed!\n\r");
    //                 return CMD_RET_SUCCESS;
    //             }
    //             else
    //             {
    //                 shell_write("\n\r-->  Operation failed!\n\r");
    //                 return CMD_RET_FAILURE;
    //             }
    //         }
    //     }
    //     break;

    //     default:
    //         break;
    // }

    return CMD_RET_USAGE;
}
//
//static void ShellGap_ParseScannedDevice(gapScannedDevice_t* pData)
//{
//    uint8_t index = 0;
//    uint8_t nameLength;
//
//    while (index < pData->dataLength)
//    {
//        gapAdStructure_t adElement;
//
//        adElement.length = pData->data[index];
//        adElement.adType = (gapAdType_t)pData->data[index + 1];
//        adElement.aData = &pData->data[index + 2];
//
//        if ((adElement.adType == gAdShortenedLocalName_c) ||
//          (adElement.adType == gAdCompleteLocalName_c))
//        {
//            nameLength = MIN(adElement.length-1, mShellGapMaxDeviceNameLength_c);
//            FLib_MemCpy(mScannedDevices[mScannedDevicesCount].name, adElement.aData, nameLength);
//
//            shell_write((char*)mScannedDevices[mScannedDevicesCount].name);
//            shell_write(" ");
//        }
//
//        /* Move on to the next AD elemnt type */
//        index += adElement.length + sizeof(uint8_t);
//    }
//
//    shell_writeHexLe(pData->aAddress, sizeof(bleDeviceAddress_t));
//    shell_write(" ");
//
//    /* Temporary store scanned data to use for connection */
//    mScannedDevices[mScannedDevicesCount].addrType = pData->addressType;
//    FLib_MemCpy(mScannedDevices[mScannedDevicesCount].aAddress,
//                pData->aAddress,
//                sizeof(bleDeviceAddress_t));
//
//    shell_writeSignedDec(pData->rssi);
//    shell_write(" dBm");
//
//    mScannedDevicesCount++;
//}
//
///*! *********************************************************************************
// * \brief        Handles BLE generic callback.
// *
// * \param[in]    pGenericEvent    Pointer to gapGenericEvent_t.
// ********************************************************************************** */
//void ShellGap_GenericCallback (gapGenericEvent_t* pGenericEvent)
//{
//    shell_write("\n\r-->  GAP Event: ");
//
//    switch(pGenericEvent->eventType)
//    {
//        case gPublicAddressRead_c:
//        {
//            shell_write("Public Address Read:");
//            shell_writeHex(pGenericEvent->eventData.aAddress, sizeof(bleDeviceAddress_t));
//            break;
//        }
//
//        case gAdvertisingDataSetupComplete_c:
//        {
//            shell_write("Advertising data successfully set.");
//            break;
//        }
//
//        case gAdvertisingParametersSetupComplete_c:
//        {
//            shell_write("Advertising parameters successfully set.");
//            break;
//        }
//
//        case gAdvertisingSetupFailed_c:
//        {
//            shell_write("Advertising setup failed.");
//            break;
//        }
//
//        case gInternalError_c:
//        {
//
//        }
//        break;
//
//        default:
//            break;
//    }
//    SHELL_NEWLINE();
//    shell_cmd_finished();
//}
//
///*! *********************************************************************************
// * \brief        Handles BLE Advertising callback from host stack.
// *
// * \param[in]    pAdvertisingEvent    Pointer to gapAdvertisingEvent_t.
// ********************************************************************************** */
//static void ShellGap_AdvertisingCallback
//(
//    gapAdvertisingEvent_t* pAdvertisingEvent
//)
//{
//    shell_write("\n\r-->  GAP Event: Advertising ");
//
//    switch (pAdvertisingEvent->eventType)
//    {
//        case gAdvertisingStateChanged_c:
//        {
//            mAdvOn = !mAdvOn;
//
//            if (mAdvOn)
//            {
//                shell_write("started.\n\r");
//            }
//            else
//            {
//                shell_write("stopped.\n\r");
//            }
//            break;
//        }
//
//        case gAdvertisingCommandFailed_c:
//        {
//            shell_write("state could not be changed.\n\r");
//            break;
//        }
//
//    default:
//        break;
//    }
//
//    shell_cmd_finished();
//}
//
///*! *********************************************************************************
// * \brief        Handles BLE Connection callback from host stack.
// *
// * \param[in]    peerDeviceId        Peer device ID.
// * \param[in]    pConnectionEvent    Pointer to gapConnectionEvent_t.
// ********************************************************************************** */
//static void ShellGap_ConnectionCallback
//(
//    deviceId_t peerDeviceId,
//    gapConnectionEvent_t* pConnectionEvent
//)
//{
//    switch (pConnectionEvent->eventType)
//    {
//        case gConnEvtConnected_c:
//        {
//            gPeerDeviceId = peerDeviceId;
//            shell_write("\n\r-->  GAP Event: Connected\n\r");
//            shell_cmd_finished();
//
//            mAdvOn = FALSE;
//            mIsBonded = FALSE;
//
//            Gap_CheckIfBonded(peerDeviceId, &mIsBonded);
//
//            if ((mIsBonded) &&
//            (gBleSuccess_c == Gap_LoadCustomPeerInformation(peerDeviceId, NULL, 0 ,0)))
//            {
//                /* Restored custom connection information. Encrypt link */
//                Gap_EncryptLink(peerDeviceId);
//            }
//
//            /* Save connection parameters */
//            FLib_MemCpy(&mConnectionParams,
//                        &pConnectionEvent->eventData.connectedEvent.connParameters,
//                        sizeof(gapConnectionParameters_t));
//        }
//        break;
//
//        case gConnEvtDisconnected_c:
//        {
//            gPeerDeviceId = gInvalidDeviceId_c;
//            shell_write("\n\r-->  GAP Event: Disconnected\n\r");
//            shell_cmd_finished();
//        }
//        break;
//
//        case gConnEvtPairingRequest_c:
//        {
//            Gap_AcceptPairingRequest(peerDeviceId, &gPairingParameters);
//        }
//        break;
//
//        case gConnEvtKeyExchangeRequest_c:
//        {
//            gapSmpKeys_t sentSmpKeys = gSmpKeys;
//
//            if (!(pConnectionEvent->eventData.keyExchangeRequestEvent.requestedKeys & gLtk_c))
//            {
//                sentSmpKeys.aLtk = NULL;
//                /* When the LTK is NULL EDIV and Rand are not sent and will be ignored. */
//            }
//
//            if (!(pConnectionEvent->eventData.keyExchangeRequestEvent.requestedKeys & gIrk_c))
//            {
//                sentSmpKeys.aIrk = NULL;
//                /* When the IRK is NULL the Address and Address Type are not sent and will be ignored. */
//            }
//
//            if (!(pConnectionEvent->eventData.keyExchangeRequestEvent.requestedKeys & gCsrk_c))
//            {
//                sentSmpKeys.aCsrk = NULL;
//            }
//
//            Gap_SendSmpKeys(peerDeviceId, &sentSmpKeys);
//        }
//        break;
//
//        case gConnEvtPasskeyRequest_c:
//        {
//            shell_write("\n\r\n\r-->  GAP Event: PIN required\n\r");
//            shell_cmd_finished();
//        }
//        break;
//
//        case gConnEvtPasskeyDisplay_c:
//        {
//            shell_write("\n\r-->  GAP Event: Passkey is ");
//            shell_writeDec(pConnectionEvent->eventData.passkeyForDisplay);
//            SHELL_NEWLINE();
//            shell_cmd_finished();
//        }
//        break;
//
//        case gConnEvtLongTermKeyRequest_c:
//        {
//            if (pConnectionEvent->eventData.longTermKeyRequestEvent.ediv == gSmpKeys.ediv &&
//                pConnectionEvent->eventData.longTermKeyRequestEvent.randSize == gSmpKeys.cRandSize)
//            {
//                Gap_LoadEncryptionInformation(peerDeviceId, gSmpKeys.aLtk, &gSmpKeys.cLtkSize);
//                /* EDIV and RAND both matched */
//                Gap_ProvideLongTermKey(peerDeviceId, gSmpKeys.aLtk, gSmpKeys.cLtkSize);
//            }
//            else
//            /* EDIV or RAND size did not match */
//            {
//                Gap_DenyLongTermKey(peerDeviceId);
//            }
//            break;
//        }
//
//        case gConnEvtPairingComplete_c:
//        {
//            if (pConnectionEvent->eventData.pairingCompleteEvent.pairingSuccessful)
//            {
//                shell_write("\n\r-->  GAP Event: Device Paired.\n\r");
//
//                /* Save Device Name in NVM*/
//                Gap_SaveDeviceName(peerDeviceId, mScannedDevices[mConnectToDeviceId].name, strlen((char*)mScannedDevices[mConnectToDeviceId].name));
//            }
//            else
//            {
//                shell_write("\n\r-->  GAP Event: Pairing Unsuccessful.\n\r");
//            }
//            shell_cmd_finished();
//        }
//        break;
//
//        case gConnEvtLeScDisplayNumericValue_c:
//        {
//            (void) pConnectionEvent->eventData.numericValueForDisplay;
//            /* Display on a screen for user confirmation then validate/invalidate based on value. */
//            Gap_LeScValidateNumericValue(peerDeviceId, TRUE);
//        }
//        break;
//
//        case gConnEvtEncryptionChanged_c:
//        {
//            if (pConnectionEvent->eventData.encryptionChangedEvent.newEncryptionState)
//            {
//                shell_write("\n\r-->  GAP Event: Link Encrypted.\n\r");
//            }
//            else
//            {
//                shell_write("\n\r-->  GAP Event: Link Not Encrypted.\n\r");
//            }
//            shell_cmd_finished();
//        }
//        break;
//
//        case gConnEvtParameterUpdateRequest_c:
//        {
//            Gap_EnableUpdateConnectionParameters(peerDeviceId, TRUE);
//        }
//        break;
//
//        case gConnEvtParameterUpdateComplete_c:
//        {
//            if (pConnectionEvent->eventData.connectionUpdateComplete.status == gBleSuccess_c)
//            {
//                mConnectionParams.connInterval = pConnectionEvent->eventData.connectionUpdateComplete.connInterval;
//                mConnectionParams.connLatency = pConnectionEvent->eventData.connectionUpdateComplete.connLatency;
//                mConnectionParams.supervisionTimeout = pConnectionEvent->eventData.connectionUpdateComplete.supervisionTimeout;
//
//                shell_write("\n\r-->  GAP Event: Connection Parameters Changed.");
//                shell_write("\n\r   -->Connection Interval: ");
//                shell_writeDec(mConnectionParams.connInterval * 10/8);
//                shell_write(" ms");
//                shell_write("\n\r   -->Connection Latency: ");
//                shell_writeDec(mConnectionParams.connLatency);
//                shell_write("\n\r   -->Supervision Timeout: ");
//                shell_writeDec(mConnectionParams.supervisionTimeout * 10);
//                shell_write(" ms");
//                shell_cmd_finished();
//            }
//            else
//            {
//                shell_write("\n\r-->  GAP Event: Connection Parameters Did Not Change.");
//            }
//        }
//        break;
//
//        default:
//        break;
//    }
//}
//
///*! *********************************************************************************
// * \brief        Handles BLE Scanning callback from host stack.
// *
// * \param[in]    pScanningEvent    Pointer to gapScanningEvent_t.
// ********************************************************************************** */
//static void ShellGap_ScanningCallback (gapScanningEvent_t* pScanningEvent)
//{
//    switch (pScanningEvent->eventType)
//    {
//        case gDeviceScanned_c:
//        {
//            if (mScannedDevicesCount >= mShellGapMaxScannedDevicesCount_c)
//            {
//                break;
//            }
//
//            shell_write("\n\r-->  GAP Event: Found device ");
//            shell_writeDec(mScannedDevicesCount);
//            shell_write(" : ");
//            ShellGap_ParseScannedDevice(&pScanningEvent->eventData.scannedDevice);
//            break;
//        }
//
//        case gScanStateChanged_c:
//        {
//            mScanningOn = !mScanningOn;
//
//            shell_write("\n\r->  GAP Event: Scan ");
//            if (mScanningOn)
//            {
//                mScannedDevicesCount = 0;
//                shell_write("started.\n\r");
//            }
//            else
//            {
//                shell_write("stopped.\n\r");
//            }
//            shell_cmd_finished();
//            break;
//        }
//
//    case gScanCommandFailed_c:
//    {
//        shell_write("\n\r-->  GAP Event: Scan state could not be changed.");
//        shell_cmd_finished();
//        break;
//    }
//    default:
//        break;
//    }
//}
//
//void ShellGap_L2caControlCallback
//(
//    l2capControlMessageType_t  messageType,
//    void* pMessage
//)
//{
//    switch (messageType)
//    {
//
//
//    default:
//      break;
//    }
//}
//
///*! *********************************************************************************
// * @}
// ********************************************************************************** */

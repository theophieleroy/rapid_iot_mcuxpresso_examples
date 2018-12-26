/*! *********************************************************************************
* \addtogroup App Config
* @{
********************************************************************************** */
/*!
* Copyright (c) 2015, Freescale Semiconductor, Inc.
* Copyright 2016-2017 NXP
*
* \file
*
* This file contains configuration data for the application and stack
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
#include "cmd_ble.h"

/************************************************************************************
*************************************************************************************
* Private macros
*************************************************************************************
************************************************************************************/

/************************************************************************************
*************************************************************************************
* Public memory declarations
*************************************************************************************
************************************************************************************/

/* Scanning and Advertising Data */
GAPSetAdvertisingDataRequest_t gAppAdvertisingData = { 0 };



GAPStartScanningRequest_t gAppScanParams =
{
    /* scanning parameters included */  TRUE, \
    {
        \
        /* type */                          GAPStartScanningRequest_ScanningParameters_Type_gPassive_c, \
        /* interval */                      0x0010, \
        /* window */                        0x0010, \
        /* ownAddressType */                GAPStartScanningRequest_ScanningParameters_OwnAddressType_gPublic_c, \
        /* filterPolicy */                  GAPStartScanningRequest_ScanningParameters_FilterPolicy_gScanAll_c \
    }
};

#define gGapAdvertisingInterval_050ms_c 0x0050
#define gGapAdvertisingInterval_100ms_c 0x00A0
#define gGapAdvertisingInterval_125ms_c 0x00C8
#define gGapAdvertisingInterval_250ms_c 0x0190
#define gGapAdvertisingInterval_500ms_c 0x0320

GAPSetAdvertisingParametersRequest_t gAdvParams =
{
    /* minInterval */         gGapAdvertisingInterval_050ms_c /* 1 s */, \
    /* maxInterval */         gGapAdvertisingInterval_100ms_c /* 1 s */, \
    /* advertisingType */     GAPSetAdvertisingParametersRequest_AdvertisingType_gConnectableUndirected_c, \
    /* addressType */         GAPSetAdvertisingParametersRequest_OwnAddressType_gPublic_c, \
    /* directedAddressType */ GAPSetAdvertisingParametersRequest_PeerAddressType_gPublic_c, \
    /* directedAddress */     {0, 0, 0, 0, 0, 0}, \
    /* channelMap */          GAPSetAdvertisingParametersRequest_ChannelMap_gChannel37_c | GAPSetAdvertisingParametersRequest_ChannelMap_gChannel38_c | GAPSetAdvertisingParametersRequest_ChannelMap_gChannel39_c, \
    /* filterPolicy */        GAPSetAdvertisingParametersRequest_FilterPolicy_gProcessAll_c \
};

/* Default Connection Request Parameters */
GAPConnectRequest_t gConnReqParams =
{
    .ScanInterval = 0x0010,
    .ScanWindow = 0x0010,
    .FilterPolicy = GAPConnectRequest_FilterPolicy_gUseDeviceAddress_c,
    .OwnAddressType = GAPConnectRequest_OwnAddressType_gPublic_c,
    .ConnIntervalMin = 0x00A0,
    .ConnIntervalMax = 0x00A0,
    .ConnLatency = 0x0000,
    .SupervisionTimeout = 0x0C80,
    .ConnEventLengthMin = 0x0000,
    .ConnEventLengthMax = 0xFFFF,
};

GAPPairRequest_t gPairingParameters =
{
    .PairingParameters = {
        .WithBonding = FALSE,
        .SecurityModeAndLevel = GAPPairRequest_PairingParameters_SecurityModeAndLevel_gMode1Level1_c,
        .MaxEncryptionKeySize = mcEncryptionKeySize_c,
        .LocalIoCapabilities = GAPPairRequest_PairingParameters_LocalIoCapabilities_gIoDisplayOnly_c,
        .OobAvailable = FALSE,
        .CentralKeys = GAPPairRequest_PairingParameters_CentralKeys_gNoKeys_c,
        .PeripheralKeys = GAPPairRequest_PairingParameters_PeripheralKeys_gNoKeys_c,
        .LeSecureConnectionSupported = TRUE,
        .UseKeypressNotifications = FALSE
    }
};

/* This structure is similar to GAPPairRequest_t, in order to simplify the process in case of a API
 * change you need to manually set the paring parameters identical to the one declared in gPairingParameters
 */
GAPSetDefaultPairingParametersRequest_t gParingParametersRequest =
{
	.PairingParametersIncluded = TRUE,
	.PairingParameters = {
		.WithBonding = FALSE,
		.SecurityModeAndLevel = GAPSetDefaultPairingParametersRequest_PairingParameters_SecurityModeAndLevel_gMode1Level1_c,
		.MaxEncryptionKeySize = mcEncryptionKeySize_c,
		.LocalIoCapabilities  = GAPSetDefaultPairingParametersRequest_PairingParameters_LocalIoCapabilities_gIoDisplayOnly_c,
		.OobAvailable = FALSE,
		.CentralKeys = GAPSetDefaultPairingParametersRequest_PairingParameters_CentralKeys_gNoKeys_c,
		.PeripheralKeys = GAPSetDefaultPairingParametersRequest_PairingParameters_PeripheralKeys_gNoKeys_c,
		.LeSecureConnectionSupported = TRUE,
		.UseKeypressNotifications = FALSE
	}
};

/* LTK */
static uint8_t smpLtk[mcEncryptionKeySize_c] =
    {0xD6, 0x93, 0xE8, 0xA4, 0x23, 0x55, 0x48, 0x99,
     0x1D, 0x77, 0x61, 0xE6, 0x63, 0x2B, 0x10, 0x8E};

/* RAND*/
static uint8_t smpRand[gcSmpMaxRandSize_c] =
    {0x26, 0x1E, 0xF6, 0x09, 0x97, 0x2E, 0xAD, 0x7E};

/* IRK */
static uint8_t smpIrk[gcSmpIrkSize_c] =
    {0x0A, 0x2D, 0xF4, 0x65, 0xE3, 0xBD, 0x7B, 0x49,
     0x1E, 0xB4, 0xC0, 0x95, 0x95, 0x13, 0x46, 0x73};

/* CSRK */
static uint8_t smpCsrk[gcSmpCsrkSize_c] =
    {0x90, 0xD5, 0x06, 0x95, 0x92, 0xED, 0x91, 0xD7,
     0xA8, 0x9E, 0x2C, 0xDC, 0x4A, 0x93, 0x5B, 0xF9};

gapSmpKeys_t gSmpKeys = {
    .cLtkSize = mcEncryptionKeySize_c,
    .aLtk = (void *)smpLtk,
    .aIrk = (void *)smpIrk,
    .aCsrk = (void *)smpCsrk,
    .aRand = (void *)smpRand,
    .cRandSize = gcSmpMaxRandSize_c,
    .ediv = smpEdiv,
};

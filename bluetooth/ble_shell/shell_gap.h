/*! *********************************************************************************
 * \defgroup SHELL GAP
 * @{
 ********************************************************************************** */
/*!
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 *
 * \file
 *
 * This file is the interface file for the GAP Shell module
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

#ifndef _SHELL_GAP_H_
#define _SHELL_GAP_H_

#include "cmd_ble.h"

/*************************************************************************************
**************************************************************************************
* Public macros
**************************************************************************************
*************************************************************************************/
#define BLE_FSCI_IF     1

#define INVALID_HANDLE                      0xFFFF

#define gcGapMaxAdStructures                (5)
#define gInvalidDeviceId_c                  (0xFF)
#define mShellGapCmdsCount_c                20
#define mShellGapMaxScannedDevicesCount_c   10
#define mShellGapMaxDeviceNameLength_c      20

/************************************************************************************
*************************************************************************************
* Public memory declarations
*************************************************************************************
********************************************************************************** */
/************************************************************************************
*************************************************************************************
* Public prototypes
*************************************************************************************
************************************************************************************/
typedef uint8_t deviceId_t;

/*! LE Security Level */
typedef enum gapSecurityLevel_tag
{
    gSecurityLevel_NoSecurity_c =           0x00,  /*!< No security (combined only with Mode 1). */
    gSecurityLevel_NoMitmProtection_c =     0x01,  /*!< Unauthenticated (no MITM protection). */
    gSecurityLevel_WithMitmProtection_c =   0x02,  /*!< Authenticated (MITM protection by PIN or OOB). */
    gSecurityLevel_LeSecureConnections_c =  0x03   /*!< Authenticated with LE Secure Connections (BLE 4.2 only). */
} gapSecurityLevel_t;

/*! LE Security Mode */
typedef enum gapSecurityMode_tag
{
    gSecurityMode_1_c = 0x10,   /*!< Mode 1 - Encryption required (except for Level 1). */
    gSecurityMode_2_c = 0x20    /*!< Mode 2 - Data Signing required. */
} gapSecurityMode_t;

#ifdef __cplusplus
extern "C" {
#endif

int8_t ShellGap_Command(uint8_t argc, char *argv[]);
int8_t ShellGap_DeviceAddress(uint8_t argc, char *argv[]);
int8_t ShellGap_ChangeAdvertisingData(uint8_t argc, char *argv[]);


void ShellGap_AppendAdvData(GAPSetAdvertisingDataRequest_t *pAdvData,
                            GAPSetAdvertisingDataRequest_AdvertisingData_AdStructures_Type_t type,
                            char *pData);

//void    ShellGap_GenericCallback (gapGenericEvent_t* pGenericEvent);
//
//void ShellGap_L2caControlCallback
//(
//    l2capControlMessageType_t  messageType,
//    void* pMessage
//);

#ifdef __cplusplus
}
#endif


#endif /* _SHELL_GAP_H_ */

/*! *********************************************************************************
 * @}
 ********************************************************************************** */

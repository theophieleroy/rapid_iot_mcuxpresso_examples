/*! *********************************************************************************
 * \defgroup SHELL GATTDB
 * @{
 ********************************************************************************** */
/*!
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 *
 * \file
 *
 * This file is the interface file for the GATTDB Shell module
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

#ifndef _SHELL_GATTDB_H_
#define _SHELL_GATTDB_H_

/*************************************************************************************
**************************************************************************************
* Public macros
**************************************************************************************
*************************************************************************************/
#define BLE_FSCI_IF     1

#define mMaxCharValueLength_d               23
/*! Maximum possible value of the ATT_MTU for this device. This is used during the MTU Exchange. */
#define gAttMaxMtu_c                        (247)
/*! Default value of the ATT_MTU */
#define gAttDefaultMtu_c                (23)

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

#ifdef __cplusplus
extern "C" {
#endif

int8_t ShellGattDb_Command(uint8_t argc, char *argv[]);
int8_t ShellGattDb_Write(uint8_t argc, char *argv[]);
void ShellGattDb_RegisterForWriteNotifications(uint8_t count, uint8_t* handler);
void ShellGattDb_SendAttributeWrittenStatusRequest(uint8_t DeviceId, uint16_t AttributeHandle, uint8_t Status);

void ShellGattDb_AddServiceGatt(void);
void ShellGattDb_AddServiceGap(void);
void ShellGattDb_AddServiceHeartRate(void);
void ShellGattDb_AddServiceOtap(void);
void ShellGattDb_AddServiceBattery(void);
void ShellGattDb_AddServiceDeviceInfo(void);
void ShellGattDb_AddServiceWeather(void);
void ShellGattDb_AddServiceMotion(void);
void ShellGattDb_AddServiceInteraction(void);
void ShellGattDb_AddServiceAuth(void);
void ShellGattDb_RegisterCallback(void);

#ifdef __cplusplus
}
#endif

#endif /* _SHELL_GATTDB_H_ */

/*! *********************************************************************************
 * @}
 ********************************************************************************** */

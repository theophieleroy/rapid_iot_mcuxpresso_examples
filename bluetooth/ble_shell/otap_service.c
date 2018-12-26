/*! *********************************************************************************
 * \addtogroup BLE OTAP Custom Profile
 * @{
 ********************************************************************************** */
/*!
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 *
 * \file
 *
 * This file contains the source for the BLE OTAP Service/Profile
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
#include "otap_interface.h"
#include "cmd_ble.h"
#include "shell_gattdb.h"
#include "shell_gatt.h"
#include "shell_gap.h"
#include "TimersManager.h"
static tmrTimerID_t     mDelayTimerID = gTmrInvalidTimerID_c;

/************************************************************************************
*************************************************************************************
* External variables
*************************************************************************************
************************************************************************************/
extern uint8_t uuid_char_otap_control_point[16];

/************************************************************************************
*************************************************************************************
* Private type definitions
*************************************************************************************
************************************************************************************/

/************************************************************************************
*************************************************************************************
* Private memory declarations
*************************************************************************************
************************************************************************************/
/*! OTAP Client Service - Subscribed Client*/
static deviceId_t mOtapCS_SubscribedClientId;

/************************************************************************************
*************************************************************************************
* Private functions prototypes
*************************************************************************************
************************************************************************************/
static void OtapCS_SendControlPointIndication (uint16_t handle);

/************************************************************************************
*************************************************************************************
* Public functions
*************************************************************************************
************************************************************************************/
void OtapCS_Start ()
{
    mOtapCS_SubscribedClientId = gInvalidDeviceId_c;

    return;
}

void OtapCS_Stop ()
{
    OtapCS_Unsubscribe();
}

void OtapCS_Subscribe(deviceId_t deviceId)
{
    mOtapCS_SubscribedClientId = deviceId;
}

void OtapCS_Unsubscribe()
{
    mOtapCS_SubscribedClientId = gInvalidDeviceId_c;
}

deviceId_t OtapCS_GetSubscribed()
{
    return mOtapCS_SubscribedClientId;
}

void OtapCS_SendCommandToOtapServer (uint16_t charHandle,
                                void* pCommand,
                                uint16_t cmdLength)
{
    GATTDBWriteAttributeRequest_t req;
    req.Handle = charHandle;
    req.ValueLength = cmdLength;
    req.Value = pCommand;

    GATTDBWriteAttributeRequest(&req, BLE_FSCI_IF);

    if (mDelayTimerID == gTmrInvalidTimerID_c)
    {
        mDelayTimerID = TMR_AllocateTimer();
    }
    TMR_StartSingleShotTimer(mDelayTimerID, (tmrTimeInMilliseconds_t)250, (pfTmrCallBack_t)OtapCS_SendControlPointIndication, (void *)((uint32_t)charHandle));

    /* Send Command to the OTAP Server via ATT Indication */
    //OtapCS_SendControlPointIndication (charHandle);
}


/************************************************************************************
*************************************************************************************
* Private functions
*************************************************************************************
************************************************************************************/

/*!**********************************************************************************
* \brief        Sends an ATT Indications for the specified handle
*               of an OTAP Control Point characteristic.
*
* \param[in]    handle   Characteristic handle.
*
* \return       gBleSuccess_c or error.
************************************************************************************/
static void OtapCS_SendControlPointIndication (uint16_t handle)
{
    GATTServerSendIndicationRequest_t req;
    req.DeviceId = mOtapCS_SubscribedClientId;
    req.Handle = handle;
    GATTServerSendIndicationRequest(&req, BLE_FSCI_IF);

#if DEBUG_BLE_OTA
//    shell_write("\r\nOtapCS_SendControlPointIndication");
#endif
}

/*! *********************************************************************************
 * @}
 ********************************************************************************** */

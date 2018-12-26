/* mbed Microcontroller Library
 * Copyright (c) 2006-2013 ARM Limited
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "nxpInstance.h"

extern "C" {
#include <stdlib.h>
#include "fsl_common.h"
#include "fsl_os_abstraction.h"
#include "app.h"
#include "qnble.h"
#include "mbed_gatt_database_dynamic.h"

void initComplete(gapServiceHandles_t handles);
void bleReset(void);
void BleConnManager_GapCommonConfig(void);
}

/**
 * The singleton which represents the NXP transport for the BLE.
 */
static nxpInstance deviceInstance;

/**
 * BLE-API requires an implementation of the following function in order to
 * obtain its transport handle.
 */
BLEInstanceBase *createBLEInstance(void)
{
    return &nxpInstance::Instance(BLE::DEFAULT_INSTANCE);
}

nxpInstance &nxpInstance::Instance(BLE::InstanceID_t instanceId)
{
    return deviceInstance;
}

nxpInstance::nxpInstance(void)
    : initialized(false),
      instanceID(BLE::DEFAULT_INSTANCE),
      gapInstance(),
      gattServerInstance(NULL),
      securityManagerInstance(NULL) //,
//    gattClientInstance(NULL)
{
}

nxpInstance::~nxpInstance(void)
{
}

const char *nxpInstance::getVersion(void)
{
    // FIXME - Correct the version string
    static char version[] = "Dummy Version 1.0";
    return version;
}

/**************************************************************************/
/*!
    @brief  Initialize the BLE stack.

    @returns    ble_error_t

    @retval     BLE_ERROR_NONE if everything executed properly and
                BLE_ERROR_ALREADY_INITIALIZED if the stack has already
                been initialized (possibly through a call to nxpInstance::init()).
                BLE_ERROR_INTERNAL_STACK_FAILURE is returned if initialization
                of the internal stack (SoftDevice) failed.

*/
/**************************************************************************/
ble_error_t nxpInstance::init(BLE::InstanceID_t instanceID,
                              FunctionPointerWithContext<BLE::InitializationCompleteCallbackContext *> callback)
{
    uint32_t primask;

    if (initialized)
    {
        BLE::InitializationCompleteCallbackContext context = {BLE::Instance(instanceID), BLE_ERROR_ALREADY_INITIALIZED};
        callback.call(&context);
        return BLE_ERROR_ALREADY_INITIALIZED;
    }

    instanceID = instanceID;
    init_callback = callback;

    primask = DisableGlobalIRQ();

    /* Board pin, clock, debug console init */
    BOARD_InitHardware();

    BLE_Init();

    nxpble_init();

    EnableGlobalIRQ(primask);

    return BLE_ERROR_NONE;
}

void nxpInstance::invokeCallback(void)
{
    initialized = true;
    BLE::InitializationCompleteCallbackContext context = {BLE::Instance(instanceID), BLE_ERROR_NONE};
    init_callback.call(&context);
}

void initComplete(gapServiceHandles_t handles)
{
    /* Call Gap Configuration */
    BleConnManager_GapCommonConfig();
    ((nxpGap &)(deviceInstance.getGap())).gapInit(handles);
    deviceInstance.invokeCallback();
}

void bleReset(void)
{
    if (deviceInstance.hasInitialized())
    {
        deviceInstance.getGattServer().reset();
    }
}

/**************************************************************************/
/*!
    @brief  Purge the BLE stack of GATT and GAP state.

    @returns    ble_error_t

    @retval     BLE_ERROR_NONE
                Everything executed properly

    @note  When using S110, GattClient::shutdown() will not be called
           since Gatt client features are not supported.
*/
/**************************************************************************/
ble_error_t nxpInstance::shutdown(void)
{
    if (!initialized)
    {
        return BLE_ERROR_INITIALIZATION_INCOMPLETE;
    }

    /* Shutdown the BLE API and glue code */
    ble_error_t error;

    if (gattServerInstance != NULL)
    {
        error = gattServerInstance->reset();
        if (error != BLE_ERROR_NONE)
        {
            return error;
        }
    }

    if (securityManagerInstance != NULL)
    {
        error = securityManagerInstance->reset();
        if (error != BLE_ERROR_NONE)
        {
            return error;
        }
    }

    //    if (gattClientInstance != NULL) {
    //        error = gattClientInstance->reset();
    //        if (error != BLE_ERROR_NONE) {
    //            return error;
    //        }
    //    }

    /* Gap instance is always present */
    error = gapInstance.reset();
    if (error != BLE_ERROR_NONE)
    {
        return error;
    }

    initialized = false;
    return BLE_ERROR_NONE;
}

void nxpInstance::waitForEvent(void)
{
    // FIXME - ToDo
}

void nxpInstance::processEvents()
{
    // FIXME - Todo
}

#if 0
void Controller_Init(void)
{
}

void Controller_TaskHandler(void)
{
}

#endif
int16_t RNG_GetPseudoRandomNo(uint8_t *pOut, uint8_t outBytes, uint8_t *pXSEED)
{
    uint32_t temp;
    for (uint8_t i = 0; i < outBytes; i += 4)
    {
        if (outBytes - i < 4)
        {
            temp = rand();
            for (uint8_t j = 0; j < outBytes - i; j++)
            {
                pOut[i + j] = (temp >> (j * 8)) & 0xFF;
            }
        }
        else
            *(uint32_t *)(&pOut[i]) = rand();
    }
    return outBytes;
}
void Controller_SendSingleAdvertisement(void)
{
}

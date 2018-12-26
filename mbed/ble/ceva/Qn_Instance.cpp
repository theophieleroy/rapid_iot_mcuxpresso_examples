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

#include "Qn_Instance.h"

extern "C" {
#include "fsl_common.h"
#include "app.h"
#include "app_ble.h"
void initComplete(void);
void bleReset(void);
}

/**
 * The singleton which represents the QN transport for the BLE.
 */
static QN_BLEInstance deviceInstance;

/**
 * BLE-API requires an implementation of the following function in order to
 * obtain its transport handle.
 */
BLEInstanceBase *createBLEInstance(void)
{
    return &QN_BLEInstance::Instance(BLE::DEFAULT_INSTANCE);
}

QN_BLEInstance& QN_BLEInstance::Instance(BLE::InstanceID_t instanceId)
{
    return deviceInstance;
}

QN_BLEInstance::QN_BLEInstance(void) :
    initialized(false),
    instanceID(BLE::DEFAULT_INSTANCE),
    gapInstance(),
    gattServerInstance(NULL)//,
//    gattClientInstance(NULL),
//    securityManagerInstance(NULL)
{
}

QN_BLEInstance::~QN_BLEInstance(void)
{
}

const char *QN_BLEInstance::getVersion(void)
{
    //FIXME - Correct the version string for CEVA Stack
    static char version[] = "Dummy Version 1.0";
    return version;
}

/**************************************************************************/
/*!
    @brief  Initialize the BLE stack.

    @returns    ble_error_t

    @retval     BLE_ERROR_NONE if everything executed properly and
                BLE_ERROR_ALREADY_INITIALIZED if the stack has already
                been initialized (possibly through a call to nRF5xn::init()).
                BLE_ERROR_INTERNAL_STACK_FAILURE is returned if initialization
                of the internal stack (SoftDevice) failed.

*/
/**************************************************************************/
ble_error_t QN_BLEInstance::init(BLE::InstanceID_t instanceID, FunctionPointerWithContext<BLE::InitializationCompleteCallbackContext *> callback)
{
    uint32_t primask;
    
    if (initialized) {
        BLE::InitializationCompleteCallbackContext context = {
            BLE::Instance(instanceID),
            BLE_ERROR_ALREADY_INITIALIZED
        };
        callback.call(&context);
        return BLE_ERROR_ALREADY_INITIALIZED;
    }

    instanceID   = instanceID;
    init_callback = callback;

    primask = DisableGlobalIRQ();

    BOARD_InitHardware();

    BLE_Init();

    APP_Init();

    EnableGlobalIRQ(primask);
    
    return BLE_ERROR_NONE;
}


void QN_BLEInstance::invokeCallback(void)
{
    initialized = true;
    BLE::InitializationCompleteCallbackContext context = {
        BLE::Instance(instanceID),
        BLE_ERROR_NONE
    };
    init_callback.call(&context);
}


void initComplete(void)
{
    deviceInstance.invokeCallback();
}

void bleReset(void)
{
    if(deviceInstance.hasInitialized())
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
ble_error_t QN_BLEInstance::shutdown(void)
{
    if (!initialized) {
        return BLE_ERROR_INITIALIZATION_INCOMPLETE;
    }

    /* Shutdown the BLE API and nRF51 glue code */
    ble_error_t error;

    if (gattServerInstance != NULL) {
        error = gattServerInstance->reset();
        if (error != BLE_ERROR_NONE) {
            return error;
        }
    }

//    if (securityManagerInstance != NULL) {
//        error = securityManagerInstance->reset();
//        if (error != BLE_ERROR_NONE) {
//            return error;
//        }
//    }

//    if (gattClientInstance != NULL) {
//        error = gattClientInstance->reset();
//        if (error != BLE_ERROR_NONE) {
//            return error;
//        }
//    }

    /* Gap instance is always present */
    error = gapInstance.reset();
    if (error != BLE_ERROR_NONE) {
        return error;
    }

    initialized = false;
    return BLE_ERROR_NONE;
}

void QN_BLEInstance::waitForEvent(void)
{
    //FIXME - ToDo
}

void QN_BLEInstance::processEvents() {
    //FIXME - Todo
}

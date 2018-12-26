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

#ifndef __BLE_H__
#define __BLE_H__

#include "blecommon.h"
#include "mbedGap.h"
#include "GattServer.h"
//#include "GattClient.h"

#include "FunctionPointerWithContext.h"

//#ifdef YOTTA_CFG_MBED_OS
//#include "mbed-drivers/mbed_error.h"
//#else
//#include "mbed_error.h"
//#endif

/* Forward declaration for the implementation class */
class BLEInstanceBase;

/**
 * The base class used to abstract away BLE-capable radio transceivers or SOCs,
 * so that the BLE API can work with any radio transparently.
 */
class BLE
{
   public:
    typedef unsigned InstanceID_t; /**< The type returned by BLE::getInstanceID(). */

    /**
     * The context provided to init-completion-callbacks (see init() below).
     *
     * @param  ble
     *             A reference to the BLE instance being initialized.
     * @param  error
     *             Captures the result of initialization. It is set to
     *             BLE_ERROR_NONE if initialization completed successfully. Else
     *             the error value is implementation specific.
     */
    struct InitializationCompleteCallbackContext
    {
        BLE &ble;          /**< Reference to the BLE object that has been initialized */
        ble_error_t error; /**< Error status of the initialization. It is set to BLE_ERROR_NONE if initialization
                              completed successfully. */
    };

    /**
     * The signature for function-pointer like callbacks for initialization-completion.
     *
     * @note There are two versions of init(). In addition to the simple
     *     function-pointer, init() can also take a <Object, member> tuple as its
     *     callback target. In case of the latter, the following declaration doesn't apply.
     */
    typedef void (*InitializationCompleteCallback_t)(InitializationCompleteCallbackContext *context);

    /**
     * Initialize the BLE controller. This should be called before using
     * anything else in the BLE API.
     *
     * init() hands control to the underlying BLE module to accomplish
     * initialization. This initialization may tacitly depend on other hardware
     * setup (such as clocks or power-modes) that happens early on during
     * system startup. It may not be safe to call init() from a global static
     * context where ordering is compiler-specific and can't be guaranteed - it
     * is safe to call BLE::init() from within main().
     *
     * @param  initCompleteCallback
     *           A callback for when initialization completes for a BLE
     *           instance. This is an optional parameter; if no callback is
     *           set up the application can still determine the status of
     *           initialization using BLE::hasInitialized() (see below).
     *
     * @return  BLE_ERROR_NONE if the initialization procedure was started
     *     successfully.
     *
     * @note If init() returns BLE_ERROR_NONE, the underlying stack must invoke
     *     the initialization completion callback at some point.
     *
     * @note Nearly all BLE APIs would return
     *     BLE_ERROR_INITIALIZATION_INCOMPLETE if used on an instance before the
     *     corresponding transport is initialized.
     *
     * @note There are two versions of init(). In addition to the simple
     *     function-pointer, init() can also take an <Object, member> tuple as its
     *     callback target.
     */
    ble_error_t init(InitializationCompleteCallback_t initCompleteCallback = NULL)
    {
        FunctionPointerWithContext<InitializationCompleteCallbackContext *> callback(initCompleteCallback);
        return initImplementation(callback);
    }

    /**
     * An alternate declaration for init(). This one takes an <Object, member> tuple as its
     * callback target.
     */
    template <typename T>
    ble_error_t init(T *object, void (T::*initCompleteCallback)(InitializationCompleteCallbackContext *context))
    {
        FunctionPointerWithContext<InitializationCompleteCallbackContext *> callback(object, initCompleteCallback);
        return initImplementation(callback);
    }

    /**
     * @return true if initialization has completed for the underlying BLE
     *     transport.
     *
     * The application can set up a callback to signal completion of
     * initialization when using init(). Otherwise, this method can be used to
     * poll the state of initialization.
     */
    bool hasInitialized(void) const;

    /**
     * Purge the BLE stack of GATT and GAP state. init() must be called
     * afterwards to re-instate services and GAP state. This API offers a way to
     * repopulate the GATT database with new services and characteristics.
     */
    ble_error_t shutdown(void);

    /**
     * This call allows the application to get the BLE stack version information.
     *
     * @return  A pointer to a const string representing the version.
     *
     * @note The string returned is owned by BLE API.
     */
    const char *getVersion(void);

    /**
     * Accessor to Gap. All Gap related functionality requires
     * going through this accessor.
     *
     * @return A reference to a Gap object associated to this BLE instance.
     */
    Gap &gap();

    /**
     * A const alternative to gap().
     *
     * @return A const reference to a Gap object associated to this BLE instance.
     */
    const Gap &gap() const;

    /**
     * Accessor to GattServer. All GattServer related functionality requires
     * going through this accessor.
     *
     * @return A reference to a GattServer object associated to this BLE instance.
     */
    GattServer &gattServer();

    /**
     * A const alternative to gattServer().
     *
     * @return A const reference to a GattServer object associated to this BLE instance.
     */
    const GattServer &gattServer() const;

    /**
     * Accessors to GattClient. All GattClient related functionality requires going
     * through this accessor.
     *
     * @return A reference to a GattClient object associated to this BLE instance.
     */
    // GattClient& gattClient();

    /**
     * A const alternative to gattClient().
     *
     * @return A const reference to a GattClient object associated to this BLE instance.
     */
    // const GattClient& gattClient() const;

    /**
     * Accessors to SecurityManager. All SecurityManager related functionality requires
     * going through this accessor.
     *
     * @return A reference to a SecurityManager object associated to this BLE instance.
     */
    SecurityManager &securityManager();

    /**
     * A const alternative to securityManager().
     *
     * @return A const reference to a SecurityManager object associated to this BLE instance.
     */
    const SecurityManager &securityManager() const;

    /**
     * Yield control to the BLE stack or to other tasks waiting for events. This
     * is a sleep function that will return when there is an application-specific
     * interrupt, but the MCU might wake up several times before
     * returning (to service the stack). This is not always interchangeable with
     * WFE().
     */
    void waitForEvent(void);

   public:
    /**
     * The value of the BLE::InstanceID_t for the default BLE instance.
     */
    static const InstanceID_t DEFAULT_INSTANCE = 0;
#ifndef YOTTA_CFG_BLE_INSTANCES_COUNT
    /**
     * The number of permitted BLE instances for the application.
     */
    static const InstanceID_t NUM_INSTANCES = 1;
#else
    /**
     * The number of permitted BLE instances for the application.
     */
    static const InstanceID_t NUM_INSTANCES = YOTTA_CFG_BLE_INSTANCES_COUNT;
#endif

    /**
     * Get a reference to the BLE singleton corresponding to a given interface.
     * There is a static array of BLE singletons.
     *
     * @note Calling Instance() is preferred over constructing a BLE object
     * directly, as it returns references to singletons.
     *
     * @param[in] id
     *              Instance-ID. This should be less than NUM_INSTANCES
     *              for the returned BLE singleton to be useful.
     *
     * @return A reference to a single object.
     */
    static BLE &Instance(InstanceID_t id = DEFAULT_INSTANCE);

    /**
     * Constructor for a handle to a BLE instance (the BLE stack). BLE handles
     * are thin wrappers around a transport object (that is, ptr. to
     * BLEInstanceBase).
     *
     * It is better to create BLE objects as singletons accessed through the
     * Instance() method. If multiple BLE handles are constructed for the same
     * interface (using this constructor), they will share the same underlying
     * transport object.
     */
    BLE(InstanceID_t instanceID = DEFAULT_INSTANCE);

    /**
     * Fetch the ID of a BLE instance. Typically there would only be the DEFAULT_INSTANCE.
     */
    InstanceID_t getInstanceID(void) const
    {
        return instanceID;
    }

   private:
    /**
     * Implementation of init() [internal to BLE_API].
     *
     * The implementation is separated into a private method because it isn't
     * suitable to be included in the header.
     */
    ble_error_t initImplementation(FunctionPointerWithContext<InitializationCompleteCallbackContext *> callback);

   private:
    BLE(const BLE &);
    BLE &operator=(const BLE &);

   private:
    InstanceID_t instanceID;
    BLEInstanceBase *transport; /* The device-specific backend */
};

typedef BLE BLEDevice; /**< @deprecated This type alias is retained for the
                        * sake of compatibility with older
                        * code. Will be dropped at some point soon.*/

#endif /* ifndef __BLE_H__ */

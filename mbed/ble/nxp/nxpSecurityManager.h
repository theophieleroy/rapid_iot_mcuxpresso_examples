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

#ifndef __NXP_SECURITY_MANAGER_H__
#define __NXP_SECURITY_MANAGER_H__

#include <stddef.h>

#include "SecurityManager.h"
#include "mbedGap.h"
extern "C" {
#include "gap_types.h"
#include "MemManager.h"
}
class nxpSecurityManager : public SecurityManager
{
   public:
    typedef enum LinkSecurityState_tag
    {
        kNotEncrypted,         /**< The link is not secured. */
        kEncryptionInProgress, /**< Link security is being established.*/
        kEncrypted             /**< The link is secure.*/
    } LinkSecurityState_t;

    /* Functions that must be implemented from SecurityManager */
    virtual ble_error_t init(bool enableBonding,
                             bool requireMITM,
                             SecurityIOCapabilities_t iocaps,
                             const Passkey_t passkey);
    virtual ble_error_t getLinkSecurity(Gap::Handle_t connectionHandle, LinkSecurityStatus_t *securityStatusP);
    virtual ble_error_t setLinkSecurity(Gap::Handle_t connectionHandle, SecurityMode_t securityMode);
    virtual ble_error_t purgeAllBondingState(void);
    virtual ble_error_t getAddressesFromBondTable(Gap::Whitelist_t &addresses) const;
    virtual ble_error_t reset(void);

    bool hasInitialized(void) const;
    gapSecurityRequirements_t *getMasterSecurityReq(void);
    LinkSecurityState_t getLinkSecurityState(deviceId_t peerDeviceId);
    void setLinkSecurityState(deviceId_t peerDeviceId, LinkSecurityState_t linkSecState);

    nxpSecurityManager()
    {
        /* empty */
    }

    nxpSecurityManager(const nxpSecurityManager &);
    const nxpSecurityManager &operator=(const nxpSecurityManager &);
    void registerInitialSecurity(void);
    void ProcessSecSetupCompletedEvent(deviceId_t peerDeviceId, gapConnectionEvent_t *pConnectionEvent);
    void ProcessPasskeyDispEvent(deviceId_t peerDeviceId, uint32_t passkey);
    void ProcessSecurityEvent(deviceId_t peerDeviceId, gapConnectionEvent_t *pConnectionEvent);

   private:
    gapSecurityRequirements_t masterSecurityReq; /* Used to provide a pointer to master Securiy requirements*/
    LinkSecurityState_t mLinkSecurityState[10];  /* To maintain Link security state of a connection handle */
};

#endif /* ifndef __NXP_SECURITY_MANAGER_H__ */

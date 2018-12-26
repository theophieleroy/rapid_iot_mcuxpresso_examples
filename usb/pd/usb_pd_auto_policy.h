/*
 * The Clear BSD License
 * Copyright 2018 NXP
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided
 * that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS LICENSE.
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

#ifndef __USB_PD_AUTO_POLICY_H__
#define __USB_PD_AUTO_POLICY_H__

/*******************************************************************************
 * Definitions
 ******************************************************************************/

typedef enum _usb_pd_auto_accept_value
{
    /*! don't support */
    kAutoRequestProcess_NotSupport = 0x00u,
    /*! auto accept request */
    kAutoRequestProcess_Accept = 0x01u,
    /*! auto reject request */
    kAutoRequestProcess_Reject = 0x02u,
    /*! auto reply wait for request */
    kAutoRequestProcess_Wait = 0x03u,
} usb_pd_auto_accept_value_t;

typedef struct _usb_pd_auto_policy
{
    /*! #pd_power_role_t values
     * kPD_PowerRoleSink : auto request as sink.
     * kPD_PowerRoleSource : auto request as source.
     * kPD_PowerRoleNone : don't support auto request.
     */
    uint32_t autoRequestPRSwap : 2;
    /*! accept swap as source or not */
    uint32_t autoAcceptPRSwapAsSource : 2;
    /*! accept swap as sink or not */
    uint32_t autoAcceptPRSwapAsSink : 2;
    /*! #pd_data_role_t values
     * kPD_DataRoleUFP : auto request as UFP.
     * kPD_DataRoleDFP : auto request as DFP.
     * kPD_DataRoleNone : don't support auto request.
     */
    uint32_t autoRequestDRSwap : 2;
    /*! accept swap as DFP or not */
    uint32_t autoAcceptDRSwapAsDFP : 2;
    /*! accept swap as UFP or not */
    uint32_t autoAcceptDRSwapAsUFP : 2;
    /*! #pd_vconn_role_t values
     * kPD_NotVconnSource : auto request to turn off vconn.
     * kPD_IsVconnSource : auto request to turn on vconn.
     * kPD_VconnNone : don't support auto request.
     */
    uint32_t autoRequestVConnSwap : 2;
    /*! accept swap to trun on Vconn or not */
    uint32_t autoAcceptVconnSwapAsOn : 2;
    /*! accept swap to trun off Vconn or not */
    uint32_t autoAcceptVconnSwapAsOff : 2;
    /*! reserved bits */
    uint32_t reserved : 14;
} pd_auto_policy_t;

/*******************************************************************************
 * API
 ******************************************************************************/

#define PD_POLICY_SUPPORT(pdHandle) (((pd_instance_t *)pdHandle)->pdPowerPortConfig->autoPolicyConfig != NULL)

#define PD_POLICY_GET_AUTO_REQUEST_PRSWAP(pdHandle) \
    ((pd_auto_policy_t *)(((pd_instance_t *)pdHandle)->pdPowerPortConfig->autoPolicyConfig))->autoRequestPRSwap
#define PD_POLICY_GET_AUTO_ACCEPT_PRSWAP_AS_SOURCE_SUPPORT(pdHandle)                          \
    (((pd_auto_policy_t *)(((pd_instance_t *)pdHandle)->pdPowerPortConfig->autoPolicyConfig)) \
         ->autoAcceptPRSwapAsSource != kAutoRequestProcess_NotSupport)
#define PD_POLICY_GET_AUTO_ACCEPT_PRSWAP_AS_SOURCE(pdHandle) \
    ((pd_auto_policy_t *)(((pd_instance_t *)pdHandle)->pdPowerPortConfig->autoPolicyConfig))->autoAcceptPRSwapAsSource
#define PD_POLICY_GET_AUTO_ACCEPT_PRSWAP_AS_SINK_SUPPORT(pdHandle)                            \
    (((pd_auto_policy_t *)(((pd_instance_t *)pdHandle)->pdPowerPortConfig->autoPolicyConfig)) \
         ->autoAcceptPRSwapAsSink != kAutoRequestProcess_NotSupport)
#define PD_POLICY_GET_AUTO_ACCEPT_PRSWAP_AS_SINK(pdHandle) \
    ((pd_auto_policy_t *)(((pd_instance_t *)pdHandle)->pdPowerPortConfig->autoPolicyConfig))->autoAcceptPRSwapAsSink
#define PD_POLICY_SET_AUTO_REQUEST_PRSWAP(pdHandle, val) \
    ((pd_auto_policy_t *)(((pd_instance_t *)pdHandle)->pdPowerPortConfig->autoPolicyConfig))->autoRequestPRSwap = val
#define PD_POLICY_SET_AUTO_ACCEPT_PRSWAP_AS_SOURCE(pdHandle, val)                            \
    ((pd_auto_policy_t *)(((pd_instance_t *)pdHandle)->pdPowerPortConfig->autoPolicyConfig)) \
        ->autoAcceptPRSwapAsSource = val
#define PD_POLICY_SET_AUTO_ACCEPT_PRSWAP_AS_SINK(pdHandle, val)                                                        \
    ((pd_auto_policy_t *)(((pd_instance_t *)pdHandle)->pdPowerPortConfig->autoPolicyConfig))->autoAcceptPRSwapAsSink = \
        val

#define PD_POLICY_GET_AUTO_REQUEST_DRSWAP(pdHandle) \
    ((pd_auto_policy_t *)(((pd_instance_t *)pdHandle)->pdPowerPortConfig->autoPolicyConfig))->autoRequestDRSwap
#define PD_POLICY_GET_AUTO_ACCEPT_DRSWAP_AS_DFP_SUPPORT(pdHandle)                             \
    (((pd_auto_policy_t *)(((pd_instance_t *)pdHandle)->pdPowerPortConfig->autoPolicyConfig)) \
         ->autoAcceptDRSwapAsDFP != kAutoRequestProcess_NotSupport)
#define PD_POLICY_GET_AUTO_ACCEPT_DRSWAP_AS_DFP(pdHandle) \
    ((pd_auto_policy_t *)(((pd_instance_t *)pdHandle)->pdPowerPortConfig->autoPolicyConfig))->autoAcceptDRSwapAsDFP
#define PD_POLICY_GET_AUTO_ACCEPT_DRSWAP_AS_UFP_SUPPORT(pdHandle)                             \
    (((pd_auto_policy_t *)(((pd_instance_t *)pdHandle)->pdPowerPortConfig->autoPolicyConfig)) \
         ->autoAcceptDRSwapAsUFP != kAutoRequestProcess_NotSupport)
#define PD_POLICY_GET_AUTO_ACCEPT_DRSWAP_AS_UFP(pdHandle) \
    ((pd_auto_policy_t *)(((pd_instance_t *)pdHandle)->pdPowerPortConfig->autoPolicyConfig))->autoAcceptDRSwapAsUFP
#define PD_POLICY_SET_AUTO_REQUEST_DRSWAP(pdHandle, val) \
    ((pd_auto_policy_t *)(((pd_instance_t *)pdHandle)->pdPowerPortConfig->autoPolicyConfig))->autoRequestDRSwap = val
#define PD_POLICY_SET_AUTO_ACCEPT_DRSWAP_AS_DFP(pdHandle, val)                                                        \
    ((pd_auto_policy_t *)(((pd_instance_t *)pdHandle)->pdPowerPortConfig->autoPolicyConfig))->autoAcceptDRSwapAsDFP = \
        val
#define PD_POLICY_SET_AUTO_ACCEPT_DRSWAP_AS_UFP(pdHandle, val)                                                        \
    ((pd_auto_policy_t *)(((pd_instance_t *)pdHandle)->pdPowerPortConfig->autoPolicyConfig))->autoAcceptDRSwapAsUFP = \
        val

#define PD_POLICY_GET_AUTO_REQUEST_VCONNSWAP(pdHandle) \
    ((pd_auto_policy_t *)(((pd_instance_t *)pdHandle)->pdPowerPortConfig->autoPolicyConfig))->autoRequestVConnSwap
#define PD_POLICY_GET_AUTO_ACCEPT_VCONNSWAP_TURN_ON_SUPPORT(pdHandle)                         \
    (((pd_auto_policy_t *)(((pd_instance_t *)pdHandle)->pdPowerPortConfig->autoPolicyConfig)) \
         ->autoAcceptVconnSwapAsOn != kAutoRequestProcess_NotSupport)
#define PD_POLICY_GET_AUTO_ACCEPT_VCONNSWAP_TURN_OFF_SUPPORT(pdHandle)                        \
    (((pd_auto_policy_t *)(((pd_instance_t *)pdHandle)->pdPowerPortConfig->autoPolicyConfig)) \
         ->autoAcceptVconnSwapAsOff != kAutoRequestProcess_NotSupport)
#define PD_POLICY_GET_AUTO_ACCEPT_VCONNSWAP_TURN_ON(pdHandle) \
    ((pd_auto_policy_t *)(((pd_instance_t *)pdHandle)->pdPowerPortConfig->autoPolicyConfig))->autoAcceptVconnSwapAsOn
#define PD_POLICY_GET_AUTO_ACCEPT_VCONNSWAP_TURN_OFF(pdHandle) \
    ((pd_auto_policy_t *)(((pd_instance_t *)pdHandle)->pdPowerPortConfig->autoPolicyConfig))->autoAcceptVconnSwapAsOff
#define PD_POLICY_SET_AUTO_REQUEST_VCONNSWAP(pdHandle, val) \
    ((pd_auto_policy_t *)(((pd_instance_t *)pdHandle)->pdPowerPortConfig->autoPolicyConfig))->autoRequestVConnSwap = val
#define PD_POLICY_SET_AUTO_ACCEPT_VCONNSWAP_TURN_ON(pdHandle, val)                           \
    ((pd_auto_policy_t *)(((pd_instance_t *)pdHandle)->pdPowerPortConfig->autoPolicyConfig)) \
        ->autoAcceptVconnSwapAsOn = val
#define PD_POLICY_SET_AUTO_ACCEPT_VCONNSWAP_TURN_OFF(pdHandle, val)                          \
    ((pd_auto_policy_t *)(((pd_instance_t *)pdHandle)->pdPowerPortConfig->autoPolicyConfig)) \
        ->autoAcceptVconnSwapAsOff = val

#endif

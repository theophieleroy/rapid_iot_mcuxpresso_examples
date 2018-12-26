/*!=================================================================================================
 * Copyright (c) 2014 - 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
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
 * o Neither the name of the copyright holder nor the names of its
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


=================================================================================================*/
/*!=================================================================================================
\file       app_serial_tun.c
\brief      This is a public source file for the Serial Tunnel interface.
==================================================================================================*/

/*==================================================================================================
Include Files
==================================================================================================*/
#include "EmbeddedTypes.h"
#include "app_stack_config.h"
/* Ip */
#include "ip_if_management.h"
#include "ip.h"
#include "ip6.h"
#include "ip_if_serial_tun.h"
/* ND */
#include "nd.h"
#include "nd_events.h"
#include "nd_tables.h"
/* utils*/
#include "nvm_adapter.h"
#include "serial_tun_driver.h"
#include "thread_utils.h"
#include "app_serial_tun.h"
#include "thread_meshcop.h"
/* DHCPv6 */
#include "dhcp6_client.h"

#if THR_SERIAL_TUN_ROUTER
/*==================================================================================================
Private macros
==================================================================================================*/
/* ND ROUTER CONFIGURATION */
#define ND6_ROUTER_CONFIGURATION  \
        /* Supported features */ \
        .devType                                = gNd6DevRouter_c, \
        .messages                               = ND_PIB_MSG_RA_RCV_MASK \
                                                 | ND_PIB_MSG_RS_RCV_MASK, \
        .options                                = 0U, \
        .behaviors                              = ND_PIB_SEND_PERIODIC_RA_MASK, \
        /* Variables Defaults */ \
        .variablesDefaults.hopLimit             = 255U, \
        .variablesDefaults.linkMTU              = 1280U, \
        .variablesDefaults.regLifetime          = 0U, \
        /* Constants */ \
        .constants.maxInitialRtrAdvInterval     = 16U,      /*!< sec */ \
        .constants.maxInitialRtrAdv             = 3U, \
        .constants.maxFinalRtrAdv               = 3U, \
        .constants.minDelayBetweenRas           = 3U,       /*!< sec */ \
        .constants.maxRaDelayTime               = 500U,     /*!< ms */ \
        .constants.maxRtrSolicitationDelay      = 1U,       /*!< sec */ \
        .constants.rtrSolicitationInterval      = 4U,       /*!< sec */ \
        .constants.maxRtrSolicitations          = 3U, \
        .constants.maxMulticastSolicit          = 3U, \
        .constants.maxUnicastSolicit            = 3U, \
        .constants.maxAnycastDelayTime          = 1U,       /*!< sec */ \
        .constants.maxNeighborAdvertisement     = 3U, \
        .constants.reachableTime                = 30U,      /*!< sec */ \
        .constants.retransTime                  = 1U,       /*!< sec */ \
        .constants.delayFirstProbeTime          = 5U,       /*!< sec */ \
        .constants.minRandomFactor              = 500U,     /*!< ms */ \
        .constants.maxRandomFactor              = 1500U,    /*!< ms */ \
        .constants.minContextChangeDelay        = 0U,       /*!< sec */ \
        .constants.tentativeNceLifetime         = 0U,       /*!< sec */ \
        .constants.multihopHopLimit             = 0U, \
        .constants.maxRtrSolicitationInterval   = 0U        /*!< sec */

/* ND HOST CONFIGURATION */
#define ND6_HOST_CONFIGURATION  \
        /* Supported features */ \
        .devType                                = gNd6DevHost_c, \
        .messages                               = ND_PIB_MSG_RA_RCV_MASK, \
        .options                                = 0U, \
        .behaviors                              = 0U, \
        /* Variables Defaults */ \
        .variablesDefaults.hopLimit             = 255U, \
        .variablesDefaults.linkMTU              = 1280U, \
        .variablesDefaults.regLifetime          = 0U, \
        /* Constants */ \
        .constants.maxInitialRtrAdvInterval     = 16U,      /*!< sec */ \
        .constants.maxInitialRtrAdv             = 3U, \
        .constants.maxFinalRtrAdv               = 3U, \
        .constants.minDelayBetweenRas           = 3U,       /*!< sec */ \
        .constants.maxRaDelayTime               = 500U,     /*!< ms */ \
        .constants.maxRtrSolicitationDelay      = 1U,       /*!< sec */ \
        .constants.rtrSolicitationInterval      = 4U,       /*!< sec */ \
        .constants.maxRtrSolicitations          = 3U, \
        .constants.maxMulticastSolicit          = 3U, \
        .constants.maxUnicastSolicit            = 3U, \
        .constants.maxAnycastDelayTime          = 1U,       /*!< sec */ \
        .constants.maxNeighborAdvertisement     = 3U, \
        .constants.reachableTime                = 30U,      /*!< sec */ \
        .constants.retransTime                  = 1U,       /*!< sec */ \
        .constants.delayFirstProbeTime          = 5U,       /*!< sec */ \
        .constants.minRandomFactor              = 500U,     /*!< ms */ \
        .constants.maxRandomFactor              = 1500U,    /*!< ms */ \
        .constants.minContextChangeDelay        = 0U,       /*!< sec */ \
        .constants.tentativeNceLifetime         = 0U,       /*!< sec */ \
        .constants.multihopHopLimit             = 0U, \
        .constants.maxRtrSolicitationInterval   = 0U        /*!< sec */

/* Serialtun interface config  */
#define SERIAL_TUN_ROUTER_CONFIGURATION \
        .ifUniqueId                             = gIpIfSerialTun_c, \
        /* Serialtun */ \
        .macAddress.eui                         = {0x00, 0x60, 0x37, 0x00, 0xFA, 0x5C}, \
        .macAddress.addrSize                    = gLlayerAddrEui48_c, \
        /* ND */ \
        .pNdPib                                 = (ndPib_t*)&mSerialtunRouterNdPibCfg, \


/*==================================================================================================
Private type definitions
==================================================================================================*/

/*==================================================================================================
Private prototypes
==================================================================================================*/
#if THR_SERIAL_TAP_ROUTER
    static void SerialTun_SetAddr(ipIfUniqueId_t ipIfUniqueId, ipAddr_t *pIpAddr, void *pData);
#endif

/*==================================================================================================
Private global variables declarations
==================================================================================================*/
#if THR_SERIAL_TUN_ENABLE_ND_ROUTER
static CONST ndPib_t mSerialtunRouterNdPibCfg = {ND6_ROUTER_CONFIGURATION};
#else
static CONST ndPib_t mSerialtunRouterNdPibCfg = {ND6_HOST_CONFIGURATION};
#endif

static const externalIfConfig_t mSerialtunCfg = {SERIAL_TUN_ROUTER_CONFIGURATION};

static ifHandle_t mIfHandleSerialtun = NULL;

#if THR_SERIAL_TAP_ROUTER
    static uint32_t mThrInstanceId = 0;
    static dhcp6ClientStartParams_t mDhcp6ClientStartParams;
#endif

/*==================================================================================================
Public global variables declarations
==================================================================================================*/
taskMsgQueue_t *gpSerialTunTaskMsgQueue = NULL;

/*==================================================================================================
Public functions
==================================================================================================*/
/*!*************************************************************************************************
\fn     void Serialtun_Start(instanceId_t thrInstanceId, taskMsgQueue_t* pTaskMsgQueue)
\brief  This is a public function used to start the Serialtun Interface.
\return void
***************************************************************************************************/
void Serialtun_Start
(
    taskMsgQueue_t *pTaskMsgQueue,
    instanceId_t thrInstanceID
)
{
    thrPrefixAttr_t externalIfPrefix;
    uint32_t error;

    if (NULL == gpSerialTunTaskMsgQueue)
    {
        gpSerialTunTaskMsgQueue = pTaskMsgQueue;

        /* Initialize Serialtun interface */
        VIRTUAL_TUN_initialize((uint8_t *)mSerialtunCfg.macAddress.eui);

        error = IP_IF_Add(gIpIfSerialTun_c, NULL, (mediaIfStruct_t *)gVirtualTunMediaIfPtr, gIpProtv6_c);
        mIfHandleSerialtun = IP_IF_GetIfHandle(gIpIfSerialTun_c);

        if (mSerialtunCfg.pNdPib && (NULL != mIfHandleSerialtun) && (gIpOk_c == error))
        {
            /* assign routing function to interface */
            mIfHandleSerialtun->ip6If.ip6UnicastForward = IP6_Forward;
            mIfHandleSerialtun->ip6If.ip6McastForward = IP6_MulticastForward;

#if !THR_SERIAL_TUN_ENABLE_ND_ROUTER
            {
                ipAddr_t ipAddr = THR_SERIAL_TUN_DEFAULT_ADDRESS;

                /* configure IP address based on global pPrefix */
                IP_IF_BindAddr6(gIpIfSerialTun_c, &ipAddr,
                                ip6AddrTypeManual_c, IP6_ADDRESS_LIFETIME_INFINITE, 64);

                /* Set the external prefix */
                FLib_MemSet(&externalIfPrefix, 0, sizeof(externalIfPrefix));
                externalIfPrefix.prefixLenBits = 64;
                FLib_MemCpy(&externalIfPrefix.prefix.addr8[0], ipAddr.addr8, 8);
                THR_SetAttr(thrInstanceID, gNwkAttrId_BrExternalIfPrefix_c, 0,
                            sizeof(thrPrefixAttr_t), &externalIfPrefix);

                /* set static route for the serial tun */
                IP6_SetStaticRoute(&externalIfPrefix.prefix, externalIfPrefix.prefixLenBits,
                                   IP6_STATIC_ROUTE_DEFAULT_METRIC, (ipAddr_t *)&in6addr_any, gIpIfSerialTun_c);

                /* linux host is a default router for serial serial tun br */
                IP6_SetStaticRoute((ipAddr_t *)&in6addr_any, 0, IP6_STATIC_ROUTE_DEFAULT_METRIC,
                                   (ipAddr_t *)&in6addr_any, gIpIfSerialTun_c);

#if THR_SERIAL_TAP_ROUTER

                /* open the ND instance */
                ND_Open(mIfHandleSerialtun, mSerialtunCfg.pNdPib);
                /* configure the external global prefix as on-link for address resolution */
                ND_PrefixListAdd(mIfHandleSerialtun, &externalIfPrefix.prefix,
                                 externalIfPrefix.prefixLenBits, IP6_ADDRESS_LIFETIME_INFINITE,
                                 IP6_ADDRESS_LIFETIME_INFINITE, TRUE, FALSE);

                /* initialize a DHCPv6-PD client on the interface */
                mDhcp6ClientStartParams.clientStartMode = gDhcp6Pd_c;
                mDhcp6ClientStartParams.ipIfId = mSerialtunCfg.ifUniqueId;
                mDhcp6ClientStartParams.deviceType = DHCP6_HW_TYPE_EUI64;
                mDhcp6ClientStartParams.relayAddr = NULL;
                mDhcp6ClientStartParams.pPrefix = NULL;
                DHCP6_Client_Init(gpSerialTunTaskMsgQueue);
                DHCP6_Client_Start(&mDhcp6ClientStartParams);
                DHCP6_Client_RegisterSetAddr(SerialTun_SetAddr);
#endif

            }
#else
            {
                thrPrefixAttr_t globalOnMeshPrefix;
                externalIfPrefix = THR_GetAttr_BrExternalIfPrefix(thrInstanceID);
                globalOnMeshPrefix = THR_GetAttr_BrGlobalOnMeshPrefix(thrInstanceID);

                /* start ND */
                ND_Open(mIfHandleSerialtun, mSerialtunCfg.pNdPib);

                /* set Router Lifetime to 0 seconds - the border router is not a default router */
                ND_DefaultRouterConfig(mIfHandleSerialtun, ND_ROUTER_LIFETIME_ZERO, gNdRoutePrefMedFlag_c);

                /* configure external global pPrefix */
                ND_PrefixListAdd(mIfHandleSerialtun, &externalIfPrefix.prefix,
                                 externalIfPrefix.prefixLenBits, IP6_ADDRESS_LIFETIME_INFINITE,
                                 IP6_ADDRESS_LIFETIME_INFINITE, 1, 1);

                /* set on-mesh route to advertise */
                ND_RouteInfoSet(globalOnMeshPrefix.prefix, globalOnMeshPrefix.prefixLenBits,
                                gNdRoutePrefMedFlag_c, ND_LIFETIME_INFINITE);

                /* configure IP address based on global pPrefix */
                IP_IF_BindAddr6(gIpIfSerialTun_c, &externalIfPrefix.prefix,
                                ip6AddrTypeAutoconfigurable_c, IP6_ADDRESS_LIFETIME_INFINITE,
                                externalIfPrefix.prefixLenBits);

                /* set static route for the enet prefix */
                //IP6_SetStaticRoute(&externalIfPrefix.prefix, externalIfPrefix.prefixLenBits,
                //         IP6_STATIC_ROUTE_DEFAULT_METRIC,(ipAddr_t*)&in6addr_any,gIpIfSerialTun_c);
            }
#endif
        }
    }
}
/*!*************************************************************************************************
\fn     Serialtun_ThreadStarted
\brief  This is a public function which handles the steps that should be done after
        the thread stack is started

\return void
***************************************************************************************************/
void Serialtun_ThreadStarted(uint32_t thrInstanceId)
{
    ipAddr_t ipAddr = THR_SERIAL_TUN_DEFAULT_ADDRESS;
    ipAddr_t mCastGroup = {0xff, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                           0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3E, 0xAD
                          };

    if (THR_GetAttr_IsDevConnected(thrInstanceId))
    {
        thrOtaBrPrefixSet_t prefixSet;
        thrPrefixAttr_t onMeshPrefix = THR_GetAttr_BrGlobalOnMeshPrefix(thrInstanceId);
        thrPrefixAttr_t externalIfPrefix = THR_GetAttr_BrExternalIfPrefix(thrInstanceId);
        bool_t  brPrefixSync = FALSE;


        /* Add the global on-mesh prefix */
        if (onMeshPrefix.prefixLenBits);

        {
            FLib_MemSet(&prefixSet, 0, sizeof(prefixSet));
            prefixSet.thrBrPrefixLength = onMeshPrefix.prefixLenBits;
            FLib_MemCpy(prefixSet.thrBrPrefixValue,
                        onMeshPrefix.prefix.addr8,
                        onMeshPrefix.prefixLenBits >> 3);
            prefixSet.thrBrPrefixFlags[1] = THR_BR_PREFIX_FLAGS_P_SLAAC_MASK;
            prefixSet.thrBrPrefixFlags[1] |= THR_BR_PREFIX_FLAGS_P_ON_MESH_MASK;

            if (THR_GetAttr_BrDefaultRoute(thrInstanceId))
            {
                prefixSet.thrBrPrefixFlags[1] |= THR_BR_PREFIX_FLAGS_P_DEFAULT_MASK;
            }

            htonal(prefixSet.thrBrPrefixLifetime, THR_ALL_FFs32);
            htonal(prefixSet.thrBrExtRouteLifetime, THR_ALL_FFs32);
            prefixSet.thrBrPrefixAdvertised = TRUE;
            prefixSet.thrBrExtRouteAdvertised = FALSE;
            THR_BrPrefixAttrAddEntry(thrInstanceId, &prefixSet);
            brPrefixSync = TRUE;
        }

        if (externalIfPrefix.prefixLenBits)
        {
            /* Add External Route prefix */
            FLib_MemSet(&prefixSet, 0, sizeof(prefixSet));

            prefixSet.thrBrPrefixLength = externalIfPrefix.prefixLenBits;
            FLib_MemCpy(prefixSet.thrBrPrefixValue, externalIfPrefix.prefix.addr8, externalIfPrefix.prefixLenBits >> 3);
            htonal(prefixSet.thrBrPrefixLifetime, THR_ALL_FFs32);
            htonal(prefixSet.thrBrExtRouteLifetime, THR_ALL_FFs32);
            prefixSet.thrBrPrefixAdvertised = FALSE;
            prefixSet.thrBrExtRouteAdvertised = TRUE;

            THR_BrPrefixAttrAddEntry(thrInstanceId, &prefixSet);
            brPrefixSync = TRUE;
        }

        /* Distribute Network Data */
        if (brPrefixSync == TRUE)
        {
            THR_BrPrefixAttrSync(thrInstanceId);
        }

        /* Initiate CoAP UDP Connection */
        MESHCOP_RegisterBrServerAddr6(thrInstanceId, gIpIfSerialTun_c, &ipAddr);

        /* Register to multicast group and add forwarding rule */
        IP6_SetStaticRoute(&mCastGroup, 128, IP6_STATIC_ROUTE_DEFAULT_METRIC,
                           (ipAddr_t *)&in6addr_any, gIpIfSlp0_c);


        IP_IF_AddMulticastGroup6(gIpIfSerialTun_c, &mCastGroup);
    }

}

#if THR_SERIAL_TAP_ROUTER
/*!*************************************************************************************************
\fn     Serialtun_RaReceived
\brief  This is a function which is called when a Prefix Information Option is present in an
        ND Router Advertisement packet.

\param  [in] pEvent     pointer to the event structure
***************************************************************************************************/
void Serialtun_RaReceived
(
    void *pEvent
)
{
    evmParams_t *pEvmParam = pEvent;
    thrPrefixAttr_t externalIfPrefix = THR_GetAttr_BrExternalIfPrefix(mThrInstanceId);
    ipPrefix_t *pPrefix = (ipPrefix_t *)pEvmParam->pBuff;

    if (pEvmParam && pEvmParam->pBuff && mIfHandleSerialtun && pPrefix->prefixLen &&
            !NWKU_CmpAddrPrefix6(pPrefix->aPrefix, externalIfPrefix.prefix.addr8, pPrefix->prefixLen))
    {
        ndCfg_t **ppNdCfg = NULL;
        thrPrefixAttr_t externalPrefix = { 0 };
        thrOtaBrPrefixSet_t prefixSet = { 0 };

        /* Store the external IF prefix */
        externalPrefix.prefixLenBits = pPrefix->prefixLen;
        FLib_MemCpy(externalPrefix.prefix.addr8, pPrefix->aPrefix, pPrefix->prefixLen >> 3);
        THR_SetAttr(mThrInstanceId, gNwkAttrId_BrExternalIfPrefix_c, 0, sizeof(thrPrefixAttr_t), &externalPrefix);
        externalIfPrefix = THR_GetAttr_BrExternalIfPrefix(mThrInstanceId);

        /* Add and Propagate External Route */
        prefixSet.thrBrPrefixLength = externalIfPrefix.prefixLenBits;
        FLib_MemCpy(prefixSet.thrBrPrefixValue, externalIfPrefix.prefix.addr8, externalIfPrefix.prefixLenBits >> 3);
        htonal(prefixSet.thrBrExtRouteLifetime, THR_ALL_FFs32);
        prefixSet.thrBrExtRouteAdvertised = TRUE;
        THR_BrPrefixAttrAddEntry(mThrInstanceId, &prefixSet);

        THR_BrPrefixAttrSync(mThrInstanceId);

        /* If an RA w/ the M flag set was received, solicit an address by DHCPv6-PD */
        ppNdCfg = ND_GetCfg(mIfHandleSerialtun);

        if (ppNdCfg && *ppNdCfg && (*ppNdCfg)->bManagedConfig)
        {
            DHCP6_Client_SolicitAddress(&mDhcp6ClientStartParams);
        }
    }

    if (pEvmParam)
    {
        if (pEvmParam->pBuff)
        {
            MEM_BufferFree(pEvmParam->pBuff);
        }

        MEM_BufferFree(pEvmParam);
    }
}

/*!*************************************************************************************************
\fn     Serialtun_RaRouteInfoReceived
\brief  This is a function which is called when a Route Information option is present in an
        ND Router Advertisement packet.

\param  [in] pEvent     pointer to the event structure
***************************************************************************************************/
void Serialtun_RaRouteInfoReceived
(
    void *pEvent
)
{
    evmParams_t *pEvmParam = pEvent;
    ndOptionRouteInfo_t *pNdRouteInfoOption = (ndOptionRouteInfo_t *)(pEvmParam->pBuff);
    ndOptionRouteInfo_t *pCurrOption = NULL;
    uint8_t iCount, noOptions = pEvmParam->buffSize / sizeof(ndOptionRouteInfo_t);
    thrOtaBrPrefixSet_t prefixSet = { 0 };

    for (iCount = 0; iCount < noOptions; iCount++)
    {
        pCurrOption = pNdRouteInfoOption + iCount;

        prefixSet.thrBrPrefixLength = pCurrOption->prefixLength;
        FLib_MemCpy(prefixSet.thrBrPrefixValue, pCurrOption->prefix.addr8, pCurrOption->prefixLength >> 3);

        /* If the received route's lifetime is zero, the route is removed from the Routing Table */
        if (ntohal(pCurrOption->lifetime) == ND_LIFETIME_ZERO)
        {
            THR_BrPrefixAttrRemoveEntry(mThrInstanceId, prefixSet.thrBrPrefixLength, prefixSet.thrBrPrefixValue);
        }
        else
        {
            FLib_MemCpy(prefixSet.thrBrExtRouteLifetime, pCurrOption->lifetime, sizeof(pCurrOption->lifetime));
            THR_BR_FLAGS_SET(prefixSet.thrBrExtRouteFlags, pCurrOption->prf >> 3,
                             THR_BR_EXT_ROUTE_FLAGS_R_PREF_MASK, THR_BR_EXT_ROUTE_FLAGS_R_PREF_OFFSET);
            prefixSet.thrBrPrefixAdvertised = FALSE;
            prefixSet.thrBrExtRouteAdvertised = TRUE;

            THR_BrPrefixAttrAddEntry(mThrInstanceId, &prefixSet);
        }

        FLib_MemSet(&prefixSet, 0, sizeof(thrOtaBrPrefixSet_t));
    }

    THR_BrPrefixAttrSync(mThrInstanceId);

    MEM_BufferFree(pEvmParam->pBuff);
    MEM_BufferFree(pEvmParam);
}

/*!*************************************************************************************************
\private
\fn     SerialTun_SetAddr
\brief  This is a private function which is called when the router allocates a prefix for us (as
        response to a DHCP Prefix Delegation request).

\param  [in] ipIfUniqueId   the unique ID of the interface
\param  [in] pIpAddr        pointer to the IP prefix assigned by the router (:: in case of none)
\param  [in] pData          pointer to a variable containing the length of the prefix (NULL in
                            case of no prefix)
***************************************************************************************************/
static void SerialTun_SetAddr
(
    ipIfUniqueId_t ipIfUniqueId,
    ipAddr_t *pIpAddr,
    void *pData
)
{
    thrOtaBrPrefixSet_t prefixSet = { 0 };

    /* Router could assign a prefix */
    if (pData && (!IP_IsAddrEqual(pIpAddr, &in6addr_any)))
    {
        uint8_t prefixLen = *(uint8_t *)pData;

        /* If we received a prefix length smaller than 64 we expand it to 64 for Thread assignment
         * as we don't need to create any new sub-networks inside Thread */
        if (prefixLen < 64)
        {
            prefixLen = 64;
        }

        /* Add Prefix TLV to Network Data, SLAAC allowed */
        prefixSet.thrBrPrefixLength = prefixLen;
        FLib_MemCpy(prefixSet.thrBrPrefixValue, pIpAddr->addr8, prefixLen >> 3);
        THR_BR_FLAGS_SET(prefixSet.thrBrPrefixFlags[1], 1,
                         THR_BR_PREFIX_FLAGS_P_SLAAC_MASK, THR_BR_PREFIX_FLAGS_P_SLAAC_OFFSET);
        htonal(prefixSet.thrBrPrefixLifetime, THR_ALL_FFs32);
        prefixSet.thrBrPrefixAdvertised = TRUE;
        THR_BrPrefixAttrAddEntry(mThrInstanceId, &prefixSet);

        THR_BrPrefixAttrSync(mThrInstanceId);
    }
}

#endif /* THR_SERIAL_TAP_ROUTER */

#endif /* THR_SERIAL_TUN_ROUTER */

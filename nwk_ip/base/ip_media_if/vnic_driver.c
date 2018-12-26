/*
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
 */
/*!=================================================================================================
\file       enet_driver.c
\brief      This is a public source file for the Ethernet driver adapter.
==================================================================================================*/

/*==================================================================================================
Include Files
==================================================================================================*/
#include "fsl_enet_driver.h"
#include "fsl_phy_driver.h"
#include "fsl_enet_hal.h"
#include "fsl_phy_driver.h"
#include "fsl_clock_manager.h"
#include "fsl_interrupt_manager.h"
#include "fsl_os_abstraction.h"
#include "pin_mux.h"
#include <string.h>
#include "embeddedtypes.h"
#include "memmanager.h"
#include "network_utils.h"
#include "FunctionLib.h"

#if FSL_RTOS_MQX
#include "bsp_config.h"
#endif

#include "enet_driver.h"

#ifndef DRIVER_ERROR_BASE
    #define DRIVER_ERROR_BASE      (0x00010000ul)
#endif
/*==================================================================================================
Private macros
==================================================================================================*/

#define ENET_DEVICE_NB 0
#define DEVICE_MAC_ADDRESS 0x00, 0x60, 0x37, 0x00, 0xFA, 0x5D

/*==================================================================================================
Private type definitions
==================================================================================================*/
typedef void* pointer;

/*! @brief Define ECB structure contains protocol type and it's related service function*/
typedef struct enetServiceStruct_tag
{
    uint16_t  protocol;
    void (* service)(void*,uint8_t *, uint32_t);
    void  *privateData;
    struct enetServiceStruct_tag *next;
}enetServiceStruct_t;

/*==================================================================================================
Private prototypes
==================================================================================================*/
uint32_t ENET_receive(void *enetHandle, enet_mac_packet_buffer_t *packetBuffer);
static uint32_t ENET_buffer_init(enet_dev_if_t * enetIfPtr, enet_buff_config_t *buffCfgPtr);
static uint32_t ENET_buffer_deinit(enet_dev_if_t * enetIfPtr);

/*==================================================================================================
Private global variables declarations
==================================================================================================*/

static const uint8_t defaultEnetAddr[] = {DEVICE_MAC_ADDRESS};

static enet_dev_if_t enetDevIf[ENET_INSTANCE_COUNT];

/*! @brief Defines the RMII/MII configuration structure*/
#define ENET_CFG_RMII_IsRxOnTxDisabled          false
#define ENET_CFG_RMII_MIILoopEnable             false
static enet_config_rmii_t enetRMIICfg =
{
    kEnetCfgRmii,                       /*!< RMII/MII mode*/
    kEnetCfgSpeed100M,                  /*!< 100M/10M Speed*/
    kEnetCfgFullDuplex,                 /*!< Full/Duplex mode*/
    ENET_CFG_RMII_IsRxOnTxDisabled,          /*!< Disable rx and tx*/
    ENET_CFG_RMII_MIILoopEnable         /*!< MII loop mode*/
};

/*! @brief Defines the basic configuration structure for the ENET device.*/

static enet_mac_config_t g_enetMacCfg[ENET_INSTANCE_COUNT] =
{
{
    kEnetMacNormalMode,                                 /*!< Mac Normal or sleep mode*/
    NULL,                                               /*!< MAC hardware address*/
    &enetRMIICfg,                                       /*!< RMII configure mode*/
    /*!< Mac control configure, it is recommended to use enet_mac_control_flag_t
        it is special control set for loop mode, sleep mode, crc forward/terminate etc*/
    kEnetRxCrcFwdEnable | kEnetTxCrcBdEnable | kEnetMacEnhancedEnable,
    NULL,                                               /*!< Receive fifo configuration, if NULL default values will be used*/
    NULL,                                               /*!< Transmit fifo configuration, if NULL default values will be used*/
    0,                                                  /*!< Receive accelerator configure, should be set when kEnetTxAccelEnable is set*/
    0,                                                  /*!< Transmit accelerator configure, should be set when kEnetRxAccelEnable is set*/
    0,                                                  /*!< Pause duration, should be set when kEnetRxFlowControlEnable is set*/
    NULL,                                               /*!< special configure for MAC to instead of default configure*/
#if FSL_FEATURE_ENET_SUPPORT_PTP
    false,                                              /*!< PTP 1588 timer configuration*/
#endif
}
};

const enet_phy_config_t g_enetPhyCfg[ENET_INSTANCE_COUNT] =
{{false, 0, false }};

#if FSL_FEATURE_ENET_SUPPORT_PTP
enet_mac_ptp_ts_data_t ptpTsRxData[ENET_PTP_RXTS_RING_LEN];
enet_mac_ptp_ts_data_t ptpTsTxData[ENET_PTP_TXTS_RING_LEN];
#endif


/*==================================================================================================
Public global variables declarations
==================================================================================================*/
//extern INT_ISR_FPTR   _int_install_isr(_mqx_int,INT_ISR_FPTR, void *);

extern void ENET_Transmit_IRQHandler(void);
extern void ENET_Receive_IRQHandler(void);

/*==================================================================================================
Private functions
==================================================================================================*/

/*!*************************************************************************************************
\fn uint32_t ENET_receive(void *enetHandle, enet_mac_packet_buffer_t *packetBuffer)
\brief  Enet receive callback function.

\param [in]  enetHandle   handle to Ethernet driver
\param [in]  packetBuffer structure containing pointer to data and lenght

\retval      none
***************************************************************************************************/
uint32_t ENET_receive
(
    void *enetHandle,
    enet_mac_packet_buffer_t *packetBuffer
)
{
    uint32_t returnStatus = ENET_OK;
    enetServiceStruct_t* serviceStructPtr;
    //uint32_t index = 0;
    //uint32_t frameSize;
    uint16_t *typePtr;
    uint16_t type;
    enet_dev_if_t *enetDevifPtr = (enet_dev_if_t *)enetHandle;

    /* Process the received frame*/
    typePtr = &((enet_ethernet_header_t *)packetBuffer[0].data)->type;
    type = ntohs((*typePtr));
    if(type == ENETPROT_8021Q)
    {
        typePtr = &((enet_8021vlan_header_t *)packetBuffer[0].data)->type;
        type = ntohs((*typePtr));
    }
    if(type <= kEnetMaxFrameDateSize)
    {
        enet_8022_header_ptr llcPtr = (enet_8022_header_ptr)(typePtr + 2);
        type = htons(llcPtr->type);
    }


    for(serviceStructPtr = (enetServiceStruct_t*)enetDevifPtr->netIfPrivate; serviceStructPtr;
        serviceStructPtr = serviceStructPtr->next)
    {
        if(serviceStructPtr->protocol == type)
        {
#if 0
            while(0 != packetBuffer[index].length)
            {
                frameSize += packetBuffer[index].length;
                index++;
            }

            uint8_t* packet = MEM_BufferAlloc(frameSize);

            index = 0;
            frameSize = 0;

            while(0 != packetBuffer[index].length)
            {
                FLib_MemCpy(packet+frameSize,packetBuffer[index].data,packetBuffer[index].length);
                frameSize += packetBuffer[index].length;
                index++;
            }
#endif
            serviceStructPtr->service(serviceStructPtr->privateData,packetBuffer[0].data,packetBuffer[0].length);
        }
    }

    return returnStatus;
}

/*==================================================================================================
Public functions
==================================================================================================*/

#if FSL_RTOS_MQX
void ENET_TxIsr
(
    void* param
)
{
    ENET_Transmit_IRQHandler();
}

void ENET_RxIsr
(
    void* param
)
{
    ENET_Receive_IRQHandler();
}
#else
void ENET_TxIsr()
{
    ENET_DRV_TxIRQHandler(0);
}

void ENET_RxIsr()
{
    ENET_DRV_RxIRQHandler(0);
}

void ENET_TsIsr()
{
    ENET_DRV_TsIRQHandler(0);
}
#endif

/*!*************************************************************************************************
\fn uint32_t ENET_get_address(void* enetHandle, uint8_t* address)
\brief  Retrieves the Ethernet address of a device.

\param [in]   enetHandle    handle to Ethernet driver
\param [out]  address      mac address

\retval       uint32_t     ENET_OK
                           error code
***************************************************************************************************/
uint32_t ENET_get_address
(
      void* enetHandle,
      uint8_t* address
)
{
    uint32_t returnStatus = ENET_OK;
    uint32_t devNumber = 0;
    enet_dev_if_t * enetIfPtr;

    /* Check input param*/
    if (NULL != enetHandle)
    {
        enetIfPtr = (enet_dev_if_t *)enetHandle;
        devNumber = enetIfPtr->deviceNumber;
        FLib_MemCpy(address, g_enetMacCfg[devNumber].macAddr, kEnetMacAddrLen);
    }
    else
    {
        returnStatus = ENETERR_INVALID_INIT_PARAM;
    }
    return returnStatus;

}


/*!*************************************************************************************************
\fn uint32_t ENET_initialize(uint8_t* address,void* enetHandle)
\brief  Initializes the chip.

\param [in]  address       the local Ethernet address
\param [out] enetHandle    handle to Ethernet driver

\retval      uint32_t     ENET_OK
                          error code
***************************************************************************************************/
uint32_t ENET_initialize
(
      uint8_t* address,
      void** enetHandle
)
{
    enet_dev_if_t * enetIfPtr;
    uint32_t result;
    const enet_mac_config_t *macCfgPtr;
    enet_buff_config_t buffCfg;
    enet_user_config_t enetUserConfig;
    uint32_t devNumber = ENET_DEVICE_NB;
    enet_phy_speed_t physpeed;
    enet_phy_duplex_t phyduplex;
    bool linkstatus;
    uint32_t returnStatus = ENET_OK;


    /* Check the device status*/
    if (FALSE == enetDevIf[devNumber].isInitialized)
    {
       /* Initialize device*/
        enetIfPtr = (enet_dev_if_t *)(&enetDevIf[devNumber]);
        *enetHandle = enetIfPtr;
        enetIfPtr->next = NULL;

        enetIfPtr->deviceNumber = devNumber;
        enetIfPtr->multiGroupPtr = NULL;

#if ENET_RECEIVE_ALL_INTERRUPT
        enetIfPtr->enetNetifcall = ENET_receive;
#endif
         g_enetMacCfg[devNumber].macAddr = OSA_MemAllocZero(kEnetMacAddrLen);
        if(NULL != address)
        {
            FLib_MemCpy(g_enetMacCfg[devNumber].macAddr, address, kEnetMacAddrLen);
        }
        else
        {
            FLib_MemCpy(g_enetMacCfg[devNumber].macAddr, (uint8_t*)defaultEnetAddr, kEnetMacAddrLen);
        }
        if(g_enetMacCfg[devNumber].macAddr[3] == 0xFF && 
           g_enetMacCfg[devNumber].macAddr[4] == 0xFF && 
           g_enetMacCfg[devNumber].macAddr[5] == 0xFF)
        {
            g_enetMacCfg[devNumber].macAddr[3] = SIM_UIDMH;
            g_enetMacCfg[devNumber].macAddr[4] = SIM_UIDML;
            g_enetMacCfg[devNumber].macAddr[5] = SIM_UIDL;       
        }

        macCfgPtr = &g_enetMacCfg[devNumber];
        /* Create sync signal*/
        //lock_create(&enetIfPtr->enetContextSync);


        /* Create sync signal*/
        OSA_MutexCreate(&enetIfPtr->enetContextSync);

        /* Initialize ENET buffers*/
        result = ENET_buffer_init(enetIfPtr, &buffCfg);
        if(result != kStatus_ENET_Success)
    {
        return result;
    }

        /* Initialize ENET  pins and install interrupts.
        *  Before it was done in BSP Init. */

        /* ENET module */
        #ifdef CPU_MK65FN2M0VMI18
        /* Set clock source for Ethernet */
        if (0 == strcmp("TWR-K65F180M", BOARD_NAME))
        {
            CLOCK_SYS_SetEnetRmiiSrc(ENET_IDX, kClockRmiiSrcExt);
        }
        #endif
        configure_enet_pins(devNumber);

        /* Select the ptp timer outclk */
        CLOCK_SYS_SetEnetTimeStampSrc(ENET_IDX, kClockTimeSrcOsc0erClk);
#if FSL_RTOS_MQX
        if(NULL == _int_install_isr(ENET_Transmit_IRQn,(INT_ISR_FPTR)ENET_DRV_TxIRQHandler, NULL))
        {
          return MQX_ERROR;
        }
        NVIC_SetPriority (ENET_Transmit_IRQn, BSP_MACNET0_INT_TX_LEVEL);
        NVIC_EnableIRQ(ENET_Transmit_IRQn);

        if(NULL == _int_install_isr(ENET_Receive_IRQn,(INT_ISR_FPTR)ENET_DRV_RxIRQHandler, NULL))
        {
          return MQX_ERROR;
        }
        NVIC_SetPriority (ENET_Receive_IRQn, BSP_MACNET0_INT_RX_LEVEL);
        NVIC_EnableIRQ(ENET_Receive_IRQn);

#if FSL_FEATURE_ENET_SUPPORT_PTP
        if(NULL == _int_install_isr(ENET_1588_Timer_IRQn,(INT_ISR_FPTR)ENET_DRV_TsIRQHandler, NULL))
        {
          return MQX_ERROR;
        }
        NVIC_SetPriority (ENET_1588_Timer_IRQn, BSP_MACNET0_INT_RX_LEVEL);
        NVIC_EnableIRQ(ENET_1588_Timer_IRQn);
#endif
#else
        OSA_InstallIntHandler(ENET_1588_Timer_IRQn, ENET_TsIsr);
        NVIC_SetPriority(ENET_1588_Timer_IRQn, 4);
        NVIC_EnableIRQ(ENET_1588_Timer_IRQn);

        OSA_InstallIntHandler(ENET_Transmit_IRQn, ENET_TxIsr);
        NVIC_SetPriority(ENET_Transmit_IRQn, 4);
        NVIC_EnableIRQ(ENET_Transmit_IRQn);

        OSA_InstallIntHandler(ENET_Receive_IRQn, ENET_RxIsr);
        NVIC_SetPriority(ENET_Receive_IRQn, 4);
        NVIC_EnableIRQ(ENET_Receive_IRQn);
#endif
        /* Initialize ENET device*/
        enetUserConfig.macCfgPtr = macCfgPtr;
        enetUserConfig.buffCfgPtr = &buffCfg;
        result = ENET_DRV_Init(enetIfPtr, &enetUserConfig);
        if (result == kStatus_ENET_Success)
        {
            /* Initialize PHY*/
            if(g_enetPhyCfg[devNumber].isAutodiscoverEnabled)
            {
                uint32_t phyAddr;
                result = PHY_DRV_Autodiscover(devNumber, &phyAddr);
                if(result != kStatus_ENET_Success)
                return result;
                enetIfPtr->phyAddr = phyAddr;
            }
            else
            {
                enetIfPtr->phyAddr = g_enetPhyCfg[devNumber].phyAddr;
            }

            PHY_DRV_Init(devNumber, enetIfPtr->phyAddr, g_enetPhyCfg[devNumber].isLoopEnabled);
            /*get negociation results and reconfigure MAC speed and duplex according to phy*/
            result = PHY_DRV_GetLinkStatus(devNumber,enetIfPtr->phyAddr,&linkstatus);
            if(result == kStatus_ENET_Success)
            {
                if(linkstatus == true)
                {
                    result = PHY_DRV_GetLinkSpeed(devNumber,enetIfPtr->phyAddr,&physpeed);
                    if(result == kStatus_ENET_Success)
                    {
                       result = PHY_DRV_GetLinkDuplex(devNumber,enetIfPtr->phyAddr,&phyduplex);
                       if(result == kStatus_ENET_Success)
                       {
                           if (physpeed == kEnetSpeed10M)
                           {
                               macCfgPtr->rmiiCfgPtr->speed = kEnetCfgSpeed10M;
                           }
                           else
                           {
                               macCfgPtr->rmiiCfgPtr->speed = kEnetCfgSpeed100M;
                           }
                           if (phyduplex == kEnetFullDuplex)
                           {
                               macCfgPtr->rmiiCfgPtr->duplex = kEnetCfgFullDuplex;
                           }
                           else
                           {
                               macCfgPtr->rmiiCfgPtr->duplex = kEnetCfgHalfDuplex;
                           }
                           ENET_HAL_SetRMIIMode(g_enetBase[devNumber], macCfgPtr->rmiiCfgPtr);

                       }
                    }
                }
            }
            enetIfPtr->isInitialized = true;
#if !ENET_RECEIVE_ALL_INTERRUPT
            osa_status_t osaFlag;
            osaFlag = OSA_EventCreate(&enetIfPtr->enetReceiveSync, kEventAutoClear);
            if(osaFlag != kStatus_OSA_Success)
            {
                return osaFlag;
            }
#if USE_RTOS
        /* Create receive task*/
            osaFlag = OSA_TaskCreate(ENET_receive, "receive", RECV_TASK_STACK_SIZE, Enet_receive_stack, ENET_RECV_TASK_PRIO, (task_param_t)enetIfPtr, false, &Enet_receive_task_handler);
            if(osaFlag != kStatus_OSA_Success)
            {
                return osaFlag;
            }
#endif
#endif
            return ENET_OK;
        }
        else
        {
            ENET_DRV_Deinit(enetIfPtr);
            ENET_buffer_deinit(enetIfPtr);
            OSA_MutexDestroy(&enetIfPtr->enetContextSync);
#if !ENET_RECEIVE_ALL_INTERRUPT
#if USE_RTOS
            OSA_TaskDestroy(Enet_receive_task_handler);
#endif
            OSA_EventDestroy(&enetIfPtr->enetReceiveSync);
#endif
            return ENET_ERROR;
        }
    }
    else
    {
        returnStatus = ENETERR_INITIALIZED_DEVICE;
    }

    return returnStatus;
}

/*!*************************************************************************************************
\fn uint32_t ENET_open(void* enetHandle, int16_t protocol, void (*service)(uint8_t *, uint32_t),
                       void* privateData)
\brief  Registers a protocol type on an Ethernet channel.

\param [in]  enetHandle    handle to Ethernet driver
\param [in]  protocol      the protocol to open
\param [in]  service       the callback function
\param [in]  privateData   private data for service function

\retval       uint32_t     ENET_OK
                           error code
***************************************************************************************************/
uint32_t ENET_open
(
    void* enetHandle,
    uint16_t protocol,
    void (*service)(void*, uint8_t *, uint32_t),
    void* privateData

)
{
    enet_dev_if_t * enetIfPtr;
    enetServiceStruct_t* serviceStructPtr;
    enetServiceStruct_t** searchPtr;
    uint32_t returnStatus = ENET_OK;

    /* Check input parameter*/
    if (NULL != enetHandle)
    {

        enetIfPtr = (enet_dev_if_t *)enetHandle;
        //lock_wait(&enetIfPtr->enetContextSync, kSyncWaitForever);

        for(searchPtr=(enetServiceStruct_t **)(&enetIfPtr->netIfPrivate); *searchPtr; searchPtr=&(*searchPtr)->next)
        {
            if ((*searchPtr)->protocol == protocol)
            {
                //lock_release(&enetIfPtr->enetContextSync);
                returnStatus = ENETERR_OPEN_PROT;
                break;
            }
        }
        if(ENET_OK == returnStatus)
        {
            serviceStructPtr = (enetServiceStruct_t*)MEM_BufferAllocForever(sizeof(enetServiceStruct_t));
            if (NULL != serviceStructPtr)
            {
                serviceStructPtr->protocol = protocol;
                serviceStructPtr->service = service;
                serviceStructPtr->privateData = privateData;
                serviceStructPtr->next = NULL;
                *searchPtr = serviceStructPtr;
                //lock_release(&enetIfPtr->enetContextSync);
            }
            else
            {
                //lock_release(&enetIfPtr->enetContextSync);
                returnStatus = ENETERR_ALLOC_ECB;
            }
        }
    }
    else
    {
        returnStatus = ENETERR_INVALID_DEVICE;
    }

    return returnStatus;
}

/*!*************************************************************************************************
\fn uint32_t ENET_close(void* enetHandle,uint16_t protocol)
\brief  Unregisters a protocol type on an Ethernet channel.

\param [in]  enetHandle   handle to Ethernet driver
\param [in]  protocol     the protocol to close

\retval      uint32_t     ENET_OK
                          error code
***************************************************************************************************/
uint32_t ENET_close
(
    void* enetHandle,
    uint16_t protocol
)
{
    return ENET_OK;

}
/*!*************************************************************************************************
\fn uint32_t ENET_send(void* enetHandle,ipPktInfo_t* packet,uint16_t protocol, uint8_t* dest,
                       uint32_t  flags)
\brief  Sends a packet.

\param [in]  enetHandle    handle to Ethernet driver
\param [in]  inData        the packet to send
\param [in]  protocol      the protocol to send
\param [in]  dest          the destination Ethernet address
\param [in]  flags         optional flags, zero = default

\retval       uint32_t     ENET_OK
                           error code
***************************************************************************************************/
uint32_t ENET_send
(
    void* enetHandle,
    ipPktInfo_t* packet,
    uint16_t protocol,
    uint8_t*  dest,
    uint32_t  flags
)
{
    uint8_t headerLen;

    uint32_t returnStatus = ENET_OK;

    nwkBuffer_t* pNwkBuffer = packet->pNwkBuff, *pTempNwkBuff;
    enet_dev_if_t *enetIfPtr;
    enet_ethernet_header_t *packetPtr;
    uint8_t *frame;

    uint32_t devNumber = ENET_DEVICE_NB;
    const enet_mac_config_t *macCfgPtr;
    volatile enet_bd_struct_t * curBd;

    uint16_t lenTemp = 0, bdNumUsed = 0, offset, bdNumUsedTmp, frameLen;

    /*Check out*/
    if ((NULL != enetHandle) && (NULL != packet))
    {
        enetIfPtr = (enet_dev_if_t *)enetHandle;
        devNumber = enetIfPtr->deviceNumber;
        macCfgPtr = &g_enetMacCfg[devNumber];


        /* Default frame header size*/
        headerLen = sizeof(enet_ethernet_header_t);
        /* Check the frame length*/
        //size = NWKU_NwkBufferTotalSize(packet->pNwkBuff);
        frameLen = NWKU_NwkBufferTotalSize(packet->pNwkBuff) + headerLen;

        /* Calculate the number of buffer descriptors required for this frame */
        if(frameLen % enetIfPtr->bdContext.txBuffSizeAlign != 0)
        {
            bdNumUsed = frameLen / enetIfPtr->bdContext.txBuffSizeAlign + 1;
        }
        else
        {
            bdNumUsed = frameLen / enetIfPtr->bdContext.txBuffSizeAlign;
        }

        /* Get current buffer descriptor */
        curBd = enetIfPtr->bdContext.txBdCurPtr;

        /* Allocate the frame */
        frame = ENET_HAL_GetBuffDescripData(curBd);


        /* Add MAC hardware address */
        packetPtr = (enet_ethernet_header_t *)frame;
        FLib_MemCpy(packetPtr->destAddr, dest, kEnetMacAddrLen);
        FLib_MemCpy(packetPtr->sourceAddr, macCfgPtr->macAddr, kEnetMacAddrLen);
        packetPtr->type = htons(protocol);

        /* Preserve the number of buffer descriptors required for this frame, it is required later on */
        bdNumUsedTmp = bdNumUsed;

        /* Add MAC payload */
        lenTemp = 0;
        offset = 0;
        if(pNwkBuffer->size <= (enetIfPtr->bdContext.txBuffSizeAlign - headerLen))
        {
          FLib_MemCpy(frame + headerLen, pNwkBuffer->pData, pNwkBuffer->size);
          lenTemp += (headerLen + pNwkBuffer->size);
          pNwkBuffer = pNwkBuffer->next;
        }
        else
        {
          FLib_MemCpy(frame + headerLen, pNwkBuffer->pData, (enetIfPtr->bdContext.txBuffSizeAlign - headerLen));
          offset += enetIfPtr->bdContext.txBuffSizeAlign;
          bdNumUsedTmp--;
        }



        while(bdNumUsedTmp--)
        {
            for(pTempNwkBuff = pNwkBuffer; pTempNwkBuff != NULL; pTempNwkBuff = pTempNwkBuff->next)
            {
              /* Send the data from the pbuf to the interface, one pbuf at a
                  time. The size of the data in each pbuf is kept in the ->len
                  variable. */
                if(enetIfPtr->bdContext.txBuffSizeAlign - lenTemp >= pTempNwkBuff->size)
                {
                      FLib_MemCpy(frame + offset + lenTemp, pTempNwkBuff->pData, pTempNwkBuff->size);
                      lenTemp += pNwkBuffer->size;
//                    memcpy(frame + lenTemp, q->payload, q->len);
//                    lenTemp += q->len;

                }
                else
                {
                      /* potential for memory leak */
                      FLib_MemCpy(frame + offset + lenTemp, pTempNwkBuff->pData, enetIfPtr->bdContext.txBuffSizeAlign - lenTemp);
                      offset += enetIfPtr->bdContext.txBuffSizeAlign;           /* need to check for overflow */

                      if(offset == (ENET_RXBD_NUM * enetIfPtr->bdContext.txBuffSizeAlign)) /* not sure if it is correct */
                      {
                            offset = 0;
                      }

                      void * tempData = (void *)((uint32_t )pNwkBuffer->pData + enetIfPtr->bdContext.txBuffSizeAlign - lenTemp);
                      uint32_t tempSize = (uint32_t )pNwkBuffer->size - enetIfPtr->bdContext.txBuffSizeAlign + lenTemp;
                      pNwkBuffer = pTempNwkBuff;
                      pNwkBuffer->pData = tempData;
                      pNwkBuffer->size = tempSize;

                      break;

//                    memcpy(frame + lenTemp, q->payload, enetIfPtr->bdContext.txBuffSizeAlign - lenTemp);
//                    p->payload =(void *)((uint32_t )q->payload + enetIfPtr->bdContext.txBuffSizeAlign - lenTemp);
//                    p->len = q->len + lenTemp - enetIfPtr->bdContext.txBuffSizeAlign;
//                    p->tot_len = q->tot_len + lenTemp - enetIfPtr->bdContext.txBuffSizeAlign;
//                    p->next = q->next;
//                    break;
                }
            }
            lenTemp = 0;
        }

        /* Send packet to the device*/
        returnStatus = ENET_DRV_SendData(enetIfPtr, frameLen, bdNumUsed);
    }
    else
    {
        returnStatus =  ENETERR_INVALID_INIT_PARAM;
    }

    /* Free the PCB buffer*/
    NWKU_FreeIpPktInfo(&packet);

    return returnStatus;
}


/*!*************************************************************************************************
\fn uint32_t ENET_get_MTU(void* enetHandle)
\brief  Get the maximum transmission unit.

\param [in]  enetHandle   handle to Ethernet driver

\retval      uint32_t     ENET MTU
***************************************************************************************************/
uint32_t ENET_get_MTU
(
    void* enetHandle
)
{
    enet_dev_if_t * enetIfPtr;

    /* Check input parameter*/
    if (enetHandle == NULL)
    {
        return kStatus_ENET_InvalidDevice;
    }

    enetIfPtr = (enet_dev_if_t *)enetHandle;
    if (!enetIfPtr->maxFrameSize)
    {
        return kEnetMaxFrameDateSize;
    }

    if (enetIfPtr->isVlanTagEnabled)
    {
        return enetIfPtr->maxFrameSize - kEnetEthernetHeadLen - kEnetFrameFcsLen;
    }
    else
    {
       return enetIfPtr->maxFrameSize - sizeof(enet_8021vlan_header_t) - kEnetFrameFcsLen;
    }
}

/*!*************************************************************************************************
\fn uint32_t ENET_join(void* enetHandle,uint8_t* address,uint16_t protocol)
\brief  Joins a multicast group on an Ethernet channel.

\param [in]  enetHandle    handle to Ethernet driver
\param [in]  address       the multicast group
\param [in]  protocol      the protocol for the multicast group(IPv4 or IPv6)

\retval      uint32_t      ENET_OK
                           error code
***************************************************************************************************/
uint32_t ENET_join
(
      void* enetHandle,
      uint8_t* address,
      uint16_t protocol
)
{
    enet_dev_if_t * enetIfPtr = (enet_dev_if_t *)enetHandle;
    enet_multicast_group_t *enetMultiGroupPtr;
    uint32_t hash;

    /* Make sure it's a multicast group*/
    if (!(address[0] & 1U))
    {
       return kStatus_ENET_NoMulticastAddr;
    }

    OSA_MutexLock(&enetIfPtr->enetContextSync, OSA_WAIT_FOREVER);

    if (!enetIfPtr->multiGroupPtr)
    {
        enetIfPtr->multiGroupPtr = OSA_MemAlloc(sizeof(enet_multicast_group_t));
        if (enetIfPtr->multiGroupPtr == NULL)
        {
            OSA_MutexUnlock(&enetIfPtr->enetContextSync);
            return kStatus_ENET_MemoryAllocateFail;
        }
        memcpy(enetIfPtr->multiGroupPtr->groupAdddr, address, kEnetMacAddrLen);
        ENET_DRV_AddMulticastGroup(enetIfPtr->deviceNumber, address, &hash);
        enetIfPtr->multiGroupPtr->hash = hash;
        enetIfPtr->multiGroupPtr->next = NULL;
        enetIfPtr->multiGroupPtr->prv = NULL;
    }
    else
    {
        /* Check if we had add the multicast group*/
        enetMultiGroupPtr = enetIfPtr->multiGroupPtr;
        while (enetMultiGroupPtr != NULL)
        {
            if (!memcmp(enetMultiGroupPtr->groupAdddr, address, kEnetMacAddrLen))
            {
                OSA_MutexUnlock(&enetIfPtr->enetContextSync);
                return kStatus_ENET_AlreadyAddedMulticast;
            }
            if (enetMultiGroupPtr->next == NULL)
            {
                break;
            }
            enetMultiGroupPtr =  enetMultiGroupPtr->next;
        }

        /* Add this multicast group*/
        enetMultiGroupPtr->next = OSA_MemAllocZero(sizeof(enet_multicast_group_t));
        if (enetMultiGroupPtr->next == NULL)
        {
            OSA_MutexUnlock(&enetIfPtr->enetContextSync);
            return kStatus_ENET_MemoryAllocateFail;
        }
        memcpy(enetMultiGroupPtr->next->groupAdddr, address, kEnetMacAddrLen);
        ENET_DRV_AddMulticastGroup(enetIfPtr->deviceNumber, address, &hash);
        enetMultiGroupPtr->next->hash = hash;
        enetMultiGroupPtr->next->next = NULL;
        enetMultiGroupPtr->next->prv = enetMultiGroupPtr;
    }

    OSA_MutexUnlock(&enetIfPtr->enetContextSync);
    return ENET_OK;
}

/*!*************************************************************************************************
\fn uint32_t ENET_leave(void* enetHandle, uint8_t* address, uint16_t protocol)
\brief  Leaves a multicast group on an Ethernet channel.

\param [in]  enetHandle    handle to Ethernet driver
\param [in]  address       the multicast group
\param [in]  protocol      the protocol for the multicast group(IPv4 or IPv6)

\retval      uint32_t      ENET_OK
                           error code
***************************************************************************************************/
uint32_t ENET_leave
(
      void* enetHandle,
      uint8_t* address,
      uint16_t protocol
)
{

    enet_dev_if_t * enetIfPtr = (enet_dev_if_t *)enetHandle;
    enet_multicast_group_t *enetMultiGroupPtr, *enetTempPtr;

    /* Make sure it's a multicast group*/
    if (!(address[0] & 1U))
    {
       return kStatus_ENET_NoMulticastAddr;
    }

    OSA_MutexLock(&enetIfPtr->enetContextSync, OSA_WAIT_FOREVER);

    if (!enetIfPtr->multiGroupPtr)
    {
        OSA_MutexUnlock(&enetIfPtr->enetContextSync);
        return kStatus_ENET_MulticastPointerNull;
    }

    /* Check if we had add the multicast group*/
    for (enetMultiGroupPtr = enetIfPtr->multiGroupPtr; enetMultiGroupPtr != NULL;enetMultiGroupPtr = enetMultiGroupPtr->next )
    {
        if (!memcmp(enetMultiGroupPtr->groupAdddr, address, kEnetMacAddrLen))
        {
            ENET_DRV_LeaveMulticastGroup(enetIfPtr->deviceNumber, address);
            memset(enetMultiGroupPtr->groupAdddr, 0, kEnetMacAddrLen);
            enetTempPtr = enetMultiGroupPtr->prv;
            if (enetTempPtr != NULL)
            {
                enetTempPtr->next = enetMultiGroupPtr->next;
            }
            if (enetMultiGroupPtr->next != NULL)
            {
                enetMultiGroupPtr->next->prv = enetTempPtr;
            }

            /* Last entry.*/
            if(enetMultiGroupPtr == enetIfPtr->multiGroupPtr)
                enetIfPtr->multiGroupPtr = NULL;

            OSA_MemFree((void *)enetMultiGroupPtr);
            break;
        }
    }

    OSA_MutexUnlock(&enetIfPtr->enetContextSync);

    return ENET_OK;
}

uint8_t *txBdPtr[ENET_INSTANCE_COUNT], *rxBdPtr[ENET_INSTANCE_COUNT];
uint8_t *txBuffer[ENET_INSTANCE_COUNT], *rxBuffer[ENET_INSTANCE_COUNT];
uint8_t *rxExtBuffer[ENET_INSTANCE_COUNT];

/*FUNCTION****************************************************************
 *
 * Function Name: ENET_buffer_init
 * Return Value: The execution status.
 * Description: Initialize enet mac buffer.
 *
 *END*********************************************************************/
static uint32_t ENET_buffer_init(enet_dev_if_t * enetIfPtr, enet_buff_config_t *buffCfgPtr)
{
    uint32_t rxBufferSizeAlign, txBufferSizeAlign;
    uint8_t  *txBufferAlign, *rxBufferAlign;
    volatile enet_bd_struct_t *txBdPtrAlign, *rxBdPtrAlign;

    /* Check input parameter*/
    if((!enetIfPtr) || (!buffCfgPtr))
    {
        return kStatus_ENET_InvalidInput;
    }

    /* Allocate ENET receive buffer descriptors*/
    txBdPtr[enetIfPtr->deviceNumber] = (uint8_t *)OSA_MemAllocZero(ENET_TXBD_NUM * sizeof(enet_bd_struct_t) + ENET_BD_ALIGNMENT);
    if (!txBdPtr[enetIfPtr->deviceNumber])
    {
        return kStatus_ENET_MemoryAllocateFail;
    }
   txBdPtrAlign = (volatile enet_bd_struct_t *)ENET_ALIGN((uint32_t)txBdPtr[enetIfPtr->deviceNumber], ENET_BD_ALIGNMENT);

    rxBdPtr[enetIfPtr->deviceNumber] = (uint8_t *)OSA_MemAllocZero(ENET_RXBD_NUM * sizeof(enet_bd_struct_t) + ENET_BD_ALIGNMENT);
    if(!rxBdPtr[enetIfPtr->deviceNumber])
    {
         OSA_MemFree(txBdPtr[enetIfPtr->deviceNumber]);
         return kStatus_ENET_MemoryAllocateFail;
    }
    rxBdPtrAlign = (volatile enet_bd_struct_t *)ENET_ALIGN((uint32_t)rxBdPtr[enetIfPtr->deviceNumber], ENET_BD_ALIGNMENT);

    /* Allocate the transmit and receive date buffers*/
    rxBufferSizeAlign = ENET_RXBuffSizeAlign(ENET_RXBUFF_SIZE);
    rxBuffer[enetIfPtr->deviceNumber] = (uint8_t *)OSA_MemAllocZero(ENET_RXBD_NUM * rxBufferSizeAlign  + ENET_RX_BUFFER_ALIGNMENT);
    if (!rxBuffer[enetIfPtr->deviceNumber])
    {
        OSA_MemFree(txBdPtr[enetIfPtr->deviceNumber]);
        OSA_MemFree(rxBdPtr[enetIfPtr->deviceNumber]);
        return kStatus_ENET_MemoryAllocateFail;
    }
    rxBufferAlign = (uint8_t *)ENET_ALIGN((uint32_t)rxBuffer[enetIfPtr->deviceNumber], ENET_RX_BUFFER_ALIGNMENT);

    txBufferSizeAlign = ENET_TXBuffSizeAlign(ENET_TXBUFF_SIZE);
    txBuffer[enetIfPtr->deviceNumber] = (uint8_t *)OSA_MemAllocZero(ENET_TXBD_NUM * txBufferSizeAlign + ENET_TX_BUFFER_ALIGNMENT);
    if (!txBuffer[enetIfPtr->deviceNumber])
    {
        OSA_MemFree(txBdPtr[enetIfPtr->deviceNumber]);
        OSA_MemFree(rxBdPtr[enetIfPtr->deviceNumber]);
        OSA_MemFree(rxBuffer[enetIfPtr->deviceNumber]);
        return kStatus_ENET_MemoryAllocateFail;
    }
    txBufferAlign = (uint8_t *)ENET_ALIGN((uint32_t)txBuffer[enetIfPtr->deviceNumber], ENET_TX_BUFFER_ALIGNMENT);

#if FSL_FEATURE_ENET_SUPPORT_PTP
    buffCfgPtr->ptpTsRxDataPtr = &ptpTsRxData[0];
    buffCfgPtr->ptpTsRxBuffNum = ENET_PTP_RXTS_RING_LEN;
    buffCfgPtr->ptpTsTxDataPtr = &ptpTsTxData[0];
    buffCfgPtr->ptpTsTxBuffNum = ENET_PTP_TXTS_RING_LEN;
#endif
#if !ENET_RECEIVE_ALL_INTERRUPT
    uint8_t *rxExtBufferAlign;
    rxExtBuffer[enetIfPtr->deviceNumber] = (uint8_t *)OSA_MemAllocZero(ENET_EXTRXBD_NUM * rxBufferSizeAlign  + ENET_RX_BUFFER_ALIGNMENT);
    if (!rxExtBuffer[enetIfPtr->deviceNumber])
    {
        OSA_MemFree(txBdPtr[enetIfPtr->deviceNumber]);
        OSA_MemFree(rxBdPtr[enetIfPtr->deviceNumber]);
        OSA_MemFree(rxBuffer[enetIfPtr->deviceNumber]);
        OSA_MemFree(txBuffer[enetIfPtr->deviceNumber]);
        return kStatus_ENET_MemoryAllocateFail;
    }
    rxExtBufferAlign = (uint8_t *)ENET_ALIGN((uint32_t)rxExtBuffer[enetIfPtr->deviceNumber], ENET_RX_BUFFER_ALIGNMENT);
    buffCfgPtr->extRxBuffQue = rxExtBufferAlign;
    buffCfgPtr->extRxBuffNum = ENET_EXTRXBD_NUM;
#else
    buffCfgPtr->extRxBuffQue = NULL;
    buffCfgPtr->extRxBuffNum = 0;
#endif

    buffCfgPtr->rxBdNumber = ENET_RXBD_NUM;
    buffCfgPtr->txBdNumber = ENET_TXBD_NUM;

    buffCfgPtr->rxBuffSizeAlign = rxBufferSizeAlign;
    buffCfgPtr->txBuffSizeAlign = txBufferSizeAlign;

    buffCfgPtr->rxBdPtrAlign = rxBdPtrAlign;
    buffCfgPtr->txBdPtrAlign = txBdPtrAlign;

    buffCfgPtr->rxBufferAlign = rxBufferAlign;
    buffCfgPtr->txBufferAlign = txBufferAlign;

    return kStatus_ENET_Success;
}

/*FUNCTION****************************************************************
 *
 * Function Name: ENET_buffer_deinit
 * Return Value: The execution status.
 * Description: Initialize enet mac buffer.
 *
 *END*********************************************************************/
static uint32_t ENET_buffer_deinit(enet_dev_if_t * enetIfPtr)
{
    /* Check input parameter*/
    if(!enetIfPtr)
    {
        return kStatus_ENET_InvalidInput;
    }

    /* Free allocated memory*/
    if(txBdPtr[enetIfPtr->deviceNumber])
    {
        OSA_MemFree(txBdPtr[enetIfPtr->deviceNumber]);
    }
    if(rxBdPtr[enetIfPtr->deviceNumber])
    {
        OSA_MemFree(rxBdPtr[enetIfPtr->deviceNumber]);
    }
    if(txBuffer[enetIfPtr->deviceNumber])
    {
        OSA_MemFree(txBuffer[enetIfPtr->deviceNumber]);
    }
    if(rxBuffer[enetIfPtr->deviceNumber])
    {
        OSA_MemFree(rxBuffer[enetIfPtr->deviceNumber]);
    }
#if !ENET_RECEIVE_ALL_INTERRUPT
    if(rxExtBuffer[enetIfPtr->deviceNumber])
    {
        OSA_MemFree(rxExtBuffer[enetIfPtr->deviceNumber]);
    }
#endif
    return kStatus_ENET_Success;
}

/*================================================================================================*/

/*==================================================================================================
Private debug functions
==================================================================================================*/

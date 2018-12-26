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
\file       virtual_enet_driver.c
\brief      This is a private source file for the media interface between IPv6 and I2C to 
            Ethernet adapter
            
==================================================================================================*/

/*==================================================================================================
Include Files
==================================================================================================*/
#include "EmbeddedTypes.h"
#include "fsl_osa_ext.h"
#include "MemManager.h"
#include "FunctionLib.h"
#include "Led.h"

#include "network_utils.h"
#include "i2c_abstraction.h"

/*==================================================================================================
Private macros
==================================================================================================*/

#define V_ENET_GENERAL_ERROR    0xA4FFU
#define V_ENET_PROCESSING_ERROR 0xFFFFU
#define V_ENET_OK               0x00000000U
#define V_ENET_ADDR_SIZE        6U

#define V_ENET_SEND_RETRY_COUNT_DEFAULT 3U

#define ENET_HDR_SIZE   14
/*==================================================================================================
Private type definitions
==================================================================================================*/

/*==================================================================================================
Private prototypes
==================================================================================================*/

void VENET_SemWait(osaSemaphoreId_t semaphore_id);
void VENET_SemRelease(osaSemaphoreId_t semaphore_id);

/*==================================================================================================
Private global variables declarations
==================================================================================================*/
static uint32_t replyStatus;
static uint8_t enetStaticAddress[6];

osaSemaphoreId_t master_lock_ve_sem_id;

void (*mediaIfCallback)(uint8_t*, uint32_t);
/*==================================================================================================
Public global variables declarations
==================================================================================================*/

/*==================================================================================================
Private functions
==================================================================================================*/
void VENET_SemWait(osaSemaphoreId_t semaphore_id)
{
    OSA_EXT_SemaphoreWait(semaphore_id, osaWaitForever_c);
    //Led3On();
}

void VENET_SemRelease(osaSemaphoreId_t semaphore_id)
{
    OSA_EXT_SemaphorePost(semaphore_id);
    //Led3Off();
}

/*==================================================================================================
Public functions
==================================================================================================*/

/*!*************************************************************************************************
\fn uint32_t VIRTUAL_ENET_initialize(uint8_t* address)
\brief  Initializes the chip.
        
\param [in]   pAddr       the local Ethernet address
       
\retval       uint32_t     ENET_OK
                           error code 
***************************************************************************************************/
uint32_t VIRTUAL_ENET_initialize
(
      uint8_t* pAddr
)
{
    uint32_t addrSize;
    uint8_t* pAddrPtr = (uint8_t*)&addrSize;

    master_lock_ve_sem_id = OSA_EXT_SemaphoreCreate(0);

    /* Initialize I2C interface for the I2C2ETH adapter */
    I2C_Abstraction_Init();

    if(pAddr == NULL)
    {
        addrSize = 1U;
    }
    else
    {
        addrSize = 6U;
        pAddrPtr = pAddr;
    }
    /* 2. Send command to SLAVE */
    I2C_MasterSend(pAddrPtr, addrSize, OPG_MASTER_TO_SLAVE, OPC_ENET_INIT,FALSE);

    /* Wait to receive packet from SLAVE */
    VENET_SemWait(master_lock_ve_sem_id);

    /* return status received from SLAVE */
    return replyStatus;
}

/*!*************************************************************************************************
\fn uint32_t VIRTUAL_ENET_open(uint16_t protocol, void (*service)(uint8_t *, uint32_t))
\brief  Registers a protocol type on an Ethernet channel.

\param [in]  protocol      the protocol to open
\param [in]  service       the callback function
       
\retval       uint32_t     ENET_OK
                           error code 
***************************************************************************************************/
uint32_t VIRTUAL_ENET_open
(
    uint16_t protocol,
    void (*service)(uint8_t *, uint32_t)
      
)
{   
    /* save media interface callback */
    mediaIfCallback = service;
    
    /* 2. Send command to SLAVE */
    I2C_MasterSend((uint8_t*)&protocol, sizeof(protocol), OPG_MASTER_TO_SLAVE, OPC_ENET_OPEN, FALSE);

    /* Wait to receive packet from SLAVE */
    VENET_SemWait(master_lock_ve_sem_id);

    /* return status received from SLAVE */
    return replyStatus;
}

/*!*************************************************************************************************
\fn uint32_t VIRTUAL_ENET_close(uint16_t protocol)
\brief  Unregisters a protocol type on an Ethernet channel.

\param [in]  protocol      the protocol to close
      
\retval       uint32_t     ENET_OK
                           error code 
***************************************************************************************************/
uint32_t VIRTUAL_ENET_close
(
    uint16_t protocol
)
{
    /* 2. Send command to SLAVE */
    I2C_MasterSend((uint8_t*)&protocol, sizeof(protocol), OPG_MASTER_TO_SLAVE, OPC_ENET_CLOSE, FALSE);

    /* Wait to receive packet from SLAVE */
    VENET_SemWait(master_lock_ve_sem_id);

    /* return status received from SLAVE */
    return replyStatus;
}

/*!*************************************************************************************************
\fn void VIRTUAL_ENET_receive(uint8_t* inData,  uint32_t inDataLen)
\brief  Enet receive callback function.

\param [in]  inData       received data
\param [in]  inDataLen    received data lenght
      
\retval      none
***************************************************************************************************/
void VIRTUAL_ENET_receive
(
    uint8_t* inData,
    uint32_t inDataLen
)
{
    /* call the actual media interface handle function */
    mediaIfCallback(inData, inDataLen);
}
/*!*************************************************************************************************
\fn uint32_t VIRTUAL_ENET_send(ipPktInfo_t* packet,uint16_t protocol, uint8_t* dest, uint32_t  flags)
\brief  Sends a packet.

\param [in]  inData        the packet to send
\param [in]  protocol      the protocol to send
\param [in]  dest          the destination Ethernet address
\param [in]  flags         optional flags, zero = default
      
\retval       uint32_t     ENET_OK
                           error code 
***************************************************************************************************/
uint32_t VIRTUAL_ENET_send
(
      ipPktInfo_t* packet,
      uint16_t protocol,
      uint8_t*  dest,
      uint32_t  flags
)
{
    uint32_t     buffer_size = 0;
    nwkBuffer_t* pNwkBuffer;
    uint8_t      *buffer = NULL;
    uint32_t     index = 0;
    uint8_t      *current_pos;
    uint32_t     buffer_index = 0;    

    /* 1. Create payload buffer */
    buffer_size =
            V_ENET_ADDR_SIZE +               /* address is 6 bytes long */
            sizeof(protocol) +               /* 2 bytes */
            sizeof(flags) +                  /* 4 bytes */
            sizeof(index);                   /* 4 bytes */

    buffer_size += NWKU_NwkBufferTotalSize(packet->pNwkBuff);
    buffer_size += ENET_HDR_SIZE;
    buffer_index = NWKU_NwkBufferNumber(packet->pNwkBuff);
    
    /* Add the fragment for ENET header to the number of existing network buffers */
    index = buffer_index;

    buffer_size += sizeof(uint32_t)*index;
    
    buffer = MEM_BufferAlloc(buffer_size);

    if(buffer == NULL)
    {
        return V_ENET_GENERAL_ERROR;
    }
    current_pos = buffer;

    /* copy address */
    FLib_MemCpy(current_pos, dest, V_ENET_ADDR_SIZE);
    current_pos += V_ENET_ADDR_SIZE;

    /* copy protocol */
    FLib_MemCpy(current_pos, &protocol, sizeof(protocol));
    current_pos += sizeof(protocol);

    /* copy flags */
    FLib_MemCpy(current_pos, &flags, sizeof(flags));
    current_pos += sizeof(flags);

    /* put number of fragments */
    FLib_MemCpy(current_pos, &index, sizeof(index));
    current_pos += sizeof(index);

    pNwkBuffer = packet->pNwkBuff;

    uint32_t tempSize = pNwkBuffer->size + ENET_HDR_SIZE;
    /* copy buffer size */
    FLib_MemCpy(current_pos, &tempSize, sizeof(uint32_t));
    current_pos += sizeof(uint32_t);

    FLib_MemSet(current_pos,0,ENET_HDR_SIZE);

    /* copy buffer */
    FLib_MemCpy(current_pos+ENET_HDR_SIZE, pNwkBuffer->pData, pNwkBuffer->size);
    current_pos += pNwkBuffer->size+ENET_HDR_SIZE;

    pNwkBuffer = pNwkBuffer->next;
    buffer_index--;
    
    while(buffer_index>0)
    {   
        
        /* copy buffer size */
        FLib_MemCpy(current_pos, &pNwkBuffer->size, sizeof(uint32_t));
        current_pos += sizeof(uint32_t);

        /* copy buffer */
        FLib_MemCpy(current_pos, pNwkBuffer->pData, pNwkBuffer->size);
        current_pos += pNwkBuffer->size;

        pNwkBuffer = pNwkBuffer->next;
        buffer_index--;
    }

    NWKU_FreeIpPktInfo(&packet);

    /* 2. Send command to SLAVE */
    I2C_MasterSend(buffer, buffer_size, OPG_MASTER_TO_SLAVE, OPC_ENET_SEND, TRUE);       

    return V_ENET_OK;
}  

/*!*************************************************************************************************
\fn uint32_t VIRTUAL_ENET_get_address(uint8_t* address)
\brief  Retrieves the Ethernet address of a device.
        
\param [out]  address      mac address
       
\retval       uint32_t     ENET_OK
                           error code 
***************************************************************************************************/
uint32_t VIRTUAL_ENET_get_address
(
      uint8_t*        address
)
{
    uint8_t     tmp = 0x05;
    
    /* 2. Send command to SLAVE */
    I2C_MasterSend(&tmp, sizeof(tmp), OPG_MASTER_TO_SLAVE, OPC_GET_ADDRESS,FALSE);

    /* Wait to receive packet from SLAVE */
    VENET_SemWait(master_lock_ve_sem_id);

    FLib_MemCpy(address, enetStaticAddress, 6U);
    
    /* return status received from SLAVE */
    return 0;
} 

/*!*************************************************************************************************
\fn uint32_t VIRTUAL_ENET_get_MTU()
\brief  Get the maximum transmission unit.
      
\retval       uint32_t     ENET MTU
***************************************************************************************************/
uint32_t VIRTUAL_ENET_get_MTU
(
)
{
    uint8_t tmp_byte = 0x05;

    /* 2. Send command to SLAVE */
    I2C_MasterSend(&tmp_byte, sizeof(tmp_byte), OPG_MASTER_TO_SLAVE, OPC_ENET_GET_MTU, FALSE);

    /* Wait to receive packet from SLAVE */
    VENET_SemWait(master_lock_ve_sem_id);

    /* return status received from SLAVE */
    return replyStatus;
}

/*!*************************************************************************************************
\fn uint32_t VIRTUAL_ENET_join(uint8_t* address,uint16_t protocol)
\brief  Joins a multicast group on an Ethernet channel.

\param [in]  address       the multicast group
\param [in]  protocol      the protocol for the multicast group(IPv4 or IPv6)

\retval      uint32_t      ENET_OK
                           error code 
***************************************************************************************************/
uint32_t VIRTUAL_ENET_join
(
      uint8_t* address,
      uint16_t protocol      
)
{
    uint8_t* tempBuffer = MEM_BufferAlloc(V_ENET_ADDR_SIZE + sizeof(uint16_t));

    /* copy address*/
    FLib_MemCpy(tempBuffer,address,V_ENET_ADDR_SIZE);

    /* copy protocol */
    FLib_MemCpy(tempBuffer+V_ENET_ADDR_SIZE,&protocol,sizeof(uint16_t));

    /* 2. Send command to SLAVE */
    I2C_MasterSend(tempBuffer, V_ENET_ADDR_SIZE + sizeof(uint16_t), OPG_MASTER_TO_SLAVE, 
                   OPC_ENET_JOIN, TRUE);

    /* Wait to receive packet from SLAVE */
    VENET_SemWait(master_lock_ve_sem_id);

    /* return status received from SLAVE */
    return replyStatus;
}

/*!*************************************************************************************************
\fn uint32_t VIRTUAL_ENET_leave(uint8_t* address, uint16_t protocol)
\brief  Leaves a multicast group on an Ethernet channel.

\param [in]  address       the multicast group
\param [in]  protocol      the protocol for the multicast group(IPv4 or IPv6)

\retval      uint32_t      ENET_OK
                           error code 
***************************************************************************************************/
uint32_t VIRTUAL_ENET_leave
(
      uint8_t*     address,
      uint16_t protocol
)
{
    uint8_t* tempBuffer = MEM_BufferAlloc(V_ENET_ADDR_SIZE + sizeof(uint16_t));

    /* copy address*/
    FLib_MemCpy(tempBuffer,address,V_ENET_ADDR_SIZE);

    /* copy protocol */
    FLib_MemCpy(tempBuffer+V_ENET_ADDR_SIZE,&protocol,sizeof(uint16_t));

    /* 2. Send command to SLAVE */
    I2C_MasterSend(tempBuffer, V_ENET_ADDR_SIZE+sizeof(uint16_t), OPG_MASTER_TO_SLAVE, 
                   OPC_ENET_LEAVE,TRUE);

    /* Wait to receive packet from SLAVE */
    VENET_SemWait(master_lock_ve_sem_id);

    /* return status received from SLAVE */
    return replyStatus;
}
/*!*************************************************************************************************
\fn void VIRTUAL_ENET_reset()
\brief  Resets the I2C to ETH adapter

\retval      none              
***************************************************************************************************/
void VIRTUAL_ENET_reset
(
)
{
    uint8_t tmp_byte = 0x05;

    /* 2. Send command to SLAVE */
    I2C_MasterSend(&tmp_byte, sizeof(tmp_byte), OPG_MASTER_TO_SLAVE, OPC_ENET_RESET,FALSE);

    OSA_EXT_TimeDelay(10);
}

/*!*************************************************************************************************
\fn void VIRTUAL_ENET_process(i2cHdr_t *pHeader,uint8_t *pBuffer)
\brief  Handles the return value from a virtual enet function call.

\param [in]  pHeader       the packet header
\param [in]  pBuffer       data pointer
      
\retval      none
***************************************************************************************************/
void VIRTUAL_ENET_process
(
    i2cHdr_t *pHeader,
    uint8_t *pBuffer
)
{
    replyStatus = V_ENET_PROCESSING_ERROR;

    if(pHeader->OPCODE == OPC_GET_ADDRESS)
    {
        FLib_MemCpy(enetStaticAddress, pBuffer, 6);
    }
    else
    {
        FLib_MemCpy(&replyStatus, pBuffer, sizeof(replyStatus));
    }

    VENET_SemRelease(master_lock_ve_sem_id);
}

/*================================================================================================*/

/*==================================================================================================
Private debug functions
==================================================================================================*/

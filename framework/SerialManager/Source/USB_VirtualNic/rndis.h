/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * All rights reserved.
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
#ifndef _RNDIS_H_
#define _RNDIS_H_  1

/*! *********************************************************************************
*************************************************************************************
* Public macros
*************************************************************************************
********************************************************************************** */
#ifndef gRNDIS_MemPoolId_c
#define gRNDIS_MemPoolId_c  1
#endif
/*! *********************************************************************************
*************************************************************************************
* Private macros
*************************************************************************************
********************************************************************************** */

/*! *********************************************************************************
*************************************************************************************
* Public type definitions
*************************************************************************************
********************************************************************************** */
typedef enum{
   gRndis_Success_c               = 0,
   gRndis_InvalidParameter_c      = 1,
   gRndis_OutOfMemory_c           = 2,
   gRndis_InitError               = 3,
   gRndis_UsbError                = 4,
   gRndis_Timeout                 = 5
   }rndisStatus_t;

typedef enum
{
  rndisMCastMacAddType_IPv4      = 0,
  rndisMCastMacAddType_IPv6      = 1
}rndisMCastMacAddType_t;

typedef void(*pfRNDISRxCallback_t)(void* pPacket, uint16_t length);

/*! *********************************************************************************
*************************************************************************************
* Public functions
*************************************************************************************
********************************************************************************** */
rndisStatus_t RNDIS_Init();
rndisStatus_t RNDIS_Send(uint8_t* pBuff, uint16_t length);
rndisStatus_t RNDIS_SetDestMacAddress(uint8_t* pBuff);
rndisStatus_t RNDIS_SetRndisMacAddress(uint8_t* pBuff);
rndisStatus_t RNDIS_RegisterMulticastMacAddress(rndisMCastMacAddType_t addType, bool_t write, uint8_t* pBuff);
rndisStatus_t RNDIS_EnableIPv6(bool_t en);
rndisStatus_t RNDIS_EnableIPv4(bool_t en);
rndisStatus_t RNDIS_RegisterRxCallback(pfRNDISRxCallback_t pCallback);

#endif /* _RNDIS_H_ */

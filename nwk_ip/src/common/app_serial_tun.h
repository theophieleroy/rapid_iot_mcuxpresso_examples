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

/*
\file       app_serial_tun.h
\brief      This is a header file for the Serialtun interface configuration module.
 */

#ifndef _APP_SERIAL_TUN_H
#define _APP_SERIAL_TUN_H



/*==================================================================================================
Include Files
==================================================================================================*/
#include "EmbeddedTypes.h"
#include "network_utils.h"

#if THR_SERIAL_TUN_ROUTER
/*==================================================================================================
Public macros
==================================================================================================*/
/* Enable/disable ND router behavior */
#define THR_SERIAL_TUN_ENABLE_ND_ROUTER    FALSE

/* On the serial TUN, if the ND is disable, an IPv6 static  address will be register
 * for the external interface  */
#define THR_SERIAL_TUN_DEFAULT_ADDRESS {0xFD, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
                                        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01}

#define THR_SERIAL_TAP_ROUTER   FALSE
/*==================================================================================================
Public type definitions
==================================================================================================*/

/*==================================================================================================
Public global variables declarations
==================================================================================================*/
extern taskMsgQueue_t *gpSerialTunTaskMsgQueue;

/*==================================================================================================
Public function prototypes
==================================================================================================*/
/*!*************************************************************************************************
\fn      Serialtun_Start
\brief  This is a public function used to start the Serialtun Interface.
\return void
***************************************************************************************************/
void Serialtun_Start(taskMsgQueue_t *pTaskMsgQueue, instanceId_t thrInstanceID);

/*!*************************************************************************************************
\fn     Serialtun_ThreadStarted
\brief  This is a public function which handles the steps that should be done after
        the thread stack is started

\return void
***************************************************************************************************/
void Serialtun_ThreadStarted(uint32_t thrInstanceId);

/*!*************************************************************************************************
\fn     Serialtun_RaReceived
\brief  This is a function which is called when a Prefix Information Option is present in an
        ND Router Advertisement packet.

\param  [in] pEvent     pointer to the event structure
***************************************************************************************************/
void Serialtun_RaReceived(void *pEvent);

/*!*************************************************************************************************
\fn     Serialtun_RaRouteInfoReceived
\brief  This is a function which is called when a Route Information option is present in an
        ND Router Advertisement packet.

\param  [in] pEvent     pointer to the event structure
***************************************************************************************************/
void Serialtun_RaRouteInfoReceived(void *pEvent);

#endif /* THR_SERIAL_TUN_ROUTER */

#endif /* _APP_SERIAL_TUN_H */

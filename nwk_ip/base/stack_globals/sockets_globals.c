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

/*!
\file       sockets_globals.c
\brief      This is the source file that contains parameters for the SOCKETS module that can
            be configured by the application 
*/

/*==================================================================================================
Include Files
==================================================================================================*/
#include "EmbeddedTypes.h"
#include "fsl_os_abstraction.h"
#include "app_thread_config.h"
#include "ip_sockets.h"
#include "sockets_cfg.h"
#include "session.h"

#if !THR_USEONETASK_ENABLE
extern void Session_Task(osaTaskParam_t argument);
#endif
/*==================================================================================================
Private macros
==================================================================================================*/


/*==================================================================================================
Private type definitions
==================================================================================================*/

/*==================================================================================================
Private prototypes
==================================================================================================*/

/*==================================================================================================
Private global variables declarations
==================================================================================================*/

/*==================================================================================================
Public global variables declarations
==================================================================================================*/

/*! The list of used sockets for this device */
const uint32_t  gSocketsSize = BSDS_MAX_SOCKETS;
sock_t          gaStaticUdpSockets[BSDS_MAX_SOCKETS] = {0};
sessEnt_t       gaSockEntList[BSDS_MAX_SOCKETS];
const uint16_t  gSocketsSelectMaxFds = BSDS_SELECT_MAX_FDS;

#if !THR_USEONETASK_ENABLE
/*! Create a Thread Definition for Session_Task */ 
OSA_TASK_DEFINE(Session_Task, OSA_PRIORITY_LOW, 1, SESSION_TASK_STACK_SIZE, 0);
#endif

/*==================================================================================================
Private functions
==================================================================================================*/

/*==================================================================================================
Public functions
==================================================================================================*/


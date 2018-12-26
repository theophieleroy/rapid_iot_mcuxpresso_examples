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

#ifndef _SERIAL_TUN_DRIVER_H
#define _SERIAL_TUN_DRIVER_H
/*!=================================================================================================
\file       serial_tun_driver.h
\brief      This is a header file for the Serial TUN media interface (UDP over THCI over UART).
==================================================================================================*/

/*==================================================================================================
Include Files
==================================================================================================*/
/* General Includes */
#include "EmbeddedTypes.h"
#include "stack_config.h"


/*==================================================================================================
Public macros
==================================================================================================*/


/*==================================================================================================
Public type definitions
==================================================================================================*/


/*==================================================================================================
Public global variables declarations
==================================================================================================*/


/*==================================================================================================
Public function prototypes
==================================================================================================*/
#ifdef __cplusplus
extern "C" {
#endif

/*!*************************************************************************************************
\fn uint32_t VIRTUAL_TUN_initialize(uint8_t* address)
\brief  Initializes the chip.
        
\param [in]  address       the local Ethernet address
       
\retval       uint32_t     ENET_OK
                           error code 
***************************************************************************************************/
uint32_t VIRTUAL_TUN_initialize(uint8_t* address);

/*!*************************************************************************************************
\fn uint32_t VIRTUAL_TUN_get_address(uint8_t* address)
\brief  Retrieves the TUN address of a device.

\param [out]  address      mac address

\retval       uint32_t     ENET_OK
                           error code
***************************************************************************************************/
uint32_t VIRTUAL_TUN_get_address(uint8_t* address);

#ifdef __cplusplus
}
#endif
/*================================================================================================*/
#endif  /*_VIRTUAL_ENET_DRIVER_H */

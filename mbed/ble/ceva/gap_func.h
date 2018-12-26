/*
 * The Clear BSD License
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided
 *  that the following conditions are met:
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

#ifndef _GAP_FUNC_H_
#define _GAP_FUNC_H_

#include "app_config.h"
#include "app_ble.h"
#include "app.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
// #define TEST_GAP_FUNC
/*${macro:start}*/
#define   BLE_ADDRESS 0x01
#define   BLE_ADDRESS_MASK 0xFE
#define FB_DEV_NAME_SET   0x02
#define FB_DEV_NAME_SET_MASK 0xFD
#define FB_DEV_UPD_PARAM   0x04
#define FB_DEV_UPD_PARAM_MASK 0xFB

#define  INVOKED_ONCE  0x01

#define FAIL 0x01

/*${macro:end}*/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
/*${prototype:start}*/

extern void *data_ptr;
extern struct gap_bdaddr ble_address;
extern volatile unsigned char func_flag;
extern volatile unsigned char func_flag1;
extern volatile unsigned char func_status;
extern uint32_t getBleGapAddress(struct gap_bdaddr *ptr_addr);
extern uint32_t updateBleConnectionParams(uint16_t conhdl, struct gapc_conn_param *params);
extern uint32_t setBleDeviceName(uint8_t *ptr_dev_name,uint16_t len);
extern uint32_t getBleDeviceName(uint8_t *dev_name,uint16_t *dev_name_len);

/*${prototype:end}*/

#endif /* _GAP_FUNC_H_ */

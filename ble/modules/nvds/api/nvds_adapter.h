/*
 * The Clear BSD License
 * Copyright (c) 2016, NXP Semiconductors, N.V.
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

#ifndef _NVDS_ADAPTER_H_
#define _NVDS_ADAPTER_H_

#include "fsl_common.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*!
 * @brief   One entry in the main timer table.
 */
typedef struct _nvds_data_st_t
{
    uint8_t magic_number[4]; /*!< Magic number indicate NVDS space valid */
    uint8_t bd_addr[6];      /*!< Bluetooth device address */
    uint8_t dev_name[8];     /*!< Device name */
    uint16_t ppm;            /*!< 32K clock drift */
    uint16_t wakeup_time;    /*!< Time for MCU exit PD, units: us */
    uint8_t xtal_loadcap;    /*!< 16/32MHz crystal load cap, refer to register description */
    uint8_t xtal32k_loadcap; /*!< 32KHz crystal load cap, refer to register description */
} nvds_data_st_t;

#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus */

/*******************************************************************************
 * API
 ******************************************************************************/

/*!
 * @brief   initialize the NVDS adapter
 */
uint8_t nvds_adapter_init(void);

#if defined(__cplusplus)
}
#endif /* __cplusplus */

#endif /* _NVDS_ADAPTER_H_ */

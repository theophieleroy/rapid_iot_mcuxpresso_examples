/*
 * The Clear BSD License
 * Copyright (c) 2017-2018, NXP Semiconductors, Inc.
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

#ifndef _FSL_ADV7535_H_
#define _FSL_ADV7535_H_

#include "fsl_common.h"
#include "fsl_video_i2c.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*!
 * @brief ADV7535 resource.
 *
 * The I2C instance should be initialized before calling @ref ADV7535_Init.
 */
typedef struct _adv7535_resource
{
    video_i2c_send_func_t i2cSendFunc;       /* I2C send function. */
    video_i2c_receive_func_t i2cReceiveFunc; /* I2C receive function. */
    uint8_t i2cAddr;                         /* I2C address for the main memory. */
} adv7535_resource_t;

extern const display_operations_t adv7535_ops;

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

status_t ADV7535_Init(display_handle_t *handle, const display_config_t *config);

status_t ADV7535_Deinit(display_handle_t *handle);

status_t ADV7535_Start(display_handle_t *handle);

status_t ADV7535_Stop(display_handle_t *handle);

#if defined(__cplusplus)
}
#endif

#endif /* _FSL_ADV7535_H_ */

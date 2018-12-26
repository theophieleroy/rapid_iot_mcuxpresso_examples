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

#ifndef _FSL_OV5640_H_
#define _FSL_OV5640_H_

#include "fsl_common.h"
#include "fsl_sccb.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*!
 * @brief OV5640 resource.
 *
 * Before initialize the OV5640, the resource must be initialized that the SCCB
 * I2C could start to work.
 */
typedef struct _ov5640_resource
{
    sccb_i2c_send_func_t i2cSendFunc;       /*!< I2C send function. */
    sccb_i2c_receive_func_t i2cReceiveFunc; /*!< I2C receive function. */
    void (*pullResetPin)(bool pullUp);      /*!< Function to pull reset pin high or low. */
    void (*pullPowerDownPin)(bool pullUp);  /*!< Function to pull the power down pin high or low. */
} ov5640_resource_t;

/*! @brief OV5640 operation functions. */
extern const camera_device_operations_t ov5640_ops;

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

#if defined(__cplusplus)
}
#endif

#endif /* _FSL_OV5640_H_ */

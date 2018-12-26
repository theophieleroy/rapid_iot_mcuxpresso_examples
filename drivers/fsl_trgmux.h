/*
 * The Clear BSD License
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
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

#ifndef _FSL_TRGMUX_H_
#define _FSL_TRGMUX_H_

#include "fsl_common.h"

/*! @addtogroup trgmux */
/*! @{ */


/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*! @name Driver version */
/*@{*/
/*! @brief TRGMUX driver version 2.0.0. */
#define FSL_TRGMUX_DRIVER_VERSION (MAKE_VERSION(2, 0, 0))
/*@}*/

/*! @brief TRGMUX configure status. */
enum _trgmux_status
{
    kStatus_TRGMUX_Locked = MAKE_STATUS(kStatusGroup_TRGMUX, 0), /*!< Configure failed for register is locked */
};

/*!
 * @brief Defines the MUX select for peripheral trigger input.
 */
typedef enum _trgmux_trigger_input
{
    kTRGMUX_TriggerInput0 = TRGMUX_TRGCFG_SEL0_SHIFT, /*!< The MUX select for peripheral trigger input 0*/
    kTRGMUX_TriggerInput1 = TRGMUX_TRGCFG_SEL1_SHIFT, /*!< The MUX select for peripheral trigger input 1*/
    kTRGMUX_TriggerInput2 = TRGMUX_TRGCFG_SEL2_SHIFT, /*!< The MUX select for peripheral trigger input 2*/
    kTRGMUX_TriggerInput3 = TRGMUX_TRGCFG_SEL3_SHIFT, /*!< The MUX select for peripheral trigger input 3*/
} trgmux_trigger_input_t;

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus*/

/*!
 *@name TRGMUX Functional Operation
 *@{*/

/*!
 * @brief Sets the flag of the register which is used to mark writeable.
 *
 * The function sets the flag of the register which is used to mark
 * writeable.
 * Example:
   @code
   TRGMUX_LockRegister(TRGMUX0,kTRGMUX_Trgmux0Dmamux0);
   @endcode
 *
 * @param base        TRGMUX peripheral base address.
 * @param index       The index of the TRGMUX register, see the enum trgmux_device_t
 *                    defined in <SOC>.h.
 */
static inline void TRGMUX_LockRegister(TRGMUX_Type *base, uint32_t index)
{
    base->TRGCFG[index] |= TRGMUX_TRGCFG_LK_MASK;
}

/*!
 * @brief Configures the trigger source of the appointed peripheral.
 *
 * The function configures the trigger source of the appointed peripheral.
 * Example:
   @code
   TRGMUX_SetTriggerSource(TRGMUX0, kTRGMUX_Trgmux0Dmamux0, kTRGMUX_TriggerInput0, kTRGMUX_SourcePortPin);
   @endcode
 * @param base        TRGMUX peripheral base address.
 * @param index       The index of the TRGMUX register, see the enum trgmux_device_t
 *                    defined in <SOC>.h.
 * @param input       The MUX select for peripheral trigger input
 * @param trigger_src The trigger inputs for various peripherals. See the enum trgmux_source_t
 *                    defined in <SOC>.h.
 * @retval  kStatus_Success  Configured successfully.
 * @retval  kStatus_TRGMUX_Locked   Configuration failed because the register is locked.
 */
status_t TRGMUX_SetTriggerSource(TRGMUX_Type *base,
                                 uint32_t index,
                                 trgmux_trigger_input_t input,
                                 uint32_t trigger_src);

/*@}*/

#if defined(__cplusplus)
}
#endif /* __cplusplus*/

/*! @}*/

#endif /* _FSL_TRGMUX_H_*/

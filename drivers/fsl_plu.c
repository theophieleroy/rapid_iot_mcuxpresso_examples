/*
 * The Clear BSD License
 * Copyright (c) 2018, NXP Semiconductors, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided
 * that the following conditions are met:
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

#include "fsl_plu.h"

/* Component ID definition, used by tools. */
#ifndef FSL_COMPONENT_ID
#define FSL_COMPONENT_ID "platform.drivers.plu"
#endif

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*!
 * @brief Gets the instance from the base address
 *
 * @param base PLU peripheral base address
 *
 * @return The PLU instance
 */
static uint32_t PLU_GetInstance(PLU_Type *base);

/*******************************************************************************
 * Variables
 ******************************************************************************/
/*! @brief Pointers to PLU bases for each instance. */
static PLU_Type *const s_pluBases[] = PLU_BASE_PTRS;

#if !(defined(FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL) && FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL)
/*! @brief Pointers to PLU clocks for each instance. */
static const clock_ip_name_t s_pluClocks[] = PLU_CLOCKS;
#endif /* FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL */

#if !(defined(FSL_SDK_DISABLE_DRIVER_RESET_CONTROL) && FSL_SDK_DISABLE_DRIVER_RESET_CONTROL)
/*! @brief Pointers to PLU resets for each instance. */
static const reset_ip_name_t s_lpuResets[] = PLU_RSTS_N;
#endif /* FSL_SDK_DISABLE_DRIVER_RESET_CONTROL */

/*******************************************************************************
 * Code
 ******************************************************************************/
static uint32_t PLU_GetInstance(PLU_Type *base)
{
    uint32_t instance;
    uint32_t pluArrayCount = (sizeof(s_pluBases) / sizeof(s_pluBases[0]));

    /* Find the instance index from base address mappings. */
    for (instance = 0; instance < pluArrayCount; instance++)
    {
        if (s_pluBases[instance] == base)
        {
            break;
        }
    }

    assert(instance < pluArrayCount);

    return instance;
}

void PLU_Init(PLU_Type *base)
{
#if !(defined(FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL) && FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL)
    /* Enable the PLU peripheral clock */
    CLOCK_EnableClock(s_pluClocks[PLU_GetInstance(base)]);
#endif /* FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL */

#if !(defined(FSL_SDK_DISABLE_DRIVER_RESET_CONTROL) && FSL_SDK_DISABLE_DRIVER_RESET_CONTROL)
    /* Reset the module. */
    RESET_PeripheralReset(s_lpuResets[PLU_GetInstance(base)]);
#endif /* FSL_SDK_DISABLE_DRIVER_RESET_CONTROL */
}

void PLU_Deinit(PLU_Type *base)
{
#if !(defined(FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL) && FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL)
    /* Gate the module clock */
    CLOCK_DisableClock((s_pluClocks[PLU_GetInstance(base)]));
#endif /* FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL */
}

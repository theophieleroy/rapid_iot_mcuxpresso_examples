/*
 * The Clear BSD License
 * Copyright 2017 NXP
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

#include "fsl_pwt.h"

/* Component ID definition, used by tools. */
#ifndef FSL_COMPONENT_ID
#define FSL_COMPONENT_ID "platform.drivers.pwt_1"
#endif


/*******************************************************************************
 * Prototypes
 ******************************************************************************/
/*!
 * @brief Gets the instance from the base address
 *
 * @param base PWT peripheral base address
 *
 * @return The PWT instance
 */
static uint32_t PWT_GetInstance(PWT_Type *base);

/*******************************************************************************
 * Variables
 ******************************************************************************/
/*! @brief Pointers to PWT bases for each instance. */
static PWT_Type *const s_pwtBases[] = PWT_BASE_PTRS;

#if !(defined(FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL) && FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL)
/*! @brief Pointers to PWT clocks for each instance. */
static const clock_ip_name_t s_pwtClocks[] = PWT_CLOCKS;
#endif /* FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL */

/*******************************************************************************
 * Code
 ******************************************************************************/
static uint32_t PWT_GetInstance(PWT_Type *base)
{
    uint32_t instance;

    /* Find the instance index from base address mappings. */
    for (instance = 0; instance < ARRAY_SIZE(s_pwtBases); instance++)
    {
        if (s_pwtBases[instance] == base)
        {
            break;
        }
    }

    assert(instance < ARRAY_SIZE(s_pwtBases));

    return instance;
}

void PWT_Init(PWT_Type *base, const pwt_config_t *config)
{
    assert(config);

#if !(defined(FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL) && FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL)
    /* Ungate the PWT module clock*/
    CLOCK_EnableClock(s_pwtClocks[PWT_GetInstance(base)]);
#endif /* FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL */

    /* Reset the module */
    PWT_Reset(base);

    /* Set clock source, prescale, input source and edge */
    base->R1 |= PWT_R1_PCLKS(config->clockSource) | PWT_R1_PRE(config->prescale) | PWT_R1_PINSEL(config->inputSelect) | PWT_R1_EDGE(config->edge);
}

void PWT_Deinit(PWT_Type *base)
{
    /* Disable the counter */
    PWT_StopTimer(base);

#if !(defined(FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL) && FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL)
    /* Gate the PWT clock */
    CLOCK_DisableClock(s_pwtClocks[PWT_GetInstance(base)]);
#endif /* FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL */
}

void PWT_GetDefaultConfig(pwt_config_t *config)
{
    assert(config);

    /* Use the Timer clock as source clock for the PWT submodule */
    config->clockSource = kPWT_TimerClock;
    /* Clock source prescale is set to divide by 1*/
    config->prescale = kPWT_Prescale_Divide_1;
    /* PWT input signal coming from Port 0 */
    config->inputSelect = kPWT_InputPort_0;
    /* The first rising edge starts, and the pulse width is captured on rising and falling edge */
    config->edge = kPWT_StartRise_CaptureRiseAndFall_Edge;
}

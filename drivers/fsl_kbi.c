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

#include "fsl_kbi.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/* Component ID definition, used by tools. */
#ifndef FSL_COMPONENT_ID
#define FSL_COMPONENT_ID "platform.drivers.kbi"
#endif


/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/

#if !(defined(FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL) && FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL)
/*! @brief Pointers to SEMC clocks for each instance. */
static const clock_ip_name_t s_kbiClock[FSL_FEATURE_SOC_KBI_COUNT] = KBI_CLOCKS;
#endif /* FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL */

/*! @brief Pointers to SEMC bases for each instance. */
static KBI_Type *const s_kbiBases[] = KBI_BASE_PTRS;

/*! @brief Pointers to Kbi IRQ number for each instance. */
static const IRQn_Type s_kbiIrqs[] = KBI_IRQS;
/*******************************************************************************
 * Code
 ******************************************************************************/
static uint32_t KBI_GetInstance(KBI_Type *base)
{
    uint32_t instance;

    /* Find the instance index from base address mappings. */
    for (instance = 0; instance < ARRAY_SIZE(s_kbiBases); instance++)
    {
        if (s_kbiBases[instance] == base)
        {
            break;
        }
    }

    assert(instance < ARRAY_SIZE(s_kbiBases));

    return instance;
}

void KBI_Init(KBI_Type *base, kbi_config_t *configure)
{
    assert(configure);

    uint8_t scReg;

    uint32_t instance = KBI_GetInstance(base);

#if !(defined(FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL) && FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL)
    /* Un-gate sdram controller clock. */
    CLOCK_EnableClock(s_kbiClock[KBI_GetInstance(base)]);
#endif /* FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL */

    /* Mask keyboard interrupts first to prevent false interrupt request during initialization */
    base->SC = 0;
    /* Setting KBI polarity. */
    base->ES = KBI_ES_KBEDG(configure->pinsEdge);
    /* Enable KBI pins. */
    base->PE = KBI_PE_KBIPE(configure->pinsEnabled);
    /* Clear any false interrupts. */
    scReg = KBI_SC_KBACK_MASK;
#if defined(FSL_FEATURE_KBI_HAS_SOURCE_PIN) && FSL_FEATURE_KBI_HAS_SOURCE_PIN
    /* Reset kbi sp register. */
    scReg |= KBI_SC_RSTKBSP_MASK | KBI_SC_KBSPEN_MASK;
#endif
    base->SC = scReg;

    /* Set KBI enable interrupts and KBI detect mode. */
    scReg = base->SC;
    base->SC = (configure->mode & KBI_SC_KBMOD_MASK) | KBI_SC_KBIE_MASK | scReg;
    /* Enable NVIC interrupt. */
    EnableIRQ(s_kbiIrqs[instance]);
}

void KBI_Deinit(KBI_Type *base)
{
    uint8_t scReg = KBI_SC_KBACK_MASK;

#if defined(FSL_FEATURE_KBI_HAS_SOURCE_PIN) && FSL_FEATURE_KBI_HAS_SOURCE_PIN
    /* Reset kbi sp register. */
    scReg |= KBI_SC_RSTKBSP_MASK;
#endif
    /* Disable interrupts. */
    base->SC = scReg;
    base->PE = 0;

#if !(defined(FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL) && FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL)
    /* Disable Kbi clock. */
    CLOCK_DisableClock(s_kbiClock[KBI_GetInstance(base)]);
#endif /* FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL */
}

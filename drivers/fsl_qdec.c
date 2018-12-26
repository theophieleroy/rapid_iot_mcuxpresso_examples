/*
 * The Clear BSD License
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
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

/**
 * @addtogroup QDEC
 * @{
 */
#include "fsl_qdec.h"

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
/*!
 * @brief Gets the instance from the base address to be used to gate or ungate the module clock
 *
 * @param base QDEC peripheral base address
 *
 * @return The QDEC instance
 */
static uint32_t QDEC_GetInstance(QDEC_Type *base);

/*******************************************************************************
 * Variables
 ******************************************************************************/
/*! @brief Pointers to QDEC bases for each instance. */
static QDEC_Type *const s_qdecBases[] = QDEC_BASE_PTRS;

#if !(defined(FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL) && FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL)
/*! @brief Pointers to QDEC clocks for each instance. */
static const clock_ip_name_t s_qdecClocks[] = QDEC_CLOCKS;
#endif /* FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL */

/*! @brief Pointers to QDEC resets for each instance. */
#if !(defined(FSL_SDK_DISABLE_DRIVER_RESET_CONTROL) && FSL_SDK_DISABLE_DRIVER_RESET_CONTROL)
static const reset_ip_name_t s_qdecResets[] = QDEC_RSTS;
#endif /* FSL_SDK_DISABLE_DRIVER_RESET_CONTROL */
       /*******************************************************************************
        * Code
        ******************************************************************************/
static uint32_t QDEC_GetInstance(QDEC_Type *base)
{
    uint32_t instance;

    /* Find the instance index from base address mappings. */
    for (instance = 0; instance < FSL_FEATURE_SOC_QDEC_COUNT; instance++)
    {
        if (s_qdecBases[instance] == base)
        {
            break;
        }
    }

    assert(instance < FSL_FEATURE_SOC_QDEC_COUNT);

    return instance;
}
void QDEC_Init(QDEC_Type *base, const qdec_config_t *config)
{
    uint32_t reg = 0;

#if !(defined(FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL) && FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL)
    /* Enable the clock. */
    CLOCK_EnableClock(s_qdecClocks[QDEC_GetInstance(base)]);
#endif /* FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL */

#if !(defined(FSL_SDK_DISABLE_DRIVER_RESET_CONTROL) && FSL_SDK_DISABLE_DRIVER_RESET_CONTROL)
    /* Reset the module. */
    RESET_PeripheralReset(s_qdecResets[QDEC_GetInstance(base)]);
#endif /* FSL_SDK_DISABLE_DRIVER_RESET_CONTROL */

    /* QDEC SAMP_CTL configuration */
    base->SAMP_CTRL = QDEC_SAMP_CTRL_DIVIDE(config->sampleClockDiv) | QDEC_SAMP_CTRL_PTS(config->samplePoint) |
                      QDEC_SAMP_CTRL_DB_SAMP_DIV(config->debounceFilterDiv);

    /* Debounce filter configure */
    if (config->debounceFilterDiv != kQDEC_DebounceFilterDivNone)
    {
        reg |= QDEC_CTRL_DB_FILTER_EN_MASK;
    }
    /* Single sample configure */
    if (config->samplePoint == kQDEC_SamplePoint1)
    {
        reg |= QDEC_CTRL_SINGLE_SAMPLE_SRST_EN_MASK;
    }

    if (config->autoClearEnable)
    {
        reg |= QDEC_CTRL_AUTO_CLR_EN_MASK;
    }
    /* Enable QDEC */
    reg |= QDEC_CTRL_QDEC_EN_MASK;

    /* QDEC QDEC_CTL configuration */
    base->CTRL = reg;
}
void QDEC_Deinit(QDEC_Type *base)
{
    /* Disable QDEC */
    base->CTRL &= ~QDEC_CTRL_QDEC_EN_MASK;

#if !(defined(FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL) && FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL)
    /* Disable the clock. */
    CLOCK_DisableClock(s_qdecClocks[QDEC_GetInstance(base)]);
#endif /* FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL */
}
void QDEC_GetDefaultConfig(qdec_config_t *config)
{
    config->samplePoint = kQDEC_SamplePoint40;
    config->sampleClockDiv = kQDEC_SampleClockDivider1;
    config->debounceFilterDiv = kQDEC_DebounceFilterDivider1;
    config->autoClearEnable = true;
}

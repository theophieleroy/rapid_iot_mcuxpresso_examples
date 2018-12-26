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

#include "fsl_dac32.h"

/* Component ID definition, used by tools. */
#ifndef FSL_COMPONENT_ID
#define FSL_COMPONENT_ID "platform.drivers.dac32"
#endif


/*******************************************************************************
 * Prototypes
 ******************************************************************************/
/*!
 * @brief Get instance number for DAC32 module.
 *
 * @param base DAC32 peripheral base address
 */
static uint32_t DAC32_GetInstance(DAC_Type *base);

/*******************************************************************************
 * Variables
 ******************************************************************************/
/*! @brief Pointers to DAC32 bases for each instance. */
static DAC_Type *const s_dac32Bases[] = DAC_BASE_PTRS;
#if !(defined(FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL) && FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL)
/*! @brief Pointers to DAC32 clocks for each instance. */
static const clock_ip_name_t s_dac32Clocks[] = DAC_CLOCKS;
#endif /* FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL */

/*******************************************************************************
 * Codes
 ******************************************************************************/
static uint32_t DAC32_GetInstance(DAC_Type *base)
{
    uint32_t instance;

    /* Find the instance index from base address mappings. */
    for (instance = 0; instance < ARRAY_SIZE(s_dac32Bases); instance++)
    {
        if (s_dac32Bases[instance] == base)
        {
            break;
        }
    }

    assert(instance < ARRAY_SIZE(s_dac32Bases));

    return instance;
}

void DAC32_Init(DAC_Type *base, const dac32_config_t *config)
{
    assert(NULL != config);

    uint32_t tmp32;

#if !(defined(FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL) && FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL)
    /* Enable the clock before any operation to DAC32 registers.*/
    CLOCK_EnableClock(s_dac32Clocks[DAC32_GetInstance(base)]);
#endif /* FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL */

    /* Configure. */
    tmp32 = base->STATCTRL & ~(DAC_STATCTRL_DACRFS_MASK | DAC_STATCTRL_LPEN_MASK);
    if (kDAC32_ReferenceVoltageSourceVref2 == config->referenceVoltageSource)
    {
        tmp32 |= DAC_STATCTRL_DACRFS_MASK;
    }
    if (config->enableLowPowerMode)
    {
        tmp32 |= DAC_STATCTRL_LPEN_MASK;
    }
    base->STATCTRL = tmp32;

    /* DAC32_Enable(base, true); */
    /* Move this function to application, so that users can enable it when they will. */
}

void DAC32_Deinit(DAC_Type *base)
{
    DAC32_Enable(base, false);

#if !(defined(FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL) && FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL)
    /* Disable the clock. */
    CLOCK_DisableClock(s_dac32Clocks[DAC32_GetInstance(base)]);
#endif /* FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL */
}

void DAC32_GetDefaultConfig(dac32_config_t *config)
{
    assert(NULL != config);

    config->referenceVoltageSource = kDAC32_ReferenceVoltageSourceVref2;
    config->enableLowPowerMode = false;
}

void DAC32_SetBufferConfig(DAC_Type *base, const dac32_buffer_config_t *config)
{
    assert(NULL != config);

    uint32_t tmp32;

    tmp32 = base->STATCTRL &
            ~(DAC_STATCTRL_DACTRGSEL_MASK | DAC_STATCTRL_DACBFMD_MASK | DAC_STATCTRL_DACBFUP_MASK |
              DAC_STATCTRL_DACBFWM_MASK);
    if (kDAC32_BufferTriggerBySoftwareMode == config->triggerMode)
    {
        tmp32 |= DAC_STATCTRL_DACTRGSEL_MASK;
    }
    tmp32 |= (DAC_STATCTRL_DACBFWM(config->watermark) | DAC_STATCTRL_DACBFMD(config->workMode) |
              DAC_STATCTRL_DACBFUP(config->upperLimit));
    base->STATCTRL = tmp32;
}

void DAC32_GetDefaultBufferConfig(dac32_buffer_config_t *config)
{
    assert(NULL != config);

    config->triggerMode = kDAC32_BufferTriggerBySoftwareMode;
    config->watermark = kDAC32_BufferWatermark1Word;
    config->workMode = kDAC32_BufferWorkAsNormalMode;
    config->upperLimit = DAC_DAT_COUNT * 2U - 1U;
}

void DAC32_SetBufferValue(DAC_Type *base, uint32_t index, uint32_t value)
{
    assert(index < (DAC_DAT_COUNT * 2U));

    if (0U == (index % 2U))
    {
        index = index / 2U; /* Register index. */
        base->DAT[index] = (base->DAT[index] & ~(DAC_DAT_DATA0_MASK)) | DAC_DAT_DATA0(value);
    }
    else
    {
        index = index / 2U; /* Register index. */
        base->DAT[index] = (base->DAT[index] & ~(DAC_DAT_DATA1_MASK)) | DAC_DAT_DATA1(value);
    }
}

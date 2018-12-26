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

#include "fsl_dac12.h"

/* Component ID definition, used by tools. */
#ifndef FSL_COMPONENT_ID
#define FSL_COMPONENT_ID "platform.drivers.dac12"
#endif


/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*!
 * @brief Get instance number for DAC12 module.
 *
 * @param base DAC12 peripheral base address
 */
static uint32_t DAC12_GetInstance(DAC_Type *base);

/*******************************************************************************
 * Variables
 ******************************************************************************/
/*! @brief Pointers to DAC bases for each instance. */
static DAC_Type *const s_dac12Bases[] = DAC_BASE_PTRS;
#if !(defined(FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL) && FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL)
/*! @brief Pointers to DAC clocks for each instance. */
static const clock_ip_name_t s_dac12Clocks[] = DAC_CLOCKS;
#endif /* FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL */

/*******************************************************************************
 * Codes
 ******************************************************************************/
static uint32_t DAC12_GetInstance(DAC_Type *base)
{
    uint32_t instance;

    /* Find the instance index from base address mappings. */
    for (instance = 0; instance < ARRAY_SIZE(s_dac12Bases); instance++)
    {
        if (s_dac12Bases[instance] == base)
        {
            break;
        }
    }

    assert(instance < ARRAY_SIZE(s_dac12Bases));

    return instance;
}

void DAC12_GetHardwareInfo(DAC_Type *base, dac12_hardware_info_t *info)
{
    assert(NULL != info);

    info->fifoSizeInfo = (dac12_fifo_size_info_t)((DAC_PARAM_FIFOSZ_MASK & base->PARAM) >> DAC_PARAM_FIFOSZ_SHIFT);
}

void DAC12_Init(DAC_Type *base, const dac12_config_t *config)
{
    assert(NULL != config);

    uint32_t tmp32;

#if !(defined(FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL) && FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL)
    /* Enable the clock. */
    CLOCK_EnableClock(s_dac12Clocks[DAC12_GetInstance(base)]);
#endif /* FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL */

    tmp32 = DAC_CR_WML(config->fifoWatermarkLevel); /* FIFO watermark level. */
    switch (config->fifoWorkMode)                   /* FIFO work mode. */
    {
        case kDAC12_FIFOWorkAsNormalMode:
            tmp32 |= DAC_CR_FIFOEN_MASK;
            break;
        case kDAC12_FIFOWorkAsSwingMode:
            tmp32 |= DAC_CR_FIFOEN_MASK | DAC_CR_SWMD_MASK;
            break;
        default: /* kDAC12_FIFODisabled. */
            break;
    }

    tmp32 |= DAC_CR_DACRFS(config->referenceVoltageSource) /* Reference voltage source. */
             | DAC_CR_TRGSEL(config->fifoTriggerMode);     /* Trigger mode. */

    base->CR = tmp32;

    /* DACx_CR2. */
    tmp32 = 0U;
    /* Reference voltage current. */
    switch (config->referenceCurrentSource)
    {
        case kDAC12_ReferenceCurrentSourceAlt0:
            tmp32 |= DAC_CR2_IREF_MASK;
            break;
        case kDAC12_ReferenceCurrentSourceAlt1:
            tmp32 |= DAC_CR2_IREF1_MASK;
            break;
        case kDAC12_ReferenceCurrentSourceAlt2:
            tmp32 |= DAC_CR2_IREF2_MASK;
            break;
        default: /* kDAC12_ReferenceCurrentSourceDisabled */
            break;
    }

    /* Speed mode. */
    switch (config->speedMode)
    {
        case kDAC12_SpeedMiddleMode:
            tmp32 |= DAC_CR2_BFMS_MASK;
            break;
        case kDAC12_SpeedHighMode:
            tmp32 |= DAC_CR2_BFHS_MASK;
            break;
        default: /* kDAC12_SpeedLowMode */
            break;
    }

    /* DAC buffered mode needs OPAMP enabled. DAC unbuffered mode needs OPAMP disabled. */
    if (config->enableAnalogBuffer)
    {
        tmp32 |= DAC_CR2_BFEN_MASK; /* OPAMP is used as buffer. */
    }
    else
    {
        tmp32 |= DAC_CR2_OEN_MASK; /* Output buffer is bypassed. */
    }
    base->CR2 = tmp32;

    base->ITRM = DAC_ITRM_TRIM(config->currentReferenceInternalTrimValue);
}

void DAC12_Deinit(DAC_Type *base)
{
    DAC12_Enable(base, false);
#if !(defined(FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL) && FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL)
    CLOCK_DisableClock(s_dac12Clocks[DAC12_GetInstance(base)]);
#endif /* FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL */
}

void DAC12_GetDefaultConfig(dac12_config_t *config)
{
    assert(NULL != config);

    config->fifoWatermarkLevel = 0U;
    config->fifoWorkMode = kDAC12_FIFODisabled;
    config->referenceVoltageSource = kDAC12_ReferenceVoltageSourceAlt1;
    config->fifoTriggerMode = kDAC12_FIFOTriggerByHardwareMode;
    config->referenceCurrentSource = kDAC12_ReferenceCurrentSourceAlt0;
    config->speedMode = kDAC12_SpeedLowMode;
    config->enableAnalogBuffer = false;
    config->currentReferenceInternalTrimValue = 0x4;
}

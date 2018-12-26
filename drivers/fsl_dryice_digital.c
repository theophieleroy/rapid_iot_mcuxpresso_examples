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

#include "fsl_dryice_digital.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/* Component ID definition, used by tools. */
#ifndef FSL_COMPONENT_ID
#define FSL_COMPONENT_ID "platform.drivers.dryice_digital"
#endif


/*******************************************************************************
 * Code
 ******************************************************************************/

void DRYICE_DIGITAL_Init(DRYICE_Type *base, const dryice_digital_config_t *config)
{
    assert(config);

    uint32_t wr_val;

    wr_val = DRYICE_DTOCR_DRYICE_EN_MASK | DRYICE_DTOCR_TEMP_HIGH_OFFSET(config->tempHighOffset) |
             DRYICE_DTOCR_TEMP_LOW_OFFSET(config->tempLowOffset);
    base->DTOCR = wr_val;

    wr_val = DRYICE_DTMR_TEMP_MON_TRIM(config->tempMonTrim) | DRYICE_DTMR_VOLT_MON_TRIM(config->voltMonTrim) |
             DRYICE_DTMR_BGR_TRIM(config->bgrTrim);
    base->DTMR = wr_val;

    base->DMCR |= DRYICE_DMCR_TEMP_DET_SEL(config->tempDetSel);
}

void DRYICE_DIGITAL_Deinit(DRYICE_Type *base)
{
    base->DTOCR &= ~DRYICE_DTOCR_DRYICE_EN_MASK;
}

void DRYICE_DIGITAL_GetDefaultConfig(DRYICE_Type *base, dryice_digital_config_t *config)
{
    assert(config);

    config->tempHighOffset = 0U;
    config->tempLowOffset = 0U;
    config->tempMonTrim = 0U;
    config->voltMonTrim = 0U;
    config->bgrTrim = 0U;
    config->tempDetSel = kDRYICE_DIGITAL_Range_105C_125C;
}

void DRYICE_DIGITAL_PinGetDefaultConfig(DRYICE_Type *base, dryice_digital_pin_config_t *pinConfig)
{
    assert(pinConfig);

    pinConfig->ignoredWindowWidth = kDRYICE_DIGITAL_NoIgnoredWindow;
    pinConfig->ignoredWinClkSel = kDRYICE_DIGITAL_32768Hz;
    pinConfig->ignoredWinStartCycle = 0U;

    pinConfig->extTamperRoutingCtrl = kDRYICE_DIGITAL_PassiveInput;
    pinConfig->passiveTamperExpDat = kDRYICE_DIGITAL_ExpectedTamperHigh;

    pinConfig->pullEnable = false;
    pinConfig->pullContinuousEnable = false;
    pinConfig->pullSelect = kDRYICE_DIGITAL_PullDown;
    pinConfig->pullTransWidth = kDRYICE_DIGITAL_1CycleTransitionWidth;

    pinConfig->driveStrength = kDRYICE_DIGITAL_DriveStrenght1;

    pinConfig->hysteresisCtrl = kDRYICE_DIGITAL_Schmitt;
    pinConfig->outputKeepEnable = false;

    pinConfig->sampleClkSel = kDRYICE_DIGITAL_32768Hz;
    pinConfig->sampleFreqSel = kDRYICE_DIGITAL_Every8Cycles;
    pinConfig->sampleWidth = kDRYICE_DIGITAL_SamplingDisabled;
}

status_t DRYICE_DIGITAL_SetPinConfig(DRYICE_Type *base,
                                     dryice_digital_tamper_pin_t pin,
                                     const dryice_digital_pin_config_t *pinConfig)
{
    assert(pinConfig);

    uint32_t wr_val = 0;

    wr_val |= DRYICE_TPCTRL_IGNWD(pinConfig->ignoredWindowWidth);
    wr_val |= DRYICE_TPCTRL_IGWCS(pinConfig->ignoredWinClkSel);
    wr_val |= DRYICE_TPCTRL_IGNWS(pinConfig->ignoredWinStartCycle);
    wr_val |= DRYICE_TPCTRL_TPEC(pinConfig->pullContinuousEnable);
    wr_val |= DRYICE_TPCTRL_ETRCTRL(pinConfig->extTamperRoutingCtrl);
    wr_val |= DRYICE_TPCTRL_PTED(pinConfig->passiveTamperExpDat);
    wr_val |= DRYICE_TPCTRL_TPS(pinConfig->pullSelect);
    wr_val |= DRYICE_TPCTRL_TPE(pinConfig->pullEnable);
    wr_val |= DRYICE_TPCTRL_HC(pinConfig->hysteresisCtrl);
    wr_val |= DRYICE_TPCTRL_OUTKEEPER_EN(pinConfig->outputKeepEnable);
    wr_val |= DRYICE_TPCTRL_DSC(pinConfig->driveStrength);
    wr_val |= DRYICE_TPCTRL_TSCS(pinConfig->sampleClkSel);
    wr_val |= DRYICE_TPCTRL_TPSF(pinConfig->sampleFreqSel);
    wr_val |= DRYICE_TPCTRL_TPSW(pinConfig->sampleWidth);
    wr_val |= DRYICE_TPCTRL_TPPTW(pinConfig->pullTransWidth);

    base->TPCTRL[pin] = wr_val;

    if (base->TPCTRL[pin] != wr_val)
    {
        return kStatus_Fail;
    }
    else
    {
        return kStatus_Success;
    }
}

void DRYICE_DIGITAL_EnableTampers(DRYICE_Type *base, uint32_t mask)
{
    uint32_t wr_val = 0;

    if (mask & kDRYICE_DIGITAL_ClockTamper)
    {
        wr_val |= DRYICE_DMCR_CLOCK_DET_EN_MASK;
    }
    if (mask & kDRYICE_DIGITAL_VoltageTamper)
    {
        wr_val |= DRYICE_DMCR_VOLT_DET_EN_MASK;
    }
    if (mask & kDRYICE_DIGITAL_TemperatureTamper)
    {
        wr_val |= DRYICE_DMCR_TEMP_DET_EN_MASK;
    }

    base->DMCR |= wr_val;
}

void DRYICE_DIGITAL_DisableTampers(DRYICE_Type *base, uint32_t mask)
{
    uint32_t wr_val = 0;

    if (mask & kDRYICE_DIGITAL_ClockTamper)
    {
        wr_val |= DRYICE_DMCR_CLOCK_DET_EN_MASK;
    }
    if (mask & kDRYICE_DIGITAL_VoltageTamper)
    {
        wr_val |= DRYICE_DMCR_VOLT_DET_EN_MASK;
    }
    if (mask & kDRYICE_DIGITAL_TemperatureTamper)
    {
        wr_val |= DRYICE_DMCR_TEMP_DET_EN_MASK;
    }

    base->DMCR &= ~wr_val;
}

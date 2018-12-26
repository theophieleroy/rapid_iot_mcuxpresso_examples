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
#include "fsl_adc12.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/* Component ID definition, used by tools. */
#ifndef FSL_COMPONENT_ID
#define FSL_COMPONENT_ID "platform.drivers.adc12"
#endif

/*! @brief Transform raw signed calibration result to unified type int32_t. */
#define ADC12_TRANSFORM_CALIBRATION_RESULT(resultValue, bitWidth) \
    (((resultValue) >= (1 << ((bitWidth)-1))) ? ((resultValue) - (1 << (bitWidth))) : (resultValue));

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
/*!
 * @brief Get instance number for ADC12 module.
 *
 * @param base ADC12 peripheral base address
 */
static uint32_t ADC12_GetInstance(ADC_Type *base);

/*!
 * @brief Check calibration failed status.
 *
 * Check if the calibration is failed by comparing the calibration result value with its limit range.
 * 1. Each calibration result value's limit range is:
 *     Symbol       Min            Typ                      Max
 *     ______________________________________________________________________________________
 *     OFS          -48            -8                        22
 *     CLP9         -12             4                        20
 *     CLPX         -16             0                        16
 *     CLPS          30             72                       120
 *     CLP0          CLPS-14        CLPS                     CLPS+14
 *     CLP1          Typ1-16        Typ1=CLP0+CLP0           Typ1+16
 *     CLP2          Typ2-20        Typ2=CLP1+CLP1-26        Typ2+20
 *     CLP3          Typ3-36        Typ3=CLP2+CLP2           Typ3+36
 * 2. To get the accurate calibration value, following conditions should be met.
 *     1). Enable hardware average and set average number to 32.
 *     2). No parallel calibration of ADCs because they will disturb each other.
 *     2). For VREFH pin on PCB, use 3 bypass capacitance in the range: 1uF, 100nF and 1nF and place them as close as
 *         possible to the VREFH pin.
 * @param base ADC12 peripheral base address.
 * @retval kStatus_Success Calibration is done successfully.
 * @retval kStatus_Fail Calibration is failed.
 */
static status_t ADC12_GetCalibrationStatus(ADC_Type *base);

/*******************************************************************************
 * Variables
 ******************************************************************************/
/*! @brief Pointers to ADC12 bases for each instance. */
static ADC_Type *const s_adc12Bases[] = ADC_BASE_PTRS;
#if !(defined(FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL) && FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL)
/*! @brief Pointers to ADC12 clocks for each instance. */
static const clock_ip_name_t s_adc12Clocks[] = ADC12_CLOCKS;
#endif /* FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL */

/*******************************************************************************
 * Code
 ******************************************************************************/
static uint32_t ADC12_GetInstance(ADC_Type *base)
{
    uint32_t instance;

    /* Find the instance index from base address mappings. */
    for (instance = 0; instance < ARRAY_SIZE(s_adc12Bases); instance++)
    {
        if (s_adc12Bases[instance] == base)
        {
            break;
        }
    }

    assert(instance < ARRAY_SIZE(s_adc12Bases));

    return instance;
}

static status_t ADC12_GetCalibrationStatus(ADC_Type *base)
{
    /* Get raw calibration result. OFS, CLP9, CLPX are signed number. The highest position bit is the signal bit.
    Other calibration value registers are unsigned number. */
    int32_t OFS = (int32_t)((base->OFS & ADC_OFS_OFS_MASK) >> ADC_OFS_OFS_SHIFT);
    int32_t CLP9 = (int32_t)((base->CLP9 & ADC_CLP9_CLP9_MASK) >> ADC_CLP9_CLP9_SHIFT);
    int32_t CLPX = (int32_t)((base->CLPX & ADC_CLPX_CLPX_MASK) >> ADC_CLPX_CLPX_SHIFT);
    uint32_t CLPS = ((base->CLPS & ADC_CLPS_CLPS_MASK) >> ADC_CLPS_CLPS_SHIFT);
    uint32_t CLP0 = ((base->CLP0 & ADC_CLP0_CLP0_MASK) >> ADC_CLP0_CLP0_SHIFT);
    uint32_t CLP1 = ((base->CLP1 & ADC_CLP1_CLP1_MASK) >> ADC_CLP1_CLP1_SHIFT);
    uint32_t CLP2 = ((base->CLP2 & ADC_CLP2_CLP2_MASK) >> ADC_CLP2_CLP2_SHIFT);
    uint32_t CLP3 = ((base->CLP3 & ADC_CLP3_CLP3_MASK) >> ADC_CLP3_CLP3_SHIFT);
    uint32_t Typ1 = (CLP0 + CLP0);
    uint32_t Typ2 = (CLP1 + CLP1 - 26U);
    uint32_t Typ3 = (CLP2 + CLP2);
    status_t error = kStatus_Success;

    /* Transform raw calibration result to unified type int32_t when the conversion result value is signed number. */
    OFS = ADC12_TRANSFORM_CALIBRATION_RESULT(OFS, 16);
    CLP9 = ADC12_TRANSFORM_CALIBRATION_RESULT(CLP9, 7);
    CLPX = ADC12_TRANSFORM_CALIBRATION_RESULT(CLPX, 7);

    /* Check the calibration result value with its limit range. */

    if ((OFS < -48) || (OFS > 22) || (CLP9 < -12) || (CLP9 > 20) || (CLPX < -16) || (CLPX > 16) || (CLPS < 30U) ||
        (CLPS > 120U) || (CLP0 < (CLPS - 14U)) || (CLP0 > (CLPS + 14U)) || (CLP1 < (Typ1 - 16U)) ||
        (CLP1 > (Typ1 + 16U)) || (CLP2 < (Typ2 - 20U)) || (CLP2 > (Typ2 + 20U)) || (CLP3 < (Typ3 - 36U)) ||
        (CLP3 > (Typ3 + 36U)))
    {
        error = kStatus_Fail;
    }

    return error;
}

void ADC12_Init(ADC_Type *base, const adc12_config_t *config)
{
    assert(config);

    uint32_t tmp32;

#if !(defined(FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL) && FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL)
    /* Enable the clock. */
    CLOCK_EnableClock(s_adc12Clocks[ADC12_GetInstance(base)]);
#endif /* FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL */

    /* ADCx_CFG1. */
    tmp32 = (base->CFG1 & ~(ADC_CFG1_ADICLK_MASK | ADC_CFG1_ADIV_MASK | ADC_CFG1_MODE_MASK));
    tmp32 |= (ADC_CFG1_ADICLK(config->clockSource) | ADC_CFG1_ADIV(config->clockDivider) |
              ADC_CFG1_MODE(config->resolution));
    base->CFG1 = tmp32;

    /* ADCx_CFG2. */
    tmp32 = (base->CFG2 & ~ADC_CFG2_SMPLTS_MASK);
    tmp32 |= ADC_CFG2_SMPLTS(config->sampleClockCount - 1U);
    base->CFG2 = tmp32;

    /* ADCx_SC2. */
    tmp32 = (base->SC2 & ~ADC_SC2_REFSEL_MASK);
    tmp32 |= ADC_SC2_REFSEL(config->referenceVoltageSource);
    base->SC2 = tmp32;

    /* ADCx_SC3. */
    tmp32 = (base->SC3 & ~ADC_SC3_ADCO_MASK);
    if (true == config->enableContinuousConversion)
    {
        tmp32 |= ADC_SC3_ADCO_MASK;
    }
    base->SC3 = tmp32;
}

void ADC12_Deinit(ADC_Type *base)
{
#if !(defined(FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL) && FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL)
    /* Disable the clock. */
    CLOCK_DisableClock(s_adc12Clocks[ADC12_GetInstance(base)]);
#endif /* FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL */
}

void ADC12_GetDefaultConfig(adc12_config_t *config)
{
    assert(config);

    config->referenceVoltageSource = kADC12_ReferenceVoltageSourceVref;
    config->clockSource = kADC12_ClockSourceAlt0;
    config->clockDivider = kADC12_ClockDivider1;
    config->resolution = kADC12_Resolution8Bit;
    config->sampleClockCount = 13U;
    config->enableContinuousConversion = false;
}

void ADC12_SetChannelConfig(ADC_Type *base, uint32_t channelGroup, const adc12_channel_config_t *config)
{
    assert(channelGroup < ADC_SC1_COUNT);
    assert(config);

    uint32_t tmp32;

    /* ADCx_SC1n. */
    tmp32 = (base->SC1[channelGroup] & ~(ADC_SC1_ADCH_MASK | ADC_SC1_AIEN_MASK));
    tmp32 |= ADC_SC1_ADCH(config->channelNumber);
    if (true == config->enableInterruptOnConversionCompleted)
    {
        tmp32 |= ADC_SC1_AIEN_MASK;
    }
    base->SC1[channelGroup] = tmp32;
}

uint32_t ADC12_GetChannelStatusFlags(ADC_Type *base, uint32_t channelGroup)
{
    assert(channelGroup < ADC_SC1_COUNT);

    uint32_t tmp32 = base->SC1[channelGroup];
    uint32_t result = 0U;

    /* ADCx_SC1n. */
    if (ADC_SC1_COCO_MASK == (tmp32 & ADC_SC1_COCO_MASK))
    {
        result |= kADC12_ChannelConversionCompletedFlag;
    }

    return result;
}

status_t ADC12_DoAutoCalibration(ADC_Type *base)
{
    bool enabledHardwareTrigger = false;
    bool enabledHardwareAverage = false;
    uint32_t averageMode;
    uint32_t tmp32;
    status_t error = kStatus_Success;

    /* Save current trigger mode. Then set to software trigger mode. */
    tmp32 = base->SC2;
    if (ADC_SC2_ADTRG_MASK == (tmp32 & ADC_SC2_ADTRG_MASK))
    {
        enabledHardwareTrigger = true;
        tmp32 &= ~ADC_SC2_ADTRG_MASK;
        base->SC2 = tmp32;
    }
    /* Save current average mode. Then enable hardware average and set average number to be maximum value. */
    tmp32 = base->SC3;
    averageMode = ((tmp32 & ADC_SC3_AVGS_MASK) >> ADC_SC3_AVGS_SHIFT);
    if (ADC_SC3_AVGE_MASK == (tmp32 & ADC_SC3_AVGE_MASK))
    {
        enabledHardwareAverage = true;
    }
    tmp32 &= ~ADC_SC3_AVGS_MASK;
    tmp32 |= (ADC_SC3_AVGE_MASK | ADC_SC3_AVGS(ADC_SC3_AVGS_MASK >> ADC_SC3_AVGS_SHIFT));

    /* Trigger calibration and wait until it complete. */
    tmp32 |= ADC_SC3_CAL_MASK;
    base->SC3 = tmp32;
    while (kADC12_ChannelConversionCompletedFlag !=
           (ADC12_GetChannelStatusFlags(base, 0U) & kADC12_ChannelConversionCompletedFlag))
    {
    }

    if (kADC12_CalibrationFailedFlag == (ADC12_GetStatusFlags(base) & kADC12_CalibrationFailedFlag))
    {
        error = kStatus_Fail;
    }
    /* Clear conversion done flag. */
    ADC12_GetChannelConversionValue(base, 0U);

    /* Restore original trigger mode. */
    if (true == enabledHardwareTrigger)
    {
        base->SC2 |= ADC_SC2_ADTRG_MASK;
    }
    /* Restore original average mode. */
    tmp32 = base->SC3;
    if (false == enabledHardwareAverage)
    {
        tmp32 &= ~ADC_SC3_AVGE_MASK;
    }
    tmp32 |= ADC_SC3_AVGS(averageMode);
    base->SC3 = tmp32;

    return error;
}

void ADC12_SetHardwareCompareConfig(ADC_Type *base, const adc12_hardware_compare_config_t *config)
{
    uint32_t tmp32;

    /* Disable hardware compare. */
    if (NULL == config)
    {
        base->SC2 &= ~(ADC_SC2_ACFE_MASK | ADC_SC2_ACFGT_MASK | ADC_SC2_ACREN_MASK);
    }
    else
    {
        /* Set the compare mode. */
        tmp32 = (base->SC2 & ~(ADC_SC2_ACFE_MASK | ADC_SC2_ACFGT_MASK | ADC_SC2_ACREN_MASK));
        switch (config->hardwareCompareMode)
        {
            case kADC12_HardwareCompareMode0:
                break;
            case kADC12_HardwareCompareMode1:
                tmp32 |= ADC_SC2_ACFGT_MASK;
                break;
            case kADC12_HardwareCompareMode2:
                tmp32 |= ADC_SC2_ACREN_MASK;
                break;
            case kADC12_HardwareCompareMode3:
                tmp32 |= (ADC_SC2_ACFGT_MASK | ADC_SC2_ACREN_MASK);
                break;
            default:
                break;
        }
        tmp32 |= ADC_SC2_ACFE_MASK;
        base->SC2 = tmp32;

        /* Set the compare value. */
        base->CV1 = config->value1;
        base->CV2 = config->value2;
    }
}

void ADC12_SetHardwareAverage(ADC_Type *base, adc12_hardware_average_mode_t mode)
{
    uint32_t tmp32 = base->SC3;

    /* ADCx_SC3. */
    tmp32 &= ~(ADC_SC3_AVGS_MASK | ADC_SC3_AVGE_MASK);
    switch (mode)
    {
        case kADC12_HardwareAverageCount4:
        case kADC12_HardwareAverageCount8:
        case kADC12_HardwareAverageCount16:
        case kADC12_HardwareAverageCount32:
            tmp32 |= (ADC_SC3_AVGS(mode) | ADC_SC3_AVGE_MASK);
            break;
        case kADC12_HardwareAverageDisabled:
            break;
        default:
            break;
    }
    base->SC3 = tmp32;
}

uint32_t ADC12_GetStatusFlags(ADC_Type *base)
{
    uint32_t result = 0;

    /* ADCx_SC2. */
    if (ADC_SC2_ADACT_MASK == (base->SC2 & ADC_SC2_ADACT_MASK))
    {
        result |= kADC12_ActiveFlag;
    }

    if (kStatus_Fail == ADC12_GetCalibrationStatus(base))
    {
        result |= kADC12_CalibrationFailedFlag;
    }

    return result;
}

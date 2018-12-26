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
#include "fsl_tsi_v5.h"

/* Component ID definition, used by tools. */
#ifndef FSL_COMPONENT_ID
#define FSL_COMPONENT_ID "platform.drivers.tsi_v5"
#endif

void TSI_InitSelfCapMode(TSI_Type *base, const tsi_selfCap_config_t *config)
{
    uint32_t temp = 0U;
    assert(config != NULL);

    bool is_module_enabled = false;
    bool is_int_enabled = false;

#if !(defined(FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL) && FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL)
    CLOCK_EnableClock(kCLOCK_Tsi0);
#endif /* FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL */
    if (base->GENCS & TSI_GENCS_TSIEN_MASK)
    {
        is_module_enabled = true;
        TSI_EnableModule(base, false);
    }
    if (base->GENCS & TSI_GENCS_TSIIEN_MASK)
    {
        is_int_enabled = true;
        TSI_DisableInterrupts(base, kTSI_GlobalInterruptEnable);
    }

    /* common settings */
    temp = (base->MODE) & ~(TSI_MODE_SETCLK_MASK | TSI_MODE_MODE_MASK | TSI_MODE_S_SEN_MASK | TSI_MODE_S_W_SHIELD_MASK);
    base->MODE = temp | (TSI_MODE_S_W_SHIELD(config->enableShield) | TSI_MODE_S_SEN(config->enableSensitivity) |
                         TSI_MODE_SETCLK(config->commonConfig.mainClock) | TSI_MODE_MODE(config->commonConfig.mode));

    base->GENCS =
        (base->GENCS & (~(ALL_FLAGS_MASK | TSI_GENCS_DVOLT_MASK))) | TSI_GENCS_DVOLT(config->commonConfig.dvolt);

    temp = (base->SINC) & ~(TSI_SINC_CUTOFF_MASK | TSI_SINC_ORDER_MASK | TSI_SINC_DECIMATION_MASK);
    base->SINC = temp | (TSI_SINC_CUTOFF(config->commonConfig.cutoff) | TSI_SINC_ORDER(config->commonConfig.order) |
                         TSI_SINC_DECIMATION(config->commonConfig.decimation));

    temp = (base->SSC0) &
           ~(TSI_SSC0_CHARGE_NUM_MASK | TSI_SSC0_BASE_NOCHARGE_NUM_MASK | TSI_SSC0_PRBS_OUTSEL_MASK |
             TSI_SSC0_SSC_PRESCALE_NUM_MASK | TSI_SSC0_SSC_MODE_MASK);
    base->SSC0 =
        temp | (TSI_SSC0_PRBS_OUTSEL(config->commonConfig.prbsOutsel) |
                TSI_SSC0_SSC_MODE(config->commonConfig.ssc_mode) | TSI_SSC0_CHARGE_NUM(config->commonConfig.chargeNum) |
                TSI_SSC0_BASE_NOCHARGE_NUM(config->commonConfig.noChargeNum) |
                TSI_SSC0_SSC_PRESCALE_NUM(config->commonConfig.ssc_prescaler));

    /* Self-cap mode specific settings */
    temp = (base->MODE) & ~(TSI_MODE_S_XDN_MASK | TSI_MODE_S_CTRIM_MASK | TSI_MODE_S_XIN_MASK | TSI_MODE_S_XCH_MASK);
    base->MODE = temp | (TSI_MODE_S_XDN(config->xdn) | TSI_MODE_S_CTRIM(config->ctrim) |
                         TSI_MODE_S_XIN(config->inputCurrent) | TSI_MODE_S_XCH(config->chargeCurrent));

    if (is_module_enabled)
    {
        TSI_EnableModule(base, true);
    }
    if (is_int_enabled)
    {
        TSI_EnableInterrupts(base, kTSI_GlobalInterruptEnable);
    }
}

void TSI_InitMutualCapMode(TSI_Type *base, const tsi_mutualCap_config_t *config)
{
    uint32_t temp = 0U;
    assert(config != NULL);

    bool is_module_enabled = false;
    bool is_int_enabled = false;

#if !(defined(FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL) && FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL)
    CLOCK_EnableClock(kCLOCK_Tsi0);
#endif /* FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL */
    if (base->GENCS & TSI_GENCS_TSIEN_MASK)
    {
        is_module_enabled = true;
        TSI_EnableModule(base, false);
    }
    if (base->GENCS & TSI_GENCS_TSIIEN_MASK)
    {
        is_int_enabled = true;
        TSI_DisableInterrupts(base, kTSI_GlobalInterruptEnable);
    }

    /* Common settings */
    temp = (base->MODE) & ~(TSI_MODE_SETCLK_MASK | TSI_MODE_MODE_MASK);
    base->MODE = temp | (TSI_MODE_SETCLK(config->commonConfig.mainClock) | TSI_MODE_MODE(config->commonConfig.mode));

    base->GENCS =
        (base->GENCS & (~(ALL_FLAGS_MASK | TSI_GENCS_DVOLT_MASK))) | TSI_GENCS_DVOLT(config->commonConfig.dvolt);

    temp = (base->SINC) & ~(TSI_SINC_CUTOFF_MASK | TSI_SINC_ORDER_MASK | TSI_SINC_DECIMATION_MASK);
    base->SINC = temp | (TSI_SINC_CUTOFF(config->commonConfig.cutoff) | TSI_SINC_ORDER(config->commonConfig.order) |
                         TSI_SINC_DECIMATION(config->commonConfig.decimation));

    temp = (base->SSC0) &
           ~(TSI_SSC0_CHARGE_NUM_MASK | TSI_SSC0_BASE_NOCHARGE_NUM_MASK | TSI_SSC0_PRBS_OUTSEL_MASK |
             TSI_SSC0_SSC_PRESCALE_NUM_MASK | TSI_SSC0_SSC_MODE_MASK);
    base->SSC0 =
        temp | (TSI_SSC0_PRBS_OUTSEL(config->commonConfig.prbsOutsel) |
                TSI_SSC0_SSC_MODE(config->commonConfig.ssc_mode) | TSI_SSC0_CHARGE_NUM(config->commonConfig.chargeNum) |
                TSI_SSC0_BASE_NOCHARGE_NUM(config->commonConfig.noChargeNum) |
                TSI_SSC0_SSC_PRESCALE_NUM(config->commonConfig.ssc_prescaler));

    /* Mutual-cap mode specific configurations */
    temp = (base->MUL0) & ~(TSI_MUL0_M_PRE_CURRENT_MASK | TSI_MUL0_M_PRE_RES_MASK | TSI_MUL0_M_SEN_RES_MASK);
    base->MUL0 = temp | (TSI_MUL0_M_PRE_CURRENT(config->preCurrent) | TSI_MUL0_M_PRE_RES(config->preResistor) |
                         TSI_MUL0_M_SEN_RES(config->senseResistor));

    temp = (base->MUL1) &
           ~(TSI_MUL1_M_SEN_BOOST_MASK | TSI_MUL1_M_MODE_MASK | TSI_MUL1_M_PMIRRORL_MASK | TSI_MUL1_M_PMIRRORR_MASK |
             TSI_MUL1_M_NMIRROR_MASK);
    base->MUL1 = temp | (TSI_MUL1_M_SEN_BOOST(config->boostCurrent) | TSI_MUL1_M_MODE(config->txDriveMode) |
                         TSI_MUL1_M_PMIRRORL(config->pmosLeftCurrent) | TSI_MUL1_M_PMIRRORR(config->pmosRightCurrent) |
                         TSI_MUL1_M_NMIRROR(config->nmosCurrent));

    if (is_module_enabled)
    {
        TSI_EnableModule(base, true);
    }
    if (is_int_enabled)
    {
        TSI_EnableInterrupts(base, kTSI_GlobalInterruptEnable);
    }
}

void TSI_Deinit(TSI_Type *base)
{
    base->GENCS = 0U;
    base->DATA = 0U;
    base->TSHD = 0U;
#if !(defined(FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL) && FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL)
    CLOCK_DisableClock(kCLOCK_Tsi0);
#endif /* FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL */
}

void TSI_GetSelfCapModeDefaultConfig(tsi_selfCap_config_t *userConfig)
{
    userConfig->commonConfig.mainClock = kTSI_MainClockSlection_0;
    userConfig->commonConfig.mode = kTSI_SensingModeSlection_Self;
    userConfig->commonConfig.dvolt = kTSI_DvoltOption_2;
    userConfig->commonConfig.cutoff = kTSI_SincCutoffDiv_0;
    userConfig->commonConfig.order = kTSI_SincFilterOrder_1;
    userConfig->commonConfig.decimation = kTSI_SincDecimationValue_8;
    userConfig->commonConfig.chargeNum = kTSI_SscChargeNumValue_3;
    userConfig->commonConfig.prbsOutsel = kTSI_SscPrbsOutsel_2;
    userConfig->commonConfig.noChargeNum = kTSI_SscNoChargeNumValue_2;
    userConfig->commonConfig.ssc_mode = kTSI_ssc_prbs_method;
    userConfig->commonConfig.ssc_prescaler = kTSI_ssc_div_by_2;
    userConfig->enableSensitivity = true;
    userConfig->enableShield = false;
    userConfig->xdn = kTSI_SensitivityXdnOption_1;
    userConfig->ctrim = kTSI_SensitivityCtrimOption_7;
    userConfig->inputCurrent = kTSI_CurrentMultipleInputValue_0;
    userConfig->chargeCurrent = kTSI_CurrentMultipleChargeValue_1;
}

void TSI_GetMutualCapModeDefaultConfig(tsi_mutualCap_config_t *userConfig)
{
    userConfig->commonConfig.mainClock = kTSI_MainClockSlection_1;
    userConfig->commonConfig.mode = kTSI_SensingModeSlection_Mutual;
    userConfig->commonConfig.dvolt = kTSI_DvoltOption_0;
    userConfig->commonConfig.cutoff = kTSI_SincCutoffDiv_0;
    userConfig->commonConfig.order = kTSI_SincFilterOrder_1;
    userConfig->commonConfig.decimation = kTSI_SincDecimationValue_8;
    userConfig->commonConfig.chargeNum = kTSI_SscChargeNumValue_4;
    userConfig->commonConfig.prbsOutsel = kTSI_SscPrbsOutsel_2;
    userConfig->commonConfig.noChargeNum = kTSI_SscNoChargeNumValue_5;
    userConfig->commonConfig.ssc_mode = kTSI_ssc_prbs_method;
    userConfig->commonConfig.ssc_prescaler = kTSI_ssc_div_by_2;
    userConfig->preCurrent = kTSI_MutualPreCurrent_4uA;
    userConfig->preResistor = kTSI_MutualPreResistor_4k;
    userConfig->senseResistor = kTSI_MutualSenseResistor_10k;
    userConfig->boostCurrent = kTSI_MutualSenseBoostCurrent_0uA;
    userConfig->txDriveMode = kTSI_MutualTxDriveModeOption_0;
    userConfig->pmosLeftCurrent = kTSI_MutualPmosCurrentMirrorLeft_32;
    userConfig->pmosRightCurrent = kTSI_MutualPmosCurrentMirrorRight_1;
    userConfig->enableNmosMirror = true;
    userConfig->nmosCurrent = kTSI_MutualNmosCurrentMirror_1;
}

void TSI_SelfCapCalibrate(TSI_Type *base, tsi_calibration_data_t *calBuff)
{
    assert(calBuff != NULL);

    uint8_t i = 0U;
    bool is_int_enabled = false;

    if (base->GENCS & TSI_GENCS_TSIIEN_MASK)
    {
        is_int_enabled = true;
        TSI_DisableInterrupts(base, kTSI_GlobalInterruptEnable);
    }
    for (i = 0U; i < FSL_FEATURE_TSI_CHANNEL_COUNT; i++)
    {
        TSI_SetSelfCapMeasuredChannel(base, i);
        TSI_StartSoftwareTrigger(base);
        while (!(TSI_GetStatusFlags(base) & kTSI_EndOfScanFlag))
        {
        }
        calBuff->calibratedData[i] = TSI_GetCounter(base);
        TSI_ClearStatusFlags(base, kTSI_EndOfScanFlag);
    }
    if (is_int_enabled)
    {
        TSI_EnableInterrupts(base, kTSI_GlobalInterruptEnable);
    }
}

void TSI_EnableInterrupts(TSI_Type *base, uint32_t mask)
{
    uint32_t regValue = base->GENCS & (~ALL_FLAGS_MASK);

    if (mask & kTSI_GlobalInterruptEnable)
    {
        regValue |= TSI_GENCS_TSIIEN_MASK;
    }
    if (mask & kTSI_OutOfRangeInterruptEnable)
    {
        regValue &= (~TSI_GENCS_ESOR_MASK);
    }
    if (mask & kTSI_EndOfScanInterruptEnable)
    {
        regValue |= TSI_GENCS_ESOR_MASK;
    }

    base->GENCS = regValue; /* write value to register */
}

void TSI_DisableInterrupts(TSI_Type *base, uint32_t mask)
{
    uint32_t regValue = base->GENCS & (~ALL_FLAGS_MASK);

    if (mask & kTSI_GlobalInterruptEnable)
    {
        regValue &= (~TSI_GENCS_TSIIEN_MASK);
    }
    if (mask & kTSI_OutOfRangeInterruptEnable)
    {
        regValue |= TSI_GENCS_ESOR_MASK;
    }
    if (mask & kTSI_EndOfScanInterruptEnable)
    {
        regValue &= (~TSI_GENCS_ESOR_MASK);
    }

    base->GENCS = regValue; /* write value to register */
}

void TSI_ClearStatusFlags(TSI_Type *base, uint32_t mask)
{
    uint32_t regValue = base->GENCS & (~ALL_FLAGS_MASK);

    if (mask & kTSI_EndOfScanFlag)
    {
        regValue |= TSI_GENCS_EOSF_MASK;
    }
    if (mask & kTSI_OutOfRangeFlag)
    {
        regValue |= TSI_GENCS_OUTRGF_MASK;
    }

    base->GENCS = regValue; /* write value to register */
}

/*
 * The Clear BSD License
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright (c) 2016, NXP
 * All rights reserved.
 *
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
 * o Neither the name of copyright holder nor the names of its
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

#include "fsl_spm.h"
#include "math.h" /* Using floor() function to convert float variable to int. */

/* Component ID definition, used by tools. */
#ifndef FSL_COMPONENT_ID
#define FSL_COMPONENT_ID "platform.drivers.spm"
#endif


void SPM_GetRegulatorStatus(SPM_Type *base, spm_regulator_status_t *info)
{
    assert(info);

    volatile uint32_t tmp32 = base->RSR; /* volatile here is to make sure this value is actually from the hardware. */

    info->isRadioRunForcePowerModeOn = (SPM_RSR_RFRUNFORCE_MASK == (tmp32 & SPM_RSR_RFRUNFORCE_MASK));
    info->radioLowPowerModeStatus =
        (spm_radio_low_power_mode_status_t)((tmp32 & SPM_RSR_RFPMSTAT_MASK) >> SPM_RSR_RFPMSTAT_SHIFT);
    info->mcuLowPowerModeStatus =
        (spm_mcu_low_power_mode_status_t)((tmp32 & SPM_RSR_MCUPMSTAT_MASK) >> SPM_RSR_MCUPMSTAT_SHIFT);
    info->isDcdcLdoOn =
        (0x4 == (0x4 & ((tmp32 & SPM_RSR_REGSEL_MASK) >> SPM_RSR_REGSEL_SHIFT))); /* 1<<2 responses DCDC LDO. */
    info->isRfLdoOn =
        (0x2 == (0x2 & ((tmp32 & SPM_RSR_REGSEL_MASK) >> SPM_RSR_REGSEL_SHIFT))); /* 1<<1 responses RF LDO. */
    info->isCoreLdoOn =
        (0x1 == (0x1 & ((tmp32 & SPM_RSR_REGSEL_MASK) >> SPM_RSR_REGSEL_SHIFT))); /* 1<<0 responses CORE LDO. */
}

void SPM_SetLowVoltDetectConfig(SPM_Type *base, const spm_low_volt_detect_config_t *config)
{
    uint32_t tmp32 = base->LVDSC1 &
                     ~(SPM_LVDSC1_VDD_LVDIE_MASK | SPM_LVDSC1_VDD_LVDRE_MASK | SPM_LVDSC1_VDD_LVDV_MASK |
                       SPM_LVDSC1_COREVDD_LVDIE_MASK | SPM_LVDSC1_COREVDD_LVDRE_MASK);

    /* VDD voltage detection. */
    tmp32 |= SPM_LVDSC1_VDD_LVDV(config->vddLowVoltDetectSelect);
    if (config->enableIntOnVddLowVolt)
    {
        tmp32 |= SPM_LVDSC1_VDD_LVDIE_MASK;
    }
    if (config->enableResetOnVddLowVolt)
    {
        tmp32 |= SPM_LVDSC1_VDD_LVDRE_MASK;
    }
    /* Clear the Low Voltage Detect Flag with previouse power detect setting. */
    tmp32 |= SPM_LVDSC1_VDD_LVDACK_MASK;

    /* COREVDD voltage detection. */
    if (config->enableIntOnCoreLowVolt)
    {
        tmp32 |= SPM_LVDSC1_COREVDD_LVDIE_MASK;
    }
    if (config->enableResetOnCoreLowVolt)
    {
        tmp32 |= SPM_LVDSC1_COREVDD_LVDRE_MASK;
    }
    tmp32 |= SPM_LVDSC1_COREVDD_LVDACK_MASK; /* Clear previous error flag. */

    base->LVDSC1 = tmp32;
}

void SPM_SetLowVoltWarningConfig(SPM_Type *base, const spm_low_volt_warning_config_t *config)
{
    uint32_t tmp32 = base->LVDSC2 & ~(SPM_LVDSC2_VDD_LVWV_MASK | SPM_LVDSC2_VDD_LVWIE_MASK);

    tmp32 |= SPM_LVDSC2_VDD_LVWV(config->vddLowVoltDetectSelect);
    if (config->enableIntOnVddLowVolt)
    {
        tmp32 |= SPM_LVDSC2_VDD_LVWIE_MASK;
    }
    tmp32 |= SPM_LVDSC2_VDD_LVWACK_MASK; /* Clear previous error flag. */

    base->LVDSC2 = tmp32;
}

void SPM_SetHighVoltDetectConfig(SPM_Type *base, const spm_high_volt_detect_config_t *config)
{
    uint32_t tmp32;
    
    tmp32 = base->HVDSC1 & ~(SPM_HVDSC1_VDD_HVDIE_MASK | SPM_HVDSC1_VDD_HVDRE_MASK |\
                             SPM_HVDSC1_VDD_HVDV_MASK);
    tmp32 |= SPM_HVDSC1_VDD_HVDV(config->vddHighVoltDetectSelect);
    if(config->enableIntOnVddHighVolt)
    {
        tmp32 |= SPM_HVDSC1_VDD_HVDIE_MASK;
    }
    if(config->enableResetOnVddHighVolt)
    {
        tmp32 |= SPM_HVDSC1_VDD_HVDRE_MASK;
    }
    tmp32 |= SPM_HVDSC1_VDD_HVDACK_MASK; /* Clear previous error flag. */
    
    base->HVDSC1 = tmp32;
}

void SPM_SetRfLdoConfig(SPM_Type *base, const spm_rf_ldo_config_t *config)
{
    uint32_t tmp32 = 0U;

    switch (config->lowPowerMode)
    {
        case kSPM_RfLdoRemainInHighPowerInLowPowerModes:
            tmp32 |= SPM_RFLDOLPCNFG_LPSEL_MASK;
            break;
        default: /* kSPM_RfLdoEnterLowPowerInLowPowerModes. */
            break;
    }
    base->RFLDOLPCNFG = tmp32;

    tmp32 = SPM_RFLDOSC_IOSSSEL(config->softStartDuration) | SPM_RFLDOSC_IOREGVSEL(config->rfIoRegulatorVolt);
    if (config->enableCurSink)
    {
        tmp32 |= SPM_RFLDOSC_ISINKEN_MASK;
    }
    base->RFLDOSC = tmp32;
}

void SPM_SetDcdcBattMonitor(SPM_Type *base, uint32_t batAdcVal)
{
    /* Clear the value and disable it at first. */
    base->DCDCC2 &= ~(SPM_DCDCC2_DCDC_BATTMONITOR_BATT_VAL_MASK | SPM_DCDCC2_DCDC_BATTMONITOR_EN_BATADJ_MASK);
    if (0U != batAdcVal)
    {
        /* When setting the value to BATT_VAL field, it should be zero before. */
        base->DCDCC2 |= SPM_DCDCC2_DCDC_BATTMONITOR_BATT_VAL(batAdcVal);
        base->DCDCC2 |= SPM_DCDCC2_DCDC_BATTMONITOR_EN_BATADJ_MASK;
    }
}

void SPM_EnableVddxStepLock(SPM_Type *base, bool enable)
{
    if (enable)
    {
        base->DCDCC3 |= (SPM_DCDCC3_DCDC_VDD1P8CTRL_DISABLE_STEP_MASK | SPM_DCDCC3_DCDC_VDD1P2CTRL_DISABLE_STEP_MASK);
    }
    else
    {
        base->DCDCC3 &= ~(SPM_DCDCC3_DCDC_VDD1P8CTRL_DISABLE_STEP_MASK | SPM_DCDCC3_DCDC_VDD1P2CTRL_DISABLE_STEP_MASK);
    }
}

void SPM_BypassDcdcBattMonitor(SPM_Type *base, bool enable, uint32_t value)
{
    if (enable)
    {
        /* Set the user-defined value before enable the bypass. */
        base->DCDCC3 = (base->DCDCC3 & ~SPM_DCDCC3_DCDC_VBAT_VALUE_MASK) | SPM_DCDCC3_DCDC_VBAT_VALUE(value);
        /* Enable the bypass and load the user-defined value. */
        base->DCDCC3 |= SPM_DCDCC3_DCDC_BYPASS_ADC_MEAS_MASK;
    }
    else
    {
        base->DCDCC3 &= ~SPM_DCDCC3_DCDC_BYPASS_ADC_MEAS_MASK;
    }
}

void SPM_SetDcdcIntegratorConfig(SPM_Type *base, const spm_dcdc_integrator_config_t *config)
{
    int32_t tmp32u;

    if (NULL == config)
    {
        base->DCDCC4 = 0U;
    }
    else
    {
        tmp32u = (int32_t)(config->vdd1p2Value / config->vBatValue * 32 - 16) * 8192;
        base->DCDCC4 = SPM_DCDCC4_PULSE_RUN_SPEEDUP_MASK | SPM_DCDCC4_INTEGRATOR_VALUE_SELECT_MASK |
                       SPM_DCDCC4_INTEGRATOR_VALUE(tmp32u);
    }
}

void SPM_SetLowPowerReqOutPinConfig(SPM_Type *base, const spm_low_power_req_out_pin_config_t *config)
{
    if ((NULL == config) || (config->pinOutEnable))
    {
        base->LPREQPINCNTRL = 0U;
    }
    else
    {
        base->LPREQPINCNTRL =
            SPM_LPREQPINCNTRL_POLARITY(config->pinOutPol) | SPM_LPREQPINCNTRL_LPREQOE_MASK; /* Enable the output. */
    }
}

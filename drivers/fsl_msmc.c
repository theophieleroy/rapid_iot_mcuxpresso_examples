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

#include "fsl_msmc.h"

/* Component ID definition, used by tools. */
#ifndef FSL_COMPONENT_ID
#define FSL_COMPONENT_ID "platform.drivers.msmc"
#endif


status_t SMC_SetPowerModeRun(SMC_Type *base)
{
    uint32_t reg;

    reg = base->PMCTRL;
    /* configure Normal RUN mode */
    reg &= ~SMC_PMCTRL_RUNM_MASK;
    reg |= (kSMC_RunNormal << SMC_PMCTRL_RUNM_SHIFT);
    base->PMCTRL = reg;

    return kStatus_Success;
}

status_t SMC_SetPowerModeHsrun(SMC_Type *base)
{
    uint32_t reg;

    reg = base->PMCTRL;
    /* configure High Speed RUN mode */
    reg &= ~SMC_PMCTRL_RUNM_MASK;
    reg |= (kSMC_Hsrun << SMC_PMCTRL_RUNM_SHIFT);
    base->PMCTRL = reg;

    return kStatus_Success;
}

status_t SMC_SetPowerModeWait(SMC_Type *base)
{
    /* configure Normal Wait mode */
    SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;
    __DSB();
    __WFI();
    __ISB();

    return kStatus_Success;
}

status_t SMC_SetPowerModeStop(SMC_Type *base, smc_partial_stop_option_t option)
{
    uint32_t reg;

    /* configure the Partial Stop mode in Noraml Stop mode */
    reg = base->PMCTRL;
    reg &= ~(SMC_PMCTRL_PSTOPO_MASK | SMC_PMCTRL_STOPM_MASK);
    reg |= ((uint32_t)option << SMC_PMCTRL_PSTOPO_SHIFT) | (kSMC_StopNormal << SMC_PMCTRL_STOPM_SHIFT);
    base->PMCTRL = reg;

    /* Set the SLEEPDEEP bit to enable deep sleep mode (stop mode) */
    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;

    /* read back to make sure the configuration valid before entering stop mode */
    (void)base->PMCTRL;
    __DSB();
    __WFI();
    __ISB();

#if (defined(FSL_FEATURE_SMC_HAS_PMCTRL_STOPA) && FSL_FEATURE_SMC_HAS_PMCTRL_STOPA)
    /* check whether the power mode enter Stop mode succeed */
    if (base->PMCTRL & SMC_PMCTRL_STOPA_MASK)
    {
        return kStatus_SMC_StopAbort;
    }
    else
    {
        return kStatus_Success;
    }
#else
    return kStatus_Success;
#endif /* FSL_FEATURE_SMC_HAS_PMCTRL_STOPA */
}

status_t SMC_SetPowerModeVlpr(SMC_Type *base)
{
    uint32_t reg;

    reg = base->PMCTRL;
    /* configure VLPR mode */
    reg &= ~SMC_PMCTRL_RUNM_MASK;
    reg |= (kSMC_RunVlpr << SMC_PMCTRL_RUNM_SHIFT);
    base->PMCTRL = reg;

    return kStatus_Success;
}

status_t SMC_SetPowerModeVlpw(SMC_Type *base)
{
    /* configure VLPW mode */
    /* Clear the SLEEPDEEP bit to disable deep sleep mode */
    SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;
    __DSB();
    __WFI();
    __ISB();

    return kStatus_Success;
}

status_t SMC_SetPowerModeVlps(SMC_Type *base)
{
    uint32_t reg;

    /* configure VLPS mode */
    reg = base->PMCTRL;
    reg &= ~SMC_PMCTRL_STOPM_MASK;
    reg |= (kSMC_StopVlps << SMC_PMCTRL_STOPM_SHIFT);
    base->PMCTRL = reg;

    /* Set the SLEEPDEEP bit to enable deep sleep mode */
    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;

    /* read back to make sure the configuration valid before enter stop mode */
    (void)base->PMCTRL;
    __DSB();
    __WFI();
    __ISB();

#if (defined(FSL_FEATURE_SMC_HAS_PMCTRL_STOPA) && FSL_FEATURE_SMC_HAS_PMCTRL_STOPA)
    /* check whether the power mode enter Stop mode succeed */
    if (base->PMCTRL & SMC_PMCTRL_STOPA_MASK)
    {
        return kStatus_SMC_StopAbort;
    }
    else
    {
        return kStatus_Success;
    }
#else
    return kStatus_Success;
#endif /* FSL_FEATURE_SMC_HAS_PMCTRL_STOPA */
}

status_t SMC_SetPowerModeLls(SMC_Type *base)
{
    uint32_t reg;

    /* configure to LLS mode */
    reg = base->PMCTRL;
    reg &= ~SMC_PMCTRL_STOPM_MASK;
    reg |= (kSMC_StopLls << SMC_PMCTRL_STOPM_SHIFT);
    base->PMCTRL = reg;

    /* Set the SLEEPDEEP bit to enable deep sleep mode */
    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;

    /* read back to make sure the configuration valid before entering stop mode */
    (void)base->PMCTRL;
    __DSB();
    __WFI();
    __ISB();

#if (defined(FSL_FEATURE_SMC_HAS_PMCTRL_STOPA) && FSL_FEATURE_SMC_HAS_PMCTRL_STOPA)
    /* check whether the power mode enter Stop mode succeed */
    if (base->PMCTRL & SMC_PMCTRL_STOPA_MASK)
    {
        return kStatus_SMC_StopAbort;
    }
    else
    {
        return kStatus_Success;
    }
#else
    return kStatus_Success;
#endif /* FSL_FEATURE_SMC_HAS_PMCTRL_STOPA */
}

#if (defined(FSL_FEATURE_SMC_HAS_SUB_STOP_MODE) && FSL_FEATURE_SMC_HAS_SUB_STOP_MODE)

#if (defined(FSL_FEATURE_SMC_HAS_STOP_SUBMODE0) && FSL_FEATURE_SMC_HAS_STOP_SUBMODE0)
status_t SMC_SetPowerModeVlls0(SMC_Type *base)
{
    uint32_t reg;

    /* configure to VLLS mode */
    reg = base->PMCTRL;
    reg &= ~SMC_PMCTRL_STOPM_MASK;
    reg |= (kSMC_StopVlls0 << SMC_PMCTRL_STOPM_SHIFT);
    base->PMCTRL = reg;

    /* Set the SLEEPDEEP bit to enable deep sleep mode */
    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;

    /* read back to make sure the configuration valid before enter stop mode */
    (void)base->PMCTRL;
    __DSB();
    __WFI();
    __ISB();

    return kStatus_Success;
}
#endif /* FSL_FEATURE_SMC_HAS_STOP_SUBMODE0 */

#if (defined(FSL_FEATURE_SMC_HAS_STOP_SUBMODE2) && FSL_FEATURE_SMC_HAS_STOP_SUBMODE2)
status_t SMC_SetPowerModeVlls2(SMC_Type *base)
{
    uint32_t reg;

    /* configure to VLLS mode */
    reg = base->PMCTRL;
    reg &= ~SMC_PMCTRL_STOPM_MASK;
    reg |= (kSMC_StopVlls2 << SMC_PMCTRL_STOPM_SHIFT);
    base->PMCTRL = reg;

    /* Set the SLEEPDEEP bit to enable deep sleep mode */
    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;

    /* read back to make sure the configuration valid before enter stop mode */
    (void)base->PMCTRL;
    __DSB();
    __WFI();
    __ISB();

    return kStatus_Success;
}
#endif /* FSL_FEATURE_SMC_HAS_STOP_SUBMODE0 */

#else /* FSL_FEATURE_SMC_HAS_SUB_STOP_MODE */
status_t SMC_SetPowerModeVlls(SMC_Type *base)
{
    uint32_t reg;

    /* configure to VLLS mode */
    reg = base->PMCTRL;
    reg &= ~SMC_PMCTRL_STOPM_MASK;
    reg |= (kSMC_StopVlls << SMC_PMCTRL_STOPM_SHIFT);
    base->PMCTRL = reg;

    /* Set the SLEEPDEEP bit to enable deep sleep mode */
    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;

    /* read back to make sure the configuration valid before enter stop mode */
    (void)base->PMCTRL;
    __DSB();
    __WFI();
    __ISB();

#if (defined(FSL_FEATURE_SMC_HAS_PMCTRL_STOPA) && FSL_FEATURE_SMC_HAS_PMCTRL_STOPA)
    /* check whether the power mode enter Stop mode succeed */
    if (base->PMCTRL & SMC_PMCTRL_STOPA_MASK)
    {
        return kStatus_SMC_StopAbort;
    }
    else
    {
        return kStatus_Success;
    }
#else
    return kStatus_Success;
#endif /* FSL_FEATURE_SMC_HAS_PMCTRL_STOPA */
}
#endif /* FSL_FEATURE_SMC_HAS_SUB_STOP_MODE */

void SMC_ConfigureResetPinFilter(SMC_Type *base, const smc_reset_pin_filter_config_t *config)
{
    assert(config);

    uint32_t reg;

    reg = SMC_RPC_FILTCFG(config->slowClockFilterCount) | SMC_RPC_FILTEN(config->enableFilter);
#if (defined(FSL_FEATURE_SMC_HAS_RPC_LPOFEN) && FSL_FEATURE_SMC_HAS_RPC_LPOFEN)
    if (config->enableLpoFilter)
    {
        reg |= SMC_RPC_LPOFEN_MASK;
    }
#endif /* FSL_FEATURE_SMC_HAS_RPC_LPOFEN */

    base->RPC = reg;
}

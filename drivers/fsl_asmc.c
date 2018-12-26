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

#include "fsl_asmc.h"

/* Component ID definition, used by tools. */
#ifndef FSL_COMPONENT_ID
#define FSL_COMPONENT_ID "platform.drivers.asmc"
#endif

status_t ASMC_SetPowerModeRun(ASMC_Type *base)
{
    uint32_t reg;

    reg = base->PMCTRL;
    /* configure Normal RUN mode */
    reg &= ~ASMC_PMCTRL_RUNM_MASK;
    reg |= (kASMC_RunNormal << ASMC_PMCTRL_RUNM_SHIFT);
    base->PMCTRL = reg;

    return kStatus_Success;
}

#if (defined(FSL_FEATURE_ASMC_HAS_HIGH_SPEED_RUN_MODE) && FSL_FEATURE_ASMC_HAS_HIGH_SPEED_RUN_MODE)
status_t ASMC_SetPowerModeHsrun(ASMC_Type *base)
{
    uint32_t reg;

    reg = base->PMCTRL;
    /* configure High Speed RUN mode */
    reg &= ~ASMC_PMCTRL_RUNM_MASK;
    reg |= (kASMC_Hsrun << ASMC_PMCTRL_RUNM_SHIFT);
    base->PMCTRL = reg;

    return kStatus_Success;
}
#endif /* FSL_FEATURE_ASMC_HAS_HIGH_SPEED_RUN_MODE */

status_t ASMC_SetPowerModeWait(ASMC_Type *base)
{
    /* configure Normal Wait mode */
    SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;
    __DSB();
    __WFI();
    __ISB();

    return kStatus_Success;
}

status_t ASMC_SetPowerModeStop(ASMC_Type *base, asmc_partial_stop_option_t option)
{
    uint32_t reg;

    /* configure the Partial Stop mode in Noraml Stop mode */
    reg = base->STOPCTRL;
    reg &= ~ASMC_STOPCTRL_PSTOPO_MASK;
    reg |= ((uint32_t)option << ASMC_STOPCTRL_PSTOPO_SHIFT);
    base->STOPCTRL = reg;

    /* configure Normal Stop mode */
    reg = base->PMCTRL;
    reg &= ~ASMC_PMCTRL_STOPM_MASK;
    reg |= (kASMC_StopNormal << ASMC_PMCTRL_STOPM_SHIFT);
    base->PMCTRL = reg;

    /* Set the SLEEPDEEP bit to enable deep sleep mode (stop mode) */
    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;

    /* read back to make sure the configuration valid before entering stop mode */
    (void)base->PMCTRL;
    __DSB();
    __WFI();
    __ISB();

    return kStatus_Success;
}

status_t ASMC_SetPowerModeVlpr(ASMC_Type *base)
{
    uint32_t reg;

    reg = base->PMCTRL;

    /* configure VLPR mode */
    reg &= ~ASMC_PMCTRL_RUNM_MASK;
    reg |= (kASMC_RunVlpr << ASMC_PMCTRL_RUNM_SHIFT);
    base->PMCTRL = reg;

    return kStatus_Success;
}

status_t ASMC_SetPowerModeVlpw(ASMC_Type *base)
{
    /* configure VLPW mode */
    /* Clear the SLEEPDEEP bit to disable deep sleep mode */
    SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;
    __DSB();
    __WFI();
    __ISB();

    return kStatus_Success;
}

status_t ASMC_SetPowerModeVlps(ASMC_Type *base)
{
    uint32_t reg;

    /* configure VLPS mode */
    reg = base->PMCTRL;
    reg &= ~ASMC_PMCTRL_STOPM_MASK;
    reg |= (kASMC_StopVlps << ASMC_PMCTRL_STOPM_SHIFT);
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

status_t ASMC_SetPowerModeLls(ASMC_Type *base)
{
    uint32_t reg;

    /* configure to LLS mode */
    reg = base->PMCTRL;
    reg &= ~ASMC_PMCTRL_STOPM_MASK;
    reg |= (kASMC_StopLls << ASMC_PMCTRL_STOPM_SHIFT);
    base->PMCTRL = reg;

    /* Set the SLEEPDEEP bit to enable deep sleep mode */
    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;

    /* read back to make sure the configuration valid before entering stop mode */
    (void)base->PMCTRL;
    __DSB();
    __WFI();
    __ISB();

    return kStatus_Success;
}

status_t ASMC_SetPowerModeVlls(ASMC_Type *base)
{
    uint32_t reg;

    /* configure to VLLS mode */
    reg = base->PMCTRL;
    reg &= ~ASMC_PMCTRL_STOPM_MASK;
    reg |= (kASMC_StopVlls << ASMC_PMCTRL_STOPM_SHIFT);
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

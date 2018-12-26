/*
 * The Clear BSD License
 * Copyright (c) 2017, NXP
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

#include "fsl_gpc.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/* Component ID definition, used by tools. */
#ifndef FSL_COMPONENT_ID
#define FSL_COMPONENT_ID "platform.drivers.gpc_2"
#endif


/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/
void GPC_Init(GPC_Type *base, uint32_t powerUpSlot, uint32_t powerDownSlot)
{
    assert(powerUpSlot < GPC_PCG_TIME_SLOT_TOTAL_NUMBER);
    assert(powerDownSlot < GPC_PCG_TIME_SLOT_TOTAL_NUMBER);

    /* Disable all M4 interrupt */
    base->IMR_M4[0U] = GPC_IMR_M4_IMR1_M4_MASK;
    base->IMR_M4[1U] = GPC_IMR_M4_IMR2_M4_MASK;
    base->IMR_M4[2U] = GPC_IMR_M4_IMR3_M4_MASK;
    base->IMR_M4[3U] = GPC_IMR_M4_IMR4_M4_MASK;
    /* not mask M4 power down request */
    base->MISC |= GPC_MISC_M4_PDN_REQ_MASK_MASK;
    /* select virtual PGC ack for M4 */
    base->PGC_ACK_SEL_M4 |= kGPC_VirtualPGCPowerUpAck | kGPC_VirtualPGCPowerDownAck;
    /* slot configurations */
    base->SLT_CFG_PU[powerDownSlot] |=
        GPC_SLT_CFG_PU_M4_PDN_SLOT_CONTROL_MASK | GPC_SLT_CFG_PU_MF_PDN_SLOT_CONTROL_MASK;
    base->SLT_CFG_PU[powerUpSlot] |= GPC_SLT_CFG_PU_M4_PUP_SLOT_CONTROL_MASK | GPC_SLT_CFG_PU_MF_PUP_SLOT_CONTROL_MASK;
    /* mapping M4 PGC power up/down slot to fastmix/megamix PGC power up/down slow */
    base->PGC_CPU_0_1_MAPPING = GPC_PGC_CPU_0_1_MAPPING_MF_M4_DOMAIN_MASK;
}

void GPC_EnableIRQ(GPC_Type *base, uint32_t irqId)
{
    uint32_t irqRegNum = irqId / 32U;
    uint32_t irqRegShiftNum = irqId % 32U;

    assert(irqRegNum <= GPC_IMR_M4_COUNT);

    base->IMR_M4[irqRegNum] &= ~(1U << irqRegShiftNum);
}

void GPC_DisableIRQ(GPC_Type *base, uint32_t irqId)
{
    uint32_t irqRegNum = irqId / 32U;
    uint32_t irqRegShiftNum = irqId % 32U;

    assert(irqRegNum <= GPC_IMR_M4_COUNT);

    base->IMR_M4[irqRegNum] |= (1U << irqRegShiftNum);
}

bool GPC_GetIRQStatusFlag(GPC_Type *base, uint32_t irqId)
{
    uint32_t isrRegNum = irqId / 32U;
    uint32_t isrRegShiftNum = irqId % 32U;
    uint32_t ret;

    assert(isrRegNum <= GPC_IMR_M4_COUNT);

    ret = base->ISR_M4[isrRegNum] & (1U << isrRegShiftNum);

    return (1U << isrRegShiftNum) == ret;
}

void GPC_EnterWaitMode(GPC_Type *base, gpc_lpm_config_t *config)
{
    uint32_t lpcr = (base->LPCR_M4) & (~GPC_LPCR_M4_LPM0_MASK);
    uint32_t slpcr = base->SLPCR;

    if (config != NULL)
    {
        lpcr &= ~(GPC_LPCR_M4_CPU_CLK_ON_LPM_MASK | GPC_LPCR_M4_EN_M4_PUP_MASK | GPC_LPCR_M4_EN_M4_PDN_MASK);
        lpcr |= (config->enCpuClk ? GPC_LPCR_M4_CPU_CLK_ON_LPM_MASK : 0U) |
                (config->enVirtualPGCPowerup ? GPC_LPCR_M4_EN_M4_PUP_MASK : 0U) |
                (config->enVirtualPGCPowerdown ? GPC_LPCR_M4_EN_M4_PDN_MASK : 0U);
        slpcr &= ~GPC_SLPCR_EN_M4_FASTWUP_WAIT_MODE_MASK;
        slpcr |= config->enFastWakeUp ? GPC_SLPCR_EN_M4_FASTWUP_WAIT_MODE_MASK : 0U;
    }

    base->SLPCR = slpcr;
    /* WAIT mode */
    base->LPCR_M4 = lpcr | kGPC_WaitMode;
}

void GPC_EnterStopMode(GPC_Type *base, gpc_lpm_config_t *config)
{
    uint32_t lpcr = (base->LPCR_M4) & (~GPC_LPCR_M4_LPM0_MASK);
    uint32_t slpcr = base->SLPCR;

    if (config != NULL)
    {
        lpcr &= ~(GPC_LPCR_M4_CPU_CLK_ON_LPM_MASK | GPC_LPCR_M4_EN_M4_PUP_MASK | GPC_LPCR_M4_EN_M4_PDN_MASK);
        lpcr |= (config->enCpuClk ? GPC_LPCR_M4_CPU_CLK_ON_LPM_MASK : 0U) |
                (config->enVirtualPGCPowerup ? GPC_LPCR_M4_EN_M4_PUP_MASK : 0U) |
                (config->enVirtualPGCPowerdown ? GPC_LPCR_M4_EN_M4_PDN_MASK : 0U);
        slpcr &= ~GPC_SLPCR_EN_M4_FASTWUP_STOP_MODE_MASK;
        slpcr |= config->enFastWakeUp ? GPC_SLPCR_EN_M4_FASTWUP_STOP_MODE_MASK : 0U;
    }

    base->SLPCR = slpcr;
    /* STOP mode */
    base->LPCR_M4 = lpcr | kGPC_StopMode;
}

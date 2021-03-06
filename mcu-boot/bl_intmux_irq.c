/*
 * The Clear BSD License
 * Copyright (c) 2014-2015, Freescale Semiconductor, Inc.
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

#include "fsl_device_registers.h"
#include <stdbool.h>

/*******************************************************************************
 * Code
 ******************************************************************************/

static INTMUX_Type *g_IntmuxBases[] = INTMUX_BASE_PTRS;

static inline bool INTMUX_IsChannelPending(uint32_t instance, uint32_t channel)
{
    return (g_IntmuxBases[instance]->CHANNEL[channel].CHn_IPR_31_0);
}

static inline uint32_t INTMUX_getVectorNumber(uint32_t instance, uint32_t channel)
{
    return (g_IntmuxBases[instance]->CHANNEL[channel].CHn_VEC);
}

static void INTMUX_irq_handler(uint32_t instance, uint32_t channel)
{
    if (INTMUX_IsChannelPending(instance, channel))
    {
        uint32_t vectorNumber = INTMUX_getVectorNumber(instance, channel);
        if (vectorNumber)
        {
            uint32_t *vectorTable = (uint32_t *)(SCB->VTOR + vectorNumber);
            uint32_t fnAddress = *vectorTable;
            void (*activeVectorIsr)(void) = (void (*)(void))fnAddress;
            activeVectorIsr();
        }
    }
}

void INTMUX0_0_IRQHandler(void)
{
    INTMUX_irq_handler(0, 0);
}

void INTMUX0_1_IRQHandler(void)
{
    INTMUX_irq_handler(0, 1);
}

void INTMUX0_2_IRQHandler(void)
{
    INTMUX_irq_handler(0, 2);
}

void INTMUX0_3_IRQHandler(void)
{
    INTMUX_irq_handler(0, 3);
}

#if (FSL_FEATURE_SOC_INTMUX_COUNT > 1)
void INTMUX1_0_IRQHandler(void)
{
    INTMUX_irq_handler(1, 0);
}

void INTMUX1_1_IRQHandler(void)
{
    INTMUX_irq_handler(1, 1);
}

void INTMUX1_2_IRQHandler(void)
{
    INTMUX_irq_handler(1, 2);
}

void INTMUX1_3_IRQHandler(void)
{
    INTMUX_irq_handler(1, 3);
}
#endif // (FSL_FEATURE_SOC_INTMUX_COUNT > 1)

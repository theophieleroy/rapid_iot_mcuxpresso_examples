/*
 * The Clear BSD License
 * Copyright 2017 NXP
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

#include "fsl_wdog8.h"

/* Component ID definition, used by tools. */
#ifndef FSL_COMPONENT_ID
#define FSL_COMPONENT_ID "platform.drivers.wdog8"
#endif


/*******************************************************************************
 * Code
 ******************************************************************************/

void WDOG8_ClearStatusFlags(WDOG_Type *base, uint8_t mask)
{
    if (mask & kWDOG8_InterruptFlag)
    {
        base->CS2 |= WDOG_CS2_FLG_MASK;
    }
}

void WDOG8_GetDefaultConfig(wdog8_config_t *config)
{
    assert(config);

    config->enableWdog8 = true;
    config->clockSource = kWDOG8_ClockSource1;
    config->prescaler = kWDOG8_ClockPrescalerDivide1;
    config->workMode.enableWait = true;
    config->workMode.enableStop = false;
    config->workMode.enableDebug = false;
    config->testMode = kWDOG8_TestModeDisabled;
    config->enableUpdate = true;
    config->enableInterrupt = false;
    config->enableWindowMode = false;
    config->windowValue = 0U;
    config->timeoutValue = 0xFFFFU;
}

void WDOG8_Init(WDOG_Type *base, const wdog8_config_t *config)
{
    assert(config);

    uint8_t value1 = 0U;
    uint8_t value2 = 0U;
    uint32_t primaskValue = 0U;

    value1 = WDOG_CS1_EN(config->enableWdog8) | WDOG_CS1_INT(config->enableInterrupt) |
             WDOG_CS1_UPDATE(config->enableUpdate) | WDOG_CS1_DBG(config->workMode.enableDebug) |
             WDOG_CS1_STOP(config->workMode.enableStop) | WDOG_CS1_WAIT(config->workMode.enableWait) |
             WDOG_CS1_TST(config->testMode);
    value2 = WDOG_CS2_CLK(config->clockSource) | WDOG_CS2_WIN(config->enableWindowMode) |
             WDOG_CS2_PRES(config->prescaler);

    /* Disable the global interrupts. Otherwise, an interrupt could effectively invalidate the unlock sequence
     * and the WCT may expire. After the configuration finishes, re-enable the global interrupts. */
    primaskValue = DisableGlobalIRQ();
    WDOG8_Unlock(base);
    WDOG8_SetWindowValue(base, config->windowValue);
    WDOG8_SetTimeoutValue(base, config->timeoutValue);
    base->CS1 = value1;
    base->CS2 = value2;
    EnableGlobalIRQ(primaskValue);
}

void WDOG8_Deinit(WDOG_Type *base)
{
    uint32_t primaskValue = 0U;

    /* Disable the global interrupts */
    primaskValue = DisableGlobalIRQ();
    WDOG8_Unlock(base);
    WDOG8_Disable(base);
    EnableGlobalIRQ(primaskValue);
}

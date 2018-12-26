/*
 * The Clear BSD License
 * Copyright (c) 2016, NXP Semiconductors, Inc.
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

#include "fsl_timer.h"
#include "board.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define LPTMR_MAX_COUNT_USED 0xFFFFU

/*******************************************************************************
 * Variables
 ******************************************************************************/
static volatile uint64_t lptmrCounter = 0;

/*******************************************************************************
 * Code
 ******************************************************************************/
/* LPTMR interrupt handler */
void LPTMR0_IRQHandler(void)
{
    LPTMR_ClearStatusFlags(LPTMR0, kLPTMR_TimerCompareFlag);
    lptmrCounter++;
    LPTMR0B_IRQHandler();
}

/* Initialize the LPTMR for counting data */
void Timer_Init(void)
{
    lptmr_config_t lptmrConfig;
    uint32_t clkFrq = CLOCK_GetFreq(kCLOCK_LpoClk);

    LPTMR_GetDefaultConfig(&lptmrConfig);
    /* Initialize the LPTMR */
    LPTMR_Init(LPTMR0, &lptmrConfig);

    /* Set timer period to 1000000 us = 1s */
    LPTMR_SetTimerPeriod(LPTMR0, USEC_TO_COUNT(1000 * 1000, clkFrq));
    /* Enable timer interrupt */
    LPTMR_EnableInterrupts(LPTMR0, kLPTMR_TimerInterruptEnable);
    /* Start counting */
    LPTMR_StartTimer(LPTMR0);

    /* Enable the NVIC */
    EnableIRQ(LPTMR0_IRQn);
}

/* Millisecond delay function */
void Timer_Delay_ms(uint32_t timeValue)
{
    uint32_t ts = Timer_GetTime() + timeValue;

    while (ts > Timer_GetTime())
    {
        __NOP();
    }
}

uint32_t Timer_GetTime(void)
{
    uint64_t count;
    uint32_t freq = CLOCK_GetFreq(kCLOCK_LpoClk);

    count = LPTMR_GetCurrentTimerCount(LPTMR0);
    count += lptmrCounter * USEC_TO_COUNT(1000 * 1000, freq);

    return COUNT_TO_MSEC(count, freq);
}

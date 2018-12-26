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

#include "fsl_irqsteer.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/* Component ID definition, used by tools. */
#ifndef FSL_COMPONENT_ID
#define FSL_COMPONENT_ID "platform.drivers.irqsteer"
#endif

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*!
 * @brief Get instance number for IRQSTEER.
 *
 * @param base IRQSTEER peripheral base address.
 */
static uint32_t IRQSTEER_GetInstance(IRQSTEER_Type *base);

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*! @brief Array to map IRQSTEER instance number to base pointer. */
static IRQSTEER_Type *const s_irqsteerBases[] = IRQSTEER_BASE_PTRS;

#if !(defined(FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL) && FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL)
/*! @brief Array to map IRQSTEER instance number to clock name. */
static const clock_ip_name_t s_irqsteerClockName[] = IRQSTEER_CLOCKS;
#endif

/*! @brief Array to map IRQSTEER instance number to IRQ number. */
static const IRQn_Type s_irqsteerIRQNumber[] = IRQSTEER_IRQS;

/*******************************************************************************
 * Code
 ******************************************************************************/

static uint32_t IRQSTEER_GetInstance(IRQSTEER_Type *base)
{
    uint32_t instance;

    /* Find the instance index from base address mappings. */
    for (instance = 0; instance < ARRAY_SIZE(s_irqsteerBases); instance++)
    {
        if (s_irqsteerBases[instance] == base)
        {
            break;
        }
    }

    assert(instance < ARRAY_SIZE(s_irqsteerBases));

    return instance;
}

void IRQSTEER_Init(IRQSTEER_Type *base)
{
    uint32_t i;
#if !(defined(FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL) && FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL)
    /* Enable clock. */
    CLOCK_EnableClock(s_irqsteerClockName[IRQSTEER_GetInstance(base)]);
#endif
    /* Mask all interrupts. */
    for (i = 0; i < FSL_FEATURE_IRQSTEER_CHn_MASK_COUNT; i++)
    {
        base->CHn_MASK[i] &= ~IRQSTEER_CHn_MASK_MASKFLD_MASK;
    }
    /* Enable NVIC vectors for all IRQSTEER master. */
    for (i = 0; i < FSL_FEATURE_IRQSTEER_MASTER_COUNT; i++)
    {
        EnableIRQ(s_irqsteerIRQNumber[i]);
    }
}

void IRQSTEER_Deinit(IRQSTEER_Type *base)
{
    uint32_t master;
#if !(defined(FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL) && FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL)
    /* Disable clock. */
    CLOCK_DisableClock(s_irqsteerClockName[IRQSTEER_GetInstance(base)]);
#endif
    /* Disable NVIC vectors for all IRQSTEER master. */
    for (master = 0; master < FSL_FEATURE_IRQSTEER_MASTER_COUNT; master++)
    {
        DisableIRQ(s_irqsteerIRQNumber[master]);
    }
}

IRQn_Type IRQSTEER_GetMasterNextInterrupt(IRQSTEER_Type *base, irqsteer_int_master_t intMasterIndex)
{
    uint32_t regIndex = FSL_FEATURE_IRQSTEER_CHn_MASK_COUNT - 1U - intMasterIndex * 2U;
    uint32_t bitOffset;

    bitOffset = __CLZ(__RBIT(base->CHn_STATUS[regIndex]));
    /* When no result found, continue the loop to parse the next CHn_STATUS register. */
    if (IRQSTEER_INT_SRC_REG_WIDTH == bitOffset)
    {
        regIndex--;
        bitOffset = __CLZ(__RBIT(base->CHn_STATUS[regIndex]));
    }

    if (IRQSTEER_INT_SRC_REG_WIDTH == bitOffset)
    {
        return NotAvail_IRQn;
    }
    else
    {
        return (IRQn_Type)(IRQSTEER_INT_SRC_NUM(regIndex, bitOffset) + FSL_FEATURE_IRQSTEER_IRQ_START_INDEX);
    }
}

static void IRQSTEER_CommonIRQHandler(IRQSTEER_Type *base, irqsteer_int_master_t intMasterIndex)
{
    IRQn_Type intSource;
    uint32_t isr;

    intSource = IRQSTEER_GetMasterNextInterrupt(base, intMasterIndex);
    if (NotAvail_IRQn != intSource)
    {
        isr = *(uint32_t *)(SCB->VTOR + ((uint32_t)(intSource + 16) << 2U));

        ((void (*)(void))isr)();
    }
#if defined(FSL_FEATURE_EDMA_IRQSTEER_INTERRUPT_PATCH)
    /*This is used for ESAI Interrupt IRQSTEER triggering*/
    else
    {
        if ((intMasterIndex == 6) && ((base->CHn_MASK[2]) & 0x10))
        {
            isr = *(uint32_t *)(SCB->VTOR + ((ADMA_ESAI0_INT_IRQn + 16) << 2U));
            ((void (*)(void))isr)();
        }
    }
#endif
/* Add for ARM errata 838869, affects Cortex-M4, Cortex-M4F Store immediate overlapping
  exception return operation might vector to incorrect interrupt */
#if defined __CORTEX_M && (__CORTEX_M == 4U)
    __DSB();
#endif
}

void IRQSTEER_0_DriverIRQHandler(void)
{
    IRQSTEER_CommonIRQHandler(IRQSTEER, kIRQSTEER_InterruptMaster0);
}

void IRQSTEER_1_DriverIRQHandler(void)
{
    IRQSTEER_CommonIRQHandler(IRQSTEER, kIRQSTEER_InterruptMaster1);
}

void IRQSTEER_2_DriverIRQHandler(void)
{
    IRQSTEER_CommonIRQHandler(IRQSTEER, kIRQSTEER_InterruptMaster2);
}

void IRQSTEER_3_DriverIRQHandler(void)
{
    IRQSTEER_CommonIRQHandler(IRQSTEER, kIRQSTEER_InterruptMaster3);
}

void IRQSTEER_4_DriverIRQHandler(void)
{
    IRQSTEER_CommonIRQHandler(IRQSTEER, kIRQSTEER_InterruptMaster4);
}

void IRQSTEER_5_DriverIRQHandler(void)
{
    IRQSTEER_CommonIRQHandler(IRQSTEER, kIRQSTEER_InterruptMaster5);
}

void IRQSTEER_6_DriverIRQHandler(void)
{
    IRQSTEER_CommonIRQHandler(IRQSTEER, kIRQSTEER_InterruptMaster6);
}

void IRQSTEER_7_DriverIRQHandler(void)
{
    IRQSTEER_CommonIRQHandler(IRQSTEER, kIRQSTEER_InterruptMaster7);
}

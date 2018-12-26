/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2018 NXP
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
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

#include "fsl_cs.h"
#include "fsl_power.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
/*!
 * @brief Gets the instance from the base address
 *
 * @param base CS peripheral base address
 *
 * @return The CS instance
 */
static uint32_t CS_GetInstance(CS_Type *base);

/*!
 * @brief CS active mode configure
 *
 * @param base CS peripheral base address
 * @param config Pointer to configuration structure, see to cs_config_t.
 */
void CS_ActiveModeConfig(CS_Type *base, const cs_config_t *config);

/*!
 * @brief CS low power mode configure
 *
 * @param base CS peripheral base address
 * @param config Pointer to configuration structure, see to cs_config_t.
 */
void CS_LowPowerModeConfig(CS_Type *base, const cs_config_t *config);

/*******************************************************************************
 * Variables
 ******************************************************************************/
/*! @brief Pointers to CS bases for each instance. */
static CS_Type *const s_csBases[] = CS_BASE_PTRS;

#if !(defined(FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL) && FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL)
static const clock_ip_name_t s_csClocks[] = CS_CLOCKS;
#endif /* FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL */

#if !(defined(FSL_SDK_DISABLE_DRIVER_RESET_CONTROL) && FSL_SDK_DISABLE_DRIVER_RESET_CONTROL)
/*! @brief Pointers to CS resets for each instance. */
static const reset_ip_name_t s_csResets[] = CS_RSTS;
#endif /* FSL_SDK_DISABLE_DRIVER_RESET_CONTROL */

uint32_t cs_inten = 0;

/*******************************************************************************
 * Code
 ******************************************************************************/
static uint32_t CS_GetInstance(CS_Type *base)
{
    uint32_t instance;

    /* Find the instance index from base address mappings. */
    for (instance = 0; instance < FSL_FEATURE_SOC_CS_COUNT; instance++)
    {
        if (s_csBases[instance] == base)
        {
            break;
        }
    }

    assert(instance < FSL_FEATURE_SOC_CS_COUNT);

    return instance;
}

void CS_ActiveModeConfig(CS_Type *base, const cs_config_t *config)
{
    uint32_t mask, tmp32;

    /* Configure clock divider and OSC frequency */
    mask = CS_CTRL0_CLK_DIV_MASK | CS_CTRL0_OSC_FREQ_MASK;
    tmp32 = CS_CTRL0_CLK_DIV(config->activeClockDivider) | CS_CTRL0_OSC_FREQ(config->activeOscFreq);
    base->CTRL0 = (base->CTRL0 & ~mask) | tmp32;

    /* Configure period */
    mask = CS_CTRL1_PERIOD_MASK;
    tmp32 = CS_CTRL1_PERIOD(config->activeDetectPeriod);
    base->CTRL1 = (base->CTRL1 & ~mask) | tmp32;

    /* Configure channel enable */
    mask = CS_CTRL1_CH_MASK;
    tmp32 = CS_CTRL1_CH(config->activeChannels);
    base->CTRL1 = (base->CTRL1 & ~mask) | tmp32;

    /* Configure the idle period */
    base->IDLE_PERIOD =
        (base->IDLE_PERIOD & ~CS_IDLE_PERIOD_IDLE_PERIOD_MASK) | CS_IDLE_PERIOD_IDLE_PERIOD(config->activeIdlePeriod);

    /* Disable sleep interrupt */
    base->LP_INTEN &= ~CS_LP_INTEN_LP_INTEN_MASK;
    /* Disable CS low power */
    base->LP_CTRL &= ~CS_LP_CTRL_LP_EN_MASK;
}

void CS_LowPowerModeConfig(CS_Type *base, const cs_config_t *config)
{
    uint32_t mask, tmp32;

    base->CTRL0 = (base->CTRL0 & ~CS_CTRL0_OSC_FREQ_MASK) | CS_CTRL0_OSC_FREQ(config->lowPowerOscFreq);

    mask = CS_LP_CTRL_DEBONCE_NUM_MASK | CS_LP_CTRL_LP_CH_MASK | CS_LP_CTRL_THR_MASK;
    tmp32 = CS_LP_CTRL_DEBONCE_NUM(config->lowPowerDebonceNum) | CS_LP_CTRL_LP_CH(config->lowPowerChannelNum) |
            CS_LP_CTRL_THR(config->lowPowerThreshold);
    base->LP_CTRL = (base->LP_CTRL & ~mask) | tmp32;

    /* Configure the idle period */
    base->IDLE_PERIOD =
        (base->IDLE_PERIOD & ~CS_IDLE_PERIOD_IDLE_PERIOD_MASK) | CS_IDLE_PERIOD_IDLE_PERIOD(config->lowPowerIdlePeriod);

    /* Enable sleep interrupt */
    base->LP_INTEN |= CS_LP_INTEN_LP_INTEN_MASK;
    /* Enable CS low power */
    base->LP_CTRL |= CS_LP_CTRL_LP_EN_MASK;
}

void CS_Init(CS_Type *base)
{
    POWER_WritePmuCtrl1(SYSCON, SYSCON_PMU_CTRL1_CAP_SEN_DIS_MASK, SYSCON_PMU_CTRL1_CAP_SEN_DIS(0));

#if !(defined(FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL) && FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL)
    /* Enable the clock. */
    CLOCK_EnableClock(s_csClocks[CS_GetInstance(base)]);
#endif /* FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL */

#if !(defined(FSL_SDK_DISABLE_DRIVER_RESET_CONTROL) && FSL_SDK_DISABLE_DRIVER_RESET_CONTROL)
    /* Reset the module. */
    RESET_PeripheralReset(s_csResets[CS_GetInstance(base)]);
#endif /* FSL_SDK_DISABLE_DRIVER_RESET_CONTROL */
}

void CS_DeInit(CS_Type *base)
{
#if !(defined(FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL) && FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL)
    /* Disable the clock. */
    CLOCK_DisableClock(s_csClocks[CS_GetInstance(base)]);
#endif /* FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL */
}

void CS_Config(CS_Type *base, const cs_config_t *config, cs_mode_t mode)
{
    if (mode == kCS_ActiveMode)
    {
        CS_ActiveModeConfig(base, config);
    }
    else
    {
        CS_LowPowerModeConfig(base, config);
    }
}

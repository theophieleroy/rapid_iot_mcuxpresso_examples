/*
 * The Clear BSD License
 * Copyright 2017 NXP
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided
 * that the following conditions are met:
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

#include "fsl_ci_pi.h"

/* Component ID definition, used by tools. */
#ifndef FSL_COMPONENT_ID
#define FSL_COMPONENT_ID "platform.drivers.ci_pi"
#endif


/*******************************************************************************
 * Prototypes
 ******************************************************************************/
/*!
 * @brief Get the CI_PI instance from peripheral base address.
 *
 * @param base CI_PI peripheral base address.
 * @return CI_PI instance.
 */
uint32_t CI_PI_GetInstance(CI_PI_Type *base);

/*******************************************************************************
 * Variables
 ******************************************************************************/
/* Array of CI_PI peripheral base address. */
static CI_PI_Type *const s_cipiBases[] = CI_PI_BASE_PTRS;
#if !(defined(FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL) && FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL)
/* Array of CI_PI clock name. */
static const clock_ip_name_t s_cipiClock[] = CI_PI_CLOCKS;
#endif /* FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL */

/*******************************************************************************
 * Code
 ******************************************************************************/
uint32_t CI_PI_GetInstance(CI_PI_Type *base)
{
    uint32_t instance;
    uint32_t cipiArrayCount = (sizeof(s_cipiBases) / sizeof(s_cipiBases[0]));

    /* Find the instance index from base address mappings. */
    for (instance = 0; instance < cipiArrayCount; instance++)
    {
        if (s_cipiBases[instance] == base)
        {
            break;
        }
    }

    assert(instance < cipiArrayCount);

    return instance;
}

void CI_PI_Init(CI_PI_Type *base, const ci_pi_config_t *config)
{
    uint32_t ifCtrl = 0U;
    uint32_t csiCtrl0 = 0U;

#if !(defined(FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL) && FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL)
    /* Enable cipi clock */
    CLOCK_EnableClock(s_cipiClock[CI_PI_GetInstance(base)]);
#endif /* FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL */

    CI_PI_Reset(base);

    CI_PI_Stop(base);

    /* Configure the pixel link. */
    ifCtrl = CI_PI_IF_CTRL_IF_ADDR(config->pixelLinkAddr);

    base->IF_CTRL = ifCtrl;

    /* Enable the pixel link. */
    base->IF_CTRL_SET = CI_PI_IF_CTRL_IF_ENABLE_MASK | CI_PI_IF_CTRL_IF_VALID_MASK;

    /* Configure the CSI. */
    csiCtrl0 = CI_PI_CSI_CTRL0_HSYNC_FORCE_EN_MASK | CI_PI_CSI_CTRL0_VSYNC_FORCE_EN_MASK |
               CI_PI_CSI_CTRL0_CCIR_VSYNC_RESET_EN_MASK | config->polarityFlags /* Signal polarity. */
               | CI_PI_CSI_CTRL0_MASK_VSYNC_COUNTER(2)                          /* Mask 2 frames. */
               | CI_PI_CSI_CTRL0_CCIR_ECC_ERR_CORRECT_EN_MASK                   /* Enable ECC correction. */
               | CI_PI_CSI_CTRL0_HSYNC_PULSE(config->hsyncWidth)                /* Set HSYNC pulse. */
               | CI_PI_CSI_CTRL0_DATA_TYPE(config->inputFormat)                 /* CSI data format. */
               | config->workMode;                                              /* CSI work mode. */

    if (config->useExtVsync)
    {
        csiCtrl0 |= CI_PI_CSI_CTRL0_CCIR_EXT_VSYNC_EN_MASK;
    }

    if (config->swapUV)
    {
        csiCtrl0 |= CI_PI_CSI_CTRL0_UV_SWAP_EN_MASK;
    }

    base->CSI_CTRL0 = csiCtrl0;

    base->CSI_CTRL1 = CI_PI_CSI_CTRL1_VSYNC_PULSE(config->vsyncWidth) | CI_PI_CSI_CTRL1_PIXEL_WIDTH(config->width - 1);
}

void CI_PI_Deinit(CI_PI_Type *base)
{
    CI_PI_Stop(base);
    CI_PI_Reset(base);

#if !(defined(FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL) && FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL)
    /* Enable cipi clock */
    CLOCK_EnableClock(s_cipiClock[CI_PI_GetInstance(base)]);
#endif /* FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL */
}

void CI_PI_GetDefaultConfig(ci_pi_config_t *config)
{
    assert(config);

    config->width = 0;
    config->vsyncWidth = 3U;
    config->hsyncWidth = 2U;
    config->polarityFlags = 0;
    config->pixelLinkAddr = 0;
    config->inputFormat = kCI_PI_InputUYVY8888_8BitBus;
    config->workMode = kCI_PI_NonGatedClockMode;
    config->useExtVsync = false;
    config->swapUV = false;
}

void CI_PI_Start(CI_PI_Type *base)
{
    /* Enable the CSI. */
    base->CSI_CTRL0_SET = CI_PI_CSI_CTRL0_CSI_ENABLE_MASK;
    /* Disable force sync, then the input signal is processed. */
    base->CSI_CTRL0_CLR = CI_PI_CSI_CTRL0_HSYNC_FORCE_EN_MASK | CI_PI_CSI_CTRL0_VSYNC_FORCE_EN_MASK;
}

void CI_PI_Stop(CI_PI_Type *base)
{
    /* Disable the CSI. */
    base->CSI_CTRL0_CLR = CI_PI_CSI_CTRL0_CSI_ENABLE_MASK;
    /* Enable force sync, the signal is latched. */
    base->CSI_CTRL0_SET = CI_PI_CSI_CTRL0_HSYNC_FORCE_EN_MASK | CI_PI_CSI_CTRL0_VSYNC_FORCE_EN_MASK;
}

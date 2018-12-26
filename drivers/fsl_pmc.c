/*
 * The Clear BSD License
 * Copyright 2017 NXP
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
#include "fsl_pmc.h"

void PMC_ConfigureLowVoltDetect(PMC_Type *base, const pmc_low_volt_detect_config_t *config)
{
    uint32_t spmsc1 = base->SPMSC1;

    /* low voltage detect configuration */
    spmsc1 &= ~(PMC_SPMSC1_LVDSE_MASK | PMC_SPMSC1_LVDRE_MASK);
    spmsc1 |= ((uint32_t)config->enableStop << PMC_SPMSC1_LVDSE_SHIFT) |
              ((uint32_t)config->enableReset << PMC_SPMSC1_LVDRE_SHIFT);
    base->SPMSC1 = spmsc1;

    /* select detect voltage */
    base->SPMSC2 |= (base->SPMSC2 & (~PMC_SPMSC2_LVDV_MASK)) | ((uint32_t)config->voltSelect << PMC_SPMSC2_LVDV_SHIFT);
}

void PMC_ConfigureLowVoltWarning(PMC_Type *base, const pmc_low_volt_warning_config_t *config)
{
    base->SPMSC1 |= (base->SPMSC1 & (~PMC_SPMSC1_LVWIE_MASK)) |
                    ((uint32_t)config->enableInt << PMC_SPMSC1_LVWIE_SHIFT) | PMC_SPMSC1_LVWACK_MASK;

    /* select detect voltage */
    base->SPMSC2 |= (base->SPMSC2 & (~PMC_SPMSC2_LVWV_MASK)) | ((uint32_t)config->voltSelect << PMC_SPMSC2_LVWV_SHIFT);
}

/*
 * The Clear BSD License
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright (c) 2017, NXP
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

#include "fsl_ieer.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/* Component ID definition, used by tools. */
#ifndef FSL_COMPONENT_ID
#define FSL_COMPONENT_ID "platform.drivers.ieer"
#endif


/*******************************************************************************
 * Code
 ******************************************************************************/
void IEER_Init(IEER_Type *base)
{
    ieer_config_t ieerConfig;

    IEER_GetDefaultConfig(&ieerConfig);

    for (ieer_region_t region = kIEER_Region0; region <= kIEER_Region3; region++)
    {
        IEER_SetRegionConfig(base, region, &ieerConfig);
    }
}

void IEER_SetRegionConfig(IEER_Type *base, ieer_region_t region, ieer_config_t *config)
{
    base->MRGDJ[region].MRGD_W0 = IEER_MRGD_W0_SRTADDR(config->startaddr);
    base->MRGDJ[region].MRGD_W2 = IEER_MRGD_W2_ENDADDR(config->endaddr);
    base->MRGDJ[region].MRGD_W4 = IEER_MRGD_W4_VLD(config->valid) | IEER_MRGD_W4_RMSG(config->rmsg);
}

void IEER_GetDefaultConfig(ieer_config_t *config)
{
    static const ieer_config_t myConfig = {0, 0x00000FFFU, 0, false};

    *config = myConfig;
}

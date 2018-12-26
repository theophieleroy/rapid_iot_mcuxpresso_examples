/*
 * The Clear BSD License
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
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
#ifndef _FSL_TSTMR_H_
#define _FSL_TSTMR_H_

#include "fsl_common.h"

/*!
 * @addtogroup tstmr_driver
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/* Component ID definition, used by tools. */
#ifndef FSL_COMPONENT_ID
#define FSL_COMPONENT_ID "platform.drivers.tstmr"
#endif


/*! @name Driver version */
/*@{*/
#define FSL_TSTMR_DRIVER_VERSION (MAKE_VERSION(2, 0, 0)) /*!< Version 2.0.0 */
                                                         /*@}*/

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @brief Reads the time stamp.
 *
 * This function reads the low and high registers and returns the 56-bit free running
 * counter value. This can be read by software at any time to determine the software ticks.
 *
 * @param base TSTMR peripheral base address.
 *
 * @return The 56-bit time stamp value.
 */
static inline uint64_t TSTMR_ReadTimeStamp(TSTMR_Type* base)
{
    return *(volatile uint64_t*)(base);
}

/*!
 * @brief Delays for a specified number of microseconds.
 *
 * This function repeatedly reads the timestamp register and waits for the user-specified
 * delay value.
 *
 * @param base      TSTMR peripheral base address.
 * @param delayInUs Delay value in microseconds.
 */
static inline void TSTMR_DelayUs(TSTMR_Type* base, uint32_t delayInUs)
{
    uint64_t startTime = TSTMR_ReadTimeStamp(base);
#if defined(FSL_FEATURE_TSTMR_CLOCK_FREQUENCY_1MHZ) && FSL_FEATURE_TSTMR_CLOCK_FREQUENCY_1MHZ
    while (TSTMR_ReadTimeStamp(base) - startTime < delayInUs)
#elif defined(FSL_FEATURE_TSTMR_CLOCK_FREQUENCY_8MHZ) && FSL_FEATURE_TSTMR_CLOCK_FREQUENCY_8MHZ
    while (TSTMR_ReadTimeStamp(base) - startTime < 8 * delayInUs)
#else
    assert(0);
#endif
    {
    }
}

#if defined(__cplusplus)
}
#endif

/*! @}*/

#endif /* _FSL_TSTMR_H_ */

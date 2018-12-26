/*
 * The Clear BSD License
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
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
/*
 * @file microseconds_tstmr.c
 * @brief Microseconds tstmr timer driver source file
 *
 * Notes: The driver configure TSTMR as lifetime timer
 */
#include "microseconds/microseconds.h"
#include <stdarg.h>
#include "bootloader_common.h"

////////////////////////////////////////////////////////////////////////////////
// Definitions
////////////////////////////////////////////////////////////////////////////////

// Below MACROs are defined in order to keep this driver compabtile among all targets.
#if defined(TSTMR0_BASE)
#define TSTMR_BASE        TSTMR0_BASE
#define TSTMRx            TSTMR0
#elif defined(TSTMRA_BASE)
#define TSTMR_BASE        TSTMRA_BASE
#define TSTMRx            TSTMRA
#endif

#if defined(TSTMR_L_VALUE_MASK)
#define TSTMR_LOW_REG32   (TSTMRx->L)
#define TSTMR_HIGH_REG32  (TSTMRx->H)
#elif defined(TSTMR_LOW_VALUE_MASK)
#define TSTMR_LOW_REG32   (TSTMRx->LOW)
#define TSTMR_HIGH_REG32  (TSTMRx->HIGH)
#endif

enum
{
    kFrequency_1MHz = 1000000UL,
    kFrequency_8MHz = 8000000UL
};

////////////////////////////////////////////////////////////////////////////////
// Variables
////////////////////////////////////////////////////////////////////////////////
static uint32_t s_tstmrModuleClock;
uint32_t s_tickPerMicrosecond; //!< This value equal to ticks per microseconds

////////////////////////////////////////////////////////////////////////////////
// Code
////////////////////////////////////////////////////////////////////////////////

//! @brief Initialize timer facilities.
//!
//! Initialize and start the timer facilities using the TSTMR.
void microseconds_init(void)
{
#if defined(FSL_FEATURE_TSTMR_CLOCK_FREQUENCY_1MHZ) && FSL_FEATURE_TSTMR_CLOCK_FREQUENCY_1MHZ
    s_tstmrModuleClock = kFrequency_1MHz;
#elif defined(FSL_FEATURE_TSTMR_CLOCK_FREQUENCY_8MHZ) && FSL_FEATURE_TSTMR_CLOCK_FREQUENCY_8MHZ
    s_tstmrModuleClock = kFrequency_8MHz;
#else
#error "Unknown TSTMR clock frequency"
#endif

    s_tickPerMicrosecond = s_tstmrModuleClock / kFrequency_1MHz;

    // Make sure this value is greater than 0
    if (!s_tickPerMicrosecond)
    {
        s_tickPerMicrosecond = 1;
    }
}

//! @brief Shutdown the microsecond timer
void microseconds_shutdown(void)
{
}

//! @brief Read back running tick count
uint64_t microseconds_get_ticks(void)
{
    // Note: the software must follow the read sequence (must read the TSTMR_L first,
    //  followed by a read to TSTMR_H to retrieve the complete value.) for correctly reading the TSTMR value.
    uint32_t low = TSTMR_LOW_REG32;
    uint64_t high = TSTMR_HIGH_REG32;

    return ((high << 32) | low);
    //return *(volatile uint64_t*)(TSTMR_BASE);
}

//! @brief Returns the conversion of ticks to actual microseconds
//!        This is used to seperate any calculations from getting a timer
//         value for speed critical scenarios
uint32_t microseconds_convert_to_microseconds(uint32_t ticks)
{
    // return the total ticks divided by the number of Mhz the system clock is at to give microseconds
    return (ticks / s_tickPerMicrosecond); //!< Assumes system clock will never be < 0.125 Mhz
}

//! @brief Returns the conversion of microseconds to ticks
uint64_t microseconds_convert_to_ticks(uint32_t microseconds)
{
    return ((uint64_t)microseconds * s_tickPerMicrosecond);
}

//! @brief Delay specified time
//!
//! @param us Delay time in microseconds unit
void microseconds_delay(uint32_t us)
{
    uint64_t currentTicks = microseconds_get_ticks();

    //! The clock value in Mhz = ticks/microsecond
    uint64_t ticksNeeded = ((uint64_t)us * s_tickPerMicrosecond) + currentTicks;
    while (microseconds_get_ticks() < ticksNeeded)
    {
        ;
    }
}

//! @brief Gets the clock value used for microseconds driver
uint32_t microseconds_get_clock(void)
{
    return s_tstmrModuleClock;
}

////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////

/*
 * The Clear BSD License
 * Copyright (c) 2013, Freescale Semiconductor, Inc.
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
#include "autobaud/autobaud.h"
#include "microseconds.h"
#include "bootloader_common.h"

////////////////////////////////////////////////////////////////////////////////
// Definitions
////////////////////////////////////////////////////////////////////////////////

enum _autobaud_counts
{
    //! the number of edge transitions being counted
    kRequiredEdges = 14,
    //! the number of bits being measured for the baud rate
    //! for this sequence of kFramingPacketStartByte(0x5A) + kFramingPacketType_Ping(0xA8) there are
    //! 20 bits total however the final rising edge is 1 bit before the stop bit so the final two bits
    //! will not be measured since this is all edge transition based. Additionally we will not start the
    //! timer until after the first rising transition from the start bit which is 2 bytes into the sequence
    //! so there are a total of:
    //! 20 - 1 (stop bit) - 1 (final rising bit) - 2 (bits to first rising edge) = 16
    //! bits we are measuring the time for
    kNumberOfBitsMeasured = 16,
    //! 250 milliseconds = 250000 microseconds
    kAutobaudDetectDelay = 250000
};

////////////////////////////////////////////////////////////////////////////////
// Prototypes
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
// Variables
////////////////////////////////////////////////////////////////////////////////

#if defined(LPUART_INSTANCE_COUNT)
static uint32_t s_initialEdge[LPUART_INSTANCE_COUNT];
#elif(defined(UART_BASES) && (UART0_INSTANCE_COUNT == 1))
static uint32_t s_initialEdge[UART0_INSTANCE_COUNT + UART_INSTANCE_COUNT];
#elif defined(UART_BASES)
static uint32_t s_initialEdge[UART_INSTANCE_COUNT];
#else
static uint32_t s_initialEdge[UART0_INSTANCE_COUNT];
#endif // defined(LPUART_INSTANCE_COUNT)

////////////////////////////////////////////////////////////////////////////////
// Code
////////////////////////////////////////////////////////////////////////////////

void autobaud_init(uint32_t instance)
{
    s_initialEdge[instance] = read_autobaud_pin(instance);
}

status_t autobaud_get_rate(uint32_t instance, uint32_t *rate)
{
    uint32_t currentEdge = read_autobaud_pin(instance);

    if (currentEdge != s_initialEdge[instance])
    {
        // If we get a transition on a UART then one of the UART peripherals
        // is going to be the active peripheral so we can block and wait for autobaud
        // to finish here
        uint32_t previousEdge = currentEdge;
        // Keep track of any time outs
        bool expired = false;
        // Stores the running timer value
        uint64_t currentTicks;
        // Keeps a starting point for timeout reference and calculation
        uint64_t startTicks = microseconds_get_ticks();
        const uint64_t ticksTimeout = microseconds_convert_to_ticks(kAutobaudDetectDelay);

        // When we get to this point we know that we are active but are somewhere in the start
        // bit trough, due to other peripheral detection it is not guaranteed to be at the very start
        // of the transition so now that we are only spinning in here we can get an exact start time
        // at the next transition
        while (1)
        {
            currentTicks = microseconds_get_ticks();
            currentEdge = read_autobaud_pin(instance);

            // Check for the second transition
            if (currentEdge != previousEdge)
            {
                break;
            }

            if ((currentTicks - startTicks) > ticksTimeout)
            {
                expired = true;
                break;
            }
        }
        previousEdge = currentEdge;

        // Now we have gotten another transition so store the time of the previous transition. Two transitions
        // have occurred now so up the counter again
        startTicks = currentTicks;
        // We have had two transitions at the point
        uint32_t transitionCount = 2;

        // keep counting our edge transitions until the required number is met
        while (transitionCount < kRequiredEdges)
        {
            currentEdge = read_autobaud_pin(instance);

            if (currentEdge != previousEdge)
            {
                transitionCount++;
                previousEdge = currentEdge;
            }

            currentTicks = microseconds_get_ticks();

            if ((currentTicks - startTicks) > ticksTimeout)
            {
                expired = true;
                break;
            }
        }

        if (!expired)
        {
            if (rate)
            {
                *rate = (SystemCoreClock * kNumberOfBitsMeasured) / (uint32_t)(currentTicks - startTicks);
            }

            return kStatus_Success;
        }
        else
        {
            // The timer has expired meaning it has been too long since an
            // edge has been detected, reset detection
            s_initialEdge[instance] = currentEdge;
            return kStatus_Fail;
        }
    }
    else
    {
        // no baud rate yet/inactive
        return kStatus_Fail;
    }
}

////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////

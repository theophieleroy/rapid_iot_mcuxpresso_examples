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

#include <stdlib.h>
#include "fsl_dspi_driver_unit_test.h"
#include "clock/fsl_clock_manager.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/

/*!
 * @brief Function to setup dspi pins
 *
 *
 */
void dspi_test_pinmux_setup(uint32_t dspi_instance)
{
/* for K70 tower board*/
#if defined(CPU_MK70FN1M0VMJ12)
    if (dspi_instance == 1)
    {
        /* TO DO, currently nothing on tower board makes a likely candidate for instance 1*/
    }

    if (dspi_instance == 2)
    {
        /* ungate the clock to port d*/
        clock_manager_set_gate(kClockModulePORT, 4, true);

        /* set up the dspi2 pin muxing on port d*/
        PORT_BWR_PCR_MUX(HW_PORTD, 12, 2);
        PORT_BWR_PCR_MUX(HW_PORTD, 13, 2);
        PORT_BWR_PCR_MUX(HW_PORTD, 14, 2);
        PORT_BWR_PCR_MUX(HW_PORTD, 15, 2);
    }

/* for K64 tower board*/
#elif defined(CPU_MK64FN1M0VMD12)
    if (dspi_instance == 1)
    {
        /* ungate the clock to port b*/
        clock_manager_set_gate(kClockModulePORT, 1, true);

        /* set up the dspi1 pin muxing on port b*/
        PORT_BWR_PCR_MUX(HW_PORTB, 10, 2);
        PORT_BWR_PCR_MUX(HW_PORTB, 11, 2);
        PORT_BWR_PCR_MUX(HW_PORTB, 16, 2);
        PORT_BWR_PCR_MUX(HW_PORTB, 17, 2);
    }

    if (dspi_instance == 2)
    {
        /* ungate the clock to port d*/
        clock_manager_set_gate(kClockModulePORT, 4, true);

        /* set up the dspi2 pin muxing on port d*/
        PORT_BWR_PCR_MUX(HW_PORTD, 11, 2);
        PORT_BWR_PCR_MUX(HW_PORTD, 12, 2);
        PORT_BWR_PCR_MUX(HW_PORTD, 13, 2);
        PORT_BWR_PCR_MUX(HW_PORTD, 14, 2);
        PORT_BWR_PCR_MUX(HW_PORTD, 15, 2);
    }

/* for K22 tower board*/
#elif defined(CPU_MK22FN512VMC12)
    if (dspi_instance == 1)
    {
        /* ungate the clock to port b*/
        clock_manager_set_gate(kClockModulePORT, 1, true);

        /* set up the dspi1 pin muxing on port b*/
        PORT_BWR_PCR_MUX(HW_PORTB, 10, 2);
        PORT_BWR_PCR_MUX(HW_PORTB, 11, 2);
        PORT_BWR_PCR_MUX(HW_PORTB, 16, 2);
        PORT_BWR_PCR_MUX(HW_PORTB, 17, 2);
    }

#endif
}

/*******************************************************************************
 * EOF
 ******************************************************************************/

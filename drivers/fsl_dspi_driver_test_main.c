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
#include "utilities/fsl_assert.h"
#include <string.h>
#include "dspi/fsl_dspi_types.h"
#include "fsl_dspi_driver_unit_test.h"
#include "clock/fsl_clock_manager.h"
#include "utilities/fsl_debug_uart.h"
#include "board.h"
#include <stdio.h>

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define DEBUG_UART_BAUD (9600)

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
void init_hardware(void);

/*******************************************************************************
 * Code
 ******************************************************************************/
void init_hardware(void)
{
    /* Ungate port IO mux */
    SIM->SCGC5 |= (SIM_SCGC5_PORTA_MASK | SIM_SCGC5_PORTB_MASK | SIM_SCGC5_PORTC_MASK | SIM_SCGC5_PORTD_MASK |
                   SIM_SCGC5_PORTE_MASK);

    /* Init uart driver for stdio.*/
    debug_uart_init(BOARD_DEBUG_UART_INSTANCE, DEBUG_UART_BAUD);
}

/*!
 * @brief Test main function for dspi peripheral driver test.
 *
 *
 */
void main(void)
{
    uint32_t failCount;

    /* init the hardware*/
    init_hardware();

    printf("\r Running DSPI driver unit subtests  \n ");

    failCount = 0;
    failCount = dspi_test_master_functions();
    if (failCount == 0)
    {
        printf("\r  DSPI master driver functions subtest passed!  \n ");
    }
    else
    {
        printf("\r  ** %d failures in DSPI master driver functions subtest   \n ", failCount);
    }

/*
 * Same Board (same SPI) Loopback test
 * Written only for K70 tower board but can work on the K46 tower board
 * Since K64 has a SPI-to-SPI loopback and it is difficult to change out wires connections
 * we'll dedicate this test tot he K70 since we cannot perform a SPI-to-SPI loopback easily
 * on the K70 board
 * Connect SPI1 SOUT (J5.10) to SIN (J5.9)
 */
#if defined(CPU_MK70FN1M0VMJ12) || defined(CPU_MK22FN512VMC12)
    failCount = 0;
    failCount = dspi_self_loopback_test();
    if (failCount == 0)
    {
        printf("\r  DSPI master driver loopback subtest passed!  \n ");
    }
    else
    {
        printf("\r  ** %d failures in DSPI master driver loopback subtest   \n ", failCount);
    }
#endif

/*
 * Same Board SPI1-to-SPI2 Loopback test
 * Written only for K64 tower board
 * Connect SPI1 (slave) to SPI2 (master) as follows
 * SOUT (J3.10) to  SIN  (R109)
 * SCK  (J3.12) to  SCK  (R105)
 * PCS0 (J3.11) to  PCS0 (R107)
 * SIN  (J3.9)  to  SOUT (R108)
 */
#if defined(CPU_MK64FN1M0VMD12)
    failCount = 0;
    failCount = dspi_sameboard_loopback_test(HW_SPI2, HW_SPI1);
    if (failCount == 0)
    {
        printf("\r  DSPI same board K64 loopback subtest passed!  \n ");
    }
    else
    {
        printf("\r  ** %d failures in DSPI same board K64 loopback subtest   \n ", failCount);
    }
#endif

    if (failCount == 0)
    {
        printf("\r\nAll DSPI driver unit subtests passed!  \n ");
    }
    else
    {
        printf("\r\n** Failures detected in DSPI driver unit subtests  \n ");
    }
}

/*******************************************************************************
 * EOF
 ******************************************************************************/

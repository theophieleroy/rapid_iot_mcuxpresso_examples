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
#include "uart/fsl_uart_types.h"
#include "fsl_uart_driver_unit_test.h"
#include "clock/fsl_clock_manager.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

static void init_tower_hardware(void);
static uint32_t s_uartInstance;

/*******************************************************************************
 * Code
 ******************************************************************************/

/*!
 * @brief Test main function for uart peripheral driver test.
 *
 *
 */
void main(void)
{
#if defined(CPU_MKL25Z128VLK4)
    s_uartInstance = 0;
#elif defined(CPU_MK70FN1M0VMJ12)
/* K70 tower board uses serial port on the serial expansion board*/
#ifdef FSL_UART_K70_USE_UART0
    /* note, this requires tower board mods to work, not a default setting*/
    /* uses uart0 on pins PTB16 and PTB17*/
    /* uart0 and uart1 are the only uarts that have 8-word fifos, others only have 1-word fifos*/
    /* Tower Board mod:  */
    /* solder wire from R116 to J3.14 (TX)*/
    /* solder wire from R115 to R68 (RX)*/
    s_uartInstance = 0;
#else
    s_uartInstance = 2;
#endif /* FSL_UART_K70_USE_UART0*/

#elif defined(CPU_MK64FN1M0VMD12) || defined(CPU_MK22FN512VMC12)
    s_uartInstance = 1;

#endif

    init_tower_hardware();

    /* run various UART tests*/
    /* refer to individual test code for details of each test*/

    /* run async test which test simultaneous transmit and receive*/
    /* Test completes when user types x's for the receive*/
    uart_async_driver_test(s_uartInstance);

    /* This test will test the transmit and receive functionality using the*/
    /* tx/rx blocking functions */
    uart_blocking_driver_test();
}

/* tower board hardware setup */
static void init_tower_hardware(void)
{
    uint32_t i;

    /* ungate IO Port module clock*/
    for (i = 0; i < PORT_INSTANCE_COUNT; i++)
    {
        clock_manager_set_gate(kClockModulePORT, i, 1);
    }

/* SoC specific clock and port set up*/
#if defined(CPU_MKL25Z128VLK4)
    SIM->CLKDIV1 = (SIM_CLKDIV1_OUTDIV1(1) | SIM_CLKDIV1_OUTDIV4(1));

    SIM->SOPT2 |= SIM_SOPT2_PLLFLLSEL_MASK /* set PLLFLLSEL to select the PLL for this clock source*/
                  | SIM_SOPT2_UART0SRC(1); /* select the PLLFLLCLK as UART0 clock source*/

    /* Enable the UART_TXD function on PTA14 */
    PORT_BWR_PCR_MUX(HW_PORTA, 14, 3); /* UART0 is alt3 function for this pin*/

    /* Enable the UART_RXD function on PTA15 */
    PORT_BWR_PCR_MUX(HW_PORTA, 15, 3); /* UART0 is alt3 function for this pin*/

    /* Enable pins for UART1.*/
    PORT_BWR_PCR_MUX(HW_PORTC, 3, 3); /* UART1_RX is ALT3 for pin C3*/
    PORT_BWR_PCR_MUX(HW_PORTC, 4, 3); /* UART1_TX is ALT3 for pin C4*/

#elif defined(CPU_MK70FN1M0VMJ12)
    if (s_uartInstance == 0)
    {
        /* configure uart0 to uart tx/rx */
        /* UART0_RX: PTB16 */
        PORT_BWR_PCR_MUX(HW_PORTB, 16, 3);
        /* UART0_TX: PTB11 */
        PORT_BWR_PCR_MUX(HW_PORTB, 17, 3);

        /* set uart2 to gpio*/
        /* UART2_RX: PTE17 */
        PORT_BWR_PCR_MUX(HW_PORTE, 17, 1);
        /* UART2_TX: PTE16 */
        PORT_BWR_PCR_MUX(HW_PORTE, 16, 1);
    }
    else if (s_uartInstance == 2)
    {
        /* configure uart2 to uart tx/rx */
        /* UART2_RX: PTE17 */
        PORT_BWR_PCR_MUX(HW_PORTE, 17, 3);
        /* UART2_TX: PTE16 */
        PORT_BWR_PCR_MUX(HW_PORTE, 16, 3);

        /* set uart0 to gpio*/
        /* UART0_RX: PTB16 */
        PORT_BWR_PCR_MUX(HW_PORTB, 16, 1);
        /* UART0_TX: PTB11 */
        PORT_BWR_PCR_MUX(HW_PORTB, 17, 1);
    }
    else
    {
        /* TO-DO*/
    }

#elif defined(CPU_MK64FN1M0VMD12)
    /* ungate clocks*/
    /* UART1*/
    PORT_BWR_PCR_MUX(HW_PORTC, 3, 3);
    PORT_BWR_PCR_MUX(HW_PORTC, 4, 3);
    /*UART5*/
    PORT_BWR_PCR_MUX(HW_PORTE, 8, 3);
    PORT_BWR_PCR_MUX(HW_PORTE, 9, 3);

#elif defined(CPU_MK22FN512VMC12)
    /* UART1*/
    PORT_BWR_PCR_MUX(HW_PORTE, 0, 3);
    PORT_BWR_PCR_MUX(HW_PORTE, 1, 3);

#endif
}

/*******************************************************************************
 * EOF
 ******************************************************************************/

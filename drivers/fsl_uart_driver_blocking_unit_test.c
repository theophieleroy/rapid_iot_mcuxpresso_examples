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

#include "utilities/fsl_assert.h"
#include <stdlib.h>
#include <string.h>
#include "uart/fsl_uart_types.h"
#include "uart/fsl_uart_driver.h"
#include "bootloader_common.h"
#include "fsl_uart_driver_unit_test.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

static void exit_error(void);
static void uart_port_control(uint32_t uartInstance);

/*******************************************************************************
 * Code
 ******************************************************************************/

/*!
 * @brief Test function for uart peripheral driver blocking test.
 *
 * Tests the transmit and receive capabilities of the uart peripheral driver.
 * The user is required to connect the tower board to a PC running a serial
 * terminal program (like Tera Term) and set it for 9600baud-8-n-1
 * The second part of the test increases the baud rate to 115200.
 *
 */
void uart_blocking_driver_test(void)
{
    uint32_t uartInstCnt;
    uint32_t brCnt;

    /* TX specific variables*/
    uint8_t sourceBuff[26] = {10,  13,  'U', 'A', 'R', 'T', ' ', ' ', 'B', 'l', 'o', 'c', 'k',
                              'i', 'n', 'g', ' ', 'T', 'x', ' ', 'T', 'e', 's', 't', 10,  13};
    uint32_t byteCount;

    /* RX specific variables*/
    uint32_t readByteCount;
    uint8_t readBuffer[10] = {0};
    uint32_t timeoutValue = 1;

    uart_user_config_t uartConfig;

    uartConfig.bitCountPerChar = kUart8BitsPerChar;
    uartConfig.parityMode = kUartParityDisabled;
    uartConfig.stopBitCount = kUartOneStopBit;

    /* cycle through some various baud rates*/
    for (brCnt = 0; brCnt <= 1; brCnt++)
    {
/* test various uart module instances */
#if defined(CPU_MKL25Z128VLK4)
        for (uartInstCnt = 0; uartInstCnt < UART_INSTANCE_COUNT; uartInstCnt++) /* KL25 uart[2:0]*/
#elif defined(CPU_MK70FN1M0VMJ12)
#ifdef FSL_UART_K70_USE_UART0
        for (uartInstCnt = 0; uartInstCnt <= 2; uartInstCnt += 2) /* K70 uart 0 and uart2 */
#else
        for (uartInstCnt = 2; uartInstCnt <= 2; uartInstCnt++) /* K70 uart 2 only*/
#endif /* FSL_UART_K70_USE_UART0*/
#elif defined(CPU_MK64FN1M0VMD12)
        for (uartInstCnt = 1; uartInstCnt <= 5; uartInstCnt += 4) /* K64 uart1 and 5*/
#elif defined(CPU_MK22FN512VMC12)
        for (uartInstCnt = 1; uartInstCnt <= 1; uartInstCnt++) /* K22 uart 1 only*/
#endif
        {
/* It is possible to wire together uart ports to the same*/
/* tower board external uart port.  However, you first need to disable*/
/* the un-used uart port to avoid contention.*/
#if defined(CPU_MKL25Z128VLK4)
            if (uartInstCnt != 0)
#endif
            {
                uart_port_control(uartInstCnt);
            }

            /* instantiate variable uartState of type uart_state_t*/
            uart_state_t uartState;

            if (brCnt == 1)
            {
                /* configure baud rate  */
                uartConfig.baudRate = 9600;
/* For KL25z, UART1 and UART2 share the same serial terminal which is
   already set up for 115200bps */
#if defined(CPU_MKL25Z128VLK4)
                if (uartInstCnt == 2)
                {
                    uartConfig.baudRate = 115200;
                }
#endif
                /* init the uart module with instance number and config structure*/
                uart_init(uartInstCnt, &uartConfig, &uartState);

                uint8_t buffBaudChange[] = "\n\rChange terminal baud rate to 115200bps then hit any key \n\r";
                uint32_t byteCountBRchange = sizeof(buffBaudChange);
                uart_send_data(&uartState, buffBaudChange, byteCountBRchange, timeoutValue);
            }

            /* configure baud rate */
            /* brCnt=0, baud=9600; brCnt=1, baud=115200*/
            uartConfig.baudRate = 9600 + 9600 * brCnt * 11;

            /* init the uart module with instance number and config structure*/
            uart_init(uartInstCnt, &uartConfig, &uartState);

            /* When switching baudrates fom 9600 to 115200*/
            /* Tell user to switch baud rate on terminal */
            /* Need to provide time for user to set up PC terminal to new baud*/
            /* So wait till any char is receive on that particular uart instance*/
            if (brCnt == 1)
            {
                uart_receive_data(&uartState, readBuffer, 1, timeoutValue);
            }

            /****************************************************************************/
            /* TX test - chars will output to a terminal program*/
            /****************************************************************************/
            /* initialize the uart port number in the data buffer to send*/
            sourceBuff[6] = '0' + uartInstCnt;
            byteCount = sizeof(sourceBuff);

            uart_send_data(&uartState, sourceBuff, byteCount, 1);

            /****************************************************************************/
            /* RX test - requires user to type in several chars in a terminal */
            /* program for uart reception. */
            /* This test runs a for loop and decrements the number of bytes to type */
            /* for each iteration to test rx fifo watermark level versus byte count*/
            /****************************************************************************/
            uint32_t i;
            readByteCount = 6;

            uint8_t buff1[] = "\n\rReceive test, type in 6 chars \n\r";
            byteCount = sizeof(buff1);

            /* decrement the bytecount each time through the loop*/
            for (i = readByteCount; i >= 1; i--)
            {
                uart_send_data(&uartState, buff1, byteCount, timeoutValue);

                /* change the test output to decrement the number of bytes to type*/
                buff1[24] = '0' + (i - 1);

                if (uart_receive_data(&uartState, readBuffer, readByteCount, timeoutValue) == kStatus_UART_Timeout)
                {
                    exit_error();
                }
                else
                {
                    uint8_t buff2[] = "Received data: ";
                    uint32_t byteCount2 = sizeof(buff2);
                    uart_send_data(&uartState, buff2, byteCount2, timeoutValue);

                    /* transmit the data received back through the uart port*/
                    uart_send_data(&uartState, readBuffer, readByteCount, timeoutValue);
                }

                /* decrement the read byte count size*/
                readByteCount--;
            }

            /* shut down the uart*/
            uart_shutdown(&uartState);
        }
    }
}

/*
 * @brief UART port control when hw mods are made to the tower boards to test multiple uarts
 *
 * For KL25Z:
 * uart1 and uart2 port control to disable port of inactive uart during testing
 * Also, the KL25Z tower board should have wire connections as follows:
 * RX: J11.14 to J24.1 to Chip pin 75
 * TX: J11.13 to J26.1 to Chip pin 76
 * This connects uart1 and uart2 to the external uart port on the
 * TWR-SER board (connector J8). The code takes care of turning off unused
 * uart ports to avoid data contention.
 *
 * For K70:
 * uart2 is the board default uart that connects to the serial board
 * uart0 can be connected to the uart2 pins to test uart0 functionality
 * Refer to the main file on details of the hw connection
 *
 */
static void uart_port_control(uint32_t uartInstance)
{
#if defined(CPU_MKL25Z128VLK4)
    if (uartInstance == 1)
    {
        /* disable uart2 port control pins on tower board*/
        PORT_BWR_PCR_MUX(HW_PORTD, 2, 0);
        PORT_BWR_PCR_MUX(HW_PORTD, 3, 0);

        /* enable uart1 port control pins on tower board*/
        /* set iomux port control for uart1 to use PTC3 and PTC4, alt3*/
        PORT_BWR_PCR_MUX(HW_PORTC, 3, 3);
        PORT_BWR_PCR_MUX(HW_PORTC, 4, 3);
    }
    else
    {
        /* disable uart1 port control pins on tower board*/
        PORT_BWR_PCR_MUX(HW_PORTC, 3, 0);
        PORT_BWR_PCR_MUX(HW_PORTC, 4, 0);

        /* enable uart2 port control pins on tower board*/
        /* set iomux port control for uart2 to use PTD2 and PTD3, alt3*/
        PORT_BWR_PCR_MUX(HW_PORTD, 2, 3);
        PORT_BWR_PCR_MUX(HW_PORTD, 3, 3);
    }
#elif defined(CPU_MK70FN1M0VMJ12)
    if (uartInstance == 0)
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
    else if (uartInstance == 2)
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
#endif
}

static void exit_error(void)
{
    /* trap here on error  */
    while (1)
    {
    }
}

/*******************************************************************************
 * EOF
 ******************************************************************************/

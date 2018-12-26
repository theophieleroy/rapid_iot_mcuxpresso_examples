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
#ifndef __FSL_UART_DRIVER_UNIT_TEST_H__
#define __FSL_UART_DRIVER_UNIT_TEST_H__

#include <stdint.h>
#include "uart/hal/fsl_uart_hal.h"
#include "uart/hal/fsl_uart_features.h"
#include "bootloader_common.h"

/*!
 * @addtogroup uart_driver_unit_test
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/* note, this requires tower board mods to work, not a default setting, by defauly, leave commented
 * uses uart0 on pins PTB16 and PTB17
 * uart0 and uart1 are the only uarts that have 8-word fifos, others only have 1-word fifos
 * Tower Board mod:
 * solder wire from R116 to J3.14 (TX)
 * solder wire from R115 to R68 (RX)
 */
/*#define FSL_UART_K70_USE_UART0*/

/*******************************************************************************
 * Function Prototypes
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @name UART Driver Unit test
 * @{
 */

/*!
 * @brief Test function for uart peripheral driver blocking test.
 *
 */
void uart_blocking_driver_test(void);

/*!
 * @brief Test function for uart peripheral driver async (non-blocking) test.
 *    This test attempts to test the full duplex nature of the uart by transmitting and
 *    receiving simultaneously as can only be acheive using the async functions.
 *    This test is a while loop that constantly transmits data to a serial terminal.
 *    While transmitting, the user types 10 charaters in the terminal window. When the
 *    10th charater is typed, the test will output that data among the transmit stream.
 *    To end the test, type 'x' or 'X' for the 10 characters.
 *
 * @param uartInstance Uart instance number
 * @return kStatus_Success if test properly runs
 */
status_t uart_async_driver_test(uint32_t uartInstance);

/*!
 * @brief Hardware init for kl25z
 *
 */
void init_hardware(void);

/*@}  */

/*! @}*/

#endif /* __FSL_UART_DRIVER_UNIT_TEST_H__*/
       /*******************************************************************************
        * EOF
        ******************************************************************************/

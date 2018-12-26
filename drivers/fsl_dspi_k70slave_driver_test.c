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
#include "dspi/test/test_driver/src/fsl_dspi_driver_unit_test.h"
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
 * Variables
 ******************************************************************************/
extern IRQn_Type dspi_irq_ids[SPI_INSTANCE_COUNT];

/*!
 * @brief Buffer for storing data received by the SPI slave driver.
 */
static uint8_t s_dspiSinkBuffer[SPI_INSTANCE_COUNT][32] = {{0}};

/*!
 * @brief Index indicating the position in the SPI slave sink buffer.
 */
static uint32_t s_dspiSinkIndex[SPI_INSTANCE_COUNT] = {0};

/*!
 * @brief Buffer that supplies data to be transmitted with the SPI slave driver.
 */
static uint8_t s_dspiSourceBuffer[SPI_INSTANCE_COUNT][32] = {0};

/*!
 * @brief Index indicating the position in the SPI slave source buffer.
 */
static int s_dspiSourceIndex[SPI_INSTANCE_COUNT] = {0};

/* Number of TX FIFO underruns */
static uint32_t s_txFifoUnderRunCount = 0;

/* Number of RX FIFO Overruns */
static uint32_t s_rxFifoOverRunCount = 0;

/*******************************************************************************
 * Code
 ******************************************************************************/
void init_hardware(void)
{
    /* The following initializes the UART*/
    SIM->SCGC5 |= (SIM_SCGC5_PORTA_MASK | SIM_SCGC5_PORTB_MASK | SIM_SCGC5_PORTC_MASK | SIM_SCGC5_PORTD_MASK |
                   SIM_SCGC5_PORTE_MASK);

    /* Init uart driver for stdio.*/
    debug_uart_init(BOARD_DEBUG_UART_INSTANCE, DEBUG_UART_BAUD);
}

/*******************************************************************************
 *
 * @brief Callback used by slave IRQ handler to report an error condition.
 *
 * This callback function is used by the SPI slave IRQ handler to report an error
 * condition. If the error is an OutOfRange error, the appropriate buffer index is
 * reset.
 *
 * @param [in] error The error reported by the driver.
 * @param [in] instance The module instance number.
 *
 * @return nothing
 *
 ******************************************************************************/
static void on_error(status_t error, uint32_t instance)
{
    switch (error)
    {
        case kStatus_DSPI_SlaveTxUnderrun:
        {
            printf("\r\nSPI Error - Slave TX Underrun.\r\n");
            printf("\r\nIndex# %d\r\n", s_dspiSourceIndex[instance]);
            s_txFifoUnderRunCount++;
        }
        break;

        case kStatus_DSPI_SlaveRxOverrun:
        {
            printf("SPI Error - Slave RX Overrun.\r\n");
            printf("\r\nIndex# %d\r\n", s_dspiSourceIndex[instance]);
            s_rxFifoOverRunCount++;
        }
        break;

        default:
        {
            printf("SPI Error - Unknown Error.\r\n");
        }
        break;
    }
}

/*******************************************************************************
 *
 * @brief Callback used by slave IRQ handler to put byte it received.
 *
 * This callback function is used by the SPI slave IRQ handler to deliver the
 * byte it received to the application. If there is no room in the buffer, the
 * callback function will report kStatus_OutOfRange.
 *
 * @param [in] sinkByte The byte of data from the driver to store in the applications buffer.
 * @param [in] instance The module instance number.
 *
 * @return kStatus_Success or kStatus_OutOfRange
 *
 ******************************************************************************/
static status_t data_sink(uint8_t sinkWord, uint32_t instance)
{
    uint32_t index = s_dspiSinkIndex[instance];

    if (index < (sizeof(s_dspiSinkBuffer[instance])))
    {
        s_dspiSinkBuffer[instance][index] = sinkWord;
        ++s_dspiSinkIndex[instance];
    }
    if (index > (sizeof(s_dspiSinkBuffer[instance])))
    {
        s_dspiSinkIndex[instance] = 0;
        return kStatus_OutOfRange;
    }

    return kStatus_Success;
}

/*******************************************************************************
 *
 * @brief Callback used by slave IRQ handler to get byte to transmit.
 *
 * This callback function is used by the SPI slave IRQ handler to supply the
 * byte it transmitted by the application to the SPI driver. If there are no more
 * bytes in the buffer, the callback function will report kStatus_OutOfRange.
 *
 * @param [out] *sourceByte The byte of data from the applications buffer to
 * return to the driver.
 * @param [in] instance The module instance number.
 *
 * @return kStatus_Success or kStatus_OutOfRange
 *
 ******************************************************************************/
static status_t data_source(uint8_t *sourceWord, uint32_t instance)
{
    uint32_t index = s_dspiSourceIndex[instance];

    if (index < (sizeof(s_dspiSourceBuffer[instance])))
    {
        *sourceWord = s_dspiSourceBuffer[instance][index];
        ++s_dspiSourceIndex[instance];
    }
    if (index > (sizeof(s_dspiSourceBuffer[instance])))
    {
        s_dspiSourceIndex[instance] = 0;
        return kStatus_OutOfRange;
    }

    return kStatus_Success;
}

/* Setup the K70 tower board as a spi slave, instance 2 */
void main(void)
{
    uint32_t slaveInstance = HW_SPI2;
    uint32_t j;
    uint32_t failCount = 0;

    /* Ungate clocks to port module and call debug uart init */
    init_hardware();

    printf("\rK70 Tower board configured as spi slave \r\n");
    printf("\rNOTE: Slave devices need to use PCS0 (PTD11)  \r\n");

    /* pinmux_setup */
    /* most pins can be found on header J5, PCS0 is not on J5
     * You need to find a good spot to solder a wire, like on the expansion board
     * on the black connector side, J1, pin 9 (connected to pin D9 on the K70 tower board)
     */
    if (slaveInstance == HW_SPI2)
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
    else
    {
        /* TO DO, currently nothing on tower board makes a likely candidate for instance 1*/
    }

    /* Initialize the source buffer, each instance with unique data from the other intances */
    for (j = 0; j < 32; j++)
    {
        s_dspiSourceBuffer[slaveInstance][j] = (10 + j);
    }

    /* Reset the sink buffer */
    for (j = 0; j < 32; j++)
    {
        s_dspiSinkBuffer[slaveInstance][j] = 0;
    }
    s_dspiSinkIndex[slaveInstance] = 0;
    s_dspiSourceIndex[slaveInstance] = 0;

    /* Set up and init the slave  */
    dspi_slave_state_t dspiSlaveState;

    dspi_slave_user_config_t slaveUserConfig;
    slaveUserConfig.callbacks.dataSink = data_sink;
    slaveUserConfig.callbacks.dataSource = data_source;
    slaveUserConfig.callbacks.onError = on_error;
    slaveUserConfig.dataConfig.bitsPerFrame = 16;
    slaveUserConfig.dataConfig.clkPhase = kDspiClockPhase_FirstEdge;
    slaveUserConfig.dataConfig.clkPolarity = kDspiClockPolarity_ActiveHigh;

    /* keep trying to capture data infinitely */
    while (1)
    {
        if (dspi_slave_init(slaveInstance, &slaveUserConfig, &dspiSlaveState) != kStatus_Success)
        {
            printf("\r\nError in slave init \r\n");
        }

        /* wait until 32 words are received in the sink buffer */
        while (s_dspiSinkIndex[slaveInstance] != 32)
        {
        }

        /* verify the sink buffer contents with known data pattern */
        for (j = 0; j < 32; j++)
        {
            if (s_dspiSinkBuffer[slaveInstance][j] != j)
            {
                failCount++;
            }
        }

        if (failCount == 0)
        {
            printf("\rK70 spi slave test passed! \r\n");
        }
        else
        {
            printf("\r**failures detected in K70 spi slave test \r\n");
        }

        /* Shut down the slave after transferring data, then re-init to prep for next transfer */
        dspi_slave_shutdown(&dspiSlaveState);

        /* Reset the sink buffer */
        for (j = 0; j < 32; j++)
        {
            s_dspiSinkBuffer[slaveInstance][j] = 0;
        }
        s_dspiSinkIndex[slaveInstance] = 0;
        s_dspiSourceIndex[slaveInstance] = 0;
    }
}

/*******************************************************************************
 * EOF
 ******************************************************************************/

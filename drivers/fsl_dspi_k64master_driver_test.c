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
 * @brief Buffer that supplies data to be transmitted with the SPI slave driver.
 */
static uint8_t s_dspiSourceBuffer[SPI_INSTANCE_COUNT][32] = {0};

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

/* Setup the K64 tower board as a spi master, instance 1 on J3 */
void main(void)
{
    uint32_t masterInstance = HW_SPI1;
    uint32_t j;
    uint32_t failCount = 0;

    /* Ungate clocks to port module and call debug uart init */
    init_hardware();

    printf("\r\nK64 Tower board configured as spi master \r\n");
    printf("\rIMPORTANT, MAKE SURE TO FIRST SET UP THE SPI SLAVE BOARD!\r\n");

    /* pinmux_setup */
    if (masterInstance == HW_SPI1)
    {
        /* ungate the clock to port b*/
        clock_manager_set_gate(kClockModulePORT, 1, true);

        /* set up the dspi1 pin muxing on port b*/
        PORT_BWR_PCR_MUX(HW_PORTB, 10, 2);
        PORT_BWR_PCR_MUX(HW_PORTB, 11, 2);
        PORT_BWR_PCR_MUX(HW_PORTB, 16, 2);
        PORT_BWR_PCR_MUX(HW_PORTB, 17, 2);
    }

    if (masterInstance == HW_SPI2)
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

    /* Initialize the source buffer */
    for (j = 0; j < 32; j++)
    {
        s_dspiSourceBuffer[masterInstance][j] = j;
    }

    /* Reset the sink buffer */
    for (j = 0; j < 32; j++)
    {
        s_dspiSinkBuffer[masterInstance][j] = 0;
    }

    /* Set up and init the master  */
    dspi_master_state_t dspiMasterState;
    uint32_t calculatedBaudRate;

    /* configure the members of the user config */
    dspi_master_user_config_t userConfig;
    userConfig.isChipSelectContinuous = false;
    userConfig.isSckContinuous = false;
    userConfig.pcsPolarity = kDspiPcs_ActiveLow;
    userConfig.whichCtar = kDspiCtar0;
    userConfig.whichPcs = kDspiPcs0;

    /* init the dspi module */
    dspi_master_init(masterInstance, &dspiMasterState, &userConfig, &calculatedBaudRate);

    /* config member of the spi device config */
    dspi_device_t spiDevice;
    spiDevice.dataBusConfig.bitsPerFrame = 16;
    spiDevice.dataBusConfig.clkPhase = kDspiClockPhase_FirstEdge;
    spiDevice.dataBusConfig.clkPolarity = kDspiClockPolarity_ActiveHigh;
    spiDevice.dataBusConfig.direction = kDspiMsbFirst;
    spiDevice.bitsPerSec = 500000;

    /* configure the spi bus */
    failCount = 0; /* reset failCount variable */
    if (dspi_master_configure_bus(&dspiMasterState, &spiDevice, &calculatedBaudRate) != kStatus_Success)
    {
        failCount++;
    }
    if (calculatedBaudRate > spiDevice.bitsPerSec)
    {
        failCount++;
    }
    if (failCount != 0)
    {
        printf("\r**Something failed in the master init \r\n");
    }

    /***************************************************************************
     * Sync test
     **************************************************************************/
    printf("\r\nSpi master SYNC test  \r\n");

    /* Start the transfer */
    dspi_master_transfer(&dspiMasterState, NULL, s_dspiSourceBuffer[masterInstance], s_dspiSinkBuffer[masterInstance],
                         32, 1);

    /* Verify the contents of the master sink buffer */
    /* note that the first word in the master sink buffer is 0 from the slave */
    /* refer to the slave driver for the expected data pattern */
    failCount = 0; /* reset failCount variable */
    for (j = 0; j < 32; j++)
    {
        if ((j == 0) || (j == 1))
        {
            if (s_dspiSinkBuffer[masterInstance][j] != 0)
            {
                failCount++;
            }
        }
        else
        {
            if (s_dspiSinkBuffer[masterInstance][j] != (10 + (j - 2)))
            {
                failCount++;
            }
        }
    }

    if (failCount == 0)
    {
        printf("\rK64 spi master sync test passed! \r\n");
    }
    else
    {
        printf("\r**failures detected in K64 spi master test \r\n");
    }

    /***************************************************************************
     * Async test
     **************************************************************************/
    printf("\r\nSpi master ASYNC test  \r\n");

    /* Slow down the baud rate so that we can test and output words transferred later */
    spiDevice.bitsPerSec = 5000;

    /* configure the spi bus */
    failCount = 0; /* reset failCount variable */
    if (dspi_master_configure_bus(&dspiMasterState, &spiDevice, &calculatedBaudRate) != kStatus_Success)
    {
        failCount++;
    }
    if (calculatedBaudRate > spiDevice.bitsPerSec)
    {
        failCount++;
    }
    if (failCount != 0)
    {
        printf("\r**Something failed in the master init \r\n");
    }

    /* Start the transfer */
    dspi_master_transfer_async(&dspiMasterState, NULL, s_dspiSourceBuffer[masterInstance],
                               s_dspiSinkBuffer[masterInstance], 32);

    uint32_t wordsXfer;
    while (dspi_master_get_transfer_status(&dspiMasterState, &wordsXfer) == kStatus_DSPI_Busy)
    {
        printf("\rWords transferred = %d\r\n", wordsXfer);
    }

    /* Verify the contents of the master sink buffer */
    /* note that the first word in the master sink buffer is 0 from the slave */
    /* refer to the slave driver for the expected data pattern */
    failCount = 0; /* reset failCount variable */
    for (j = 0; j < 32; j++)
    {
        if ((j == 0) || (j == 1))
        {
            if (s_dspiSinkBuffer[masterInstance][j] != 0)
            {
                failCount++;
            }
        }
        else
        {
            if (s_dspiSinkBuffer[masterInstance][j] != (10 + (j - 2)))
            {
                failCount++;
            }
        }
    }

    if (failCount == 0)
    {
        printf("\rK64 spi master async test passed! \r\n");
    }
    else
    {
        printf("\r**failures detected in K64 spi master test \r\n");
    }
}

/*******************************************************************************
 * EOF
 ******************************************************************************/

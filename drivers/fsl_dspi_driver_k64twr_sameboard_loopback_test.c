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
#include <stdio.h>

/*******************************************************************************
 * Definitions
 ******************************************************************************/

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

/*******************************************************************************
 * Sameboard loopback test (for K64 tower board)
 *
 * IMPORTANT HARDWARE CONSIDERATION
 * On the board, physically connect master SPI and slave SPI instances
 * as follows:
 *  master SPI SCK  to slave SPI SCK
 *  master SPI PCS0 to slave SPI PCS0
 *  master SPI SIN  to slave SPI SOUT
 *  master SPI SOUT to slave SPI SIN
 ******************************************************************************/

uint32_t dspi_sameboard_loopback_test(uint32_t masterInstance, uint32_t slaveInstance)
{
    uint32_t testCount = 0;        /* First test sync transfer then async */
    uint32_t bitPerFrameCount = 0; /* First various bit/frame values */

    /* Configure interrupt priorities for master and slave
     * Set the slave to a higher priority interrupt than the master
     * The slave needs to respond first to set up it's transmit buffer to
     * enable it to transfer data for the next tranfer from the amster
     */
    NVIC_SetPriority(dspi_irq_ids[masterInstance], 10);
    NVIC_SetPriority(dspi_irq_ids[slaveInstance], 1);

    /* Configure the pin mux for the dspi instances */
    dspi_test_pinmux_setup(masterInstance);
    dspi_test_pinmux_setup(slaveInstance);

    uint32_t failCount = 0;

    /* Test various bits/frame values:
     * bitPerFrameCount = 0; 8 bits/frame
     * bitPerFrameCount = 1; 11 bits/frame
     * bitPerFrameCount = 0; 8 bits/frame
     */
    for (bitPerFrameCount = 8; bitPerFrameCount <= 16; bitPerFrameCount++)
    {
        /* Initialize the source buffer, each instance with unique data from the other intances */
        uint32_t i, j;
        for (i = 0; i < SPI_INSTANCE_COUNT; i++)
        {
            for (j = 0; j < 32; j++)
            {
                s_dspiSourceBuffer[i][j] = (10 * i) + j;
                /* When testing > 8 bits/frame, every other byte is only (bit/frame-8)bits in length
                 * hence need to mask off upper bits
                 */
                if (bitPerFrameCount > 8)
                {
                    j++;
                    s_dspiSourceBuffer[i][j] = ((10 * i) + j) & (~(0xFF << (bitPerFrameCount - 8)));
                }
            }
        }

        /* Test sync transfer (testCount=0) then async (testCount=1)*/
        for (testCount = 0; testCount < 2; testCount++)
        {
            /* Reset the sink buffer */
            for (i = 0; i < SPI_INSTANCE_COUNT; i++)
            {
                for (j = 0; j < 32; j++)
                {
                    s_dspiSinkBuffer[i][j] = 0;
                }
                s_dspiSinkIndex[i] = 0;
                s_dspiSourceIndex[i] = 0;
            }

            /* Set up and init the slave  */
            dspi_slave_state_t dspiSlaveState;

            dspi_slave_user_config_t slaveUserConfig;
            slaveUserConfig.callbacks.dataSink = data_sink;
            slaveUserConfig.callbacks.dataSource = data_source;
            slaveUserConfig.callbacks.onError = on_error;
            slaveUserConfig.dataConfig.bitsPerFrame = bitPerFrameCount;
            slaveUserConfig.dataConfig.clkPhase = kDspiClockPhase_FirstEdge;
            slaveUserConfig.dataConfig.clkPolarity = kDspiClockPolarity_ActiveHigh;

            dspi_slave_init(slaveInstance, &slaveUserConfig, &dspiSlaveState);

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
            spiDevice.dataBusConfig.bitsPerFrame = bitPerFrameCount;
            spiDevice.dataBusConfig.clkPhase = kDspiClockPhase_FirstEdge;
            spiDevice.dataBusConfig.clkPolarity = kDspiClockPolarity_ActiveHigh;
            spiDevice.dataBusConfig.direction = kDspiMsbFirst;
            spiDevice.bitsPerSec = 500000;

            /* configure the spi bus */
            if (dspi_master_configure_bus(&dspiMasterState, &spiDevice, &calculatedBaudRate) != kStatus_Success)
            {
                failCount++;
            }
            if (calculatedBaudRate > spiDevice.bitsPerSec)
            {
                failCount++;
            }

            if (testCount == 0)
            {
                /* Start the sync transfer */
                dspi_master_transfer(&dspiMasterState, NULL, s_dspiSourceBuffer[masterInstance],
                                     s_dspiSinkBuffer[masterInstance], 32, 1);
            }
            else
            {
                /* Start the Async transfer */
                dspi_master_transfer_async(&dspiMasterState, NULL, s_dspiSourceBuffer[masterInstance],
                                           s_dspiSinkBuffer[masterInstance], 32);

                /* Because of aync mode, we have to ping the master driver to
                 * see when the transfer is complete, put NULL for transfer count since we don't care
                 */
                while (dspi_master_get_transfer_status(&dspiMasterState, NULL) == kStatus_DSPI_Busy)
                {
                }
            }

            /* Verify the contents of the master sink buffer */
            /* note that the first word in the master sink buffer is 0 from the slave */
            for (i = 0; i < 32; i++)
            {
                if (i == 0)
                {
                    if (s_dspiSinkBuffer[masterInstance][i] != 0)
                    {
                        failCount++;
                    }
                    /* If bit/frame is greater than one byte, then the next byte will also be 0 */
                    if (bitPerFrameCount > 8)
                    {
                        i++;
                        if (s_dspiSinkBuffer[masterInstance][i] != 0)
                        {
                            failCount++;
                        }
                    }
                }
                else
                {
                    /* If bit/frame is greater than one byte (i.e. two bytes), then when to compare
                     * starting from the 3rd byte, else start from the 2nd byte
                     */
                    if (bitPerFrameCount > 8)
                    {
                        if (s_dspiSinkBuffer[masterInstance][i] != s_dspiSourceBuffer[slaveInstance][i - 2])
                        {
                            failCount++;
                        }
                    }
                    else
                    {
                        if (s_dspiSinkBuffer[masterInstance][i] != s_dspiSourceBuffer[slaveInstance][i - 1])
                        {
                            failCount++;
                        }
                    }
                }
            }

            /* Verify the contents of the slave sink buffer */
            for (i = 0; i < 32; i++)
            {
                if (s_dspiSinkBuffer[slaveInstance][i] != s_dspiSourceBuffer[masterInstance][i])
                {
                    failCount++;
                }
            }

            /* add to faiCount any tx underruns or rx overruns */
            failCount += s_txFifoUnderRunCount;
            failCount += s_rxFifoOverRunCount;

            dspi_slave_shutdown(&dspiSlaveState);
            dspi_master_shutdown(&dspiMasterState);
        }
    }

    return failCount;
}

/*******************************************************************************
 * EOF
 ******************************************************************************/

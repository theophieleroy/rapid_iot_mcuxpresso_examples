/*
 * The Clear BSD License
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
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

#include "fsl_dspi_test.h"

/*******************************************************************************
* Variables
******************************************************************************/

#if 1
/* Note: the first and last value will be updated to minimum and maximum value at runtime. */
uint32_t g_baudrate[] = {300,      500,      1000,     5000,     10000,    25000,    50000,   75000,
                         100000,   500000,   750000,   1000000,  2500000,  5000000,  7500000, 10000000,
                         12500000, 15000000, 17500000, 20000000, 25000000, 27500000, 30000000};
#else
uint32_t g_baudrate[] = {15000000};
#endif

uint32_t g_dspi_baudrate;

dspi_master_handle_t g_m_handle;
dspi_slave_handle_t g_s_handle;
dspi_master_edma_handle_t g_dspi_edma_m_handle;
dspi_slave_edma_handle_t g_dspi_edma_s_handle;

edma_handle_t dspiEdmaMasterRxRegToRxDataHandle;
edma_handle_t dspiEdmaMasterTxDataToIntermediaryHandle;
edma_handle_t dspiEdmaMasterIntermediaryToTxRegHandle;
edma_handle_t dspiEdmaSlaveRxRegToRxDataHandle;
edma_handle_t dspiEdmaSlaveTxDataToTxRegHandle;

uint8_t masterReceiveBuffer[BUFFER_SIZE];
uint8_t masterSendBuffer[BUFFER_SIZE];
uint8_t slaveReceiveBuffer[BUFFER_SIZE];
uint8_t slaveSendBuffer[BUFFER_SIZE];

/* Declare a 32-byte aligned software transfer control descriptor (DMA requirement) */
#if defined(__GNUC__) /* For toolchains like Atollic and KDS */
edma_tcd_t dspiMaterSoftwareTcd __attribute__((aligned(32)));
edma_tcd_t dspiSlaveSoftwareTcd __attribute__((aligned(32)));

#elif defined(__ARMCC_VERSION) /* For toolchains like Keil */
edma_tcd_t dspiSlaveSoftwareTcd __attribute__((aligned(32)));
edma_tcd_t dspiMaterSoftwareTcd __attribute__((aligned(32)));

#elif defined(__IAR_SYSTEMS_ICC__) /* For toolchains like IAR */
#pragma data_alignment = 32
edma_tcd_t dspiMaterSoftwareTcd;
#pragma data_alignment = 32
edma_tcd_t dspiSlaveSoftwareTcd;
#endif

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static void DSPI_test_setIrqPriority(void);

/*******************************************************************************
 * Code
 ******************************************************************************/

/*******************************************************************************
 * Sameboard loopback test
 *
 * IMPORTANT HARDWARE CONSIDERATION
 * On the board, physically connect master SPI and slave SPI instances
 * as follows:
 *  master SPI SCK  to slave SPI SCK
 *  master SPI PCS0 to slave SPI PCS0
 *  master SPI SOUT to slave SPI SIN
 *  master SPI SIN  to slave SPI SOUT
 *
 * This will test one SPI instance as master and another as slave, hence the physical
 * connections are needed as stated above.
 *
 * This test has several test loops, listed below:
 * - Baud rate count loop: covered minimum, medium, and max.
 * - bitPerFrameCount: this tests all supported master bits/frame, currently 4~16 bits/frame.
 * - Transfer bytes count: this tests 1 ~ MAX_COUNT bytes tranfer.
 * - Transfer mode: tests polling, interrupt, dma mode with TX/RX buffer is NULL.
 * - SPI mode: Continuous SCK, clock phase, clock polarity, continuous PCS.
 * - Others: currently coverd transfer without using FIFO and master use CTAR1.
 *
 ******************************************************************************/
int main(void)
{
    uint32_t sourceClock;
    uint32_t maxBaudrate, minBaudrate;
    uint32_t i, k;
    uint32_t baudrateListCount;

    BOARD_InitHardware();

    /* Set dspi slave interrupt priority higher than master. */
    DSPI_test_setIrqPriority();

    PRINTF("\r\nDSPI unit test.\r\n");

    UnityBegin("DSPI_test");

    PRINTF("\r\nNote: the baudrate can not be set too slow or high due to the process delay and IP setting!\r\n");

    sourceClock = CLOCK_GetFreq(DSPI_MASTER_CLK_SRC);

    minBaudrate = sourceClock / (32768 * 7) + 10; /* add 10 to round up*/

    /* Set Baud rate to be 1/4 DSPI source clock.  If set to be 1/2 soruce clock
     * which is the absolute maximum, then user may need to increase drive strength even for self loopback.  */
    maxBaudrate = sourceClock / 4;
    baudrateListCount = sizeof(g_baudrate) / sizeof(g_baudrate[0]);

    /* Insert the min and max baudrate into the baudrate list. */
    g_baudrate[0] = minBaudrate;
    g_baudrate[baudrateListCount - 1] = maxBaudrate;

    PRINTF("For low baudrate please wait a moment for the transfer complete. \r\n");

    /* API test.*/
    RUN_TEST(DSPI_test_legacy_API, MAKE_UNITY_NUM(k_unity_dspi, 0));

    /* Function test. */
    for (i = 0; i < baudrateListCount; i++)
    {
        k = 1;
        g_dspi_baudrate = g_baudrate[i];
        if (g_dspi_baudrate >= maxBaudrate)
        {
            /* Test the max baudrate. */
            g_dspi_baudrate = maxBaudrate;
            i = 0xFF;
        }

        PRINTF("\r\nThe set baud rate is %d bps.\r\n", g_dspi_baudrate);

        /* Basic functional common usecase test. */
        RUN_TEST(DSPI_test_masterPolling_SlaveInt, MAKE_UNITY_NUM(k_unity_dspi, k++));
        RUN_TEST(DSPI_test_masterInt_SlaveInt, MAKE_UNITY_NUM(k_unity_dspi, k++));
        RUN_TEST(DSPI_test_masterDMA_SlaveDMA, MAKE_UNITY_NUM(k_unity_dspi, k++));
        RUN_TEST(DSPI_test_masterSlaveBlocking, MAKE_UNITY_NUM(k_unity_dspi, k++));

        /* Vary transfer count with different bits/frame using Polling, Interrupt, DMA. */
        RUN_TEST(DSPI_test_vary_transfer_MasterSendPolling, MAKE_UNITY_NUM(k_unity_dspi, k++));
        RUN_TEST(DSPI_test_vary_transfer_MasterSendInterrupt, MAKE_UNITY_NUM(k_unity_dspi, k++));
        RUN_TEST(DSPI_test_vary_transfer_MasterSendDMA, MAKE_UNITY_NUM(k_unity_dspi, k++));
        RUN_TEST(DSPI_test_vary_transfer_MasterSendReceivePolling, MAKE_UNITY_NUM(k_unity_dspi, k++));
        RUN_TEST(DSPI_test_vary_transfer_MasterSendReceiveInterrupt, MAKE_UNITY_NUM(k_unity_dspi, k++));
        RUN_TEST(DSPI_test_vary_transfer_MasterSendReceiveDMA, MAKE_UNITY_NUM(k_unity_dspi, k++));

        /* Vary polarity and phase. */
        RUN_TEST(DSPI_test_vary_polarity_phase, MAKE_UNITY_NUM(k_unity_dspi, k++));

        /* Continuous SCK mode. */
        RUN_TEST(DSPI_test_transfer_continuousSCK, MAKE_UNITY_NUM(k_unity_dspi, k++));

        /* No need to test really low baud rates. */
        if (g_dspi_baudrate > 1000000)
        {
            RUN_TEST(DSPI_test_transferPerformancePolling, MAKE_UNITY_NUM(k_unity_dspi, k++));
            RUN_TEST(DSPI_test_transferPerformanceInterrupt, MAKE_UNITY_NUM(k_unity_dspi, k++));
            RUN_TEST(DSPI_test_transferPerformanceDMA, MAKE_UNITY_NUM(k_unity_dspi, k++));
        }

        RUN_TEST(DSPI_test_vary_legacy, MAKE_UNITY_NUM(k_unity_dspi, k++));
    }

    UnityEnd();
    while (1)
    {
    }
}

static void DSPI_test_setIrqPriority(void)
{
/* Set dspi slave interrupt priority higher. */
#if (EXAMPLE_DSPI_SLAVE_BASE == SPI0_BASE)
    NVIC_SetPriority(SPI0_IRQn, 0);
#elif(EXAMPLE_DSPI_SLAVE_BASE == SPI1_BASE)
    NVIC_SetPriority(SPI1_IRQn, 0);
#elif(EXAMPLE_DSPI_SLAVE_BASE == SPI2_BASE)
    NVIC_SetPriority(SPI2_IRQn, 0);
#endif

#if (EXAMPLE_DSPI_MASTER_BASE == SPI0_BASE)
    NVIC_SetPriority(SPI0_IRQn, 1);
#elif(EXAMPLE_DSPI_MASTER_BASE == SPI1_BASE)
    NVIC_SetPriority(SPI1_IRQn, 1);
#elif(EXAMPLE_DSPI_MASTER_BASE == SPI2_BASE)
    NVIC_SetPriority(SPI2_IRQn, 1);
#endif
}

void setUp(void)
{
}

void tearDown(void)
{
    DSPI_Deinit(EXAMPLE_DSPI_MASTER_BASEADDR);
    DSPI_Deinit(EXAMPLE_DSPI_SLAVE_BASEADDR);
}

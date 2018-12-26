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
* Definitions
******************************************************************************/
static bool isTransferCompleted;

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

static void DSPI_test_transferPerformance(dspi_transfer_mode_t master_mode);

/*******************************************************************************
 * Code
 ******************************************************************************/

static void dspi_slave_callback(SPI_Type *base, dspi_slave_handle_t *handle, status_t status, void *isXferCompleted)
{
    TEST_ASSERT_TRUE(status == kStatus_Success);
    *((bool *)isXferCompleted) = true;
}

static void dspi_slave_edma_callback(SPI_Type *base,
                                     dspi_slave_edma_handle_t *handle,
                                     status_t status,
                                     void *isXferCompleted)
{
    TEST_ASSERT_TRUE(status == kStatus_Success);
    *((bool *)isXferCompleted) = true;
}

void DSPI_test_transferPerformancePolling(void)
{
    DSPI_test_transferPerformance(kPolling);
}

void DSPI_test_transferPerformanceInterrupt(void)
{
    DSPI_test_transferPerformance(kInterrupt);
}

void DSPI_test_transferPerformanceDMA(void)
{
    DSPI_test_transferPerformance(kDMA);
}

/* Use PIT to count transfer time. */
static void timerInit(void)
{
#if defined(FSL_FEATURE_SOC_PIT_COUNT) && (FSL_FEATURE_SOC_PIT_COUNT)
    uint32_t clockSource;
    pit_config_t pitConfig;
    /*
     * pitConfig.enableRunInDebug = false;
     */
    PIT_GetDefaultConfig(&pitConfig);
    PIT_Init(PIT, &pitConfig);
    /* Set timer period for channel 0 */
    clockSource = CLOCK_GetFreq(BUS_CLK);
    PIT_SetTimerPeriod(PIT, kPIT_Chnl_0, USEC_TO_COUNT(0x1000000U, clockSource));
#endif /* FSL_FEATURE_SOC_PIT_COUNT */
}

static void timerStart(void)
{
#if defined(FSL_FEATURE_SOC_PIT_COUNT) && (FSL_FEATURE_SOC_PIT_COUNT)
    PIT_StartTimer(PIT, kPIT_Chnl_0);
#endif /* FSL_FEATURE_SOC_PIT_COUNT */
}

static void timerStop(void)
{
#if defined(FSL_FEATURE_SOC_PIT_COUNT) && (FSL_FEATURE_SOC_PIT_COUNT)
    PIT_StopTimer(PIT, kPIT_Chnl_0);
#endif /* FSL_FEATURE_SOC_PIT_COUNT */
}

static uint32_t getTimerCount(void)
{
#if defined(FSL_FEATURE_SOC_PIT_COUNT) && (FSL_FEATURE_SOC_PIT_COUNT)
    /* Get current timer count, and reverse it to up-counting.*/
    return ~PIT_GetCurrentTimerCount(PIT, kPIT_Chnl_0);
#else
    return 0;
#endif /* FSL_FEATURE_SOC_PIT_COUNT */
}

static void DSPI_test_transferPerformance(dspi_transfer_mode_t master_mode)
{
    /* How many bits/frame tested, used in performance test. */
    uint8_t bitsList[] = {8, 12, 16};

    uint32_t sourceClock;
    uint32_t i, j;
    uint32_t status = kStatus_Fail;
    uint32_t time1, time2, usedTime;
    uint32_t calculatedBaudRate;

    dspi_master_config_t masterConfig;
    dspi_slave_config_t slaveConfig;
    dspi_transfer_t masterXfer;
    dspi_transfer_t slaveXfer;

    /*If use DMA, configure DMA and DMAMUX. */
    if (master_mode == kDMA)
    {
        DMAMGR_Init();

        /* Setup master dma manager. */
        if (DMAMGR_RequestChannel((dma_request_source_t)(EXAMPLE_DSPI_MASTER_DMA_RX_REQUEST_SOURCE | 0x100U),
                                  DMAMGR_DYNAMIC_ALLOCATE, &dspiEdmaMasterRxRegToRxDataHandle) != kStatus_Success)
        {
            PRINTF("\r\n DMA RequestChannel Failed!\r\n");
        }
#if defined(EXAMPLE_DSPI_MASTER_DMA_TX_REQUEST_SOURCE) /* Set up channels for separate RX/TX DMA requests*/
        if (DMAMGR_RequestChannel((dma_request_source_t)(EXAMPLE_DSPI_MASTER_DMA_TX_REQUEST_SOURCE | 0x100U),
                                  DMAMGR_DYNAMIC_ALLOCATE, &dspiEdmaMasterIntermediaryToTxRegHandle) != kStatus_Success)
        {
            PRINTF("\r\n DMA RequestChannel Failed!\r\n");
        }
#else  /* Set up channels for shared RX/TX DMA request */
        if (DMAMGR_RequestChannel(kDmaRequestMux0Disable, DMAMGR_DYNAMIC_ALLOCATE,
                                  &dspiEdmaMasterIntermediaryToTxRegHandle) != kStatus_Success)
        {
            PRINTF("\r\n DMA RequestChannel Failed!\r\n");
        }
#endif /*EXAMPLE_DSPI_MASTER_DMA_TX_REQUEST_SOURCE*/
        if (DMAMGR_RequestChannel(kDmaRequestMux0Disable, DMAMGR_DYNAMIC_ALLOCATE,
                                  &dspiEdmaMasterTxDataToIntermediaryHandle) != kStatus_Success)
        {
            PRINTF("\r\n DMA RequestChannel Failed!\r\n");
        }

        /* Setup slave dma manager. */
        if (DMAMGR_RequestChannel((dma_request_source_t)(EXAMPLE_DSPI_SLAVE_DMA_RX_REQUEST_SOURCE | 0x100U),
                                  DMAMGR_DYNAMIC_ALLOCATE, &dspiEdmaSlaveRxRegToRxDataHandle) != kStatus_Success)
        {
            PRINTF("\r\n DMA RequestChannel Failed!\r\n");
        }
#if defined(EXAMPLE_DSPI_SLAVE_DMA_TX_REQUEST_SOURCE) /* Set up channels for separate RX/TX DMA requests*/
        if (DMAMGR_RequestChannel((dma_request_source_t)(EXAMPLE_DSPI_SLAVE_DMA_TX_REQUEST_SOURCE | 0x100U),
                                  DMAMGR_DYNAMIC_ALLOCATE, &dspiEdmaSlaveTxDataToTxRegHandle) != kStatus_Success)
        {
            PRINTF("\r\n DMA RequestChannel Failed!\r\n");
        }
#else  /* Set up channels for shared RX/TX DMA request */
        if (DMAMGR_RequestChannel(kDmaRequestMux0Disable, DMAMGR_DYNAMIC_ALLOCATE, &dspiEdmaSlaveTxDataToTxRegHandle) !=
            kStatus_Success)
        {
            PRINTF("\r\n DMA RequestChannel Failed!\r\n");
        }
#endif /*EXAMPLE_DSPI_SLAVE_DMA_TX_REQUEST_SOURCE*/

        DSPI_SlaveTransferCreateHandleEDMA(EXAMPLE_DSPI_SLAVE_BASEADDR, &g_dspi_edma_s_handle, dspi_slave_edma_callback,
                                           &isTransferCompleted, &dspiEdmaSlaveRxRegToRxDataHandle,
                                           &dspiEdmaSlaveTxDataToTxRegHandle);

        DSPI_MasterTransferCreateHandleEDMA(
            EXAMPLE_DSPI_MASTER_BASEADDR, &g_dspi_edma_m_handle, NULL, NULL, &dspiEdmaMasterRxRegToRxDataHandle,
            &dspiEdmaMasterTxDataToIntermediaryHandle, &dspiEdmaMasterIntermediaryToTxRegHandle);
    }

    /*
     * masterConfig.whichPcs = kDSPI_Pcs0;
     * masterConfig.pcsActiveHighOrLow = kDSPI_PcsActiveLow;
     * masterConfig.enableContinuousSCK = false;
     * masterConfig.enableRxFifoOverWrite = false;
     * masterConfig.enableModifiedTimingFormat = false;
     * masterConfig.samplePoint = kDSPI_SckToSin0Clock;
     */
    DSPI_MasterGetDefaultConfig(&masterConfig);

    masterConfig.whichCtar = kDSPI_Ctar0;
    masterConfig.ctarConfig.baudRate = g_dspi_baudrate;
    masterConfig.ctarConfig.bitsPerFrame = 8;
    masterConfig.ctarConfig.cpol = kDSPI_ClockPolarityActiveHigh;
    masterConfig.ctarConfig.cpha = kDSPI_ClockPhaseFirstEdge;
    masterConfig.ctarConfig.direction = kDSPI_MsbFirst;
    masterConfig.ctarConfig.pcsToSckDelayInNanoSec = 2000;
    masterConfig.ctarConfig.lastSckToPcsDelayInNanoSec = 2000;
    masterConfig.ctarConfig.betweenTransferDelayInNanoSec = 1000;

    masterConfig.whichPcs = kDSPI_Pcs0;
    masterConfig.pcsActiveHighOrLow = kDSPI_PcsActiveLow;

    masterConfig.enableContinuousSCK = false;
    masterConfig.enableRxFifoOverWrite = false;
    masterConfig.enableModifiedTimingFormat = false;
    masterConfig.samplePoint = kDSPI_SckToSin0Clock;

    sourceClock = CLOCK_GetFreq(DSPI_MASTER_CLK_SRC);
    DSPI_MasterInit(EXAMPLE_DSPI_MASTER_BASEADDR, &masterConfig, sourceClock);
    DSPI_MasterTransferCreateHandle(EXAMPLE_DSPI_MASTER_BASEADDR, &g_m_handle, NULL, NULL);

    calculatedBaudRate =
        DSPI_MasterSetBaudRate(EXAMPLE_DSPI_MASTER_BASEADDR, kDSPI_Ctar0, g_dspi_baudrate, sourceClock);

    /*
     * slaveConfig.enableContinuousSCK = false;
     * slaveConfig.enableRxFifoOverWrite = false;
     * slaveConfig.enableModifiedTimingFormat = false;
     * slaveConfig.samplePoint = kDSPI_SckToSin0Clock;
     */
    DSPI_SlaveGetDefaultConfig(&slaveConfig);

    slaveConfig.whichCtar = kDSPI_Ctar0;
    slaveConfig.ctarConfig.bitsPerFrame = masterConfig.ctarConfig.bitsPerFrame;
    slaveConfig.ctarConfig.cpol = masterConfig.ctarConfig.cpol;
    slaveConfig.ctarConfig.cpha = masterConfig.ctarConfig.cpha;
    slaveConfig.enableContinuousSCK = masterConfig.enableContinuousSCK;
    slaveConfig.enableRxFifoOverWrite = masterConfig.enableRxFifoOverWrite;
    slaveConfig.enableModifiedTimingFormat = masterConfig.enableModifiedTimingFormat;
    slaveConfig.samplePoint = masterConfig.samplePoint;

    DSPI_SlaveInit(EXAMPLE_DSPI_SLAVE_BASEADDR, &slaveConfig);
    DSPI_SlaveTransferCreateHandle(EXAMPLE_DSPI_SLAVE_BASEADDR, &g_s_handle, dspi_slave_callback, &isTransferCompleted);

    masterXfer.txData = masterSendBuffer;
    masterXfer.rxData = masterReceiveBuffer;
    masterXfer.dataSize = BUFFER_SIZE;
    masterXfer.configFlags = kDSPI_MasterCtar0 | kDSPI_MasterPcs0 | kDSPI_MasterPcsContinuous;

    slaveXfer.txData = slaveSendBuffer;
    slaveXfer.rxData = slaveReceiveBuffer;
    slaveXfer.dataSize = BUFFER_SIZE;
    slaveXfer.configFlags = kDSPI_MasterPcs0;

    PRINTF("\r\nPerformance: baudrate  bits/frame  speed(Bytes/s)\r\n");
    for (j = 0; j < (sizeof(bitsList) / sizeof(bitsList[0])); j++)
    {
        /* Re-configure master and slave. */
        masterConfig.ctarConfig.bitsPerFrame = bitsList[j];
        slaveConfig.ctarConfig.bitsPerFrame = masterConfig.ctarConfig.bitsPerFrame;

        DSPI_MasterInit(EXAMPLE_DSPI_MASTER_BASEADDR, &masterConfig, sourceClock);
        DSPI_SlaveInit(EXAMPLE_DSPI_SLAVE_BASEADDR, &slaveConfig);

        /* Init buffer. */
        for (i = 0; i < BUFFER_SIZE; i++)
        {
            masterSendBuffer[i] = (i + 1) % 16;
            slaveSendBuffer[i] = 15 - i % 16;
            slaveReceiveBuffer[i] = 0;
            masterReceiveBuffer[i] = 0;
            /* When testing > 8 bits/frame, every other byte is only (bit/frame-8)bits in
             * length hence need to mask off upper bits.
             */
            if (bitsList[j] > 8)
            {
                masterSendBuffer[i] = ((i + 1) % 16U) & (~(0xFFU << (bitsList[j] - 8)));
                slaveSendBuffer[i] = (15 - i % 16U) & (~(0xFFU << (bitsList[j] - 8)));
            }
        }

        /* Initialize transfer complete flag and install callback. */
        isTransferCompleted = false;

        /*Slave Start transfer. */
        if (master_mode == kDMA)
        {
            //            DSPI_SlaveSetTransferCallback(&(g_dspi_edma_s_handle.dspiHandle), dspi_slave_callback,
            //                                          &isTransferCompleted);
            status = DSPI_SlaveTransferEDMA(EXAMPLE_DSPI_SLAVE_BASEADDR, &g_dspi_edma_s_handle, &slaveXfer);
        }
        else
        {
            //            DSPI_SlaveSetTransferCallback(&g_s_handle, dspi_slave_callback, &isTransferCompleted);
            status = DSPI_SlaveTransferNonBlocking(EXAMPLE_DSPI_SLAVE_BASEADDR, &g_s_handle, &slaveXfer);
        }
        TEST_ASSERT_TRUE(status == kStatus_Success);

        /* Init and start timer*/
        timerInit();
        timerStart();
        time1 = getTimerCount();

        /* Master execute transfer. */
        switch (master_mode)
        {
            case kPolling:
                status = DSPI_MasterTransferBlocking(EXAMPLE_DSPI_MASTER_BASEADDR, &g_m_handle, &masterXfer);
                break;
            case kInterrupt:
                status = DSPI_MasterTransferNonBlocking(EXAMPLE_DSPI_MASTER_BASEADDR, &g_m_handle, &masterXfer);
                break;
            case kDMA:
#if defined(FSL_FEATURE_SOC_EDMA_COUNT) && (FSL_FEATURE_SOC_EDMA_COUNT)
                status = DSPI_MasterTransferEDMA(EXAMPLE_DSPI_MASTER_BASEADDR, &g_dspi_edma_m_handle, &masterXfer);
#endif
                break;
            default:
                break;
        }

        /* Wait slave to transfer all data. */
        while (!isTransferCompleted)
        {
        }

        /* Get used time and stop timer. */
        time2 = getTimerCount();
        timerStop();

        TEST_ASSERT_TRUE(status == kStatus_Success);

        for (i = 0; i < BUFFER_SIZE; i++)
        {
            TEST_ASSERT_TRUE(slaveReceiveBuffer[i] == masterSendBuffer[i]);
            TEST_ASSERT_TRUE(masterReceiveBuffer[i] == slaveSendBuffer[i]);
        }

        usedTime = COUNT_TO_USEC((time2 - time1), CLOCK_GetFreq(BUS_CLK));
        /* Calculate baudrate. */
        PRINTF("\r%21d %10d %10d\r\n", calculatedBaudRate, bitsList[j], (1000000U / usedTime) * BUFFER_SIZE);
    }

    /* Release DMA channel*/
    if (master_mode == kDMA)
    {
        DMAMGR_ReleaseChannel(&dspiEdmaMasterRxRegToRxDataHandle);
        DMAMGR_ReleaseChannel(&dspiEdmaMasterIntermediaryToTxRegHandle);
        DMAMGR_ReleaseChannel(&dspiEdmaMasterTxDataToIntermediaryHandle);

        DMAMGR_ReleaseChannel(&dspiEdmaSlaveRxRegToRxDataHandle);
        DMAMGR_ReleaseChannel(&dspiEdmaSlaveTxDataToTxRegHandle);

        DMAMGR_Deinit();
    }
}

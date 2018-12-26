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

#define TRANSFER_SIZE (32U)      /* Tranfer size in function test. */
#define MIN_BITS_PER_FRAME (4U)  /* minimum bits/frame */
#define MAX_BITS_PER_FRAME (16U) /* maximum bits/frame */
#define MAX_COUNT (6U)           /* Test cycle in vary transfer count test case. */

/*******************************************************************************
 * Variables
 ******************************************************************************/

uint8_t g_StepSize = 1; /* Bits/frame step size. Used for bits/frame iteration.*/

static bool isTransferCompleted;

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
/*
 * @brief Vary transfer count and bits/frame with or without using RxBuff.
 * Slave uses Interrupt or DMA mode, master uses Interrupt, DMA or Polling mode.
 * @param master_mode master transfer mode
 * @param useRxBuff master send with receive or not.
 * @param cpol clock polarity
 * @param cpha clock phase
 * @param isContinuousSCK use continuous SCK or not. Continuous SCK only work with cpha = 1;
 */
static void DSPI_test_vary_transferCnt_bitsPerFrame(dspi_transfer_mode_t master_mode,
                                                    bool useRxBuff,
                                                    dspi_clock_polarity_t cpol,
                                                    dspi_clock_phase_t cpha,
                                                    bool isContinuousSCK);
/*
 * @brief Common function used in this project, do transfer using different transfer mode.
 * This function will block and wait tranfer complete.
 */
static void DSPI_test_exeTransfer(dspi_transfer_t *masterXfer,
                                  dspi_transfer_t *slaveXfer,
                                  dspi_transfer_mode_t master_mode);
/*
 * @brief Basic usecase, first master send slave receive, then slave send master receive, and
 * then master send receive.
 */
static void DSPI_test_master_Slave(dspi_transfer_mode_t master_mode);

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

static void DSPI_test_exeTransfer(dspi_transfer_t *masterXfer,
                                  dspi_transfer_t *slaveXfer,
                                  dspi_transfer_mode_t master_mode)
{
    uint32_t status = kStatus_Fail;

    /* Initialize transfer complete flag and install callback. */
    isTransferCompleted = false;
    if (master_mode == kDMA)
    {
        //        DSPI_SlaveSetTransferCallback(&(g_dspi_edma_s_handle.dspiHandle), dspi_slave_callback,
        //        &isTransferCompleted);
        status = DSPI_SlaveTransferEDMA(EXAMPLE_DSPI_SLAVE_BASEADDR, &g_dspi_edma_s_handle, slaveXfer);
    }
    else
    {
        /* Setup transfer finished callback.*/
        //        DSPI_SlaveSetTransferCallback(&g_s_handle, dspi_slave_callback, &isTransferCompleted);
        /* Begin transfer. */
        status = DSPI_SlaveTransferNonBlocking(EXAMPLE_DSPI_SLAVE_BASEADDR, &g_s_handle, slaveXfer);
    }
    TEST_ASSERT_TRUE(status == kStatus_Success);

    switch (master_mode)
    {
        case kPolling:
            status = DSPI_MasterTransferBlocking(EXAMPLE_DSPI_MASTER_BASEADDR, &g_m_handle, masterXfer);
            break;
        case kInterrupt:
            status = DSPI_MasterTransferNonBlocking(EXAMPLE_DSPI_MASTER_BASEADDR, &g_m_handle, masterXfer);
            break;
        case kDMA:
            status = DSPI_MasterTransferEDMA(EXAMPLE_DSPI_MASTER_BASEADDR, &g_dspi_edma_m_handle, masterXfer);
            break;
        default:
            break;
    }
    /* Wait slave transferd all data. */
    while (!isTransferCompleted)
    {
    }
    TEST_ASSERT_TRUE(status == kStatus_Success);
}

static void DSPI_test_master_Slave(dspi_transfer_mode_t master_mode)
{
    uint32_t sourceClock;
    uint32_t i;

    dspi_master_config_t masterConfig;
    dspi_slave_config_t slaveConfig;

    dspi_transfer_t masterXfer;
    dspi_transfer_t slaveXfer;
    /* Master configure. */
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

    /* Slave configure.*/
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

        /* Setup transfer finished callback.*/
        //        DSPI_SlaveSetTransferCallback(&g_s_handle, dspi_slave_callback, &isTransferCompleted);

        DSPI_MasterTransferCreateHandleEDMA(
            EXAMPLE_DSPI_MASTER_BASEADDR, &g_dspi_edma_m_handle, NULL, NULL, &dspiEdmaMasterRxRegToRxDataHandle,
            &dspiEdmaMasterTxDataToIntermediaryHandle, &dspiEdmaMasterIntermediaryToTxRegHandle);
    }

    /* Init buffer. */
    for (i = 0; i < TRANSFER_SIZE; i++)
    {
        masterSendBuffer[i] = i % 16; // i is 0-15, in case bits/frame is not 8 or 16
        slaveReceiveBuffer[i] = 0;

        masterReceiveBuffer[i] = 0;
        slaveSendBuffer[i] = 15 - (i % 16);
    }

    /* Test 1. Master send, slave receive. */
    masterXfer.txData = masterSendBuffer;
    masterXfer.rxData = NULL;
    masterXfer.dataSize = TRANSFER_SIZE;
    masterXfer.configFlags = kDSPI_MasterCtar0 | kDSPI_MasterPcs0; //| kDSPI_MasterPcsContinuous;

    slaveXfer.txData = NULL;
    slaveXfer.rxData = slaveReceiveBuffer;
    slaveXfer.dataSize = TRANSFER_SIZE;
    slaveXfer.configFlags = kDSPI_MasterPcs0;

    /* Begin transfer. */
    DSPI_test_exeTransfer(&masterXfer, &slaveXfer, master_mode);

    for (i = 0; i < TRANSFER_SIZE; i++)
    {
        TEST_ASSERT_TRUE(slaveReceiveBuffer[i] == masterSendBuffer[i]);
    }

    /* Test 2. slave send, master receive. */
    masterXfer.txData = NULL;
    masterXfer.rxData = masterReceiveBuffer;
    masterXfer.dataSize = TRANSFER_SIZE;
    masterXfer.configFlags = kDSPI_MasterCtar0 | kDSPI_MasterPcs0 | kDSPI_MasterPcsContinuous;

    slaveXfer.txData = slaveSendBuffer;
    slaveXfer.rxData = NULL;
    slaveXfer.dataSize = TRANSFER_SIZE;
    slaveXfer.configFlags = kDSPI_MasterPcs0;

    DSPI_test_exeTransfer(&masterXfer, &slaveXfer, master_mode);

    for (i = 0; i < TRANSFER_SIZE; i++)
    {
        TEST_ASSERT_TRUE(slaveSendBuffer[i] == masterReceiveBuffer[i]);
    }

    /* Test 3. master send and receive*/
    for (i = 0; i < TRANSFER_SIZE; i++)
    {
        masterSendBuffer[i] = i % 16;
        slaveReceiveBuffer[i] = 0;

        masterReceiveBuffer[i] = 0;
        slaveSendBuffer[i] = 15 - (i % 16);
    }

    masterXfer.txData = masterSendBuffer;
    masterXfer.rxData = masterReceiveBuffer;
    masterXfer.dataSize = TRANSFER_SIZE;
    masterXfer.configFlags = kDSPI_MasterCtar0 | kDSPI_MasterPcs0 | kDSPI_MasterPcsContinuous;

    slaveXfer.txData = slaveSendBuffer;
    slaveXfer.rxData = slaveReceiveBuffer;
    slaveXfer.dataSize = TRANSFER_SIZE;
    slaveXfer.configFlags = kDSPI_SlaveCtar0;

    DSPI_test_exeTransfer(&masterXfer, &slaveXfer, master_mode);

    for (i = 0; i < TRANSFER_SIZE; i++)
    {
        TEST_ASSERT_TRUE(slaveReceiveBuffer[i] == masterSendBuffer[i]);
        TEST_ASSERT_TRUE(slaveSendBuffer[i] == masterReceiveBuffer[i]);
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

static void DSPI_test_vary_transferCnt_bitsPerFrame(dspi_transfer_mode_t master_mode,
                                                    bool useRxBuff,
                                                    dspi_clock_polarity_t cpol,
                                                    dspi_clock_phase_t cpha,
                                                    bool isContinuousSCK)
{
    uint32_t sourceClock;
    uint32_t errorCode;
    uint32_t i, j;

    uint32_t transferByteCnt;
    uint32_t bitsPerFrame;

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
    masterConfig.ctarConfig.cpol = cpol;
    masterConfig.ctarConfig.cpha = cpha;
    masterConfig.ctarConfig.direction = kDSPI_MsbFirst;
    masterConfig.ctarConfig.pcsToSckDelayInNanoSec = 2000;
    masterConfig.ctarConfig.lastSckToPcsDelayInNanoSec = 2000;
    masterConfig.ctarConfig.betweenTransferDelayInNanoSec = 1000;
    masterConfig.whichPcs = kDSPI_Pcs0;
    masterConfig.pcsActiveHighOrLow = kDSPI_PcsActiveLow;
    masterConfig.enableContinuousSCK = isContinuousSCK;
    masterConfig.enableRxFifoOverWrite = false;
    masterConfig.enableModifiedTimingFormat = false;
    masterConfig.samplePoint = kDSPI_SckToSin0Clock;

    sourceClock = CLOCK_GetFreq(DSPI_MASTER_CLK_SRC);

    DSPI_MasterInit(EXAMPLE_DSPI_MASTER_BASEADDR, &masterConfig, sourceClock);
    DSPI_MasterTransferCreateHandle(EXAMPLE_DSPI_MASTER_BASEADDR, &g_m_handle, NULL, NULL);

    /*
     * slaveConfig.enableContinuousSCK = false;
     * slaveConfig.enableRxFifoOverWrite = false;
     * slaveConfig.enableModifiedTimingFormat = false;
     * slaveConfig.samplePoint = kDSPI_SckToSin0Clock;
     */
    DSPI_SlaveGetDefaultConfig(&slaveConfig);
    slaveConfig.whichCtar = kDSPI_Ctar0;
    slaveConfig.ctarConfig.cpol = masterConfig.ctarConfig.cpol;
    slaveConfig.ctarConfig.cpha = masterConfig.ctarConfig.cpha;
    slaveConfig.enableContinuousSCK = masterConfig.enableContinuousSCK;
    slaveConfig.enableRxFifoOverWrite = masterConfig.enableRxFifoOverWrite;
    slaveConfig.enableModifiedTimingFormat = masterConfig.enableModifiedTimingFormat;
    slaveConfig.samplePoint = masterConfig.samplePoint;

    DSPI_SlaveInit(EXAMPLE_DSPI_SLAVE_BASEADDR, &slaveConfig);
    DSPI_SlaveTransferCreateHandle(EXAMPLE_DSPI_SLAVE_BASEADDR, &g_s_handle, dspi_slave_callback, &isTransferCompleted);

    masterXfer.txData = masterSendBuffer;
    masterXfer.configFlags = kDSPI_MasterCtar0 | kDSPI_MasterPcs0 | kDSPI_MasterPcsContinuous;
    ;

    slaveXfer.configFlags = kDSPI_SlaveCtar0;
    slaveXfer.rxData = slaveReceiveBuffer;

    if (useRxBuff) // use receive buffer or not.
    {
        masterXfer.rxData = masterReceiveBuffer;
        slaveXfer.txData = slaveSendBuffer;
    }
    else
    {
        masterXfer.rxData = NULL;
        slaveXfer.txData = NULL;
    }

    /* Test odd and even count of data transfer with different bits/frame. */
    for (j = 1; j < TRANSFER_SIZE / 2; j++)
    {
        transferByteCnt = j;

        /* Needless to test too much tranfer count case. */
        if (j > MAX_COUNT)
        {
            break;
        }

        errorCode = 0;

        /* Vary bits/frame. */
        for (bitsPerFrame = MIN_BITS_PER_FRAME; bitsPerFrame <= MAX_BITS_PER_FRAME; bitsPerFrame += g_StepSize)
        {
            masterConfig.ctarConfig.bitsPerFrame = bitsPerFrame;
            slaveConfig.ctarConfig.bitsPerFrame = bitsPerFrame;

            masterXfer.dataSize = transferByteCnt;
            slaveXfer.dataSize = transferByteCnt;

            DSPI_MasterInit(EXAMPLE_DSPI_MASTER_BASEADDR, &masterConfig, sourceClock);
            DSPI_SlaveInit(EXAMPLE_DSPI_SLAVE_BASEADDR, &slaveConfig);

            /*
             *Slave EDMA transfer cannot support the situation that transfer_size is 1 when the bitsPerFrame is greater
             * than 8.
             */
            if ((master_mode == kDMA) && (transferByteCnt < 2) && (bitsPerFrame > 8))
            {
                continue;
            }

            /* Init buffer. */
            for (i = 0; i < transferByteCnt; i++)
            {
                masterSendBuffer[i] = (i + 1) % 16;
                slaveSendBuffer[i] = 15 - i % 16;
                slaveReceiveBuffer[i] = 0;
                masterReceiveBuffer[i] = 0;
                /* When testing > 8 bits/frame, every other byte is only (bit/frame-8)bits in
                 * length hence need to mask off upper bits.
                 */
                if (bitsPerFrame > 8)
                {
                    masterSendBuffer[i] = ((i + 1) % 16U) & (~(0xFFU << (bitsPerFrame - 8)));
                    slaveSendBuffer[i] = (15 - i % 16U) & (~(0xFFU << (bitsPerFrame - 8)));
                }
            }

            /* Do transfer*/
            if (useRxBuff)
            {
                /* Do transfer: master slave send and receive. */
                DSPI_test_exeTransfer(&masterXfer, &slaveXfer, master_mode);
            }
            else
            {
                /* Do transfer: master send slave receive. */
                masterXfer.rxData = NULL;
                slaveXfer.txData = NULL;
                masterXfer.txData = masterSendBuffer;
                slaveXfer.rxData = slaveReceiveBuffer;

                DSPI_test_exeTransfer(&masterXfer, &slaveXfer, master_mode);

                /* Do transfer: slave send master receive. */
                masterXfer.rxData = masterReceiveBuffer;
                slaveXfer.txData = slaveSendBuffer;
                masterXfer.txData = NULL;
                slaveXfer.rxData = NULL;

                DSPI_test_exeTransfer(&masterXfer, &slaveXfer, master_mode);
            }

            /* Verify transfered data. */
            for (i = 0; i < transferByteCnt; i++)
            {
                if ((masterSendBuffer[i] != slaveReceiveBuffer[i]) || (slaveSendBuffer[i] != masterReceiveBuffer[i]))
                {
                    errorCode |= 1U << bitsPerFrame; // record the error state.
                }
            }
        }
        /* Print transferByteCnt transfer result. */
        for (i = MIN_BITS_PER_FRAME; i <= MAX_BITS_PER_FRAME; i++)
        {
            if (errorCode & (1U << i))
            {
                PRINTF("DSPI vary transfer count test: %d bits/frame, %d Bytes transfered %d using RxBuff: FAILED\r\n",
                       i, transferByteCnt, useRxBuff);
            }
        }
        TEST_ASSERT_TRUE(errorCode == 0);
    }
}

void DSPI_test_masterPolling_SlaveInt(void)
{
    DSPI_test_master_Slave(kPolling);
}

void DSPI_test_masterInt_SlaveInt(void)
{
    DSPI_test_master_Slave(kInterrupt);
}

void DSPI_test_masterDMA_SlaveDMA(void)
{
    DSPI_test_master_Slave(kDMA);
}

void DSPI_test_vary_transfer_MasterSendPolling(void)
{
    g_StepSize = 4;

    DSPI_test_vary_transferCnt_bitsPerFrame(kPolling, false, kDSPI_ClockPolarityActiveHigh, kDSPI_ClockPhaseFirstEdge,
                                            false);
}

void DSPI_test_vary_transfer_MasterSendInterrupt(void)
{
    g_StepSize = 4;

    DSPI_test_vary_transferCnt_bitsPerFrame(kInterrupt, false, kDSPI_ClockPolarityActiveHigh, kDSPI_ClockPhaseFirstEdge,
                                            false);
}

void DSPI_test_vary_transfer_MasterSendDMA(void)
{
    g_StepSize = 4;

    DSPI_test_vary_transferCnt_bitsPerFrame(kDMA, false, kDSPI_ClockPolarityActiveHigh, kDSPI_ClockPhaseFirstEdge,
                                            false);
}

void DSPI_test_vary_transfer_MasterSendReceivePolling(void)
{
    g_StepSize = 1;

    DSPI_test_vary_transferCnt_bitsPerFrame(kPolling, true, kDSPI_ClockPolarityActiveHigh, kDSPI_ClockPhaseFirstEdge,
                                            false);
}

void DSPI_test_vary_transfer_MasterSendReceiveInterrupt(void)
{
    g_StepSize = 1;

    DSPI_test_vary_transferCnt_bitsPerFrame(kInterrupt, true, kDSPI_ClockPolarityActiveHigh, kDSPI_ClockPhaseFirstEdge,
                                            false);
}

void DSPI_test_vary_transfer_MasterSendReceiveDMA(void)
{
    g_StepSize = 8;

    DSPI_test_vary_transferCnt_bitsPerFrame(kDMA, true, kDSPI_ClockPolarityActiveHigh, kDSPI_ClockPhaseFirstEdge,
                                            false);
}

/* Extra test. This case just tested transfer without FIFO. */
void DSPI_test_vary_polarity_phase(void)
{
    g_StepSize = 4;

    DSPI_test_vary_transferCnt_bitsPerFrame(kPolling, true, kDSPI_ClockPolarityActiveHigh, kDSPI_ClockPhaseFirstEdge,
                                            false);
    DSPI_test_vary_transferCnt_bitsPerFrame(kInterrupt, true, kDSPI_ClockPolarityActiveHigh, kDSPI_ClockPhaseSecondEdge,
                                            false);
    DSPI_test_vary_transferCnt_bitsPerFrame(kPolling, true, kDSPI_ClockPolarityActiveLow, kDSPI_ClockPhaseFirstEdge,
                                            false);
    DSPI_test_vary_transferCnt_bitsPerFrame(kInterrupt, true, kDSPI_ClockPolarityActiveLow, kDSPI_ClockPhaseSecondEdge,
                                            false);
}

void DSPI_test_transfer_continuousSCK(void)
{
    /*bits/frame step by 4. */
    g_StepSize = 4;

    /* Note:continuous SCK is only supported for CPHA = 1 */
    DSPI_test_vary_transferCnt_bitsPerFrame(kInterrupt, true, kDSPI_ClockPolarityActiveHigh, kDSPI_ClockPhaseSecondEdge,
                                            false);
    //    DSPI_test_vary_transferCnt_bitsPerFrame(kPolling, true, kDSPI_ClockPolarityActiveLow,
    //    kDSPI_ClockPhaseSecondEdge,
    //                                            true);
}

/* Test ctar1 and clock polarity and clock phase RX and TX without using FIFO. */
void DSPI_test_vary_legacy(void)
{
    uint32_t sourceClock;
    uint32_t i;

    dspi_master_config_t masterConfig;
    dspi_slave_config_t slaveConfig;
    dspi_transfer_t masterXfer;
    dspi_transfer_t slaveXfer;

    /*
     * masterConfig.whichPcs = kDSPI_Pcs0;
     * masterConfig.pcsActiveHighOrLow = kDSPI_PcsActiveLow;
     * masterConfig.enableContinuousSCK = false;
     * masterConfig.enableRxFifoOverWrite = false;
     * masterConfig.enableModifiedTimingFormat = false;
     * masterConfig.samplePoint = kDSPI_SckToSin0Clock;
     */
    DSPI_MasterGetDefaultConfig(&masterConfig);
    masterConfig.whichCtar = kDSPI_Ctar1;
    masterConfig.ctarConfig.baudRate = g_dspi_baudrate;
    masterConfig.ctarConfig.bitsPerFrame = 8;
    masterConfig.ctarConfig.cpol = kDSPI_ClockPolarityActiveLow;
    masterConfig.ctarConfig.cpha = kDSPI_ClockPhaseSecondEdge;
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

    /* Disable FIFO*/
    DSPI_SetFifoEnable(EXAMPLE_DSPI_SLAVE_BASEADDR, false, false);
    DSPI_SetFifoEnable(EXAMPLE_DSPI_MASTER_BASEADDR, false, false);

    memset(&masterXfer, 0, sizeof(masterXfer));
    memset(&slaveXfer, 0, sizeof(slaveXfer));

    for (i = 1; i < TRANSFER_SIZE; i++)
    {
        masterSendBuffer[i] = i % 256;
        slaveReceiveBuffer[i] = 0;

        masterReceiveBuffer[i] = 0;
        slaveSendBuffer[i] = ~(i % 256);
    }

    masterXfer.txData = masterSendBuffer;
    masterXfer.rxData = masterReceiveBuffer;
    masterXfer.dataSize = TRANSFER_SIZE;
    masterXfer.configFlags = kDSPI_MasterCtar1 | kDSPI_MasterPcs0 | kDSPI_MasterPcsContinuous;

    slaveXfer.txData = slaveSendBuffer;
    slaveXfer.rxData = slaveReceiveBuffer;
    slaveXfer.dataSize = TRANSFER_SIZE;
    slaveXfer.configFlags = kDSPI_SlaveCtar0;

    DSPI_test_exeTransfer(&masterXfer, &slaveXfer, kInterrupt);

    for (i = 0; i < TRANSFER_SIZE; i++)
    {
        TEST_ASSERT_TRUE(slaveReceiveBuffer[i] == masterSendBuffer[i]);
        TEST_ASSERT_TRUE(slaveSendBuffer[i] == masterReceiveBuffer[i]);
    }
}

/* master and slave both use polling method. */
void DSPI_test_masterSlaveBlocking(void)
{
    uint32_t sourceClock;
    uint32_t i;

    SPI_Type *masterBase = EXAMPLE_DSPI_MASTER_BASEADDR;
    SPI_Type *slaveBase = EXAMPLE_DSPI_SLAVE_BASEADDR;
    dspi_master_config_t masterConfig;
    dspi_slave_config_t slaveConfig;

    dspi_command_data_config_t command; // CTAR command

    memset(&masterConfig, 0, sizeof(masterConfig));
    memset(&slaveConfig, 0, sizeof(slaveConfig));

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
    DSPI_SlaveTransferCreateHandle(EXAMPLE_DSPI_SLAVE_BASEADDR, &g_s_handle, NULL, NULL);

    /* Init buffer. */
    for (i = 0; i < TRANSFER_SIZE; i++)
    {
        masterSendBuffer[i] = i % 256; // i is 0-15, in case bits/frame is not 8 or 16
        slaveReceiveBuffer[i] = 0;

        masterReceiveBuffer[i] = 0;
        slaveSendBuffer[i] = i % 256 + 1;
    }

    /* Slave transfer init. */
    DSPI_StopTransfer(slaveBase);
    DSPI_FlushFifo(slaveBase, true, true);
    DSPI_ClearStatusFlags(slaveBase, kDSPI_AllStatusFlag);
    DSPI_StartTransfer(slaveBase);

    command.whichPcs = kDSPI_Pcs0;
    command.isPcsContinuous = true;
    command.isEndOfQueue = false;
    command.clearTransferCount = false;
    command.whichCtar = kDSPI_Ctar0;

    DSPI_StopTransfer(masterBase);
    DSPI_FlushFifo(masterBase, true, true);
    DSPI_ClearStatusFlags(masterBase, kDSPI_AllStatusFlag);
    DSPI_StartTransfer(masterBase);

    for (i = 0; i < TRANSFER_SIZE; i++)
    {
        /* Master send. */
        while (!(DSPI_GetStatusFlags(masterBase) & kDSPI_TxFifoFillRequestFlag))
        {
        }
        DSPI_MasterWriteDataBlocking(masterBase, &command, (uint16_t)masterSendBuffer[i]);
        DSPI_ClearStatusFlags(masterBase, kDSPI_TxFifoFillRequestFlag);
        while (!(DSPI_GetStatusFlags(masterBase) & kDSPI_TxCompleteFlag))
        {
        }

        /* Slave receive.*/
        while (!(DSPI_GetStatusFlags(slaveBase) & kDSPI_RxFifoDrainRequestFlag))
        {
        }
        slaveReceiveBuffer[i] = DSPI_ReadData(slaveBase);
        DSPI_ClearStatusFlags(slaveBase, kDSPI_RxFifoDrainRequestFlag);
    }
    for (i = 0; i < TRANSFER_SIZE; i++)
    {
        TEST_ASSERT_TRUE(masterSendBuffer[i] == slaveReceiveBuffer[i]);
    }
}

/* Legacy API test. */
void DSPI_test_legacy_API(void)
{
    dspi_master_config_t masterConfig;
    dspi_slave_config_t slaveConfig;
    uint32_t sourceClock;

    /* Master config, use default config. */
    /*
     * masterConfig.whichPcs = kDSPI_Pcs0;
     * masterConfig.pcsActiveHighOrLow = kDSPI_PcsActiveLow;
     * masterConfig.enableContinuousSCK = false;
     * masterConfig.enableRxFifoOverWrite = false;
     * masterConfig.enableModifiedTimingFormat = false;
     * masterConfig.samplePoint = kDSPI_SckToSin0Clock;
     */
    DSPI_MasterGetDefaultConfig(&masterConfig);
    sourceClock = CLOCK_GetFreq(DSPI_MASTER_CLK_SRC);
    DSPI_MasterInit(EXAMPLE_DSPI_MASTER_BASEADDR, &masterConfig, sourceClock);

    /* Slave config, use default config. */
    /*
     * slaveConfig.enableContinuousSCK = false;
     * slaveConfig.enableRxFifoOverWrite = false;
     * slaveConfig.enableModifiedTimingFormat = false;
     * slaveConfig.samplePoint = kDSPI_SckToSin0Clock;
     */
    DSPI_SlaveGetDefaultConfig(&slaveConfig);
    DSPI_SlaveInit(EXAMPLE_DSPI_SLAVE_BASEADDR, &slaveConfig);

    DSPI_SetFifoEnable(EXAMPLE_DSPI_MASTER_BASEADDR, false, false); // disable FIFO

    TEST_ASSERT_TRUE((EXAMPLE_DSPI_MASTER_BASEADDR->MCR & SPI_MCR_DIS_RXF_MASK) == SPI_MCR_DIS_RXF_MASK);
    TEST_ASSERT_TRUE((EXAMPLE_DSPI_MASTER_BASEADDR->MCR & SPI_MCR_DIS_TXF_MASK) == SPI_MCR_DIS_TXF_MASK);
}

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

#ifndef _FSL_DSPI_TEST_H_
#define _FSL_DSPI_TEST_H_

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "board.h"
#include "fsl_debug_console.h"
#include "fsl_dspi.h"
#if defined(FSL_FEATURE_SOC_PIT_COUNT) && (FSL_FEATURE_SOC_PIT_COUNT)
#include "fsl_pit.h"
#endif /* FSL_FEATURE_SOC_PIT_COUNT */
#include "unity.h"
#include "app.h"
#include "fsl_dmamux.h"
#include "fsl_dspi_edma.h"
#include "fsl_edma.h"
#include "fsl_dma_manager.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define EXAMPLE_DSPI_MASTER_BASEADDR ((SPI_Type *)EXAMPLE_DSPI_MASTER_BASE)
#define EXAMPLE_DSPI_SLAVE_BASEADDR ((SPI_Type *)EXAMPLE_DSPI_SLAVE_BASE)

#define BUFFER_SIZE (256U)

#if (EXAMPLE_DSPI_MASTER_BASE == SPI0_BASE)
#define DSPI_MASTER_CLK_SRC (DSPI0_CLK_SRC)
#elif(EXAMPLE_DSPI_MASTER_BASE == SPI1_BASE)
#define DSPI_MASTER_CLK_SRC (DSPI1_CLK_SRC)
#elif(EXAMPLE_DSPI_MASTER_BASE == SPI2_BASE)
#define DSPI_MASTER_CLK_SRC (DSPI2_CLK_SRC)
#elif(EXAMPLE_DSPI_MASTER_BASE == SPI3_BASE)
#define DSPI_MASTER_CLK_SRC (DSPI3_CLK_SRC)
#elif(EXAMPLE_DSPI_MASTER_BASE == SPI4_BASE)
#define DSPI_MASTER_CLK_SRC (DSPI4_CLK_SRC)
#else
#error Should define the DSPI_MASTER_CLK_SRC!
#endif /* EXAMPLE_DSPI_MASTER_BASE == SPI0_BASE */

typedef enum _dspi_transfer_mode_t
{
    kPolling = 0,
    kInterrupt,
    kDMA
} dspi_transfer_mode_t;

#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus */

/*******************************************************************************
 * Variables
 ******************************************************************************/
/* Handle used by transcation API. */
extern dspi_master_handle_t g_m_handle;
extern dspi_slave_handle_t g_s_handle;
extern dspi_master_edma_handle_t g_dspi_edma_m_handle;
extern dspi_slave_edma_handle_t g_dspi_edma_s_handle;

extern edma_handle_t dspiEdmaMasterRxRegToRxDataHandle;
extern edma_handle_t dspiEdmaMasterTxDataToIntermediaryHandle;
extern edma_handle_t dspiEdmaMasterIntermediaryToTxRegHandle;

extern edma_handle_t dspiEdmaSlaveRxRegToRxDataHandle;
extern edma_handle_t dspiEdmaSlaveTxDataToTxRegHandle;

/* Baud Rate. */
extern uint32_t g_baudrate[];
extern uint32_t g_dspi_baudrate;

/* Buffers used in tranfer. */
extern uint8_t masterReceiveBuffer[BUFFER_SIZE];
extern uint8_t masterSendBuffer[BUFFER_SIZE];
extern uint8_t slaveReceiveBuffer[BUFFER_SIZE];
extern uint8_t slaveSendBuffer[BUFFER_SIZE];

/* DMA software transfer control descriptor */
extern edma_tcd_t dspiMaterSoftwareTcd;
extern edma_tcd_t dspiSlaveSoftwareTcd;

/*******************************************************************************
 * API
 ******************************************************************************/

void setUp(void);
void tearDown(void);

/* Basic API test.Test potential uncovered API. */
void DSPI_test_legacy_API(void);

/*
 *Basic functional common usecase test.
 *  Covered: master -->  slave
 *          master <--  slave
 *          master <--> slave
 */
void DSPI_test_masterPolling_SlaveInt(void);
void DSPI_test_masterInt_SlaveInt(void);
void DSPI_test_masterDMA_SlaveDMA(void);
void DSPI_test_masterSlaveBlocking(void); // Both slave and master polling.

/* Vary transfer count with different bits/frame using Polling, Interrupt, DMA. */
void DSPI_test_vary_transfer_MasterSendPolling(void);
void DSPI_test_vary_transfer_MasterSendInterrupt(void);
void DSPI_test_vary_transfer_MasterSendDMA(void);
void DSPI_test_vary_transfer_MasterSendReceivePolling(void);
void DSPI_test_vary_transfer_MasterSendReceiveInterrupt(void);
void DSPI_test_vary_transfer_MasterSendReceiveDMA(void);

/*
 *Vary polarity and phase.
 *Covered: clock polarity and phase
 *        no RX and TX FIFO
*/
void DSPI_test_vary_polarity_phase(void);

/* Test continuous SCK mode. */
void DSPI_test_transfer_continuousSCK(void);

/* Test performance. */
void DSPI_test_transferPerformancePolling(void);
void DSPI_test_transferPerformanceInterrupt(void);
void DSPI_test_transferPerformanceDMA(void);

/* Extra test. This case just tested transfer using CTAR1 without FIFO. */
void DSPI_test_vary_legacy(void);

#if defined(__cplusplus)
}
#endif /* __cplusplus */
#endif /*_FSL_DSPI_CONFIG_H_*/

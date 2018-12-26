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

#include "fsl_dspi_dma.h"

/***********************************************************************************************************************
* Definitions
***********************************************************************************************************************/

/* Component ID definition, used by tools. */
#ifndef FSL_COMPONENT_ID
#define FSL_COMPONENT_ID "platform.drivers.dspi_dma"
#endif

/*!
* @brief Structure definition for dspi_master_dma_private_handle_t. The structure is private.
*/
typedef struct _dspi_master_dma_private_handle
{
    SPI_Type *base;                   /*!< DSPI peripheral base address. */
    dspi_master_dma_handle_t *handle; /*!< dspi_master_dma_handle_t handle */
} dspi_master_dma_private_handle_t;

/*!
* @brief Structure definition for dspi_slave_dma_private_handle_t. The structure is private.
*/
typedef struct _dspi_slave_dma_private_handle
{
    SPI_Type *base;                  /*!< DSPI peripheral base address. */
    dspi_slave_dma_handle_t *handle; /*!< dspi_master_dma_handle_t handle */
} dspi_slave_dma_private_handle_t;

/***********************************************************************************************************************
* Prototypes
***********************************************************************************************************************/
/*!
* @brief DMA_DspiMasteRxCallback after the DSPI master transfer completed by using DMA.
* This is not a public API.
*/
static void DMA_DspiMasteRxCallback(dma_handle_t *dmaHandle, void *g_dspiDmaPrivateHandle);

static void DMA_DspiMasteTxCallback(dma_handle_t *dmaHandle, void *g_dspiDmaPrivateHandle);

/*!
* @brief DMA_DspiSlaveRxCallback after the DSPI slave transfer completed by using DMA.
* This is not a public API.
*/
static void DMA_DspiSlaveRxCallback(dma_handle_t *dmaHandle, void *g_dspiDmaPrivateHandle);

/***********************************************************************************************************************
* Variables
***********************************************************************************************************************/

/*! @brief Pointers to dspi dma handles for each instance. */
static dspi_master_dma_private_handle_t s_dspiMasterDmaPrivateHandle[FSL_FEATURE_SOC_DSPI_COUNT];
static dspi_slave_dma_private_handle_t s_dspiSlaveDmaPrivateHandle[FSL_FEATURE_SOC_DSPI_COUNT];

/***********************************************************************************************************************
* Code
***********************************************************************************************************************/

void DSPI_MasterTransferCreateHandleDMA(SPI_Type *base,
                                        dspi_master_dma_handle_t *handle,
                                        dspi_master_dma_transfer_callback_t callback,
                                        void *userData,
                                        dma_handle_t *dmaRxRegToRxDataHandle,
                                        dma_handle_t *dmaTxDataToIntermediaryHandle,
                                        dma_handle_t *dmaIntermediaryToTxRegHandle)
{
    assert(handle);
    assert(dmaRxRegToRxDataHandle);
    assert(dmaTxDataToIntermediaryHandle);
    assert(dmaIntermediaryToTxRegHandle);

    /* Zero the handle. */
    memset(handle, 0, sizeof(*handle));

    uint32_t instance = DSPI_GetInstance(base);

    s_dspiMasterDmaPrivateHandle[instance].base = base;
    s_dspiMasterDmaPrivateHandle[instance].handle = handle;

    handle->callback = callback;
    handle->userData = userData;

    handle->dmaRxRegToRxDataHandle = dmaRxRegToRxDataHandle;
    handle->dmaTxDataToIntermediaryHandle = dmaTxDataToIntermediaryHandle;
    handle->dmaIntermediaryToTxRegHandle = dmaIntermediaryToTxRegHandle;
}

status_t DSPI_MasterTransferDMA(SPI_Type *base, dspi_master_dma_handle_t *handle, dspi_transfer_t *transfer)
{
    assert(handle);
    assert(transfer);

    /* If the transfer count is zero, then return immediately.*/
    if (transfer->dataSize == 0)
    {
        return kStatus_InvalidArgument;
    }

    /* If both send buffer and receive buffer is null */
    if ((!(transfer->txData)) && (!(transfer->rxData)))
    {
        return kStatus_InvalidArgument;
    }

    /* Check that we're not busy.*/
    if (handle->state == kDSPI_Busy)
    {
        return kStatus_DSPI_Busy;
    }

    uint32_t instance = DSPI_GetInstance(base);
    uint16_t wordToSend = 0;
    uint8_t dummyData = g_dspiDummyData[instance];
    uint8_t dataAlreadyFed = 0;
    uint8_t dataFedMax = 2;
    bool writeOneDataAfterStart = false;

    uint32_t rxAddr = DSPI_GetRxRegisterAddress(base);
    uint32_t txAddr = DSPI_MasterGetTxRegisterAddress(base);

    dma_transfer_config_t transferConfigA;
    dma_transfer_config_t transferConfigB;
    dma_transfer_config_t transferConfigC;

    dma_channel_link_config_t channelConfig;

    handle->txBuffIfNull = ((uint32_t)dummyData << 8) | dummyData;

    handle->state = kDSPI_Busy;

    dspi_command_data_config_t commandStruct;
    DSPI_StopTransfer(base);
    DSPI_FlushFifo(base, true, true);
    DSPI_ClearStatusFlags(base, kDSPI_AllStatusFlag);

    commandStruct.whichPcs =
        (dspi_which_pcs_t)(1U << ((transfer->configFlags & DSPI_MASTER_PCS_MASK) >> DSPI_MASTER_PCS_SHIFT));
    commandStruct.isEndOfQueue = false;
    commandStruct.clearTransferCount = false;
    commandStruct.whichCtar =
        (dspi_ctar_selection_t)((transfer->configFlags & DSPI_MASTER_CTAR_MASK) >> DSPI_MASTER_CTAR_SHIFT);
    commandStruct.isPcsContinuous = (bool)(transfer->configFlags & kDSPI_MasterPcsContinuous);
    handle->command = DSPI_MasterGetFormattedCommand(&(commandStruct));

    commandStruct.isEndOfQueue = true;
    commandStruct.isPcsContinuous = (bool)(transfer->configFlags & kDSPI_MasterActiveAfterTransfer);
    handle->lastCommand = DSPI_MasterGetFormattedCommand(&(commandStruct));

    handle->bitsPerFrame = ((base->CTAR[commandStruct.whichCtar] & SPI_CTAR_FMSZ_MASK) >> SPI_CTAR_FMSZ_SHIFT) + 1;

    if ((handle->bitsPerFrame > 8) && ((transfer->dataSize % 2) != 0))
    {
        handle->state = kDSPI_Idle;
        return kStatus_InvalidArgument;
    }

    if ((base->MCR & SPI_MCR_DIS_RXF_MASK) || (base->MCR & SPI_MCR_DIS_TXF_MASK))
    {
        handle->fifoSize = 1;
    }
    else
    {
        handle->fifoSize = FSL_FEATURE_DSPI_FIFO_SIZEn(base);
    }

    handle->txData = transfer->txData;
    handle->rxData = transfer->rxData;
    handle->remainingSendByteCount = transfer->dataSize;
    handle->remainingReceiveByteCount = transfer->dataSize;
    handle->totalByteCount = transfer->dataSize;

    DSPI_DisableDMA(base, kDSPI_RxDmaEnable | kDSPI_TxDmaEnable);

    DMA_SetCallback(handle->dmaRxRegToRxDataHandle, DMA_DspiMasteRxCallback, &s_dspiMasterDmaPrivateHandle[instance]);

    DMA_SetCallback(handle->dmaIntermediaryToTxRegHandle, DMA_DspiMasteTxCallback,
                    &s_dspiMasterDmaPrivateHandle[instance]);

    /*If dspi has separate dma request , prepare the first data in "intermediary" .
    else (dspi has shared dma request) , send first 2 data if there is fifo or send first 1 data if there is no fifo*/
    if (1 == FSL_FEATURE_DSPI_HAS_SEPARATE_DMA_RX_TX_REQn(base))
    {
        /* For DSPI instances with separate RX/TX DMA requests, we'll use the TX DMA request to
        * trigger the TX DMA channel and RX DMA request to trigger the RX DMA channel
        */
        /*Prepare the firt data*/
        if (handle->bitsPerFrame > 8)
        {
            /* If it's the last word */
            if (handle->remainingSendByteCount <= 2)
            {
                if (handle->txData)
                {
                    wordToSend = *(handle->txData);
                    ++handle->txData; /* increment to next data byte */
                    wordToSend |= (unsigned)(*(handle->txData)) << 8U;
                }
                else
                {
                    wordToSend = ((uint32_t)dummyData << 8) | dummyData;
                }
                handle->lastCommand = (handle->lastCommand & 0xffff0000U) | wordToSend;
            }
            else /* For all words except the last word , frame > 8bits */
            {
                if (handle->txData)
                {
                    wordToSend = *(handle->txData);
                    ++handle->txData; /* increment to next data byte */
                    wordToSend |= (unsigned)(*(handle->txData)) << 8U;
                    ++handle->txData; /* increment to next data byte */
                }
                else
                {
                    wordToSend = ((uint32_t)dummyData << 8) | dummyData;
                }
                handle->command = (handle->command & 0xffff0000U) | wordToSend;
            }
        }
        else /* Optimized for bits/frame less than or equal to one byte. */
        {
            if (handle->txData)
            {
                wordToSend = *(handle->txData);
                ++handle->txData; /* increment to next data word*/
            }
            else
            {
                wordToSend = dummyData;
            }

            if (handle->remainingSendByteCount == 1)
            {
                handle->lastCommand = (handle->lastCommand & 0xffff0000U) | wordToSend;
            }
            else
            {
                handle->command = (handle->command & 0xffff0000U) | wordToSend;
            }
        }
    }

    else /*dspi has shared dma request*/

    {
        /* For DSPI instances with shared RX/TX DMA requests, we'll use the RX DMA request to
        * trigger ongoing transfers and will link to the TX DMA channel from the RX DMA channel.
        */

        /* If bits/frame is greater than one byte */
        if (handle->bitsPerFrame > 8)
        {
            while (DSPI_GetStatusFlags(base) & kDSPI_TxFifoFillRequestFlag)
            {
                if (handle->remainingSendByteCount <= 2)
                {
                    if (handle->txData)
                    {
                        wordToSend = *(handle->txData);
                        ++handle->txData;
                        wordToSend |= (unsigned)(*(handle->txData)) << 8U;
                    }
                    else
                    {
                        wordToSend = ((uint32_t)dummyData << 8) | dummyData;
                    }
                    handle->remainingSendByteCount = 0;
                    base->PUSHR = (handle->lastCommand & 0xffff0000U) | wordToSend;
                }
                /* For all words except the last word */
                else
                {
                    if (handle->txData)
                    {
                        wordToSend = *(handle->txData);
                        ++handle->txData;
                        wordToSend |= (unsigned)(*(handle->txData)) << 8U;
                        ++handle->txData;
                    }
                    else
                    {
                        wordToSend = ((uint32_t)dummyData << 8) | dummyData;
                    }
                    handle->remainingSendByteCount -= 2;
                    base->PUSHR = (handle->command & 0xffff0000U) | wordToSend;
                }

                /* Try to clear the TFFF; if the TX FIFO is full this will clear */
                DSPI_ClearStatusFlags(base, kDSPI_TxFifoFillRequestFlag);

                dataAlreadyFed += 2;

                /* exit loop if send count is zero, else update local variables for next loop */
                if ((handle->remainingSendByteCount == 0) || (dataAlreadyFed == (dataFedMax * 2)))
                {
                    break;
                }
            } /* End of TX FIFO fill while loop */
        }
        else /* Optimized for bits/frame less than or equal to one byte. */
        {
            while (DSPI_GetStatusFlags(base) & kDSPI_TxFifoFillRequestFlag)
            {
                if (handle->txData)
                {
                    wordToSend = *(handle->txData);
                    ++handle->txData;
                }
                else
                {
                    wordToSend = dummyData;
                }

                if (handle->remainingSendByteCount == 1)
                {
                    base->PUSHR = (handle->lastCommand & 0xffff0000U) | wordToSend;
                }
                else
                {
                    base->PUSHR = (handle->command & 0xffff0000U) | wordToSend;
                }

                /* Try to clear the TFFF; if the TX FIFO is full this will clear */
                DSPI_ClearStatusFlags(base, kDSPI_TxFifoFillRequestFlag);

                --handle->remainingSendByteCount;

                dataAlreadyFed++;

                /* exit loop if send count is zero, else update local variables for next loop */
                if ((handle->remainingSendByteCount == 0) || (dataAlreadyFed == dataFedMax))
                {
                    break;
                }
            } /* End of TX FIFO fill while loop */
        }
    }

    DMA_ResetChannel(handle->dmaRxRegToRxDataHandle->base, handle->dmaRxRegToRxDataHandle->channel);
    DMA_ResetChannel(handle->dmaTxDataToIntermediaryHandle->base, handle->dmaTxDataToIntermediaryHandle->channel);
    DMA_ResetChannel(handle->dmaIntermediaryToTxRegHandle->base, handle->dmaIntermediaryToTxRegHandle->channel);

    /***channel_A *** used for carry the data from Rx_Data_Register(POPR) to User_Receive_Buffer*/
    transferConfigA.srcAddr = (uint32_t)rxAddr;
    transferConfigA.enableSrcIncrement = false;

    if (handle->rxData)
    {
        transferConfigA.destAddr = (uint32_t) & (handle->rxData[0]);
        transferConfigA.enableDestIncrement = true;
    }
    else
    {
        transferConfigA.destAddr = (uint32_t) & (handle->rxBuffIfNull);
        transferConfigA.enableDestIncrement = false;
    }

    transferConfigA.destSize = kDMA_Transfersize8bits;

    if (handle->bitsPerFrame <= 8)
    {
        transferConfigA.srcSize = kDMA_Transfersize8bits;
    }
    else
    {
        transferConfigA.srcSize = kDMA_Transfersize16bits;
    }

    transferConfigA.transferSize = handle->remainingReceiveByteCount;

    DMA_SetTransferConfig(handle->dmaRxRegToRxDataHandle->base, handle->dmaRxRegToRxDataHandle->channel,
                          &transferConfigA);
    DMA_EnableInterrupts(handle->dmaRxRegToRxDataHandle->base, handle->dmaRxRegToRxDataHandle->channel);

    /***channel_B *** used for carry the data from User_Send_Buffer to "intermediary" because the SPIx_PUSHR should
    write the 32bits at once time . Then use channel_C to carry the "intermediary" to SPIx_PUSHR. Note that the
    SPIx_PUSHR upper 16 bits are the "command" and the low 16bits are data */

    if (((handle->remainingSendByteCount > 1) && (handle->bitsPerFrame <= 8)) ||
        ((handle->remainingSendByteCount > 2) && (handle->bitsPerFrame > 8)))
    {
        if (handle->txData)
        {
            transferConfigB.srcAddr = (uint32_t)(handle->txData);
            transferConfigB.enableSrcIncrement = true;
        }
        else
        {
            transferConfigB.srcAddr = (uint32_t)(&handle->txBuffIfNull);
            transferConfigB.enableSrcIncrement = false;
        }

        transferConfigB.destAddr = (uint32_t)(&handle->command);
        transferConfigB.enableDestIncrement = 0;

        transferConfigB.srcSize = kDMA_Transfersize8bits;

        if (handle->bitsPerFrame <= 8)
        {
            transferConfigB.destSize = kDMA_Transfersize8bits;
            transferConfigB.transferSize = handle->remainingSendByteCount - 1;
        }
        else
        {
            transferConfigB.destSize = kDMA_Transfersize16bits;
            transferConfigB.transferSize = handle->remainingSendByteCount - 2;
        }

        DMA_SetTransferConfig(handle->dmaTxDataToIntermediaryHandle->base,
                              handle->dmaTxDataToIntermediaryHandle->channel, &transferConfigB);
    }

    /*Prepare the last data*/
    if (((handle->remainingSendByteCount > 0) && (1 != FSL_FEATURE_DSPI_HAS_SEPARATE_DMA_RX_TX_REQn(base))) ||
        ((((handle->remainingSendByteCount > 1) && (handle->bitsPerFrame <= 8)) ||
          ((handle->remainingSendByteCount > 2) && (handle->bitsPerFrame > 8))) &&
         (1 == FSL_FEATURE_DSPI_HAS_SEPARATE_DMA_RX_TX_REQn(base))))
    {
        if (handle->txData)
        {
            uint32_t bufferIndex = 0;

            if (1 == FSL_FEATURE_DSPI_HAS_SEPARATE_DMA_RX_TX_REQn(base))
            {
                if (handle->bitsPerFrame <= 8)
                {
                    bufferIndex = handle->remainingSendByteCount - 1;
                }
                else
                {
                    bufferIndex = handle->remainingSendByteCount - 2;
                }
            }
            else
            {
                bufferIndex = handle->remainingSendByteCount;
            }

            if (handle->bitsPerFrame <= 8)
            {
                handle->lastCommand = (handle->lastCommand & 0xffff0000U) | handle->txData[bufferIndex - 1];
            }
            else
            {
                if (handle->isThereExtraByte)
                {
                    handle->lastCommand = (handle->lastCommand & 0xffff0000U) | handle->txData[bufferIndex - 2] |
                                          ((uint32_t)dummyData << 8);
                }
                else
                {
                    handle->lastCommand = (handle->lastCommand & 0xffff0000U) |
                                          ((uint32_t)handle->txData[bufferIndex - 1] << 8) |
                                          handle->txData[bufferIndex - 2];
                }
            }
        }
        else
        {
            if (handle->bitsPerFrame <= 8)
            {
                wordToSend = dummyData;
            }
            else
            {
                wordToSend = ((uint32_t)dummyData << 8) | dummyData;
            }
            handle->lastCommand = (handle->lastCommand & 0xffff0000U) | wordToSend;
        }
    }

    /***channel_C ***carry the "intermediary" to SPIx_PUSHR. */
    if (((handle->remainingSendByteCount > 1) && (handle->bitsPerFrame <= 8)) ||
        ((handle->remainingSendByteCount > 2) && (handle->bitsPerFrame > 8)))
    {
        transferConfigC.srcAddr = (uint32_t)(&(handle->command));
        transferConfigC.destAddr = (uint32_t)txAddr;

        transferConfigC.srcSize = kDMA_Transfersize32bits;
        transferConfigC.destSize = kDMA_Transfersize32bits;
        transferConfigC.enableSrcIncrement = 0;
        transferConfigC.enableDestIncrement = 0;

        if (handle->bitsPerFrame <= 8)
        {
            transferConfigC.transferSize = (handle->remainingSendByteCount - 1) * 4;
        }
        else
        {
            transferConfigC.transferSize = (handle->remainingSendByteCount / 2 - 1) * 4;
        }

        DMA_SetTransferConfig(handle->dmaIntermediaryToTxRegHandle->base, handle->dmaIntermediaryToTxRegHandle->channel,
                              &transferConfigC);

        DMA_EnableInterrupts(handle->dmaIntermediaryToTxRegHandle->base, handle->dmaIntermediaryToTxRegHandle->channel);
    }
    else
    {
        if (handle->remainingSendByteCount != 0)
        {
            writeOneDataAfterStart = true;
        }
    }

    /*Set the channel link.*/
    if (1 == FSL_FEATURE_DSPI_HAS_SEPARATE_DMA_RX_TX_REQn(base))
    {
        /* For DSPI instances with separate RX and TX DMA requests:
        *Rx DMA request -> channel_A;
        *Tx DMA request -> channel_C -> channel_B . (so need prepare the first data in "intermediary"  before the DMA
        *transfer and then channel_B is used to prepare the next data to "intermediary" )
        *if there is Tx DMA request , carry the 32bits data (handle->command) to PUSHR first , then link to channelB
        to prepare the next 32bits data (User_send_buffer to handle->command) .
        Note that the last data is sent out by completion callback , and it should not have channel link to prepare the
        next data if there is only one data transfer by DMA.
        */
        if (((handle->remainingSendByteCount > 2) && (handle->bitsPerFrame <= 8)) ||
            ((handle->remainingSendByteCount > 4) && (handle->bitsPerFrame > 8)))
        {
            channelConfig.linkType = kDMA_ChannelLinkChannel1;                       /*!< Channel link type */
            channelConfig.channel1 = handle->dmaTxDataToIntermediaryHandle->channel; /*!< The index of channel 1 */
            channelConfig.channel2 = 0;                                              /*!< The index of channel 2 */
            DMA_SetChannelLinkConfig(handle->dmaIntermediaryToTxRegHandle->base,
                                     handle->dmaIntermediaryToTxRegHandle->channel, &channelConfig);
        }

        DSPI_EnableDMA(base, kDSPI_RxDmaEnable | kDSPI_TxDmaEnable);
    }
    else
    {
        /*For DSPI instances with shared RX/TX DMA requests: Rx DMA request -> channel_A -> channel_B-> channel_C.
        Note that we already sent first 2 data if there is fifo or send first 1 data if there is no fifo. And the last
        data is transferred by completion callback.*/
        if (((handle->remainingSendByteCount > 1) && (handle->bitsPerFrame <= 8)) ||
            ((handle->remainingSendByteCount > 2) && (handle->bitsPerFrame > 8)))
        {
            channelConfig.linkType = kDMA_ChannelLinkChannel1;                       /*!< Channel link type */
            channelConfig.channel1 = handle->dmaTxDataToIntermediaryHandle->channel; /*!< The index of channel 1 */
            channelConfig.channel2 = 0;                                              /*!< The index of channel 2 */

            DMA_SetChannelLinkConfig(handle->dmaRxRegToRxDataHandle->base, handle->dmaRxRegToRxDataHandle->channel,
                                     &channelConfig);

            channelConfig.linkType = kDMA_ChannelLinkChannel1;                      /*!< Channel link type */
            channelConfig.channel1 = handle->dmaIntermediaryToTxRegHandle->channel; /*!< The index of channel 1 */
            channelConfig.channel2 = 0;                                             /*!< The index of channel 2 */

            DMA_SetChannelLinkConfig(handle->dmaTxDataToIntermediaryHandle->base,
                                     handle->dmaTxDataToIntermediaryHandle->channel, &channelConfig);
        }

        DSPI_EnableDMA(base, kDSPI_RxDmaEnable);
    }

    DSPI_StartTransfer(base);

    /*Start the DMA channel_A , channel_B , channel_C transfer*/
    DMA_StartTransfer(handle->dmaRxRegToRxDataHandle);
    DMA_StartTransfer(handle->dmaTxDataToIntermediaryHandle);
    DMA_StartTransfer(handle->dmaIntermediaryToTxRegHandle);

    if (writeOneDataAfterStart)
    {
        DMA_DspiMasteTxCallback(handle->dmaIntermediaryToTxRegHandle, &s_dspiMasterDmaPrivateHandle[instance]);
    }

    return kStatus_Success;
}

static void DMA_DspiMasteRxCallback(dma_handle_t *dmaHandle, void *g_dspiDmaPrivateHandle)
{
    assert(dmaHandle);
    assert(g_dspiDmaPrivateHandle);

    dspi_master_dma_private_handle_t *dspiDmaPrivateHandle;

    dspiDmaPrivateHandle = (dspi_master_dma_private_handle_t *)g_dspiDmaPrivateHandle;

    DSPI_DisableDMA((dspiDmaPrivateHandle->base), kDSPI_RxDmaEnable | kDSPI_TxDmaEnable);

    DMA_ClearChannelStatusFlags(dmaHandle->base, dmaHandle->channel, kDMA_TransactionsDoneFlag);
    DMA_StopTransfer(dmaHandle);

    dspiDmaPrivateHandle->handle->state = kDSPI_Idle;

    if (dspiDmaPrivateHandle->handle->callback)
    {
        dspiDmaPrivateHandle->handle->callback(dspiDmaPrivateHandle->base, dspiDmaPrivateHandle->handle,
                                               kStatus_Success, dspiDmaPrivateHandle->handle->userData);
    }
}

static void DMA_DspiMasteTxCallback(dma_handle_t *dmaHandle, void *g_dspiDmaPrivateHandle)
{
    assert(dmaHandle);
    assert(g_dspiDmaPrivateHandle);

    dspi_master_dma_private_handle_t *dspiDmaPrivateHandle;

    dspiDmaPrivateHandle = (dspi_master_dma_private_handle_t *)g_dspiDmaPrivateHandle;

    DSPI_DisableDMA((dspiDmaPrivateHandle->base), kDSPI_TxDmaEnable);

    DMA_ClearChannelStatusFlags(dmaHandle->base, dmaHandle->channel, kDMA_TransactionsDoneFlag);
    DMA_StopTransfer(dmaHandle);

    /*Send the last data*/
    DSPI_ClearStatusFlags(dspiDmaPrivateHandle->base, kDSPI_TxFifoFillRequestFlag);
    while (!(DSPI_GetStatusFlags(dspiDmaPrivateHandle->base) & kDSPI_TxFifoFillRequestFlag))
    {
    }

    dspiDmaPrivateHandle->base->PUSHR = dspiDmaPrivateHandle->handle->lastCommand;

    DSPI_ClearStatusFlags(dspiDmaPrivateHandle->base, kDSPI_TxFifoFillRequestFlag);
}

void DSPI_MasterTransferAbortDMA(SPI_Type *base, dspi_master_dma_handle_t *handle)
{
    assert(handle);

    DSPI_StopTransfer(base);

    DSPI_DisableDMA(base, kDSPI_RxDmaEnable | kDSPI_TxDmaEnable);

    DMA_AbortTransfer(handle->dmaRxRegToRxDataHandle);
    DMA_AbortTransfer(handle->dmaTxDataToIntermediaryHandle);
    DMA_AbortTransfer(handle->dmaIntermediaryToTxRegHandle);

    handle->state = kDSPI_Idle;
}

status_t DSPI_MasterTransferGetCountDMA(SPI_Type *base, dspi_master_dma_handle_t *handle, size_t *count)
{
    assert(handle);

    if (!count)
    {
        return kStatus_InvalidArgument;
    }

    /* Catch when there is not an active transfer. */
    if (handle->state != kDSPI_Busy)
    {
        *count = 0;
        return kStatus_NoTransferInProgress;
    }

    size_t bytes;

    bytes = DMA_GetRemainingBytes(handle->dmaRxRegToRxDataHandle->base, handle->dmaRxRegToRxDataHandle->channel);

    *count = handle->totalByteCount - bytes;

    return kStatus_Success;
}

void DSPI_SlaveTransferCreateHandleDMA(SPI_Type *base,
                                       dspi_slave_dma_handle_t *handle,
                                       dspi_slave_dma_transfer_callback_t callback,
                                       void *userData,
                                       dma_handle_t *dmaRxRegToRxDataHandle,
                                       dma_handle_t *dmaTxDataToTxRegHandle)
{
    assert(handle);
    assert(dmaRxRegToRxDataHandle);
    assert(dmaTxDataToTxRegHandle);

    /* Zero the handle. */
    memset(handle, 0, sizeof(*handle));

    uint32_t instance = DSPI_GetInstance(base);

    s_dspiSlaveDmaPrivateHandle[instance].base = base;
    s_dspiSlaveDmaPrivateHandle[instance].handle = handle;

    handle->callback = callback;
    handle->userData = userData;

    handle->dmaRxRegToRxDataHandle = dmaRxRegToRxDataHandle;
    handle->dmaTxDataToTxRegHandle = dmaTxDataToTxRegHandle;
}

status_t DSPI_SlaveTransferDMA(SPI_Type *base, dspi_slave_dma_handle_t *handle, dspi_transfer_t *transfer)
{
    assert(handle);
    assert(transfer);

    /* If send/receive length is zero */
    if (transfer->dataSize == 0)
    {
        return kStatus_InvalidArgument;
    }

    /* If both send buffer and receive buffer is null */
    if ((!(transfer->txData)) && (!(transfer->rxData)))
    {
        return kStatus_InvalidArgument;
    }

    /* Check that we're not busy.*/
    if (handle->state == kDSPI_Busy)
    {
        return kStatus_DSPI_Busy;
    }

    handle->state = kDSPI_Busy;

    uint32_t instance = DSPI_GetInstance(base);
    uint8_t whichCtar = (transfer->configFlags & DSPI_SLAVE_CTAR_MASK) >> DSPI_SLAVE_CTAR_SHIFT;
    handle->bitsPerFrame =
        (((base->CTAR_SLAVE[whichCtar]) & SPI_CTAR_SLAVE_FMSZ_MASK) >> SPI_CTAR_SLAVE_FMSZ_SHIFT) + 1;

    if ((handle->bitsPerFrame > 8) && (transfer->dataSize % 2 != 0))
    {
        handle->state = kDSPI_Idle;
        return kStatus_InvalidArgument;
    }

    DMA_SetCallback(handle->dmaRxRegToRxDataHandle, DMA_DspiSlaveRxCallback, &s_dspiSlaveDmaPrivateHandle[instance]);

    /* Store transfer information */
    handle->txData = transfer->txData;
    handle->rxData = transfer->rxData;
    handle->remainingSendByteCount = transfer->dataSize;
    handle->remainingReceiveByteCount = transfer->dataSize;
    handle->totalByteCount = transfer->dataSize;
    handle->errorCount = 0;

    uint16_t wordToSend = 0;
    uint8_t dummyData = g_dspiDummyData[instance];
    uint8_t dataAlreadyFed = 0;
    uint8_t dataFedMax = 2;

    uint32_t rxAddr = DSPI_GetRxRegisterAddress(base);
    uint32_t txAddr = DSPI_SlaveGetTxRegisterAddress(base);

    dma_transfer_config_t transferConfigA;
    dma_transfer_config_t transferConfigC;
    dma_channel_link_config_t channelConfig;

    DSPI_StopTransfer(base);

    DSPI_FlushFifo(base, true, true);
    DSPI_ClearStatusFlags(base, kDSPI_AllStatusFlag);

    DSPI_DisableDMA(base, kDSPI_RxDmaEnable | kDSPI_TxDmaEnable);

    DSPI_StartTransfer(base);

    /*if dspi has separate dma request , need not prepare data first .
    else (dspi has shared dma request) , send first 2 data into fifo if there is fifo or send first 1 data to
    slaveGetTxRegister if there is no fifo*/
    if (1 != FSL_FEATURE_DSPI_HAS_SEPARATE_DMA_RX_TX_REQn(base))
    {
        /* For DSPI instances with shared RX/TX DMA requests, we'll use the RX DMA request to
        * trigger ongoing transfers and will link to the TX DMA channel from the RX DMA channel.
        */
        /* If bits/frame is greater than one byte */
        if (handle->bitsPerFrame > 8)
        {
            while (DSPI_GetStatusFlags(base) & kDSPI_TxFifoFillRequestFlag)
            {
                if (handle->txData)
                {
                    wordToSend = *(handle->txData);
                    ++handle->txData;
                    wordToSend |= (unsigned)(*(handle->txData)) << 8U;
                    ++handle->txData;
                }
                else
                {
                    wordToSend = ((uint32_t)dummyData << 8) | dummyData;
                }
                handle->remainingSendByteCount -= 2;
                base->PUSHR_SLAVE = wordToSend;

                /* Try to clear the TFFF; if the TX FIFO is full this will clear */
                DSPI_ClearStatusFlags(base, kDSPI_TxFifoFillRequestFlag);

                dataAlreadyFed += 2;

                /* Exit loop if send count is zero, else update local variables for next loop */
                if ((handle->remainingSendByteCount == 0) || (dataAlreadyFed == (dataFedMax * 2)))
                {
                    break;
                }
            } /* End of TX FIFO fill while loop */
        }
        else /* Optimized for bits/frame less than or equal to one byte. */
        {
            while (DSPI_GetStatusFlags(base) & kDSPI_TxFifoFillRequestFlag)
            {
                if (handle->txData)
                {
                    wordToSend = *(handle->txData);
                    /* Increment to next data word*/
                    ++handle->txData;
                }
                else
                {
                    wordToSend = dummyData;
                }

                base->PUSHR_SLAVE = wordToSend;

                /* Try to clear the TFFF; if the TX FIFO is full this will clear */
                DSPI_ClearStatusFlags(base, kDSPI_TxFifoFillRequestFlag);
                /* Decrement remainingSendByteCount*/
                --handle->remainingSendByteCount;

                dataAlreadyFed++;

                /* Exit loop if send count is zero, else update local variables for next loop */
                if ((handle->remainingSendByteCount == 0) || (dataAlreadyFed == dataFedMax))
                {
                    break;
                }
            } /* End of TX FIFO fill while loop */
        }
    }

    DMA_ResetChannel(handle->dmaRxRegToRxDataHandle->base, handle->dmaRxRegToRxDataHandle->channel);
    DMA_ResetChannel(handle->dmaTxDataToTxRegHandle->base, handle->dmaTxDataToTxRegHandle->channel);

    /***channel_A *** used for carry the data from Rx_Data_Register(POPR) to User_Receive_Buffer*/
    if (handle->remainingReceiveByteCount > 0)
    {
        transferConfigA.srcAddr = (uint32_t)rxAddr;
        transferConfigA.enableSrcIncrement = false;

        if (handle->rxData)
        {
            transferConfigA.destAddr = (uint32_t) & (handle->rxData[0]);
            transferConfigA.enableDestIncrement = true;
        }
        else
        {
            transferConfigA.destAddr = (uint32_t) & (handle->rxBuffIfNull);
            transferConfigA.enableDestIncrement = false;
        }

        transferConfigA.destSize = kDMA_Transfersize8bits;

        if (handle->bitsPerFrame <= 8)
        {
            transferConfigA.srcSize = kDMA_Transfersize8bits;
        }
        else
        {
            transferConfigA.srcSize = kDMA_Transfersize16bits;
        }

        transferConfigA.transferSize = handle->remainingReceiveByteCount;

        DMA_SetTransferConfig(handle->dmaRxRegToRxDataHandle->base, handle->dmaRxRegToRxDataHandle->channel,
                              &transferConfigA);
        DMA_EnableInterrupts(handle->dmaRxRegToRxDataHandle->base, handle->dmaRxRegToRxDataHandle->channel);
    }

    /***channel_C *** used for carry the data from User_Send_Buffer to Tx_Data_Register(PUSHR_SLAVE)*/
    if (handle->remainingSendByteCount > 0)
    {
        transferConfigC.destAddr = (uint32_t)txAddr;
        transferConfigC.enableDestIncrement = false;

        if (handle->txData)
        {
            transferConfigC.srcAddr = (uint32_t)(&(handle->txData[0]));
            transferConfigC.enableSrcIncrement = true;
        }
        else
        {
            transferConfigC.srcAddr = (uint32_t)(&handle->txBuffIfNull);
            transferConfigC.enableSrcIncrement = false;
            if (handle->bitsPerFrame <= 8)
            {
                handle->txBuffIfNull = dummyData;
            }
            else
            {
                handle->txBuffIfNull = ((uint32_t)dummyData << 8) | dummyData;
            }
        }

        transferConfigC.srcSize = kDMA_Transfersize8bits;

        if (handle->bitsPerFrame <= 8)
        {
            transferConfigC.destSize = kDMA_Transfersize8bits;
        }
        else
        {
            transferConfigC.destSize = kDMA_Transfersize16bits;
        }

        transferConfigC.transferSize = handle->remainingSendByteCount;

        DMA_SetTransferConfig(handle->dmaTxDataToTxRegHandle->base, handle->dmaTxDataToTxRegHandle->channel,
                              &transferConfigC);
    }

    /*Set the channel link.*/
    if (1 != FSL_FEATURE_DSPI_HAS_SEPARATE_DMA_RX_TX_REQn(base))
    {
        /*For DSPI instances with shared RX/TX DMA requests: Rx DMA request -> channel_A -> channel_C.*/
        if (handle->remainingSendByteCount > 0)
        {
            channelConfig.linkType = kDMA_ChannelLinkChannel1;
            channelConfig.channel1 = handle->dmaTxDataToTxRegHandle->channel;
            channelConfig.channel2 = 0;
            DMA_SetChannelLinkConfig(handle->dmaRxRegToRxDataHandle->base, handle->dmaRxRegToRxDataHandle->channel,
                                     &channelConfig);
        }
        DSPI_EnableDMA(base, kDSPI_RxDmaEnable);
    }
    else
    {
        /*For DSPI instances with separate RX and TX DMA requests:
       Rx DMA request -> channel_A
       Tx DMA request -> channel_C */
        DSPI_EnableDMA(base, kDSPI_RxDmaEnable | kDSPI_TxDmaEnable);
    }

    DMA_StartTransfer(handle->dmaRxRegToRxDataHandle);

    DMA_StartTransfer(handle->dmaTxDataToTxRegHandle);

    return kStatus_Success;
}

static void DMA_DspiSlaveRxCallback(dma_handle_t *dmaHandle, void *g_dspiDmaPrivateHandle)
{
    assert(dmaHandle);
    assert(g_dspiDmaPrivateHandle);

    DMA_ClearChannelStatusFlags(dmaHandle->base, dmaHandle->channel, kDMA_TransactionsDoneFlag);
    DMA_StopTransfer(dmaHandle);

    dspi_slave_dma_private_handle_t *dspiDmaPrivateHandle;

    dspiDmaPrivateHandle = (dspi_slave_dma_private_handle_t *)g_dspiDmaPrivateHandle;
    DSPI_DisableDMA((dspiDmaPrivateHandle->base), kDSPI_RxDmaEnable | kDSPI_TxDmaEnable);

    DMA_ClearChannelStatusFlags((dspiDmaPrivateHandle->handle)->dmaTxDataToTxRegHandle->base,
                                (dspiDmaPrivateHandle->handle)->dmaTxDataToTxRegHandle->channel,
                                kDMA_TransactionsDoneFlag);
    DMA_StopTransfer(dspiDmaPrivateHandle->handle->dmaTxDataToTxRegHandle);

    dspiDmaPrivateHandle->handle->state = kDSPI_Idle;

    if (dspiDmaPrivateHandle->handle->callback)
    {
        dspiDmaPrivateHandle->handle->callback(dspiDmaPrivateHandle->base, dspiDmaPrivateHandle->handle,
                                               kStatus_Success, dspiDmaPrivateHandle->handle->userData);
    }
}

void DSPI_SlaveTransferAbortDMA(SPI_Type *base, dspi_slave_dma_handle_t *handle)
{
    assert(handle);

    DSPI_StopTransfer(base);

    DSPI_DisableDMA(base, kDSPI_RxDmaEnable | kDSPI_TxDmaEnable);

    DMA_AbortTransfer(handle->dmaRxRegToRxDataHandle);
    DMA_AbortTransfer(handle->dmaTxDataToTxRegHandle);

    handle->state = kDSPI_Idle;
}

status_t DSPI_SlaveTransferGetCountDMA(SPI_Type *base, dspi_slave_dma_handle_t *handle, size_t *count)
{
    assert(handle);

    if (!count)
    {
        return kStatus_InvalidArgument;
    }

    /* Catch when there is not an active transfer. */
    if (handle->state != kDSPI_Busy)
    {
        *count = 0;
        return kStatus_NoTransferInProgress;
    }

    size_t bytes;

    bytes = DMA_GetRemainingBytes(handle->dmaRxRegToRxDataHandle->base, handle->dmaRxRegToRxDataHandle->channel);

    *count = handle->totalByteCount - bytes;

    return kStatus_Success;
}

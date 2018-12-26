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

#include "fsl_dspi_hal.h"
#include "fsl_device_registers.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/

/*FUNCTION**********************************************************************
 *
 * Function Name : dspi_hal_master_init
 * Description   : Configure the DSPI peripheral in master mode.
 * This function will initialize the module to user defined settings and default settings in master
 * mode.  Here is an example demonstrating how to define the dspi_master_config_t structure and call
 * the dspi_hal_master_init function:
 *    dspi_master_config_t dspiConfig;
 *    dspiConfig.isEnabled = false;
 *    dspiConfig.whichCtar = kDspiCtar0;
 *    dspiConfig.bitsPerSec = 0;
 *    dspiConfig.sourceClockInHz = dspiSourceClock;
 *    dspiConfig.isSckContinuous = false;
 *    dspiConfig.whichPcs = kDspiPcs0;
 *    dspiConfig.pcsPolarity = kDspiPcs_ActiveLow;
 *    dspiConfig.masterInSample = kDspiSckToSin_0Clock;
 *    dspiConfig.isModifiedTimingFormatEnabled = false;
 *    dspiConfig.isTxFifoDisabled = false;
 *    dspiConfig.isRxFifoDisabled = false;
 *    dspiConfig.dataConfig.bitsPerFrame = 16;
 *    dspiConfig.dataConfig.clkPolarity = kDspiClockPolarity_ActiveHigh;
 *    dspiConfig.dataConfig.clkPhase = kDspiClockPhase_FirstEdge;
 *    dspiConfig.dataConfig.direction = kDspiMsbFirst;
 *    dspi_hal_master_init(baseAddr, &dspiConfig, calculatedBaudRate);
 *
 *END**************************************************************************/
status_t dspi_hal_master_init(SPI_Type *baseAddr, const dspi_master_config_t *config, uint32_t *calculatedBaudRate)
{
    /* Enable or disable the module. */
    /* Note, to enable the module, MDIS must be cleared.  However, the member isEnabled*/
    /* must be true (1) to enable module, hence we negate the value of isEnabled to properly*/
    /* configure the MDIS bit*/
    SPI_BWR_MCR_MDIS(baseAddr, ~(config->isEnabled == true));

    /* Configure baud rate if a value is provided.*/
    if (config->bitsPerSec != 0U)
    {
        *calculatedBaudRate =
            dspi_hal_set_baud(baseAddr, config->whichCtar, config->bitsPerSec, config->sourceClockInHz);
    }
    else
    {
        *calculatedBaudRate = 0;
    }

    /* Set master or slave mode.*/
    dspi_hal_set_master_slave(baseAddr, kDspiMaster);

    /* Configure data format.*/
    if (dspi_hal_configure_data_format(baseAddr, config->whichCtar, &config->dataConfig) != kStatus_Success)
    {
        return kStatus_DSPI_InvalidBitCount;
    }

    /* Configure for continuous SCK operation*/
    dspi_hal_configure_continuous_sck(baseAddr, config->isSckContinuous);

    /* Configure for peripheral chip select polarity*/
    dspi_hal_configure_pcs_polarity(baseAddr, config->whichPcs, config->pcsPolarity);

    /* Configure sample point for data in, master mode*/
    dspi_hal_set_datain_samplepoint(baseAddr, config->masterInSample);

    /* Configure for modified timing format*/
    dspi_hal_configure_modified_timing_format(baseAddr, config->isModifiedTimingFormatEnabled);

    /* Configure for fifo operation*/
    dspi_hal_configure_fifos(baseAddr, config->isTxFifoDisabled, config->isRxFifoDisabled);

    /* finally, clear the DSPI CONFIGURATION (DCONF), even though this is cleared in some IPs*/
    /* by default and other bit settings are reserved*/
    SPI_CLR_MCR(baseAddr, SPI_MCR_DCONF_MASK);

    return kStatus_Success;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : dspi_hal_slave_init
 * Description   : Configure the DSPI peripheral in slave mode.
 * This function initializes the DSPI module for slave mode. Here is an example demonstrating how
 * to define the dspi_slave_config_t structure and call the dspi_hal_slave_init function:
 *    dspi_slave_config_t dspiConfig;
 *    dspiConfig.isEnabled = false;
 *    dspiConfig.isTxFifoDisabled = false;
 *    dspiConfig.isRxFifoDisabled = false;
 *    dspiConfig.dataConfig.bitsPerFrame = 16;
 *    dspiConfig.dataConfig.clkPolarity = kDspiClockPolarity_ActiveHigh;
 *    dspiConfig.dataConfig.clkPhase = kDspiClockPhase_FirstEdge;
 *    dspi_hal_slave_init(baseAddr, &dspiConfig);
 *
 *END**************************************************************************/
status_t dspi_hal_slave_init(SPI_Type *baseAddr, const dspi_slave_config_t *config)
{
    /* Enable or disable the module.
     * Note, to enable the module, MDIS must be cleared.  However, the member isEnabled
     * must be true (1) to enable module, hence we negate the value of isEnabled to properly
     * configure the MDIS bit
     */
    SPI_BWR_MCR_MDIS(baseAddr, ~(config->isEnabled == true));

    /* Set master or slave moe. */
    dspi_hal_set_master_slave(baseAddr, kDspiSlave);

    /* Configure data format. For slave mode, only CTAR0 is available for use */
    if (dspi_hal_configure_data_format(baseAddr, kDspiCtar0, &config->dataConfig) != kStatus_Success)
    {
        return kStatus_DSPI_InvalidBitCount;
    }

    /* Configure for fifo operation */
    dspi_hal_configure_fifos(baseAddr, config->isTxFifoDisabled, config->isRxFifoDisabled);

    /* finally, clear the DSPI CONFIGURATION (DCONF), even though this is cleared in some IPs
     * by default and other bit settings are reserved
     */
    SPI_CLR_MCR(baseAddr, SPI_MCR_DCONF_MASK);

    return kStatus_Success;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : dspi_hal_reset
 * Description   : Restore DSPI to reset configuration.
 * This function basically resets all of the DSPI registers to their default setting including
 * disabling the module.
 *
 *END**************************************************************************/
void dspi_hal_reset(SPI_Type *baseAddr)
{
    /* first, make sure the module is enabled to allow writes to certain registers*/
    dspi_hal_enable(baseAddr);

    /* Halt all transfers*/
    SPI_WR_MCR(baseAddr, SPI_MCR_HALT_MASK);

    /* flush the fifos*/
    dspi_hal_flush_fifos(baseAddr, true, true);

    /* set the registers to their default states*/
    /* clear the status bits (write-1-to-clear)*/
    SPI_WR_SR(baseAddr, SPI_SR_TCF_MASK | SPI_SR_EOQF_MASK | SPI_SR_TFUF_MASK | SPI_SR_TFFF_MASK | SPI_SR_RFOF_MASK |
                            SPI_SR_RFDF_MASK);
    SPI_WR_TCR(baseAddr, 0);
    SPI_WR_CTAR(baseAddr, 0, 0); /* CTAR0*/
    SPI_WR_CTAR(baseAddr, 1, 0); /* CTAR1*/
    SPI_WR_RSER(baseAddr, 0);
    /* disable the module*/
    SPI_WR_MCR(baseAddr, SPI_MCR_MDIS_MASK | SPI_MCR_HALT_MASK);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : dspi_hal_set_baud
 * Description   : Set the DSPI baud rate in bits per second.
 * This function will take in the desired bitsPerSec (baud rate) and will calculate the nearest
 * possible baud rate without exceeding the desired baud rate, and will return the calculated
 * baud rate in bits-per-second. It requires that the caller also provide the frequency of the
 * module source clock (in Hz).
 *
 *END**************************************************************************/
uint32_t dspi_hal_set_baud(SPI_Type *baseAddr,
                           dspi_ctar_selection_t whichCtar,
                           uint32_t bitsPerSec,
                           uint32_t sourceClockInHz)
{
    /* for master mode configuration, if slave mode detected, return 0*/
    if (SPI_RD_MCR_MSTR(baseAddr) != 1)
    {
        return 0;
    }

    uint32_t prescaler, bestPrescaler;
    uint32_t scaler, bestScaler;
    uint32_t dbr, bestDbr;
    uint32_t realBaudrate, bestBaudrate;
    uint32_t diff, min_diff;
    uint32_t baudrate = bitsPerSec;

    /* find combination of prescaler and scaler resulting in baudrate closest to the */
    /* requested value */
    min_diff = 0xFFFFFFFFU;
    bestPrescaler = 0;
    bestScaler = 0;
    bestDbr = 1;
    bestBaudrate = 0; /* required to avoid compilation warning */

    /* In all for loops, if min_diff = 0, the exit for loop*/
    for (prescaler = 0; (prescaler < 4) && min_diff; prescaler++)
    {
        for (scaler = 0; (scaler < 16) && min_diff; scaler++)
        {
            for (dbr = 1; (dbr < 3) && min_diff; dbr++)
            {
                realBaudrate =
                    ((sourceClockInHz * dbr) / (s_baudratePrescaler[prescaler] * (s_baudrateScaler[scaler])));

                /* calculate the baud rate difference based on the conditional statement*/
                /* that states that the calculated baud rate must not exceed the desired baud rate*/
                if (baudrate >= realBaudrate)
                {
                    diff = baudrate - realBaudrate;
                    if (min_diff > diff)
                    {
                        /* a better match found */
                        min_diff = diff;
                        bestPrescaler = prescaler;
                        bestScaler = scaler;
                        bestBaudrate = realBaudrate;
                        bestDbr = dbr;
                    }
                }
            }
        }
    }

    uint32_t temp;
    /* write the best dbr, prescalar, and baud rate scalar to the CTAR*/
    temp = SPI_RD_CTAR(baseAddr, whichCtar); /* save register contents*/
    temp &= ~(SPI_CTAR_DBR_MASK | SPI_CTAR_PBR_MASK | SPI_CTAR_BR_MASK);
    temp |= SPI_CTAR_DBR(bestDbr - 1) | SPI_CTAR_PBR(bestPrescaler) | SPI_CTAR_BR(bestScaler);
    SPI_WR_CTAR(baseAddr, whichCtar, temp);

    /* return the actual calculated baud rate*/
    return bestBaudrate;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : dspi_hal_set_baud_divisors
 * Description   : Configure the baud rate divisors manually.
 * This function allows the caller to manually set the baud rate divisors in the event that
 * these dividers are known and the caller does not wish to call the dspi_hal_set_baud function.
 *
 *END**************************************************************************/
void dspi_hal_set_baud_divisors(SPI_Type *baseAddr,
                                dspi_ctar_selection_t whichCtar,
                                const dspi_baud_rate_divisors_t *divisors)
{
    uint32_t temp;

    /* these settings are only relevant in master mode*/
    if (SPI_RD_MCR_MSTR(baseAddr) == 1)
    {
        temp = SPI_RD_CTAR(baseAddr, whichCtar);                             /* save register contents*/
        temp &= ~(SPI_CTAR_DBR_MASK | SPI_CTAR_PBR_MASK | SPI_CTAR_BR_MASK); /* clear dividers*/
        temp |= SPI_CTAR_DBR(divisors->doubleBaudRate) | SPI_CTAR_PBR(divisors->prescaleDivisor) |
                SPI_CTAR_BR(divisors->baudRateDivisor);
        SPI_WR_CTAR(baseAddr, whichCtar, temp);
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : dspi_hal_configure_pcs_polarity
 * Description   : Configure DSPI peripheral chip select polarity.
 * This function will take in the desired peripheral chip select (PCS) and it's
 * corresponding desired polarity and will configure the PCS signal to operate with the
 * desired characteristic.
 *
 *END**************************************************************************/
void dspi_hal_configure_pcs_polarity(SPI_Type *baseAddr,
                                     dspi_which_pcs_config_t pcs,
                                     dspi_pcs_polarity_config_t activeLowOrHigh)
{
    uint32_t temp;

    temp = SPI_RD_MCR_PCSIS(baseAddr);

    if (activeLowOrHigh == kDspiPcs_ActiveLow)
    {
        temp |= pcs;
    }
    else /* kDspiPcsPolarity_ActiveHigh*/
    {
        temp &= ~(unsigned)pcs;
    }

    SPI_BWR_MCR_PCSIS(baseAddr, temp);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : dspi_hal_configure_fifos
 * Description   : Configure DSPI fifos.
 * This function with allow the caller to disable/enable the TX and RX FIFOs (independently).
 * Note that to disable, the caller must pass in a logic 1 (true) for the particular FIFO
 * configuration.  To enable, the caller must pass in a logic 0 (false).  For example, to enable
 * both the TX and RX FIFOs, the caller will make this function call (where baseAddr is the
 *
 *END**************************************************************************/
void dspi_hal_configure_fifos(SPI_Type *baseAddr, bool disableTxFifo, bool disableRxFifo)
{
    /* first see if MDIS is set or cleared */
    uint32_t isMdisSet = SPI_RD_MCR_MDIS(baseAddr);

    if (isMdisSet)
    {
        /* clear the MDIS bit to allow us to write to the fifo disables */
        SPI_CLR_MCR(baseAddr, SPI_MCR_MDIS_MASK);
    }

    SPI_BWR_MCR_DIS_TXF(baseAddr, (disableTxFifo == true));
    SPI_BWR_MCR_DIS_RXF(baseAddr, (disableRxFifo == true));

    /* set MDIS if it was set to begin with */
    if (isMdisSet)
    {
        SPI_SET_MCR(baseAddr, SPI_MCR_MDIS_MASK);
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : dspi_hal_flush_fifos
 * Description   : Flush DSPI fifos.
 *
 *END**************************************************************************/
void dspi_hal_flush_fifos(SPI_Type *baseAddr, bool enableFlushTxFifo, bool enableFlushRxFifo)
{
    SPI_BWR_MCR_CLR_TXF(baseAddr, (enableFlushTxFifo == true));
    SPI_BWR_MCR_CLR_RXF(baseAddr, (enableFlushRxFifo == true));
}

/*FUNCTION**********************************************************************
 *
 * Function Name : dspi_hal_configure_data_format
 * Description   : Configure the data format for a particular CTAR.
 * This function configures the bits-per-frame, polarity, phase, and shift direction for a
 * particular CTAR. An example use case is as follows:
 *    dspi_data_format_config_t dataFormat;
 *    dataFormat.bitsPerFrame = 16;
 *    dataFormat.clkPolarity = kDspiClockPolarity_ActiveLow;
 *    dataFormat.clkPhase = kDspiClockPhase_FirstEdge;
 *    dataFormat.direction = kDspiMsbFirst;
 *    dspi_hal_configure_data_format(baseAddr, kDspiCtar0, &dataFormat);
 *
 *END**************************************************************************/
status_t dspi_hal_configure_data_format(SPI_Type *baseAddr,
                                        dspi_ctar_selection_t whichCtar,
                                        const dspi_data_format_config_t *config)
{
    /* check bits-per-frame value to make sure it it within the proper range*/
    /* in either master or slave mode*/
    if ((config->bitsPerFrame < 4) || ((config->bitsPerFrame > 16) && (SPI_RD_MCR_MSTR(baseAddr) == 1)) ||
        ((config->bitsPerFrame > 32) && (SPI_RD_MCR_MSTR(baseAddr) == 0)))
    {
        return kStatus_DSPI_InvalidBitCount;
    }

    uint32_t temp;

    /* for master mode configuration*/
    if (SPI_RD_MCR_MSTR(baseAddr) == 1)
    {
        temp = SPI_RD_CTAR(baseAddr, whichCtar); /* save register contents*/
        temp &= ~(SPI_CTAR_FMSZ_MASK | SPI_CTAR_CPOL_MASK | SPI_CTAR_CPHA_MASK | SPI_CTAR_LSBFE_MASK);
        temp |= SPI_CTAR_FMSZ(config->bitsPerFrame - 1) | SPI_CTAR_CPOL(config->clkPolarity) |
                SPI_CTAR_CPHA(config->clkPhase) | SPI_CTAR_LSBFE(config->direction);
        SPI_WR_CTAR(baseAddr, whichCtar, temp);
    }
    else /* for slave mode configuration*/
    {
        temp = SPI_RD_CTAR_SLAVE(baseAddr, whichCtar); /* save register contents*/
        temp &= ~(SPI_CTAR_FMSZ_MASK | SPI_CTAR_CPOL_MASK | SPI_CTAR_CPHA_MASK);
        temp |= SPI_CTAR_SLAVE_FMSZ(config->bitsPerFrame - 1) | SPI_CTAR_SLAVE_CPOL(config->clkPolarity) |
                SPI_CTAR_SLAVE_CPHA(config->clkPhase);
        SPI_WR_CTAR_SLAVE(baseAddr, whichCtar, temp);
    }
    return kStatus_Success;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : dspi_hal_configure_delays
 * Description   : Configure the delays for a particular CTAR, master mode only.
 * This function configures the PCS to SCK delay prescalar (PCSSCK),
 * the PCS to SCK Delay scalar (CSSCK),
 * the After SCK delay prescalar (PASC),
 * the After SCK delay scalar (ASC),
 * the Delay after transfer prescalar (PDT),
 * and the Delay after transfer scalar (DT).
 * The following is an example use case of this function:
 *    dspi_delay_settings_config_t delayConfig;
 *    delayConfig.pcsToSckPre = 0x3;
 *    delayConfig.pcsToSck = 0xF;
 *    delayConfig.afterSckPre = 0x2;
 *    delayConfig.afterSck = 0xA;
 *    delayConfig.afterTransferPre = 0x1;
 *    delayConfig.afterTransfer = 0x5;
 *    dspi_hal_configure_delays(baseAddr, kDspiCtar0, &delayConfig);
 *
 *END**************************************************************************/
void dspi_hal_configure_delays(SPI_Type *baseAddr,
                               dspi_ctar_selection_t whichCtar,
                               const dspi_delay_settings_config_t *config)
{
    uint32_t temp;

    /* these settings are only relevant in master mode*/
    if (SPI_RD_MCR_MSTR(baseAddr) == 1)
    {
        temp = SPI_RD_CTAR(baseAddr, whichCtar); /* save register contents*/
        temp &= ~(SPI_CTAR_PCSSCK_MASK | SPI_CTAR_PASC_MASK | SPI_CTAR_PDT_MASK | SPI_CTAR_CSSCK_MASK |
                  SPI_CTAR_ASC_MASK | SPI_CTAR_DT_MASK);
        temp |= SPI_CTAR_PCSSCK(config->pcsToSckPre) | SPI_CTAR_PASC(config->afterSckPre) |
                SPI_CTAR_PDT(config->afterTransferPre) | SPI_CTAR_CSSCK(config->pcsToSck) |
                SPI_CTAR_ASC(config->afterSck) | SPI_CTAR_DT(config->afterTransfer);
        SPI_WR_CTAR(baseAddr, whichCtar, temp);
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : dspi_hal_configure_dma
 * Description   : Configure transmit and receive DMA requests.
 * This function configures the FIFOs to generate a DMA or interrupt request.
 *
 *END**************************************************************************/
void dspi_hal_configure_dma(SPI_Type *baseAddr, bool enableTransmit, bool enableReceive)
{
    SPI_BWR_RSER_TFFF_DIRS(baseAddr, enableTransmit);
    SPI_BWR_RSER_RFDF_DIRS(baseAddr, enableReceive);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : dspi_hal_configure_interrupt
 * Description   : Configure DSPI interrupts.
 * This function will configure the various interrupt sources of the DSPI.  The parameters to pass
 * in are baseAddr, interrupt source, and enable/disable setting.
 * The interrupt source will be of a typedef enum whose value will be the bit position of the
 * interrupt source setting within the RSER register.  In the DSPI, all of the interrupt
 * configuration settings reside within the one register.  The typedef enum will equate each
 * interrupt source to the bit position defined in the device header file.
 * The function will use these bit positions in its algorithm to enable/disable the
 * interrupt source, where interrupt source is of type dspi_status_and_interrupt_request_t.
 *    temp = (SPI_RD_RSER(baseAddr) & ~interruptSrc) | (enable << interruptSrc);
 *    SPI_WR_RSER(baseAddr, temp);
 *
 *    dspi_hal_configure_interrupt(baseAddr, kDspiTxComplete, true); <- example use-case
 *
 *END**************************************************************************/
void dspi_hal_configure_interrupt(SPI_Type *baseAddr, dspi_status_and_interrupt_request_t interruptSrc, bool enable)
{
    uint32_t temp;

    temp = (SPI_RD_RSER(baseAddr) & ~(0x1U << interruptSrc)) | ((uint32_t)enable << interruptSrc);
    SPI_WR_RSER(baseAddr, temp);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : dspi_hal_get_fifo_data
 * Description   : Read fifo registers for debug purposes.
 *
 *END**************************************************************************/
uint32_t dspi_hal_get_fifo_data(SPI_Type *baseAddr, dspi_fifo_t whichFifo, uint32_t whichFifoEntry)
{
    if (whichFifo == kDspiTxFifo)
    {
        if (whichFifoEntry == 0)
        {
            return SPI_RD_TXFR0(baseAddr);
        }
        else if (whichFifoEntry == 1)
        {
            return SPI_RD_TXFR1(baseAddr);
        }
        else if (whichFifoEntry == 2)
        {
            return SPI_RD_TXFR2(baseAddr);
        }
        else
        {
            return SPI_RD_TXFR3(baseAddr);
        }
    }
    else
    {
        if (whichFifoEntry == 0)
        {
            return SPI_RD_RXFR0(baseAddr);
        }
        else if (whichFifoEntry == 1)
        {
            return SPI_RD_RXFR1(baseAddr);
        }
        else if (whichFifoEntry == 2)
        {
            return SPI_RD_RXFR2(baseAddr);
        }
        else
        {
            return SPI_RD_RXFR3(baseAddr);
        }
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : dspi_hal_write_data_master_mode
 * Description   : Write data into the data buffer, master mode.
 * In master mode, the 16-bit data is appended with the 16-bit command info. The command portion
 * provides characteristics of the data being sent such as: optional continuous chip select
 * operation between transfers, the desired Clock and Transfer Attributes register to use for the
 * associated SPI frame, the desired PCS signal to use for the data transfer, whether the current
 * transfer is the last in the queue, and whether to clear the transfer count (normally needed when
 * sending the first frame of a data packet). An example use case is as follows:
 *    dspi_command_config_t commandConfig;
 *    commandConfig.isChipSelectContinuous = true;
 *    commandConfig.whichCtar = kDspiCtar0;
 *    commandConfig.whichPcs = kDspiPcs1;
 *    commandConfig.clearTransferCount = false;
 *    commandConfig.isEndOfQueue = false;
 *    dspi_hal_write_data_master_mode(baseAddr, &commandConfig, dataWord);
 *
 *END**************************************************************************/
void dspi_hal_write_data_master_mode(SPI_Type *baseAddr, dspi_command_config_t *command, uint16_t data)
{
    uint32_t temp;

    temp = SPI_PUSHR_CONT(command->isChipSelectContinuous) | SPI_PUSHR_CTAS(command->whichCtar) |
           SPI_PUSHR_PCS(command->whichPcs) | SPI_PUSHR_EOQ(command->isEndOfQueue) |
           SPI_PUSHR_CTCNT(command->clearTransferCount) | SPI_PUSHR_TXDATA(data);

    SPI_WR_PUSHR(baseAddr, temp);
}

/*******************************************************************************
 * EOF
 ******************************************************************************/

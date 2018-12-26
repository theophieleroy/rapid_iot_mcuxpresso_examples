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

#include "dspi/hal/fsl_dspi_hal.h"
#include "fsl_dspi_hal_unit_test.h"
#include "fsl_device_registers.h"
#include "clock/fsl_clock_manager.h"
#include "utilities/fsl_debug_uart.h"
#include "board.h"
#include <stdio.h>
#include "utilities/fsl_assert.h"
#include <string.h>

/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define DEBUG_UART_BAUD (9600)

/*******************************************************************************
 * Variables
 ******************************************************************************/
/* save off various instances of this struct  */
static dspi_test_status_t s_dspiTestStatus[SPI_INSTANCE_COUNT];

/*******************************************************************************
 * Code
 ******************************************************************************/
/* prototype*/
void init_hardware(void);

void init_hardware(void)
{
    SIM->SCGC5 |= (SIM_SCGC5_PORTA_MASK | SIM_SCGC5_PORTB_MASK | SIM_SCGC5_PORTC_MASK | SIM_SCGC5_PORTD_MASK |
                   SIM_SCGC5_PORTE_MASK);

    /*Enable pins for UART2*/
    PORT_BWR_PCR_MUX(HW_PORTE, 17, 3);
    PORT_BWR_PCR_MUX(HW_PORTE, 16, 3);

    /* Enable clock gate to UART 2 module*/
    SIM_SET_SCGC4(SIM_SCGC4_UART2_MASK);

    /* Init uart driver for stdio.*/
    debug_uart_init(BOARD_DEBUG_UART_INSTANCE, DEBUG_UART_BAUD);
}

uint32_t main()
{
    uint32_t instance = 0;
    uint32_t dspiBusClock;
    uint32_t i;
    uint32_t fifoSizeCount;
    uint32_t errorFlag = 0;

    init_hardware();

    printf("\r\nDSPI Hal unit test \n");

    /**************************************************************************/
    /* The following initializes the hardware such as clock gates and clock*/
    /* configuration*/
    /**************************************************************************/

    /* for K70 and K60*/
    /* ungate the clocks to the dspi module*/
    for (i = 0; i < SPI_INSTANCE_COUNT; i++)
    {
        clock_manager_set_gate(kClockModuleSPI, i, 1);
    }
    /* obtain clock frequency for dspi module, which should be the bus clock*/
    clock_manager_get_frequency(kBusClock, &dspiBusClock);

    for (instance = 0; instance < SPI_INSTANCE_COUNT; instance++)
    {
        /* Clear the test status struct for this (and each) instance.*/
        memset(&s_dspiTestStatus[instance], 0, sizeof(dspi_test_status_t));

        /* configure the fifo size based on the dspi module instance*/
        fifoSizeCount = (uint32_t)FSL_FEATURE_SPI_FIFO_SIZEn(instance);

        /**********************************************/
        /* Test the module enable/disable*/
        /**********************************************/
        dspi_hal_enable(instance);
        if (SPI_RD_MCR_MDIS(instance) != 0)
        {
            s_dspiTestStatus[instance].B.testStatusDspiEnableDisable += 1;
        }

        dspi_hal_disable(instance);
        if (SPI_RD_MCR_MDIS(instance) != 1)
        {
            s_dspiTestStatus[instance].B.testStatusDspiEnableDisable += 1;
        }

        /**********************************************/
        /* Test the dspi_hal_set_master_slave */
        /**********************************************/
        dspi_hal_set_master_slave(instance, kDspiMaster);
        if (SPI_RD_MCR_MSTR(instance) != 1)
        {
            s_dspiTestStatus[instance].B.testStatusDspiMasterSlave += 1;
        }

        dspi_hal_set_master_slave(instance, kDspiSlave);
        if (SPI_RD_MCR_MSTR(instance) != 0)
        {
            s_dspiTestStatus[instance].B.testStatusDspiMasterSlave += 1;
        }

        /**********************************************/
        /* Test the baud rate divisors (master mode only)*/
        /**********************************************/
        /* first set to master mode as baud rate registers only work in master mode*/
        dspi_hal_set_master_slave(instance, kDspiMaster);

        dspi_baud_rate_divisors_t brDivisors;

        brDivisors.doubleBaudRate = 0x1;
        brDivisors.prescaleDivisor = 0x3;
        brDivisors.baudRateDivisor = 0xF;

        dspi_hal_set_baud_divisors(instance, kDspiCtar0, &brDivisors);

        /* read back divisor values to see if they were programmed correctly */
        if ((SPI_RD_CTARn_DBR(instance, kDspiCtar0) != 0x1) || (SPI_RD_CTARn_PBR(instance, kDspiCtar0) != 0x3) ||
            (SPI_RD_CTARn_BR(instance, kDspiCtar0) != 0xF))
        {
            s_dspiTestStatus[instance].B.testStatusDspiBaudRateDiv = 1;
        }

        /**********************************************/
        /* Test the dspi baud rate function (master mode only)*/
        /* Master mode set in previous test*/
        /**********************************************/
        /* this test specifically uses a bus cock of 60Mhz for it's calculation to ensure a*/
        /* baud match match.  */
        uint32_t sourceClkHz = 60000000;
        uint32_t baudRates[10] = {261, 523, 1220, 5859, 78125, 187500, 625000, 1875000, 15000000, 30000000};
        uint32_t reCalculatedBaudRate, dbr, pbr, br;

        /* spot check a few baud rates*/
        for (i = 0; i < 10; i++)
        {
            if (dspi_hal_set_baud(instance, kDspiCtar0, baudRates[i], sourceClkHz) != baudRates[i])
            {
                s_dspiTestStatus[instance].B.testStatusDspiBaudRateTest = +1;
            }

            /* now re-calculate the baud rate based on the dividers programmed*/
            dbr = HW_SPI_CTARn(instance, kDspiCtar0).B.DBR;
            pbr = HW_SPI_CTARn(instance, kDspiCtar0).B.PBR;
            br = HW_SPI_CTARn(instance, kDspiCtar0).B.BR;
            reCalculatedBaudRate = ((sourceClkHz * (dbr + 1)) / (s_baudratePrescaler[pbr] * (s_baudrateScaler[br])));

            if (reCalculatedBaudRate != baudRates[i])
            {
                s_dspiTestStatus[instance].B.testStatusDspiBaudRateTest = +1;
            }
        }

        /* now, select a few baud rates that are not exact matches and ensure they "round down"*/
        /* to the closest baud rate (and to ensure the calculated baud rate doesn't exceed*/
        /* the desired baud rate)*/
        uint32_t desiredBaudRates[11] = {100,    270,    1000,    1500,     6000,    79000,
                                         200000, 720000, 1900000, 18000000, 35000000};
        uint32_t roundDownBaudRates[11] = {0, 261, 915, 1464, 5859, 78125, 187500, 625000, 1875000, 15000000, 30000000};
        for (i = 0; i < 11; i++)
        {
            if (dspi_hal_set_baud(instance, kDspiCtar0, desiredBaudRates[i], sourceClkHz) != roundDownBaudRates[i])
            {
                s_dspiTestStatus[instance].B.testStatusDspiBaudRateTest = +1;
            }
        }

        /**********************************************/
        /* Test the dspi_hal_configure_continuous_sck */
        /**********************************************/
        dspi_hal_configure_continuous_sck(instance, true);
        if (SPI_RD_MCR_CONT_SCKE(instance) != 1)
        {
            s_dspiTestStatus[instance].B.testStatusDspiContSck += 1;
        }

        dspi_hal_configure_continuous_sck(instance, false);
        if (SPI_RD_MCR_CONT_SCKE(instance) != 0)
        {
            s_dspiTestStatus[instance].B.testStatusDspiContSck += 1;
        }

        /**********************************************/
        /* Test the dspi_hal_configure_halt_in_debug_mode */
        /**********************************************/
        dspi_hal_configure_halt_in_debug_mode(instance, true);
        if (SPI_RD_MCR_FRZ(instance) != 1)
        {
            s_dspiTestStatus[instance].B.testStatusDspiFreeze += 1;
        }

        dspi_hal_configure_halt_in_debug_mode(instance, false);
        if (SPI_RD_MCR_FRZ(instance) != 0)
        {
            s_dspiTestStatus[instance].B.testStatusDspiFreeze += 1;
        }

        /**********************************************/
        /* Test the dspi_hal_configure_modified_timing_format */
        /**********************************************/
        dspi_hal_configure_modified_timing_format(instance, true);
        if (SPI_RD_MCR_MTFE(instance) != 1)
        {
            s_dspiTestStatus[instance].B.testStatusDspiModTimingFmt += 1;
        }

        dspi_hal_configure_modified_timing_format(instance, false);
        if (SPI_RD_MCR_MTFE(instance) != 0)
        {
            s_dspiTestStatus[instance].B.testStatusDspiModTimingFmt += 1;
        }

        /**********************************************/
        /* Test the dspi_hal_configure_pcs_strobe */
        /**********************************************/
        dspi_hal_configure_pcs_strobe(instance, true);
        if (SPI_RD_MCR_PCSSE(instance) != 1)
        {
            s_dspiTestStatus[instance].B.testStatusDspiPcsStrobe += 1;
        }

        dspi_hal_configure_pcs_strobe(instance, false);
        if (SPI_RD_MCR_PCSSE(instance) != 0)
        {
            s_dspiTestStatus[instance].B.testStatusDspiPcsStrobe += 1;
        }

        /**********************************************/
        /* Test the dspi_hal_configure_rx_fifo_overwrite */
        /**********************************************/
        dspi_hal_configure_rx_fifo_overwrite(instance, true);
        if (SPI_RD_MCR_ROOE(instance) != 1)
        {
            s_dspiTestStatus[instance].B.testStatusDspiRxFifoOvrwrite += 1;
        }

        dspi_hal_configure_rx_fifo_overwrite(instance, false);
        if (SPI_RD_MCR_ROOE(instance) != 0)
        {
            s_dspiTestStatus[instance].B.testStatusDspiRxFifoOvrwrite += 1;
        }

        /**********************************************/
        /* Test the dspi_hal_configure_pcs_polarity */
        /**********************************************/
        dspi_hal_configure_pcs_polarity(instance, kDspiPcs0, kDspiPcs_ActiveHigh);
        dspi_hal_configure_pcs_polarity(instance, kDspiPcs1, kDspiPcs_ActiveHigh);
        dspi_hal_configure_pcs_polarity(instance, kDspiPcs2, kDspiPcs_ActiveHigh);
        dspi_hal_configure_pcs_polarity(instance, kDspiPcs3, kDspiPcs_ActiveHigh);
        dspi_hal_configure_pcs_polarity(instance, kDspiPcs4, kDspiPcs_ActiveHigh);
        dspi_hal_configure_pcs_polarity(instance, kDspiPcs5, kDspiPcs_ActiveHigh);

        if (SPI_RD_MCR_PCSIS(instance) != 0)
        {
            s_dspiTestStatus[instance].B.testStatusDspiPcsPolarity += 1;
        }

        dspi_hal_configure_pcs_polarity(instance, kDspiPcs0, kDspiPcs_ActiveLow);
        dspi_hal_configure_pcs_polarity(instance, kDspiPcs1, kDspiPcs_ActiveLow);
        dspi_hal_configure_pcs_polarity(instance, kDspiPcs2, kDspiPcs_ActiveLow);
        dspi_hal_configure_pcs_polarity(instance, kDspiPcs3, kDspiPcs_ActiveLow);
        dspi_hal_configure_pcs_polarity(instance, kDspiPcs4, kDspiPcs_ActiveLow);
        dspi_hal_configure_pcs_polarity(instance, kDspiPcs5, kDspiPcs_ActiveLow);

        if (SPI_RD_MCR_PCSIS(instance) != 0x3F)
        {
            s_dspiTestStatus[instance].B.testStatusDspiPcsPolarity += 1;
        }

        /**********************************************/
        /* Test the dspi_hal_configure_doze_mode */
        /**********************************************/
        dspi_hal_configure_doze_mode(instance, true);
        if (SPI_RD_MCR_DOZE(instance) != 1)
        {
            s_dspiTestStatus[instance].B.testStatusDspiDozeMode += 1;
        }

        dspi_hal_configure_doze_mode(instance, false);
        if (SPI_RD_MCR_DOZE(instance) != 0)
        {
            s_dspiTestStatus[instance].B.testStatusDspiDozeMode += 1;
        }

        /**********************************************/
        /* Test the dspi_hal_set_datain_samplepoint */
        /**********************************************/
        dspi_hal_set_datain_samplepoint(instance, kDspiSckToSin_2Clock);
        if (SPI_RD_MCR_SMPL_PT(instance) != kDspiSckToSin_2Clock)
        {
            s_dspiTestStatus[instance].B.testStatusDspiSamplePoint += 1;
        }

        dspi_hal_set_datain_samplepoint(instance, kDspiSckToSin_1Clock);
        if (SPI_RD_MCR_SMPL_PT(instance) != kDspiSckToSin_1Clock)
        {
            s_dspiTestStatus[instance].B.testStatusDspiSamplePoint += 1;
        }

        dspi_hal_set_datain_samplepoint(instance, kDspiSckToSin_0Clock);
        if (SPI_RD_MCR_SMPL_PT(instance) != kDspiSckToSin_0Clock)
        {
            s_dspiTestStatus[instance].B.testStatusDspiSamplePoint += 1;
        }

        /**********************************************/
        /* Test the dpsi stop and start transfer functions*/
        /**********************************************/
        dspi_hal_stop_transfer(instance);
        if (SPI_RD_MCR_HALT(instance) != 1)
        {
            s_dspiTestStatus[instance].B.testStatusDspiStopStart += 1;
        }

        dspi_hal_start_transfer(instance);
        if (SPI_RD_MCR_HALT(instance) != 0)
        {
            s_dspiTestStatus[instance].B.testStatusDspiStopStart += 1;
        }

        /**********************************************/
        /* Test the dpsi preset and set transfer count functions*/
        /**********************************************/
        /* first enable the module*/
        dspi_hal_enable(instance);

        dspi_hal_preset_transfer_count(instance, 0xFFFF);
        if (dspi_hal_get_transfer_count(instance) != 0xFFFF)
        {
            s_dspiTestStatus[instance].B.testStatusDspiTransferCount += 1;
        }

        dspi_hal_preset_transfer_count(instance, 0xAA55);
        if (dspi_hal_get_transfer_count(instance) != 0xAA55)
        {
            s_dspiTestStatus[instance].B.testStatusDspiTransferCount += 1;
        }

        dspi_hal_preset_transfer_count(instance, 0x0000);
        if (dspi_hal_get_transfer_count(instance) != 0x0000)
        {
            s_dspiTestStatus[instance].B.testStatusDspiTransferCount += 1;
        }

        /* lastly disable the module*/
        dspi_hal_disable(instance);

        /**********************************************/
        /* Test the dspi set data format function*/
        /**********************************************/
        dspi_data_format_config_t dataFormat;

        dataFormat.bitsPerFrame = 32; /* only for slave mode, master mode gets changed later*/
        dataFormat.clkPolarity = kDspiClockPolarity_ActiveLow;
        dataFormat.clkPhase = kDspiClockPhase_SecondEdge;
        dataFormat.direction = kDspiLsbFirst; /* LSBFE not applicable in slave mode*/

        /* test in slave mode*/
        dspi_hal_set_master_slave(instance, kDspiSlave);

        /* first, ensure that this function returned successfully*/
        if (dspi_hal_configure_data_format(instance, kDspiCtar0, &dataFormat) != kStatus_Success)
        {
            s_dspiTestStatus[instance].B.testStatusDspiDataFormat += 1;
        }

        /* read individual bit values to ensure they've been programmed correctly*/
        if ((SPI_RD_CTARn_SLAVE_FMSZ(instance, kDspiCtar0) != (dataFormat.bitsPerFrame - 1)) ||
            (SPI_RD_CTARn_SLAVE_CPOL(instance, kDspiCtar0) != dataFormat.clkPolarity) ||
            (SPI_RD_CTARn_SLAVE_CPHA(instance, kDspiCtar0) != dataFormat.clkPhase))
        {
            s_dspiTestStatus[instance].B.testStatusDspiDataFormat += 1;
        }

        /* test in master mode*/
        dspi_hal_set_master_slave(instance, kDspiMaster);

        dataFormat.bitsPerFrame = 8;

        /* first, ensure that this function returned successfully*/
        if (dspi_hal_configure_data_format(instance, kDspiCtar1, &dataFormat) != kStatus_Success)
        {
            s_dspiTestStatus[instance].B.testStatusDspiDataFormat += 1;
        }

        /* read individual bit values to ensure they've been programmed correctly*/
        if ((SPI_RD_CTARn_FMSZ(instance, kDspiCtar1) != (dataFormat.bitsPerFrame - 1)) ||
            (SPI_RD_CTARn_CPOL(instance, kDspiCtar1) != dataFormat.clkPolarity) ||
            (SPI_RD_CTARn_CPHA(instance, kDspiCtar1) != dataFormat.clkPhase) ||
            (SPI_RD_CTARn_LSBFE(instance, kDspiCtar1) != dataFormat.direction))
        {
            s_dspiTestStatus[instance].B.testStatusDspiDataFormat += 1;
        }

        /**********************************************/
        /* Test the dspi configure delays function*/
        /**********************************************/
        dspi_delay_settings_config_t delayConfig;
        delayConfig.pcsToSckPre = 0x3;
        delayConfig.pcsToSck = 0xF;
        delayConfig.afterSckPre = 0x2;
        delayConfig.afterSck = 0xA;
        delayConfig.afterTransferPre = 0x1;
        delayConfig.afterTransfer = 0x5;

        dspi_hal_configure_delays(instance, kDspiCtar0, &delayConfig);

        /* read individual bit values to ensure they've been programmed correctly*/
        if ((SPI_RD_CTARn_PCSSCK(instance, kDspiCtar0) != delayConfig.pcsToSckPre) ||
            (SPI_RD_CTARn_CSSCK(instance, kDspiCtar0) != delayConfig.pcsToSck) ||
            (SPI_RD_CTARn_PASC(instance, kDspiCtar0) != delayConfig.afterSckPre) ||
            (SPI_RD_CTARn_ASC(instance, kDspiCtar0) != delayConfig.afterSck) ||
            (SPI_RD_CTARn_PDT(instance, kDspiCtar0) != delayConfig.afterTransferPre) ||
            (SPI_RD_CTARn_DT(instance, kDspiCtar0) != delayConfig.afterTransfer))

        {
            s_dspiTestStatus[instance].B.testStatusDspiConfigDelays = 1;
        }

        /**********************************************/
        /* Test the interrupt functions*/
        /**********************************************/
        /* set the interrupt enable*/
        dspi_hal_configure_interrupt(instance, kDspiTxComplete, true);
        if (dspi_hal_get_interrupt_config(instance, kDspiTxComplete) != 1)
        {
            s_dspiTestStatus[instance].B.testStatusDspiInterruptConfig += 1;
        }
        dspi_hal_configure_interrupt(instance, kDspiEndOfQueue, true);
        if (dspi_hal_get_interrupt_config(instance, kDspiEndOfQueue) != 1)
        {
            s_dspiTestStatus[instance].B.testStatusDspiInterruptConfig += 1;
        }
        dspi_hal_configure_interrupt(instance, kDspiTxFifoUnderflow, true);
        if (dspi_hal_get_interrupt_config(instance, kDspiTxFifoUnderflow) != 1)
        {
            s_dspiTestStatus[instance].B.testStatusDspiInterruptConfig += 1;
        }
        dspi_hal_configure_interrupt(instance, kDspiTxFifoFillRequest, true);
        if (dspi_hal_get_interrupt_config(instance, kDspiTxFifoFillRequest) != 1)
        {
            s_dspiTestStatus[instance].B.testStatusDspiInterruptConfig += 1;
        }
        dspi_hal_configure_interrupt(instance, kDspiRxFifoOverflow, true);
        if (dspi_hal_get_interrupt_config(instance, kDspiRxFifoOverflow) != 1)
        {
            s_dspiTestStatus[instance].B.testStatusDspiInterruptConfig += 1;
        }
        dspi_hal_configure_interrupt(instance, kDspiRxFifoDrainRequest, true);
        if (dspi_hal_get_interrupt_config(instance, kDspiRxFifoDrainRequest) != 1)
        {
            s_dspiTestStatus[instance].B.testStatusDspiInterruptConfig += 1;
        }

        /* clear the interrupt enable*/
        dspi_hal_configure_interrupt(instance, kDspiTxComplete, false);
        if (dspi_hal_get_interrupt_config(instance, kDspiTxComplete) != 0)
        {
            s_dspiTestStatus[instance].B.testStatusDspiInterruptConfig += 1;
        }
        dspi_hal_configure_interrupt(instance, kDspiEndOfQueue, false);
        if (dspi_hal_get_interrupt_config(instance, kDspiEndOfQueue) != 0)
        {
            s_dspiTestStatus[instance].B.testStatusDspiInterruptConfig += 1;
        }
        dspi_hal_configure_interrupt(instance, kDspiTxFifoUnderflow, false);
        if (dspi_hal_get_interrupt_config(instance, kDspiTxFifoUnderflow) != 0)
        {
            s_dspiTestStatus[instance].B.testStatusDspiInterruptConfig += 1;
        }
        dspi_hal_configure_interrupt(instance, kDspiTxFifoFillRequest, false);
        if (dspi_hal_get_interrupt_config(instance, kDspiTxFifoFillRequest) != 0)
        {
            s_dspiTestStatus[instance].B.testStatusDspiInterruptConfig += 1;
        }
        dspi_hal_configure_interrupt(instance, kDspiRxFifoOverflow, false);
        if (dspi_hal_get_interrupt_config(instance, kDspiRxFifoOverflow) != 0)
        {
            s_dspiTestStatus[instance].B.testStatusDspiInterruptConfig += 1;
        }
        dspi_hal_configure_interrupt(instance, kDspiRxFifoDrainRequest, false);
        if (dspi_hal_get_interrupt_config(instance, kDspiRxFifoDrainRequest) != 0)
        {
            s_dspiTestStatus[instance].B.testStatusDspiInterruptConfig += 1;
        }

        /**********************************************/
        /* Test the dspi_hal_configure_fifos */
        /**********************************************/
        /* need to first enable the dspi to write to the DSI_TXF and DIS_RXF*/
        dspi_hal_enable(instance);

        dspi_hal_configure_fifos(instance, true, true); /* disable fifos*/
        if ((SPI_RD_MCR_DIS_TXF(instance) != 1) || (SPI_RD_MCR_DIS_RXF(instance) != 1))
        {
            s_dspiTestStatus[instance].B.testStatusDspiFifoConfig = 1;
        }

        dspi_hal_configure_fifos(instance, false, false); /* enable fifos*/
        if ((SPI_RD_MCR_DIS_TXF(instance) != 0) || (SPI_RD_MCR_DIS_RXF(instance) != 0))
        {
            s_dspiTestStatus[instance].B.testStatusDspiFifoConfig = 1;
        }

        /**********************************************/
        /* Test the filling the TX Fifo and the TX Fifo Counter and Fifo entry data*/
        /* Note: this test proceeds the status flag test as status flags will set*/
        /*       after a transfer is started*/
        /**********************************************/
        /* first, set for master mode and halt (stop) transfers to allow us to fill the tx fifo*/
        dspi_hal_set_master_slave(instance, kDspiMaster);
        dspi_hal_stop_transfer(instance);

        /* fill up a src buffer with data (4 16-bit words)*/
        uint16_t src[4];
        for (i = 0; i < fifoSizeCount; i++)
        {
            src[i] = 0xFFF0 + i;
        }

        /* fill in members of the dspi_command_data_config_t struct*/
        dspi_command_config_t commandConfig;
        commandConfig.isChipSelectContinuous = true;
        commandConfig.whichCtar = kDspiCtar0;
        commandConfig.whichPcs = kDspiPcs1;
        commandConfig.clearTransferCount = false;
        commandConfig.isEndOfQueue = false;

        /* tx fifo counter should be empty (zero)*/
        if (dspi_hal_get_fifo_counter_or_pointer(instance, kDspiTxFifoCounter) != 0)
        {
            s_dspiTestStatus[instance].B.testStatusDspiTxFifoCounter = 1;
        }

        /* now fill the tx fifo*/
        for (i = 0; i < fifoSizeCount; i++)
        {
            if (i == 0) /* first data word*/
            {
                commandConfig.clearTransferCount = true; /* set to true for first word*/

                /* if fifoSizeCount is 1, then set the end of queue flag  */
                if (fifoSizeCount == 1)
                {
                    commandConfig.isEndOfQueue = true;
                }

                dspi_hal_write_data_master_mode(instance, &commandConfig, src[i]);
                commandConfig.clearTransferCount = false; /* set back to false after first word*/

                /* tx fifo counter should be increased by one*/
                if (dspi_hal_get_fifo_counter_or_pointer(instance, kDspiTxFifoCounter) != 1)
                {
                    s_dspiTestStatus[instance].B.testStatusDspiTxFifoCounter = 1;
                }
            }
            else if (i == 3) /* last data word*/
            {
                commandConfig.isEndOfQueue = true;
                dspi_hal_write_data_master_mode(instance, &commandConfig, src[i]);

                /* tx fifo counter should be full (four)*/
                if (dspi_hal_get_fifo_counter_or_pointer(instance, kDspiTxFifoCounter) != 4)
                {
                    s_dspiTestStatus[instance].B.testStatusDspiTxFifoCounter = 1;
                }
            }
            else
            {
                dspi_hal_write_data_master_mode(instance, &commandConfig, src[i]);

                /* tx fifo counter should be increased by one*/
                if (dspi_hal_get_fifo_counter_or_pointer(instance, kDspiTxFifoCounter) != (1 + i))
                {
                    s_dspiTestStatus[instance].B.testStatusDspiTxFifoCounter = 1;
                }
            }
        }

        /* read back the fifo contents of the transmit fifo registers (TXFRn)*/
        for (i = 0; i < fifoSizeCount; i++)
        {
            if (i == 0) /* first data word*/
            {
                /* if fifoSizeCount is 1, then set the end of queue flag  */
                if (fifoSizeCount == 1)
                {
                    /* 0x8C020000 => CONT, EOQ, CTCNT, and PCS[1] are set*/
                    if (dspi_hal_get_fifo_data(instance, kDspiTxFifo, i) != (0x8C020000 | src[i]))
                    {
                        s_dspiTestStatus[instance].B.testStatusDspiTxFifoTest = 1;
                    }
                }
                else
                {
                    /* 0x84020000 => CONT, CTCNT, and PCS[1] are set*/
                    if (dspi_hal_get_fifo_data(instance, kDspiTxFifo, i) != (0x84020000 | src[i]))
                    {
                        s_dspiTestStatus[instance].B.testStatusDspiTxFifoTest = 1;
                    }
                }
            }
            else if (i == 3) /* last data word*/
            {
                /* 0x88020000 => CONT, EOQ, and PCS[1] are set*/
                if (dspi_hal_get_fifo_data(instance, kDspiTxFifo, i) != (0x88020000 | src[i]))
                {
                    s_dspiTestStatus[instance].B.testStatusDspiTxFifoTest = 1;
                }
            }
            else
            {
                /* 0x80020000 => CONT and PCS[1] are set*/
                if (dspi_hal_get_fifo_data(instance, kDspiTxFifo, i) != (0x80020000 | src[i]))
                {
                    s_dspiTestStatus[instance].B.testStatusDspiTxFifoTest = 1;
                }
            }
        }

        /**********************************************/
        /* Test the status flag function (and RX fifo counter and pop next pointer test)*/
        /**********************************************/
        /* set the baud rate to the slowest value from the baud rate test above*/
        dspi_hal_set_baud(instance, kDspiCtar0, baudRates[0], sourceClkHz);
        /* the following status flags should be set once a transfer has started*/
        dspi_hal_start_transfer(instance);

        /* immediately check the run status flag*/
        if (dspi_hal_get_status_flag(instance, kDspiTxAndRxStatus) != 1)
        {
            s_dspiTestStatus[instance].B.testStatusDspiStatusFlagSet += 1;
        }

        /* now wait for transfer to complete by waiting till transfer count gets to */
        /* "fifoSizeCount" words*/
        while (dspi_hal_get_transfer_count(instance) != fifoSizeCount)
        {
        }

        /* first start with the RX fifo counter and POPNXTPRT flags, even though nothing was */
        /* connected to the rx line the receive fifo will still shift in data even if */
        /* nothing is connected.  It should be equal to fifoSizeCount.*/
        if (dspi_hal_get_fifo_counter_or_pointer(instance, kDspiRxFifoCounter) != fifoSizeCount)
        {
            s_dspiTestStatus[instance].B.testStatusDspiRxFifoCounter += 1;
        }
        /* now read from the rx fifo */
        for (i = 0; i < fifoSizeCount; i++)
        {
            /* see if rx pop next pointer incremented*/
            if (dspi_hal_get_fifo_counter_or_pointer(instance, kDspiRxFifoPointer) != i)
            {
                s_dspiTestStatus[instance].B.testStatusDspiRxFifoCounter += 1;
            }

            /* read from fifo*/
            dspi_hal_read_data(instance);
            /* see if rx fifo counter decremented*/
            if (dspi_hal_get_fifo_counter_or_pointer(instance, kDspiRxFifoCounter) != ((fifoSizeCount - 1) - i))
            {
                s_dspiTestStatus[instance].B.testStatusDspiRxFifoCounter += 1;
            }
        }

        /* Check the various status flags to ensure they were properly set*/
        if (dspi_hal_get_status_flag(instance, kDspiTxComplete) != 1)
        {
            s_dspiTestStatus[instance].B.testStatusDspiStatusFlagSet += 1;
        }
        if (dspi_hal_get_status_flag(instance, kDspiEndOfQueue) != 1)
        {
            s_dspiTestStatus[instance].B.testStatusDspiStatusFlagSet += 1;
        }
        if (dspi_hal_get_status_flag(instance, kDspiTxFifoFillRequest) != 1)
        {
            s_dspiTestStatus[instance].B.testStatusDspiStatusFlagSet += 1;
        }
        if (dspi_hal_get_status_flag(instance, kDspiRxFifoDrainRequest) != 1)
        {
            s_dspiTestStatus[instance].B.testStatusDspiStatusFlagSet += 1;
        }

        /* clear the status flags*/
        dspi_hal_clear_status_flag(instance, kDspiTxComplete);
        if (dspi_hal_get_status_flag(instance, kDspiTxComplete) != 0)
        {
            s_dspiTestStatus[instance].B.testStatusDspiStatusFlagClr += 1;
        }
        dspi_hal_clear_status_flag(instance, kDspiEndOfQueue);
        if (dspi_hal_get_status_flag(instance, kDspiEndOfQueue) != 0)
        {
            s_dspiTestStatus[instance].B.testStatusDspiStatusFlagClr += 1;
        }
        dspi_hal_clear_status_flag(instance, kDspiRxFifoDrainRequest);
        if (dspi_hal_get_status_flag(instance, kDspiRxFifoDrainRequest) != 0)
        {
            s_dspiTestStatus[instance].B.testStatusDspiStatusFlagClr += 1;
        }

        /* TFFF (tx fifo fil1 request, is a special case. Normally, this status flag operates in*/
        /* conjunction with the DMA.  However, without DMA, this bit will only clear if:*/
        /* 1. Verify TFFF is set (TX fifo not full)*/
        /* 2. The TX fifo is full*/
        /* 3. We manually clear it (w1c), if tx fifo is full, this bit should stay cleared*/

        /* Step 0. First, halt spi transfers, though the fifo will fill quicker than the */
        /*         the transmit shift register can shift out data, but for now halt transfer*/
        /*         We call this step 0 since it is not an actual needed step.*/
        dspi_hal_stop_transfer(instance);

        /* Step 1. Verify TFFF is set*/
        if (dspi_hal_get_status_flag(instance, kDspiTxFifoFillRequest) != 1)
        {
            s_dspiTestStatus[instance].B.testStatusDspiStatusFlagSet += 1;
        }

        /* Step 2. Fill TX fifo. This routine will keep filling the TX fifo and then manually*/
        /*         clear TFFF and if it stays clear, exits, else it keeps filling the fifo*/
        /*         We'll set a limit of 20 loops such that if we hit the limit, we'll know something*/
        /*         didn't work since the spi fifos are not configured for such large fifo depths*/
        for (i = 0; i < 20; i++)
        {
            /* note, commandConfig defined above, don't care what the conents are*/
            dspi_hal_write_data_master_mode(instance, &commandConfig, i);

            dspi_hal_clear_status_flag(instance, kDspiTxFifoFillRequest);
            if (dspi_hal_get_status_flag(instance, kDspiTxFifoFillRequest) == 0)
            {
                break;
            }
        }

        /* see if we hit the limit*/
        if (i == 20)
        {
            s_dspiTestStatus[instance].B.testStatusDspiStatusFlagTfffClr += 1;
        }
        /* double check to make sure TFFF is cleared*/
        if (dspi_hal_get_status_flag(instance, kDspiTxFifoFillRequest) != 0)
        {
            s_dspiTestStatus[instance].B.testStatusDspiStatusFlagTfffClr += 1;
        }

        /* now, for one final test of the TFFF bit, let's see if it sets when the transfer */
        /* is started*/
        dspi_hal_start_transfer(instance);
        /* wait for tx to complete*/
        while (dspi_hal_get_status_flag(instance, kDspiTxComplete) != 1)
        {
        }

        if (dspi_hal_get_status_flag(instance, kDspiTxFifoFillRequest) != 1)
        {
            s_dspiTestStatus[instance].B.testStatusDspiStatusFlagTfffSet += 1;
        }

        /**********************************************/
        /* Test the TX and RX Fifo Flush*/
        /**********************************************/
        /* flush the tx fifo*/
        dspi_hal_flush_fifos(instance, true, true);

        /* read back the fifo contents of the transmit fifo registers (TXFRn)*/
        /* should be cleared*/
        for (i = 0; i < fifoSizeCount; i++)
        {
            if ((dspi_hal_get_fifo_data(instance, kDspiTxFifo, i) != 0x0) ||
                (dspi_hal_get_fifo_data(instance, kDspiRxFifo, i) != 0x0))
            {
                s_dspiTestStatus[instance].B.testStatusDspiFifoFlush = 1;
            }
        }

        /**********************************************/
        /* Test the DMA Configure functions*/
        /**********************************************/
        dspi_hal_configure_dma(instance, true, false);
        if ((SPI_RD_RSER_TFFF_DIRS(instance) != 1) || (SPI_RD_RSER_RFDF_DIRS(instance) != 0))
        {
            s_dspiTestStatus[instance].B.testStatusDspiDmaConfig += 1;
        }

        dspi_hal_configure_dma(instance, false, true);
        if ((SPI_RD_RSER_TFFF_DIRS(instance) != 0) || (SPI_RD_RSER_RFDF_DIRS(instance) != 1))
        {
            s_dspiTestStatus[instance].B.testStatusDspiDmaConfig += 1;
        }

        /**********************************************/
        /* Test the dspi reset function*/
        /**********************************************/
        dspi_hal_reset(instance);

        if ((SPI_RD_MCR(instance) != (SPI_MCR_MDIS_MASK | SPI_MCR_HALT_MASK)) || (SPI_RD_TCR(instance) != 0) ||
            (SPI_RD_CTARn(instance, kDspiCtar0) != 0) || (SPI_RD_CTARn(instance, kDspiCtar1) != 0) ||
            (SPI_RD_RSER(instance) != 0))
        {
            s_dspiTestStatus[instance].B.testStatusDspiReset = 1;
        }

        /**********************************************/
        /* Test the dspi_hal_master_init function and associated config structures*/
        /**********************************************/
        uint32_t calculatedBaudRate;

        /* fill out the members of the dspi_master_config_t structure*/
        dspi_master_config_t dspiMasterConfig;

        dspiMasterConfig.isEnabled = false;
        dspiMasterConfig.whichCtar = kDspiCtar0;
        dspiMasterConfig.bitsPerSec = 0;
        dspiMasterConfig.sourceClockInHz = dspiBusClock;

        /* data format field*/
        dspiMasterConfig.dataConfig.bitsPerFrame = 16;
        dspiMasterConfig.dataConfig.clkPolarity = kDspiClockPolarity_ActiveHigh;
        dspiMasterConfig.dataConfig.clkPhase = kDspiClockPhase_FirstEdge;
        dspiMasterConfig.dataConfig.direction = kDspiMsbFirst;

        dspiMasterConfig.isSckContinuous = false;
        dspiMasterConfig.whichPcs = kDspiPcs1;
        dspiMasterConfig.pcsPolarity = kDspiPcs_ActiveHigh;
        dspiMasterConfig.masterInSample = kDspiSckToSin_0Clock;
        dspiMasterConfig.isModifiedTimingFormatEnabled = false;
        dspiMasterConfig.isTxFifoDisabled = false; /* enable tx fifo*/
        dspiMasterConfig.isRxFifoDisabled = false; /* enable rx fifo*/

        /* make sure the init function doesn't return error */
        if (dspi_hal_master_init(instance, &dspiMasterConfig, &calculatedBaudRate) != kStatus_Success)
        {
            s_dspiTestStatus[instance].B.testStatusDspiMasterInit = +1;
        }

        /*  calculatedBaudRate should be 0 since we are setting it to 0*/
        if (calculatedBaudRate != 0)
        {
            s_dspiTestStatus[instance].B.testStatusDspiMasterInit = +1;
        }

        /* read the various bitfields to ensure they've been programmed correctly*/
        if (SPI_RD_MCR(instance) != (SPI_MCR_MSTR_MASK | SPI_MCR_MDIS_MASK | SPI_MCR_HALT_MASK))
        {
            s_dspiTestStatus[instance].B.testStatusDspiMasterInit = +1;
        }

        if (SPI_RD_CTARn(instance, kDspiCtar0) != 0x78000000)
        {
            s_dspiTestStatus[instance].B.testStatusDspiMasterInit = +1;
        }

        /********************************************/
        /* Test the dspi_hal_slave_init function and associated config structures */
        /********************************************/
        /* fill out the members of the dspi_slave_config_t structure */
        dspi_slave_config_t dspiSlaveConfig;

        dspiSlaveConfig.isEnabled = false;
        /* data format field */
        dspiSlaveConfig.dataConfig.bitsPerFrame = 32;
        dspiSlaveConfig.dataConfig.clkPolarity = kDspiClockPolarity_ActiveHigh;
        dspiSlaveConfig.dataConfig.clkPhase = kDspiClockPhase_FirstEdge;

        dspiSlaveConfig.isTxFifoDisabled = true; /* disable tx fifo */
        dspiSlaveConfig.isRxFifoDisabled = true; /* disable rx fifo */

        /* make sure the init function doesn't return error */
        if (dspi_hal_slave_init(instance, &dspiSlaveConfig) != kStatus_Success)
        {
            s_dspiTestStatus[instance].B.testStatusDspiSlaveInit = +1;
        }

        /* read the various bitfields to ensure they've been programmed correctly */
        if (SPI_RD_MCR(instance) !=
            (SPI_MCR_MDIS_MASK | SPI_MCR_DIS_TXF_MASK | SPI_MCR_DIS_RXF_MASK | SPI_MCR_HALT_MASK))
        {
            s_dspiTestStatus[instance].B.testStatusDspiSlaveInit = +1;
        }

        if (SPI_RD_CTARn(instance, kDspiCtar0) != 0xF8000000)
        {
            s_dspiTestStatus[instance].B.testStatusDspiSlaveInit = +1;
        }

    } /* end of for-loop hal tests*/

    /**************************************************************************/
    /* Error reporting*/
    /**************************************************************************/
    for (instance = 0; instance < SPI_INSTANCE_COUNT; instance++)
    {
        if (s_dspiTestStatus[instance].all)
        {
            printf("\r\n**failures seen on instance %d\n", instance);

            if (s_dspiTestStatus[instance].B.testStatusDspiEnableDisable == 1)
            {
                printf("\r  DspiEnableDisable failed \n ");
            }
            if (s_dspiTestStatus[instance].B.testStatusDspiBaudRateDiv == 1)
            {
                printf("\r  DspiBaudRateDiv failed \n ");
            }
            if (s_dspiTestStatus[instance].B.testStatusDspiMasterSlave == 1)
            {
                printf("\r  DspiMasterSlave failed \n ");
            }
            if (s_dspiTestStatus[instance].B.testStatusDspiContSck == 1)
            {
                printf("\r  DspiContSck failed \n ");
            }
            if (s_dspiTestStatus[instance].B.testStatusDspiFreeze == 1)
            {
                printf("\r  DspiFreeze failed \n ");
            }
            if (s_dspiTestStatus[instance].B.testStatusDspiModTimingFmt == 1)
            {
                printf("\r  DspiModTimingFmt failed \n ");
            }
            if (s_dspiTestStatus[instance].B.testStatusDspiPcsStrobe == 1)
            {
                printf("\r  DspiPcsStrobe failed \n ");
            }
            if (s_dspiTestStatus[instance].B.testStatusDspiRxFifoOvrwrite == 1)
            {
                printf("\r  DspiRxFifoOvrwrite failed \n ");
            }
            if (s_dspiTestStatus[instance].B.testStatusDspiPcsPolarity == 1)
            {
                printf("\r  DspiPcsPolarity failed \n ");
            }
            if (s_dspiTestStatus[instance].B.testStatusDspiDozeMode == 1)
            {
                printf("\r  DspiDozeMode failed \n ");
            }
            if (s_dspiTestStatus[instance].B.testStatusDspiSamplePoint == 1)
            {
                printf("\r  DspiSamplePoint failed \n ");
            }
            if (s_dspiTestStatus[instance].B.testStatusDspiStopStart == 1)
            {
                printf("\r  DspiStopStart failed \n ");
            }
            if (s_dspiTestStatus[instance].B.testStatusDspiTransferCount == 1)
            {
                printf("\r  DspiTransferCount failed \n ");
            }
            if (s_dspiTestStatus[instance].B.testStatusDspiDataFormat == 1)
            {
                printf("\r  DspiDataFormat failed \n ");
            }
            if (s_dspiTestStatus[instance].B.testStatusDspiConfigDelays == 1)
            {
                printf("\r  DspiConfigDelays failed \n ");
            }
            if (s_dspiTestStatus[instance].B.testStatusDspiInterruptConfig == 1)
            {
                printf("\r  DspiInterruptConfig failed \n ");
            }
            if (s_dspiTestStatus[instance].B.testStatusDspiFifoConfig == 1)
            {
                printf("\r  DspiFifoConfig failed \n ");
            }
            if (s_dspiTestStatus[instance].B.testStatusDspiTxFifoCounter == 1)
            {
                printf("\r  DspiTxFifoCounter failed \n ");
            }
            if (s_dspiTestStatus[instance].B.testStatusDspiRxFifoCounter == 1)
            {
                printf("\r  DspiRxFifoCounter failed \n ");
            }
            if (s_dspiTestStatus[instance].B.testStatusDspiTxFifoTest == 1)
            {
                printf("\r  DspiTxFifoTest failed \n ");
            }
            if (s_dspiTestStatus[instance].B.testStatusDspiStatusFlagSet == 1)
            {
                printf("\r  DspiStatusFlagSet failed \n ");
            }
            if (s_dspiTestStatus[instance].B.testStatusDspiStatusFlagClr == 1)
            {
                printf("\r  DspiStatusFlagClr failed \n ");
            }
            if (s_dspiTestStatus[instance].B.testStatusDspiFifoFlush == 1)
            {
                printf("\r  DspiFifoFlush failed \n ");
            }
            if (s_dspiTestStatus[instance].B.testStatusDspiDmaConfig == 1)
            {
                printf("\r  DspiDmaConfig failed \n ");
            }
            if (s_dspiTestStatus[instance].B.testStatusDspiBaudRateTest == 1)
            {
                printf("\r  DspiBaudRateTest failed \n ");
            }
            if (s_dspiTestStatus[instance].B.testStatusDspiMasterInit == 1)
            {
                printf("\r  DspiMasterInit failed \n ");
            }
            if (s_dspiTestStatus[instance].B.testStatusDspiSlaveInit == 1)
            {
                printf("\r  DspiSlaveInit failed \n ");
            }
            if (s_dspiTestStatus[instance].B.testStatusDspiReset == 1)
            {
                printf("\r  DspiReset failed \n ");
            }
            errorFlag = 1;
        }
        else
        {
            printf("\r\nAll Tests PASSED, instance %d\n", instance);
        }
    }

    /* trap here if any failures were reported (useful for cases where the uart is not used) */
    if (errorFlag == 1)
    {
        while (1)
        {
        }
    }

    return 0;
}

/*******************************************************************************
 * EOF
 ******************************************************************************/

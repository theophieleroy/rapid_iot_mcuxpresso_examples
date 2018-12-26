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
#include "dspi/fsl_dspi_master_driver.h"
#include "fsl_dspi_driver_unit_test.h"
#include "clock/fsl_clock_manager.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/

/*!
 * @brief Test function for dspi peripheral driver test.
 *        This subtest will test some of the master driver functions to ensure
 *        the module gets configured as expected.
 *        This does not test all of the functions like transfer, which are covered
 *        in other tests.  Mainly test the init and config type of functions.
 *
 *
 */
uint32_t dspi_test_master_functions(void)
{
    uint32_t failCount = 0;

    dspi_master_user_config_t userConfig;
    dspi_device_t spiDevice;
    dspi_master_state_t dspiMasterState;
    uint32_t calculatedBaudRate;
    uint32_t instance = 0;
    uint32_t baudRate = 78125;
    uint32_t bitsPerFrame16 = 16;
    uint32_t bitsPerFrame8 = 8;

    for (instance = 0; instance < SPI_INSTANCE_COUNT; instance++)
    {
        /* configure the members of the user config*/
        userConfig.isChipSelectContinuous = true;
        userConfig.isSckContinuous = true;
        userConfig.pcsPolarity = kDspiPcs_ActiveLow;
        userConfig.whichCtar = kDspiCtar0;
        userConfig.whichPcs = kDspiPcs0;

        /* config member of the spi device config, change it from what was programmed above*/
        /* and enusre the changes took*/
        spiDevice.bitsPerSec = baudRate;
        spiDevice.dataBusConfig.bitsPerFrame = bitsPerFrame8;
        spiDevice.dataBusConfig.clkPhase = kDspiClockPhase_SecondEdge;
        spiDevice.dataBusConfig.clkPolarity = kDspiClockPolarity_ActiveLow;
        spiDevice.dataBusConfig.direction = kDspiLsbFirst;

        /* init the dspi module*/
        dspi_master_init(instance, &dspiMasterState, &userConfig, &calculatedBaudRate);

        if (calculatedBaudRate != 0)
        {
            failCount++;
        }
        /* test the MCR register settings against what was programmed*/
        if ((SPI_RD_MCR_MSTR(instance) != kDspiMaster) || (SPI_RD_MCR_CONT_SCKE(instance) != true) ||
            (SPI_RD_MCR_MTFE(instance) != false) || (SPI_RD_MCR_PCSIS(instance) != kDspiPcs0) ||
            (SPI_RD_MCR_MDIS(instance) != false) || (SPI_RD_MCR_SMPL_PT(instance) != kDspiSckToSin_0Clock))
        {
            failCount++;
        }
        /* test the ctar register contents*/
        if ((SPI_RD_CTARn_FMSZ(instance, kDspiCtar0) != (bitsPerFrame16 - 1)) ||
            (SPI_RD_CTARn_CPOL(instance, kDspiCtar0) != kDspiClockPolarity_ActiveHigh) ||
            (SPI_RD_CTARn_CPHA(instance, kDspiCtar0) != kDspiClockPhase_FirstEdge) ||
            (SPI_RD_CTARn_LSBFE(instance, kDspiCtar0) != kDspiMsbFirst))
        {
            failCount++;
        }

        /* test the configure bus function*/
        dspi_master_configure_bus(&dspiMasterState, &spiDevice, &calculatedBaudRate);
        if (calculatedBaudRate == 0)
        {
            failCount++;
        }
        /* test the changes in the ctar register contents*/
        if ((SPI_RD_CTARn_FMSZ(instance, kDspiCtar0) != (bitsPerFrame8 - 1)) ||
            (SPI_RD_CTARn_CPOL(instance, kDspiCtar0) != kDspiClockPolarity_ActiveLow) ||
            (SPI_RD_CTARn_CPHA(instance, kDspiCtar0) != kDspiClockPhase_SecondEdge) ||
            (SPI_RD_CTARn_LSBFE(instance, kDspiCtar0) != kDspiLsbFirst))
        {
            failCount++;
        }

        /* test the dspi_master_get_transfer_status*/
        uint32_t wordsTransferred = 0;
        if (dspi_master_get_transfer_status(&dspiMasterState, &wordsTransferred) != kStatus_Success)
        {
            failCount++;
        }
        if (wordsTransferred != 0) /* there should be no words transferred, hence 0*/
        {
            failCount++;
        }

        /* test the dspi_master_configure_modified_transfer_format */
        dspi_master_configure_modified_transfer_format(&dspiMasterState, true, kDspiSckToSin_2Clock);
        /* test the MCR register settings against what was programmed*/
        if ((SPI_RD_MCR_MTFE(instance) != true) || (SPI_RD_MCR_SMPL_PT(instance) != kDspiSckToSin_2Clock))
        {
            failCount++;
        }

        /* Do this test last */
        /* test the shutdown function*/
        dspi_master_shutdown(&dspiMasterState);
        /* Ungate the clock for SPI, it gets gated in shutdown.*/
        /* If clock to dspi module is gated, we'll get a fault trying to read back it's registers*/
        clock_manager_set_gate(kClockModuleSPI, instance, true);

        if ((SPI_RD_MCR(instance) != (SPI_MCR_MDIS_MASK | SPI_MCR_HALT_MASK)) || (SPI_RD_TCR(instance) != 0) ||
            (SPI_RD_CTARn(instance, kDspiCtar0) != 0) || (SPI_RD_CTARn(instance, kDspiCtar1) != 0) ||
            (SPI_RD_RSER(instance) != 0))
        {
            failCount++;
        }
    }

    return failCount;
}

/*******************************************************************************
 * EOF
 ******************************************************************************/
